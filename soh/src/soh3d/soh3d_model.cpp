// SoH3D model bridge: connects the runtime C++ asset loader (asset/) to the
// libultraship direct-GL renderer (SoH3D_GL_*). Owns the model registry (actor id
// -> 3DS asset + world scale), lazily parses+decodes a model from the decrypted
// .3ds the first time it's drawn (on the render thread, GL current), and serves the
// renderer's provider callback with the CPU data to upload. No baked-in C arrays;
// the .3ds path comes from env SOH3D_3DS_ROM (never hardcoded — repo rule).
#include "asset/ctr_rom.h"
#include "asset/zar.h"
#include "asset/zsi.h"
#include "asset/cmb.h"
#include "asset/csab.h"
#include "asset/mat4.h"
#include "asset/pica_texture.h"
#include "fast/soh3d_gl.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Matches the typedef in soh3d.h (which this pure-C++ TU does not include). The N64
// floor-height callback used by the terrain warp; soh3d.c supplies the implementation.
typedef float (*SoH3D_FloorFn)(float x, float z);

namespace {

struct ModelSpec {
    const char* zarPath;
    float worldScale;
    const char* cmbName; // substring to select the .cmb inside the ZAR (nullptr = first one).
                         // Needed when a ZAR holds several CMBs (e.g. a main model + a debris
                         // "hahen" variant) and firstWithSuffix would grab the wrong one.
};

// Registry keyed by modelId (the index). The actor->modelId mapping lives in
// soh3d.c (which has the ACTOR_* ids); this stays pure-C++ / engine-agnostic.
//   0 = geldwoman (white Gerudo, En_Ge1)
//   1 = large wooden crate (Obj_Kibako2) — pick the intact box, not the debris CMB
//   2 = bush (En_Kusa) — the intact bush, not the smaller obj_kusa03 variant
//   3 = pot (Obj_Tsubo) — the intact pot, not the tubo2_hahen debris CMB
const ModelSpec kModels[] = {
    { "/actor/zelda_ge1.zar", 0.011f, nullptr },
    { "/actor/zelda_kibako2.zar", 0.10f, "CIkibako_model" },
    { "/actor/zelda_kusa.zar", 0.5f, "obj_kusa01_model" },
    { "/actor/zelda_tsubo.zar", 0.12f, "tubo2_model" },
};

// Scene-room models live in a SEPARATE id range so they never collide with the actor
// table above. A room's geometry is a single embedded CMB inside a ZSI (no skeleton,
// no animation), drawn at the world origin. Ids are allocated on demand by the game
// (SoH3D_RoomModelId) keyed by the room's ZSI path. See soh3d.c's room-draw hook.
const int kSceneModelBase = 1000;

// Auto-replaced actor models live in a THIRD id range (above scene rooms) so the
// SOH3D_AUTO path can allocate ids for arbitrary actor ZARs (discovered at runtime
// from the object id -> ZAR table) without colliding with the hand-listed actor
// models (0..N) or scene rooms (1000..). Keyed by ZAR path; main CMB picked by the
// "largest non-debris" heuristic. See SoH3D_AutoModelId / loadAutoModel.
const int kAutoModelBase = 2000;

// Loaded CPU data for a model, kept alive so the renderer can upload from it and
// so the provider can hand back stable pointers. The Zar + Cmb stay resident so the
// animation layer can load CSABs and recompute skin matrices per frame on demand.
struct LoadedModel {
    std::vector<SoH3D::CmbDrawGroup> groups;       // interleaved verts (CmbVertex == SoH3DGlVtx layout)
    std::vector<std::vector<uint8_t>> texRgba;     // decoded RGBA8 per CMB texture
    std::vector<SoH3DGlGroup> cGroups;             // C-API view
    std::vector<SoH3DGlTex> cTexs;                 // C-API view
    std::unique_ptr<SoH3D::Zar> zar;               // resident archive (for CSAB lookup)
    std::unique_ptr<SoH3D::Cmb> cmb;               // resident model (skeleton + bind matrices)
    std::unordered_map<std::string, std::unique_ptr<SoH3D::Csab>> anims; // cached by full name
    bool ok = false;
    bool skinned = false; // auto models: CMB has an articulated skeleton (>1 bone) -> the
                          // auto path skips it (no anim => T-pose), leaving it to N64.
    // Per-XZ ground-delta field D(x,z) = N64_floor - OoT3D_floor for a scene room, computed
    // once (deltaReady). The render mesh is LEFT UNTOUCHED (pixel-faithful OoT3D); instead
    // actors are offset by -D so they stand on the visible OoT3D ground (inverse of the old
    // render warp, which smeared at N64 collision steps). minx/minz/nx/nz/step describe the grid.
    bool deltaReady = false;
    std::vector<float> delta;
    float dMinX = 0, dMinZ = 0, dStep = 100.0f;
    int dNx = 0, dNz = 0;
};

std::unordered_map<int, std::unique_ptr<LoadedModel>> g_loaded;
std::unique_ptr<SoH3D::CtrRom> g_rom;

// Scene-room id allocation: ZSI path -> model id (>= kSceneModelBase), and the reverse
// list so loadModel can recover the path from the id.
std::unordered_map<std::string, int> g_sceneRoomIds;
std::vector<std::string> g_sceneRoomPaths; // index = modelId - kSceneModelBase

// Auto-replaced actor id allocation: ZAR path -> model id (>= kAutoModelBase), and the
// reverse list so loadAutoModel can recover the path from the id.
std::unordered_map<std::string, int> g_autoModelIds;
std::vector<std::string> g_autoModelPaths; // index = modelId - kAutoModelBase

SoH3D::CtrRom* rom() {
    if (!g_rom) {
        const char* path = getenv("SOH3D_3DS_ROM");
        if (!path || !*path) {
            fprintf(stderr, "[SoH3D] SOH3D_3DS_ROM not set — cannot load OoT3D assets\n");
            return nullptr;
        }
        g_rom = std::make_unique<SoH3D::CtrRom>(path);
        if (!g_rom->ok()) {
            fprintf(stderr, "[SoH3D] CtrRom(%s): %s\n", path, g_rom->error().c_str());
            g_rom.reset();
            return nullptr;
        }
    }
    return g_rom.get();
}

// Decode an already-parsed CMB (out->cmb) into the renderer's CPU views: bind-pose
// draw groups (model-space verts + bone bindings; GPU skinning applies the pose, or
// identity = bind pose for skeleton-less scene rooms), decoded RGBA8 textures, and the
// C-API group/texture views. Shared by the actor (ZAR) and scene-room (ZSI) paths.
// bakedVertexColor: keep the CMB's per-vertex color (OoT3D baked scene lighting). Only
// SCENE ROOMS use it; characters/props are lit dynamically (scene ambient tint), and their
// CMB color attribute is unused/garbage (e.g. geldwoman reads ~0 -> would render black),
// so for those we force white (the verified-correct behavior).
static void buildFromCmb(LoadedModel* out, bool bakedVertexColor) {
    SoH3D::Cmb& cmb = *out->cmb;
    out->groups = cmb.buildDrawGroups();
    if (!bakedVertexColor) {
        for (auto& g : out->groups)
            for (auto& v : g.verts) { v.color[0] = v.color[1] = v.color[2] = v.color[3] = 1.0f; }
    }

    const auto& texs = cmb.textures();
    out->texRgba.resize(texs.size());
    out->cTexs.resize(texs.size());
    for (size_t i = 0; i < texs.size(); i++) {
        auto raw = cmb.textureRaw(texs[i]);
        out->texRgba[i] = SoH3D::PicaDecode(texs[i].glFormat(), texs[i].width, texs[i].height, raw);
        out->cTexs[i] = { out->texRgba[i].data(), texs[i].width, texs[i].height };
    }

    out->cGroups.reserve(out->groups.size());
    for (const auto& g : out->groups) {
        const SoH3D::CmbMaterial* mat =
            (g.material_index >= 0 && g.material_index < (int)cmb.materials().size()) ? &cmb.materials()[g.material_index]
                                                                                      : nullptr;
        SoH3DGlGroup cg{};
        cg.verts = reinterpret_cast<const SoH3DGlVtx*>(g.verts.data());
        cg.vertCount = (int)g.verts.size();
        cg.texIndex = cmb.materialTexture(g.material_index);
        cg.alphaTest = mat && mat->alpha_test ? 1 : 0;
        cg.alphaRef = mat ? mat->alpha_ref : 0.0f;
        cg.wrapS = mat ? mat->wrap_s : 0x2901;
        cg.wrapT = mat ? mat->wrap_t : 0x2901;
        cg.blendEnable = mat && mat->blend_enable ? 1 : 0;
        cg.blendSrcRGB = mat ? mat->blend_src_rgb : 0x0302;
        cg.blendDstRGB = mat ? mat->blend_dst_rgb : 0x0303;
        cg.blendEqRGB = mat ? mat->blend_eq_rgb : 0x8006;
        cg.blendSrcA = mat ? mat->blend_src_a : 0x0001;
        cg.blendDstA = mat ? mat->blend_dst_a : 0x0000;
        cg.blendEqA = mat ? mat->blend_eq_a : 0x8006;
        cg.depthWrite = mat ? (mat->depth_write ? 1 : 0) : 1;
        cg.polygonOffset = mat ? mat->polygon_offset : 0.0f;
        for (int k = 0; k < 4; k++) cg.blendColor[k] = mat ? mat->blend_color[k] : (k == 3 ? 1.0f : 0.0f);
        out->cGroups.push_back(cg);
    }
    out->ok = true;
}

// Load a scene-room model: read its ZSI, extract the single embedded room CMB, and
// build draw groups (no skeleton/animation — drawn at the world origin).
static void loadSceneRoom(int modelId, LoadedModel* out) {
    int idx = modelId - kSceneModelBase;
    if (idx < 0 || idx >= (int)g_sceneRoomPaths.size()) return;
    const std::string& path = g_sceneRoomPaths[idx];
    SoH3D::CtrRom* r = rom();
    if (!r) return;
    auto bytes = r->read(path);
    if (bytes.empty()) { fprintf(stderr, "[SoH3D] zsi not found: %s\n", path.c_str()); return; }
    SoH3D::Zsi zsi(std::move(bytes));
    if (!zsi.ok()) { fprintf(stderr, "[SoH3D] Zsi %s: %s\n", path.c_str(), zsi.error().c_str()); return; }
    if (!zsi.hasGeometry()) { fprintf(stderr, "[SoH3D] no room geometry in %s\n", path.c_str()); return; }
    out->cmb = std::make_unique<SoH3D::Cmb>(zsi.cmbBytes());
    if (!out->cmb->ok()) { fprintf(stderr, "[SoH3D] Cmb %s: %s\n", path.c_str(), out->cmb->error().c_str()); return; }
    buildFromCmb(out, /*bakedVertexColor=*/true); // scene rooms carry OoT3D baked vertex lighting
    printf("[SoH3D] loaded scene-room model %d (%s): %zu groups, %zu textures\n", modelId, path.c_str(),
           out->cGroups.size(), out->cTexs.size());
}

// Load an actor model: read its ZAR, find the .cmb, build groups (+ keep the ZAR/CMB
// resident so the animation layer can load CSABs and recompute skin matrices).
static void loadActorModel(int modelId, LoadedModel* out) {
    SoH3D::CtrRom* r = rom();
    if (!r) return;
    auto zarBytes = r->read(kModels[modelId].zarPath);
    if (zarBytes.empty()) { fprintf(stderr, "[SoH3D] zar not found: %s\n", kModels[modelId].zarPath); return; }
    out->zar = std::make_unique<SoH3D::Zar>(std::move(zarBytes));
    if (!out->zar->ok()) { fprintf(stderr, "[SoH3D] Zar: %s\n", out->zar->error().c_str()); return; }
    const SoH3D::ZarFile* cmbf = nullptr;
    const char* want = kModels[modelId].cmbName;
    if (want) {
        for (const auto& f : out->zar->files())
            if (f.name.find(want) != std::string::npos && f.name.size() >= 4 &&
                f.name.compare(f.name.size() - 4, 4, ".cmb") == 0) {
                cmbf = &f;
                break;
            }
    }
    if (!cmbf) cmbf = out->zar->firstWithSuffix(".cmb"); // fallback: single-CMB ZARs
    if (!cmbf) { fprintf(stderr, "[SoH3D] no .cmb in %s\n", kModels[modelId].zarPath); return; }
    out->cmb = std::make_unique<SoH3D::Cmb>(out->zar->read(*cmbf));
    if (!out->cmb->ok()) { fprintf(stderr, "[SoH3D] Cmb: %s\n", out->cmb->error().c_str()); return; }
    buildFromCmb(out, /*bakedVertexColor=*/false); // characters/props: dynamic lighting, color attr unused
    printf("[SoH3D] loaded model %d (%s): %zu groups, %zu textures\n", modelId, kModels[modelId].zarPath,
           out->cGroups.size(), out->cTexs.size());
}

// Geometric bounding-box diagonal of a model's draw groups, in the model's own
// local space. Used by the auto-scale path as a rotation-invariant size measure: the
// world scale for an auto-replaced actor = (measured N64 world bbox diagonal) / (this
// OoT3D model diagonal). Returns 0 if the model has no geometry.
static float bboxDiag(const std::vector<SoH3D::CmbDrawGroup>& groups) {
    float mn[3] = { 1e30f, 1e30f, 1e30f }, mx[3] = { -1e30f, -1e30f, -1e30f };
    bool any = false;
    for (const auto& g : groups)
        for (const auto& v : g.verts) {
            any = true;
            for (int k = 0; k < 3; k++) {
                mn[k] = std::min(mn[k], v.pos[k]);
                mx[k] = std::max(mx[k], v.pos[k]);
            }
        }
    if (!any) return 0.0f;
    float dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Local-space Y (height) extent of a model's draw groups. The auto-scale path matches
// this against the N64 actor's measured world height (the dimension the manual scales
// were calibrated on), so worldScale = N64_world_height / model_height. Returns 0 if no
// geometry. OoT3D actor/prop models are authored Y-up.
static float bboxHeight(const std::vector<SoH3D::CmbDrawGroup>& groups) {
    float mn = 1e30f, mx = -1e30f;
    for (const auto& g : groups)
        for (const auto& v : g.verts) {
            mn = std::min(mn, v.pos[1]);
            mx = std::max(mx, v.pos[1]);
        }
    return (mn <= mx) ? (mx - mn) : 0.0f;
}

// Heuristic to pick a ZAR's MAIN model CMB when it holds several. OoT3D actor ZARs
// often bundle debris/effect variants (a crate's "hahen" shards, a "modelT" effect
// mesh) alongside the intact model. We skip those by name and, among the remainder,
// pick the CMB with the largest geometry (the main body). Returns nullptr if none.
static bool isDebrisCmbName(const std::string& n) {
    static const char* kSkip[] = { "hahen", "broke", "_bf", "kakera", "fragment" };
    std::string lo = n;
    for (auto& c : lo) c = (char)std::tolower((unsigned char)c);
    for (const char* s : kSkip)
        if (lo.find(s) != std::string::npos) return true;
    return false;
}

// A CMB whose geometry is FLAT (one bbox dimension ~0) is a billboard/sprite/decal quad
// (e.g. wood02's wd_model [800,655,0], the *_modelT transparency sprites), not a real 3D
// model. The auto path skips these so it picks the actual mesh (a 3D tree, not a flat white
// quad on the ground). True if the smallest extent is a tiny fraction of the largest.
static bool isFlatGroups(const std::vector<SoH3D::CmbDrawGroup>& groups) {
    float mn[3] = { 1e30f, 1e30f, 1e30f }, mx[3] = { -1e30f, -1e30f, -1e30f };
    bool any = false;
    for (const auto& g : groups)
        for (const auto& v : g.verts) {
            any = true;
            for (int k = 0; k < 3; k++) {
                mn[k] = std::min(mn[k], v.pos[k]);
                mx[k] = std::max(mx[k], v.pos[k]);
            }
        }
    if (!any) return true;
    float e0 = mx[0] - mn[0], e1 = mx[1] - mn[1], e2 = mx[2] - mn[2];
    float emax = std::max(e0, std::max(e1, e2));
    float emin = std::min(e0, std::min(e1, e2));
    return emax <= 1e-3f || emin < 0.02f * emax;
}

static size_t vertCountGroups(const std::vector<SoH3D::CmbDrawGroup>& groups) {
    size_t n = 0;
    for (const auto& g : groups) n += g.verts.size();
    return n;
}

// Load an auto-replaced actor model: read the ZAR at its registered path, pick the main
// CMB (largest non-debris), build draw groups. No hand-tuned cmbName — the heuristic
// generalizes the manual selection used by the explicit kModels[] table. Characters/props
// are dynamically lit, so vertex color is forced white like loadActorModel.
static void loadAutoModel(int modelId, LoadedModel* out) {
    int idx = modelId - kAutoModelBase;
    if (idx < 0 || idx >= (int)g_autoModelPaths.size()) return;
    const std::string& zarPath = g_autoModelPaths[idx];
    SoH3D::CtrRom* r = rom();
    if (!r) return;
    auto zarBytes = r->read(zarPath);
    if (zarBytes.empty()) { fprintf(stderr, "[SoH3D] auto: zar not found: %s\n", zarPath.c_str()); return; }
    out->zar = std::make_unique<SoH3D::Zar>(std::move(zarBytes));
    if (!out->zar->ok()) { fprintf(stderr, "[SoH3D] auto Zar %s: %s\n", zarPath.c_str(), out->zar->error().c_str()); return; }

    // Pick the MAIN model CMB. Parse each candidate once (one-time per object). Prefer the
    // most-detailed real mesh: skip debris (by name) and flat billboard/sprite quads (e.g.
    // wood02's wd_model is a flat [800,655,0] decal that, picked by raw size, rendered as a
    // white quad on the ground). Among the rest, the CMB with the most vertices is the main
    // body (a 3D tree, not its sprite LOD). Fall back progressively so a ZAR with only
    // flat/debris CMBs still yields something rather than nothing.
    const SoH3D::ZarFile* best = nullptr;
    std::unique_ptr<SoH3D::Cmb> bestCmb;
    size_t bestVerts = 0;
    const SoH3D::ZarFile* fbFile = nullptr; // best non-debris (incl. flat), by diagonal
    std::unique_ptr<SoH3D::Cmb> fbCmb;
    float fbDiag = -1.0f;
    int nCmb = 0;
    for (const auto& f : out->zar->files()) {
        if (f.name.size() < 4 || f.name.compare(f.name.size() - 4, 4, ".cmb") != 0) continue;
        nCmb++;
        if (isDebrisCmbName(f.name)) continue;
        auto cmb = std::make_unique<SoH3D::Cmb>(out->zar->read(f));
        if (!cmb->ok()) continue;
        auto groups = cmb->buildDrawGroups();
        float d = bboxDiag(groups);
        if (d > fbDiag) { fbDiag = d; fbFile = &f; fbCmb = std::make_unique<SoH3D::Cmb>(out->zar->read(f)); }
        if (isFlatGroups(groups)) continue; // billboard/sprite/decal -> not the main mesh
        size_t nv = vertCountGroups(groups);
        if (nv > bestVerts) { bestVerts = nv; best = &f; bestCmb = std::move(cmb); }
    }
    if (!bestCmb) { best = fbFile; bestCmb = std::move(fbCmb); } // all flat? take largest non-debris
    // Last resort: if every CMB looked like debris (or none parsed), take the first .cmb.
    if (!bestCmb) {
        const SoH3D::ZarFile* f = out->zar->firstWithSuffix(".cmb");
        if (f) { bestCmb = std::make_unique<SoH3D::Cmb>(out->zar->read(*f)); best = f; }
    }
    if (!bestCmb || !bestCmb->ok()) { fprintf(stderr, "[SoH3D] auto: no usable .cmb in %s\n", zarPath.c_str()); return; }
    out->cmb = std::move(bestCmb);
    // Articulated (>1 bone) => skinned character. With no animation it would render in a
    // frozen bind/T-pose, so the auto path skips it and leaves the N64 model. Calibrated,
    // animated characters go through the explicit sModelTable (with an anim resolver).
    out->skinned = out->cmb->bones().size() > 1;
    buildFromCmb(out, /*bakedVertexColor=*/false);
    printf("[SoH3D] auto-loaded model %d (%s): cmb '%s' of %d, height=%.1f, bones=%zu%s, %zu groups, %zu textures\n",
           modelId, zarPath.c_str(), best ? best->name.c_str() : "?", nCmb, bboxHeight(out->groups),
           out->cmb->bones().size(), out->skinned ? " (skinned->skip)" : "", out->cGroups.size(), out->cTexs.size());
}

LoadedModel* loadModel(int modelId) {
    auto it = g_loaded.find(modelId);
    if (it != g_loaded.end()) return it->second.get();

    auto lm = std::make_unique<LoadedModel>();
    LoadedModel* out = lm.get();
    g_loaded[modelId] = std::move(lm);

    if (modelId >= kAutoModelBase) {
        loadAutoModel(modelId, out);
    } else if (modelId >= kSceneModelBase) {
        loadSceneRoom(modelId, out);
    } else if (modelId >= 0 && modelId < (int)(sizeof(kModels) / sizeof(kModels[0]))) {
        loadActorModel(modelId, out);
    }
    return out;
}

// Renderer provider: hand back the CPU data for a model id (loads lazily).
int provider(int modelId, const SoH3DGlGroup** groups, int* groupCount, const SoH3DGlTex** texs, int* texCount) {
    LoadedModel* lm = loadModel(modelId);
    if (!lm || !lm->ok || lm->cGroups.empty()) return 0;
    *groups = lm->cGroups.data();
    *groupCount = (int)lm->cGroups.size();
    *texs = lm->cTexs.data();
    *texCount = (int)lm->cTexs.size();
    return 1;
}

bool g_registered = false;

// --- Terrain warp: re-level the OoT3D room render mesh's walkable ground to the N64
// collision floor (so Link, who walks on N64 collision, stands on the visible ground)
// while preserving OoT3D cliff/mountain relief. We build a per-XZ displacement field
// D(x,z) = N64_floor - OoT3D_floor on a grid (structure outliers rejected, hole-filled
// from nearby ground), then shift every vertex Y by the bilinear sample. A whole column
// (ground + any building/cliff above it) shifts by the same local ground correction, so
// relief is preserved. Mirrors tools/soh3d_warp.py (the offline-verified oracle). ---

constexpr float kWarpStep = 100.0f;   // grid spacing (world units)
constexpr float kWarpReject = 120.0f; // |D| above this = structure, not ground -> hole-fill
constexpr float kNoFloor = -31000.0f; // floorFn returns <= this when there is no floor

// Topmost-or-nearest upward-facing (floor) triangle Y at (x,z) over a room's draw groups;
// returns false if no floor covers the point. If hasTarget, picks the floor hit closest
// to target (isolates the same surface across datasets, avoiding roof-vs-ground mixups).
static bool meshFloor(const std::vector<SoH3D::CmbDrawGroup>& groups, float x, float z, bool hasTarget,
                      float target, float* outY, bool lowest = false) {
    bool found = false;
    float best = 0.0f;
    for (const auto& g : groups) {
        const auto& v = g.verts;
        for (size_t i = 0; i + 2 < v.size(); i += 3) {
            const float* p0 = v[i].pos;
            const float* p1 = v[i + 1].pos;
            const float* p2 = v[i + 2].pos;
            float ax = p0[0], az = p0[2], bx = p1[0], bz = p1[2], cx = p2[0], cz = p2[2];
            float d = (bz - cz) * (ax - cx) + (cx - bx) * (az - cz);
            if (d > -1e-6f && d < 1e-6f) continue;
            float u = ((bz - cz) * (x - cx) + (cx - bx) * (z - cz)) / d;
            float w = ((cz - az) * (x - cx) + (ax - cx) * (z - cz)) / d;
            float t = 1.0f - u - w;
            if (u < -1e-4f || w < -1e-4f || t < -1e-4f) continue;
            // floor test: world normal.y > 0
            float ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
            float vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
            float ny = uz * vx - ux * vz;
            float nl = std::sqrt((uy * vz - uz * vy) * (uy * vz - uz * vy) + ny * ny +
                                 (ux * vy - uy * vx) * (ux * vy - uy * vx));
            if (nl < 1e-9f || ny / nl <= 0.5f) continue;
            float y = u * p0[1] + w * p1[1] + t * p2[1];
            if (!found) {
                best = y;
                found = true;
            } else if (lowest ? (y < best) : (hasTarget ? (std::fabs(y - target) < std::fabs(best - target)) : (y > best))) {
                best = y;
            }
        }
    }
    if (found) *outY = best;
    return found;
}

// Compute the per-XZ ground-delta field D(x,z) = N64_floor - OoT3D_floor for a scene room and
// store it on the model. The render mesh is NOT modified — actors are later offset by -D (so
// they stand on the visible OoT3D ground) via SoH3D_RoomGroundDeltaAt. This is the inverse of
// the old render-mesh warp, which had to smooth D and so smeared corrections across N64
// collision steps (ledges), wrongly lifting already-correct ground and floating fences/posts.
static void computeRoomGroundDelta(LoadedModel* lm, SoH3D_FloorFn floorFn) {
    if (lm->deltaReady || lm->groups.empty() || !floorFn) return;
    lm->deltaReady = true; // mark up front: a failed/no-op compute must not retry every frame

    float minx = 1e30f, maxx = -1e30f, minz = 1e30f, maxz = -1e30f;
    for (const auto& g : lm->groups)
        for (const auto& v : g.verts) {
            minx = std::min(minx, v.pos[0]); maxx = std::max(maxx, v.pos[0]);
            minz = std::min(minz, v.pos[2]); maxz = std::max(maxz, v.pos[2]);
        }
    if (minx > maxx) return;
    int nx = (int)((maxx - minx) / kWarpStep) + 2;
    int nz = (int)((maxz - minz) / kWarpStep) + 2;
    if ((long)nx * nz > 2000000) return; // sanity guard against pathological extents

    std::vector<float> D((size_t)nx * nz, 0.0f);
    std::vector<char> valid((size_t)nx * nz, 0);
    int nValid = 0;
    for (int j = 0; j < nz; j++) {
        for (int i = 0; i < nx; i++) {
            float x = minx + i * kWarpStep, z = minz + j * kWarpStep;
            float n64 = floorFn(x, z);
            if (n64 <= kNoFloor) continue;
            float oot;
            if (!meshFloor(lm->groups, x, z, true, n64, &oot)) continue;
            float d = n64 - oot;
            if (std::fabs(d) <= kWarpReject) {
                D[(size_t)j * nx + i] = d;
                valid[(size_t)j * nx + i] = 1;
                nValid++;
            }
        }
    }
    if (nValid == 0) return;

    // Hole-fill: BFS so structure / off-mesh cells inherit the nearest valid ground D (so an
    // actor over a spot with no OoT3D floor sample still gets a sane offset from nearby ground).
    std::vector<char> filled = valid;
    std::vector<int> q;
    q.reserve((size_t)nx * nz);
    for (int k = 0; k < nx * nz; k++)
        if (valid[k]) q.push_back(k);
    for (size_t head = 0; head < q.size(); head++) {
        int k = q[head], i = k % nx, j = k / nx;
        const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
        for (int e = 0; e < 4; e++) {
            int ni = i + di[e], nj = j + dj[e];
            if (ni < 0 || ni >= nx || nj < 0 || nj >= nz) continue;
            int nk = nj * nx + ni;
            if (!filled[nk]) {
                D[nk] = D[k];
                filled[nk] = 1;
                q.push_back(nk);
            }
        }
    }

    lm->delta = std::move(D);
    lm->dMinX = minx; lm->dMinZ = minz; lm->dNx = nx; lm->dNz = nz; lm->dStep = kWarpStep;
    fprintf(stderr, "[SoH3D] ground-delta field: %dx%d grid, %d ground cells (actors offset to OoT3D ground)\n",
            nx, nz, nValid);
}

} // namespace

extern "C" {

// Register the renderer's model provider once. Safe to call repeatedly.
void SoH3D_EnsureModelProvider(void) {
    if (!g_registered) {
        SoH3D_GL_SetModelProvider(provider);
        g_registered = true;
    }
}

float SoH3D_ModelScaleById(int modelId) {
    if (modelId < 0 || modelId >= (int)(sizeof(kModels) / sizeof(kModels[0]))) return 1.0f;
    return kModels[modelId].worldScale;
}

// Get-or-allocate a stable model id for a scene room, keyed by its ZSI path
// (/scene/<name>_<R>_info.zsi). The geometry loads lazily on first draw via the
// provider. Returns -1 if sceneName is null/empty. The game calls this from its
// room-draw hook with the OoT3D scene name (kSoH3dSceneNames) + room number.
int SoH3D_RoomModelId(const char* sceneName, int roomNum) {
    if (!sceneName || !*sceneName || roomNum < 0) return -1;
    std::string path = "/scene/" + std::string(sceneName) + "_" + std::to_string(roomNum) + "_info.zsi";
    auto it = g_sceneRoomIds.find(path);
    if (it != g_sceneRoomIds.end()) return it->second;
    int id = kSceneModelBase + (int)g_sceneRoomPaths.size();
    g_sceneRoomPaths.push_back(path);
    g_sceneRoomIds[path] = id;
    return id;
}

// Get-or-allocate a stable model id for an auto-replaced actor model, keyed by its ZAR
// path (e.g. "/actor/zelda_box.zar"). The geometry loads lazily on first draw via the
// provider. Returns -1 if zarPath is null/empty. The game calls this from the SOH3D_AUTO
// actor path with the ZAR resolved from the actor's object id (kSoH3dObjectZars).
int SoH3D_AutoModelId(const char* zarPath) {
    if (!zarPath || !*zarPath) return -1;
    std::string path(zarPath);
    auto it = g_autoModelIds.find(path);
    if (it != g_autoModelIds.end()) return it->second;
    int id = kAutoModelBase + (int)g_autoModelPaths.size();
    g_autoModelPaths.push_back(path);
    g_autoModelIds[path] = id;
    return id;
}

// Local-space Y (height) extent of a loaded model. The auto-scale path uses it as the
// OoT3D-side size when deriving worldScale = N64_world_height / model_height. Loads the
// model lazily; returns 0 if it failed to load or has no geometry.
float SoH3D_AutoModelHeight(int modelId) {
    LoadedModel* lm = loadModel(modelId);
    if (!lm || !lm->ok) return 0.0f;
    return bboxHeight(lm->groups);
}

// 1 if a loaded auto model is skinned (articulated skeleton -> the auto path leaves it to
// N64 to avoid a frozen T-pose), else 0. Loads the model lazily; treats a load failure as
// "skinned" (==skip) so a bad model never auto-replaces.
int SoH3D_AutoModelSkinned(int modelId) {
    LoadedModel* lm = loadModel(modelId);
    if (!lm || !lm->ok) return 1;
    return lm->skinned ? 1 : 0;
}

// Render-mesh floor height at world (x,z) for a loaded scene-room model (the warped
// geometry, since the warp runs in-place). Returns 0 and leaves *outY untouched if no
// floor covers the point or the model is not a loaded scene room. For verifying that the
// warp made the drawn ground match N64 (compare to the REPL `floorat`).
int SoH3D_RoomMeshFloorAt(int modelId, float x, float z, float* outY) {
    if (modelId < kSceneModelBase) return 0;
    LoadedModel* lm = loadModel(modelId);
    if (!lm || !lm->ok) return 0;
    return meshFloor(lm->groups, x, z, false, 0.0f, outY) ? 1 : 0;
}

// Compute & cache a scene-room model's ground-delta field (N64 - OoT3D per XZ), once.
// `floorFn` raycasts the N64 collision (provided by soh3d.c, which has the PlayState). The
// render mesh is NOT modified; actors are offset by -D via SoH3D_RoomGroundDeltaAt. Call
// before the room is first drawn.
void SoH3D_ComputeRoomGroundDelta(int modelId, SoH3D_FloorFn floorFn) {
    if (modelId < kSceneModelBase) return; // scene rooms only
    LoadedModel* lm = loadModel(modelId);
    if (lm && lm->ok) computeRoomGroundDelta(lm, floorFn);
}

// Sample the cached ground-delta field: *outD = N64_floor - OoT3D_floor at world (x,z) for a
// scene room (bilinear). Returns 1 on success, 0 if the model isn't a scene room or the field
// isn't ready. Actors add -(*outD) to their render Y to stand on the visible OoT3D ground.
int SoH3D_RoomGroundDeltaAt(int modelId, float x, float z, float* outD) {
    if (modelId < kSceneModelBase) return 0;
    LoadedModel* lm = loadModel(modelId);
    if (!lm || !lm->ok || !lm->deltaReady || lm->delta.empty()) return 0;
    float fx = (x - lm->dMinX) / lm->dStep, fz = (z - lm->dMinZ) / lm->dStep;
    int ix = (int)std::floor(fx), iz = (int)std::floor(fz);
    float tx = fx - ix, tz = fz - iz;
    auto cell = [&](int i, int j) -> float {
        i = i < 0 ? 0 : (i >= lm->dNx ? lm->dNx - 1 : i);
        j = j < 0 ? 0 : (j >= lm->dNz ? lm->dNz - 1 : j);
        return lm->delta[(size_t)j * lm->dNx + i];
    };
    *outD = cell(ix, iz) * (1 - tx) * (1 - tz) + cell(ix + 1, iz) * tx * (1 - tz) +
            cell(ix, iz + 1) * (1 - tx) * tz + cell(ix + 1, iz + 1) * tx * tz;
    return 1;
}

} // extern "C"

// Get-or-load the parsed CSAB for `animName` (base name or full "Anim/<n>.csab"),
// caching it on the model. Returns nullptr if missing/unparseable (logged once via
// the cached null entry). Shared by the frame- and phase-based update entry points.
static SoH3D::Csab* getCsab(LoadedModel* lm, const char* animName) {
    std::string nm(animName);
    std::string full = (nm.rfind("Anim/", 0) == 0) ? nm : ("Anim/" + nm + ".csab");
    auto it = lm->anims.find(full);
    if (it == lm->anims.end()) {
        const SoH3D::ZarFile* af = nullptr;
        for (const auto& f : lm->zar->files()) if (f.name == full) { af = &f; break; }
        std::unique_ptr<SoH3D::Csab> csab;
        if (af) {
            csab = std::make_unique<SoH3D::Csab>(lm->zar->read(*af));
            if (!csab->ok()) { fprintf(stderr, "[SoH3D] Csab %s: %s\n", full.c_str(), csab->error().c_str()); csab.reset(); }
        } else {
            fprintf(stderr, "[SoH3D] anim not found: %s\n", full.c_str());
        }
        it = lm->anims.emplace(full, std::move(csab)).first;
    }
    return it->second.get();
}

// Retarget a live N64 SkelAnime pose onto the OoT3D skeleton. `jointRots` points to the
// actor's per-limb rotations (jointTable[1..limbCount], each a Vec3s of binang x,y,z; the
// caller skips jointTable[0] which is the root translation). OoT3D bone id i corresponds to
// N64 limb (i+1) for same-rig characters (Grezzo preserved the skeletons), so bone i takes
// jointRots[i]. The N64 jointTable already encodes each limb's FULL local orientation (the
// standing pose's big rotations included -- e.g. En_Ge1 limb1 = (-90,0,-90), matching OoT3D
// bone0's rest), exactly like a CSAB rotation track REPLACES the bone's rest rotation. So we
// use the N64 rotation as the local rotation directly (Rz*Ry*Rx, same order as csab.cpp) and
// do NOT compose it with the CMB rest rotation -- composing double-applies the orientation and
// contorts the pose. Convention derived QUANTITATIVELY (tools/soh3d_anim_derive.py: diff CSAB
// ge1_s_wait skin matrices vs N64-joint-driven ones -> struct=replace, euler order ZYX wins
// over every compose variant). L = T(rest)*Rz*Ry*Rx(n64)*S(rest); skin = animWorld*bindInverse.
extern "C" void SoH3D_UpdateAnimN64(int modelId, const int16_t* jointRots, int rotCount) {
    using namespace SoH3D;
    LoadedModel* lm = loadModel(modelId);
    if (!lm || !lm->ok || !lm->cmb) { SoH3D_GL_SetBones(modelId, nullptr, 0); return; }
    const auto& bones = lm->cmb->bones();
    const auto& bind = lm->cmb->boneMatrices();
    const float kBinangToRad = 3.14159265358979f / 32768.0f;

    std::vector<Mat4> aw(bind.size(), matId());
    std::vector<char> done(bind.size(), 0);
    std::vector<const CmbBone*> byId(bind.size(), nullptr);
    for (const auto& bn : bones)
        if (bn.id >= 0 && (size_t)bn.id < byId.size()) byId[bn.id] = &bn;

    std::function<Mat4(int)> world = [&](int id) -> Mat4 {
        if (id < 0 || (size_t)id >= aw.size() || !byId[id]) return matId();
        if (done[id]) return aw[id];
        const CmbBone* bn = byId[id];
        Mat4 L = matT(bn->trans[0], bn->trans[1], bn->trans[2]);
        if (id < rotCount) {
            // Use the N64 joint rotation AS the bone's local rotation (replacing the CMB rest
            // rotation), in csab.cpp's Rz*Ry*Rx order. The jointTable already carries the full
            // limb orientation, so composing it with the rest rotation double-applies and
            // contorts (verified by tools/soh3d_anim_derive.py: replace beats compose).
            float rx = jointRots[id * 3 + 0] * kBinangToRad;
            float ry = jointRots[id * 3 + 1] * kBinangToRad;
            float rz = jointRots[id * 3 + 2] * kBinangToRad;
            L = matMul(L, matMul(matMul(matRz(rz), matRy(ry)), matRx(rx)));
        } else {
            // No live joint for this bone: keep its CMB rest orientation (bind pose).
            L = matMul(L, matMul(matMul(matRz(bn->rot[2]), matRy(bn->rot[1])), matRx(bn->rot[0])));
        }
        L = matMul(L, matS(bn->scale[0], bn->scale[1], bn->scale[2]));
        Mat4 W = (bn->parent < 0) ? L : matMul(world(bn->parent), L);
        aw[id] = W;
        done[id] = 1;
        return W;
    };
    for (const auto& bn : bones) world(bn.id);

    std::vector<std::array<float, 16>> sm(bind.size());
    for (size_t id = 0; id < bind.size(); id++) sm[id] = matMul(aw[id], matInverse(bind[id]));
    SoH3D_GL_SetBones(modelId, sm.empty() ? nullptr : sm.front().data(), (int)sm.size());
}

extern "C" {

// Set the model's GPU skinning pose to `animName` (CSAB base name, e.g. "ge1_s_wait")
// at `frame`. animName==NULL/"" resets to the bind pose. Loads the model + caches the
// parsed CSAB on first use; recomputes skin matrices each call (cheap: <=32 bones).
// Call once per game frame before the SoH3D draw. Safe to call repeatedly.
void SoH3D_UpdateAnim(int modelId, const char* animName, float frame) {
    if (!animName || !*animName) { SoH3D_GL_SetBones(modelId, nullptr, 0); return; }
    LoadedModel* lm = loadModel(modelId);
    if (!lm || !lm->ok || !lm->cmb || !lm->zar) return;

    SoH3D::Csab* anim = getCsab(lm, animName);
    if (!anim) { SoH3D_GL_SetBones(modelId, nullptr, 0); return; }

    std::vector<std::array<float, 16>> sm;
    anim->skinMatrices(*lm->cmb, frame, sm);
    // vector<array<float,16>> is contiguous -> hand the renderer a flat float buffer.
    SoH3D_GL_SetBones(modelId, sm.empty() ? nullptr : sm.front().data(), (int)sm.size());
}

} // extern "C"
