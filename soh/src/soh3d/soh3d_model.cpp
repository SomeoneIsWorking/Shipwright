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
#include "asset/pica_texture.h"
#include "fast/soh3d_gl.h"

#include <algorithm>
#include <cmath>
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
};

// Registry keyed by modelId (the index). The actor->modelId mapping lives in
// soh3d.c (which has the ACTOR_* ids); this stays pure-C++ / engine-agnostic.
//   0 = geldwoman (white Gerudo, En_Ge1)
const ModelSpec kModels[] = {
    { "/actor/zelda_ge1.zar", 0.011f },
};

// Scene-room models live in a SEPARATE id range so they never collide with the actor
// table above. A room's geometry is a single embedded CMB inside a ZSI (no skeleton,
// no animation), drawn at the world origin. Ids are allocated on demand by the game
// (SoH3D_RoomModelId) keyed by the room's ZSI path. See soh3d.c's room-draw hook.
const int kSceneModelBase = 1000;

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
    bool warped = false; // scene rooms: terrain Y already re-leveled to N64 collision
};

std::unordered_map<int, std::unique_ptr<LoadedModel>> g_loaded;
std::unique_ptr<SoH3D::CtrRom> g_rom;

// Scene-room id allocation: ZSI path -> model id (>= kSceneModelBase), and the reverse
// list so loadModel can recover the path from the id.
std::unordered_map<std::string, int> g_sceneRoomIds;
std::vector<std::string> g_sceneRoomPaths; // index = modelId - kSceneModelBase

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
    const SoH3D::ZarFile* cmbf = out->zar->firstWithSuffix(".cmb");
    if (!cmbf) { fprintf(stderr, "[SoH3D] no .cmb in %s\n", kModels[modelId].zarPath); return; }
    out->cmb = std::make_unique<SoH3D::Cmb>(out->zar->read(*cmbf));
    if (!out->cmb->ok()) { fprintf(stderr, "[SoH3D] Cmb: %s\n", out->cmb->error().c_str()); return; }
    buildFromCmb(out, /*bakedVertexColor=*/false); // characters/props: dynamic lighting, color attr unused
    printf("[SoH3D] loaded model %d (%s): %zu groups, %zu textures\n", modelId, kModels[modelId].zarPath,
           out->cGroups.size(), out->cTexs.size());
}

LoadedModel* loadModel(int modelId) {
    auto it = g_loaded.find(modelId);
    if (it != g_loaded.end()) return it->second.get();

    auto lm = std::make_unique<LoadedModel>();
    LoadedModel* out = lm.get();
    g_loaded[modelId] = std::move(lm);

    if (modelId >= kSceneModelBase) {
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
                      float target, float* outY) {
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
            } else if (hasTarget ? (std::fabs(y - target) < std::fabs(best - target)) : (y > best)) {
                best = y;
            }
        }
    }
    if (found) *outY = best;
    return found;
}

static void warpRoomMesh(LoadedModel* lm, SoH3D_FloorFn floorFn) {
    if (lm->warped || lm->groups.empty() || !floorFn) return;
    lm->warped = true; // mark up front: a failed/no-op warp must not retry every frame

    // Mesh XZ bounds.
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

    // Hole-fill: BFS so structure / off-mesh cells inherit the nearest valid ground D.
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

    auto sample = [&](float x, float z) -> float {
        float fx = (x - minx) / kWarpStep, fz = (z - minz) / kWarpStep;
        int ix = (int)std::floor(fx), iz = (int)std::floor(fz);
        float tx = fx - ix, tz = fz - iz;
        auto cell = [&](int i, int j) -> float {
            i = i < 0 ? 0 : (i >= nx ? nx - 1 : i);
            j = j < 0 ? 0 : (j >= nz ? nz - 1 : j);
            return D[(size_t)j * nx + i];
        };
        return cell(ix, iz) * (1 - tx) * (1 - tz) + cell(ix + 1, iz) * tx * (1 - tz) +
               cell(ix, iz + 1) * (1 - tx) * tz + cell(ix + 1, iz + 1) * tx * tz;
    };

    for (auto& g : lm->groups)
        for (auto& v : g.verts)
            v.pos[1] += sample(v.pos[0], v.pos[2]);
    fprintf(stderr, "[SoH3D] terrain warp: %dx%d grid, %d ground cells, mesh re-leveled to N64\n", nx, nz, nValid);
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

// Re-level a scene-room model's render ground to the N64 collision floor (idempotent;
// the warp runs once per model). `floorFn` raycasts the N64 collision (provided by
// soh3d.c, which has the PlayState). Call before the room is first drawn.
void SoH3D_WarpRoomToN64(int modelId, SoH3D_FloorFn floorFn) {
    if (modelId < kSceneModelBase) return; // scene rooms only
    LoadedModel* lm = loadModel(modelId);
    if (lm && lm->ok) warpRoomMesh(lm, floorFn);
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
