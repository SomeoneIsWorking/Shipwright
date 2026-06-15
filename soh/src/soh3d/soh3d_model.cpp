// SoH3D model bridge: connects the runtime C++ asset loader (asset/) to the
// libultraship direct-GL renderer (SoH3D_GL_*). Owns the model registry (actor id
// -> 3DS asset + world scale), lazily parses+decodes a model from the decrypted
// .3ds the first time it's drawn (on the render thread, GL current), and serves the
// renderer's provider callback with the CPU data to upload. No baked-in C arrays;
// the .3ds path comes from env SOH3D_3DS_ROM (never hardcoded — repo rule).
#include "asset/ctr_rom.h"
#include "asset/zar.h"
#include "asset/cmb.h"
#include "asset/csab.h"
#include "asset/pica_texture.h"
#include "fast/soh3d_gl.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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

// Loaded CPU data for a model, kept alive so the renderer can upload from it and
// so the provider can hand back stable pointers.
struct LoadedModel {
    std::vector<SoH3D::CmbDrawGroup> groups;       // interleaved verts (CmbVertex == SoH3DGlVtx layout)
    std::vector<std::vector<uint8_t>> texRgba;     // decoded RGBA8 per CMB texture
    std::vector<SoH3DGlGroup> cGroups;             // C-API view
    std::vector<SoH3DGlTex> cTexs;                 // C-API view
    bool ok = false;
};

std::unordered_map<int, std::unique_ptr<LoadedModel>> g_loaded;
std::unique_ptr<SoH3D::CtrRom> g_rom;

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

LoadedModel* loadModel(int modelId) {
    auto it = g_loaded.find(modelId);
    if (it != g_loaded.end()) return it->second.get();

    auto lm = std::make_unique<LoadedModel>();
    LoadedModel* out = lm.get();
    g_loaded[modelId] = std::move(lm);

    if (modelId < 0 || modelId >= (int)(sizeof(kModels) / sizeof(kModels[0]))) return out;
    SoH3D::CtrRom* r = rom();
    if (!r) return out;

    auto zarBytes = r->read(kModels[modelId].zarPath);
    if (zarBytes.empty()) { fprintf(stderr, "[SoH3D] zar not found: %s\n", kModels[modelId].zarPath); return out; }
    SoH3D::Zar zar(std::move(zarBytes));
    if (!zar.ok()) { fprintf(stderr, "[SoH3D] Zar: %s\n", zar.error().c_str()); return out; }
    const SoH3D::ZarFile* cmbf = zar.firstWithSuffix(".cmb");
    if (!cmbf) { fprintf(stderr, "[SoH3D] no .cmb in %s\n", kModels[modelId].zarPath); return out; }
    SoH3D::Cmb cmb(zar.read(*cmbf));
    if (!cmb.ok()) { fprintf(stderr, "[SoH3D] Cmb: %s\n", cmb.error().c_str()); return out; }

    // Optional CSAB skinning: SOH3D_ANIM=<csab base name> [SOH3D_FRAME=<float>].
    // Loaded from the same zar; skin matrices applied once at this frame (the live
    // per-frame in-game path is a later layer — this verifies a static deformed frame).
    const char* animName = getenv("SOH3D_ANIM");
    if (animName && *animName) {
        std::string nm(animName);
        std::string full = (nm.rfind("Anim/", 0) == 0) ? nm : ("Anim/" + nm + ".csab");
        const SoH3D::ZarFile* af = nullptr;
        for (const auto& f : zar.files()) if (f.name == full) { af = &f; break; }
        if (af) {
            SoH3D::Csab anim(zar.read(*af));
            float frame = getenv("SOH3D_FRAME") ? (float)atof(getenv("SOH3D_FRAME")) : 0.0f;
            if (anim.ok()) {
                std::vector<std::array<float, 16>> sm;
                anim.skinMatrices(cmb, frame, sm);
                out->groups = cmb.buildDrawGroupsSkinned(sm.data(), sm.size());
                fprintf(stderr, "[SoH3D] applied anim %s frame %.2f (%d bones, %d anods)\n",
                        full.c_str(), frame, anim.boneCount(), anim.animNodeCount());
            } else {
                fprintf(stderr, "[SoH3D] Csab %s: %s\n", full.c_str(), anim.error().c_str());
                out->groups = cmb.buildDrawGroups();
            }
        } else {
            fprintf(stderr, "[SoH3D] anim not found: %s\n", full.c_str());
            out->groups = cmb.buildDrawGroups();
        }
    } else {
        out->groups = cmb.buildDrawGroups();
    }

    // Decode every texture (index aligns with CMB texture index / materialTexture()).
    const auto& texs = cmb.textures();
    out->texRgba.resize(texs.size());
    out->cTexs.resize(texs.size());
    for (size_t i = 0; i < texs.size(); i++) {
        auto raw = cmb.textureRaw(texs[i]);
        out->texRgba[i] = SoH3D::PicaDecode(texs[i].glFormat(), texs[i].width, texs[i].height, raw);
        out->cTexs[i] = { out->texRgba[i].data(), texs[i].width, texs[i].height };
    }

    // Build the C-API group views.
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
        out->cGroups.push_back(cg);
    }
    out->ok = true;
    printf("[SoH3D] loaded model %d (%s): %zu groups, %zu textures\n", modelId, kModels[modelId].zarPath,
           out->cGroups.size(), out->cTexs.size());
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

} // extern "C"
