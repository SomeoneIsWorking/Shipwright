// charcompare — N64 (OoT) viewport. See cc_n64.h.
#include "cc_n64.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <ship/Context.h>
#include <ship/resource/ResourceManager.h>
#include <ship/resource/ResourceLoader.h>
#include <ship/resource/File.h> // RESOURCE_FORMAT_BINARY

#include <fast/resource/ResourceType.h>
#include <fast/resource/factory/DisplayListFactory.h>
#include <fast/resource/factory/VertexFactory.h>
#include <fast/resource/factory/TextureFactory.h>
#include <fast/resource/factory/MatrixFactory.h>

#include "soh/resource/type/SohResourceType.h"
#include "soh/resource/type/Skeleton.h"
#include "soh/resource/type/SkeletonLimb.h"
#include "soh/resource/type/Animation.h"
#include "soh/resource/importer/SkeletonFactory.h"
#include "soh/resource/importer/SkeletonLimbFactory.h"
#include "soh/resource/importer/AnimationFactory.h"

#include <fast/interpreter.h> // SCREEN_WIDTH/HEIGHT

namespace cc {

// --- N64 MtxF math (column-major mf[col][row], M*v convention, as OoT sys_matrix) -------------
static float sinS(int16_t a) { return sinf((float)a * (3.14159265358979f / 32768.0f)); }
static float cosS(int16_t a) { return cosf((float)a * (3.14159265358979f / 32768.0f)); }

static void matIdentity(MtxF& m) {
    memset(m.mf, 0, sizeof(m.mf));
    m.mf[0][0] = m.mf[1][1] = m.mf[2][2] = m.mf[3][3] = 1.0f;
}

// Verbatim port of Matrix_TranslateRotateZYX (sys_matrix.c): cmf = cmf * T(t) * Rz * Ry * Rx.
static void matTranslateRotateZYX(MtxF* cmf, const float t[3], const int16_t r[3]) {
    float sin = sinS(r[2]), cos = cosS(r[2]);
    float temp1, temp2;
    temp1 = cmf->xx; temp2 = cmf->xy;
    cmf->xw += temp1 * t[0] + temp2 * t[1] + cmf->xz * t[2];
    cmf->xx = temp1 * cos + temp2 * sin; cmf->xy = temp2 * cos - temp1 * sin;
    temp1 = cmf->yx; temp2 = cmf->yy;
    cmf->yw += temp1 * t[0] + temp2 * t[1] + cmf->yz * t[2];
    cmf->yx = temp1 * cos + temp2 * sin; cmf->yy = temp2 * cos - temp1 * sin;
    temp1 = cmf->zx; temp2 = cmf->zy;
    cmf->zw += temp1 * t[0] + temp2 * t[1] + cmf->zz * t[2];
    cmf->zx = temp1 * cos + temp2 * sin; cmf->zy = temp2 * cos - temp1 * sin;
    temp1 = cmf->wx; temp2 = cmf->wy;
    cmf->ww += temp1 * t[0] + temp2 * t[1] + cmf->wz * t[2];
    cmf->wx = temp1 * cos + temp2 * sin; cmf->wy = temp2 * cos - temp1 * sin;
    if (r[1] != 0) {
        sin = sinS(r[1]); cos = cosS(r[1]);
        temp1 = cmf->xx; temp2 = cmf->xz; cmf->xx = temp1 * cos - temp2 * sin; cmf->xz = temp1 * sin + temp2 * cos;
        temp1 = cmf->yx; temp2 = cmf->yz; cmf->yx = temp1 * cos - temp2 * sin; cmf->yz = temp1 * sin + temp2 * cos;
        temp1 = cmf->zx; temp2 = cmf->zz; cmf->zx = temp1 * cos - temp2 * sin; cmf->zz = temp1 * sin + temp2 * cos;
        temp1 = cmf->wx; temp2 = cmf->wz; cmf->wx = temp1 * cos - temp2 * sin; cmf->wz = temp1 * sin + temp2 * cos;
    }
    if (r[0] != 0) {
        sin = sinS(r[0]); cos = cosS(r[0]);
        temp1 = cmf->xy; temp2 = cmf->xz; cmf->xy = temp1 * cos + temp2 * sin; cmf->xz = temp2 * cos - temp1 * sin;
        temp1 = cmf->yy; temp2 = cmf->yz; cmf->yy = temp1 * cos + temp2 * sin; cmf->yz = temp2 * cos - temp1 * sin;
        temp1 = cmf->zy; temp2 = cmf->zz; cmf->zy = temp1 * cos + temp2 * sin; cmf->zz = temp2 * cos - temp1 * sin;
        temp1 = cmf->wy; temp2 = cmf->wz; cmf->wy = temp1 * cos + temp2 * sin; cmf->wz = temp2 * cos - temp1 * sin;
    }
}

// Transform a model-space point by an MtxF (column-major, M*v) — for FK bbox.
static void matApply(const MtxF& m, const float v[3], float out[3]) {
    for (int row = 0; row < 3; row++)
        out[row] = m.mf[0][row] * v[0] + m.mf[1][row] * v[1] + m.mf[2][row] * v[2] + m.mf[3][row];
}

// --- Factory registration --------------------------------------------------------------------
static bool g_registered = false;
void RegisterN64Factories() {
    if (g_registered) return;
    auto loader = Ship::Context::GetRawInstance()->GetResourceManager()->GetResourceLoader();
    using namespace Fast;
    loader->RegisterResourceFactory(std::make_shared<ResourceFactoryBinaryTextureV0>(), RESOURCE_FORMAT_BINARY,
                                    "Texture", static_cast<uint32_t>(ResourceType::Texture), 0);
    loader->RegisterResourceFactory(std::make_shared<ResourceFactoryBinaryTextureV1>(), RESOURCE_FORMAT_BINARY,
                                    "Texture", static_cast<uint32_t>(ResourceType::Texture), 1);
    loader->RegisterResourceFactory(std::make_shared<ResourceFactoryBinaryVertexV0>(), RESOURCE_FORMAT_BINARY, "Vertex",
                                    static_cast<uint32_t>(ResourceType::Vertex), 0);
    loader->RegisterResourceFactory(std::make_shared<ResourceFactoryBinaryDisplayListV0>(), RESOURCE_FORMAT_BINARY,
                                    "DisplayList", static_cast<uint32_t>(ResourceType::DisplayList), 0);
    loader->RegisterResourceFactory(std::make_shared<ResourceFactoryBinaryMatrixV0>(), RESOURCE_FORMAT_BINARY, "Matrix",
                                    static_cast<uint32_t>(ResourceType::Matrix), 0);
    loader->RegisterResourceFactory(std::make_shared<SOH::ResourceFactoryBinarySkeletonV0>(), RESOURCE_FORMAT_BINARY,
                                    "Skeleton", static_cast<uint32_t>(SOH::ResourceType::SOH_Skeleton), 0);
    loader->RegisterResourceFactory(std::make_shared<SOH::ResourceFactoryBinarySkeletonLimbV0>(), RESOURCE_FORMAT_BINARY,
                                    "SkeletonLimb", static_cast<uint32_t>(SOH::ResourceType::SOH_SkeletonLimb), 0);
    loader->RegisterResourceFactory(std::make_shared<SOH::ResourceFactoryBinaryAnimationV0>(), RESOURCE_FORMAT_BINARY,
                                    "Animation", static_cast<uint32_t>(SOH::ResourceType::SOH_Animation), 0);
    g_registered = true;
    fprintf(stderr, "[cc_n64] registered resource factories\n");
}

static Ship::ResourceManager* rm() { return Ship::Context::GetRawInstance()->GetResourceManager().get(); }

void SetAnimN64(ModelN64& m, const std::string& animName) {
    if (animName.empty()) return;
    std::string path = m.objectPath + "/" + animName;
    auto res = rm()->LoadResource(path);
    if (!res) { fprintf(stderr, "[cc_n64] anim not found: %s\n", path.c_str()); return; }
    m.animRes = res;
    m.anim = res->GetRawPointer();
    m.animName = animName;
    auto* ah = (SOH::AnimationHeader*)m.anim;
    m.animFrameCount = ah->common.frameCount;
}

ModelN64 LoadN64(const std::string& objectPath, const std::string& skelName,
                 const std::vector<std::string>& animNames) {
    RegisterN64Factories();
    ModelN64 m;
    m.objectPath = objectPath;
    m.skelName = skelName;
    m.anims = animNames;

    std::string skelPath = objectPath + "/" + skelName;
    auto res = rm()->LoadResource(skelPath);
    if (!res) { m.error = "skeleton not found: " + skelPath; return m; }
    m.skelRes = res;
    m.skelHeader = res->GetRawPointer();
    auto* fsh = (SOH::FlexSkeletonHeader*)m.skelHeader;
    m.limbCount = fsh->sh.limbCount;
    m.ok = (m.limbCount > 0 && fsh->sh.segment != nullptr);
    if (!m.ok) { m.error = "skeleton has no limbs"; return m; }

    if (!animNames.empty()) SetAnimN64(m, animNames[0]);

    fprintf(stderr, "[cc_n64] loaded %s: %d limbs, %zu anims, anim '%s' (%d frames)\n", skelPath.c_str(), m.limbCount,
            animNames.size(), m.animName.c_str(), m.animFrameCount);
    return m;
}

// Sample the animation into jointTable[limbCount+1] (out[0]=root pos, out[1..]=rotations).
static void sampleAnim(const ModelN64& m, float frame, std::vector<std::array<int16_t, 3>>& out) {
    out.assign(m.limbCount + 1, { 0, 0, 0 });
    if (!m.anim || m.animFrameCount <= 0) return;
    auto* ah = (SOH::AnimationHeader*)m.anim;
    const int16_t* fd = ah->frameData;
    const SOH::JointIndex* ji = ah->jointIndices;
    uint16_t smax = ah->staticIndexMax;
    int f = ((int)frame % m.animFrameCount + m.animFrameCount) % m.animFrameCount;
    for (int i = 0; i < m.limbCount + 1; i++) {
        out[i][0] = (ji[i].x >= smax) ? fd[ji[i].x + f] : fd[ji[i].x];
        out[i][1] = (ji[i].y >= smax) ? fd[ji[i].y + f] : fd[ji[i].y];
        out[i][2] = (ji[i].z >= smax) ? fd[ji[i].z + f] : fd[ji[i].z];
    }
}

// Recursive limb walk mirroring SkelAnime_DrawLimbOpa. `parent` is the parent's MtxF; emit each
// limb's loaded matrix + gSPDisplayList. fkBbox (non-null) => only FK: accumulate joint bbox, no emit.
static void walkLimb(ModelN64& m, int limbIndex, const MtxF& parent,
                     const std::vector<std::array<int16_t, 3>>& joints, std::vector<Gfx>* dl,
                     std::unordered_map<Mtx*, MtxF>* mtx, DlistKeys* keys, float* lo, float* hi) {
    auto** seg = (SOH::StandardLimb**)((SOH::FlexSkeletonHeader*)m.skelHeader)->sh.segment;
    SOH::StandardLimb* limb = seg[limbIndex];
    bool isRoot = (limbIndex == 0);
    float pos[3] = { (float)limb->jointPos.x, (float)limb->jointPos.y, (float)limb->jointPos.z };
    if (isRoot) { pos[0] = joints[0][0]; pos[1] = joints[0][1]; pos[2] = joints[0][2]; }
    const auto& r = joints[limbIndex + 1];
    int16_t rot[3] = { r[0], r[1], r[2] };

    MtxF cur = parent;
    matTranslateRotateZYX(&cur, pos, rot);

    if (lo) { // FK bbox: track the limb origin (0,0,0 in limb space)
        float o[3]; float zero[3] = { 0, 0, 0 };
        matApply(cur, zero, o);
        for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], o[k]); hi[k] = std::max(hi[k], o[k]); }
    } else if (limb->dList != nullptr) {
        // limb->dList is an OTR PATH string ("__OTR__objects/.../someDL"), not a runnable Gfx*.
        // Mirror the game's gSPDisplayList wrapper (GbiWrap.cpp): resolve the path to the
        // DisplayList resource's real Gfx* at build time, then emit a plain G_DL of it.
        std::string p = (const char*)limb->dList;
        if (p.rfind("__OTR__", 0) == 0) p = p.substr(7);
        auto res = rm()->LoadResource(p);
        Gfx* g = res ? (Gfx*)res->GetRawPointer() : nullptr;
        if (g && getenv("CC_N64_DUMPDL")) {
            fprintf(stderr, "[cc_n64] limb %d DL %s opcodes:", limbIndex, p.c_str());
            for (int i = 0; i < 24; i++) {
                uint8_t op = (uint8_t)((uintptr_t)g[i].words.w0 >> 24);
                fprintf(stderr, " %02X", op);
                if (op == 0xDF) break; // G_ENDDL (F3DEX2)
            }
            fprintf(stderr, "\n");
        }
        if (g) {
            m.limbRes.push_back(res); // keep the DL resource (and its resolved pointer) alive
            keys->mtxStore.push_back(std::make_unique<Mtx>());
            Mtx* key = keys->mtxStore.back().get();
            (*mtx)[key] = cur;
            { Gfx gm = gsSPMatrix(key, G_MTX_NOPUSH | G_MTX_LOAD | G_MTX_MODELVIEW); dl->push_back(gm); }
            if (!getenv("CC_N64_NODL")) { Gfx gd = gsSPDisplayList(g); dl->push_back(gd); }
        }
    }

    if (limb->child != LIMB_DONE)
        walkLimb(m, limb->child, cur, joints, dl, mtx, keys, lo, hi);
    if (limb->sibling != LIMB_DONE)
        walkLimb(m, limb->sibling, parent, joints, dl, mtx, keys, lo, hi);
}

void EmitDlistN64(ModelN64& m, float frame, std::vector<Gfx>& dl, std::unordered_map<Mtx*, MtxF>& mtx,
                  DlistKeys& keys, float rx, float ry, float rz) {
    if (!m.ok) return;
    std::vector<std::array<int16_t, 3>> joints;
    sampleAnim(m, frame, joints);

    // FK pass to get the posed joint bbox (auto-fit framing).
    float lo[3] = { 1e30f, 1e30f, 1e30f }, hi[3] = { -1e30f, -1e30f, -1e30f };
    MtxF id; matIdentity(id);
    walkLimb(m, 0, id, joints, nullptr, nullptr, nullptr, lo, hi);
    float ctr[3], ext[3];
    for (int k = 0; k < 3; k++) {
        ctr[k] = (lo[k] + hi[k]) * 0.5f;
        ext[k] = std::max((hi[k] - lo[k]) * 0.5f, 1.0f);
    }

    // Framing matrix F (column-major M*v), same NDC fit as cc_3ds: scale + R(rx,ry,rz), center.
    const float fit = 0.8f / std::max(ext[0], ext[1]);
    const float fitZ = 0.4f / ext[2];
    const float S[3] = { fit, fit, fitZ };
    auto rad = [](float d) { return d * 3.14159265358979f / 180.0f; };
    float cx = cosf(rad(rx)), sx = sinf(rad(rx)), cyr = cosf(rad(ry)), syr = sinf(rad(ry)), cz = cosf(rad(rz)),
          sz = sinf(rad(rz));
    float Rx[3][3] = { { 1, 0, 0 }, { 0, cx, -sx }, { 0, sx, cx } };
    float Ry[3][3] = { { cyr, 0, syr }, { 0, 1, 0 }, { -syr, 0, cyr } };
    float Rz[3][3] = { { cz, -sz, 0 }, { sz, cz, 0 }, { 0, 0, 1 } };
    auto mul3 = [](const float A[3][3], const float B[3][3], float O[3][3]) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) { O[i][j] = 0; for (int k = 0; k < 3; k++) O[i][j] += A[i][k] * B[k][j]; }
    };
    float Rzy[3][3], R[3][3];
    mul3(Rz, Ry, Rzy);
    mul3(Rzy, Rx, R);
    // F maps point p: clip_j = S_j * sum_k R[j][k]*(p_k - ctr_k) + bias_j. In column-major MtxF
    // (mf[col][row], clip_row = sum_col mf[col][row]*p_col + mf[3][row]):
    MtxF F;
    memset(F.mf, 0, sizeof(F.mf));
    const float bias[3] = { 0, 0, 0.5f };
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) F.mf[col][row] = S[row] * R[row][col];
        float t = bias[row];
        for (int col = 0; col < 3; col++) t -= F.mf[col][row] * ctr[col];
        F.mf[3][row] = t;
    }
    F.mf[3][3] = 1.0f;

    // Viewport + scissor (full screen for now; the side-by-side split is set in main/Phase 4).
    keys.vpStore.push_back(std::make_unique<Vp>());
    Vp* vp = keys.vpStore.back().get();
    vp->vp.vscale[0] = (SCREEN_WIDTH / 2) * 4;  vp->vp.vscale[1] = (SCREEN_HEIGHT / 2) * 4;
    vp->vp.vscale[2] = G_MAXZ;                  vp->vp.vscale[3] = 0;
    vp->vp.vtrans[0] = (SCREEN_WIDTH / 2) * 4;  vp->vp.vtrans[1] = (SCREEN_HEIGHT / 2) * 4;
    vp->vp.vtrans[2] = 0;                       vp->vp.vtrans[3] = 0;
    { Gfx g = gsSPViewport(vp); dl.push_back(g); }
    { Gfx g = gsDPSetScissor(G_SC_NON_INTERLACE, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT); dl.push_back(g); }
    // Identity projection (the framing is baked into the per-limb modelview, like cc_3ds).
    keys.mtxStore.push_back(std::make_unique<Mtx>());
    Mtx* projKey = keys.mtxStore.back().get();
    MtxF& proj = mtx[projKey];
    matIdentity(proj);
    { Gfx g = gsSPMatrix(projKey, G_MTX_NOPUSH | G_MTX_LOAD | G_MTX_PROJECTION); dl.push_back(g); }
    // A textured-tri combiner so the limb DLs (which set their own combiner) start from a sane state.
    { Gfx g = gsDPSetCombineMode(G_CC_MODULATERGB, G_CC_MODULATERGB); dl.push_back(g); }

    walkLimb(m, 0, F, joints, &dl, &mtx, &keys, nullptr, nullptr);
}

} // namespace cc
