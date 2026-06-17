#include "csab.h"
#include "cmb.h"
#include "mat4.h"
#include <cstring>
#include <cmath>
#include <functional>

namespace SoH3D {

static uint16_t u16(const uint8_t* b, size_t o) { uint16_t v; memcpy(&v, b + o, 2); return v; }
static int16_t  s16(const uint8_t* b, size_t o) { int16_t v; memcpy(&v, b + o, 2); return v; }
static uint32_t u32(const uint8_t* b, size_t o) { uint32_t v; memcpy(&v, b + o, 4); return v; }
static float    f32(const uint8_t* b, size_t o) { float v; memcpy(&v, b + o, 4); return v; }

Csab::Track Csab::parseTrack(uint32_t o, bool isRotInt16) const {
    const uint8_t* b = mData.data();
    Track t;
    t.type = (int)u32(b, o);
    uint32_t nkf = u32(b, o + 4);
    t.timeStart = (int)u32(b, o + 8);
    t.timeEnd = (int)u32(b, o + 0xC) + 1;
    uint32_t p = o + 0x10;
    t.present = true;
    t.frames.reserve(nkf);
    if (t.type == LINEAR) {
        for (uint32_t i = 0; i < nkf; i++) {
            Keyframe k{ (float)u32(b, p), f32(b, p + 4), 0, 0 };
            t.frames.push_back(k);
            p += 8;
        }
    } else if (t.type == HERMITE) {
        if (isRotInt16) {
            // Quantized rotation keyframes (8 bytes): u16 time, then value/in-tangent/out-tangent as
            // s16 fixed-point ANGLES — full circle = 0x10000, so radians = s16 * 2pi/0x10000 = s16 *
            // pi/0x8000. Tangents are angle/frame, same scale. (ge1 uses float HERMITE so this path
            // was previously untested; using the s16s raw as radians exploded the link mesh.)
            const float kAngle = 3.14159265358979f / 32768.0f;
            for (uint32_t i = 0; i < nkf; i++) {
                Keyframe k{ (float)u16(b, p), s16(b, p + 2) * kAngle, s16(b, p + 4) * kAngle,
                            s16(b, p + 6) * kAngle };
                t.frames.push_back(k);
                p += 8;
            }
        } else {
            for (uint32_t i = 0; i < nkf; i++) {
                Keyframe k{ (float)u32(b, p), f32(b, p + 4), f32(b, p + 8), f32(b, p + 0xC) };
                t.frames.push_back(k);
                p += 0x10;
            }
        }
    } else {
        t.present = false; // CONSTANT / unknown -> no track (fall back to rest TRS)
    }
    return t;
}

Csab::AnimNode Csab::parseAnod(uint32_t o) const {
    const uint8_t* b = mData.data();
    AnimNode n;
    n.boneIndex = u16(b, o + 4);
    n.isRotInt16 = u16(b, o + 6) != 0;
    for (int i = 0; i < 9; i++) {
        uint16_t off = u16(b, o + 8 + 2 * i);
        // isRotInt16 applies only to the rotation slots (3,4,5 = rX/rY/rZ); translation/scale tracks
        // stay float even in an int16 anod. (Translation is usually LINEAR float anyway.)
        if (off) n.tracks[i] = parseTrack(o + off, n.isRotInt16 && (i >= 3 && i <= 5));
    }
    return n;
}

Csab::Csab(std::vector<uint8_t> data) : mData(std::move(data)) {
    const uint8_t* b = mData.data();
    if (mData.size() < 0x38 || memcmp(b, "csab", 4) != 0) { mErr = "not a CSAB"; return; }
    uint32_t subver = u32(b, 0x08);
    if (subver != 0x03) { mErr = "only Ocarina subversion 3 supported"; return; }
    uint32_t anodBase = u32(b, 0x14); // = 0x18
    mDuration = (int)u32(b, 0x28) + 1;
    uint32_t anodCount = u32(b, 0x30);
    mBoneCount = (int)u32(b, 0x34);
    mBoneToAnim.resize(mBoneCount);
    for (int i = 0; i < mBoneCount; i++) mBoneToAnim[i] = s16(b, 0x38 + 2 * i);
    uint32_t idx = 0x38 + 2 * (uint32_t)mBoneCount;
    idx = (idx + 3) & ~3u;
    mNodes.reserve(anodCount);
    for (uint32_t i = 0; i < anodCount; i++) {
        uint32_t off = u32(b, idx + 4 * i);
        if (memcmp(b + anodBase + off, "anod", 4) != 0) { mErr = "bad anod"; return; }
        mNodes.push_back(parseAnod(anodBase + off));
    }
    mOk = true;
}

const Csab::AnimNode* Csab::nodeForBone(int boneId) const {
    if (boneId >= 0 && boneId < (int)mBoneToAnim.size()) {
        int ai = mBoneToAnim[boneId];
        if (ai >= 0 && ai < (int)mNodes.size()) return &mNodes[ai];
    }
    return nullptr;
}

float Csab::animFrame(float frame) const {
    float last = (float)mDuration;
    while (frame > last) frame -= last; // REPEAT
    return frame;
}

// ---- sampling (matches tools/csab.py / noclip) ----
static float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static float lerpAngle(float v0, float v1, float t) {
    const float TAU = 6.283185307179586f;
    float da = fmodf(v1 - v0, TAU);
    float dist = fmodf(2 * da, TAU) - da;
    return v0 + dist * t;
}
static float pointCubic(const float cf[4], float t) {
    return ((cf[0] * t + cf[1]) * t + cf[2]) * t + cf[3];
}

float Csab::sampleTrack(const Track& t, float frame, bool rotation) {
    const auto& f = t.frames;
    if (f.empty()) return 0;
    int i1 = -1;
    for (int i = 0; i < (int)f.size(); i++) { if (frame < f[i].time) { i1 = i; break; } }
    if (t.type == LINEAR) {
        if (i1 == 0) return f[0].value;
        if (i1 < 0) return f.back().value;
        const Keyframe& k0 = f[i1 - 1]; const Keyframe& k1 = f[i1];
        float tt = (frame - k0.time) / (k1.time - k0.time);
        return rotation ? lerpAngle(k0.value, k1.value, tt) : lerpf(k0.value, k1.value, tt);
    }
    // HERMITE
    const Keyframe* k0; const Keyframe* k1;
    if (i1 <= 0) { k0 = &f.back(); k1 = &f.front(); }
    else { k0 = &f[i1 - 1]; k1 = &f[i1]; }
    float length = fmodf(k1->time - k0->time, (float)t.timeEnd);
    if (length == 0) return k0->value;
    float tt = (frame - k0->time) / length;
    float p0 = k0->value, p1 = k1->value;
    if (rotation) {
        // int16 rotation values are wrapped to [-pi,pi); the cubic must interpolate the
        // CONTINUOUS curve, so unwrap p1 to the branch nearest p0 (take the short way).
        // Without this, a keyframe pair straddling +-pi (e.g. +176deg -> -168deg, a +16deg
        // move) sweeps the long way (~-344deg) and the bone spins all the way around — seen
        // live as Link's head/back-shield/arms "spinning weird". Tangents are slopes
        // (angle/frame), invariant under adding 2*pi, so they stay valid.
        const float PI = 3.14159265358979f, TAU = 6.283185307179586f;
        p1 = p0 + (fmodf(p1 - p0 + PI + TAU, TAU) - PI);
    }
    float s0 = k0->tangentOut * length, s1 = k1->tangentIn * length;
    float cf[4] = { 2 * p0 - 2 * p1 + s0 + s1, -3 * p0 + 3 * p1 - 2 * s0 - s1, s0, p0 };
    return pointCubic(cf, tt);
}

void Csab::animatedBoneWorld(const Cmb& model, float frame, std::vector<std::array<float, 16>>& out) const {
    const auto& bones = model.bones();
    const auto& bind = model.boneMatrices();
    out.assign(bind.size(), matId());
    std::vector<char> done(bind.size(), 0);
    std::vector<const CmbBone*> byId(bind.size(), nullptr);
    for (const auto& bn : bones) if (bn.id >= 0 && (size_t)bn.id < byId.size()) byId[bn.id] = &bn;
    float fr = animFrame(frame);

    // resolve a bone's animated world matrix, recursing through parents.
    std::function<Mat4(int)> world = [&](int id) -> Mat4 {
        if (id < 0 || (size_t)id >= out.size() || !byId[id]) return matId();
        if (done[id]) return out[id];
        const CmbBone* bn = byId[id];
        float s[3] = { bn->scale[0], bn->scale[1], bn->scale[2] };
        float r[3] = { bn->rot[0], bn->rot[1], bn->rot[2] };
        float t[3] = { bn->trans[0], bn->trans[1], bn->trans[2] };
        const AnimNode* node = nodeForBone(id);
        if (node) {
            // track slots: 0..2 tX/Y/Z, 3..5 rX/Y/Z, 6..8 sX/Y/Z
            if (node->tracks[6].present) s[0] = sampleTrack(node->tracks[6], fr, false);
            if (node->tracks[7].present) s[1] = sampleTrack(node->tracks[7], fr, false);
            if (node->tracks[8].present) s[2] = sampleTrack(node->tracks[8], fr, false);
            if (node->tracks[3].present) r[0] = sampleTrack(node->tracks[3], fr, true);
            if (node->tracks[4].present) r[1] = sampleTrack(node->tracks[4], fr, true);
            if (node->tracks[5].present) r[2] = sampleTrack(node->tracks[5], fr, true);
            if (node->tracks[0].present) t[0] = sampleTrack(node->tracks[0], fr, false);
            if (node->tracks[1].present) t[1] = sampleTrack(node->tracks[1], fr, false);
            if (node->tracks[2].present) t[2] = sampleTrack(node->tracks[2], fr, false);
        }
        Mat4 L = matMul(matT(t[0], t[1], t[2]),
                        matMul(matMul(matRz(r[2]), matRy(r[1])), matRx(r[0])));
        L = matMul(L, matS(s[0], s[1], s[2]));
        Mat4 W = (bn->parent < 0) ? L : matMul(world(bn->parent), L);
        out[id] = W;
        done[id] = 1;
        return W;
    };
    for (const auto& bn : bones) world(bn.id);
}

void Csab::skinMatrices(const Cmb& model, float frame, std::vector<std::array<float, 16>>& out) const {
    std::vector<std::array<float, 16>> aw;
    animatedBoneWorld(model, frame, aw);
    const auto& bind = model.boneMatrices();
    out.assign(bind.size(), matId());
    for (size_t id = 0; id < bind.size(); id++)
        out[id] = matMul(aw[id], matInverse(bind[id]));
}

} // namespace SoH3D
