#include "cmb.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <functional>

namespace SoH3D {

static uint8_t  u8(const uint8_t* b, size_t o) { return b[o]; }
static int16_t  s16(const uint8_t* b, size_t o) { int16_t v; memcpy(&v, b + o, 2); return v; }
static uint16_t u16(const uint8_t* b, size_t o) { uint16_t v; memcpy(&v, b + o, 2); return v; }
static uint32_t u32(const uint8_t* b, size_t o) { uint32_t v; memcpy(&v, b + o, 4); return v; }
static float    f32(const uint8_t* b, size_t o) { float v; memcpy(&v, b + o, 4); return v; }

// GL data types
enum { DT_BYTE = 0x1400, DT_UBYTE, DT_SHORT, DT_USHORT, DT_INT, DT_UINT, DT_FLOAT };
static int dtSize(uint16_t t) {
    switch (t) {
        case DT_BYTE: case DT_UBYTE: return 1;
        case DT_SHORT: case DT_USHORT: return 2;
        default: return 4; // INT/UINT/FLOAT
    }
}
static float dtRead(const uint8_t* b, size_t o, uint16_t t) {
    switch (t) {
        case DT_BYTE: return (float)(int8_t)b[o];
        case DT_UBYTE: return (float)b[o];
        case DT_SHORT: return (float)s16(b, o);
        case DT_USHORT: return (float)u16(b, o);
        case DT_INT: { int32_t v; memcpy(&v, b + o, 4); return (float)v; }
        case DT_UINT: return (float)u32(b, o);
        case DT_FLOAT: return f32(b, o);
        default: return 0;
    }
}

// Attribute slots. OoT3D (v6) has no tangent; MM3D (v7) inserts it after normal.
struct AttrDef { const char* name; int comps; };
static const AttrDef ATTRS_OOT3D[] = {
    { "position", 3 }, { "normal", 3 }, { "color", 4 },
    { "texCoord0", 2 }, { "texCoord1", 2 }, { "texCoord2", 2 },
    { "boneIndices", 0 }, { "boneWeights", 0 }
};
static const AttrDef ATTRS_MM3D[] = {
    { "position", 3 }, { "normal", 3 }, { "tangent", 3 }, { "color", 4 },
    { "texCoord0", 2 }, { "texCoord1", 2 }, { "texCoord2", 2 },
    { "boneIndices", 0 }, { "boneWeights", 0 }
};

// ---- 4x4 row-major matrix helpers (out_i = sum_j M[i][j]*v_j, v_3=1) ----
typedef std::array<float, 16> Mat4;
static Mat4 matId() { Mat4 m{}; for (int i = 0; i < 4; i++) m[i * 4 + i] = 1; return m; }
static Mat4 matMul(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += A[i * 4 + k] * B[k * 4 + j];
            C[i * 4 + j] = s;
        }
    return C;
}
static Mat4 matT(float x, float y, float z) { Mat4 m = matId(); m[3] = x; m[7] = y; m[11] = z; return m; }
static Mat4 matS(float x, float y, float z) { Mat4 m = matId(); m[0] = x; m[5] = y; m[10] = z; return m; }
static Mat4 matRx(float a) { float c = cosf(a), s = sinf(a); Mat4 m = matId(); m[5] = c; m[6] = -s; m[9] = s; m[10] = c; return m; }
static Mat4 matRy(float a) { float c = cosf(a), s = sinf(a); Mat4 m = matId(); m[0] = c; m[2] = s; m[8] = -s; m[10] = c; return m; }
static Mat4 matRz(float a) { float c = cosf(a), s = sinf(a); Mat4 m = matId(); m[0] = c; m[1] = -s; m[4] = s; m[5] = c; return m; }
static void matApplyPos(const Mat4& M, const float* p, float* out) {
    for (int i = 0; i < 3; i++) out[i] = M[i * 4 + 0] * p[0] + M[i * 4 + 1] * p[1] + M[i * 4 + 2] * p[2] + M[i * 4 + 3];
}
static void matApplyDir(const Mat4& M, const float* v, float* out) {
    for (int i = 0; i < 3; i++) out[i] = M[i * 4 + 0] * v[0] + M[i * 4 + 1] * v[1] + M[i * 4 + 2] * v[2];
}

Cmb::Cmb(std::vector<uint8_t> data) : mData(std::move(data)) {
    const uint8_t* b = mData.data();
    if (mData.size() < 0x44 || memcmp(b, "cmb ", 4) != 0) { mErr = "not a CMB"; return; }
    mVersion = u32(b, 0x08);
    { size_t e = 0x10; while (e < 0x20 && b[e]) e++; mName.assign((const char*)b + 0x10, e - 0x10); }
    mIndexCount = u32(b, 0x20);
    mSklPtr = u32(b, 0x24);
    mMatsPtr = u32(b, 0x28);
    mTexPtr = u32(b, 0x2C);
    mSklmPtr = u32(b, 0x30);
    mVatrPtr = u32(b, 0x38);
    mIdxPtr = u32(b, 0x3C);
    mTexdataPtr = u32(b, 0x40);
    if (!parseSkl()) return;
    computeBoneMatrices();
    if (!parseMats()) return;
    if (!parseVatr()) return;
    if (!parseTex()) return;
    if (!parseSklm()) return;
    mOk = true;
}

static const AttrDef* Cmb_attrsDef(uint32_t version, int* count) {
    if (version >= 7) { *count = (int)(sizeof(ATTRS_MM3D) / sizeof(AttrDef)); return ATTRS_MM3D; }
    *count = (int)(sizeof(ATTRS_OOT3D) / sizeof(AttrDef));
    return ATTRS_OOT3D;
}

bool Cmb::parseSkl() {
    const uint8_t* b = mData.data();
    uint32_t p = mSklPtr;
    if (memcmp(b + p, "skl ", 4) != 0) { mErr = "bad skl"; return false; }
    uint32_t n = u32(b, p + 8);
    uint32_t o = p + 0x10;
    mBones.resize(n);
    for (uint32_t i = 0; i < n; i++) {
        CmbBone& bn = mBones[i];
        bn.id = u8(b, o);
        bn.parent = s16(b, o + 2);
        for (int k = 0; k < 3; k++) bn.scale[k] = f32(b, o + 4 + 4 * k);
        for (int k = 0; k < 3; k++) bn.rot[k] = f32(b, o + 0x10 + 4 * k);
        for (int k = 0; k < 3; k++) bn.trans[k] = f32(b, o + 0x1C + 4 * k);
        o += 0x28;
    }
    return true;
}

void Cmb::computeBoneMatrices() {
    int maxId = 0;
    for (const auto& bn : mBones) maxId = std::max(maxId, bn.id);
    mBoneMatrix.assign(maxId + 1, matId());
    std::vector<char> done(maxId + 1, 0);
    std::vector<const CmbBone*> byId(maxId + 1, nullptr);
    for (const auto& bn : mBones) byId[bn.id] = &bn;

    // iterative resolve (parents always have lower-or-defined id; recurse via stack)
    std::function<Mat4(int)> world = [&](int id) -> Mat4 {
        if (done[id]) return mBoneMatrix[id];
        const CmbBone* bn = byId[id];
        Mat4 L = matMul(matT(bn->trans[0], bn->trans[1], bn->trans[2]),
                        matMul(matMul(matRz(bn->rot[2]), matRy(bn->rot[1])), matRx(bn->rot[0])));
        L = matMul(L, matS(bn->scale[0], bn->scale[1], bn->scale[2]));
        Mat4 W = (bn->parent < 0) ? L : matMul(world(bn->parent), L);
        mBoneMatrix[id] = W;
        done[id] = 1;
        return W;
    };
    for (const auto& bn : mBones) world(bn.id);
}

bool Cmb::parseMats() {
    const uint8_t* b = mData.data();
    uint32_t p = mMatsPtr;
    if (p == 0 || memcmp(b + p, "mats", 4) != 0) return true; // optional
    uint32_t n = u32(b, p + 8);
    uint32_t stride = (mVersion <= 6) ? 0x15C : 0x16C;
    uint32_t o = p + 0x0C;
    mMaterials.resize(n);
    for (uint32_t i = 0; i < n; i++) {
        CmbMaterial& m = mMaterials[i];
        m.index = (int)i;
        m.cull = u8(b, o + 4);
        uint32_t bo = o + 0x10;
        m.tex0_idx = s16(b, bo);
        m.wrap_s = u16(b, bo + 8);
        m.wrap_t = u16(b, bo + 0x0A);
        uint32_t co = o + 0x58;
        m.scale_s = f32(b, co + 4);
        m.scale_t = f32(b, co + 8);
        m.trans_s = f32(b, co + 12);
        m.trans_t = f32(b, co + 16);
        m.rot = f32(b, co + 20);
        m.alpha_test = u8(b, o + 0x130) != 0;
        m.alpha_ref = u8(b, o + 0x131) / 255.0f;
        o += stride;
    }
    return true;
}

int Cmb::materialTexture(int matIndex) const {
    if (matIndex >= 0 && matIndex < (int)mMaterials.size()) {
        int t = mMaterials[matIndex].tex0_idx;
        return t >= 0 ? t : 0;
    }
    return 0;
}

bool Cmb::parseVatr() {
    const uint8_t* b = mData.data();
    uint32_t p = mVatrPtr;
    if (memcmp(b + p, "vatr", 4) != 0) { mErr = "bad vatr"; return false; }
    int count = 0;
    const AttrDef* defs = Cmb_attrsDef(mVersion, &count);
    (void)defs;
    mVatr.resize(count);
    uint32_t o = p + 0x0C;
    for (int i = 0; i < count; i++) {
        uint32_t size = u32(b, o);
        uint32_t off = u32(b, o + 4); // size first, then offset
        mVatr[i].off = mVatrPtr + off;
        mVatr[i].size = size;
        o += 8;
    }
    return true;
}

bool Cmb::parseTex() {
    if (mTexPtr == 0) return true;
    const uint8_t* b = mData.data();
    uint32_t p = mTexPtr;
    if (memcmp(b + p, "tex ", 4) != 0) return true;
    uint32_t n = u32(b, p + 8);
    uint32_t o = p + 0x0C;
    mTextures.resize(n);
    for (uint32_t i = 0; i < n; i++) {
        CmbTexture& t = mTextures[i];
        t.data_len = u32(b, o);
        t.etc1 = u8(b, o + 6) != 0;
        t.width = u16(b, o + 8);
        t.height = u16(b, o + 0x0A);
        t.fmt = u16(b, o + 0x0C);
        t.data_type = u16(b, o + 0x0E);
        t.data_offset = u32(b, o + 0x10);
        size_t e = o + 0x14;
        while (e < o + 0x24 && b[e]) e++;
        t.name.assign((const char*)b + o + 0x14, e - (o + 0x14));
        o += 0x24;
    }
    return true;
}

std::vector<uint8_t> Cmb::textureRaw(const CmbTexture& t) const {
    size_t base = mTexdataPtr + t.data_offset;
    if (base + t.data_len > mData.size()) return {};
    return std::vector<uint8_t>(mData.begin() + base, mData.begin() + base + t.data_len);
}

bool Cmb::parseSklm() {
    const uint8_t* b = mData.data();
    uint32_t p = mSklmPtr;
    if (memcmp(b + p, "sklm", 4) != 0) { mErr = "bad sklm"; return false; }
    uint32_t mshsOff = p + u32(b, p + 0x08);
    uint32_t shpOff = p + u32(b, p + 0x0C);
    if (memcmp(b + mshsOff, "mshs", 4) != 0) { mErr = "bad mshs"; return false; }
    uint32_t meshCount = u32(b, mshsOff + 8);
    uint32_t mo = mshsOff + 0x10;
    mMeshes.resize(meshCount);
    for (uint32_t i = 0; i < meshCount; i++) {
        mMeshes[i].sepd_index = u16(b, mo);
        mMeshes[i].material_index = u8(b, mo + 2);
        mMeshes[i].mesh_id = u8(b, mo + 3);
        mo += 4;
    }
    if (memcmp(b + shpOff, "shp ", 4) != 0) { mErr = "bad shp"; return false; }
    uint32_t sepdCount = u32(b, shpOff + 8);
    mSepds.resize(sepdCount);
    for (uint32_t i = 0; i < sepdCount; i++) {
        uint16_t ptr = u16(b, shpOff + 0x10 + 2 * i);
        mSepds[i] = parseSepd(shpOff + ptr);
    }
    return true;
}

Cmb::Sepd Cmb::parseSepd(uint32_t p) {
    const uint8_t* b = mData.data();
    Sepd s;
    if (memcmp(b + p, "sepd", 4) != 0) { mErr = "bad sepd"; return s; }
    s.prim_count = u16(b, p + 8);
    int count = 0;
    const AttrDef* defs = Cmb_attrsDef(mVersion, &count);
    (void)defs;
    s.attrs.resize(count);
    uint32_t o = p + 0x24;
    for (int i = 0; i < count; i++) {
        SepdAttr& a = s.attrs[i];
        a.start = u32(b, o);
        a.scale = f32(b, o + 4);
        a.data_type = u16(b, o + 8);
        a.mode = u16(b, o + 0x0A);
        for (int k = 0; k < 4; k++) a.constant[k] = f32(b, o + 0x0C + 4 * k);
        a.present = true;
        o += 0x1C;
    }
    s.bone_dimension = u16(b, o);
    o += 4;
    s.prms.resize(s.prim_count);
    for (uint16_t i = 0; i < s.prim_count; i++) {
        uint16_t ptr = u16(b, o + 2 * i);
        s.prms[i] = parsePrms(p + ptr);
    }
    return s;
}

Cmb::Prms Cmb::parsePrms(uint32_t p) {
    const uint8_t* b = mData.data();
    Prms r;
    if (memcmp(b + p, "prms", 4) != 0) { mErr = "bad prms"; return r; }
    r.skinning_mode = u16(b, p + 0x0C);
    uint16_t btCount = u16(b, p + 0x0E);
    uint32_t btOff = p + u32(b, p + 0x10);
    uint32_t prmOff = p + u32(b, p + 0x14);
    r.bone_table.resize(btCount);
    for (uint16_t i = 0; i < btCount; i++) r.bone_table[i] = u16(b, btOff + 2 * i);
    r.prm = parsePrm(prmOff);
    return r;
}

Cmb::Prm Cmb::parsePrm(uint32_t p) {
    const uint8_t* b = mData.data();
    Prm r;
    if (memcmp(b + p, "prm ", 4) != 0) { mErr = "bad prm"; return r; }
    r.index_type = u16(b, p + 0x10);
    r.count = u16(b, p + 0x14);
    r.first = u16(b, p + 0x16);
    return r;
}

void Cmb::readAttr(const SepdAttr& attr, int attrSlot, uint32_t idx, int comps, float* out) const {
    if (attr.mode == 1) { // constant
        for (int i = 0; i < comps; i++) out[i] = attr.constant[i];
        return;
    }
    const uint8_t* b = mData.data();
    uint32_t base = mVatr[attrSlot].off;
    int sz = dtSize(attr.data_type);
    uint32_t off = base + attr.start + idx * comps * sz;
    for (int i = 0; i < comps; i++) out[i] = dtRead(b, off + i * sz, attr.data_type) * attr.scale;
}

std::vector<CmbDrawGroup> Cmb::buildDrawGroups() const {
    const uint8_t* b = mData.data();
    int count = 0;
    const AttrDef* defs = Cmb_attrsDef(mVersion, &count);
    // locate attribute slots by name
    int slotPos = -1, slotNrm = -1, slotUv0 = -1, slotBi = -1, slotBw = -1;
    for (int i = 0; i < count; i++) {
        if (!strcmp(defs[i].name, "position")) slotPos = i;
        else if (!strcmp(defs[i].name, "normal")) slotNrm = i;
        else if (!strcmp(defs[i].name, "texCoord0")) slotUv0 = i;
        else if (!strcmp(defs[i].name, "boneIndices")) slotBi = i;
        else if (!strcmp(defs[i].name, "boneWeights")) slotBw = i;
    }

    // accumulate per material index
    std::vector<CmbDrawGroup> groups;
    auto groupFor = [&](int mat) -> CmbDrawGroup& {
        for (auto& g : groups) if (g.material_index == mat) return g;
        groups.push_back({ mat, {} });
        return groups.back();
    };

    for (const auto& mesh : mMeshes) {
        if (mesh.sepd_index >= mSepds.size()) continue;
        const Sepd& sepd = mSepds[mesh.sepd_index];
        int bd = sepd.bone_dimension;
        bool hasNormal = slotNrm >= 0 && sepd.attrs[slotNrm].present;
        CmbDrawGroup& g = groupFor(mesh.material_index);
        for (const auto& prms : sepd.prms) {
            const Prm& prm = prms.prm;
            int boneId = prms.bone_table.empty() ? 0 : prms.bone_table[0];
            bool smooth = bd > 1 && slotBi >= 0 && slotBw >= 0 &&
                          sepd.attrs[slotBi].mode == 0 && sepd.attrs[slotBw].mode == 0;
            Mat4 M = smooth ? matId()
                            : (boneId < (int)mBoneMatrix.size() ? mBoneMatrix[boneId] : matId());
            int isz = dtSize(prm.index_type);
            size_t ibase = mIdxPtr + (size_t)prm.first * isz;
            std::vector<CmbVertex> verts;
            verts.reserve(prm.count);
            for (uint16_t k = 0; k < prm.count; k++) {
                uint32_t idx = (uint32_t)dtRead(b, ibase + (size_t)k * isz, prm.index_type);
                float pos[3], nrm[3] = { 0, 0, 1 }, uv[2] = { 0, 0 };
                readAttr(sepd.attrs[slotPos], slotPos, idx, 3, pos);
                if (hasNormal) readAttr(sepd.attrs[slotNrm], slotNrm, idx, 3, nrm);
                if (slotUv0 >= 0 && sepd.attrs[slotUv0].present) readAttr(sepd.attrs[slotUv0], slotUv0, idx, 2, uv);
                CmbVertex v;
                matApplyPos(M, pos, v.pos);
                matApplyDir(M, nrm, v.nrm);
                v.uv[0] = uv[0];
                v.uv[1] = uv[1];
                verts.push_back(v);
            }
            // triangle list (matches cmb.py: groups of 3, drop trailing remainder)
            size_t n = verts.size();
            for (size_t i = 0; i + 2 < n; i += 3) {
                g.verts.push_back(verts[i]);
                g.verts.push_back(verts[i + 1]);
                g.verts.push_back(verts[i + 2]);
            }
        }
    }
    return groups;
}

} // namespace SoH3D
