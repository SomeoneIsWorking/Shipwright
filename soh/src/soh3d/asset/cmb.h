// Parser for OoT3D CMB models (geometry + skeleton + material/texture refs).
// Port of tools/cmb.py. Pure C++ (no SoH/LUS deps). Produces, per material, an
// assembled triangle-soup of interleaved vertices ready for a GL VBO, plus the
// material/texture metadata the renderer needs. Bind-pose skinning matches cmb.py
// (rigid bone_dim==1 -> bound-bone world matrix; smooth bone_dim>1 -> raw model
// space). Animation (live bone matrices) is a later layer that reuses the skeleton.
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <array>

namespace SoH3D {

struct CmbBone {
    int id = 0;
    int parent = -1;
    float scale[3] = { 1, 1, 1 };
    float rot[3] = { 0, 0, 0 };
    float trans[3] = { 0, 0, 0 };
};

struct CmbMaterial {
    int index = 0;
    int tex0_idx = -1;
    uint16_t wrap_s = 0x2901, wrap_t = 0x2901; // GL enums
    float scale_s = 1, scale_t = 1, trans_s = 0, trans_t = 0, rot = 0;
    int cull = 0;
    bool alpha_test = false;
    float alpha_ref = 0;
    // Blend state. The CMB stores GL-ES enum values directly (e.g. 0x0302 GL_SRC_ALPHA,
    // 0x0001 GL_ONE, 0x8006 GL_FUNC_ADD), identical to desktop GL — used verbatim. When
    // blend_enable is false the material is opaque (alpha-test only). Additive light-shaft
    // materials have dst_rgb = GL_ONE; without honoring this they render opaque.
    bool blend_enable = false;
    uint16_t blend_src_rgb = 0x0302, blend_dst_rgb = 0x0303; // GL_SRC_ALPHA / GL_ONE_MINUS_SRC_ALPHA
    uint16_t blend_src_a = 0x0001, blend_dst_a = 0x0000;     // GL_ONE / GL_ZERO
    uint16_t blend_eq_rgb = 0x8006, blend_eq_a = 0x8006;     // GL_FUNC_ADD
    float blend_color[4] = { 0, 0, 0, 1 };                   // for CONSTANT_COLOR/ALPHA factors
    bool depth_write = true;                                 // translucent volumes usually disable this
};

struct CmbTexture {
    std::string name;
    int width = 0, height = 0;
    uint16_t fmt = 0, data_type = 0;
    bool etc1 = false;
    uint32_t data_offset = 0, data_len = 0;
    uint32_t glFormat() const { return ((uint32_t)data_type << 16) | fmt; }
};

// Interleaved render vertex: position (model space), normal, uv0, and skinning
// bindings (up to 4 bone ids + weights). MUST stay byte-compatible with
// SoH3DGlVtx (soh3d_gl.h) — the bridge reinterpret_casts between them.
struct CmbVertex {
    float pos[3];
    float nrm[3];
    float uv[2];
    float boneIds[4] = { 0, 0, 0, 0 };
    float weights[4] = { 0, 0, 0, 0 };
};

// One draw batch: all triangles that use a given material, as a triangle list.
struct CmbDrawGroup {
    int material_index = 0;
    std::vector<CmbVertex> verts; // multiple of 3
};

class Cmb {
  public:
    explicit Cmb(std::vector<uint8_t> data);
    bool ok() const { return mOk; }
    const std::string& error() const { return mErr; }

    const std::string& name() const { return mName; }
    uint32_t version() const { return mVersion; }
    const std::vector<CmbBone>& bones() const { return mBones; }
    const std::vector<CmbMaterial>& materials() const { return mMaterials; }
    const std::vector<CmbTexture>& textures() const { return mTextures; }
    // Bind-pose world matrix per bone id (row-major flat 16-float). Used by CSAB
    // skinning to form skinMatrix = animWorld . inverse(bindWorld).
    const std::vector<std::array<float, 16>>& boneMatrices() const { return mBoneMatrix; }

    // Texture index used by a material's primary binding (0 if unknown/none).
    int materialTexture(int matIndex) const;
    // Raw (still-encoded) bytes of a texture, sliced from the CMB texdata block.
    std::vector<uint8_t> textureRaw(const CmbTexture& t) const;

    // Assemble all meshes into per-material draw groups (bind pose).
    std::vector<CmbDrawGroup> buildDrawGroups() const;

    // Same, but with CSAB skinning applied: skinMats is indexed by bone id and is
    // skinMatrix = animWorld . bindInverse for each bone (see asset/csab). Each
    // vertex is taken to MODEL space exactly as buildDrawGroups() does (rigid:
    // .bindWorld; smooth: raw), then transformed by the weighted blend of its bones'
    // skinMats. skinMats == nullptr (n==0) is identity -> byte-identical to
    // buildDrawGroups() (the bind pose). Mirrors tools/csab.py skinned_triangles.
    std::vector<CmbDrawGroup> buildDrawGroupsSkinned(const std::array<float, 16>* skinMats, size_t n) const;

  private:
    bool mOk = false;
    std::string mErr;
    std::vector<uint8_t> mData;

    uint32_t mVersion = 0;
    std::string mName;
    uint32_t mIndexCount = 0;
    uint32_t mSklPtr = 0, mMatsPtr = 0, mTexPtr = 0, mSklmPtr = 0, mVatrPtr = 0, mIdxPtr = 0, mTexdataPtr = 0;

    std::vector<CmbBone> mBones;
    // bind-pose world matrix per bone id (4x4 row-major), as a flat 16-float array.
    std::vector<std::array<float, 16>> mBoneMatrix; // indexed by bone id
    std::vector<CmbMaterial> mMaterials;
    std::vector<CmbTexture> mTextures;

    // VATR: attribute name index -> (abs offset, size)
    struct VatrBuf { uint32_t off = 0, size = 0; };
    std::vector<VatrBuf> mVatr; // one per attribute slot in attrs def

    struct SepdAttr {
        uint32_t start = 0;
        float scale = 1;
        uint16_t data_type = 0;
        uint16_t mode = 0;       // 0 array, 1 constant
        float constant[4] = { 0, 0, 0, 0 };
        bool present = false;
    };
    struct Prm { uint16_t index_type = 0; uint16_t count = 0; uint16_t first = 0; };
    struct Prms { uint16_t skinning_mode = 0; std::vector<uint16_t> bone_table; Prm prm; };
    struct Sepd {
        std::vector<SepdAttr> attrs; // indexed by attribute slot
        uint16_t prim_count = 0;
        uint16_t bone_dimension = 0;
        std::vector<Prms> prms;
    };
    struct Mesh { uint16_t sepd_index = 0; uint8_t material_index = 0; uint8_t mesh_id = 0; };

    std::vector<Sepd> mSepds;
    std::vector<Mesh> mMeshes;

    bool parseSkl();
    void computeBoneMatrices();
    bool parseMats();
    bool parseVatr();
    bool parseTex();
    bool parseSklm();
    Sepd parseSepd(uint32_t p);
    Prms parsePrms(uint32_t p);
    Prm parsePrm(uint32_t p);

    // Read a single attribute value (comps components) at element idx for a sepd attr.
    void readAttr(const SepdAttr& attr, int attrSlot, uint32_t idx, int comps, float* out) const;
};

} // namespace SoH3D
