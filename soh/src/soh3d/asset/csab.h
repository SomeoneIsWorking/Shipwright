// Parser for OoT3D CSAB skeletal animations (CTR Skeletal Animation Binary).
// Port of tools/csab.py (itself a port of noclip OcarinaOfTime3D/csab.ts, Ocarina
// subversion-3 path). Pure C++ (no SoH/LUS deps). Produces per-bone skinning
// matrices for a frame, which Cmb::buildDrawGroupsSkinned() applies. The matrix
// convention (T.Rz.Ry.Rx.S) is identical to Cmb's bind-pose computeBoneMatrices,
// so with anim==rest the skin matrices are identity and the bind-pose render is
// unchanged (the property the renderer relies on). Verified against ge1_s_wait.
#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <string>

namespace SoH3D {

class Cmb;

class Csab {
  public:
    explicit Csab(std::vector<uint8_t> data);
    bool ok() const { return mOk; }
    const std::string& error() const { return mErr; }

    int duration() const { return mDuration; }     // frame count (raw+1)
    int boneCount() const { return mBoneCount; }
    int animNodeCount() const { return (int)mNodes.size(); }

    // Per-bone-id skinning matrix at `frame` = animWorld(bone) . inverse(bindWorld).
    // out is sized to match model.boneMatrices(); identity for any bone without anim.
    void skinMatrices(const Cmb& model, float frame, std::vector<std::array<float, 16>>& out) const;

    // Per-bone-id animated world matrix at `frame` (rest TRS overridden by tracks).
    void animatedBoneWorld(const Cmb& model, float frame, std::vector<std::array<float, 16>>& out) const;

  private:
    enum { CONSTANT = 0, LINEAR = 1, HERMITE = 2 };
    struct Keyframe { float time, value, tangentIn, tangentOut; };
    struct Track {
        int type = CONSTANT;
        int timeStart = 0, timeEnd = 1;
        bool present = false;
        std::vector<Keyframe> frames;
    };
    struct AnimNode {
        int boneIndex = 0;
        bool isRotInt16 = false;
        Track tracks[9]; // tX tY tZ rX rY rZ sX sY sZ
    };

    bool mOk = false;
    std::string mErr;
    std::vector<uint8_t> mData;

    int mDuration = 1;
    int mBoneCount = 0;
    std::vector<int16_t> mBoneToAnim;
    std::vector<AnimNode> mNodes;

    Track parseTrack(uint32_t o, bool isRotInt16) const;
    AnimNode parseAnod(uint32_t o) const;

    const AnimNode* nodeForBone(int boneId) const;
    static float sampleTrack(const Track& t, float frame, bool rotation);
    float animFrame(float frame) const;
};

} // namespace SoH3D
