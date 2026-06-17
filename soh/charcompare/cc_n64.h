// charcompare — N64 (OoT) viewport.
//
// Renders an N64 OoT character through libultraship's Fast3D, the real engine path:
// load the skeleton + animation + limb display-list resources from oot.o2r, pose the
// skeleton (a reimplementation of the SkelAnime_DrawFlex walk), and emit a Fast3D
// display list of per-limb matrices + gSPDisplayList(limb DL). Side-by-side with the
// 3DS model (cc_3ds) for the N64<->3DS anim-map comparison.
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <fast/types.h>                // MtxF, Mtx
#include <libultraship/libultra/gbi.h> // Gfx, Vp

#include "cc_3ds.h" // cc::DlistKeys (shared dlist-key storage)

namespace cc {

// One loaded N64 character: the resolved skeleton header, the current animation, and
// framing info. The skeleton/anim are kept alive (their resolved pointers / OTR-path
// strings must outlive every interp->Run).
struct ModelN64 {
    bool ok = false;
    std::string objectPath;  // e.g. "objects/object_geldb"
    std::string skelName;    // e.g. "gGerudoRedSkel"
    int limbCount = 0;
    void* skelHeader = nullptr; // FlexSkeletonHeader* (resolved limb pointers)
    void* anim = nullptr;       // AnimationHeader*
    std::string animName;
    int animFrameCount = 0;
    std::vector<std::string> anims; // animation base names found for this object
    std::string error;
    // Auto-fit framing derived from the posed skeleton bbox (FK over joint positions).
    float fitScale = 1.0f;
    float center[3] = { 0, 0, 0 };
    // keep the resource shared_ptrs alive
    std::shared_ptr<void> skelRes, animRes;
    std::vector<std::shared_ptr<void>> limbRes;
};

// Register the resource factories the N64 path needs (Fast DisplayList/Vertex/Texture/
// Matrix + SOH Skeleton/SkeletonLimb/Animation/PlayerAnimation). Call once after the
// Context's window/resource-manager are initialized. Safe to call repeatedly.
void RegisterN64Factories();

// Load an N64 character: object resource folder + skeleton symbol name + the list of
// animation symbol names (from tools/skeldata/n64_anims.json). Loads + resolves the
// skeleton and enumerates anims; the first anim is selected.
ModelN64 LoadN64(const std::string& objectPath, const std::string& skelName,
                 const std::vector<std::string>& animNames);

// Select the current animation (by symbol name) and recompute frame count + framing.
void SetAnimN64(ModelN64& m, const std::string& animName);

// Append the posed skeleton's draw commands to `dl` for animation frame `frame`, into the
// given viewport (rx/ry/rz degrees orient the model like cc_3ds). keys holds the per-limb
// MtxF storage (must outlive interp->Run).
void EmitDlistN64(ModelN64& m, float frame, std::vector<Gfx>& dl, std::unordered_map<Mtx*, MtxF>& mtx,
                  DlistKeys& keys, float rx, float ry, float rz, const Rect& vp);

} // namespace cc
