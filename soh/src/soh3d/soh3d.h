// SoH3D — render OoT3D (3DS) models in place of N64 assets.
// See repo-root PROGRESS.md for the overall design.
#ifndef SOH3D_H
#define SOH3D_H

#include "global.h"
#include "soh3d_pot_model.h"

// Returns true when OoT3D-model rendering is enabled (env SOH3D=1). Cached.
int SoH3D_Enabled(void);

// Draws an OoT3D model display list at an actor's world position/yaw with an
// explicit world scale (OoT3D model units -> N64 world units). Builds its own
// MTXMODE_NEW matrix rather than inheriting the actor's N64-tuned 0.01 scale, so
// SoH3D controls the model's true world size. Emits into POLY_OPA.
void SoH3D_DrawModel(PlayState* play, Gfx* dlist, Actor* actor, float worldScale);

// World scale for the OoT3D pot (OoT3D model units -> N64 world units). Tuned by
// matching the rendered height of the OoT3D pot to the N64 pot at the same spot
// (spawn comparison, Deku Tree). The OoT3D model is ~162 units tall; ~0.12 lands
// it at the N64 pot's height. See PROGRESS.md calibration.
#define SOH3D_POT_WORLD_SCALE 0.12f

// Headless verification: when env SOH3D_WARP is set, boot straight into the
// debug Select overlay and auto-warp into a scene so pots are reachable without
// scripting title/file-select input. Entrance defaults to Kakariko Village
// (lots of pots); override with env SOH3D_ENTRANCE (decimal entrance index).
int SoH3D_AutoWarpEnabled(void);
int SoH3D_AutoWarpEntrance(void);

// Verification helper, called each frame from Play_Draw. When env
// SOH3D_SPAWNPOT=1, spawns one real Obj_Tsubo beside Link so the actual
// ObjTsubo_Draw path can be A/B'd (SOH3D=0 N64 pot vs SOH3D=1 OoT3D pot) in the
// same scene. No-op otherwise.
void SoH3D_DebugDrawPot(PlayState* play);

#endif
