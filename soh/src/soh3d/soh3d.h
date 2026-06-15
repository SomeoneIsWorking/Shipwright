// SoH3D — render OoT3D (3DS) models in place of N64 assets.
// See repo-root PROGRESS.md for the overall design.
#ifndef SOH3D_H
#define SOH3D_H

#include "global.h"
#include "soh3d_pot_model.h"
#include "soh3d_gs_model.h"

// Returns true when OoT3D-model rendering is enabled (env SOH3D=1). Cached.
int SoH3D_Enabled(void);

// Generalised per-actor divert, called from Actor_Draw for every actor. If SoH3D
// is enabled and the actor's id has an OoT3D model registered in the table, draws
// that model (via SoH3D_DrawModel) and returns 1 so the caller skips the actor's
// N64 draw; returns 0 otherwise (caller draws the N64 model as normal). Replaces
// the old per-actor `if (SoH3D_Enabled())` edits in each actor's Draw.
int SoH3D_TryDrawActor(PlayState* play, Actor* actor);

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

// World scale for the OoT3D Gossip Stone (OoT3D model units -> N64 world units).
// The model is ~485 units tall; calibrated against the N64 Gossip Stone via the
// SOH3D_SPAWNGS A/B spawn. Second object proving the MULTI-MATERIAL pipeline:
// 2 materials, 2 distinct fully-opaque textures (128x128 Sheikah-eye face +
// 128x64 stone body), each drawn with its own texture in one display list.
#define SOH3D_GS_WORLD_SCALE 0.13f

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

// Verification helper, called each frame from Play_Draw. When env
// SOH3D_SPAWNGS=1, spawns one real En_Gs (Gossip Stone) in front of Link so the
// actual EnGs_Draw path runs. SOH3D=0 draws the N64 Gossip Stone, SOH3D=1 the
// OoT3D multi-material one — a true same-scene comparison. Needs OBJECT_GS
// loaded (a scene with Gossip Stones, e.g. the default Kakariko warp). No-op
// otherwise.
void SoH3D_DebugDrawGs(PlayState* play);

#endif
