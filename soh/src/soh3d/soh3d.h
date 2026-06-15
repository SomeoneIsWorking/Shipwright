// SoH3D — render OoT3D (3DS) models in place of N64 assets.
// See repo-root PROGRESS.md for the overall design.
#ifndef SOH3D_H
#define SOH3D_H

#include "global.h"
#include "soh3d_pot_model.h"

// Returns true when OoT3D-model rendering is enabled (env SOH3D=1). Cached.
int SoH3D_Enabled(void);

// Headless verification: when env SOH3D_WARP is set, boot straight into the
// debug Select overlay and auto-warp into a scene so pots are reachable without
// scripting title/file-select input. Entrance defaults to Kakariko Village
// (lots of pots); override with env SOH3D_ENTRANCE (decimal entrance index).
int SoH3D_AutoWarpEnabled(void);
int SoH3D_AutoWarpEntrance(void);

// Debug-draws the OoT3D pot model at Link's position (env SOH3D_DEBUGPOT=1),
// so the CMB->LUS render path can be verified in any scene. Draw-time uniform
// scale is tunable with env SOH3D_SCALE (default 1.0). No-op otherwise.
void SoH3D_DebugDrawPot(PlayState* play);

#endif
