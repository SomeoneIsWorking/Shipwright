// SoH3D — render OoT3D (3DS) models in place of N64 assets.
// See repo-root PROGRESS.md for the overall design.
#ifndef SOH3D_H
#define SOH3D_H

#include "global.h"
#include "soh3d_pot_model.h"
#include "soh3d_gs_model.h"
#include "soh3d_kibako_model.h"
#include "soh3d_geldwoman_model.h"

// Returns true when OoT3D-model rendering is enabled (env SOH3D=1). Cached.
int SoH3D_Enabled(void);

// Generalised per-actor divert, called from Actor_Draw for every actor. If SoH3D
// is enabled and the actor's id has an OoT3D model registered in the table, draws
// that model (via SoH3D_DrawModel) and returns 1 so the caller skips the actor's
// N64 draw; returns 0 otherwise (caller draws the N64 model as normal). Replaces
// the old per-actor `if (SoH3D_Enabled())` edits in each actor's Draw.
int SoH3D_TryDrawActor(PlayState* play, Actor* actor);

// Called from Actor_Draw immediately AFTER an actor's N64 draw (only when
// SoH3D_TryDrawActor returned 0, i.e. the N64 model drew). Closes the auto-scale
// measure bracket opened by SoH3D_TryDrawActor so the SOH3D_AUTO path can measure the
// actor's drawn world size on this frame and derive its OoT3D model scale. No-op unless
// a measurement was opened for this actor.
void SoH3D_AfterActorDraw(PlayState* play, Actor* actor);

// N64-animation port hook, called at the top of the common SkelAnime draw choke points
// (SkelAnime_DrawSkeletonOpa / SkelAnime_DrawSkeleton2). When the actor currently being
// drawn is registered for N64-anim replacement (sModelTable n64anim flag, enabled via
// SOH3D_N64ANIM) this retargets the OoT3D model's skeleton from the live N64 jointTable and
// draws it, returning 1 so the caller SKIPS the N64 limb draw. Returns 0 otherwise (the N64
// skeleton draws as normal — also the fallback for actors whose draw path isn't hooked).
// This is what makes N64-anim replacement generic: no per-actor jointTable accessor needed.
int SoH3D_SkelAnimeDraw(PlayState* play, SkelAnime* skelAnime);

// Draws an OoT3D model display list at an actor's world position/yaw with an
// explicit world scale (OoT3D model units -> N64 world units). Builds its own
// MTXMODE_NEW matrix rather than inheriting the actor's N64-tuned 0.01 scale, so
// SoH3D controls the model's true world size. Emits into POLY_OPA.
void SoH3D_DrawModel(PlayState* play, Gfx* dlist, Actor* actor, float worldScale);

// Generalised per-room scene divert, called from Room_Draw. If SoH3D is enabled and
// the current scene has an OoT3D mapping (kSoH3dSceneNames) with a room CMB for
// room->num, draws that room geometry at the world origin (identity model matrix +
// the game camera, depth-correct via the scene pass) and returns 1 so the caller
// skips the N64 room mesh; returns 0 otherwise (caller draws the N64 room as normal).
int SoH3D_TryDrawRoom(PlayState* play, Room* room);

// Floor-height callback: returns the N64 collision floor Y at world (x,z), or a value
// <= -31000 if there is no floor. Provided by soh3d.c (it has the PlayState/colCtx).
typedef float (*SoH3D_FloorFn)(float x, float z);

// Compute & cache an OoT3D scene-room's ground-delta field D(x,z)=N64_floor-OoT3D_floor (the
// render mesh is left untouched; actors are offset by -D to stand on the visible OoT3D
// ground). Idempotent per model. Call from the room-draw hook before the room is drawn.
void SoH3D_ComputeRoomGroundDelta(int modelId, SoH3D_FloorFn floorFn);

// Sample that field: *outD = N64_floor - OoT3D_floor at world (x,z). Returns 1 on success.
int SoH3D_RoomGroundDeltaAt(int modelId, float x, float z, float* outD);

// Render-Y offset for an actor so it stands on the visible OoT3D ground instead of floating
// at the N64 collision height (= OoT3D_ground - N64_ground at the actor's XZ, i.e. -D).
// Returns 0 when SoH3D is off, the scene has no OoT3D room, or no delta covers the actor.
// Called from Actor_Draw: add to actor->world.pos.y around the draw, then subtract (physics
// stays N64). The inverse of the old render warp (which distorted the mesh).
float SoH3D_ActorRenderYOffset(PlayState* play, Actor* actor);

// Query the (warped) OoT3D room render-mesh floor Y at world (x,z). Returns 1 + *outY
// on a floor hit, else 0. For verifying the warp aligned the drawn ground to N64.
int SoH3D_RoomMeshFloorAt(int modelId, float x, float z, float* outY);

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

// World scale for the OoT3D large wooden crate (Obj_Kibako2). Model is ~600 units
// wide; calibrated against the N64 large crate via the SOH3D_SPAWNKIBAKO A/B spawn
// in Gerudo Valley (ENTR 0x117=279). Third object proving the table-driven divert:
// adding it was one sModelTable[] row + this macro + the generated include.
#define SOH3D_KIBAKO_WORLD_SCALE 0.10f

// World scale for the OoT3D Gerudo (En_Ge1). FIRST CHARACTER divert: the OoT3D
// model is smooth-skinned and baked UPRIGHT + grounded (cmb_to_c --rotx 180
// --ground), so it drops into the same Translate*RotateY*Scale path as the props
// with no orientation special-casing. The model is ~6358 units tall; 0.011 lands
// it at the N64 Gerudo's height — CONFIRMED in-game A/B (Gerudo Fortress): the
// OoT3D figure is 186 px tall vs the N64 En_Ge1's 187 px head->shadow. Pair with
// SOH3D_GELDWOMAN_GROUND_OFFSET (vertical grounding). Re-tune via `scale geldwoman`.
#define SOH3D_GELDWOMAN_WORLD_SCALE 0.011f

// Vertical grounding offset for En_Ge1, in MODEL units, applied BEFORE the world
// scale (so it scales together with SOH3D_GELDWOMAN_WORLD_SCALE — re-tuning scale
// never desyncs grounding). The skinned ge1_s_wait model sits with its feet ~1000
// model units (≈11 world units at scale 0.011) above the actor origin; this drops
// the feet onto the actor's ground pos. CALIBRATED in-game (REPL `yoff geldwoman`):
// -600 floats, -1000 grounds the soles on the actor's shadow, -1400 sinks to ankles.
// Height itself is correct: at scale 0.011 the model is 186 px tall vs the N64
// En_Ge1's 187 px in the same Gerudo-Fortress shot (head->shadow), so scale is kept.
#define SOH3D_GELDWOMAN_GROUND_OFFSET -1000.0f

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

// Verification helper, called each frame from Play_Draw. When env
// SOH3D_SPAWNKIBAKO=1, spawns one real Obj_Kibako2 (large wooden crate) in front
// of Link so the actual crate draw path runs (SOH3D=0 N64 crate vs SOH3D=1 OoT3D
// crate). Needs OBJECT_KIBAKO2 loaded (a scene with large crates, e.g. Gerudo
// Valley ENTR 0x117). Logs whether the spawn succeeded. No-op otherwise.
void SoH3D_DebugDrawKibako(PlayState* play);

// Interactive REPL poll, called once per frame from Play_Draw. When env
// SOH3D_REPL=<fifo path> is set, reads control commands from that FIFO and applies
// them live (tint, world scale, model spawn, on-demand frame dump) so a single
// long-lived headless instance can be poked without a rebuild/restart. No-op when
// SOH3D_REPL is unset. Drive it with tools/soh3d_repl.py.
void SoH3D_ReplPoll(PlayState* play);

#endif
