// SoH3D — render OoT3D (3DS) models in place of N64 assets.
// See repo-root PROGRESS.md for the overall design.
#ifndef SOH3D_H
#define SOH3D_H

#include "global.h"

// Returns true when OoT3D-model rendering is enabled (env SOH3D=1). Cached.
int SoH3D_Enabled(void);

// Generalised per-actor divert, called from Actor_Draw for every actor. If SoH3D
// is enabled and the actor's id has an OoT3D model registered in the table, draws
// that model (via the direct-GL path) and returns 1 so the caller skips the actor's
// N64 draw; returns 0 otherwise (caller draws the N64 model as normal). Replaces
// the old per-actor `if (SoH3D_Enabled())` edits in each actor's Draw.
int SoH3D_TryDrawActor(PlayState* play, Actor* actor);

// Pure predicate (no drawing): does this actor have an OoT3D replacement right now? Used by the
// engine draw-distance check so replaced actors keep drawing/updating past the N64 cull distance.
int SoH3D_ActorHasReplacement(PlayState* play, Actor* actor);

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

// Record the limb-draw override callback (+arg) the actor passed to its SkelAnime_Draw* call, so
// the N64-anim auto-replace path can replay a PROCEDURAL per-limb rotation the override adds (e.g.
// the cucco wing-flap, which lives in EnNiw_OverrideLimbDraw, not in any animation) onto the
// matching OoT3D bones. `kind`: 0 = OverrideLimbDrawOpa (6 args), 1 = OverrideLimbDraw (7 args).
// Call right before SoH3D_SkelAnimeDraw / ...Raw at each choke point that has an override on hand;
// NULL override clears it. Consumed once per retarget. #23.
void SoH3D_SetLimbOverride(void* overrideFn, void* arg, int kind);

// Raw variant of the N64-anim hook for draw choke points that don't have a SkelAnime* on hand
// (SkelAnime_DrawFlexOpa / SkelAnime_DrawOpa, called directly by many actors). Same effect as
// SoH3D_SkelAnimeDraw; derives limbCount from the skeleton tree. Returns 1 if it drew the OoT3D
// model (caller skips the N64 limbs).
int SoH3D_SkelAnimeDrawRaw(PlayState* play, void** skeleton, Vec3s* jointTable);

// Record the live N64 animation pointer (an OTR path string in SoH) for the actor currently
// deferred for replacement, so the auto CSAB resolver can map it to the matching OoT3D CSAB. Called
// from the SkelAnime-bearing draw wrappers (func_80034BA0/CC4) whose inner SkelAnime_DrawFlex (the
// raw hook) has only the skeleton, not the animation. No-op when no replacement is pending.
void SoH3D_SetCurAnim(void* animation);

// Generalised per-room scene divert, called from Room_Draw. If SoH3D is enabled and
// the current scene has an OoT3D mapping (kSoH3dSceneNames) with a room CMB for
// room->num, draws that room geometry at the world origin (identity model matrix +
// the game camera, depth-correct via the scene pass) and returns 1 so the caller
// skips the N64 room mesh; returns 0 otherwise (caller draws the N64 room as normal).
int SoH3D_TryDrawRoom(PlayState* play, Room* room);

// Draw the OoT3D sky (BlueSky.zar tenkyu gradient dome) in place of the N64 normal-sky skybox.
// Called from Play_Draw at the skybox point; returns 1 if it drew the OoT3D sky (caller skips the
// N64 SkyboxDraw_Draw), 0 otherwise (caller draws the N64 skybox as normal). #28.
int SoH3D_TryDrawSky(PlayState* play);

// Draw the OoT3D sun/moon discs (BlueSky.zar fine_sun.ctxb / fine_moon0.ctxb billboards) in place
// of the N64 Environment_DrawSunAndMoon sprites. Called from Play_Draw at that call site; returns 1
// if it drew the OoT3D sun/moon (caller skips the N64 path), 0 otherwise (caller draws N64). #28e.
int SoH3D_TryDrawSunMoon(PlayState* play);

// Emit the once-per-frame SoH3D render-pass marker (drains all SoH3D draws collected this frame
// in one GL-state-bracketed pass). Call from Play_Draw right after the actor draw-all.
void SoH3D_EmitRenderPass(PlayState* play);
// Per-frame, before the display list is built: drop any SoH3D draws left unrendered from a prior
// frame. Call once per frame ahead of Play_Draw (e.g. alongside the REPL poll).
void SoH3D_FrameBegin(void);

// Floor-height callback: returns the N64 collision floor Y at world (x,z), or a value
// <= -31000 if there is no floor. Provided by soh3d.c (it has the PlayState/colCtx).
typedef float (*SoH3D_FloorFn)(float x, float z);

// Compute & cache an OoT3D scene-room's ground-delta field D(x,z)=N64_floor-OoT3D_floor (the
// render mesh is left untouched; actors are offset by -D to stand on the visible OoT3D
// ground). Idempotent per model. Call from the room-draw hook before the room is drawn.
void SoH3D_ComputeRoomGroundDelta(int modelId, SoH3D_FloorFn floorFn);

// Sample that field: *outD = N64_floor - OoT3D_floor at world (x,z). Returns 1 on success.
int SoH3D_RoomGroundDeltaAt(int modelId, float x, float z, float* outD);

// OoT3D render-mesh floor at (x,z) for a scene room, the floor hit closest to `target`.
// Returns 1 + *outY on a hit. Grounds actors exactly on the visible OoT3D ground (exact, no
// grid approximation; XZ-bbox-culled so per-actor-per-frame is cheap).
int SoH3D_RoomOoT3DFloorAt(int modelId, float x, float z, float target, float* outY);

// Render-Y offset for an actor so it stands on the visible OoT3D ground instead of floating
// at the N64 collision height (= OoT3D_ground - N64_ground at the actor's XZ, i.e. -D).
// Returns 0 when SoH3D is off, the scene has no OoT3D room, or no delta covers the actor.
// Called from Actor_Draw: add to actor->world.pos.y around the draw, then subtract (physics
// stays N64). The inverse of the old render warp (which distorted the mesh).
float SoH3D_ActorRenderYOffset(PlayState* play, Actor* actor);

// Query the (warped) OoT3D room render-mesh floor Y at world (x,z). Returns 1 + *outY
// on a floor hit, else 0. For verifying the warp aligned the drawn ground to N64.
int SoH3D_RoomMeshFloorAt(int modelId, float x, float z, float* outY);

// #5 — real stepped-polygon stairs: replace OoT3D fake-flat "kaidan" ramps with actual
// treads+risers. SetStairs evicts cached scene-room models so the scene rebuilds (live A/B).
void SoH3D_SetStairs(int on);
int SoH3D_GetStairs(void);

// World scale for the OoT3D pot (OoT3D model units -> N64 world units). Tuned by
// matching the rendered height of the OoT3D pot to the N64 pot at the same spot
// (spawn comparison, Deku Tree). The OoT3D model is ~162 units tall; ~0.12 lands
// it at the N64 pot's height. See PROGRESS.md calibration.
#define SOH3D_POT_WORLD_SCALE 0.12f

// World scale for the OoT3D large wooden crate (Obj_Kibako2). Model is ~600 units
// wide; calibrated against the N64 large crate via the SOH3D_SPAWNKIBAKO A/B spawn
// in Gerudo Valley (ENTR 0x117=279). Third object proving the table-driven divert:
// adding it was one sModelTable[] row + this macro + the generated include.
#define SOH3D_KIBAKO_WORLD_SCALE 0.10f

// World scale for the OoT3D field bush reused for Obj_Hana's bush variant (params&3==2),
// the cuttable shrub the grass-cutting Kokiri picks. Same kusa model as En_Kusa (glModelId 2);
// starts at the kusa scale and is fine-tuned against the N64 Obj_Hana bush. REPL `scale kusa`.
#define SOH3D_HANABUSH_WORLD_SCALE 0.5f

// World scales for the field-keep props (En_Ishi rocks, Obj_Hana flower) reused from
// zelda_field_keep.zar. Starting estimates against the N64 actor scale (rock 0.1/0.4,
// flower 0.01); fine-tune live with REPL `scale rock_s|rock_l|flower`.
#define SOH3D_ROCK_SMALL_WORLD_SCALE 0.12f  // calibrated vs N64 in Kokiri Forest
#define SOH3D_ROCK_LARGE_WORLD_SCALE 0.12f  // UNCALIBRATED: no silver rocks in Kokiri Forest yet
#define SOH3D_FLOWER_WORLD_SCALE 0.12f      // UNCALIBRATED: no field flowers in Kokiri Forest yet

// Kakariko well + windmill (Bg_Spot01_Fusya / _Idohashira / _Idomizu), all from one shared ZAR
// coordinate space (zelda_spot01_objects.zar). Seeded from the auto-derived per-object scale
// (n64h 130 / fusya CMB 10255 ~= 0.0127). Tune live with REPL `gscale 7|8|9 <f>`.
#define ZSPOT01 "/actor/zelda_spot01_objects.zar"
#define SOH3D_SPOT01_WORLD_SCALE 0.01268f

// Kakariko DM-trail gate (Bg_Gate_Shutter, OBJECT_SPOT01_MATOYAB). It shares
// zelda_spot01_matoyab.zar with the windmill mechanism (c_matoate_before), but the two CMBs are
// authored at DIFFERENT unit scales (matoate ~1402 units tall, the gate c_s01tomegate ~111), so the
// gate needs its OWN scale — calibrated to 1.4 in-game (gate fills the DM-trail archway). REPL
// `gscale 10 <f>`.
#define ZMATOYAB "/actor/zelda_spot01_matoyab.zar"
#define SOH3D_MATOYAB_WORLD_SCALE 1.4f

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
// 1 = cold boot the auto-warp save as a clean NEW game (not the vanilla debug save). env SOH3D_COLDBOOT.
int SoH3D_ColdBoot(void);

// OoT3D get-item replacement, called from GetItem_Draw (the single get-item draw choke).
// When SoH3D + items are enabled and the drawId has an OoT3D /actor/zelda_gi_*.zar model,
// draws that model at the caller's current matrix and returns 1 so GetItem_Draw skips the
// N64 item DL; returns 0 otherwise (N64 item draws as normal). Covers chest contents,
// held-aloft rewards, shop displays and cutscene items uniformly (all route through here).
int SoH3D_TryDrawGetItem(PlayState* play, s16 drawId);

// Verification helper, called each frame from Play_Draw (before SoH3D_EmitRenderPass).
// When env SOH3D_SPAWNGI=<gid> is set, draws that get-item in front of Link via the real
// GetItem_Draw path so SOH3D=0 (N64) vs SOH3D=1 (OoT3D) can be A/B'd. No-op otherwise.
void SoH3D_DebugDrawGetItem(PlayState* play);

// OoT3D Link (player) replacement, called from Player_Draw just before the N64 body draw
// (Player_DrawGameplay). PROOF-OF-HOOK STAGE: when SoH3D + the link sub-toggle (env
// SOH3D_LINK, default OFF) are enabled, draws the OoT3D link_boy/child_new body CMB at the
// player's world transform in BIND POSE and returns 1 so the caller skips the N64 body.
// Returns 0 otherwise (N64 Link draws). Animation (N64-joint retarget) + held equipment are
// the next stage — see scratch/handoff_link.md.
int SoH3D_TryDrawPlayer(PlayState* play, Actor* actor);

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

// OoT3D collision: build a SoH CollisionHeader from the current scene's OoT3D scene-collision
// mesh, or NULL when disabled/unavailable (caller then uses the N64 collision). Called from
// Scene_CommandCollisionHeader at scene load; if it returns non-NULL the engine installs that
// header into play->colCtx via BgCheck_Allocate, so ALL gameplay collision (Link physics,
// floor/wall/ceiling) runs on OoT3D geometry — one geometry for visuals + gameplay. The header
// + arrays are kept resident for the scene lifetime (freed on the next build). Gated by
// SoH3D_CollisionEnabled() (env SOH3D_COLLISION, default ON; REPL `collision`). `n64` is the
// scene's N64 CollisionHeader (its waterboxes + camera regions are copied into the OoT3D header
// since those sub-lists aren't REd yet; pass NULL to skip).
CollisionHeader* SoH3D_BuildSceneCollision(PlayState* play, CollisionHeader* n64);
int SoH3D_CollisionEnabled(void);

// Interactive REPL poll, called once per frame from Play_Draw. When env
// SOH3D_REPL=<fifo path> is set, reads control commands from that FIFO and applies
// them live (tint, world scale, model spawn, on-demand frame dump) so a single
// long-lived headless instance can be poked without a rebuild/restart. No-op when
// SOH3D_REPL is unset. Drive it with tools/soh3d_repl.py.
void SoH3D_ReplPoll(PlayState* play);

// Inject the `walkhold` REPL control-stick value into player input. Call from Play_Main right
// BEFORE Play_Update so the player reads it (input is re-sampled each frame). No-op unless active.
void SoH3D_WalkInject(PlayState* play);

// Force the env SOH3D_TIME time-of-day into the save context. Call from Play_Init before the
// day/night scene setup layer is chosen, so the loaded actor set matches the forced clock.
void SoH3D_ApplyForceTime(void);

#endif
