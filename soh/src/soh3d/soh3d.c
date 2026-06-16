// SoH3D runtime toggle + helpers. See repo-root PROGRESS.md.
#include "soh3d.h"
#include "overlays/actors/ovl_En_Ge1/z_en_ge1.h" // EnGe1 (read live SkelAnime state)
#include "objects/object_ge1/object_ge1.h"       // dgGerudoWhite*Anim OTR-path strings
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

// --- Live tunables, pokeable at runtime via the REPL (SoH3D_ReplPoll) ---
// All initialised from env on first use (back-compat with the old SOH3D_* env
// flow), then overridable live over the control FIFO so experiments don't need a
// rebuild/restart. See tools/soh3d_repl.py and PROGRESS.md.
float gSoH3dTintDiff = 0.5f; // diffuse fraction in the flat scene tint
float gSoH3dTintMul = 1.0f;  // overall tint brightness multiplier
int gSoH3dEnabled = -1;      // -1 = uninit (read env), 0/1 = OoT3D render off/on

// Live debug orientation (degrees) applied in SoH3D_DrawModel BEFORE the model
// dlist, so the correct in-game rest->upright bake can be found over the REPL
// without a rebuild. Once a value is confirmed in-game it gets baked into the
// generated model via cmb_to_c --rotx/--roty/--rotz and these reset to 0.
// (The harness is NOT a faithful orientation proxy — see PROGRESS.md.)
float gSoH3dRotX = 0.0f;
float gSoH3dRotY = 0.0f;
float gSoH3dRotZ = 0.0f;

// Live CSAB animation playback (GPU skinning). gSoH3dAnimRate = anim-frames advanced
// per draw (the OoT3D logic tick is ~20 fps; tune live over the REPL). The frame is
// a free-running accumulator — the CSAB wraps it (REPEAT) internally.
float gSoH3dAnimFrame = 0.0f;
float gSoH3dAnimRate = 1.0f; // 0 = paused (hold current frame)
// 1 = drive the CSAB from the actor's live N64 SkelAnime (correct anim + speed,
// see SoH3D_AnimResolver); 0 = free-running gSoH3dAnimFrame for REPL scrubbing.
int gSoH3dAnimLive = 1;
int gSoH3dAnimDebug = 0; // REPL `animdbg 1`: log resolved csab/curFrame/phase each ~20 draws

// Per-GL-model live playback state, so multiple DISTINCT GL characters animate
// independently (gSoH3dAnimRate is the shared speed knob; the frame accumulator and
// last-played CSAB are per model). Indexed by glModelId. NOTE: this is per MODEL, not
// per actor instance — two instances of the same GL model still share one pose (the
// skin matrices are uploaded per modelId); independent per-instance poses would need
// per-actor bone buffers, out of scope here.
#define SOH3D_GL_MODEL_MAX 16
static struct {
    float frame;
    const char* lastCsab;
} gSoH3dGlAnim[SOH3D_GL_MODEL_MAX];

// Direct-GL model path (soh3d_model.cpp bridge + libultraship SoH3D_GL_*). Models
// flagged with glModelId>=0 in sModelTable render through this PC-native path
// (runtime-loaded 3DS asset, our own GL shader) instead of the legacy N64 dlist.
void SoH3D_EnsureModelProvider(void);
void SoH3D_UpdateAnim(int modelId, const char* animName, float frame);
// Get-or-allocate a scene-room model id (soh3d_model.cpp). Keyed by ZSI path; loads
// the embedded room CMB lazily on first draw. Returns -1 for an unmapped scene.
int SoH3D_RoomModelId(const char* sceneName, int roomNum);

// SoH sceneNum -> OoT3D scene folder name (kSoH3dSceneNames). Generated, names only.
#include "soh3d_scene_names.inc"

// Scene-geometry world transform (REPL-pokeable). OoT3D scene coords are already
// WORLD-space at (apparently) the N64 unit scale, so the defaults are identity:
// scale 1.0 at the world origin. Tunable live to confirm the unit/origin match.
float gSoH3dSceneScale = 1.0f;
float gSoH3dSceneOffX = 0.0f, gSoH3dSceneOffY = 0.0f, gSoH3dSceneOffZ = 0.0f;

// --- Terrain warp: re-level the OoT3D room render ground to the N64 collision floor
// (so Link, who walks on N64 collision, stands on the visible ground). The mesh re-level
// runs in soh3d_model.cpp (SoH3D_WarpRoomToN64); this side supplies the N64 floor probe
// and the on/off gate. Default ON; disable with env SOH3D_TERRAIN_WARP=0 for A/B. ---
int gSoH3dTerrainWarp = 1;
static PlayState* sWarpPlay = NULL; // current PlayState for the floor callback (set per draw)

// --- Force time-of-day (debugging): when gSoH3dForceTime >= 0, pin gSaveContext.dayTime
// to it every frame so a scene loads/stays at a chosen time (e.g. day instead of night).
// 0x8000 = noon, 0x4000 = dawn, 0xC000 = dusk, 0x0000 = midnight. Set via env SOH3D_TIME
// (decimal or 0xHEX) at launch, or live via REPL `time`. -1 = leave the game's clock alone. ---
int gSoH3dForceTime = -1;

static void SoH3D_InitForceTime(void) {
    static int done = 0;
    const char* v;
    if (done) {
        return;
    }
    done = 1;
    v = getenv("SOH3D_TIME");
    if (v != NULL && v[0] != '\0') {
        gSoH3dForceTime = (int)strtol(v, NULL, 0); // 0 base: accepts 0x.. hex or decimal
    }
}

static int SoH3D_TerrainWarpEnabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char* v = getenv("SOH3D_TERRAIN_WARP");
        cached = (v != NULL && v[0] == '0') ? 0 : 1; // default ON
    }
    return cached && gSoH3dTerrainWarp;
}

// N64 collision floor height at world (x,z): raycast straight down through BgCheck from
// high above (same as the REPL `floorat`). Used by SoH3D_WarpRoomToN64 to build the warp.
static float SoH3D_N64FloorCb(float x, float z) {
    Vec3f pos;
    CollisionPoly* poly = NULL;
    f32 y;
    if (sWarpPlay == NULL) {
        return -32000.0f;
    }
    pos.x = x;
    pos.y = 10000.0f;
    pos.z = z;
    y = BgCheck_EntityRaycastFloor1(&sWarpPlay->colCtx, &poly, &pos);
    return (poly != NULL) ? y : -32000.0f;
}

// --- Diagnostic camera override (REPL `cam` / `camorbit` / `camfreeze`) ---
// When gSoH3dCamOverride != 0, SoH3D_ReplPoll forces play->view.eye/lookAt/up every
// frame. The camera engine recomputes the view in Play_Update; the poll runs AFTER
// Play_Update and BEFORE Play_Draw, so re-applying there wins for the rendered frame.
// Purpose: freeze the world and ORBIT the camera about a fixed look point. The OoT3D
// scene (GL) and N64 actors (Fast3D) share the same MP matrix, so under a pure camera
// rotation they can ONLY drift apart if their WORLD coords differ (origin/scale
// mismatch). A controlled orbit makes that drift measurable instead of eyeballed.
int gSoH3dCamOverride = 0;
float gSoH3dCamEye[3] = { 0, 0, 0 };
float gSoH3dCamAt[3] = { 0, 0, 0 };

// Resolve the actor's CURRENT animation to a CSAB base name, by reading the actor's
// live N64 state, so the OoT3D model plays the same animation the game logic chose
// (idle/talk/gate-open). Returns the CSAB base name (NULL = bind pose). The CSAB is
// then free-run at its own authored rate (see SoH3D_DrawModelGL) rather than locked to
// the N64 SkelAnime frame: several N64 anims (notably En_Ge1's 2-frame idle stub, whose
// life comes from procedural limb fidget, not keyframes) carry no frame motion to sync
// to, so the OoT3D CSAB's own motion is the faithful source.
typedef const char* (*SoH3D_AnimResolver)(Actor* actor);
static void SoH3D_DrawModelGL(PlayState* play, int modelId, Actor* actor, float worldScale,
                              const char* animName, float groundOffset, SoH3D_AnimResolver resolveAnim);

// On-demand frame dump trigger, defined in libultraship's gfx_sdl2.cpp.
extern char gSoh3dDumpPath[1024];
extern volatile int gSoh3dDumpPending;

// Flat scene-ambient tint for the unlit OoT3D dlist. The converter's unlit dlist
// modulates its texture by the PRIMITIVE register (G_CC_MODULATERGBA_PRIM) rather
// than vertex SHADE, so a single per-draw colour tints the whole model — it
// darkens/colour-shifts with the room without the per-vertex banding that N64
// lighting produces on these low-poly meshes.
//
// N64 shade = ambient + Σ diffuse·max(0, N·L). For one flat value we approximate
// with ambient + a fraction of the scene's two (opposed) directional lights, read
// LIVE from the interpolated scene light settings so it tracks time of day. The
// diffuse fraction and an overall brightness are calibrated against the N64 model
// in the same scene (see PROGRESS.md) and tunable via SOH3D_TINT_* for re-cal.
static void SoH3D_SceneTint(PlayState* play, u8 out[3]) {
    EnvLightSettings* ls = &play->envCtx.lightSettings;
    static int init = 0;
    s32 i;
    if (!init) {
        const char* fv = getenv("SOH3D_TINT_DIFF");
        const char* mv = getenv("SOH3D_TINT_MUL");
        if (fv != NULL && fv[0] != '\0') gSoH3dTintDiff = (float)atof(fv);
        if (mv != NULL && mv[0] != '\0') gSoH3dTintMul = (float)atof(mv);
        init = 1;
    }
    for (i = 0; i < 3; i++) {
        float v = ((float)ls->ambientColor[i] +
                   gSoH3dTintDiff * ((float)ls->light1Color[i] + (float)ls->light2Color[i])) *
                  gSoH3dTintMul;
        out[i] = (v <= 0.0f) ? 0 : (v >= 255.0f) ? 255 : (u8)(v + 0.5f);
    }
}

// Draw an OoT3D model at an actor's world position/yaw with an explicit world
// scale. Builds its own MTXMODE_NEW matrix instead of inheriting the actor's
// N64-tuned 0.01 scale: that inherited fixed-point matrix fails to render the
// model at all (the OoT3D dlist needs SoH3D's own transform), and it would size
// the full-res model wrongly besides.
void SoH3D_DrawModel(PlayState* play, Gfx* dlist, Actor* actor, float worldScale) {
    u8 tint[3];
    OPEN_DISPS(play->state.gfxCtx);

    Gfx_SetupDL_25Opa(play->state.gfxCtx);
    Matrix_Translate(actor->world.pos.x, actor->world.pos.y, actor->world.pos.z, MTXMODE_NEW);
    Matrix_RotateY(BINANG_TO_RAD(actor->shape.rot.y), MTXMODE_APPLY);
    Matrix_Scale(worldScale, worldScale, worldScale, MTXMODE_APPLY);
    // Debug rest->upright orientation, found live then baked into the model (see above).
    if (gSoH3dRotX != 0.0f) Matrix_RotateX(gSoH3dRotX * (3.14159265f / 180.0f), MTXMODE_APPLY);
    if (gSoH3dRotY != 0.0f) Matrix_RotateY(gSoH3dRotY * (3.14159265f / 180.0f), MTXMODE_APPLY);
    if (gSoH3dRotZ != 0.0f) Matrix_RotateZ(gSoH3dRotZ * (3.14159265f / 180.0f), MTXMODE_APPLY);
    gSPMatrix(POLY_OPA_DISP++, MATRIX_NEWMTX(play->state.gfxCtx), G_MTX_MODELVIEW | G_MTX_LOAD);
    // Flat scene tint -> PRIMITIVE; the unlit dlist's combiner is TEXEL0 * PRIM.
    // Must be set before the dlist runs (the dlist deliberately sets no prim).
    SoH3D_SceneTint(play, tint);
    gDPSetPrimColor(POLY_OPA_DISP++, 0, 0, tint[0], tint[1], tint[2], 255);
    gSPDisplayList(POLY_OPA_DISP++, dlist);

    CLOSE_DISPS(play->state.gfxCtx);
}

// Direct-GL draw: same world matrix as SoH3D_DrawModel, but instead of a Fast3D
// dlist it loads the modelview and emits the OTR_G_SOH3D_DRAW opcode. At dlist-exec
// time libultraship runs our GL renderer (SoH3D_GL_Draw) with the current MP_matrix
// — model verts are raw 3DS geometry, textures uploaded from the runtime loader, no
// N64 TMEM/segment path. Depth-correct because it draws inside the scene pass.
static void SoH3D_DrawModelGL(PlayState* play, int modelId, Actor* actor, float worldScale,
                              const char* animName, float groundOffset, SoH3D_AnimResolver resolveAnim) {
    u8 tint[3];
    OPEN_DISPS(play->state.gfxCtx);

    SoH3D_EnsureModelProvider();
    // Apply this model's skeletal animation (GPU skinning), once per Actor_Draw.
    // Live (gSoH3dAnimLive): the resolver picks WHICH CSAB by the actor's live N64
    // state (idle/talk/gate-open); the CSAB then free-runs at its own authored rate,
    // restarting from frame 0 whenever the selection changes (so a one-shot like the
    // gate-open clap begins at its start). Each GL model keeps its own frame accumulator
    // (gSoH3dGlAnim[modelId]) so distinct characters don't share a playhead. Scrub
    // (live=0 or no resolver): the global gSoH3dAnimFrame on the fixed table anim, so
    // the REPL animframe/animrate knobs still work for debugging one model.
    const char* animToPlay = animName;
    float* frame = &gSoH3dAnimFrame; // scrub default
    if (gSoH3dAnimLive && resolveAnim != NULL && modelId >= 0 && modelId < SOH3D_GL_MODEL_MAX) {
        const char* csab = resolveAnim(actor);
        const char* prev = gSoH3dGlAnim[modelId].lastCsab;
        int changed = (prev == NULL || csab == NULL) ? (prev != csab) : (strcmp(prev, csab) != 0);
        if (changed) {
            gSoH3dGlAnim[modelId].frame = 0.0f; // anim changed -> restart playback
            gSoH3dGlAnim[modelId].lastCsab = csab;
        }
        animToPlay = csab;
        frame = &gSoH3dGlAnim[modelId].frame;
    }
    if (animToPlay != NULL) {
        SoH3D_UpdateAnim(modelId, animToPlay, *frame);
        *frame += gSoH3dAnimRate;
    }
    Gfx_SetupDL_25Opa(play->state.gfxCtx);
    Matrix_Translate(actor->world.pos.x, actor->world.pos.y, actor->world.pos.z, MTXMODE_NEW);
    Matrix_RotateY(BINANG_TO_RAD(actor->shape.rot.y), MTXMODE_APPLY);
    Matrix_Scale(worldScale, worldScale, worldScale, MTXMODE_APPLY);
    if (gSoH3dRotX != 0.0f) Matrix_RotateX(gSoH3dRotX * (3.14159265f / 180.0f), MTXMODE_APPLY);
    if (gSoH3dRotY != 0.0f) Matrix_RotateY(gSoH3dRotY * (3.14159265f / 180.0f), MTXMODE_APPLY);
    if (gSoH3dRotZ != 0.0f) Matrix_RotateZ(gSoH3dRotZ * (3.14159265f / 180.0f), MTXMODE_APPLY);
    // Ground offset: applied innermost (model space, pre-scale) so it scales with
    // worldScale and brings the model's feet onto the actor's ground pos.
    if (groundOffset != 0.0f) Matrix_Translate(0.0f, groundOffset, 0.0f, MTXMODE_APPLY);
    gSPMatrix(POLY_OPA_DISP++, MATRIX_NEWMTX(play->state.gfxCtx), G_MTX_MODELVIEW | G_MTX_LOAD);
    SoH3D_SceneTint(play, tint);
    gSPSoH3DDraw(POLY_OPA_DISP++, modelId, tint[0], tint[1], tint[2]);

    CLOSE_DISPS(play->state.gfxCtx);
}

// Per-actor OoT3D model table. Maps an N64 actor id to the OoT3D model dlist that
// replaces its N64 draw, plus that model's world scale. This is the generalised
// divert: instead of editing each actor's Draw with an `if (SoH3D_Enabled())`
// block, Actor_Draw consults this table once for every actor (SoH3D_TryDrawActor)
// and, on a hit, draws the OoT3D model and skips the N64 draw. Add an object by
// adding a row here — no actor-source edits.
// En_Ge1 (white Gerudo): map her N64 animation -> the OoT3D CSAB, and phase-sync to her
// SkelAnime clock. The N64 actor stores the current anim as an OTR-path string in
// this->animation (SoH ALIGN_ASSET pattern), so identify it by strcmp. Mapping (by use
// site in z_en_ge1.c): Idle->ge1_s_wait, Clap(open-gate)->ge1_mon_akeru, Dismissive
// (post-talk reaction)->ge1_hanasi. ge1_matsu is unused by this actor's 3 N64 anims.
static const char* SoH3D_ResolveAnim_EnGe1(Actor* actor) {
    EnGe1* ge = (EnGe1*)actor;
    const char* n64 = (const char*)ge->animation;
    const char* csab = "ge1_s_wait"; // idle / unknown
    if (n64 != NULL) {
        if (strcmp(n64, dgGerudoWhiteClapAnim) == 0) {
            csab = "ge1_mon_akeru";
        } else if (strcmp(n64, dgGerudoWhiteDismissiveAnim) == 0) {
            csab = "ge1_hanasi";
        }
    }
    if (gSoH3dAnimDebug) {
        static int dbg = 0;
        if ((dbg++ % 20) == 0) {
            printf("SOH3D anim: csab=%s curFrame=%.2f animLength=%.2f n64=%s\n",
                   csab, ge->skelAnime.curFrame, ge->skelAnime.animLength, n64 ? n64 : "(null)");
            fflush(stdout);
        }
    }
    return csab;
}

typedef struct {
    s16 actorId;
    const char* name; // REPL handle for `scale <name>` / `spawn <name>`
    Gfx* dlist;       // legacy Fast3D dlist (used when glModelId < 0)
    float worldScale; // live (REPL-pokeable)
    int glModelId;    // >=0 = render via the direct-GL path with this asset id; -1 = legacy dlist
    const char* anim; // CSAB base name to play on the GL path (NULL = bind pose / no anim).
                      // Used as the fallback when resolveAnim is NULL or scrubbing live=0.
    float groundOffset; // model-space Y added BEFORE scale, so the model's feet land on
                        // the actor's ground pos. Pre-scale => scales with worldScale, so
                        // re-tuning scale does not desync grounding. REPL `yoff <name> <f>`.
    SoH3D_AnimResolver resolveAnim; // NULL = no live anim state (use `anim` + free frame)
} SoH3D_ModelEntry;

// Non-const so the REPL can tune worldScale/groundOffset live.
static SoH3D_ModelEntry sModelTable[] = {
    { ACTOR_OBJ_TSUBO, "pot", soh3d_pot_model_dl, SOH3D_POT_WORLD_SCALE, -1, NULL, 0.0f, NULL },
    { ACTOR_EN_GS, "gs", soh3d_gs_model_dl, SOH3D_GS_WORLD_SCALE, -1, NULL, 0.0f, NULL },
    { ACTOR_OBJ_KIBAKO2, "kibako", soh3d_kibako_model_dl, SOH3D_KIBAKO_WORLD_SCALE, -1, NULL, 0.0f, NULL },
    { ACTOR_EN_GE1, "geldwoman", soh3d_geldwoman_model_dl, SOH3D_GELDWOMAN_WORLD_SCALE, 0, "ge1_s_wait",
      SOH3D_GELDWOMAN_GROUND_OFFSET, SoH3D_ResolveAnim_EnGe1 },
};

int SoH3D_TryDrawActor(PlayState* play, Actor* actor) {
    s32 i;
    if (!SoH3D_Enabled()) {
        return 0;
    }
    for (i = 0; i < ARRAY_COUNT(sModelTable); i++) {
        if (sModelTable[i].actorId == actor->id) {
            if (sModelTable[i].glModelId >= 0) {
                SoH3D_DrawModelGL(play, sModelTable[i].glModelId, actor, sModelTable[i].worldScale,
                                  sModelTable[i].anim, sModelTable[i].groundOffset, sModelTable[i].resolveAnim);
            } else {
                SoH3D_DrawModel(play, sModelTable[i].dlist, actor, sModelTable[i].worldScale);
            }
            return 1;
        }
    }
    return 0;
}

int SoH3D_Enabled(void) {
    if (gSoH3dEnabled < 0) {
        const char* v = getenv("SOH3D");
        gSoH3dEnabled = (v != NULL && v[0] == '1') ? 1 : 0;
    }
    return gSoH3dEnabled;
}

// OoT3D scene folder name for the current scene number, or NULL if unmapped (no OoT3D
// equivalent — caller falls back to the N64 room).
static const char* SoH3D_SceneName(PlayState* play) {
    s32 n = play->sceneNum;
    if (n < 0 || n >= (s32)ARRAY_COUNT(kSoH3dSceneNames)) {
        return NULL;
    }
    return kSoH3dSceneNames[n];
}

// Direct-GL room draw: same dlist path as the character GL draw, but the model matrix
// is IDENTITY (scene CMB verts are already world-space) — just an optional debug
// offset + uniform scale. MP_matrix at opcode time is then model(identity)·view·proj =
// the game camera, so the room lands at the world origin, depth-correct in the scene
// pass. Tinted by the live scene ambient like the characters.
static void SoH3D_DrawRoomGL(PlayState* play, int modelId) {
    u8 tint[3];
    OPEN_DISPS(play->state.gfxCtx);

    SoH3D_EnsureModelProvider();
    Gfx_SetupDL_25Opa(play->state.gfxCtx);
    Matrix_Translate(gSoH3dSceneOffX, gSoH3dSceneOffY, gSoH3dSceneOffZ, MTXMODE_NEW);
    Matrix_Scale(gSoH3dSceneScale, gSoH3dSceneScale, gSoH3dSceneScale, MTXMODE_APPLY);
    gSPMatrix(POLY_OPA_DISP++, MATRIX_NEWMTX(play->state.gfxCtx), G_MTX_MODELVIEW | G_MTX_LOAD);
    SoH3D_SceneTint(play, tint);
    gSPSoH3DDraw(POLY_OPA_DISP++, modelId, tint[0], tint[1], tint[2]);

    CLOSE_DISPS(play->state.gfxCtx);
}

int SoH3D_TryDrawRoom(PlayState* play, Room* room) {
    const char* sceneName;
    int modelId;
    // Debug isolation: SOH3D_SCENE=0 disables ONLY the scene/room divert (actors still
    // divert), so a crash can be bisected room-divert vs actor-divert without a rebuild.
    static int sceneDivert = -1;
    if (sceneDivert < 0) {
        const char* v = getenv("SOH3D_SCENE");
        sceneDivert = (v != NULL && v[0] != '\0') ? atoi(v) : 1; // 0=off,1=draw,2=skip-only
    }
    if (sceneDivert == 0 || !SoH3D_Enabled() || room == NULL) {
        return 0;
    }
    sceneName = SoH3D_SceneName(play);
    if (sceneName == NULL) {
        return 0; // scene has no OoT3D mapping -> N64 room
    }
    modelId = SoH3D_RoomModelId(sceneName, room->num);
    if (modelId < 0) {
        return 0;
    }
    // Debug isolation: SOH3D_SCENE=2 skips the N64 room mesh but draws NOTHING (no GL),
    // to bisect "skipping the N64 room corrupts state" vs "our GL draw corrupts state".
    if (sceneDivert != 2) {
        // Re-level the room's render ground to the N64 collision floor (once per model).
        // Done here (not in the lazy provider) because we have the PlayState/colCtx for
        // the floor probe; the warp marks the model done so it is a one-time cost.
        if (SoH3D_TerrainWarpEnabled()) {
            sWarpPlay = play;
            SoH3D_WarpRoomToN64(modelId, SoH3D_N64FloorCb);
        }
        SoH3D_DrawRoomGL(play, modelId);
    }
    return 1; // drew the OoT3D room -> caller skips the N64 mesh
}

int SoH3D_AutoWarpEnabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char* v = getenv("SOH3D_WARP");
        cached = (v != NULL && v[0] != '\0') ? 1 : 0;
    }
    return cached;
}

int SoH3D_AutoWarpEntrance(void) {
    const char* v = getenv("SOH3D_ENTRANCE");
    if (v != NULL && v[0] != '\0') {
        return atoi(v);
    }
    return ENTR_KAKARIKO_VILLAGE_FRONT_GATE;
}

void SoH3D_DebugDrawPot(PlayState* play) {
    // Verification: spawn one real Obj_Tsubo beside Link (env SOH3D_SPAWNPOT=1) so
    // the actual ObjTsubo_Draw path runs. SOH3D=0 draws the N64 pot, SOH3D=1 the
    // OoT3D model — a true same-scene comparison. params=0 is the
    // gameplay_dangeon_keep pot variant (object loaded in any dungeon, e.g. Deku
    // Tree / SOH3D_ENTRANCE=0).
    const char* sp = getenv("SOH3D_SPAWNPOT");
    static unsigned char spawned = 0;
    if (sp != NULL && sp[0] == '1' && !spawned) {
        Player* p = GET_PLAYER(play);
        s16 yaw = p->actor.shape.rot.y + 0x4000; // Link's right (avoid the Deku entrance pit)
        float fx = p->actor.world.pos.x + 60.0f * Math_SinS(yaw);
        float fz = p->actor.world.pos.z + 60.0f * Math_CosS(yaw);
        Actor_Spawn(&play->actorCtx, play, ACTOR_OBJ_TSUBO, fx, p->actor.world.pos.y, fz, 0, 0, 0, 0);
        spawned = 1;
    }
}

void SoH3D_DebugDrawGs(PlayState* play) {
    // Verification: spawn one real En_Gs (Gossip Stone) in front of Link
    // (env SOH3D_SPAWNGS=1) so the actual EnGs_Draw path runs. SOH3D=0 draws the
    // N64 Gossip Stone, SOH3D=1 the OoT3D multi-material one. Needs OBJECT_GS in
    // the scene (a Gossip-Stone scene, e.g. the default Kakariko warp).
    const char* sp = getenv("SOH3D_SPAWNGS");
    static unsigned char spawned = 0;
    if (sp != NULL && sp[0] == '1' && !spawned) {
        Player* p = GET_PLAYER(play);
        s16 yaw = p->actor.shape.rot.y; // in front of Link (where the camera looks)
        float fx = p->actor.world.pos.x + 90.0f * Math_SinS(yaw);
        float fz = p->actor.world.pos.z + 90.0f * Math_CosS(yaw);
        // Face the Sheikah-eye front toward Link/camera.
        s16 gsYaw = p->actor.shape.rot.y;
        Actor_Spawn(&play->actorCtx, play, ACTOR_EN_GS, fx, p->actor.world.pos.y, fz, 0, gsYaw, 0, 0);
        spawned = 1;
    }
}

void SoH3D_DebugDrawKibako(PlayState* play) {
    // Verification: spawn one real Obj_Kibako2 (large crate) in front of Link
    // (env SOH3D_SPAWNKIBAKO=1). Needs OBJECT_KIBAKO2 in the scene (e.g. Gerudo
    // Valley). Logs spawn success so scene-object presence can be confirmed from
    // the log without interpreting pixels.
    const char* sp = getenv("SOH3D_SPAWNKIBAKO");
    static unsigned char spawned = 0;
    if (sp != NULL && sp[0] == '1' && !spawned) {
        Player* p = GET_PLAYER(play);
        s16 yaw = p->actor.shape.rot.y;
        float fx = p->actor.world.pos.x + 120.0f * Math_SinS(yaw);
        float fz = p->actor.world.pos.z + 120.0f * Math_CosS(yaw);
        Actor* a = Actor_Spawn(&play->actorCtx, play, ACTOR_OBJ_KIBAKO2, fx, p->actor.world.pos.y, fz, 0,
                               p->actor.shape.rot.y, 0, 0);
        printf("SOH3D: SPAWNKIBAKO Actor_Spawn(OBJ_KIBAKO2) -> %s\n", a != NULL ? "OK" : "FAILED (object not in scene)");
        fflush(stdout);
        spawned = 1;
    }
}

// ===========================================================================
// SoH3D REPL — interactive control of a long-lived headless instance.
//
// Tooling-first: instead of the env-flag -> rebuild -> 7-min headless render
// loop, keep ONE soh.elf running and poke it live over a control FIFO. Iterating
// on tint, world scale, model selection, spawns and on-demand frame dumps then
// costs seconds, not a rebuild. Enabled by env SOH3D_REPL=<fifo path>; the C side
// mkfifo()s it and replies to "<fifo>.out". Drive it with tools/soh3d_repl.py.
//
// Commands (one per line):
//   mul <f>            overall tint brightness          diff <f>  diffuse fraction
//   tint <diff> <mul>  set both                         enable <0|1>  OoT3D render
//   scale <name> <f>   live world scale for a model (pot|gs|kibako|geldwoman)
//   yoff <name> <f>    live ground offset (model-space Y, pre-scale) for a model
//   rotx/roty/rotz <f> live debug orientation (deg) for the GL model
//   animlive <0|1>     1=drive CSAB from the actor's SkelAnime; 0=scrub w/ animframe
//   animrate <f>       free-running frames/draw (scrub mode)  animframe <f>  set frame
//   spawn <name>       spawn that actor in front of Link (front-right, clears Link)
//   dump <path.ppm>    capture the current frame to <path> (no exit)
//   state              report all tunables + the current computed tint
// ===========================================================================

static SoH3D_ModelEntry* SoH3D_FindModel(const char* name) {
    s32 i;
    for (i = 0; i < ARRAY_COUNT(sModelTable); i++) {
        if (strcmp(sModelTable[i].name, name) == 0) {
            return &sModelTable[i];
        }
    }
    return NULL;
}

static Actor* SoH3D_SpawnInFront(PlayState* play, s16 actorId, float dist) {
    Player* p = GET_PLAYER(play);
    s16 yaw = p->actor.shape.rot.y;
    s16 right = yaw + 0x4000; // Link's right, to clear his body so feet/ground are visible
    float fx = p->actor.world.pos.x + dist * Math_SinS(yaw) + 55.0f * Math_SinS(right);
    float fz = p->actor.world.pos.z + dist * Math_CosS(yaw) + 55.0f * Math_CosS(right);
    return Actor_Spawn(&play->actorCtx, play, actorId, fx, p->actor.world.pos.y, fz, 0, p->actor.shape.rot.y, 0, 0);
}

static void SoH3D_ReplReply(const char* outPath, const char* fmt, ...) {
    char msg[512];
    va_list ap;
    FILE* f;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    printf("SOH3D REPL: %s\n", msg);
    fflush(stdout);
    f = fopen(outPath, "a");
    if (f != NULL) {
        fprintf(f, "%s\n", msg);
        fclose(f);
    }
}

static void SoH3D_ReplExec(PlayState* play, char* line, const char* outPath) {
    char cmd[32];
    char arg[64];
    char path[1024];
    float f1, f2, f3;
    while (*line == ' ' || *line == '\t' || *line == '\r') {
        line++;
    }
    if (*line == '\0' || *line == '#') {
        return;
    }
    if (sscanf(line, "%31s", cmd) != 1) {
        return;
    }
    if (strcmp(cmd, "mul") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dTintMul = f1;
        SoH3D_ReplReply(outPath, "mul=%.3f", gSoH3dTintMul);
    } else if (strcmp(cmd, "diff") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dTintDiff = f1;
        SoH3D_ReplReply(outPath, "diff=%.3f", gSoH3dTintDiff);
    } else if (strcmp(cmd, "tint") == 0 && sscanf(line, "%*s %f %f", &f1, &f2) == 2) {
        gSoH3dTintDiff = f1;
        gSoH3dTintMul = f2;
        SoH3D_ReplReply(outPath, "diff=%.3f mul=%.3f", gSoH3dTintDiff, gSoH3dTintMul);
    } else if (strcmp(cmd, "enable") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dEnabled = (int)f1;
        SoH3D_ReplReply(outPath, "enabled=%d", gSoH3dEnabled);
    } else if (strcmp(cmd, "tp") == 0 && sscanf(line, "%*s %f %f %f", &f1, &f2, &f3) == 3) {
        Player* p = GET_PLAYER(play);
        p->actor.world.pos.x = f1;
        p->actor.world.pos.y = f2;
        p->actor.world.pos.z = f3;
        p->actor.prevPos = p->actor.world.pos;
        SoH3D_ReplReply(outPath, "tp -> (%.0f,%.0f,%.0f)", f1, f2, f3);
    } else if (strcmp(cmd, "move") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        Player* p = GET_PLAYER(play);
        s16 yaw = p->actor.shape.rot.y;
        p->actor.world.pos.x += f1 * Math_SinS(yaw);
        p->actor.world.pos.z += f1 * Math_CosS(yaw);
        p->actor.prevPos = p->actor.world.pos;
        SoH3D_ReplReply(outPath, "move %.0f -> (%.0f,%.0f,%.0f)", f1, p->actor.world.pos.x, p->actor.world.pos.y,
                        p->actor.world.pos.z);
    } else if (strcmp(cmd, "turn") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        Player* p = GET_PLAYER(play);
        s16 yaw = (s16)(f1 * 182.0444f); // deg -> binang
        p->actor.shape.rot.y = yaw;
        p->actor.world.rot.y = yaw;
        SoH3D_ReplReply(outPath, "turn -> %.0f deg (yaw=%d)", f1, yaw);
    } else if (strcmp(cmd, "posinfo") == 0) {
        Player* p = GET_PLAYER(play);
        Camera* c = GET_ACTIVE_CAM(play);
        SoH3D_ReplReply(outPath,
                        "scene=0x%x link=(%.0f,%.0f,%.0f) yaw=%d | cam eye=(%.0f,%.0f,%.0f) at=(%.0f,%.0f,%.0f)",
                        play->sceneNum, p->actor.world.pos.x, p->actor.world.pos.y, p->actor.world.pos.z,
                        p->actor.shape.rot.y, c->eye.x, c->eye.y, c->eye.z, c->at.x, c->at.y, c->at.z);
    } else if (strcmp(cmd, "floorat") == 0 && sscanf(line, "%*s %f %f", &f1, &f2) == 2) {
        // Authoritative N64-collision floor height at world (x,z): raycast straight down
        // through SoH's BgCheck from high above. This is exactly the surface Link stands
        // on, so it is the ground truth the OoT3D render mesh must be warped to match.
        Vec3f pos = { f1, 10000.0f, f2 };
        CollisionPoly* poly = NULL;
        f32 y = BgCheck_EntityRaycastFloor1(&play->colCtx, &poly, &pos);
        if (poly != NULL) {
            SoH3D_ReplReply(outPath, "floorat (%.0f,%.0f) y=%.2f ny=%.4f", f1, f2, y,
                            COLPOLY_GET_NORMAL(poly->normal.y));
        } else {
            SoH3D_ReplReply(outPath, "floorat (%.0f,%.0f) NO FLOOR", f1, f2);
        }
    } else if (strcmp(cmd, "floorgrid") == 0) {
        // Batch raycast a regular XZ grid into a CSV (looped in C -> one FIFO round-trip,
        // not thousands). Used offline to build the dense N64 floor field for terrain warp.
        float x0, z0, x1, z1, step;
        char gpath[1024];
        if (sscanf(line, "%*s %f %f %f %f %f %1023s", &x0, &z0, &x1, &z1, &step, gpath) == 6 && step > 0.0f) {
            FILE* gf = fopen(gpath, "w");
            if (gf == NULL) {
                SoH3D_ReplReply(outPath, "floorgrid: cannot open %s", gpath);
            } else {
                int hits = 0;
                float x, z;
                fprintf(gf, "x,z,y,ny\n");
                for (z = z0; z <= z1; z += step) {
                    for (x = x0; x <= x1; x += step) {
                        Vec3f pos = { x, 10000.0f, z };
                        CollisionPoly* poly = NULL;
                        f32 y = BgCheck_EntityRaycastFloor1(&play->colCtx, &poly, &pos);
                        if (poly != NULL) {
                            fprintf(gf, "%.1f,%.1f,%.2f,%.4f\n", x, z, y, COLPOLY_GET_NORMAL(poly->normal.y));
                            hits++;
                        } else {
                            fprintf(gf, "%.1f,%.1f,nan,nan\n", x, z);
                        }
                    }
                }
                fclose(gf);
                SoH3D_ReplReply(outPath, "floorgrid -> %s (%d floor hits)", gpath, hits);
            }
        } else {
            SoH3D_ReplReply(outPath, "floorgrid needs: x0 z0 x1 z1 step path");
        }
    } else if (strcmp(cmd, "terrainwarp") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        // Toggle the terrain re-level. Note: the warp is applied once per room model and
        // CACHED, so toggling off does not un-warp already-loaded rooms (re-enter the
        // scene, or use env SOH3D_TERRAIN_WARP=0 from launch, for a clean A/B).
        gSoH3dTerrainWarp = (int)f1;
        SoH3D_ReplReply(outPath, "terrainwarp=%d (applies to rooms loaded after this)", gSoH3dTerrainWarp);
    } else if (strcmp(cmd, "time") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        // Pin time-of-day (0x8000=noon, 0x4000=dawn, 0xC000=dusk, 0=midnight). Negative
        // releases the game clock. Accepts a raw u16 value.
        gSoH3dForceTime = (f1 < 0.0f) ? -1 : ((int)f1 & 0xFFFF);
        SoH3D_ReplReply(outPath, "time=%d (0x%04x)%s", gSoH3dForceTime, gSoH3dForceTime < 0 ? 0 : gSoH3dForceTime,
                        gSoH3dForceTime < 0 ? " (clock released)" : "");
    } else if (strcmp(cmd, "meshfloor") == 0 && sscanf(line, "%*s %f %f", &f1, &f2) == 2) {
        // Height of the OoT3D render mesh's floor at (x,z) for the room Link is in. After
        // the terrain warp this should match `floorat` (N64) on walkable ground.
        const char* sn = SoH3D_SceneName(play);
        int mid = (sn != NULL) ? SoH3D_RoomModelId(sn, play->roomCtx.curRoom.num) : -1;
        float my;
        if (mid >= 0 && SoH3D_RoomMeshFloorAt(mid, f1, f2, &my)) {
            SoH3D_ReplReply(outPath, "meshfloor (%.0f,%.0f) y=%.2f (room model %d)", f1, f2, my, mid);
        } else {
            SoH3D_ReplReply(outPath, "meshfloor (%.0f,%.0f) no hit (model %d)", f1, f2, mid);
        }
    } else if (strcmp(cmd, "scale") == 0 && sscanf(line, "%*s %63s %f", arg, &f1) == 2) {
        SoH3D_ModelEntry* e = SoH3D_FindModel(arg);
        if (e != NULL) {
            e->worldScale = f1;
            SoH3D_ReplReply(outPath, "scale %s=%.4f", e->name, e->worldScale);
        } else {
            SoH3D_ReplReply(outPath, "no model '%s'", arg);
        }
    } else if (strcmp(cmd, "yoff") == 0 && sscanf(line, "%*s %63s %f", arg, &f1) == 2) {
        SoH3D_ModelEntry* e = SoH3D_FindModel(arg);
        if (e != NULL) {
            e->groundOffset = f1;
            SoH3D_ReplReply(outPath, "yoff %s=%.1f", e->name, e->groundOffset);
        } else {
            SoH3D_ReplReply(outPath, "no model '%s'", arg);
        }
    } else if (strcmp(cmd, "spawn") == 0 && sscanf(line, "%*s %63s", arg) == 1) {
        SoH3D_ModelEntry* e = SoH3D_FindModel(arg);
        if (e != NULL) {
            Actor* a = SoH3D_SpawnInFront(play, e->actorId, 120.0f);
            SoH3D_ReplReply(outPath, "spawn %s -> %s", e->name, a != NULL ? "OK" : "FAILED (object not in scene)");
        } else {
            SoH3D_ReplReply(outPath, "no model '%s'", arg);
        }
    } else if (strcmp(cmd, "rotx") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dRotX = f1;
        SoH3D_ReplReply(outPath, "rot=(%.0f,%.0f,%.0f)", gSoH3dRotX, gSoH3dRotY, gSoH3dRotZ);
    } else if (strcmp(cmd, "roty") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dRotY = f1;
        SoH3D_ReplReply(outPath, "rot=(%.0f,%.0f,%.0f)", gSoH3dRotX, gSoH3dRotY, gSoH3dRotZ);
    } else if (strcmp(cmd, "rotz") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dRotZ = f1;
        SoH3D_ReplReply(outPath, "rot=(%.0f,%.0f,%.0f)", gSoH3dRotX, gSoH3dRotY, gSoH3dRotZ);
    } else if (strcmp(cmd, "animrate") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dAnimRate = f1;
        SoH3D_ReplReply(outPath, "animrate=%.3f frame=%.1f", gSoH3dAnimRate, gSoH3dAnimFrame);
    } else if (strcmp(cmd, "animframe") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dAnimFrame = f1;
        SoH3D_ReplReply(outPath, "animframe=%.1f (rate=%.3f)", gSoH3dAnimFrame, gSoH3dAnimRate);
    } else if (strcmp(cmd, "animlive") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dAnimLive = (int)f1;
        SoH3D_ReplReply(outPath, "animlive=%d (1=actor SkelAnime, 0=scrub animframe)", gSoH3dAnimLive);
    } else if (strcmp(cmd, "animdbg") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dAnimDebug = (int)f1;
        SoH3D_ReplReply(outPath, "animdbg=%d", gSoH3dAnimDebug);
    } else if (strcmp(cmd, "scenescale") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dSceneScale = f1;
        SoH3D_ReplReply(outPath, "scenescale=%.4f", gSoH3dSceneScale);
    } else if (strcmp(cmd, "sceneoff") == 0 && sscanf(line, "%*s %f %f %f", &f1, &f2, &f3) == 3) {
        gSoH3dSceneOffX = f1;
        gSoH3dSceneOffY = f2;
        gSoH3dSceneOffZ = f3;
        SoH3D_ReplReply(outPath, "sceneoff=(%.1f,%.1f,%.1f)", gSoH3dSceneOffX, gSoH3dSceneOffY, gSoH3dSceneOffZ);
    } else if (strcmp(cmd, "camfreeze") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        // Capture the current camera and hold it (1), or release back to the engine (0).
        if (f1 != 0.0f) {
            gSoH3dCamEye[0] = play->view.eye.x;
            gSoH3dCamEye[1] = play->view.eye.y;
            gSoH3dCamEye[2] = play->view.eye.z;
            gSoH3dCamAt[0] = play->view.lookAt.x;
            gSoH3dCamAt[1] = play->view.lookAt.y;
            gSoH3dCamAt[2] = play->view.lookAt.z;
            gSoH3dCamOverride = 1;
            SoH3D_ReplReply(outPath, "camfreeze ON eye=(%.0f,%.0f,%.0f) at=(%.0f,%.0f,%.0f)", gSoH3dCamEye[0],
                            gSoH3dCamEye[1], gSoH3dCamEye[2], gSoH3dCamAt[0], gSoH3dCamAt[1], gSoH3dCamAt[2]);
        } else {
            gSoH3dCamOverride = 0;
            SoH3D_ReplReply(outPath, "camfreeze OFF (camera returned to engine)");
        }
    } else if (strcmp(cmd, "cam") == 0) {
        // cam <eyeX eyeY eyeZ atX atY atZ> — set the frozen camera explicitly + hold it.
        float c[6];
        if (sscanf(line, "%*s %f %f %f %f %f %f", &c[0], &c[1], &c[2], &c[3], &c[4], &c[5]) == 6) {
            gSoH3dCamEye[0] = c[0];
            gSoH3dCamEye[1] = c[1];
            gSoH3dCamEye[2] = c[2];
            gSoH3dCamAt[0] = c[3];
            gSoH3dCamAt[1] = c[4];
            gSoH3dCamAt[2] = c[5];
            gSoH3dCamOverride = 1;
            SoH3D_ReplReply(outPath, "cam eye=(%.0f,%.0f,%.0f) at=(%.0f,%.0f,%.0f)", c[0], c[1], c[2], c[3], c[4],
                            c[5]);
        } else {
            SoH3D_ReplReply(outPath, "cam needs 6 floats: eyeX eyeY eyeZ atX atY atZ");
        }
    } else if (strcmp(cmd, "camorbit") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        // Rotate the frozen eye about the frozen `at` by f1 degrees around world +Y,
        // preserving radius and height. Auto-freezes from the live camera first if not
        // already held, so `camorbit 15` works without a prior `camfreeze 1`. This is
        // the parallax-sweep primitive: hold `at`, step the azimuth, dump at each step.
        float dx, dz, c, s, nx, nz, rad;
        if (!gSoH3dCamOverride) {
            gSoH3dCamEye[0] = play->view.eye.x;
            gSoH3dCamEye[1] = play->view.eye.y;
            gSoH3dCamEye[2] = play->view.eye.z;
            gSoH3dCamAt[0] = play->view.lookAt.x;
            gSoH3dCamAt[1] = play->view.lookAt.y;
            gSoH3dCamAt[2] = play->view.lookAt.z;
            gSoH3dCamOverride = 1;
        }
        dx = gSoH3dCamEye[0] - gSoH3dCamAt[0];
        dz = gSoH3dCamEye[2] - gSoH3dCamAt[2];
        c = cosf(f1 * (3.14159265f / 180.0f));
        s = sinf(f1 * (3.14159265f / 180.0f));
        nx = dx * c - dz * s;
        nz = dx * s + dz * c;
        gSoH3dCamEye[0] = gSoH3dCamAt[0] + nx;
        gSoH3dCamEye[2] = gSoH3dCamAt[2] + nz;
        rad = sqrtf(nx * nx + nz * nz);
        SoH3D_ReplReply(outPath, "camorbit %+.1fdeg eye=(%.0f,%.0f,%.0f) at=(%.0f,%.0f,%.0f) rad=%.0f", f1,
                        gSoH3dCamEye[0], gSoH3dCamEye[1], gSoH3dCamEye[2], gSoH3dCamAt[0], gSoH3dCamAt[1],
                        gSoH3dCamAt[2], rad);
    } else if (strcmp(cmd, "dump") == 0 && sscanf(line, "%*s %1023s", path) == 1) {
        strncpy(gSoh3dDumpPath, path, sizeof(gSoh3dDumpPath) - 1);
        gSoh3dDumpPath[sizeof(gSoh3dDumpPath) - 1] = '\0';
        gSoh3dDumpPending = 1;
        SoH3D_ReplReply(outPath, "dump -> %s (pending)", gSoh3dDumpPath);
    } else if (strcmp(cmd, "state") == 0) {
        u8 tint[3];
        char scales[256];
        s32 n = 0;
        s32 k;
        SoH3D_SceneTint(play, tint);
        for (k = 0; k < ARRAY_COUNT(sModelTable) && n < (s32)sizeof(scales) - 1; k++) {
            n += snprintf(scales + n, sizeof(scales) - n, "%s%s=%.4f(yoff %.0f)", k ? " " : "",
                          sModelTable[k].name, sModelTable[k].worldScale, sModelTable[k].groundOffset);
        }
        SoH3D_ReplReply(outPath, "enabled=%d diff=%.3f mul=%.3f tint=(%d,%d,%d) anim(live=%d frame=%.1f rate=%.3f) scale: %s",
                        SoH3D_Enabled(), gSoH3dTintDiff, gSoH3dTintMul, tint[0], tint[1], tint[2], gSoH3dAnimLive,
                        gSoH3dAnimFrame, gSoH3dAnimRate, scales);
    } else {
        SoH3D_ReplReply(outPath, "? '%s' (cmds: mul diff tint enable scale yoff rotx roty rotz animrate animframe animlive spawn cam camorbit camfreeze floorat floorgrid dump state)", line);
    }
}

void SoH3D_ReplPoll(PlayState* play) {
    static int fd = -2; // -2 uninit, -1 disabled
    static char outPath[1100];
    static char buf[8192];
    static int buflen = 0;
    char* start;
    char* nl;
    ssize_t n;

    // Force time-of-day (e.g. day instead of night). Applied every frame, before the
    // FIFO handling, so it holds regardless of whether the REPL is connected.
    SoH3D_InitForceTime();
    if (gSoH3dForceTime >= 0) {
        gSaveContext.dayTime = (u16)gSoH3dForceTime;
        gSaveContext.skyboxTime = (u16)gSoH3dForceTime;
    }

    if (fd == -2) {
        const char* p = getenv("SOH3D_REPL");
        if (p == NULL || p[0] == '\0') {
            fd = -1;
            return;
        }
        mkfifo(p, 0666); // ignore EEXIST
        fd = open(p, O_RDWR | O_NONBLOCK); // O_RDWR: we keep a writer so reads never EOF
        snprintf(outPath, sizeof(outPath), "%s.out", p);
        if (fd >= 0) {
            FILE* f = fopen(outPath, "w");
            if (f != NULL) {
                fprintf(f, "SOH3D REPL ready (fifo=%s)\n", p);
                fclose(f);
            }
        }
    }
    if (fd < 0) {
        return;
    }
    for (;;) {
        if (buflen >= (int)sizeof(buf) - 1) {
            buflen = 0; // overflow guard: drop garbage
        }
        n = read(fd, buf + buflen, sizeof(buf) - 1 - buflen);
        if (n <= 0) {
            break;
        }
        buflen += (int)n;
    }
    buf[buflen] = '\0';
    start = buf;
    while ((nl = strchr(start, '\n')) != NULL) {
        *nl = '\0';
        SoH3D_ReplExec(play, start, outPath);
        start = nl + 1;
    }
    buflen = (int)strlen(start);
    memmove(buf, start, buflen + 1);

    // Hold the diagnostic camera: re-apply every frame so the engine's per-update
    // recompute doesn't reclaim it. up is forced to world +Y (an orbit never rolls).
    if (gSoH3dCamOverride) {
        play->view.eye.x = gSoH3dCamEye[0];
        play->view.eye.y = gSoH3dCamEye[1];
        play->view.eye.z = gSoH3dCamEye[2];
        play->view.lookAt.x = gSoH3dCamAt[0];
        play->view.lookAt.y = gSoH3dCamAt[1];
        play->view.lookAt.z = gSoH3dCamAt[2];
        play->view.up.x = 0.0f;
        play->view.up.y = 1.0f;
        play->view.up.z = 0.0f;
    }
}
