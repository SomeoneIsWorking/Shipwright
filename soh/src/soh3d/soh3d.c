// SoH3D runtime toggle + helpers. See repo-root PROGRESS.md.
#include "soh3d.h"
#include "soh3d_collision.h" // C-ABI bridge for OoT3D scene collision (soh3d_model.cpp)
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
// LIVE anim-compare tooling: REPL `animforce <csab-base>` pins that CSAB on replaced actors (empty
// = auto-resolve); `animlist` prints the CSABs of the last replaced model (gSoH3dLastAutoModel).
char gSoH3dForceCsab[64] = "";
int gSoH3dLastAutoModel = -1;

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
void SoH3D_GL_FrameBegin(void); // drop any SoH3D draws left unrendered from a prior frame
void SoH3D_GL_SetLightDir(const float dirWorld[3]); // scene sun dir (world space) for the form term
void SoH3D_GL_EmitPose(int modelId); // snapshot this actor's pose at emit time (per-item skinning)
void SoH3D_UpdateAnim(int modelId, const char* animName, float frame);
// Retarget a live N64 SkelAnime pose onto the OoT3D skeleton (GPU skinning). jointRots =
// &jointTable[1] (per-limb binang Vec3s; root translation jointTable[0] is skipped),
// rotCount = limbCount. See soh3d_model.cpp. The OoT3D model must share the N64 rig order.
void SoH3D_UpdateAnimN64(int modelId, const s16* jointRots, int rotCount);

// 1 = drive replaced skinned characters from their live N64 SkelAnime joints (port N64
// animations onto the OoT3D skeleton) instead of a CSAB. Env SOH3D_N64ANIM (default OFF —
// WIP, see SoH3D_N64AnimEnabled) + REPL `n64anim`. The CSAB path stays available for A/B.
int gSoH3dN64Anim = -1;

// N64-anim deferral state. When SoH3D_TryDrawActor sees an n64anim-flagged actor (and
// SOH3D_N64ANIM is on) it records the actor + its OoT3D model here and returns 0, letting
// the actor's own Draw run; the SkelAnime_Draw hook (SoH3D_SkelAnimeDraw) then retargets the
// OoT3D model from the live jointTable and skips the N64 limb draw. Cleared in
// SoH3D_AfterActorDraw. gSoH3dPendingModel = -1 means no pending replacement this actor.
static Actor* gSoH3dPendingActor = NULL;
static int gSoH3dPendingModel = -1;
static float gSoH3dPendingScale = 1.0f;
static float gSoH3dPendingGroundOff = 0.0f;
static int gSoH3dPendingAuto = 0; // 1 = auto-replaced (apply the rig-mismatch guard); 0 = hand-verified table entry

// Get-or-allocate a scene-room model id (soh3d_model.cpp). Keyed by ZSI path; loads
// the embedded room CMB lazily on first draw. Returns -1 for an unmapped scene.
int SoH3D_RoomModelId(const char* sceneName, int roomNum);
// Auto-replace path (soh3d_model.cpp): get-or-allocate a GL model id for an actor ZAR
// (keyed by path), and the OoT3D model's local bbox diagonal (for auto-scale).
int SoH3D_AutoModelId(const char* zarPath);
float SoH3D_AutoModelHeight(int modelId);
float SoH3D_AutoModelMinY(int modelId);
int SoH3D_AutoModelSkinned(int modelId);
int SoH3D_AutoModelBoneCount(int modelId);
float SoH3D_AutoModelBoneLenSum(int modelId); // Σ|trans| of non-root OoT3D bones (skeleton size)
const char* SoH3D_AutoModelDefaultAnim(int modelId);     // default (idle) OoT3D CSAB base name
void SoH3D_UpdateAnimAuto(int modelId, const char* animName, float rate, float n64CurFrame,
                          float n64AnimLength); // play OoT3D's own CSAB, phase-locked to the N64 anim
void SoH3D_DumpModelBones(int modelId); // oracle: print OoT3D skeleton (gated by caller)

// SoH sceneNum -> OoT3D scene folder name (defined below).
static const char* SoH3D_SceneName(PlayState* play);

// SoH sceneNum -> OoT3D scene folder name (kSoH3dSceneNames). Generated, names only.
#include "soh3d_scene_names.inc"
// N64 object id -> OoT3D actor ZAR path (kSoH3dObjectZars). Generated, paths only.
#include "soh3d_object_zars.inc"
// Per-character N64<->OoT3D bone correspondence + scale (kSoH3dBoneMaps). Generated offline by
// tools/soh3d_skel_export.py; used by the SkelAnime retarget for topology-divergent rigs.
#include "soh3d_bonemap.inc"

// Retarget the OoT3D skeleton from a live N64 pose with an explicit bone correspondence (NULL
// map = identity). Defined in soh3d_model.cpp.
void SoH3D_UpdateAnimN64Mapped(int modelId, const s16* jointRots, int rotCount, const signed char* boneToLimb,
                               int mapCount);

// Find the precomputed bone map for a ZAR path, or NULL if none (-> identity retarget + runtime
// rest-pose scale).
static const SoH3DBoneMap* SoH3D_FindBoneMap(const char* zar) {
    if (zar == NULL) {
        return NULL;
    }
    for (s32 i = 0; i < (s32)ARRAY_COUNT(kSoH3dBoneMaps); i++) {
        if (strcmp(kSoH3dBoneMaps[i].zar, zar) == 0) {
            return &kSoH3dBoneMaps[i];
        }
    }
    return NULL;
}

// Precomputed bone map for the actor currently deferred for N64-anim replacement (NULL = none ->
// identity retarget). Set alongside gSoH3dPendingModel when an auto actor is deferred.
static const SoH3DBoneMap* gSoH3dPendingBoneMap = NULL;

// Per-character N64-animation -> OoT3D-CSAB map (kSoH3dAnimMaps). Lets an AUTO skinned actor play
// the CSAB corresponding to whatever animation the N64 game logic is running (walk->walk, talk->
// talk), instead of a single fixed idle. Hand-maintained; seeded by tools/soh3d_anim_export.py.
#include "soh3d_animmap.inc"

// Resolve the actor's LIVE N64 animation (skelAnime->animation, an OTR path string in SoH) to the
// CSAB base it maps to, or NULL if unlisted (-> caller falls back to the default idle). The runtime
// string carries an "__OTR__" prefix the map keys omit, so skip it before the strcmp.
static const char* SoH3D_ResolveAutoCsab(const char* n64AnimOtr) {
    if (n64AnimOtr == NULL) {
        return NULL;
    }
    if (strncmp(n64AnimOtr, "__OTR__", 7) == 0) {
        n64AnimOtr += 7;
    }
    for (s32 i = 0; i < (s32)ARRAY_COUNT(kSoH3dAnimMaps); i++) {
        if (strcmp(kSoH3dAnimMaps[i].n64otr, n64AnimOtr) == 0) {
            return kSoH3dAnimMaps[i].csab;
        }
    }
    return NULL;
}

// Live N64 animation OTR path for the actor currently deferred for auto replacement. Reset per
// actor in SoH3D_TryDrawActor, captured by the SkelAnime-bearing choke points (SoH3D_SkelAnimeDraw
// and func_80034BA0/CC4 via SoH3D_SetCurAnim), consumed by the auto branch of SoH3D_DoRetarget.
// NULL -> no live anim known (default idle).
static const char* gSoH3dPendingAnimOtr = NULL;

// Live N64 animation playhead (curFrame) + length for the actor deferred for auto replacement,
// captured from the SkelAnime in SoH3D_SkelAnimeDraw. Lets the auto branch phase-lock the OoT3D
// CSAB to the N64 anim's actual progress (fixes "OoT3D anims too fast"). The raw (SkelAnime-less)
// choke point has no playhead -> animLength stays 0 -> free-run.
static float gSoH3dPendingN64CurFrame = 0.0f;
static float gSoH3dPendingN64AnimLength = 0.0f;

void SoH3D_SetCurAnim(void* animation) {
    if (gSoH3dAnimDebug) {
        static int dbg = 0;
        if ((dbg++ % 60) == 0) {
            fprintf(stderr, "[SetCurAnim] pendingModel=%d anim=%s\n", gSoH3dPendingModel,
                    animation ? (const char*)animation : "(null)");
            fflush(stderr);
        }
    }
    if (gSoH3dPendingModel >= 0) { // only meaningful while an actor is deferred for replacement
        gSoH3dPendingAnimOtr = (const char*)animation;
    }
}

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

// Apply the forced time-of-day to the save context NOW. Called from Play_Init BEFORE the
// scene's day/night setup layer is chosen (and its actor set spawned): pinning dayTime only
// per-frame in SoH3D_ReplPoll is too late — the scene already loaded the wrong (e.g. night)
// NPC set, which the actors lock in at Init. Forcing it here makes the INITIAL load match the
// clock (day NPCs for SOH3D_TIME=0x8000), and the per-frame pin keeps it there afterward.
void SoH3D_ApplyForceTime(void) {
    SoH3D_InitForceTime();
    if (gSoH3dForceTime >= 0) {
        gSaveContext.dayTime = (u16)gSoH3dForceTime;
        gSaveContext.skyboxTime = (u16)gSoH3dForceTime;
    }
}

static int SoH3D_TerrainWarpEnabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char* v = getenv("SOH3D_TERRAIN_WARP");
        cached = (v != NULL && v[0] == '0') ? 0 : 1; // default ON
    }
    // The per-actor render Y-offset and OoT3D collision are mutually exclusive fixes for the
    // same problem: once Link walks the OoT3D collision (== render) ground, offsetting actors
    // onto the render floor would double-correct. Collision wins.
    return cached && gSoH3dTerrainWarp && !SoH3D_CollisionEnabled();
}

// --- OoT3D collision: drive gameplay (BgCheck floors/walls) from the OoT3D scene collision
// mesh so Link physically walks the OoT3D world (see PROGRESS.md "USE OoT3D COLLISION"). The
// render mesh and the collision are then ONE geometry — fixes both floor height AND walls,
// which no render-side Y-offset can. SoH3D_BuildSceneCollision converts the parsed OoT3D
// collision into a SoH CollisionHeader; Scene_CommandCollisionHeader installs it instead of
// the N64 one. Gate: SoH3D_Enabled() + env SOH3D_COLLISION (default ON; =0 for A/B) + REPL
// `collision` (takes effect on next scene load / warp). ---
int gSoH3dCollision = 1;

int SoH3D_CollisionEnabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char* v = getenv("SOH3D_COLLISION");
        cached = (v != NULL && v[0] == '0') ? 0 : 1; // default ON
    }
    return SoH3D_Enabled() && cached && gSoH3dCollision;
}

// Build a SoH CollisionHeader from the current scene's OoT3D collision, or NULL when
// disabled / unavailable (caller then uses the N64 collision). The header + its arrays are
// malloc'd and kept resident for the scene lifetime; the previous build is freed here, so a
// scene change / warp recycles it (BgCheck_Allocate stores the pointer and references the
// arrays, so they must outlive the call). Verts are N64-unit world-space (same frame as the
// render mesh), so no transform — direct copy. One generic SurfaceType (plain ground) backs
// all polys; floor/wall/ceiling classification comes from each poly's normal, not the type.
CollisionHeader* SoH3D_BuildSceneCollision(PlayState* play, CollisionHeader* n64) {
    static CollisionHeader* sHeader = NULL;
    static SurfaceType* sSurfaceTypes = NULL;
    static CamData* sCamData = NULL;
    static WaterBox* sWaterBoxes = NULL;
    const char* sceneName;
    SoH3D_RawCollision raw;
    CollisionHeader* h;
    Vec3s* vtx;
    CollisionPoly* poly;
    int i;
    int nSurf;         // surfaceType entries allocated (>=1)
    s16 minX, minY, minZ, maxX, maxY, maxZ;
    size_t camLen = 1; // entries in sCamData (>=1: a dummy when no N64 list)

    if (play == NULL || !SoH3D_CollisionEnabled()) {
        return NULL;
    }
    sceneName = SoH3D_SceneName(play);
    if (sceneName == NULL) {
        return NULL; // scene has no OoT3D mapping -> N64 collision
    }
    if (!SoH3D_LoadSceneCollisionRaw(sceneName, &raw)) {
        return NULL;
    }

    // Free the previous scene's build (its arrays were referenced by the old colCtx, which is
    // being replaced now).
    if (sHeader != NULL) {
        free(sHeader->vtxList);
        free(sHeader->polyList);
        free(sHeader);
    }
    free(sSurfaceTypes);
    free(sCamData);
    free(sWaterBoxes);
    sCamData = NULL;
    sWaterBoxes = NULL;

    nSurf = (raw.numSurf > 0) ? raw.numSurf : 1;
    h = (CollisionHeader*)calloc(1, sizeof(CollisionHeader));
    vtx = (Vec3s*)malloc(sizeof(Vec3s) * raw.numVerts);
    poly = (CollisionPoly*)calloc(raw.numPolys, sizeof(CollisionPoly));
    sSurfaceTypes = (SurfaceType*)calloc(nSurf, sizeof(SurfaceType));
    if (h == NULL || vtx == NULL || poly == NULL || sSurfaceTypes == NULL) {
        free(h); free(vtx); free(poly); free(sSurfaceTypes);
        sHeader = NULL; sSurfaceTypes = NULL;
        SoH3D_FreeRawCollision(&raw);
        return NULL;
    }

    // SurfaceTypes carry the per-poly gameplay semantics: data[0] low byte = camDataIndex (which
    // camera region), (data[0]>>8)&0x1F = scene EXIT index (how Link leaves the area), plus
    // floor/wall flags; data[1] = floor type / material. OoT3D uses the SAME bitfield layout as
    // N64 (same game), and its cam/exit indices line up with the N64 cameraDataList / exit list
    // SoH loads (same scenes) — so copy them verbatim. This is what makes exits, per-region
    // cameras, and special floors work (vs the earlier single generic type that broke exits +
    // forced one camera scene-wide).
    for (i = 0; i < raw.numSurf; i++) {
        sSurfaceTypes[i].data[0] = raw.surf0[i];
        sSurfaceTypes[i].data[1] = raw.surf1[i];
    }

    // Waterboxes + camera REGION data are gameplay volumes actors index + DEREFERENCE (e.g.
    // Bg_Spot01_Idomizu writes waterBoxes[0].ySurface -> NULL crash if absent; the surfaceType cam
    // index selects cameraDataList[idx]). We have not REd those OoT3D sub-lists yet; copy them from
    // the N64 header (same world space + same indices since same game). The CamData entries keep
    // their N64 camPosData pointers (valid for the scene), so fixed/pivot cameras still work.
    if (n64 != NULL && n64->numWaterBoxes > 0 && n64->waterBoxes != NULL) {
        sWaterBoxes = (WaterBox*)malloc(sizeof(WaterBox) * n64->numWaterBoxes);
        if (sWaterBoxes != NULL) {
            memcpy(sWaterBoxes, n64->waterBoxes, sizeof(WaterBox) * n64->numWaterBoxes);
        }
    }
    if (n64 != NULL && n64->cameraDataList != NULL && n64->cameraDataListLen > 0) {
        sCamData = (CamData*)malloc(sizeof(CamData) * n64->cameraDataListLen);
        if (sCamData != NULL) {
            memcpy(sCamData, n64->cameraDataList, sizeof(CamData) * n64->cameraDataListLen);
            camLen = n64->cameraDataListLen;
        }
    }
    if (sCamData == NULL) {
        sCamData = (CamData*)calloc(1, sizeof(CamData)); // fallback: CAM_SET_NORMAL0 follow cam
        sCamData[0].cameraSType = CAM_SET_NORMAL0;
        camLen = 1;
    }

    minX = maxX = raw.verts[0]; minY = maxY = raw.verts[1]; minZ = maxZ = raw.verts[2];
    for (i = 0; i < raw.numVerts; i++) {
        s16 x = raw.verts[i * 3 + 0], y = raw.verts[i * 3 + 1], z = raw.verts[i * 3 + 2];
        vtx[i].x = x; vtx[i].y = y; vtx[i].z = z;
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }
    for (i = 0; i < raw.numPolys; i++) {
        u16 ty = raw.polyType[i];
        poly[i].type = (ty < (u16)raw.numSurf) ? ty : 0; // index into surfaceTypeList
        poly[i].flags_vIA = raw.polyVtx[i * 3 + 0] & 0x1FFF;
        poly[i].flags_vIB = raw.polyVtx[i * 3 + 1] & 0x1FFF;
        poly[i].vIC = raw.polyVtx[i * 3 + 2] & 0x1FFF;
        poly[i].normal.x = raw.polyNrm[i * 3 + 0];
        poly[i].normal.y = raw.polyNrm[i * 3 + 1];
        poly[i].normal.z = raw.polyNrm[i * 3 + 2];
        // SoH plane: normal.p + dist == 0; OoT3D plane: n.p == -dist -> SoH dist == OoT3D dist.
        poly[i].dist = (s16)lrintf(raw.polyDist[i]);
    }

    h->minBounds.x = minX; h->minBounds.y = minY; h->minBounds.z = minZ;
    h->maxBounds.x = maxX; h->maxBounds.y = maxY; h->maxBounds.z = maxZ;
    h->numVertices = (u16)raw.numVerts;
    h->vtxList = vtx;
    h->numPolygons = (u16)raw.numPolys;
    h->polyList = poly;
    h->surfaceTypeList = sSurfaceTypes;
    h->cameraDataList = sCamData;
    h->cameraDataListLen = camLen;
    h->numWaterBoxes = (sWaterBoxes != NULL && n64 != NULL) ? n64->numWaterBoxes : 0;
    h->waterBoxes = sWaterBoxes;

    SoH3D_FreeRawCollision(&raw);
    sHeader = h;
    return h;
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

// Resolve the actor's live N64 SkelAnime pose for the N64-animation port path. On success
// returns 1 and sets *outJointRots = &jointTable[1] (per-limb binang rotations; the root
// translation jointTable[0] is skipped) and *outLimbCount = the limb count. Per-actor (the
// SkelAnime sits at an actor-specific struct offset). NULL/return 0 -> no N64 joints.
typedef int (*SoH3D_JointResolver)(Actor* actor, const s16** outJointRots, int* outLimbCount);

static void SoH3D_DrawModelGL(PlayState* play, int modelId, Actor* actor, float worldScale,
                              const char* animName, float groundOffset, SoH3D_AnimResolver resolveAnim,
                              SoH3D_JointResolver resolveJoints);

static int SoH3D_N64AnimEnabled(void) {
    if (gSoH3dN64Anim < 0) {
        const char* v = getenv("SOH3D_N64ANIM");
        // Default OFF: the N64-joint retarget math is WIP (pose still contorted — the per-limb
        // rotation convention needs work), so calibrated characters keep the working CSAB path
        // until it's correct. Opt in with SOH3D_N64ANIM=1 / REPL `n64anim 1`.
        gSoH3dN64Anim = (v != NULL && v[0] == '1') ? 1 : 0;
    }
    return gSoH3dN64Anim;
}

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
// Emit the OoT3D model draw at an actor's world position/yaw/scale (+ground offset) into
// POLY_OPA. Assumes the model's GPU pose (skin matrices) was already set this frame (via
// SoH3D_UpdateAnim or SoH3D_UpdateAnimN64). Shared by the table/auto draw path and the
// generic N64-anim SkelAnime hook.
static void SoH3D_EmitModelDraw(PlayState* play, int modelId, Actor* actor, float worldScale,
                                float groundOffset) {
    u8 tint[3];
    OPEN_DISPS(play->state.gfxCtx);
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
    // Snapshot this actor's pose NOW (its SkelAnime/CSAB pose was just set via SoH3D_UpdateAnim*),
    // before a later same-model actor overwrites the per-model bone store; the deferred draw is
    // interpreted long after build, so per-item pose must be captured here. See SoH3D_GL_EmitPose.
    SoH3D_GL_EmitPose(modelId);
    // High bit of the handle = "lit": apply the half-Lambert FORM term. Characters/props carry no
    // baked vertex lighting, so without this they render flat; scene rooms (other emit site) keep
    // their bit clear so their baked vColor AO isn't double-shaded.
    gSPSoH3DDraw(POLY_OPA_DISP++, modelId | (int)0x80000000, tint[0], tint[1], tint[2]);
    CLOSE_DISPS(play->state.gfxCtx);
}

static void SoH3D_DrawModelGL(PlayState* play, int modelId, Actor* actor, float worldScale,
                              const char* animName, float groundOffset, SoH3D_AnimResolver resolveAnim,
                              SoH3D_JointResolver resolveJoints) {
    SoH3D_EnsureModelProvider();
    // N64-animation port: drive the OoT3D skeleton straight from the actor's live N64
    // SkelAnime joints (the pose the game logic computed this frame), so the replacement
    // animates with the SAME animation the N64 actor plays — no per-actor CSAB mapping.
    // Wins over the CSAB path when enabled and the actor exposes its joints.
    if (SoH3D_N64AnimEnabled() && gSoH3dAnimLive && resolveJoints != NULL) {
        const s16* jointRots = NULL;
        int limbCount = 0;
        if (resolveJoints(actor, &jointRots, &limbCount) && jointRots != NULL && limbCount > 0) {
            SoH3D_UpdateAnimN64(modelId, jointRots, limbCount);
            goto draw; // pose set from N64 joints; skip the CSAB path
        }
    }
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
draw:
    SoH3D_EmitModelDraw(play, modelId, actor, worldScale, groundOffset);
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

// En_Ge1 joints for the N64-animation port: hand back &jointTable[1] (per-limb binang
// rotations; skip jointTable[0] root translation) + limbCount. The OoT3D geldwoman skeleton
// is the SAME rig as N64 En_Ge1 (15 limbs, same order: WAIST, L/R legs ×3, TORSO, L/R arms
// ×3, HEAD), so OoT3D bone i == N64 limb (i+1) and SoH3D_UpdateAnimN64 maps bone i <- rots[i].
static int SoH3D_Joints_EnGe1(Actor* actor, const s16** outJointRots, int* outLimbCount) {
    EnGe1* ge = (EnGe1*)actor;
    if (ge->skelAnime.jointTable == NULL || ge->skelAnime.limbCount <= 0) {
        return 0;
    }
    *outJointRots = (const s16*)&ge->skelAnime.jointTable[1]; // [0] = root translation, skip it
    *outLimbCount = ge->skelAnime.limbCount;
    return 1;
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
    SoH3D_JointResolver resolveJoints; // NULL = no N64-joint port (use CSAB path). Legacy
                                       // per-actor accessor; superseded by `n64anim` below.
    int n64anim; // 1 = drive this model's OoT3D skeleton from the actor's LIVE N64 SkelAnime
                 // joints via the generic SkelAnime_Draw hook (SoH3D_SkelAnimeDraw) when
                 // SOH3D_N64ANIM is on — no per-actor jointTable accessor needed. The OoT3D
                 // rig must correspond to the N64 one (bone i <- jointTable[i+1]); verify per
                 // actor before setting. 0 = use the CSAB path (resolveAnim).
} SoH3D_ModelEntry;

// Non-const so the REPL can tune worldScale/groundOffset live.
static SoH3D_ModelEntry sModelTable[] = {
    { ACTOR_OBJ_TSUBO, "pot", soh3d_pot_model_dl, SOH3D_POT_WORLD_SCALE, 3, NULL, 0.0f, NULL, NULL, 0 },
    { ACTOR_EN_GS, "gs", soh3d_gs_model_dl, SOH3D_GS_WORLD_SCALE, -1, NULL, 0.0f, NULL, NULL, 0 },
    { ACTOR_OBJ_KIBAKO2, "kibako", soh3d_kibako_model_dl, SOH3D_KIBAKO_WORLD_SCALE, 1, NULL, 0.0f, NULL, NULL, 0 },
    { ACTOR_EN_KUSA, "kusa", NULL, 0.5f, 2, NULL, 0.0f, NULL, NULL, 0 }, // bush (scale tuned live via REPL)
    { ACTOR_EN_GE1, "geldwoman", soh3d_geldwoman_model_dl, SOH3D_GELDWOMAN_WORLD_SCALE, 0, "ge1_s_wait",
      SOH3D_GELDWOMAN_GROUND_OFFSET, SoH3D_ResolveAnim_EnGe1, SoH3D_Joints_EnGe1, 1 },
};

// ===========================================================================
// SOH3D_AUTO — programmatic actor replacement with auto-scale.
//
// Instead of hand-listing every actor, an actor whose loaded object has a matching
// OoT3D ZAR (kSoH3dObjectZars[objectId]) is replaced by that ZAR's main model, drawn
// via the same direct-GL path. The world scale is NOT a magic constant: it is MEASURED
// per object. The first time such an actor is seen, we let its N64 model draw and bracket
// that draw with the OTR_G_SOH3D_MEASURE opcode; the interpreter accumulates the actor's
// eye-space (== world-space) bbox and reports its diagonal back via SoH3D_MeasureResult.
// scale = measured_N64_world_diag / OoT3D_model_local_diag. The next frame the OoT3D
// model draws at that scale. Explicit sModelTable entries always win (they carry
// calibrated scale + anim resolvers) unless SOH3D_AUTO=2 (validation: route ALL through
// the auto path so the derived scale can be checked against the hand-tuned values).
//
// Gated behind env SOH3D_AUTO (0=off default, 1=fill non-table actors, 2=auto for ALL)
// + REPL `auto`. Static props only animate correctly (no skeleton); skinned characters
// come out in bind pose (frozen) — acceptable per the session-13 plan; sModelTable still
// drives the calibrated/animated ones at AUTO=1.
// ===========================================================================
int gSoH3dAuto = -1; // -1 = uninit (read env), 0=off, 1=fill, 2=all (validation)

static int SoH3D_AutoMode(void) {
    if (gSoH3dAuto < 0) {
        const char* v = getenv("SOH3D_AUTO");
        gSoH3dAuto = (v != NULL && v[0] != '\0') ? atoi(v) : 0;
    }
    return gSoH3dAuto;
}

// Per-object auto-replace cache, indexed by object id.
//   state: 0 unseen, 1 measuring (bracket emitted, awaiting result), 2 ready, 3 failed
typedef struct {
    float measuredH; // N64 world-space height from the measure pass (0 = none yet)
    float scale;     // derived worldScale (valid when state==2)
    float groundOff; // model-local groundOffset (= -bind-pose minY) for skinned actors
    int modelId;     // allocated GL model id (0 = not yet allocated; ids are >= 2000)
    signed char state;
    signed char tries;   // measure attempts (cap so a never-drawn actor doesn't loop forever)
    signed char skinned; // 1 = articulated -> drive via the generic N64-anim SkelAnime hook
} SoH3D_AutoEntry;
static SoH3D_AutoEntry sAuto[ARRAY_COUNT(kSoH3dObjectZars)];
static int sPendingMeasureKey = -1; // object id whose measure bracket is open this draw

// Interpreter callback (libultraship): the measure bracket closed for `key` (object id)
// with the actor's measured world-space bbox diagonal. Store it; the scale is derived
// lazily in SoH3D_TryDrawActor next frame (needs the OoT3D model diagonal, loaded there).
void SoH3D_MeasureResult(int key, float height) {
    if (key >= 0 && key < (int)ARRAY_COUNT(sAuto)) {
        sAuto[key].measuredH = height;
    }
}

// Emit a measure bracket opcode (begin/end) into POLY_OPA around an actor's N64 draw.
static void SoH3D_EmitMeasure(PlayState* play, int key, int begin) {
    OPEN_DISPS(play->state.gfxCtx);
    gSPSoH3DMeasure(POLY_OPA_DISP++, key, begin);
    CLOSE_DISPS(play->state.gfxCtx);
}

// The object id an actor's geometry depends on (its loaded object bank slot), or -1.
static int SoH3D_ActorObjectId(PlayState* play, Actor* actor) {
    s8 idx = actor->objBankIndex;
    if (idx < 0 || idx >= play->objectCtx.num) {
        return -1;
    }
    return play->objectCtx.status[idx].id;
}

// Try the SOH3D_AUTO path for an actor with no explicit sModelTable entry. Returns 1 if
// it drew the OoT3D model (caller skips N64), 0 to let the N64 model draw (possibly while
// measuring it this frame). mode is SoH3D_AutoMode() (>=1).
static int SoH3D_TryAuto(PlayState* play, Actor* actor) {
    int objId = SoH3D_ActorObjectId(play, actor);
    SoH3D_AutoEntry* e;
    const char* zar;
    if (objId < 0 || objId >= (int)ARRAY_COUNT(kSoH3dObjectZars)) {
        return 0;
    }
    zar = kSoH3dObjectZars[objId];
    if (zar == NULL) {
        return 0; // no OoT3D model for this object -> N64
    }
    // OBJECT_KANBAN (signpost) stays on N64. The assembly-merge can render the intact sign, but
    // En_Kanban's CUT behaviour spawns more En_Kanban actors for the broken pieces — those get
    // auto-replaced as whole signs again, so slashing a sign "spawns more signs" instead of
    // breaking. Until the break pieces are handled, keep the faithful N64 sign (it breaks right).
    if (objId == OBJECT_KANBAN) {
        return 0;
    }
    e = &sAuto[objId];
    if (e->state == 3) {
        return 0; // known-unreplaceable -> N64
    }
    if (e->modelId == 0) {
        e->modelId = SoH3D_AutoModelId(zar);
        if (e->modelId < 0) {
            e->state = 3;
            return 0;
        }
        // Skinned characters: drive the OoT3D skeleton from the actor's LIVE N64 SkelAnime
        // joints via the generic SkelAnime hook (same mechanism as the calibrated sModelTable
        // n64anim entries). Requires SOH3D_N64ANIM; otherwise a frozen bind pose looks like a
        // T-pose, so skip -> N64. Grezzo mostly preserved the rigs (bone i <-> jointTable[i+1]),
        // so this broadly works; characters whose rig doesn't correspond will pose wrong (add a
        // per-objId skip if one shows up).
        if (SoH3D_AutoModelSkinned(e->modelId)) {
            if (!SoH3D_N64AnimEnabled() || !gSoH3dAnimLive) {
                e->state = 3;
                return 0;
            }
            e->skinned = 1;
            e->groundOff = -SoH3D_AutoModelMinY(e->modelId); // feet -> actor world Y
            // Skinned scale is derived from the rest skeletons (bone-length ratio) in the
            // SkelAnime hook — NOT the bbox measure, which over-measures articulated actors and
            // made them giant (Boj: measured n64h~1235 -> scale 0.18; the true scale is ~0.0102).
            // Go straight to ready; the hook computes the real scale and retargets the pose.
            e->state = 2;
        }
    }
    if (e->state == 2) {
        if (e->skinned) {
            // Defer to the actor's own Draw so the SkelAnime hook retargets the OoT3D skeleton
            // from the live N64 jointTable (returns 0 -> actor->draw runs; the hook draws it).
            gSoH3dPendingActor = actor;
            gSoH3dPendingModel = e->modelId;
            gSoH3dPendingScale = e->scale;
            gSoH3dPendingGroundOff = e->groundOff;
            gSoH3dPendingAuto = 1;
            gSoH3dPendingBoneMap = SoH3D_FindBoneMap(zar); // precomputed correspondence (or NULL)
            return 0;
        }
        // Ready static prop: draw the OoT3D model at the measured scale. No anim.
        SoH3D_DrawModelGL(play, e->modelId, actor, e->scale, NULL, 0.0f, NULL, NULL);
        return 1;
    }
    // state 0 or 1: derive scale if the measurement has arrived, else (re)measure.
    if (e->measuredH > 0.0f) {
        float modelH = SoH3D_AutoModelHeight(e->modelId);
        if (modelH > 1e-3f) {
            e->scale = e->measuredH / modelH;
            e->state = 2;
            if (SoH3D_AutoMode() >= 1) {
                printf("SOH3D AUTO: obj 0x%x %s -> scale=%.5f (n64h=%.1f modelh=%.1f)%s\n", objId, zar, e->scale,
                       e->measuredH, modelH, e->skinned ? " [n64anim]" : "");
                fflush(stdout);
            }
            if (e->skinned) {
                // Defer to the SkelAnime hook (drive the OoT3D skeleton from live N64 joints).
                gSoH3dPendingActor = actor;
                gSoH3dPendingModel = e->modelId;
                gSoH3dPendingScale = e->scale;
                gSoH3dPendingGroundOff = e->groundOff;
                gSoH3dPendingAuto = 1;
                return 0;
            }
            SoH3D_DrawModelGL(play, e->modelId, actor, e->scale, NULL, 0.0f, NULL, NULL);
            return 1;
        }
        e->state = 3; // model has no geometry -> cannot scale -> N64
        return 0;
    }
    // Need a measurement: bracket this actor's N64 draw (begin here, end in AfterActorDraw).
    if (e->tries >= 8) {
        e->state = 3; // never produced a measurement (always culled / off-screen) -> give up
        return 0;
    }
    e->tries++;
    e->state = 1;
    SoH3D_EmitMeasure(play, objId, /*begin=*/1);
    sPendingMeasureKey = objId;
    return 0; // let the N64 model draw so it can be measured
}

int SoH3D_TryDrawActor(PlayState* play, Actor* actor) {
    s32 i;
    if (!SoH3D_Enabled()) {
        return 0;
    }
    // Per-actor reset of the live-anim capture: this is the single entry consulted once for every
    // actor, before its own Draw runs the SkelAnime choke points that record the current anim.
    gSoH3dPendingAnimOtr = NULL;
    // Reset the N64 playhead too: if only the SkelAnime-less raw choke point fires for this actor,
    // animLength stays 0 -> the auto branch free-runs (no stale phase-lock from a prior actor).
    gSoH3dPendingN64CurFrame = 0.0f;
    gSoH3dPendingN64AnimLength = 0.0f;
    // Explicit table wins (calibrated scale + anim resolvers), unless validation mode (=2)
    // routes everything through the auto path to check the derived scale.
    if (SoH3D_AutoMode() != 2) {
        for (i = 0; i < ARRAY_COUNT(sModelTable); i++) {
            if (sModelTable[i].actorId == actor->id) {
                // N64-anim path: defer to the actor's own Draw so the generic SkelAnime hook
                // (SoH3D_SkelAnimeDraw) can grab the live jointTable and retarget the OoT3D
                // skeleton. Record the pending replacement; return 0 so actor->draw runs.
                if (sModelTable[i].glModelId >= 0 && sModelTable[i].n64anim && SoH3D_N64AnimEnabled() &&
                    gSoH3dAnimLive) {
                    gSoH3dPendingActor = actor;
                    gSoH3dPendingModel = sModelTable[i].glModelId;
                    gSoH3dPendingScale = sModelTable[i].worldScale;
                    gSoH3dPendingGroundOff = sModelTable[i].groundOffset;
                    gSoH3dPendingAuto = 0; // hand-verified entry -> skip the rig-mismatch guard
                    gSoH3dPendingBoneMap = NULL; // hand-calibrated entries use the identity retarget
                    return 0;
                }
                if (sModelTable[i].glModelId >= 0) {
                    SoH3D_DrawModelGL(play, sModelTable[i].glModelId, actor, sModelTable[i].worldScale,
                                      sModelTable[i].anim, sModelTable[i].groundOffset, sModelTable[i].resolveAnim,
                                      sModelTable[i].resolveJoints);
                } else {
                    SoH3D_DrawModel(play, sModelTable[i].dlist, actor, sModelTable[i].worldScale);
                }
                return 1;
            }
        }
    }
    if (SoH3D_AutoMode() >= 1) {
        return SoH3D_TryAuto(play, actor);
    }
    return 0;
}

// Walk a live N64 skeleton's limb TREE (child/sibling from the root), invoking cb(limbIndex,
// limb) for every limb EXCEPT the root (limb 0). MUST walk the tree, not a blind 0..limbCount-1
// loop: some skeletons carry an unreferenced trailing limb whose skeleton[]/jointTable[] slots
// are out of bounds (Boj: limbCount=16 but only limbs 0..14 are reachable) — blind indexing
// reads OOB and crashes. Bounded by limbCount visits; null/out-of-range indices are skipped.
typedef void (*SoH3D_LimbCb)(int limbIndex, StandardLimb* limb, void* ud);
static void SoH3D_WalkN64Skeleton(void** skeleton, int limbCap, SoH3D_LimbCb cb, void* ud) {
    if (skeleton == NULL || limbCap <= 0) {
        return;
    }
    StandardLimb* root = (StandardLimb*)SEGMENTED_TO_VIRTUAL(skeleton[0]);
    if (root == NULL || root->child == LIMB_DONE) {
        return;
    }
    int stack[128];
    int sp = 0;
    int visited = 0;
    stack[sp++] = root->child;
    while (sp > 0 && visited <= limbCap) {
        int idx = stack[--sp];
        if (idx < 0 || idx >= limbCap) {
            continue;
        }
        StandardLimb* lb = (StandardLimb*)SEGMENTED_TO_VIRTUAL(skeleton[idx]);
        if (lb == NULL) {
            continue;
        }
        visited++;
        cb(idx, lb, ud);
        if (lb->sibling != LIMB_DONE && sp < (int)ARRAY_COUNT(stack)) {
            stack[sp++] = lb->sibling;
        }
        if (lb->child != LIMB_DONE && sp < (int)ARRAY_COUNT(stack)) {
            stack[sp++] = lb->child;
        }
    }
}

static void SoH3D_AccumBoneLen(int limbIndex, StandardLimb* lb, void* ud) {
    (void)limbIndex;
    float x = lb->jointPos.x, y = lb->jointPos.y, z = lb->jointPos.z;
    *(float*)ud += sqrtf(x * x + y * y + z * z);
}

// Σ of N64 bone lengths (|jointPos| of every non-root reachable limb) — the rotation-invariant
// N64 skeleton size, for the rest-pose scale derivation. See SoH3D_AutoModelBoneLenSum.
static float SoH3D_N64SkelBoneLenSum(void** skeleton, int limbCap) {
    float sum = 0.0f;
    SoH3D_WalkN64Skeleton(skeleton, limbCap, SoH3D_AccumBoneLen, &sum);
    return sum;
}

static void SoH3D_MaxLimbCb(int limbIndex, StandardLimb* lb, void* ud) {
    (void)lb;
    if (limbIndex > *(int*)ud) *(int*)ud = limbIndex;
}

// Derive a usable limbCount for a raw skeleton (no SkelAnime handy): the highest reachable limb
// index + 1, so jointTable[limb+1] indexing stays in bounds. Capped at 64.
static int SoH3D_CountN64Limbs(void** skeleton) {
    int maxIdx = 0;
    SoH3D_WalkN64Skeleton(skeleton, 64, SoH3D_MaxLimbCb, &maxIdx);
    return maxIdx + 1;
}

static void SoH3D_DumpLimbCb(int limbIndex, StandardLimb* lb, void* ud) {
    Vec3s* jointTable = (Vec3s*)ud;
    Vec3s rot = jointTable[limbIndex + 1]; // reachable limb -> jointTable slot is valid
    fprintf(stderr, "[SKELDUMP] N64 limb=%d jointPos=(%d,%d,%d) child=%d sibling=%d rot=(%d,%d,%d)\n", limbIndex,
            lb->jointPos.x, lb->jointPos.y, lb->jointPos.z, lb->child, lb->sibling, rot.x, rot.y, rot.z);
}

// Core N64-anim retarget: given the live N64 skeleton + jointTable + limbCount for the actor
// deferred for replacement (gSoH3dPending*), retarget its OoT3D model and draw it; return 1 so
// the N64 limbs are skipped. Shared by the SkelAnime* wrapper and the raw (skeleton,jointTable)
// wrapper so all the common draw choke points get coverage.
static int SoH3D_DoRetarget(PlayState* play, void** skeleton, Vec3s* jointTable, int limbCount) {
    // ORACLE DUMP (SOH3D_SKELDUMP=1): print the live N64 skeleton + the OoT3D skeleton once per
    // model, for offline analysis. Tree walk is OOB-safe.
    {
        static int skeldump = -1;
        if (skeldump < 0) {
            const char* v = getenv("SOH3D_SKELDUMP");
            skeldump = (v != NULL && v[0] == '1') ? 1 : 0;
        }
        if (skeldump) {
            static int dumped[64];
            static int nDumped = 0;
            int already = 0;
            for (int d = 0; d < nDumped; d++)
                if (dumped[d] == gSoH3dPendingModel) {
                    already = 1;
                    break;
                }
            if (!already && nDumped < (int)ARRAY_COUNT(dumped)) {
                dumped[nDumped++] = gSoH3dPendingModel;
                Vec3f sc = gSoH3dPendingActor->scale;
                fprintf(stderr, "[SKELDUMP] N64 actor=0x%x model=%d limbCount=%d actorScale=(%.5f,%.5f,%.5f)\n",
                        gSoH3dPendingActor->id, gSoH3dPendingModel, limbCount, sc.x, sc.y, sc.z);
                SoH3D_WalkN64Skeleton(skeleton, limbCount, SoH3D_DumpLimbCb, jointTable);
                fflush(stderr);
                SoH3D_DumpModelBones(gSoH3dPendingModel);
            }
        }
    }
    const SoH3DBoneMap* bm = gSoH3dPendingBoneMap;
    if (gSoH3dPendingAuto) {
        // OWN-ANIMATION path (user direction): the OoT3D model plays its OWN authored CSAB —
        // correct for its own rig — instead of retargeting live N64 joints (which explodes on
        // rigs whose rest pose differs from N64). We only need the SCALE (rest-skeleton
        // bone-length ratio, same character) + a CSAB; no bone correspondence, no count guard, so
        // ANY skinned auto-actor with a CSAB renders.
        float n64sum = SoH3D_N64SkelBoneLenSum(skeleton, limbCount);
        float oot3dsum = SoH3D_AutoModelBoneLenSum(gSoH3dPendingModel);
        if (n64sum > 1e-3f && oot3dsum > 1e-3f) {
            gSoH3dPendingScale = gSoH3dPendingActor->scale.x * (n64sum / oot3dsum);
        }
        // Select the CSAB from the actor's LIVE N64 animation (true N64->3DS anim mapping): map the
        // current animation OTR path through kSoH3dAnimMaps; if it isn't mapped, fall back to the
        // model's default idle so an unmapped state still reads as standing rather than freezing.
        const char* mapped = SoH3D_ResolveAutoCsab(gSoH3dPendingAnimOtr);
        const char* csab = (mapped != NULL) ? mapped : SoH3D_AutoModelDefaultAnim(gSoH3dPendingModel);
        // LIVE anim-compare tooling: REPL `animforce <base>` pins a chosen CSAB on every replaced
        // actor so its motion can be eyeballed against the N64 anim (toggle `auto 0/1`). Empty = auto.
        gSoH3dLastAutoModel = gSoH3dPendingModel; // for REPL `animlist`
        if (gSoH3dForceCsab[0] != '\0') {
            csab = gSoH3dForceCsab;
        }
        if (gSoH3dAnimDebug) {
            static int dbg = 0;
            if ((dbg++ % 30) == 0) {
                const char* otr = gSoH3dPendingAnimOtr ? gSoH3dPendingAnimOtr : "(none)";
                int locked = (gSoH3dPendingN64AnimLength > 4.0f);
                printf("SOH3D ANIM: model %d n64=%s -> csab=%s%s scale=%.5f n64frame=%.1f/%.1f %s\n",
                       gSoH3dPendingModel, otr, csab ? csab : "(bind pose)", mapped ? "" : " [default-idle]",
                       gSoH3dPendingScale, gSoH3dPendingN64CurFrame, gSoH3dPendingN64AnimLength,
                       locked ? "[PHASE-LOCK]" : "[free-run]");
                fflush(stdout);
            }
        }
        SoH3D_UpdateAnimAuto(gSoH3dPendingModel, csab, gSoH3dAnimRate, gSoH3dPendingN64CurFrame,
                             gSoH3dPendingN64AnimLength);
        SoH3D_EmitModelDraw(play, gSoH3dPendingModel, gSoH3dPendingActor, gSoH3dPendingScale, gSoH3dPendingGroundOff);
        gSoH3dPendingModel = -1;
        gSoH3dPendingBoneMap = NULL;
        return 1;
    }
    // Hand-calibrated table entry (e.g. En_Ge1): retarget from the live N64 joints (the bone map,
    // if any, fixes the correspondence). Kept for the few hand-verified rigs that work this way.
    if (bm != NULL) {
        SoH3D_UpdateAnimN64Mapped(gSoH3dPendingModel, (const s16*)&jointTable[1], limbCount, bm->boneToLimb,
                                  bm->boneCount);
    } else {
        SoH3D_UpdateAnimN64(gSoH3dPendingModel, (const s16*)&jointTable[1], limbCount);
    }
    SoH3D_EmitModelDraw(play, gSoH3dPendingModel, gSoH3dPendingActor, gSoH3dPendingScale, gSoH3dPendingGroundOff);
    gSoH3dPendingModel = -1; // drawn once this actor; don't re-draw on a second SkelAnime call
    gSoH3dPendingBoneMap = NULL;
    return 1;
}

// Generic N64-anim hook (declared in soh3d.h). Two entry points so ALL the common draw choke
// points are covered: this one takes a SkelAnime* (SkelAnime_DrawSkeletonOpa/DrawSkeleton2 and
// func_80034BA0/CC4), and SoH3D_SkelAnimeDrawRaw takes the raw skeleton+jointTable
// (SkelAnime_DrawFlexOpa/DrawOpa, which many actors call directly without a SkelAnime*).
int SoH3D_SkelAnimeDraw(PlayState* play, SkelAnime* skelAnime) {
    if (gSoH3dPendingModel < 0 || gSoH3dPendingActor == NULL) {
        return 0; // no pending replacement for the current actor
    }
    if (skelAnime == NULL || skelAnime->jointTable == NULL || skelAnime->limbCount == 0) {
        return 0; // no usable pose -> let the N64 skeleton draw
    }
    // This is the only choke point with a SkelAnime*, so it's where the live N64 animation pointer
    // (an OTR path string in SoH) is available — stash it for the auto CSAB resolver below.
    gSoH3dPendingAnimOtr = (const char*)skelAnime->animation;
    // Capture the live N64 playhead so the auto branch can phase-lock the OoT3D CSAB to it.
    gSoH3dPendingN64CurFrame = skelAnime->curFrame;
    gSoH3dPendingN64AnimLength = skelAnime->animLength;
    return SoH3D_DoRetarget(play, skelAnime->skeleton, skelAnime->jointTable, skelAnime->limbCount);
}

int SoH3D_SkelAnimeDrawRaw(PlayState* play, void** skeleton, Vec3s* jointTable) {
    if (gSoH3dPendingModel < 0 || gSoH3dPendingActor == NULL) {
        return 0; // no pending replacement -> cheap early out (this fires for every limbed draw)
    }
    if (skeleton == NULL || jointTable == NULL) {
        return 0;
    }
    int limbCount = SoH3D_CountN64Limbs(skeleton);
    if (limbCount <= 0) {
        return 0;
    }
    // No SkelAnime here -> no animation pointer. Don't clear gSoH3dPendingAnimOtr: a wrapper with
    // the SkelAnime (func_80034BA0/CC4) may have already captured it before routing to DrawFlex.
    return SoH3D_DoRetarget(play, skeleton, jointTable, limbCount);
}

void SoH3D_AfterActorDraw(PlayState* play, Actor* actor) {
    if (sPendingMeasureKey >= 0) {
        SoH3D_EmitMeasure(play, sPendingMeasureKey, /*begin=*/0);
        sPendingMeasureKey = -1;
    }
    // Clear any N64-anim deferral for this actor (whether or not the SkelAnime hook fired —
    // if it didn't, the actor's N64 model drew as the fallback).
    gSoH3dPendingActor = NULL;
    gSoH3dPendingModel = -1;
    gSoH3dPendingBoneMap = NULL;
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

// Emit the once-per-frame SoH3D render-pass marker into POLY_OPA. When the interpreter reaches
// it, every SoH3D draw collected this frame is rendered in ONE GL-state-bracketed pass
// (libultraship SoH3D_GL_RenderPass) — so OoT3D content composites after Fast3D's opaque 3D and
// before the 2D/UI pass, and our GL state never leaks into Fast3D's. Called from Play_Draw right
// after the actor draw-all (func_800315AC).
// Feed the GL form-light its world-space key direction from the scene's live (time-of-day
// interpolated) directional light. lightSettings.light1Dir is the F3DEX "direction TO the light"
// (OoT copies it straight into dirLight1.params.dir), which is exactly the L the half-Lambert
// term wants. NO view transform is needed: OoT folds the camera into the PROJECTION matrix
// (z_view.c loads viewing with G_MTX_PROJECTION), so the GL shader's normal is in WORLD space —
// same frame as light1Dir. Degenerate (near-zero) dirs are skipped so the previous value holds.
// REPL `lightdir`: when set, hold a fixed world-space light dir instead of the scene's, so the
// plumbing can be exercised / a direction A/B'd live. Also remembers the last live dir for `state`.
int gSoH3dLightDirOverride = 0;
float gSoH3dLightDirLast[3] = { 0.40f, 0.55f, 0.73f };

static void SoH3D_UpdateLight(PlayState* play) {
    EnvLightSettings* ls = &play->envCtx.lightSettings;
    float d[3];
    float len;
    if (gSoH3dLightDirOverride) {
        return; // held by REPL `lightdir x y z`
    }
    d[0] = (float)ls->light1Dir[0];
    d[1] = (float)ls->light1Dir[1];
    d[2] = (float)ls->light1Dir[2];
    len = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (len < 1.0f) {
        return; // no usable directional light this frame; keep the last/default dir
    }
    d[0] /= len;
    d[1] /= len;
    d[2] /= len;
    gSoH3dLightDirLast[0] = d[0];
    gSoH3dLightDirLast[1] = d[1];
    gSoH3dLightDirLast[2] = d[2];
    SoH3D_GL_SetLightDir(d);
}

void SoH3D_EmitRenderPass(PlayState* play) {
    if (!SoH3D_Enabled()) {
        return;
    }
    SoH3D_UpdateLight(play);
    OPEN_DISPS(play->state.gfxCtx);
    gSPSoH3DRenderPass(POLY_OPA_DISP++);
    CLOSE_DISPS(play->state.gfxCtx);
}

// Per-frame, before the display list is built: drop any SoH3D draws left unrendered from a prior
// frame (e.g. a scene-transition early-out that emitted draws but never reached the render pass)
// so stale items can't double-draw next frame. Cheap no-op when the list is empty.
void SoH3D_FrameBegin(void) {
    if (!SoH3D_Enabled()) {
        return;
    }
    SoH3D_GL_FrameBegin();
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
        // Render mesh is left UNTOUCHED (pixel-faithful OoT3D). Actors are grounded onto the
        // visible OoT3D floor per-actor at draw time (SoH3D_ActorRenderYOffset, direct mesh
        // raycast) — no precomputed warp/grid here.
        SoH3D_DrawRoomGL(play, modelId);
    }
    return 1; // drew the OoT3D room -> caller skips the N64 mesh
}

float SoH3D_ActorRenderYOffset(PlayState* play, Actor* actor) {
    const char* sceneName;
    int modelId, room;
    float n64, oot;
    if (actor == NULL || !SoH3D_Enabled() || !SoH3D_TerrainWarpEnabled()) {
        return 0.0f;
    }
    sceneName = SoH3D_SceneName(play);
    if (sceneName == NULL) {
        return 0.0f; // scene has no OoT3D mapping
    }
    // Use the actor's room when it has one, else the current room (e.g. -1 = persistent actor).
    room = (actor->room >= 0) ? actor->room : play->roomCtx.curRoom.num;
    modelId = SoH3D_RoomModelId(sceneName, room);
    if (modelId < 0) {
        return 0.0f;
    }
    // Ground the render EXACTLY on the visible OoT3D mesh: offset = OoT3D_floor - N64_floor at
    // the actor's XZ (the OoT3D floor closest to the N64 floor, so multi-level spots pick the
    // right surface). Direct raycast of the actual render mesh — no 100u grid approximation
    // (which hole-filled/smeared and sank actors). For an airborne actor this shifts by the
    // ground delta, preserving its height above ground.
    sWarpPlay = play; // SoH3D_N64FloorCb needs the PlayState/colCtx
    n64 = SoH3D_N64FloorCb(actor->world.pos.x, actor->world.pos.z);
    if (n64 <= -31000.0f) {
        return 0.0f; // no N64 floor under the actor -> can't reconcile, leave it
    }
    if (!SoH3D_RoomOoT3DFloorAt(modelId, actor->world.pos.x, actor->world.pos.z, n64, &oot)) {
        return 0.0f; // no OoT3D render floor here -> no offset
    }
    return oot - n64; // lift/drop the render onto the visible OoT3D ground
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
//   actorscan <id>     list world pos + dist of every live actor with id (dec or 0xHEX)
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
    int iv;
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
    } else if (strcmp(cmd, "warp") == 0 && sscanf(line, "%*s %i", &iv) == 1) {
        // Trigger an in-game scene transition to an entrance index (decimal or 0x-hex), so
        // the live instance can hop scenes without a relaunch (e.g. `warp 0xee` = Kokiri
        // Forest). Same mechanism actors use to send Link through a loading zone.
        play->nextEntranceIndex = iv;
        play->transitionTrigger = TRANS_TRIGGER_START;
        play->transitionType = TRANS_TYPE_FADE_BLACK;
        SoH3D_ReplReply(outPath, "warp -> entrance 0x%x (%d)", iv, iv);
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
    } else if (strcmp(cmd, "actors") == 0) {
        // List actors (id + object id + world pos + distance from Link), so an NPC can be
        // located and framed (cam/tp) without hunting. Default: NPC category only; "actors all"
        // lists every category. Used to drive character-replacement verification.
        Player* p = GET_PLAYER(play);
        int wantAll = (strstr(line, "all") != NULL);
        s32 cat, shown = 0;
        for (cat = 0; cat < ACTORCAT_MAX; cat++) {
            if (!wantAll && cat != ACTORCAT_NPC && cat != ACTORCAT_ENEMY && cat != ACTORCAT_BOSS) {
                continue;
            }
            Actor* a = play->actorCtx.actorLists[cat].head;
            for (; a != NULL && shown < 40; a = a->next) {
                float dx = a->world.pos.x - p->actor.world.pos.x;
                float dz = a->world.pos.z - p->actor.world.pos.z;
                int objId = -1;
                if (a->objBankIndex >= 0 && a->objBankIndex < play->objectCtx.num) {
                    objId = play->objectCtx.status[a->objBankIndex].id;
                }
                SoH3D_ReplReply(outPath, "actor id=0x%x cat=%d obj=0x%x pos=(%.0f,%.0f,%.0f) dist=%.0f", a->id, cat,
                                objId, a->world.pos.x, a->world.pos.y, a->world.pos.z, sqrtf(dx * dx + dz * dz));
                shown++;
            }
        }
        if (!shown) {
            SoH3D_ReplReply(outPath, "actors: none in the requested categories");
        }
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
    } else if (strcmp(cmd, "exitat") == 0 && sscanf(line, "%*s %f %f", &f1, &f2) == 2) {
        // Report the floor poly's SurfaceType gameplay data at (x,z): scene exit index, camera
        // index, and floor type. Verifies the OoT3D surfaceType list is wired (exits/cameras).
        Vec3f pos = { f1, 10000.0f, f2 };
        CollisionPoly* poly = NULL;
        f32 y = BgCheck_EntityRaycastFloor1(&play->colCtx, &poly, &pos);
        if (poly != NULL) {
            u32 exitIdx = SurfaceType_GetSceneExitIndex(&play->colCtx, poly, BGCHECK_SCENE);
            u32 camIdx = SurfaceType_GetCamDataIndex(&play->colCtx, poly, BGCHECK_SCENE);
            SoH3D_ReplReply(outPath, "exitat (%.0f,%.0f) y=%.1f type=%d exit=%d cam=%d",
                            f1, f2, y, poly->type, exitIdx, camIdx);
        } else {
            SoH3D_ReplReply(outPath, "exitat (%.0f,%.0f) NO FLOOR", f1, f2);
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
    } else if (strcmp(cmd, "collision") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        // Toggle OoT3D-collision gameplay. The collision is built+installed at scene load, so
        // this takes effect on the NEXT scene load / `warp` (the current colCtx stays as-is).
        gSoH3dCollision = (int)f1;
        SoH3D_ReplReply(outPath, "collision=%d (applies on next scene load / warp)", gSoH3dCollision);
    } else if (strcmp(cmd, "time") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        // Pin time-of-day (0x8000=noon, 0x4000=dawn, 0xC000=dusk, 0=midnight). Negative
        // releases the game clock. Accepts a raw u16 value.
        gSoH3dForceTime = (f1 < 0.0f) ? -1 : ((int)f1 & 0xFFFF);
        SoH3D_ReplReply(outPath, "time=%d (0x%04x)%s", gSoH3dForceTime, gSoH3dForceTime < 0 ? 0 : gSoH3dForceTime,
                        gSoH3dForceTime < 0 ? " (clock released)" : "");
    } else if (strcmp(cmd, "auto") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dAuto = (int)f1;
        SoH3D_ReplReply(outPath, "auto=%d (0=off,1=fill non-table actors,2=ALL/validation)", gSoH3dAuto);
    } else if (strcmp(cmd, "n64anim") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        gSoH3dN64Anim = (int)f1;
        SoH3D_ReplReply(outPath, "n64anim=%d (1=N64 SkelAnime joints on OoT3D skeleton, 0=CSAB)", gSoH3dN64Anim);
    } else if (strcmp(cmd, "animlist") == 0) {
        // LIVE anim-compare: print the CSABs of the last replaced model so they can be `animforce`d.
        extern void SoH3D_AutoModelCsabList(int modelId, char* out, int outsz);
        static char buf[3072];
        buf[0] = '\0';
        if (gSoH3dLastAutoModel >= 0) {
            SoH3D_AutoModelCsabList(gSoH3dLastAutoModel, buf, (int)sizeof(buf));
        }
        SoH3D_ReplReply(outPath, "animlist model=%d: %s", gSoH3dLastAutoModel, buf[0] ? buf : "(none seen yet)");
    } else if (strcmp(cmd, "animforce") == 0) {
        // `animforce <csab-base>` pins that CSAB on EVERY replaced actor (eyeball it vs the N64 anim,
        // toggle `auto 0/1`); `animforce off` / no-arg returns to the auto resolver.
        char name[64] = "";
        if (sscanf(line, "%*s %63s", name) == 1 && strcmp(name, "off") != 0) {
            strncpy(gSoH3dForceCsab, name, sizeof(gSoH3dForceCsab) - 1);
            gSoH3dForceCsab[sizeof(gSoH3dForceCsab) - 1] = '\0';
            SoH3D_ReplReply(outPath, "animforce='%s' (forced on all replaced actors; `animforce off` to release)",
                            gSoH3dForceCsab);
        } else {
            gSoH3dForceCsab[0] = '\0';
            SoH3D_ReplReply(outPath, "animforce OFF (auto-resolve restored)");
        }
    } else if (strcmp(cmd, "autostate") == 0) {
        // Dump every object that the auto path has touched: state + derived scale, so the
        // measured scale can be checked against the hand-tuned values (pot/crate/bush/...).
        s32 k;
        int shown = 0;
        for (k = 0; k < (s32)ARRAY_COUNT(sAuto); k++) {
            if (sAuto[k].state != 0 || sAuto[k].measuredH > 0.0f) {
                SoH3D_ReplReply(outPath, "auto[0x%x] %s state=%d scale=%.5f n64h=%.1f model=%d", k,
                                kSoH3dObjectZars[k] ? kSoH3dObjectZars[k] : "?", sAuto[k].state, sAuto[k].scale,
                                sAuto[k].measuredH, sAuto[k].modelId);
                shown++;
            }
        }
        if (!shown) {
            SoH3D_ReplReply(outPath, "autostate: no auto-replaced objects seen yet (auto=%d)", SoH3D_AutoMode());
        }
    } else if (strcmp(cmd, "jointdump") == 0 && sscanf(line, "%*s %1023s", path) == 1) {
        // Dump the live En_Ge1 SkelAnime jointTable to a CSV, for the QUANTITATIVE
        // N64->OoT3D retarget derivation: idx 0 = root translation (Vec3s), idx 1..limbCount =
        // per-limb binang rotations (x,y,z). Combined offline with the CMB rest rotations and
        // the CSAB ge1_s_wait animated rotations (tools/soh3d_anim_derive.py) to solve the
        // per-limb rotation convention numerically instead of eyeballing rotation orders.
        EnGe1* ge = NULL;
        s32 cat;
        for (cat = 0; cat < ACTORCAT_MAX && ge == NULL; cat++) {
            Actor* a = play->actorCtx.actorLists[cat].head;
            for (; a != NULL; a = a->next) {
                if (a->id == ACTOR_EN_GE1) { ge = (EnGe1*)a; break; }
            }
        }
        if (ge == NULL || ge->skelAnime.jointTable == NULL || ge->skelAnime.limbCount <= 0) {
            SoH3D_ReplReply(outPath, "jointdump: no live En_Ge1 with a jointTable found");
        } else {
            FILE* jf = fopen(path, "w");
            if (jf == NULL) {
                SoH3D_ReplReply(outPath, "jointdump: cannot open %s", path);
            } else {
                const char* n64 = (const char*)ge->animation;
                s32 li;
                fprintf(jf, "# En_Ge1 jointTable; limbCount=%d curFrame=%.3f animLength=%.1f anim=%s\n",
                        ge->skelAnime.limbCount, ge->skelAnime.curFrame, ge->skelAnime.animLength,
                        n64 ? n64 : "(null)");
                fprintf(jf, "idx,x,y,z\n");
                for (li = 0; li <= ge->skelAnime.limbCount; li++) {
                    Vec3s* j = &ge->skelAnime.jointTable[li];
                    fprintf(jf, "%d,%d,%d,%d\n", li, j->x, j->y, j->z);
                }
                fclose(jf);
                SoH3D_ReplReply(outPath, "jointdump -> %s (limbCount=%d curFrame=%.2f anim=%s)", path,
                                ge->skelAnime.limbCount, ge->skelAnime.curFrame, n64 ? n64 : "(null)");
            }
        }
    } else if (strcmp(cmd, "actorscan") == 0 && sscanf(line, "%*s %i", &iv) == 1) {
        // List world positions of every live actor with id `iv` (decimal or 0xHEX), plus
        // distance from Link — for framing multi-instance actors (e.g. En_Hata flags, id
        // 0x26) to verify per-item pose. Tooling-first: replaces blind scene-wandering.
        Player* pl = GET_PLAYER(play);
        s32 cat, n = 0;
        SoH3D_ReplReply(outPath, "actorscan id=0x%X:", iv);
        for (cat = 0; cat < ACTORCAT_MAX; cat++) {
            Actor* a = play->actorCtx.actorLists[cat].head;
            for (; a != NULL; a = a->next) {
                if (a->id == iv) {
                    float dx = a->world.pos.x - pl->actor.world.pos.x;
                    float dy = a->world.pos.y - pl->actor.world.pos.y;
                    float dz = a->world.pos.z - pl->actor.world.pos.z;
                    SoH3D_ReplReply(outPath, "  [%d] pos=(%.0f,%.0f,%.0f) dist=%.0f cat=%d", n,
                                    a->world.pos.x, a->world.pos.y, a->world.pos.z,
                                    sqrtf(dx * dx + dy * dy + dz * dz), cat);
                    n++;
                }
            }
        }
        SoH3D_ReplReply(outPath, "actorscan: %d found", n);
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
    } else if (strcmp(cmd, "light") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        extern int gSoH3dLightEnable; // libultraship soh3d_gl.cpp: character/prop form lighting
        gSoH3dLightEnable = (int)f1;
        SoH3D_ReplReply(outPath, "light=%d (1=half-Lambert form on characters/props, 0=flat tint)",
                        gSoH3dLightEnable);
    } else if (strcmp(cmd, "statecheck") == 0 && sscanf(line, "%*s %f", &f1) == 1) {
        // GL state-leak detector (libultraship soh3d_gl.cpp). Flip on the moment the skybox/HUD
        // stripe corruption appears: every render pass then verifies it handed back all captured GL
        // state, logging any leaked field to stderr/run.log. Has per-frame glGet overhead -> off normally.
        extern int gSoH3dStateCheck;
        gSoH3dStateCheck = (int)f1;
        SoH3D_ReplReply(outPath, "statecheck=%d (1=log any GL state our render pass fails to restore)",
                        gSoH3dStateCheck);
    } else if (strcmp(cmd, "lightdir") == 0) {
        // `lightdir x y z` overrides the world-space form-light dir (held until `lightdir auto`);
        // `lightdir auto` returns to the scene's live light1Dir; `lightdir` alone prints the dir.
        float v[3];
        char sub[32];
        if (sscanf(line, "%*s %f %f %f", &v[0], &v[1], &v[2]) == 3) {
            float len = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (len < 1e-4f) len = 1.0f;
            v[0] /= len; v[1] /= len; v[2] /= len;
            gSoH3dLightDirOverride = 1;
            gSoH3dLightDirLast[0] = v[0]; gSoH3dLightDirLast[1] = v[1]; gSoH3dLightDirLast[2] = v[2];
            SoH3D_GL_SetLightDir(v);
            SoH3D_ReplReply(outPath, "lightdir OVERRIDE=(%.3f,%.3f,%.3f)", v[0], v[1], v[2]);
        } else if (sscanf(line, "%*s %31s", sub) == 1 && strcmp(sub, "auto") == 0) {
            gSoH3dLightDirOverride = 0;
            SoH3D_ReplReply(outPath, "lightdir AUTO (scene light1Dir)");
        } else {
            SoH3D_ReplReply(outPath, "lightdir=(%.3f,%.3f,%.3f) %s", gSoH3dLightDirLast[0], gSoH3dLightDirLast[1],
                            gSoH3dLightDirLast[2], gSoH3dLightDirOverride ? "(override)" : "(auto/live light1Dir)");
        }
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
        SoH3D_ReplReply(outPath, "? '%s' (cmds: mul diff tint enable auto autostate scale yoff rotx roty rotz animrate animframe animlive spawn cam camorbit camfreeze floorat floorgrid dump state)", line);
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
