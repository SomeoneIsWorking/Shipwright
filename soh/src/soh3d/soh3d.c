// SoH3D runtime toggle + helpers. See repo-root PROGRESS.md.
#include "soh3d.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
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

// Direct-GL model path (soh3d_model.cpp bridge + libultraship SoH3D_GL_*). Models
// flagged with glModelId>=0 in sModelTable render through this PC-native path
// (runtime-loaded 3DS asset, our own GL shader) instead of the legacy N64 dlist.
void SoH3D_EnsureModelProvider(void);
static void SoH3D_DrawModelGL(PlayState* play, int modelId, Actor* actor, float worldScale);

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
static void SoH3D_DrawModelGL(PlayState* play, int modelId, Actor* actor, float worldScale) {
    u8 tint[3];
    OPEN_DISPS(play->state.gfxCtx);

    SoH3D_EnsureModelProvider();
    Gfx_SetupDL_25Opa(play->state.gfxCtx);
    Matrix_Translate(actor->world.pos.x, actor->world.pos.y, actor->world.pos.z, MTXMODE_NEW);
    Matrix_RotateY(BINANG_TO_RAD(actor->shape.rot.y), MTXMODE_APPLY);
    Matrix_Scale(worldScale, worldScale, worldScale, MTXMODE_APPLY);
    if (gSoH3dRotX != 0.0f) Matrix_RotateX(gSoH3dRotX * (3.14159265f / 180.0f), MTXMODE_APPLY);
    if (gSoH3dRotY != 0.0f) Matrix_RotateY(gSoH3dRotY * (3.14159265f / 180.0f), MTXMODE_APPLY);
    if (gSoH3dRotZ != 0.0f) Matrix_RotateZ(gSoH3dRotZ * (3.14159265f / 180.0f), MTXMODE_APPLY);
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
typedef struct {
    s16 actorId;
    const char* name; // REPL handle for `scale <name>` / `spawn <name>`
    Gfx* dlist;       // legacy Fast3D dlist (used when glModelId < 0)
    float worldScale; // live (REPL-pokeable)
    int glModelId;    // >=0 = render via the direct-GL path with this asset id; -1 = legacy dlist
} SoH3D_ModelEntry;

// Non-const so the REPL can tune worldScale live.
static SoH3D_ModelEntry sModelTable[] = {
    { ACTOR_OBJ_TSUBO, "pot", soh3d_pot_model_dl, SOH3D_POT_WORLD_SCALE, -1 },
    { ACTOR_EN_GS, "gs", soh3d_gs_model_dl, SOH3D_GS_WORLD_SCALE, -1 },
    { ACTOR_OBJ_KIBAKO2, "kibako", soh3d_kibako_model_dl, SOH3D_KIBAKO_WORLD_SCALE, -1 },
    { ACTOR_EN_GE1, "geldwoman", soh3d_geldwoman_model_dl, SOH3D_GELDWOMAN_WORLD_SCALE, 0 },
};

int SoH3D_TryDrawActor(PlayState* play, Actor* actor) {
    s32 i;
    if (!SoH3D_Enabled()) {
        return 0;
    }
    for (i = 0; i < ARRAY_COUNT(sModelTable); i++) {
        if (sModelTable[i].actorId == actor->id) {
            if (sModelTable[i].glModelId >= 0) {
                SoH3D_DrawModelGL(play, sModelTable[i].glModelId, actor, sModelTable[i].worldScale);
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
//   scale <name> <f>   live world scale for a model (pot|gs|kibako)
//   spawn <name>       spawn that actor in front of Link
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
    float fx = p->actor.world.pos.x + dist * Math_SinS(yaw);
    float fz = p->actor.world.pos.z + dist * Math_CosS(yaw);
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
    float f1, f2;
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
    } else if (strcmp(cmd, "scale") == 0 && sscanf(line, "%*s %63s %f", arg, &f1) == 2) {
        SoH3D_ModelEntry* e = SoH3D_FindModel(arg);
        if (e != NULL) {
            e->worldScale = f1;
            SoH3D_ReplReply(outPath, "scale %s=%.4f", e->name, e->worldScale);
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
            n += snprintf(scales + n, sizeof(scales) - n, "%s%s=%.4f", k ? " " : "", sModelTable[k].name,
                          sModelTable[k].worldScale);
        }
        SoH3D_ReplReply(outPath, "enabled=%d diff=%.3f mul=%.3f tint=(%d,%d,%d) scale: %s", SoH3D_Enabled(),
                        gSoH3dTintDiff, gSoH3dTintMul, tint[0], tint[1], tint[2], scales);
    } else {
        SoH3D_ReplReply(outPath, "? '%s' (cmds: mul diff tint enable scale rotx roty rotz spawn dump state)", line);
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
}
