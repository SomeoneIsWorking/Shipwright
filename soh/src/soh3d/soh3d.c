// SoH3D runtime toggle. See PROGRESS.md.
#include "soh3d.h"
#include <stdlib.h>

int SoH3D_Enabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char* v = getenv("SOH3D");
        cached = (v != NULL && v[0] == '1') ? 1 : 0;
    }
    return cached;
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

static int SoH3D_DebugPotEnabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char* v = getenv("SOH3D_DEBUGPOT");
        cached = (v != NULL && v[0] == '1') ? 1 : 0;
    }
    return cached;
}

static float SoH3D_DrawScale(void) {
    const char* v = getenv("SOH3D_SCALE");
    if (v != NULL && v[0] != '\0') {
        float f = (float)atof(v);
        if (f > 0.0f) {
            return f;
        }
    }
    return 1.0f;
}

void SoH3D_DebugDrawPot(PlayState* play) {
    Player* player;
    float scale;

    if (!SoH3D_DebugPotEnabled()) {
        return;
    }

    player = GET_PLAYER(play);
    scale = SoH3D_DrawScale();

    // Place the model at Link's feet; Gfx_DrawDListOpa loads the current matrix
    // stack top (MATRIX_NEWMTX) before emitting the dlist.
    Matrix_Translate(player->actor.world.pos.x, player->actor.world.pos.y, player->actor.world.pos.z, MTXMODE_NEW);
    Matrix_Scale(scale, scale, scale, MTXMODE_APPLY);
    Gfx_DrawDListOpa(play, soh3d_pot_model_dl);
}
