// charcompare — N64-vs-3DS character comparison tool.
//
// One window, two viewports (left = N64 via libultraship Fast3D, right = OoT3D/3DS
// via the soh3d asset parsers + the engine's direct-GL skinned renderer), each
// showing the SAME character + animation, to drive the N64<->3DS anim-map curation.
//
// PHASE 2: the 3DS viewport. Loads an OoT3D character from the .3ds and renders it
// (animated) through the engine's OTR_G_SOH3D_DRAW/_RENDERPASS path, by driving the
// Fast3D interpreter with a hand-built display list each frame (mirrors the in-game
// render path and the dlist_harness). ImGui panel selects the animation + frame.

#include <fast/Fast3dWindow.h>
#include <fast/interpreter.h>
#include <ship/Context.h>

#include <imgui.h>

#include <SDL2/SDL.h>
#define GL_GLEXT_PROTOTYPES 1
#include <SDL2/SDL_opengl.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "cc_3ds.h"

namespace fs = std::filesystem;

// The Fast3D interpreter's OTR_G_SOH3D_MEASURE opcode handler calls back into this
// game-provided symbol (soh3d.c) to report an actor's measured world height for the
// auto-scale path. The comparison tool never emits that opcode, but the reference
// must resolve at link time — provide a no-op stub.
extern "C" void SoH3D_MeasureResult(int /*key*/, float /*height*/) {}

// Deterministic verification: dump the front buffer to a PPM (top-to-bottom). Used by
// CC_SHOT=<path> CC_SHOT_FRAME=<n> to grab a frame headlessly (the WM may hide the SDL
// window behind other windows, so an OS screenshot is unreliable).
static void dumpFrontBuffer(const std::string& path, int w, int h) {
    std::vector<unsigned char> rgb((size_t)w * h * 3);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
    fs::create_directories(fs::path(path).parent_path());
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; y--) fwrite(rgb.data() + (size_t)y * w * 3, 1, (size_t)w * 3, f); // GL row 0 = bottom
    fclose(f);
    size_t nonBlack = 0;
    for (size_t i = 0; i < rgb.size(); i += 3)
        if (rgb[i] | rgb[i + 1] | rgb[i + 2]) nonBlack++;
    fprintf(stderr, "[charcompare] dumped %s (%dx%d, %zu/%d non-black px)\n", path.c_str(), w, h, nonBlack, w * h);
}

// Find an archive (soh.o2r / oot.o2r) given how this tool is launched. run.sh-style
// usage cds into build-cmake/soh; the binary itself lands in build-cmake/soh/charcompare.
static std::string locateArchive(const std::string& name, const char* argv0) {
    std::vector<fs::path> dirs;
    dirs.push_back(fs::current_path());
    if (argv0) {
        std::error_code ec;
        fs::path exe = fs::weakly_canonical(fs::path(argv0), ec);
        if (!ec) {
            dirs.push_back(exe.parent_path());               // .../charcompare/
            dirs.push_back(exe.parent_path().parent_path()); // .../soh/  (where the o2r live)
        }
    }
    for (const auto& d : dirs) {
        fs::path p = d / name;
        if (fs::exists(p)) return p.string();
    }
    return {};
}

int main(int argc, char** argv) {
    printf("[charcompare] starting\n");

    // Default character; override with argv[1] (a ZAR path).
    std::string zarPath = (argc > 1) ? argv[1] : "/actor/zelda_ge1.zar";

    std::vector<std::string> archivePaths;
    std::string soh = locateArchive("soh.o2r", argv[0]);
    std::string oot = locateArchive("oot.o2r", argv[0]);
    if (!soh.empty()) archivePaths.push_back(soh);
    if (!oot.empty()) archivePaths.push_back(oot);
    if (soh.empty())
        fprintf(stderr, "[charcompare] warning: soh.o2r not found (GUI font / GL shaders may be missing)\n");
    for (const auto& p : archivePaths) printf("[charcompare] archive: %s\n", p.c_str());

    auto ctx = Ship::Context::CreateUninitializedInstance("CharCompare", "charcmp", "charcompare.json");
    ctx->InitConfiguration();
    ctx->InitConsoleVariables();
    ctx->InitControlDeck();
    ctx->InitResourceManager(archivePaths, {}, 3, true);
    ctx->InitConsole();

    auto window = std::make_shared<Fast::Fast3dWindow>(std::vector<std::shared_ptr<Ship::GuiWindow>>({}));
    ctx->InitWindow(window);
    ctx->InitLogging();

    auto interp = window->GetInterpreterWeak().lock();
    if (!interp) {
        fprintf(stderr, "[charcompare] no interpreter — cannot render\n");
        return 1;
    }

    printf("[charcompare] window backend: %s (%ux%u)\n", window->GetWindowBackendName().c_str(),
           window->GetWidth(), window->GetHeight());

    // Load the 3DS character.
    cc::Model3ds model = cc::Load(zarPath);
    if (!model.ok) fprintf(stderr, "[charcompare] 3DS load failed: %s\n", model.error.c_str());

    // UI / animation state.
    int animIdx = model.anims.empty() ? -1 : 0;
    float frame = 0.0f;
    bool playing = true;
    float playSpeed = 0.5f; // frames per render-frame
    float rx = 0, ry = 180, rz = 0; // face the camera (characters author facing +Z away)

    // Headless verification hooks.
    std::string shotPath = getenv("CC_SHOT") ? getenv("CC_SHOT") : "";
    int shotFrame = getenv("CC_SHOT_FRAME") ? atoi(getenv("CC_SHOT_FRAME")) : 120;
    long frameCount = 0;

    auto gui = window->GetGui();
    while (window->IsRunning()) {
        window->HandleEvents();
        if (!window->IsFrameReady()) {
            continue;
        }

        // Advance the animation.
        std::string animName = (animIdx >= 0 && animIdx < (int)model.anims.size()) ? model.anims[animIdx] : "";
        if (playing && !animName.empty()) frame += playSpeed;
        cc::SetAnim(model, animName, frame);

        // Build this frame's display list (3DS viewport for now).
        std::vector<Gfx> dl;
        std::unordered_map<Mtx*, MtxF> mtx;
        cc::DlistKeys keys;
        cc::EmitDlist(model, dl, mtx, keys, rx, ry, rz);
        Gfx end = gsSPEndDisplayList();
        dl.push_back(end);

        gui->StartDraw();
        window->StartFrame();
        interp->Run(dl.data(), mtx);

        // ImGui controls.
        ImGui::SetNextWindowSize(ImVec2(380, 320), ImGuiCond_FirstUseEver);
        ImGui::Begin("CharCompare - 3DS");
        ImGui::Text("%s", zarPath.c_str());
        if (!model.ok) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "load failed: %s", model.error.c_str());
        } else {
            ImGui::Text("model id %d, %zu anims", model.modelId, model.anims.size());
            if (animIdx >= 0) {
                const char* cur = model.anims[animIdx].c_str();
                if (ImGui::BeginCombo("anim", cur)) {
                    for (int i = 0; i < (int)model.anims.size(); i++) {
                        bool sel = (i == animIdx);
                        if (ImGui::Selectable(model.anims[i].c_str(), sel)) {
                            animIdx = i;
                            frame = 0.0f;
                        }
                        if (sel) ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }
            ImGui::Checkbox("play", &playing);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("speed", &playSpeed, 0.0f, 2.0f);
            ImGui::SliderFloat("frame", &frame, 0.0f, 200.0f);
            ImGui::SliderFloat("rotX", &rx, -180, 180);
            ImGui::SliderFloat("rotY", &ry, -180, 180);
            ImGui::SliderFloat("rotZ", &rz, -180, 180);
        }
        ImGui::End();

        gui->EndDraw();

        // Dump after EndDraw (scene + ImGui composited into the back buffer) but before
        // EndFrame's swap, reading GL_BACK.
        if (!shotPath.empty() && frameCount == shotFrame) {
            dumpFrontBuffer(shotPath, (int)window->GetWidth(), (int)window->GetHeight());
            window->EndFrame();
            break;
        }
        window->EndFrame();
        frameCount++;
    }

    printf("[charcompare] window closed, exiting\n");
    Ship::Context::DestroyInstance();
    return 0;
}
