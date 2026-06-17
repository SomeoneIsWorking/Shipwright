// charcompare — N64-vs-3DS character comparison tool.
//
// One window, two viewports (left = N64 via libultraship Fast3D, right = OoT3D/3DS
// via the soh3d asset parsers + a direct-GL skinned renderer), each showing the SAME
// character + animation at a shared camera, to drive the N64<->3DS anim-map curation.
//
// PHASE 1 (this file, for now): stand up libultraship standalone — open a window,
// run a frame loop, draw an ImGui panel. De-risks the embed before the viewports.
//
// Boot mirrors soh's OTRGlobals ctor (the minimal subset): an uninitialized
// Ship::Context, then config/cvars/control-deck/resource-manager/console, a
// Fast3dWindow, and the StartFrame/RunGuiOnly/EndFrame loop. Archives (soh.o2r for
// the GUI font, oot.o2r for the N64 char data later) are located next to the build.

#include <fast/Fast3dWindow.h>
#include <ship/Context.h>

#include <imgui.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// The Fast3D interpreter's OTR_G_SOH3D_MEASURE opcode handler calls back into this
// game-provided symbol (soh3d.c) to report an actor's measured world height for the
// auto-scale path. The comparison tool never emits that opcode, but the reference
// must resolve at link time — provide a no-op stub.
extern "C" void SoH3D_MeasureResult(int /*key*/, float /*height*/) {}

// Find an archive (soh.o2r / oot.o2r) given how this tool is launched. run.sh-style
// usage cds into build-cmake/soh; the binary itself lands in build-cmake/soh/charcompare.
// Search cwd, the executable's dir, and one level up (the soh build dir).
static std::string locateArchive(const std::string& name, const char* argv0) {
    std::vector<fs::path> dirs;
    dirs.push_back(fs::current_path());
    if (argv0) {
        std::error_code ec;
        fs::path exe = fs::weakly_canonical(fs::path(argv0), ec);
        if (!ec) {
            dirs.push_back(exe.parent_path());        // .../charcompare/
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

    std::vector<std::string> archivePaths;
    std::string soh = locateArchive("soh.o2r", argv[0]);
    std::string oot = locateArchive("oot.o2r", argv[0]);
    if (!soh.empty()) archivePaths.push_back(soh);
    if (!oot.empty()) archivePaths.push_back(oot);
    if (soh.empty())
        fprintf(stderr, "[charcompare] warning: soh.o2r not found (GUI font may be missing)\n");
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

    printf("[charcompare] window backend: %s (%ux%u)\n", window->GetWindowBackendName().c_str(),
           window->GetWidth(), window->GetHeight());

    auto gui = window->GetGui();
    while (window->IsRunning()) {
        window->HandleEvents();
        if (!window->IsFrameReady()) {
            continue;
        }
        gui->StartDraw();
        window->StartFrame();
        window->RunGuiOnly();

        ImGui::SetNextWindowSize(ImVec2(360, 140), ImGuiCond_FirstUseEver);
        ImGui::Begin("CharCompare");
        ImGui::Text("Phase 1: libultraship standalone window is up.");
        ImGui::Text("Window: %ux%u  backend: %s", window->GetWidth(), window->GetHeight(),
                    window->GetWindowBackendName().c_str());
        ImGui::Text("Next: 3DS viewport, N64 viewport, selectors.");
        ImGui::End();

        gui->EndDraw();
        window->EndFrame();
    }

    printf("[charcompare] window closed, exiting\n");
    Ship::Context::DestroyInstance();
    return 0;
}
