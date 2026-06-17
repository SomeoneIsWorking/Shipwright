// charcompare — N64-vs-3DS character comparison tool.
//
// One window, two viewports (left = N64 via libultraship Fast3D, right = OoT3D/3DS
// via the soh3d asset parsers + the engine's direct-GL skinned renderer), showing the
// SAME character + (mapped) animation, to drive the N64<->3DS anim-map curation.
//
// Phase 4: cascading selectors (TYPE -> character -> ANIMATION) driven by the generated
// character index (cc_index.h / charcompare_index.inc, from tools/skeldata/animmap.json).
// Selecting a character loads its 3DS ZAR (right) and N64 object+skeleton (left); selecting
// an animation plays the N64 anim and its best-matched 3DS CSAB side by side.

#include <fast/Fast3dWindow.h>
#include <fast/interpreter.h>
#include <ship/Context.h>

#include <imgui.h>

#include <SDL2/SDL.h>
#define GL_GLEXT_PROTOTYPES 1
#include <SDL2/SDL_opengl.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "cc_3ds.h"
#include "cc_n64.h"
#include "cc_index.h"

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

// --- charcompare selection state -----------------------------------------------------------
// The generated index is a flat array sorted by category then name. We build the list of unique
// categories and, on demand, the list of entry indices within the selected category.
struct AppState {
    std::vector<std::string> categories;
    int catSel = 0, entrySel = 0, animSel = 0;
    cc::Model3ds model;            // current 3DS model (right)
    cc::ModelN64 n64;              // current N64 model (left)
    float frame = 0.0f;
    bool playing = true;
    float playSpeed = 0.5f;        // frames per render-frame
    float rx = 0, ry = 180, rz = 0; // face the camera (characters author facing +Z away)
};

static std::vector<int> entriesInCategory(const std::string& cat) {
    std::vector<int> v;
    const cc::IndexEntry* idx = cc::CcIndex();
    for (int i = 0; i < cc::CcIndexCount(); i++)
        if (cat == idx[i].category) v.push_back(i);
    return v;
}

// Load the currently selected character into both viewports and reset the animation.
static void loadSelection(AppState& s) {
    auto es = entriesInCategory(s.categories[s.catSel]);
    if (es.empty()) return;
    s.entrySel = std::clamp(s.entrySel, 0, (int)es.size() - 1);
    const cc::IndexEntry& e = cc::CcIndex()[es[s.entrySel]];

    s.model = cc::Load(e.zar);
    if (!s.model.ok) fprintf(stderr, "[charcompare] 3DS load failed (%s): %s\n", e.zar, s.model.error.c_str());

    std::vector<std::string> animSyms;
    for (int a = 0; a < e.animCount; a++) animSyms.push_back(e.anims[a].n64);
    s.n64 = cc::LoadN64Auto(std::string("objects/") + e.object, animSyms);
    if (!s.n64.ok) fprintf(stderr, "[charcompare] N64 load failed (%s): %s\n", e.object, s.n64.error.c_str());

    s.animSel = 0;
    s.frame = 0.0f;
}

// Apply the selected animation to both models for the current frame.
static void applyAnim(AppState& s) {
    auto es = entriesInCategory(s.categories[s.catSel]);
    if (es.empty()) return;
    const cc::IndexEntry& e = cc::CcIndex()[es[s.entrySel]];
    if (e.animCount == 0) return;
    s.animSel = std::clamp(s.animSel, 0, e.animCount - 1);
    const cc::IndexAnim& a = e.anims[s.animSel];
    cc::SetAnimN64(s.n64, a.n64);
    cc::SetAnim(s.model, a.csab && a.csab[0] ? a.csab : "", s.frame);
}

int main(int argc, char** argv) {
    printf("[charcompare] starting\n");

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

    // Build the category list (the index is pre-sorted by category, so first-seen order is stable).
    AppState st;
    const cc::IndexEntry* idx = cc::CcIndex();
    for (int i = 0; i < cc::CcIndexCount(); i++)
        if (std::find(st.categories.begin(), st.categories.end(), idx[i].category) == st.categories.end())
            st.categories.push_back(idx[i].category);
    if (st.categories.empty()) { fprintf(stderr, "[charcompare] empty character index\n"); return 1; }

    // Optional starting character: argv[1] = a ZAR path; otherwise default to ge1 if present.
    std::string startZar = (argc > 1) ? argv[1] : "/actor/zelda_ge1.zar";
    for (int i = 0; i < cc::CcIndexCount(); i++) {
        if (startZar == idx[i].zar) {
            auto it = std::find(st.categories.begin(), st.categories.end(), idx[i].category);
            st.catSel = (int)(it - st.categories.begin());
            auto es = entriesInCategory(idx[i].category);
            st.entrySel = (int)(std::find(es.begin(), es.end(), i) - es.begin());
            break;
        }
    }
    loadSelection(st);

    // Headless verification hooks.
    std::string shotPath = getenv("CC_SHOT") ? getenv("CC_SHOT") : "";
    int shotFrame = getenv("CC_SHOT_FRAME") ? atoi(getenv("CC_SHOT_FRAME")) : 120;
    long frameCount = 0;

    // Single-model diagnostics (full screen) vs the default side-by-side split.
    static const bool n64Only = getenv("CC_N64") != nullptr;
    static const bool ds3Only = getenv("CC_3DS") != nullptr;
    static const bool noGui = getenv("CC_NOGUI") != nullptr;
    const cc::Rect full{ 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT };
    const cc::Rect leftHalf{ 0, 0, SCREEN_WIDTH / 2, SCREEN_HEIGHT };
    const cc::Rect rightHalf{ SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, SCREEN_HEIGHT };

    auto gui = window->GetGui();
    while (window->IsRunning()) {
        window->HandleEvents();
        if (!window->IsFrameReady()) {
            continue;
        }

        if (st.playing) st.frame += st.playSpeed;
        applyAnim(st);

        // Build this frame's display list: N64 (Fast3D) left, 3DS (SoH3D) right, in one Run.
        std::vector<Gfx> dl;
        std::unordered_map<Mtx*, MtxF> mtx;
        cc::DlistKeys keys;
        if (n64Only) {
            cc::EmitDlistN64(st.n64, st.frame, dl, mtx, keys, st.rx, st.ry, st.rz, full);
        } else if (ds3Only) {
            cc::EmitDlist(st.model, dl, mtx, keys, st.rx, st.ry, st.rz, full);
        } else {
            // N64 limbs first (Fast3D), then the 3DS draw + render pass last so it composites on top.
            if (st.n64.ok) cc::EmitDlistN64(st.n64, st.frame, dl, mtx, keys, st.rx, st.ry, st.rz, leftHalf);
            if (st.model.ok) cc::EmitDlist(st.model, dl, mtx, keys, st.rx, st.ry, st.rz, rightHalf);
        }
        Gfx end = gsSPEndDisplayList();
        dl.push_back(end);

        gui->StartDraw();
        window->StartFrame();
        interp->Run(dl.data(), mtx);

        if (!noGui) {
            // Cascading selectors: TYPE -> character -> ANIMATION. A top strip keeps the two
            // model halves visible (it's draggable if it overlaps a head).
            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2((float)window->GetWidth(), 120), ImGuiCond_FirstUseEver);
            ImGui::Begin("CharCompare  (N64 left | 3DS right)");

            auto es = entriesInCategory(st.categories[st.catSel]);
            const cc::IndexEntry& e = cc::CcIndex()[es.empty() ? 0 : es[std::clamp(st.entrySel, 0, (int)es.size() - 1)]];

            // TYPE
            ImGui::SetNextItemWidth(140);
            if (ImGui::BeginCombo("type", st.categories[st.catSel].c_str())) {
                for (int i = 0; i < (int)st.categories.size(); i++) {
                    bool sel = (i == st.catSel);
                    if (ImGui::Selectable(st.categories[i].c_str(), sel)) {
                        st.catSel = i;
                        st.entrySel = 0;
                        loadSelection(st);
                    }
                    if (sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            // CHARACTER
            ImGui::SameLine();
            ImGui::SetNextItemWidth(160);
            if (ImGui::BeginCombo("character", e.name)) {
                for (int i = 0; i < (int)es.size(); i++) {
                    bool sel = (i == st.entrySel);
                    if (ImGui::Selectable(cc::CcIndex()[es[i]].name, sel)) {
                        st.entrySel = i;
                        loadSelection(st);
                    }
                    if (sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            // ANIMATION (label shows N64 anim -> mapped 3DS CSAB)
            ImGui::SameLine();
            ImGui::SetNextItemWidth(260);
            const char* curAnim = (e.animCount > 0) ? e.anims[std::clamp(st.animSel, 0, e.animCount - 1)].n64 : "(none)";
            if (ImGui::BeginCombo("anim", curAnim)) {
                for (int i = 0; i < e.animCount; i++) {
                    bool sel = (i == st.animSel);
                    char lbl[256];
                    snprintf(lbl, sizeof(lbl), "%s  ->  %s", e.anims[i].n64,
                             e.anims[i].csab[0] ? e.anims[i].csab : "(no csab)");
                    if (ImGui::Selectable(lbl, sel)) {
                        st.animSel = i;
                        st.frame = 0.0f;
                    }
                    if (sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            ImGui::Text("%s  N64:%s  3DS-csab:%s", e.zar, st.n64.ok ? st.n64.skelName.c_str() : "FAIL",
                        (e.animCount > 0 && e.anims[std::clamp(st.animSel, 0, e.animCount - 1)].csab[0])
                            ? e.anims[std::clamp(st.animSel, 0, e.animCount - 1)].csab
                            : "-");
            if (!st.n64.ok) ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "N64: %s", st.n64.error.c_str());
            if (!st.model.ok) ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "3DS: %s", st.model.error.c_str());

            ImGui::Checkbox("play", &st.playing);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("speed", &st.playSpeed, 0.0f, 2.0f);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(160);
            ImGui::SliderFloat("frame", &st.frame, 0.0f, 200.0f);
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("rotY", &st.ry, -180, 180);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("rotX", &st.rx, -180, 180);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("rotZ", &st.rz, -180, 180);
            ImGui::End();
        }

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
