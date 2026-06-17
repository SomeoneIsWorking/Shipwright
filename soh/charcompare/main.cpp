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
#include <ship/config/Config.h>

#include <imgui.h>

#include <SDL2/SDL.h>
#define GL_GLEXT_PROTOTYPES 1
#include <SDL2/SDL_opengl.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
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

// Interpreter model-space bbox measure (libultraship): the N64 viewport frames its DEPTH axis by the
// model's true geometry extent (not the joint bbox). We measure once per loaded N64 model — wrap the
// first frame's interp->Run with these so the Fast3D vertex transform records the bbox.
extern "C" void Cc_BboxMeasureBegin();
extern "C" void Cc_BboxMeasureEnd(float* mn, float* mx);

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
    // 3/4 view (slightly turned from dead-front): a far better default than ry=180 — front-on hides
    // depth and makes wing/cape/weapon spread look like a flat "kite". CC_ROTX/Y/Z override.
    float rx = 0, ry = 150, rz = 0;
    // Hand-curated N64-anim -> 3DS-CSAB corrections. Key = "<zar>|<n64anim>", value = chosen CSAB.
    // Overrides the generated best-match; persisted to overridesPath so the next run + the index
    // generator (gen_charcompare_index.py reads it) pick up the corrections.
    std::map<std::string, std::string> overrides;
    std::string overridesPath;
    std::string saveMsg;
};

static std::string overrideKey(const char* zar, const char* n64) {
    return std::string(zar) + "|" + n64;
}

// Curation file lives at <repo>/tools/skeldata/charcompare_overrides.tsv so it travels with the
// data and the index generator can read it. Derive the repo from the executable path
// (<repo>/Shipwright/build-cmake/soh/charcompare/charcompare); fall back to the cwd.
static std::string locateOverridesPath(const char* argv0) {
    if (const char* e = getenv("CC_OVERRIDES")) return e;
    std::error_code ec;
    fs::path exe = fs::weakly_canonical(fs::path(argv0 ? argv0 : ""), ec);
    if (!ec) {
        fs::path repo = exe.parent_path().parent_path().parent_path().parent_path().parent_path();
        fs::path p = repo / "tools" / "skeldata";
        if (fs::exists(p)) return (p / "charcompare_overrides.tsv").string();
    }
    return "charcompare_overrides.tsv";
}

static void loadOverrides(AppState& s) {
    std::ifstream f(s.overridesPath);
    if (!f) return;
    std::string line;
    while (std::getline(f, line)) {
        size_t t1 = line.find('\t'), t2 = (t1 == std::string::npos) ? t1 : line.find('\t', t1 + 1);
        if (t1 == std::string::npos || t2 == std::string::npos) continue;
        s.overrides[line.substr(0, t1) + "|" + line.substr(t1 + 1, t2 - t1 - 1)] = line.substr(t2 + 1);
    }
    fprintf(stderr, "[charcompare] loaded %zu anim overrides from %s\n", s.overrides.size(), s.overridesPath.c_str());
}

static void saveOverrides(AppState& s) {
    std::ofstream f(s.overridesPath);
    if (!f) { s.saveMsg = "SAVE FAILED: " + s.overridesPath; return; }
    for (const auto& [k, v] : s.overrides) {
        size_t bar = k.find('|');
        f << k.substr(0, bar) << '\t' << k.substr(bar + 1) << '\t' << v << '\n';
    }
    s.saveMsg = "saved " + std::to_string(s.overrides.size()) + " -> " + s.overridesPath;
}

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

    // Print the available 3DS CSABs (the override candidates) so the CLI/curation loop can see them.
    fprintf(stderr, "[charcompare] %s 3DS CSABs (%zu):", e.zar, s.model.anims.size());
    for (const auto& cs : s.model.anims) fprintf(stderr, " %s", cs.c_str());
    fprintf(stderr, "\n");

    // Default to an IDLE anim, not anims[0] — the first anim is often a dramatic cutscene/attack
    // (e.g. Saria's first = the arms-raised Seal-Ganon; a gerudo's = a combat crouch), which makes
    // the character pose look broken by default. Prefer a name that reads as idle/wait/stand.
    s.animSel = 0;
    static const char* idleKeys[] = { "neutral", "wait", "idle", "stand", "wai", "matsu", "w4" };
    for (int a = 0; a < e.animCount && s.animSel == 0; a++) {
        std::string n = e.anims[a].n64;
        std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return std::tolower(c); });
        for (const char* k : idleKeys)
            if (n.find(k) != std::string::npos) { s.animSel = a; break; }
    }
    s.frame = 0.0f;
}

// The 3DS CSAB currently used for an N64 anim: CC_CSAB (transient render override, for the
// AI-driven "render this pairing and screenshot it" loop) wins; then a saved hand override; then
// the generated best-match from the index.
static std::string effectiveCsab(const AppState& s, const cc::IndexEntry& e, const cc::IndexAnim& a) {
    static const char* forceCsab = getenv("CC_CSAB");
    if (forceCsab && forceCsab[0]) return forceCsab;
    auto it = s.overrides.find(overrideKey(e.zar, a.n64));
    if (it != s.overrides.end()) return it->second;
    return a.csab ? a.csab : "";
}

// The active animation's playback length (frames): the longer of the N64 anim and the mapped 3DS
// CSAB, so the GUI frame slider / wrap covers both. Each side wraps internally to its own length
// (N64 sampleAnim modulo animFrameCount; 3DS Csab::animFrame REPEATs), so a shared max just keeps
// the frame counter bounded without desyncing either. Returns 0 if neither side has a length.
static int activeAnimLen(const AppState& s) {
    auto es = entriesInCategory(s.categories[s.catSel]);
    if (es.empty()) return 0;
    const cc::IndexEntry& e = cc::CcIndex()[es[std::clamp(s.entrySel, 0, (int)es.size() - 1)]];
    int n64Len = s.n64.ok ? s.n64.animFrameCount : 0;
    int ds3Len = 0;
    if (e.animCount > 0) {
        int ai = std::clamp(s.animSel, 0, e.animCount - 1);
        ds3Len = cc::AnimLength(s.model, effectiveCsab(s, e, e.anims[ai]));
    }
    return std::max(n64Len, ds3Len);
}

// Apply the selected animation to both models for the current frame.
static void applyAnim(AppState& s) {
    auto es = entriesInCategory(s.categories[s.catSel]);
    if (es.empty()) return;
    const cc::IndexEntry& e = cc::CcIndex()[es[s.entrySel]];
    if (e.animCount == 0) return;
    s.animSel = std::clamp(s.animSel, 0, e.animCount - 1);
    const cc::IndexAnim& a = e.anims[s.animSel];
    std::string csab = effectiveCsab(s, e, a);
    cc::SetAnimN64(s.n64, a.n64);
    cc::SetAnim(s.model, csab.empty() ? "" : csab.c_str(), s.frame);
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

    // Window size: the default 640x480 is too small to read the side-by-side models. Force a much
    // larger 4:3 default (matches the N64 320x240 native aspect, so no stretch); CC_WIDTH/CC_HEIGHT
    // override. Fast3dWindow::Init reads Window.Width/Height from the config, so set it before
    // InitWindow. The interpreter's render resolution (mCurDimensions) is taken from this launch
    // size, so geometry rasterises at the full window resolution — not an upscaled 640x480.
    {
        int winW = getenv("CC_WIDTH") ? atoi(getenv("CC_WIDTH")) : 1280;
        int winH = getenv("CC_HEIGHT") ? atoi(getenv("CC_HEIGHT")) : 960;
        ctx->GetConfig()->SetInt("Window.Width", winW);
        ctx->GetConfig()->SetInt("Window.Height", winH);
        printf("[charcompare] window size %dx%d\n", winW, winH);
    }

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
    st.overridesPath = locateOverridesPath(argv[0]);
    loadOverrides(st);
    loadSelection(st);

    // Inspection rotation overrides (degrees) — view the models from any angle for diagnosis.
    if (const char* e = getenv("CC_ROTX")) st.rx = (float)atof(e);
    if (const char* e = getenv("CC_ROTY")) st.ry = (float)atof(e);
    if (const char* e = getenv("CC_ROTZ")) st.rz = (float)atof(e);

    // CLI/env driving for the AI-curation loop: CC_N64ANIM selects a specific N64 anim by symbol
    // (substring match); CC_CSAB (handled in effectiveCsab) forces the 3DS CSAB. Combined with
    // CC_NOGUI + CC_SHOT this renders one (character, N64 anim, 3DS CSAB) pairing headlessly.
    if (const char* wantAnim = getenv("CC_N64ANIM")) {
        auto es = entriesInCategory(st.categories[st.catSel]);
        if (!es.empty()) {
            const cc::IndexEntry& e = cc::CcIndex()[es[st.entrySel]];
            for (int i = 0; i < e.animCount; i++)
                if (std::string(e.anims[i].n64).find(wantAnim) != std::string::npos) { st.animSel = i; break; }
        }
    }

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
        // Wrap the frame counter to the active animation's length so playback loops instead of
        // running forever (and so the slider has a meaningful range). 0 = unknown length -> leave it.
        int animLen = activeAnimLen(st);
        if (animLen > 0 && st.frame >= (float)animLen) st.frame = fmodf(st.frame, (float)animLen);
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
        // One-time per-model measure: capture the N64 model-space mesh bbox so EmitDlistN64 can frame
        // the depth axis by true geometry (fixes the head/face z-fighting from a joint-bbox z-scale).
        bool measuring = st.n64.ok && !st.n64.meshMeasured;
        if (measuring) Cc_BboxMeasureBegin();
        interp->Run(dl.data(), mtx);
        if (measuring) {
            Cc_BboxMeasureEnd(st.n64.meshMin, st.n64.meshMax);
            if (st.n64.meshMax[0] >= st.n64.meshMin[0]) st.n64.meshMeasured = true; // valid bbox captured
        }

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
            // N64 ANIMATION (a "*" marks anims with a hand override)
            ImGui::SameLine();
            ImGui::SetNextItemWidth(240);
            int ai = (e.animCount > 0) ? std::clamp(st.animSel, 0, e.animCount - 1) : 0;
            const char* curAnim = (e.animCount > 0) ? e.anims[ai].n64 : "(none)";
            if (ImGui::BeginCombo("N64 anim", curAnim)) {
                for (int i = 0; i < e.animCount; i++) {
                    bool sel = (i == st.animSel);
                    bool ov = st.overrides.count(overrideKey(e.zar, e.anims[i].n64)) > 0;
                    char lbl[256];
                    snprintf(lbl, sizeof(lbl), "%s%s  ->  %s", ov ? "* " : "", e.anims[i].n64,
                             e.anims[i].csab[0] ? e.anims[i].csab : "(no csab)");
                    if (ImGui::Selectable(lbl, sel)) {
                        st.animSel = i;
                        st.frame = 0.0f;
                    }
                    if (sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            // 3DS CSAB override: pick ANY of this model's CSABs (or reset to the auto best-match).
            // Selecting one records an override for (this character, this N64 anim).
            ImGui::SameLine();
            ImGui::SetNextItemWidth(220);
            std::string key = (e.animCount > 0) ? overrideKey(e.zar, e.anims[ai].n64) : "";
            std::string eff = (e.animCount > 0) ? effectiveCsab(st, e, e.anims[ai]) : "";
            bool overridden = st.overrides.count(key) > 0;
            std::string preview = (overridden ? "* " : "") + (eff.empty() ? std::string("(none)") : eff);
            if (ImGui::BeginCombo("3DS csab", preview.c_str())) {
                const char* best = (e.animCount > 0 && e.anims[ai].csab[0]) ? e.anims[ai].csab : "(none)";
                char autoLbl[256];
                snprintf(autoLbl, sizeof(autoLbl), "<auto: %s>", best);
                if (ImGui::Selectable(autoLbl, !overridden)) {
                    st.overrides.erase(key);
                    st.frame = 0.0f;
                }
                for (const auto& cs : st.model.anims) {
                    bool sel = overridden && eff == cs;
                    if (ImGui::Selectable(cs.c_str(), sel)) {
                        st.overrides[key] = cs;
                        st.frame = 0.0f;
                    }
                    if (sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if (ImGui::Button("Save overrides")) saveOverrides(st);

            ImGui::Text("%s  N64:%s  3DS-csab:%s%s", e.zar, st.n64.ok ? st.n64.skelName.c_str() : "FAIL",
                        eff.empty() ? "-" : eff.c_str(), overridden ? "  (override)" : "");
            if (!st.saveMsg.empty()) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 1, 0.5f, 1), "[%s]", st.saveMsg.c_str());
            }
            if (!st.n64.ok) ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "N64: %s", st.n64.error.c_str());
            if (!st.model.ok) ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "3DS: %s", st.model.error.c_str());

            ImGui::Checkbox("play", &st.playing);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::SliderFloat("speed", &st.playSpeed, 0.0f, 2.0f);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(160);
            // Slider range = the active animation length (N64 / 3DS, whichever is longer); the
            // "frame" label shows the count so the curation loop can read it.
            float frameMax = (animLen > 0) ? (float)animLen : 200.0f;
            char frameLbl[32];
            snprintf(frameLbl, sizeof(frameLbl), "frame (/%d)", animLen);
            ImGui::SliderFloat(frameLbl, &st.frame, 0.0f, frameMax);
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
