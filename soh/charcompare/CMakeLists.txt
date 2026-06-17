# charcompare — standalone N64-vs-3DS character comparison tool.
#
# Embeds libultraship (Fast3D) for the N64 side and compiles the self-contained
# soh3d asset/ parsers (no engine deps) for the 3DS side. Added via
# add_subdirectory(charcompare) at the END of soh/CMakeLists.txt, so the SDL2 /
# Threads imported targets that soh's find_package() created are already visible.
#
# Build:  cmake --build Shipwright/build-cmake --target charcompare
# (New files need a reconfigure: cmake Shipwright/build-cmake)

set(SOH3D_ASSET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src/soh3d/asset)

add_executable(charcompare
    main.cpp
    ${SOH3D_ASSET_DIR}/ctr_rom.cpp
    ${SOH3D_ASSET_DIR}/zar.cpp
    ${SOH3D_ASSET_DIR}/cmb.cpp
    ${SOH3D_ASSET_DIR}/csab.cpp
    ${SOH3D_ASSET_DIR}/pica_texture.cpp
)

target_include_directories(charcompare PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libultraship/include
)

# libultraship is PUBLIC-linked against ImGui + the GL backend, so linking it
# transitively gives us <imgui.h> and the SDL/GL headers/libs we need.
target_link_libraries(charcompare PRIVATE
    libultraship
    SDL2::SDL2
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

# ENABLE_OPENGL guards the GL code paths we reuse (matches the libultraship build).
target_compile_definitions(charcompare PRIVATE ENABLE_OPENGL)

add_dependencies(charcompare libultraship)
