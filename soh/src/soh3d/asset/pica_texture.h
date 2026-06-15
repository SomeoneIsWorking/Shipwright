// Decode PICA200 (3DS) textures to RGBA8. Port of tools/pica_texture.py
// (itself a port of noclip oot3d/pica_texture.ts). Pure C++ (no SoH/LUS deps).
#pragma once
#include <cstdint>
#include <vector>

namespace SoH3D {

// Decode raw texture bytes of the given glFormat ((dataType<<16)|formatConstant)
// into width*height*4 RGBA8 bytes. Returns empty on unknown/short input.
std::vector<uint8_t> PicaDecode(uint32_t glFormat, int width, int height, const std::vector<uint8_t>& data);

} // namespace SoH3D
