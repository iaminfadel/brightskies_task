// Small self-contained SHA-256 used to verify the saved map and bundle.
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace brightdrive {

std::array<std::uint8_t, 32> sha256(const std::uint8_t* data, std::size_t size);

std::string sha256_file_hex(const std::string& path);

std::string to_hex(const std::array<std::uint8_t, 32>& digest);

}  // namespace brightdrive
