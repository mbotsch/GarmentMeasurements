#pragma once

#include <filesystem>

namespace template_fitting {

class Character;

bool read_bim(const std::filesystem::path& path, Character* character);
bool read_fbx(const std::filesystem::path& path, Character* character);

}
