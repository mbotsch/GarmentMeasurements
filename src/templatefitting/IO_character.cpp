#include "IO_character.h"

#include <filesystem>

#include "Character.h"
#include "FBX_file.h"

namespace template_fitting {

bool read_fbx(const std::filesystem::path& path, Character* character)
{
    FBX_file fbx_file;
    return fbx_file.read(path, character);
}

}
