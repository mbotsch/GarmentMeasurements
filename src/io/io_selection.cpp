#include "io_selection.h"

#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

// =====================================================================================================================

using json = nlohmann::json;

// =====================================================================================================================

bool read_selection(const std::string& fn_selection, std::vector<unsigned int>& vertex_ids)
{
    std::ifstream index_file(fn_selection);

    if (!index_file)
    {
        printf("[ERROR] Cannot read %s for reading vertex selection\n", fn_selection.c_str());
        return false;
    }

    std::string line;
    while (std::getline(index_file, line))
    {
        int index;
        std::stringstream ss(line);
        ss >> index;

        if (!ss.fail())
        {
            vertex_ids.push_back(index);
        }
    }
    index_file.close();

    return true;
}

bool read_selection(const std::string& fn_selection, std::vector<int>& vertex_ids)
{
    std::ifstream index_file(fn_selection);

    if (!index_file)
    {
        printf("[ERROR] Cannot read %s for reading vertex selection\n", fn_selection.c_str());
        return false;
    }

    std::string line;
    while (std::getline(index_file, line))
    {
        int index;
        std::stringstream ss(line);
        ss >> index;

        if (!ss.fail())
        {
            vertex_ids.push_back(index);
        }
    }
    index_file.close();

    return true;
}

bool write_selection(const std::string& fn_selection, const std::vector<int>& vertex_ids)
{
    std::ofstream ofs(fn_selection);

    if (!ofs)
    {
        printf("[ERROR] Cannot open %s for writing\n", fn_selection.c_str());
        return false;
    }

    for (int id : vertex_ids)
    {
        ofs << id << std::endl;
    }

    ofs.close();

    return true;
}


