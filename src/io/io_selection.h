#pragma once

#include <string>
#include <vector>


bool read_selection(const std::string& fn_selection, std::vector<unsigned int>& vertex_ids);

bool read_selection(const std::string& fn_selection, std::vector<int>& vertex_ids);

bool write_selection(const std::string& fn_selection, const std::vector<int>& vertex_ids);

