#include <cstdio>
#include <cfloat>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>

#include <nlohmann/json.hpp>

int main(int argc, char** argv)
{
    namespace fs = std::filesystem;
    using json = nlohmann::json;

    if (argc < 2)
    {
        fprintf(stderr, "[USAGE] %s folder_with_meta_files\n", argv[0]);
        return -1;
    }

    fs::path input_folder(argv[1]);

    struct extremes {
        int min_index = -1;
        int max_index = -1;
        float min_val = FLT_MAX;
        float max_val = -FLT_MAX;
    };

    const int num_dims_to_search = 5;
    extremes exs[num_dims_to_search] = {};

    for (const fs::directory_entry& entry : fs::directory_iterator(input_folder))
    {
        fs::path ext = entry.path().extension();

        if (ext == ".json")
        {
            std::ifstream ifs(entry.path());
            json meta_json = json::parse(ifs);
            std::vector<double> pca_params = meta_json.value("pca_weights", std::vector<double>());

            int index = std::stoi(entry.path().filename());

            for (int i = 0; i < num_dims_to_search; ++i)
            {
                double p = pca_params[i];
                if (exs[i].min_val > p)
                {
                    exs[i].min_val = p;
                    exs[i].min_index = index;
                }

                if (exs[i].max_val < p)
                {
                    exs[i].max_val = p;
                    exs[i].max_index = index;
                }
            }
        }
    }

    for (int i = 0; i < num_dims_to_search; ++i)
    {
        std::cout << "Dim " << i << std::endl;
        std::cout << "Max: " << exs[i].max_val << " at " << std::setw(5) << std::setfill('0') << exs[i].max_index << std::endl;
        std::cout << "Min: " << exs[i].min_val << " at " << std::setw(5) << std::setfill('0') << exs[i].min_index << std::endl;
    }
    fprintf(stderr, "Hi!\n");
    return 0;
}
