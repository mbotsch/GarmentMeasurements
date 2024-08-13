#include "Viewer.h"

#include <random>
#include <sstream>

#include <imgui.h>
#include <nlohmann/json.hpp>

#include <pmp/algorithms/utilities.h>
#include <pmp/io/io.h>

#include <character/CharacterRenderer.h>
#include <character/Mean_value_coordinates.h>
#include <fstream>

using json = nlohmann::json;

Viewer::Viewer(const char* title, int width, int height)
    : TrackballViewer(title, width, height)
{
    // setup draw modes
    clear_draw_modes();
    add_draw_mode("Points");
    add_draw_mode("Hidden Line");
    add_draw_mode("Smooth Shading");
    add_draw_mode("Texture");
    set_draw_mode("Smooth Shading");

    // Debug load fbx file
    std::string fn_character_fbx = "../data/template/male.fbx";
    if (!character_.read(fn_character_fbx))
    {
        fprintf(stderr, "[ERROR] Cannot read %s\n", fn_character_fbx.c_str());
    }
    else
    {
        if (!character_.set_selected_skin("skin"))
        {
            if (!character_.set_selected_skin("H_DDS_HighResShape"))
            {
                fprintf(stderr, "[ERROR] Cannot select main skin mesh\n");
            }
        }

        character_.only_keep_selected_skin();

        character::mvc::compute_weights_from_current_joint_positions(&character_);

        pmp::BoundingBox bbox;
        for (const auto& mesh : character_.meshes()) { bbox += bounds(mesh.mesh_); }
        set_scene(bbox.center(), bbox.size() * 0.5);

        renderer_ = new character::CharacterRenderer(character_);
        renderer_->update_opengl_buffers();

        joint_angles_.resize(3 * character_.skeleton().joints_.size(), 0);

        // Set default pose:
        //   narrower legs
        //   no bend at elbow

        character::Skeleton& skeleton = character_.skeleton();

        size_t thigh_L_index = skeleton.get_joint_idx("thigh_L");
        size_t thigh_R_index = skeleton.get_joint_idx("thigh_R");
        size_t elbow_L_index = skeleton.get_joint_idx("lower_arm_01_L");
        size_t elbow_R_index = skeleton.get_joint_idx("lower_arm_01_R");

        // joint angles in [x, y, z] serialized vector
//        joint_angles_[3 * thigh_L_index + 1] =  12.0f;
//        joint_angles_[3 * thigh_R_index + 1] = -12.0f;

//        joint_angles_[3 * thigh_L_index + 2] =  6.2f;
//        joint_angles_[3 * thigh_R_index + 2] = -6.2f;

//        joint_angles_[3 * elbow_L_index + 0] = -25.0f;
//        joint_angles_[3 * elbow_R_index + 0] = -25.0f;

        // call this after changing skinning parameters
        evaluate_pose();
    }

    if (!pca_.load("../data/pca/point.pca"))
    {
        fprintf(stderr, "[ERROR] Cannot load PCA\n");
    }
    else
    {
        shape_parameters_.resize(pca_.components(), 0.0f);
        // call this after changing PCA weights
        evaluate_shape();
    }

}

Viewer::~Viewer()
= default;

void Viewer::draw(const std::string& drawMode)
{
    renderer_->draw(projection_matrix_, modelview_matrix_, drawMode);
}

void Viewer::evaluate_pose()
{
    std::vector<character::Joint*>& joints = character_.skeleton().joints_;

    size_t num_joints = joints.size();
    std::vector<pmp::mat4> pose_vector(num_joints, pmp::mat4::identity());

    for (size_t i = 0; i < num_joints; ++i)
    {
        pose_vector[i] = pmp::rotation_matrix_z(joint_angles_[3 * i + 2]) *
                         pmp::rotation_matrix_y(joint_angles_[3 * i + 1]) *
                         pmp::rotation_matrix_x(joint_angles_[3 * i + 0]);
    }

    character_.skeleton().set_pose_vector(pose_vector);
    character_.set_opengl_update_required();
}

void Viewer::evaluate_shape()
{
    Eigen::VectorXd pca_params(shape_parameters_.size());
    for (size_t i = 0; i < shape_parameters_.size(); ++i)
    {
        pca_params[i] = shape_parameters_[i];
    }

    Eigen::VectorXd pca_result = pca_.evaluate(pca_.scale_parameters(pca_params));

    character::SkinnedMesh& mesh = character_.get_selected_skin();
    size_t n_vertices = mesh.mesh_.n_vertices();

    std::vector<pmp::vec3>& positions = mesh.mesh_.positions();

    for (size_t i = 0; i < n_vertices; ++i)
    {
        positions[i] = pca_result.segment<3>(3 * i);
    }

    character_.on_shape_changed();
    character_.set_opengl_update_required();
}

void Viewer::sample_shape()
{
    // generates a batch of mesh samples
    // sample normal distributed for pca weights
    std::normal_distribution normal_dist { 0.0F, stddev_ }; // mean and stddev

    std::uniform_real_distribution<float> uniform_dist { -15.0F, 15.0F };
    std::random_device random_device {};
    std::mt19937 random_generator { random_device() };

    // sample normal distributed
    for (float & shape_parameter : shape_parameters_)
    {
        shape_parameter = static_cast<float> (normal_dist(random_generator));
    }

    // indices for joint angles
    character::Skeleton& skeleton = character_.skeleton();
    size_t upper_arm_L_index = skeleton.get_joint_idx("upper_arm_L");
    size_t upper_arm_R_index = skeleton.get_joint_idx("upper_arm_R");

    // sample uniform distributed arm angle
    arm_inclination_ = uniform_dist(random_generator);
    joint_angles_[upper_arm_L_index * 3 + 2] = 0.0F + arm_inclination_;
    joint_angles_[upper_arm_R_index * 3 + 2] = 0.0F - arm_inclination_;

    // generate mesh
    evaluate_shape();
    evaluate_pose();
}

void Viewer::batch_sample_shapes()
{
    // create output dir
    std::filesystem::path output_dir { "output" };
    std::filesystem::path mesh_dir { output_dir / "meshes" };
    std::filesystem::path meta_dir { output_dir / "meta" };
    std::filesystem::create_directory(output_dir);
    std::filesystem::create_directory(mesh_dir);
    std::filesystem::create_directory(meta_dir);

    character::Skeleton& skeleton = character_.skeleton();
    size_t thigh_L_index = skeleton.get_joint_idx("thigh_L");
    size_t thigh_R_index = skeleton.get_joint_idx("thigh_R");

    float max_pca_weight = 0.0f;

    for (int index = 0; index < num_batch_samples; ++index) {
        // filename
        std::stringstream ss_file_number;
        ss_file_number << std::setw(5) << std::setfill('0') << index;
        std::string file_number = ss_file_number.str();

        printf("%d / %d     \r", index, num_batch_samples);
        fflush(stdout);

        // sample a new person
        sample_shape();

        for (const float& w : shape_parameters_)
        {
            max_pca_weight = std::max(max_pca_weight, std::abs(w));
        }

        // save metadata
        json meta_data = {
            {"arm_inclination", arm_inclination_},
            {"pca_weights", shape_parameters_}
        };
        try {
            std::ofstream ofs(meta_dir / (file_number + "_meta.json"));
            ofs << std::setw(2) << meta_data << std::endl;
        } catch (std::exception& error) {
            std::cerr << error.what();
        }

        // legs apart
        joint_angles_[3 * thigh_L_index + 2] =  2.2f;
        joint_angles_[3 * thigh_R_index + 2] = -2.2f;
        evaluate_pose();
        save_mesh(mesh_dir / (file_number + "_apart.obj"));

        // legs straight
        joint_angles_[3 * thigh_L_index + 2] =  6.2f;
        joint_angles_[3 * thigh_R_index + 2] = -6.2f;
        evaluate_pose();
        save_mesh(mesh_dir / (file_number + "_straight.obj"));
    }
    printf("\n");

    fprintf(stderr, "[DEBUG] Max pca weight: %f\n", max_pca_weight);
}

void Viewer::save_mesh(const std::string& filename)
{
    // Skinning is only done on GPU for rendering,
    // actually create CPU version of skinned vertices
    character_.apply_skinning();

    // Store bindpose positions
    character::SkinnedMesh result_mesh = character_.get_selected_skin();
    std::vector<pmp::vec3> pos_backup = result_mesh.mesh_.positions();

    // Swap bindpose positions with skinned positions
    auto skinned_pos = result_mesh.get_skinned_pos();
    result_mesh.mesh_.positions() = skinned_pos.vector();

    pmp::write(result_mesh.mesh_, filename);

    // Swap back
    result_mesh.mesh_.positions() = pos_backup;
}

void Viewer::process_imgui()
{
    // ImGui::ShowDemoWindow();
    if (ImGui::CollapsingHeader("Output", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::Button("Save skinned mesh"))
        {
            save_mesh("output.obj");
        }

        if (ImGui::Button("Random sample")) {
            sample_shape();
        }

        ImGui::NewLine();
        ImGui::SliderInt("Number of samples", &num_batch_samples, 1, 10000);
        ImGui::SliderFloat("Stddev of pca weights", &stddev_, 0.5f, 2.0f);
        if (ImGui::Button("Batch generate and save")) {
            batch_sample_shapes();
        }
    }

    if (ImGui::CollapsingHeader("Shape Control", ImGuiTreeNodeFlags_DefaultOpen))
    {
        bool shape_parameters_changed = false;
        for (size_t i = 0; i < shape_parameters_.size(); ++i)
        {
            std::string s = "Param " + std::to_string(i);
            if (ImGui::SliderFloat(s.c_str(), &shape_parameters_[i], -3.0f, 3.0f, "%.2f"))
            {
                shape_parameters_changed = true;
            }
        }

        if (shape_parameters_changed)
        {
            evaluate_shape();
        }
    }

    if (ImGui::CollapsingHeader("Pose Control"))
    {
        std::vector<character::Joint*>& joints = character_.skeleton().joints_;
        size_t num_joints = joints.size();

        bool angles_changed = false;
        for (size_t i = 0; i < num_joints; ++i)
        {
            const std::string& joint_name = joints[i]->name_;
            if (ImGui::SliderFloat3(joint_name.c_str(), &joint_angles_[3 * i], -90.0f, 90.0f, "%.1f"))
            {
                angles_changed = true;
            }
        }

        if (angles_changed)
        {
            evaluate_pose();
        }
    }
}
