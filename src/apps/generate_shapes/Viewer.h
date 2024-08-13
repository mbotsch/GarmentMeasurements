#pragma once

#include <pmp/surface_mesh.h>
#include <pmp/visualization/renderer.h>
#include <pmp/visualization/trackball_viewer.h>

#include <character/Character.h>
#include <character/PCA.h>

namespace character { class CharacterRenderer; }

class Viewer : public pmp::TrackballViewer
{
public:
    Viewer(const char* title, int width, int height);
    ~Viewer();

    virtual void draw(const std::string& drawMode) override;
    virtual void process_imgui() override;

private:

    void evaluate_pose();
    void evaluate_shape();

    void sample_shape();
    void batch_sample_shapes();

    void save_mesh(const std::string& filename);

    character::Character character_;
    character::CharacterRenderer* renderer_ = nullptr;

    PCA pca_;

    float stddev_ = 0.7;

    std::vector<float> joint_angles_;
    std::vector<float> shape_parameters_;
    int num_batch_samples = 100;

    // arm angle from generator
    float arm_inclination_ = 0.0F;
};
