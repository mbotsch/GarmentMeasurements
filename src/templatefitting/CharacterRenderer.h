#pragma once

#include <GL/glew.h>

#include <pmp/mat_vec.h>
#include "pmp/visualization/shader.h"

namespace template_fitting { class Character; }

namespace template_fitting {

class CharacterRenderer
{
public:
    CharacterRenderer(Character& character);
    ~CharacterRenderer();

    void update_opengl_buffers();

    void draw(const pmp::mat4& projection_matrix, const pmp::mat4& modelview_matrix,
              const std::string& draw_mode);

    void draw_skeleton(const pmp::mat4& projection_matrix, const pmp::mat4& modelview_matrix);

private:

    void create_opengl_buffers();
    void delete_opengl_buffers();

    void update_skeleton_opengl_buffers();

    Character& character_;

    GLuint vao_ = 0;
    GLuint vertex_buffer_ = 0;
    GLuint normal_buffer_ = 0;
    GLuint weights1_buffer_ = 0;
    GLuint weights2_buffer_ = 0;
    GLuint depends1_buffer_ = 0;
    GLuint depends2_buffer_ = 0;
    GLuint skinning_matrix_buffer_ = 0;

    // Buffer for local coordinate systems of joints
    GLuint skeleton_vao_ = 0;
    GLuint skeleton_cs_buffer_ = 0;
    GLuint skeleton_cs_color_buffer_ = 0;

    size_t n_vertices_ = 0;
    size_t n_skeleton_vertices_ = 0;

    pmp::Shader character_shader_;
    pmp::Shader skeleton_shader_;

    float ambient_   = 0.1;
    float diffuse_   = 0.8;
    float specular_  = 0.6;
    float shininess_ = 100.0;
    float alpha_     = 1.0;

    float point_size_ = 5.0;

    pmp::vec3 front_color_ = pmp::vec3(0.6, 0.6, 0.6);
    pmp::vec3 back_color_ = pmp::vec3(0.5, 0.0, 0.0);

    bool draw_skeleton_ = true;
};

}
