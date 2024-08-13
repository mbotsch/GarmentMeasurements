#include "CharacterRenderer.h"

#include <pmp/bounding_box.h>
#include <pmp/algorithms/normals.h>
#include <pmp/algorithms/utilities.h>
#include <pmp/visualization/gl.h>

#include "Character.h"
#include "Character_shader.h"
#include "Skeleton_shader.h"

namespace character {

CharacterRenderer::CharacterRenderer(Character& character)
    : character_(character)
{

}

CharacterRenderer::~CharacterRenderer()
{
    delete_opengl_buffers();
}

void CharacterRenderer::update_opengl_buffers()
{
    if (!vao_)
    {
        create_opengl_buffers();
    }

    glBindVertexArray(vao_);

    size_t n_vertices_to_upload = 0;
    for (SkinnedMesh& mesh : character_.meshes())
    {
        n_vertices_to_upload += mesh.mesh_.n_faces() * 3;
    }

    // Vertex positions
    std::vector<pmp::vec3> position_array;
    position_array.reserve(n_vertices_to_upload);
    for (const SkinnedMesh& mesh : character_.meshes())
    {
        for (pmp::Face f : mesh.mesh_.faces())
        {
            for (pmp::Vertex v : mesh.mesh_.vertices(f))
            {
                position_array.push_back(mesh.mesh_.position(v));
            }
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

    // Upload vertex positions
    if (!position_array.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBufferData(GL_ARRAY_BUFFER, position_array.size() * 3 * sizeof(float),
                     position_array.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
        n_vertices_ = position_array.size();
    }
    else
    {
        glDisableVertexAttribArray(0);
        n_vertices_ = 0;
    }

    // Normals
    std::vector<pmp::vec3> normal_array;
    normal_array.reserve(n_vertices_to_upload);
    for (SkinnedMesh& mesh : character_.meshes())
    {
        pmp::vertex_normals(mesh.mesh_);
        auto normal_prop = mesh.mesh_.get_vertex_property<pmp::vec3>("v:normal");

        for (pmp::Face f : mesh.mesh_.faces())
        {
            for (pmp::Vertex v : mesh.mesh_.vertices(f))
            {
                normal_array.push_back(normal_prop[v]);
            }
        }
    }

    // Upload normals
    glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
    glBufferData(GL_ARRAY_BUFFER, normal_array.size() * 3 * sizeof(float),
                 normal_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(1);

    // Skinning weights
    std::vector<pmp::vec4> weights1_array;
    std::vector<pmp::vec4> depends1_array;
    std::vector<pmp::vec4> weights2_array;
    std::vector<pmp::vec4> depends2_array;
    weights1_array.reserve(n_vertices_to_upload);
    depends1_array.reserve(n_vertices_to_upload);
    weights2_array.reserve(n_vertices_to_upload);
    depends2_array.reserve(n_vertices_to_upload);

    for (SkinnedMesh& mesh : character_.meshes())
    {
        auto weights1 = mesh.get_weights1();
        auto weights2 = mesh.get_weights2();
        auto depends1 = mesh.get_depends1();
        auto depends2 = mesh.get_depends2();

        for (pmp::Face f : mesh.mesh_.faces())
        {
            for (pmp::Vertex v : mesh.mesh_.vertices(f))
            {
                weights1_array.push_back(weights1[v]);
                weights2_array.push_back(weights2[v]);
                depends1_array.push_back(depends1[v]);
                depends2_array.push_back(depends2[v]);
            }
        }
    }

    // Upload skinning weights
    glBindBuffer(GL_ARRAY_BUFFER, weights1_buffer_);
    glBufferData(GL_ARRAY_BUFFER, weights1_array.size() * sizeof(pmp::vec4),
                 weights1_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, depends1_buffer_);
    glBufferData(GL_ARRAY_BUFFER, depends1_array.size() * sizeof(pmp::vec4),
                 depends1_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(3);

    glBindBuffer(GL_ARRAY_BUFFER, weights2_buffer_);
    glBufferData(GL_ARRAY_BUFFER, weights2_array.size() * sizeof(pmp::vec4),
                 weights2_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(4);

    glBindBuffer(GL_ARRAY_BUFFER, depends2_buffer_);
    glBufferData(GL_ARRAY_BUFFER, depends2_array.size() * sizeof(pmp::vec4),
                 depends2_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(5);

    // Upload Skinning matrices
    glBindBuffer(GL_UNIFORM_BUFFER, skinning_matrix_buffer_);

    glBufferData(GL_UNIFORM_BUFFER,
                 (unsigned int) character_.skeleton().skinning_matrices_.size()*sizeof(pmp::mat4),
                 character_.skeleton().skinning_matrices_.data(),
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    glBindVertexArray(0);

    update_skeleton_opengl_buffers();
}

void CharacterRenderer::create_opengl_buffers()
{
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vertex_buffer_);
    glGenBuffers(1, &normal_buffer_);
    glGenBuffers(1, &weights1_buffer_);
    glGenBuffers(1, &weights2_buffer_);
    glGenBuffers(1, &depends1_buffer_);
    glGenBuffers(1, &depends2_buffer_);
    glGenBuffers(1, &skinning_matrix_buffer_);

    glGenVertexArrays(1, &skeleton_vao_);
    glGenBuffers(1, &skeleton_cs_buffer_);
    glGenBuffers(1, &skeleton_cs_color_buffer_);
}

void CharacterRenderer::delete_opengl_buffers()
{
    glDeleteBuffers(1, &skeleton_cs_buffer_);
    glDeleteBuffers(1, &skeleton_cs_color_buffer_);

    glDeleteVertexArrays(1, &skeleton_vao_);

    glDeleteBuffers(1, &skinning_matrix_buffer_);
    glDeleteBuffers(1, &weights1_buffer_);
    glDeleteBuffers(1, &weights2_buffer_);
    glDeleteBuffers(1, &depends1_buffer_);
    glDeleteBuffers(1, &depends2_buffer_);
    glDeleteBuffers(1, &normal_buffer_);
    glDeleteBuffers(1, &vertex_buffer_);

    glDeleteVertexArrays(1, &vao_);
}

void CharacterRenderer::update_skeleton_opengl_buffers()
{
    std::vector<Joint*>& joints = character_.skeleton().joints_;
    size_t num_joints = joints.size();

    std::vector<pmp::vec3> position_array;
    std::vector<pmp::vec3> color_array;

    n_skeleton_vertices_ = 6 * num_joints;

    for (Joint* j : joints)
    {
        if (j->parent_)
        {
            n_skeleton_vertices_ += 2;
        }
    }

    position_array.reserve(n_skeleton_vertices_);
    color_array.reserve(n_skeleton_vertices_);

    pmp::BoundingBox bb;
    for (SkinnedMesh& mesh : character_.meshes())
    {
        bb += pmp::bounds(mesh.mesh_);
    }

    float cs_size = bb.size() * 0.01f;

    const static pmp::vec3 x_axis = pmp::vec3(1.0f, 0.0f, 0.0f);
    const static pmp::vec3 y_axis = pmp::vec3(0.0f, 1.0f, 0.0f);
    const static pmp::vec3 z_axis = pmp::vec3(0.0f, 0.0f, 1.0f);
    const static pmp::vec3 gray = pmp::vec3(0.3f, 0.3f, 0.3f);

    for (size_t joint_index = 0; joint_index < num_joints; ++joint_index)
    {
        Joint* j = joints[joint_index];
        pmp::vec3 jp = j->get_global_translation();

        if (j->parent_)
        {
            position_array.push_back(jp);
            position_array.push_back(j->parent_->get_global_translation());

            color_array.push_back(gray);
            color_array.push_back(gray);
        }

        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, x_axis));
        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, y_axis));
        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, z_axis));

        color_array.push_back(x_axis);
        color_array.push_back(x_axis);
        color_array.push_back(y_axis);
        color_array.push_back(y_axis);
        color_array.push_back(z_axis);
        color_array.push_back(z_axis);
    }

    glBindVertexArray(skeleton_vao_);

    glBindBuffer(GL_ARRAY_BUFFER, skeleton_cs_buffer_);
    glBufferData(GL_ARRAY_BUFFER, position_array.size() * sizeof(pmp::vec3),
                 position_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, skeleton_cs_color_buffer_);
    glBufferData(GL_ARRAY_BUFFER, color_array.size() * sizeof(pmp::vec3),
                 color_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}


void CharacterRenderer::draw(const pmp::mat4& projection_matrix, const pmp::mat4& modelview_matrix,
                             const std::string& draw_mode)
{
    // did we generate buffers already?
    bool any_dirty = false;
    for (SkinnedMesh& mesh : character_.meshes())
    {
        if (mesh.is_dirty())
        {
            any_dirty = true;
        }

        // Clear dirty flag, since we already stored, if we need to reupload opengl buffers
        mesh.set_dirty(false);
    }

    if (!vao_ || any_dirty)
    {
        update_opengl_buffers();
    }

    if (!character_shader_.is_valid())
    {
        //try
        {
            character_shader_.source(character_vshader, character_fshader);
        }
        //catch (const pmp::GLException& e)
        //{
        //    std::cerr << e.what() << std::endl;
        //    exit(1);
        //}
    }

    // activate VAO
    glBindVertexArray(vao_);


    // allow for transparent objects
    glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);

    // setup matrices
    pmp::mat4 mv_matrix = modelview_matrix;
    pmp::mat4 mvp_matrix = projection_matrix * modelview_matrix;
    pmp::mat3 n_matrix = inverse(transpose(linear_part(mv_matrix)));

    // setup shader
    character_shader_.use();
    character_shader_.set_uniform("modelview_projection_matrix", mvp_matrix);
    character_shader_.set_uniform("modelview_matrix", mv_matrix);
    character_shader_.set_uniform("normal_matrix", n_matrix);
    character_shader_.set_uniform("point_size", point_size_);
    character_shader_.set_uniform("light1", pmp::vec3(1.0, 1.0, 1.0));
    character_shader_.set_uniform("light2", pmp::vec3(-1.0, 1.0, 1.0));
    character_shader_.set_uniform("front_color", front_color_);
    character_shader_.set_uniform("back_color", back_color_);
    character_shader_.set_uniform("ambient", ambient_);
    character_shader_.set_uniform("diffuse", diffuse_);
    character_shader_.set_uniform("specular", specular_);
    character_shader_.set_uniform("shininess", shininess_);
    character_shader_.set_uniform("alpha", alpha_);
    character_shader_.set_uniform("use_lighting", true);
    //character_shader_.set_uniform("use_texture", false);
    character_shader_.set_uniform("use_srgb", false);
    character_shader_.bind_ubo("skinning_matrices", skinning_matrix_buffer_, 0);
    //character_shader_.set_uniform("show_texture_layout", false);
    //character_shader_.set_uniform("use_vertex_color",
    //                          has_vertex_colors_ && use_colors_);


    if (draw_mode == "Points")
    {
        glDrawArrays(GL_POINTS, 0, n_vertices_);
    }
    else
    {
        glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
    }

    character_shader_.disable();


    if (draw_skeleton_)
    {
        draw_skeleton(projection_matrix, modelview_matrix);
    }

    // disable transparency (doesn't work well with imgui)
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);

    glBindVertexArray(0);
}

void CharacterRenderer::draw_skeleton(const pmp::mat4& projection_matrix, const pmp::mat4& modelview_matrix)
{

    if (!skeleton_shader_.is_valid())
    {
        skeleton_shader_.source(skeleton_vshader, skeleton_fshader);
    }

    pmp::mat4 mvp = projection_matrix * modelview_matrix;

    glBindVertexArray(skeleton_vao_);

    skeleton_shader_.use();
    skeleton_shader_.set_uniform("modelview_projection_matrix", mvp);

    glDisable(GL_DEPTH_TEST);

    glDrawArrays(GL_LINES, 0, n_skeleton_vertices_);

    glEnable(GL_DEPTH_TEST);

    skeleton_shader_.disable();

    glBindVertexArray(0);
}

}
