#include "Mean_value_coordinates.h"

#include "SkinnedMesh.h"

namespace template_fitting {
namespace mvc {

void compute_weights(SkinnedMesh* mesh,
                     const std::vector<pmp::vec3>& points)
{
    auto mvc_prop = mesh->mesh_.vertex_property<std::vector<float>>(properties::mvc_weights);

    auto vps = mesh->mesh_.get_vertex_property<pmp::vec3>("v:point");

    for (pmp::Vertex v : mesh->mesh_.vertices())
    {
        mvc_prop[v].resize(points.size());
    }

    for (size_t i = 0; i < points.size(); ++i)
    {
        double w_sum = 0.0;

        for (pmp::Vertex v : mesh->mesh_.vertices())
        {
            pmp::dvec3 v_i = (pmp::dvec3)(vps[v] - points[i]);

            double w_i = 0.0;
            double r   = 1.0 / norm(v_i);

            pmp::dvec3 e_i = normalize(v_i);

            //loop local neighborhood
            auto vj_iter = mesh->mesh_.vertices(v);
            if (mesh->mesh_.is_boundary(*vj_iter))
                continue;
            auto vj_iter_end = vj_iter;

            // Neighborhood data
            pmp::dvec3 v_j, e_j, v_k, e_k, n_jk, n_ij, n_ki;

            do
            {
               v_j = (pmp::dvec3)(vps[*vj_iter] - points[i]);
               e_j = normalize(v_j);
               if (mesh->mesh_.is_boundary(*vj_iter))
               {
                   w_i = 0.0;
                   break;
               }

               ++vj_iter; // !! next vertex from here on !!

               if (mesh->mesh_.is_boundary(*vj_iter))
               {
                   w_i = 0.0;
                   break;
               }

               v_k = (pmp::dvec3)(vps[*vj_iter] - points[i]);
               e_k = normalize(v_k);

               double beta_jk = std::abs(acos(dot(e_j, e_k)));
               double beta_ij = std::abs(acos(dot(e_i, e_j)));
               double beta_ki = std::abs(acos(dot(e_k, e_i)));

               n_jk = normalize(cross(e_j, e_k));
               n_ij = normalize(cross(e_i, e_j));
               n_ki = normalize(cross(e_k, e_i));

               double w = beta_jk + beta_ij*dot(n_ij, n_jk) + beta_ki*dot(n_ki, n_jk);

               if (!(w == w))
               {
                   std::cerr << "[ERROR] compute_mean_value_coordinates: Weight is nan!" << std::endl;
               }

               w *= 1.0 / (2.0*dot(e_i, n_jk));
               w_i += w;
            } while (vj_iter != vj_iter_end);

            w_i *= r;
            //if (w_i < 0){
            //   std::cerr << "w_i is smaller 0:" << w_i << std::endl;
            //}
            w_sum += w_i;
            mvc_prop[v][i] = w_i;
        }

        // normalization of all weights
        for (pmp::Vertex v : mesh->mesh_.vertices())
        {
           mvc_prop[v][i] /= w_sum;
        }

        pmp::vec3 p_comp(0.0);
        for (pmp::Vertex v : mesh->mesh_.vertices())
        {
            p_comp += mvc_prop[v][i] * vps[v];
        }
        float d = distance(p_comp, points[i]);
        if (d > 0.001)
        {
           std::cerr << "MVC points difference: Distance = " << d << " | real position "<<p_comp<<" -> calc. position "<<points[i]<< std::endl;
        }
    }
}

bool compute_weights_from_current_joint_positions(Character* character)
{
    if (character == NULL)
    {
        std::cerr << "compute_mean_value_coords_from_current_joint_positions: [ERROR] No character available!" << std::endl;
        return false;
    }

    SkinnedMesh& mesh = character->get_selected_skin();

    auto mvc_prop = mesh.get_mvc_weights();
    if (mvc_prop)
    {
        std::cerr <<"compute_mean_value_coords_from_current_joint_positions: [INFO] Can't compute MVC since you already have them!" << std::endl;
        return false;
    }

    const Skeleton& skeleton = character->skeleton();

    std::vector<pmp::vec3> joint_positions(skeleton.joints_.size());
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        joint_positions[i] = skeleton.joints_[i]->get_global_translation();
    }

    compute_weights(&mesh, joint_positions);

    return true;
}

bool compute_new_joint_positions(SkinnedMesh* mesh,
                                 std::vector<pmp::vec3>& points)
{
    auto mvc_prop = mesh->get_mvc_weights();

    auto vpoint = mesh->mesh_.get_vertex_property<pmp::vec3>("v:point");

    if (!mvc_prop || !vpoint)
    {
        return false;
    }

    size_t i;
    pmp::vec3 p;

    points.clear();
    points.resize(mvc_prop[pmp::Vertex(0)].size());

    for (i=0; i < points.size(); ++i)
    {
        p = pmp::vec3(0.0f);
        for (pmp::Vertex v : mesh->mesh_.vertices())
        {
            p += mvc_prop[v][i] * vpoint[v];
        }

        points[i] = p;
    }

    return true;
}

}
}
