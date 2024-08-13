// =====================================================================================================================

#include "measurement_utils.h"

#include <cmath>

// =====================================================================================================================

// epsilon for intersections (numerical instabilities)
#include "measurement_defs.h"

#ifdef MEASUREMENT_UTILS_WRITE_UNCLOSED_LOOPS
    #include <chrono>
    #include "io/io_toolbox.h"
#endif

// =====================================================================================================================

NAMESPACE_BEGIN(mes_utils)

// ---------------------------------------------------------------------------------------------------------------------

auto centroid(const std::vector<pmp::Point>& positions) -> pmp::Point
{
    pmp::Point centroid {};
    for (const auto& point : positions) {
        centroid += point;
    }
    if (!positions.empty()) {
        centroid /= static_cast<float>(positions.size());
    }
    return centroid;
}

// ---------------------------------------------------------------------------------------------------------------------

auto nearest_vertex(const SurfaceMesh& mesh, const pmp::vec3& point) -> pmp::Vertex
{
    if (mesh.n_vertices() == 0) {
        throw std::runtime_error("Mesh has 0 vertices.");
    }

    // assume first vertex as nearest
    pmp::IndexType vertex_id = PMP_MAX_INDEX;
    float min_distance = FLT_MAX;

    for (auto vertex : mesh.vertices()) {
        if (pmp::distance(mesh.position(vertex), point) < min_distance) {
            min_distance = pmp::distance(mesh.position(vertex), point);
            vertex_id = vertex.idx();
        }
    }
    return pmp::Vertex { vertex_id };
}

// ---------------------------------------------------------------------------------------------------------------------

auto axis_aligned_centroid_plane(const SurfaceMesh& mesh, const Selection& selection, const MeasurementOptions& options) -> Plane
{
    size_t axis = options.measurement_plane_normal_axis.axis_index();
    pmp::vec3 plane_n {};
    plane_n[axis] = 1.0;

    // centroid of points, minimizes distance of plane to all points of circumference selection
    float plane_distance = 0.0f;
    for (int id : selection)
    {
        pmp::Vertex v(id);
        plane_distance += mesh.position(v)[axis];
    }
    plane_distance /= static_cast<float> (selection.size());

    // our plane is parallel to the ground plane (y-up)
    //pmp::vec3 plane_n = pmp::vec3(0, 1, 0);

    pmp::vec3 plane_p = pmp::vec3(0, 0, 0);
    plane_p[axis] = plane_distance;

    return { /*point=*/ plane_p, /*normal=*/ plane_n };
}

// ---------------------------------------------------------------------------------------------------------------------

auto signed_distance(const Plane& plane, const pmp::Point& point) -> double
{
    // d -> the shortest distance to plane from origin
    // note: Normal must not be normalized
    auto d = pmp::dot(plane.n, plane.p);
    return (pmp::dot(point, plane.n) - d) / pmp::norm(plane.n);
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * Measure angle between plane normal and given vector
 * the angle is measured in degrees, 0 is marking a vector perpendicular to the normal
 * For vectors on the normal side of the plane, the angle is positive
 *
 * @param plane (normalized normal vector)
 * @param vector (any vector)
 * @return angle between plane and vector in degrees.
 */
auto angle_plane_vec(const Plane& plane, pmp::vec3 vector) -> double
{
    vector = pmp::normalize(vector);

    double cos_angle = pmp::dot(plane.n, vector);
    double theta = acos(cos_angle) - M_PI_2;
    return -theta * (180.0 / M_PI);

//    auto cross_norm = pmp::norm(pmp::cross(plane.n, vector));
//    auto dot = pmp::dot(plane.n, vector);
//    double theta = atan2(cross_norm, dot);
//    // convert to output range
//    return -(theta - M_PI_2) * (180.0 / M_PI) ;
}

// ---------------------------------------------------------------------------------------------------------------------

auto build_halfspaces(MeasurementContext& context, const Measurement& measurement) -> std::vector<Halfspace>
{
    std::vector<Halfspace> result {};
    result.reserve(measurement.options.halfspace_recipes.size());

    for (auto& recipe : measurement.options.halfspace_recipes) {
        result.emplace_back(recipe.to_halfspace(context));
    }
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * Intersects secments with half space. Create new segments for boundary elements
 * @param surface_segments = Edges
 * @param halfspace
 * @return surface segments in half space
 */
auto segments_in_half_space(const std::vector<FaceSegment>& surface_segments, const Halfspace& halfspace) -> std::vector<FaceSegment>
{
    std::vector<FaceSegment> result {};

    for (auto& segment : surface_segments) {
        // distance to plane
        auto d0 = signed_distance(halfspace, segment.p0);
        auto d1 = signed_distance(halfspace, segment.p1);

        if (d0 >= -INTERSECTION_EPS && d1 >= -INTERSECTION_EPS) {
            // both points on one side -> ok, add start
            result.emplace_back(segment.p0, segment.p1);
        } else if (d0 >= -INTERSECTION_EPS && d1 < -INTERSECTION_EPS) {
            // p1 is 'behind' plane
            // cast ray to plane, add point on plane
            auto ray_v = segment.p1 - segment.p0; // to - from
            auto plane_d = dot(halfspace.n, halfspace.p);
            auto denom = pmp::dot(ray_v, halfspace.n);

            if (fabs(denom) > FLT_MIN) {
                auto t = -(pmp::dot(halfspace.n, segment.p0) - plane_d) / denom;
                auto intersection_point = segment.p0 + t * ray_v;

                result.emplace_back(segment.p0, intersection_point);
//            std::cout << "[DEBUG] p0(" << p0 << ") before and p1(" << p1 << ") behind plane, added " << intersection_point << std::endl;
//            std::cout << "d0=" << d0 << " d1=" << d1 << std::endl;
//            std::cout << "t=" << t << " dn=" << denom << std::endl;
            }
        } else if (d0 < -INTERSECTION_EPS && d1 >= -INTERSECTION_EPS) {
            // p0 is 'behind' plane
            auto ray_v = segment.p0 - segment.p1;
            auto plane_d = dot(halfspace.n, halfspace.p);
            auto denom = pmp::dot(ray_v, halfspace.n);

            if (fabs(denom) > FLT_MIN) {
                auto t = -(pmp::dot(halfspace.n, segment.p1) - plane_d) / denom;
                auto intersection_point = segment.p1 + t * ray_v;

                result.emplace_back(intersection_point, segment.p1);
//            std::cout << "[DEBUG] p0(" << p0 << ") behind and p1(" << p1 << ") before plane, added " << intersection_point << std::endl;
//            std::cout << "d0=" << d0 << " d1=" << d1 << std::endl;
//            std::cout << "t=" << t << " dn=" << denom << std::endl;
            }
        }
    }

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * Extend selection with N-face rings around vertices
 * @param mesh
 * @param selection
 * @param num_rings
 * @return
 */
auto face_ring_around_selection(const SurfaceMesh& mesh, const Selection& selection, const size_t num_rings) -> std::set<pmp::Face>
{
    // use face candidate list around selection vertices
    std::set<pmp::Face> face_candidates {};

    // build list of initial faces around selection vertices
    for (auto vertex_id : selection) {
        auto v = pmp::Vertex(vertex_id);
        // 1-ring neighbourhood
        for (auto v_face: mesh.faces(v)) {
            face_candidates.insert(v_face);
        }
    }

    // extend neighbourhood by one ring
    for (size_t ring = 0; ring < num_rings - 1; ++ring) {
        // include next-ring neighbourhood of faces
        std::vector<pmp::Face> new_candidates {};
        for (auto face : face_candidates) {
            // next-ring vertices (and previous ring)
            for (auto f_vertex : mesh.vertices(face)) {
                for (auto face_2_ring : mesh.faces(f_vertex)) {
                    new_candidates.emplace_back(face_2_ring);
                }
            }
        }

        // new candidates in final set
        face_candidates.insert(new_candidates.begin(), new_candidates.end());
    }

    return face_candidates;
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 *
 * @param p0 first point
 * @param p1 second point
 * @param plane intersection plane
 * @return
 */
auto ray_plane_intersection(const pmp::Point& p0, const pmp::Point& p1, const Plane& plane) -> bool
{
    auto d0 = signed_distance(plane, p0);
    auto d1 = signed_distance(plane, p1);

    if ((d0 >= -INTERSECTION_EPS && d1 < -INTERSECTION_EPS) || (d0 < -INTERSECTION_EPS && d1 >= -INTERSECTION_EPS)) {
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

auto ray_plane_intersection_t(const pmp::Point& p0, const pmp::Point& p1, const Plane& plane, pmp::Point& intersection_point) -> float
{
    auto ray_v = p1 - p0; // to - from
    auto plane_d = dot(plane.n, plane.p);
    auto denom = pmp::dot(ray_v, plane.n);

    if (fabs(denom) > FLT_MIN) {
        auto t = -(pmp::dot(plane.n, p0) - plane_d) / denom;
        intersection_point = p0 + t * ray_v;
        return t;
    } else {
        return std::numeric_limits<float>::quiet_NaN();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * helper for mesh_plane_intersection_loops_ordered
 */
auto mesh_plane_intersection_ordered_ring(
    SurfaceMesh& mesh,
    const Plane& plane, // intersection plane
    const std::set<pmp::Face>& face_candidates,
    const pmp::Face initial_face,
    pmp::FaceProperty<bool>& visited,
    bool enforce_candidate_list,
    bool raise_warning,
    int& processed_faces_num,
    bool& error
) -> std::list<FaceSegment>
{
    bool forward_direction = false;
    pmp::Halfedge current_he;
    pmp::Face current_face;

    std::list<FaceSegment> result;

    // extend first intersection found in both directions
    for (auto initial_he : mesh.halfedges(initial_face)) {
        // triangle has exactly two intersections with plane
        if (ray_plane_intersection(mesh.position(mesh.from_vertex(initial_he)),
                                   mesh.position(mesh.to_vertex(initial_he)), plane)) {
            // first outgoing edge found
            current_he = initial_he;
            current_face = initial_face;

            // reset visited property on first face (search loop cancels otherwise)
            visited[current_face] = false;

            forward_direction = not forward_direction;

            // find all triangles along opposite intersecting halfedge
            // end, if face is already visited OR not in candidate list
            // make sure, that the selection region is not left!
            while (not visited[current_face]) {
                if (face_candidates.find(current_face) == face_candidates.end()) {
                    visited[current_face] = true;
                    // do not add face, when not in candidate list
                    if (enforce_candidate_list) {
                        error = true;
                        if (raise_warning) {
                            std::cout << "[WARN] mesh_plane_intersection_loops_ordered: Found intersection with Face "
                                      << current_face
                                      << " which was not in candidate list. Check if you can increase num_face_rings"
                                         " for this measurement. \n";
                        }

                        // do not add this loop to intersection list
                        continue;
                    }
                } // check if face in candidate list

                // two of three edges intersect the plane (in regular case)
                // - edge is in plane (not handled or tested)
                // - irregular case (both points in plane)?
                for (auto he: mesh.halfedges(current_face)) {
                    if (ray_plane_intersection(mesh.position(mesh.from_vertex(he)), mesh.position(mesh.to_vertex(he)),
                                               plane)) {
                        // check, if the halfedge found is not the opposite of the last triangle
                        if (he != current_he) {
                            // next halfedge (in face) found
                            pmp::Point p0, p1;
                            ray_plane_intersection_t(
                                mesh.position(mesh.from_vertex(current_he)),
                                mesh.position(mesh.to_vertex(current_he)),
                                plane, p0);
                            ray_plane_intersection_t(
                                mesh.position(mesh.from_vertex(he)),
                                mesh.position(mesh.to_vertex(he)),
                                plane, p1);


                            if (forward_direction) {
                                result.emplace_back(p0, p1);
                                processed_faces_num++;
                            } else if (current_face != initial_face) {
                                result.emplace_front(p1, p0);
                                processed_faces_num++;
                            }

                            // set intersection test to next face
                            visited[current_face] = true;
                            current_he = mesh.opposite_halfedge(he);
                            current_face = mesh.face(current_he);
                            break;
                        }
                    } // if intersecting plane
                } // for face halfedges
            } // while not visited[current_face]
        }
    }

    return result;
}


// ---------------------------------------------------------------------------------------------------------------------

/**
 * Note: We use the property that our surface mesh is a non-manifold object
 * We can not process eye and mouth region!
 * @param mesh
 * @param face_candidates
 * @param plane
 * @return
 */
auto mesh_plane_intersection_loops_ordered(
    SurfaceMesh& mesh,
    const std::set<pmp::Face>& face_candidates,
    const Plane& plane,
    bool& error,
    bool enforce_candidate_list,
    bool complete_loop,
    bool raise_warning
) -> std::vector<std::vector<FaceSegment>>
{
    // mark faces as visited
    if (mesh.has_face_property("f:intersection_visited")) {
        auto fprop = mesh.get_face_property<bool>("f:intersection_visited");
        mesh.remove_face_property(fprop);
    }
    auto visited = mesh.add_face_property<bool>("f:intersection_visited", false);

    // find ANY intersection in set of face and plane
    // from there, find intersecting halfedge, and search for intersection in face on opposite halfedge

    // no error yet
    error = false;
    int candidate_face_intersection_num = 0;
    std::vector<pmp::Face> intersecting_faces;

    // find first triangle inztersection with plane
    for (auto& face : face_candidates) {
        bool intersection_found = false;
        for (auto he : mesh.halfedges(face)) {
            if (ray_plane_intersection(mesh.position(mesh.from_vertex(he)), mesh.position(mesh.to_vertex(he)), plane)) {
                intersection_found = true;
                continue; // skip second halfedge
            }
        }
        if (intersection_found) {
            intersecting_faces.emplace_back(face);
            candidate_face_intersection_num++;
        }
    }

    // no intersection with candidates found
    if (intersecting_faces.empty()) {
        std::cerr << "[WARN] No intersecting faces found in given candidate list. \n";
        mesh.remove_face_property(visited);
        error = true;
        return {};
    }

    int processed_faces_num = 0;

    // a list containing all found loops
    std::vector<std::vector<FaceSegment>> result {};

    // process all intersecting faces
    for (auto current_face : intersecting_faces) {
        if (not visited[current_face]) {
            std::list<FaceSegment> current_loop = mesh_plane_intersection_ordered_ring(
                mesh, plane, face_candidates, current_face, visited,
                enforce_candidate_list, raise_warning, processed_faces_num, error
            );

            // Check segment looping property (should be continuous)
            if (complete_loop && current_loop.size() > 1 && (pmp::distance(current_loop.front().p0, current_loop.back().p1) >= POINT_EQUAL_EPS)) {
                error = true;
                std::cout << "[DEBUG] Did not find a complete intersection loop.\n";
                std::cout << "front.p0 = " << current_loop.front().p0 << "\n"
                          << "front.p1 = " << current_loop.front().p1 << "\n"
                          << "back.p0  = " << current_loop.back().p0  << "\n"
                          << "back.p1  = " << current_loop.back().p1  << "\n";

#ifdef MEASUREMENT_UTILS_WRITE_UNCLOSED_LOOPS
                // temporary dir
                std::filesystem::path debug_dir (MEASUREMENTS_UTILS_WRITE_UNCLOSED_LOOPS_DIR);
                std::chrono::microseconds now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                debug_dir = debug_dir / ("DBG_trace_" + std::to_string(now_us.count()) + ".txt");

                std::vector<FaceSegment> loop_vector { std::begin(current_loop), std::end(current_loop) };
                write_face_segments_trace(debug_dir.string(), loop_vector);
#endif
            }

            // insert resulting list into final intersection segment list (which may contain multiple loops at this point)
            std::vector<FaceSegment> result_vector {
                std::make_move_iterator(std::begin(current_loop)),
                std::make_move_iterator(std::end(current_loop))
            };
            // place loop into result set
            result.emplace_back(result_vector);
        }
    }

    // not all loops processed => this is an error!
    if (processed_faces_num < candidate_face_intersection_num) {
        error = true;
        // this may happen, if the person has sagging boobs, resulting in more than one intersection for the plane
        std::cerr << "[WARN] mesh_plane_intersection_loops_ordered: There are multiple face loops intersecting the plane. "
                  << "CANDIDATE_INTERSECTING_FACES=" << candidate_face_intersection_num << " LOOP_FACES=" << processed_faces_num
                  << "\n";
    } else if (processed_faces_num > candidate_face_intersection_num) {
        // this is only an error, if candidate list is enforced
        if (enforce_candidate_list) {
            error = true;
            std::cerr << "[INFO] mesh_plane_intersection_loops_ordered: Intersection contains faces not in candidate list. "
                      << "CANDIDATE_INTERSECTING_FACES=" << candidate_face_intersection_num << " LOOP_FACES=" << processed_faces_num
                      << "\n";
        }
    }

    // cleanup & converting to vector
    mesh.remove_face_property(visited);
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto select_intersection_loop(
    // a list containing all intersection loops
    const std::vector<std::vector<FaceSegment>> &loops,
    bool merge_loops
) -> std::vector<FaceSegment>
{
    std::vector<FaceSegment> result;
    if (merge_loops) {
        for (const auto& loop : loops) {
            result.insert(std::end(result), std::begin(loop), std::end(loop));
        }
    } else {
        auto current_max_length = 0.0;
        for (const auto& loop : loops) {
            auto loop_length = measure_surface_segments(loop);
            if (loop_length > current_max_length) {
                result = loop;
                current_max_length = loop_length;
            }
        }
    }

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto make_convex_ring(
    const std::vector<FaceSegment>& surface_segments,
    const Plane& intersection_plane,
    const Axis& projection_plane_normal
) -> std::vector<FaceSegment>
{
    // 3D to 2D
    std::vector<Point_2> points_2d;
    if (projection_plane_normal == Axis::X)
    {
        for (const auto& segment : surface_segments) {
            points_2d.emplace_back(segment.p0[1], segment.p0[2]);
            points_2d.emplace_back(segment.p1[1], segment.p1[2]);
        }
    }
    else if (projection_plane_normal == Axis::Y)
    {
        for (const auto& segment : surface_segments) {
            points_2d.emplace_back(segment.p0[0], segment.p0[2]);
            points_2d.emplace_back(segment.p1[0], segment.p1[2]);
        }
    }
    else
    {
        for (const auto& segment : surface_segments) {
            points_2d.emplace_back(segment.p0[0], segment.p0[1]);
            points_2d.emplace_back(segment.p1[0], segment.p1[1]);
        }
    }

    // Compute convex hull
    std::vector<Point_2> convex_hull {};
    std::vector<pmp::Point> convex_hull_3d;
    convex_hull_3d.reserve(points_2d.size()); // small speedup (no re-allocation)
    CGAL::ch_graham_andrew(points_2d.begin(), points_2d.end(), std::back_inserter(convex_hull));

    // 2D to 3D
    if (projection_plane_normal == Axis::X)
    {
        for (auto& p : convex_hull) { convex_hull_3d.emplace_back(intersection_plane.p[0], p[0], p[1]); }
    }
    else if (projection_plane_normal == Axis::Y)
    {
        for (auto& p : convex_hull) { convex_hull_3d.emplace_back(p[0], intersection_plane.p[1], p[1]); }
    }
    else
    {
        for (auto& p : convex_hull) { convex_hull_3d.emplace_back(p[0], p[1], intersection_plane.p[2]); }
    }

    // convert back into line segments
    std::vector<FaceSegment> result;
    result.reserve(convex_hull_3d.size());

    for (long index = 0; index < static_cast<long> (convex_hull_3d.size()) - 1; ++index) {
        result.emplace_back(convex_hull_3d[index], convex_hull_3d[index+1]);
    }

    // last segment (close loop)
    if (result.size() > 2) {
        result.emplace_back(convex_hull_3d.back(), convex_hull_3d.front());
    }

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto make_convex_curve(
    const std::vector<FaceSegment>& surface_segments,
    const Plane& intersection_plane,
    const Axis& projection_plane_normal_axis,
    const pmp::vec3& dominant_normal,
    const float deviance_min_cos
) -> std::vector<FaceSegment>
{
    std::vector<FaceSegment> convex_segments = make_convex_ring(surface_segments, intersection_plane,
                                                                projection_plane_normal_axis);

    // the convex segments are ordered ccw, compare normals of two neighboring segments
    // if found < 90 degree, we found a terminating point of the curve (start or end)
    // choose the path in direction dominant segment direction to the other terminating point
    // reject opposite path
    auto projection_plane_normal = projection_plane_normal_axis.as_normal();

    std::vector<pmp::vec3> seg_normals {};
    seg_normals.reserve(convex_segments.size());

    // normals for segments
    for (auto& segment : convex_segments) {
        auto seg_normal = pmp::cross(pmp::normalize(segment.p1 - segment.p0), projection_plane_normal);
        seg_normals.emplace_back(seg_normal);
    }

    // add segments which do not deviate too much from mean halfspace normal
    std::vector<FaceSegment> result;

    for (long index = 0; index < static_cast<long>(seg_normals.size()); ++index) {
        auto normal_cos = pmp::dot(dominant_normal, seg_normals[index]);
        if (normal_cos > deviance_min_cos) {
            result.emplace_back(convex_segments[index]);
        }
    }

//    std::cout << "INPUT size=" << convex_segments.size() << " OUTPUT size=" << result.size() << '\n';

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_surface_segments(const std::vector<FaceSegment>& surface_segments) -> double
{
    double result = 0.0;

    for (const auto& segment : surface_segments) {
        result += pmp::distance(segment.p0, segment.p1);
    }

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

/**
* Precondition: ordered list of selection vertices
*
* @param mesh
* @param selection
* @param options
* @return
*/
auto measure_vertex_trace(const SurfaceMesh& mesh, const Selection& selection, const MeasurementOptions& options) -> double
{
    double distance = 0.0;

    for (long i = 0; i < static_cast<long>(selection.size()) - 1; ++i)
    {
        pmp::Vertex v1(selection[i + 0]);
        pmp::Vertex v2(selection[i + 1]);

        distance += pmp::distance(mesh.position(v1), mesh.position(v2));
    }

    if (options.is_ring && !selection.empty())
    {
        // first to last

        pmp::Vertex v1(selection[0]);
        pmp::Vertex v2(selection[selection.size() - 1]);

        distance += pmp::distance(mesh.position(v1), mesh.position(v2));
    }

    return distance;
}

// ---------------------------------------------------------------------------------------------------------------------

NAMESPACE_END(mes_utils)

// =====================================================================================================================
