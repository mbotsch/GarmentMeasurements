//
// Created by fabian on 11/29/23.
//

#include "measure_body.h"

#include <fstream>

#include <nlohmann/json.hpp>
#include <pmp/algorithms/geodesics.h>
#include <pmp/algorithms/utilities.h>

#include <io/io_toolbox.h>

#include "measurement_defs.h"
#include "measurement_utils.h"

// =====================================================================================================================

using json = nlohmann::json;
using namespace mes_utils;

// =====================================================================================================================
// definitions for measurements

// up or down
const float MEASUREMENT_OPTIMIZATION_UNIDIRECTIONAL_MAX = 0.100F; // meters = 10 cm
// maximum distance above and below initial plane
const float MEASUREMENT_OPTIMIZATION_BIDIRECTIONAL_MAX = 0.020F; // meters = 2 cm
const float MEASUREMENT_OPTIMIZATION_STEP = 0.005F; // meters = 5 mm

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto load_landmarks(const std::filesystem::path& path, const pmp::SurfaceMesh& mesh, std::map<std::string, Landmark>& landmarks) -> bool
{
    // Load JSON file
    std::ifstream ifs(path);
    if (!ifs)
    {
        fprintf(stderr, "[ERROR] Cannot read landmarks from %s\n", path.c_str());
        return false;
    }

    // Parse JSON file
    json loaded_json;
    ifs >> loaded_json;

    // Extract landmarks
    for (auto it = loaded_json["landmarks"].begin(); it != loaded_json["landmarks"].end(); ++it)
    {
        landmarks[it.key()] = {it.value(), pmp::vec3(0,0,0)};
    }

    for (auto& it : landmarks)
    {
        it.second.position = mesh.position(pmp::Vertex(it.second.vertex_id));
    }

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

auto construct_measurement_plane(MeasurementContext& context, Measurement& measurement) -> Plane
{
    Plane result;
    if (measurement.options.measurement_plane_type == MeasurementPlaneType::CENTROID)
    {
        if (measurement.selection.empty()) {
            std::cerr << "[ERROR] Centroid: Empty selection for " << measurement.name << '\n';
        }
        // calculate centroid for vertex selection, use as plane
        result = axis_aligned_centroid_plane(context.mesh, measurement.selection, measurement.options);
        // make normal axis aligned
        result.n = measurement.options.measurement_plane_normal_axis.as_normal();
    }
    else if (measurement.options.measurement_plane_type == MeasurementPlaneType::LANDMARKS)
    {
        if (measurement.options.measurement_plane_landmarks.size() == 1)
        {
            // normal axis + anker
            auto& l0 = context.get_landmark(measurement.options.measurement_plane_landmarks[0]);
            result.p = l0.position;
            result.n = measurement.options.measurement_plane_normal_axis.as_normal();
        }
        else if (measurement.options.measurement_plane_landmarks.size() == 2)
        {
            // normal axis + anker
            // create plane from l0 to l1 as first plane vector
            // normal is second plane vector
            auto& l0 = context.get_landmark(measurement.options.measurement_plane_landmarks[0]);
            auto& l1 = context.get_landmark(measurement.options.measurement_plane_landmarks[1]);
            result.p = l0.position;
            result.n = pmp::cross(
                pmp::normalize(l1.position - l0.position),
                measurement.options.measurement_plane_landmarks_second_axis.as_normal()
            );
        }
        else {
            std::cerr << "[ERROR] Invalid landmark count for " << measurement.name << '\n';
        }
    }
    else
    {
        std::cerr << "[ERROR] Could not construct measurement plane for " << measurement.name
            << ". No measurement plane type given.\n";
    }

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_axis_aligned_circumference(MeasurementContext& context, const std::string& measurement_name) -> void
{
    auto& measurement = context.get_measurement(measurement_name);
    auto& mesh = context.mesh;
    auto halfspaces = build_halfspaces(context, measurement);
    bool raise_intersection_warning = measurement.options.is_ring;
    pmp::vec3 convex_hull_normal = measurement.options.convex_hull_normal;

    // do not raise intersection warning for legs
    if (measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::UP || measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::DOWN) {
        raise_intersection_warning = false;
    }

    if (not halfspaces.empty() and (convex_hull_normal == pmp::vec3 {0.0, 0.0, 0.0})) {
        std::cout << "[WARN] measure_axis_aligned_circumference: Dominant normal is undefined.\n";
        std::cout << convex_hull_normal << '\n';
    }

    // construct measurement plane
    auto measurement_plane = construct_measurement_plane(context, measurement);
    measurement.measurement_plane = measurement_plane;

    // search area for initial face intersection
    auto intersection_face_candidates = face_ring_around_selection(mesh, measurement.selection, measurement.options.num_face_rings);
    auto& plane_normal_axis = measurement.options.measurement_plane_normal_axis;

    // create intersection for mesh with plane
    bool intersection_error = false;
    auto intersection_segments = select_intersection_loop(
        mesh_plane_intersection_loops_ordered(
            /*mesh=*/mesh,
            /*face_candidates=*/intersection_face_candidates,
            /*plane=*/measurement_plane,
            /*error=*/intersection_error,
            /*add_non_candidates=*/measurement.options.enforce_candidate_list,
            /*complete_loop=*/measurement.options.is_ring,
            /*raise_warning=*/raise_intersection_warning
        )
    );
    // store initial intersection
    measurement.intersection_segments = intersection_segments;

    // Concept:
    // - create intersection segments
    // - cut half-spaces
    // - create convex hull (optionally)
    // - keep only "up-pointing" part of convex hull

    // filter intersection by half_spaces
    for (auto& halfspace : halfspaces) {
        intersection_segments = segments_in_half_space(intersection_segments, halfspace);
    }

    // convex hull projection?
    if (measurement.options.convex_hull) {
        if (halfspaces.empty()) {
            intersection_segments = make_convex_ring(intersection_segments, measurement_plane, plane_normal_axis);
        } else {
            intersection_segments = make_convex_curve(
                intersection_segments, measurement_plane, plane_normal_axis, convex_hull_normal,
                measurement.options.convex_hull_normal_deviance_min_cos);
        }
    }

    // initial measurement
    measurement.measured_value = measure_surface_segments(intersection_segments);

    // write selections
    if (context.options.write_debug_files) {
        write_face_vertex_selection(context.options.debug_base_dir / (measurement_name + "_face_candidates.txt"), mesh, intersection_face_candidates);
        write_face_segments_trace(context.options.debug_base_dir / (measurement_name + "_trace.txt"), intersection_segments);
    }

    // optimize intersection plane (MIN or MAX)
    if (measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::MAX
        || measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::MIN
    ) {
        auto initial_plane = measurement_plane;

        // start below intersection plane at maximum distance
        float offset = -MEASUREMENT_OPTIMIZATION_BIDIRECTIONAL_MAX;

        while (offset <= MEASUREMENT_OPTIMIZATION_BIDIRECTIONAL_MAX) {
            // calculate offset to initial intersection plane
            measurement_plane.p = initial_plane.p + offset * initial_plane.n;
            // intersect with mesh
            //auto initial_intersection = mesh_aa_plane_intersection(mesh, intersection_faces, intersection_plane.p, measurement.options.measurement_axis);
            bool intersection_error_placeholder = false;
            auto initial_intersection = select_intersection_loop(
                mesh_plane_intersection_loops_ordered(
                    /*mesh=*/mesh,
                    /*face_candidates=*/intersection_face_candidates,
                    /*plane=*/measurement_plane,
                    /*error=*/intersection_error_placeholder,
                    /*add_non_candidates=*/measurement.options.enforce_candidate_list,
                    /*complete_loop=*/measurement.options.is_ring,
                    /*raise_warning=*/raise_intersection_warning
                )
            );
            // store for later use
            intersection_segments = initial_intersection;

            // filter intersection by half-spaces
            for (auto& half_space : halfspaces) {
                intersection_segments = segments_in_half_space(intersection_segments, half_space);
            }

            // convex hull projection?
            if (measurement.options.convex_hull) {
                if (halfspaces.empty()) {
                    intersection_segments = make_convex_ring(intersection_segments, measurement_plane,
                                                             plane_normal_axis);
                } else {
                    intersection_segments = make_convex_curve(
                        intersection_segments, measurement_plane, plane_normal_axis, convex_hull_normal,
                        measurement.options.convex_hull_normal_deviance_min_cos);
                }
            }

            double optimized_value = measure_surface_segments(intersection_segments);

            // update measurement
            if ((optimized_value < measurement.measured_value && measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::MIN)
                || (optimized_value > measurement.measured_value && measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::MAX))
            {
//                // DEBUGGING IMPROVEMENTS MODE
//                if (measurement_name == "bust") {
//                    std::cout << "Found better measurement: " << optimized_value <<"\n";
//                }

                // store optimized results
                measurement.measured_value = optimized_value;
                measurement.measurement_plane = measurement_plane;
                measurement.intersection_segments = initial_intersection;
                measurement.intersection_segments_convex = intersection_segments;
            }
            // shift plane along plane normal
            offset += MEASUREMENT_OPTIMIZATION_STEP;
        }

        if (context.options.write_debug_files) {
            // write convex hull (for debugging)
            auto segments = measurement.intersection_segments;
            if (measurement.options.convex_hull) {
                segments = measurement.intersection_segments_convex;
            }
            write_face_segments_trace(context.options.debug_base_dir / (measurement_name + "_trace_opt.txt"), segments);
            write_face_segments(context.options.debug_base_dir / (measurement_name + "_intersection_opt.txt"), segments);
        }
    }


    // LEG CIRCUMFERENCE (up or down optimization until no error is left)
    if (intersection_error
        && (measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::DOWN
            || measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::UP
        )
    ) {
        auto initial_plane = measurement_plane;

        if (context.options.print_stdout) {
            std::cout << "[INFO] Intersecting faces ware not in candidate list. Optimizing measurement.\n";
        }

        // start below intersection plane at maximum distance
        float offset = MEASUREMENT_OPTIMIZATION_STEP;

        while (offset <= MEASUREMENT_OPTIMIZATION_UNIDIRECTIONAL_MAX) {
            // calculate offset to initial intersection plane (up or down shifted)
            if (measurement.options.measurement_plane_optimization == MeasurementPlaneOptimization::UP) {
                measurement_plane.p = initial_plane.p + offset * initial_plane.n;
            } else {
                measurement_plane.p = initial_plane.p - offset * initial_plane.n;
            }

            // intersect with mesh
            auto initial_intersection = select_intersection_loop(
                mesh_plane_intersection_loops_ordered(
                        /*mesh=*/mesh,
                        /*face_candidates=*/intersection_face_candidates,
                        /*plane=*/measurement_plane,
                        /*error=*/intersection_error,
                        /*add_non_candidates=*/measurement.options.enforce_candidate_list,
                        /*complete_loop=*/measurement.options.is_ring,
                        /*raise_warning=*/raise_intersection_warning
                )
            );
            // store for later use
            intersection_segments = initial_intersection;

            // filter intersection by half-spaces
            for (auto& half_space : halfspaces) {
                intersection_segments = segments_in_half_space(intersection_segments, half_space);
            }

            // convex hull projection?
            if (measurement.options.convex_hull) {
                if (halfspaces.empty()) {
                    intersection_segments = make_convex_ring(intersection_segments, measurement_plane,
                                                             plane_normal_axis);
                } else {
                    intersection_segments = make_convex_curve(
                        intersection_segments, measurement_plane, plane_normal_axis, convex_hull_normal,
                        measurement.options.convex_hull_normal_deviance_min_cos);
                }
            }

            double optimized_value = measure_surface_segments(intersection_segments);

            // update measurement
            if (not intersection_error) {
                // store result without intersection error
                measurement.measured_value = optimized_value;
                measurement.measurement_plane = measurement_plane;
                measurement.intersection_segments = initial_intersection;
                break;
            }
            // shift plane along plane normal
            offset += MEASUREMENT_OPTIMIZATION_STEP;
        }

        if (offset >= MEASUREMENT_OPTIMIZATION_UNIDIRECTIONAL_MAX) {
            std::cerr << "[WARN] Optimization went to far without finding good circumference.\n";
        }

        // raise error (invalid measurement)
        if (intersection_error) {
            measurement.measured_value = -1;
            std::cerr << "[ERROR] Could not take measurement for " << measurement_name << ".\n";
            return;
        }

        if (context.options.write_debug_files) {
            write_face_segments_trace(context.options.debug_base_dir / (measurement_name + "_trace_opt.txt"), measurement.intersection_segments);
            write_face_segments(context.options.debug_base_dir / (measurement_name + "_intersection_opt.txt"), measurement.intersection_segments);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_geodesic_distance(MeasurementContext& context, const std::string& measurement_name) -> void
/**
 * Calculate geodesic distance between two landmarks on surface
 * Expect two landmarks in measurement.landmarks
 * @param context Measurement context
 * @param measurement_name name for this measurement
 */
{
    // measurement type check
    auto& measurement = context.get_measurement(measurement_name);

    // landmark count
    if (measurement.landmarks.size() != 2) {
        throw std::runtime_error("Need exactly two landmarks.");
    }

    // first landmark is start
    Landmark l0 = context.get_landmark(measurement.landmarks[0]);
    Landmark l1 = context.get_landmark(measurement.landmarks[1]);

    pmp::Vertex start_vertex = nearest_vertex(context.mesh, l0.position);
    pmp::Vertex end_vertex = nearest_vertex(context.mesh, l1.position);

    // geodedics_heat(context.mesh, { start_vertex }); // for arbitrary meshes
    pmp::geodesics(context.mesh, {start_vertex}); // for triangular meshes

    auto geodesic_distance = context.mesh.vertex_property<pmp::Scalar>("geodesic:distance");
    measurement.measured_value = geodesic_distance[end_vertex];
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_vertex_trace(MeasurementContext& context, const std::string& measurement_name) -> void
/**
 * Use a given vertex trace by the measurement.selection indices. Measure the euclidean distance between
 * all vertices in the trace. We assume the vertex trace to be ordered, so we measure
 * || vertex_0 - vertex_1 || + || vertex_1 + vertex_2 || + ...
 * @param context Measurement context
 * @param measurement_name name for this measurement
 */
{
    auto& measurement = context.get_measurement(measurement_name);
    measurement.measured_value = measure_vertex_trace(context.mesh, measurement.selection, measurement.options);

    if (context.options.write_debug_files) {
        write_vertex_selection(context.options.debug_base_dir / (measurement_name + "_selection.txt"), context.mesh, measurement.selection);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_bounding_box(MeasurementContext& context, const std::string& measurement_name) -> void
/**
 * Calculate bounding box of the body mesh and take the difference between maximum and minimum of the Y-axis
 * as measurement
 * @param context Measurement context
 * @param measurement_name name for this measurement
 */
{
    auto& measurement = context.get_measurement(measurement_name);
    auto bounding_box = pmp::bounds(context.mesh);
    // todo: do not use hardcoded Y-axis
    measurement.measured_value = bounding_box.max()[1] - bounding_box.min()[1];
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_landmark_distance_euclidean(MeasurementContext& context, const std::string& measurement_name) -> void
/**
 * Measure the euclidean distance between two given landmarks given in the measurement landmark list
 * @param context Measurement context
 * @param measurement_name name for this measurement
 */
{
    auto& measurement = context.get_measurement(measurement_name);

    if (measurement.landmarks.size() != 2) {
        throw std::runtime_error("Measure landmark distance (euclidean) needs exactly two landmarks. Measure " + measurement_name);
    }

    auto l0 = context.get_landmark(measurement.landmarks[0]);
    auto l1 = context.get_landmark(measurement.landmarks[1]);

    measurement.measured_value = pmp::distance(l0.position, l1.position);
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_head_l(MeasurementContext& context, const std::string& measurement_name) -> void
{
    auto& measurement = context.get_measurement(measurement_name);
    pmp::BoundingBox bbox = pmp::bounds(context.mesh);
    Landmark l0 = context.get_landmark("nape_of_neck");

    // y-axis for measurement
    const size_t axis = 1;
    measurement.measured_value = abs(bbox.max()[axis] - l0.position[axis]);
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * Measure against measurement plane normal axis
 * @param context
 * @param measurement_name
 */
auto measure_axis_inclination(MeasurementContext& context, const std::string& measurement_name) -> void
{
    // measure angle between layer normal and landmark vector
    // 0 degrees is perpendicular to the normal of the plane
    // rotate result 90 degrees (into plane), so that values < 0 are on the normal side
    // values > 0 are on the opposite side

    auto& measurement = context.get_measurement(measurement_name);

    if (measurement.landmarks.size() != 2) {
        throw std::runtime_error("Measure landmark distance (euclidean) needs exactly two landmarks. Measure " + measurement_name);
    }

    pmp::vec3 reference_axis = measurement.options.measurement_plane_normal_axis.as_normal();
    Landmark l0 = context.get_landmark(measurement.landmarks[0]);
    Landmark l1 = context.get_landmark(measurement.landmarks[1]);

    Plane reference_plane = Plane(pmp::vec3{0, 0, 0}, reference_axis);
    pmp::vec3 vector = l1.position - l0.position;

    measurement.measured_value = angle_plane_vec(reference_plane, vector);
}

// ---------------------------------------------------------------------------------------------------------------------

auto measure_landmark_distance_axis_aligned(MeasurementContext& context, const std::string& measurement_name) -> void
{
    auto& measurement = context.get_measurement(measurement_name);

    if (measurement.landmarks.size() != 2) {
        throw std::runtime_error("Axis aligned landmark distance needs two landmarks.");
    }

    Landmark l0 = context.get_landmark(measurement.landmarks[0]);
    Landmark l1 = context.get_landmark(measurement.landmarks[1]);
    auto axis_index = measurement.options.measurement_plane_normal_axis.axis_index();

    measurement.measured_value = abs(l0.position[axis_index] - l1.position[axis_index]);
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * Finds the nearest intersection point in the point set of intersection points of plane and mesh
 * Does not optimize to find a nearest point on a face
 * @param context
 * @param landmark_name which landmark should be updated?
 * @param measurement_name
 */
auto recompute_landmark_position(MeasurementContext& context, const std::string& landmark_name, const std::string& measurement_name) -> void
{
    Measurement& measurement = context.get_measurement(measurement_name);
    Landmark& landmark = context.get_landmark(landmark_name);

    pmp::vec3 current_landmark_pos = context.mesh.position(pmp::Vertex(landmark.vertex_id));
    landmark.position = current_landmark_pos;

    float min_dist = std::numeric_limits<float>::max();

    for (auto& segment : measurement.intersection_segments) {
        // p0
        float dist = pmp::distance(current_landmark_pos, segment.p0);
        if (min_dist > dist) {
            min_dist = dist;
            landmark.position = segment.p0;
        }

        // p1
        dist = pmp::distance(current_landmark_pos, segment.p1);
        if (min_dist > dist) {
            min_dist = dist;
            landmark.position = segment.p1;
        }
    }
}


// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================
