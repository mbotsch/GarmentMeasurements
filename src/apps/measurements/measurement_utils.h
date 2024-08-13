// =====================================================================================================================

#ifndef SHAPESFORGARMENTS_MEASUREMENT_UTILS_H
#define SHAPESFORGARMENTS_MEASUREMENT_UTILS_H

// =====================================================================================================================

#include "defs.h"

#include <cfloat>
#include <cmath>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_graham_andrew.h>
#include <nlohmann/json.hpp>

#include "measurement.h"

// =====================================================================================================================

// uncomment both lines, if you want to write unclosed loops (which should be closed)
//#define MEASUREMENT_UTILS_WRITE_UNCLOSED_LOOPS
//#define MEASUREMENTS_UTILS_WRITE_UNCLOSED_LOOPS_DIR "/mnt/ldata/fabian/tmp/eth-measurements/debug"

// =====================================================================================================================

NAMESPACE_BEGIN(mes_utils)

// =====================================================================================================================

using json = nlohmann::json;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;

// =====================================================================================================================

auto centroid(const std::vector<pmp::Point>& positions) -> pmp::Point;
auto nearest_vertex(const SurfaceMesh& mesh, const pmp::vec3& point) -> pmp::Vertex;

auto axis_aligned_centroid_plane(const SurfaceMesh& mesh, const Selection& selection, const MeasurementOptions& options) -> Plane;

auto signed_distance(const Plane& plane, const pmp::Point& point) -> double;
auto angle_plane_vec(const Plane& plane, pmp::vec3 vector) -> double;

auto build_halfspaces(MeasurementContext& context, const Measurement& measurement) -> std::vector<Halfspace>;
auto segments_in_half_space(const std::vector<FaceSegment>& surface_segments, const Halfspace& halfspace) -> std::vector<FaceSegment>;

auto face_ring_around_selection(const SurfaceMesh& mesh, const Selection& selection, size_t num_rings) -> std::set<pmp::Face>;

auto ray_plane_intersection(const pmp::Point& p0, const pmp::Point& p1, const Plane& plane) -> bool;
auto ray_plane_intersection_t(const pmp::Point& p0, const pmp::Point& p1, const Plane& plane, pmp::Point& intersection_point) -> float;
auto mesh_plane_intersection_loops_ordered(
    SurfaceMesh& mesh,
    const std::set<pmp::Face>& face_candidates,
    const Plane& plane,
    bool& error,
    bool add_non_candidates,
    bool complete_loop,
    bool raise_warning
) -> std::vector<std::vector<FaceSegment>>;

auto select_intersection_loop(
    // a list containing all intersection loops
    const std::vector<std::vector<FaceSegment>>& loops,
    bool merge_loops = false
) -> std::vector<FaceSegment>;

auto make_convex_ring(
    const std::vector<FaceSegment>& surface_segments,
    const Plane& intersection_plane,
    const Axis& projection_plane_normal
) -> std::vector<FaceSegment>;

auto make_convex_curve(
    const std::vector<FaceSegment>& surface_segments,
    const Plane& intersection_plane,
    const Axis& projection_plane_normal_axis,
    const pmp::vec3& dominant_normal,
    float deviance_min_cos = 0.0F
) -> std::vector<FaceSegment>;


// measure face segments given
auto measure_surface_segments(const std::vector<FaceSegment>& surface_segments) -> double;

// measure vertex trace given
auto measure_vertex_trace(const SurfaceMesh& mesh, const Selection& selection, const MeasurementOptions& options) -> double;


// =====================================================================================================================

NAMESPACE_END(mes_utils)

// =====================================================================================================================

#endif //SHAPESFORGARMENTS_MEASUREMENT_UTILS_H

// =====================================================================================================================
