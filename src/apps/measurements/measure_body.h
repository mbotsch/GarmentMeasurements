#ifndef SHAPESFORGARMENTS_MEASURE_BODY_H
#define SHAPESFORGARMENTS_MEASURE_BODY_H


#include "measurement.h"


// =====================================================================================================================

auto load_landmarks(const std::filesystem::path& path, const pmp::SurfaceMesh& mesh, std::map<std::string, Landmark>& landmarks) -> bool;

auto measure_axis_aligned_circumference(MeasurementContext& context, const std::string& measurement_name) -> void;

auto measure_geodesic_distance(MeasurementContext& context, const std::string& measurement_name) -> void;

auto measure_vertex_trace(MeasurementContext& context, const std::string& measurement_name) -> void;

auto measure_bounding_box(MeasurementContext& context, const std::string& measurement_name) -> void;

auto measure_landmark_distance_euclidean(MeasurementContext& context, const std::string& measurement_name) -> void;

auto measure_landmark_distance_axis_aligned(MeasurementContext& context, const std::string& measurement_name) -> void;

auto measure_head_l(MeasurementContext& context, const std::string& measurement_name) -> void;

auto measure_axis_inclination(MeasurementContext& context, const std::string& measurement_name) -> void;

auto recompute_landmark_position(MeasurementContext& context, const std::string& landmark_name, const std::string& measurement_name) -> void;


// =====================================================================================================================

#endif //SHAPESFORGARMENTS_MEASURE_BODY_H
