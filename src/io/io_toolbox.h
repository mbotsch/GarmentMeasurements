// =====================================================================================================================

#ifndef SHAPESFORGARMENTS_IO_TOOLBOX_H
#define SHAPESFORGARMENTS_IO_TOOLBOX_H

// =====================================================================================================================

#include <fstream>
#include <set>
#include <string>

#include <pmp/surface_mesh.h>

#include "apps/measurements/measurement.h"

// =====================================================================================================================

auto write_face_vertex_selection(const std::string& filename, const pmp::SurfaceMesh& mesh, const std::set<pmp::Face>& faces) -> void;
auto write_vertex_selection(const std::string& filename, const pmp::SurfaceMesh& mesh, const std::vector<int>& vertices) -> void;
auto write_points(const std::string& filename, const std::vector<pmp::Point>& points) -> void;
auto write_landmarks(const std::string& filename, const MeasurementContext& context, const std::vector<std::string>& landmark_names) -> void;
auto write_face_segments(const std::string& filename, const std::vector<FaceSegment>& face_segments) -> void;
auto write_face_segments_trace(const std::string& filename, const std::vector<FaceSegment>& face_segments, unsigned int trace_points = 10) -> void;

// =====================================================================================================================

#endif //SHAPESFORGARMENTS_IO_TOOLBOX_H

// =====================================================================================================================
