// =====================================================================================================================

#include "io_toolbox.h"

// =====================================================================================================================

using SurfaceMesh = pmp::SurfaceMesh;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto write_face_vertex_selection(const std::string& filename, const SurfaceMesh& mesh, const std::set<pmp::Face>& faces) -> void
{
    try {
        std::ofstream ofs (filename);

        for (auto face : faces) {
            for (auto vertex : mesh.vertices(face)) {
                auto point = mesh.position(vertex);
                ofs << point << " 120 240 50 0.3 0.3 0.3" << '\n';
            }
        }
    } catch(std::exception& error) {
        std::cerr << error.what();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_vertex_selection(const std::string& filename, const pmp::SurfaceMesh& mesh, const std::vector<int>& vertices) -> void
{
    try {
        std::ofstream ofs (filename);
        for (auto vertex_index : vertices) {
            auto point = mesh.position(pmp::Vertex(vertex_index));
            ofs << point << " 240 120 50 0.3 0.3 0.3" << '\n';
        }
    } catch(std::exception& error) {
        std::cerr << error.what();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_points(const std::string& filename, const std::vector<pmp::Point>& points) -> void
{
    try {
        std::ofstream ofs (filename);
        for (auto point : points) {
            ofs << point << " 120 50 50 0.3 0.3 0.3" << '\n';
        }
    } catch(std::exception& error) {
        std::cerr << error.what();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_landmarks(const std::string& filename, const MeasurementContext& context, const std::vector<std::string>& landmark_names) -> void
{
    try {
        std::ofstream ofs (filename);
        for (const auto& name : landmark_names) {
            const auto& landmark = context.get_landmark(name);
            ofs << landmark.position << "120 0 120 0.3 0.3 0.3" << '\n';
        }
    } catch(std::exception& error) {
        std::cerr << error.what();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_face_segments(const std::string& filename, const std::vector<FaceSegment>& face_segments) -> void
{
    try {
        std::ofstream ofs (filename);
        for (auto segment : face_segments) {
            ofs << segment.p0 << " 120 50 50 0.3 0.3 0.3" << '\n';
            ofs << segment.p1 << " 120 50 120 0.3 0.3 0.3" << '\n';
        }
    } catch(std::exception& error) {
        std::cerr << error.what();
    }
}

// ---------------------------------------------------------------------------------------------------------------------


/**
 * Fill a trace of vertex segments
 *
 * @param filename
 * @param face_segments
 */
auto write_face_segments_trace(const std::string& filename, const std::vector<FaceSegment>& face_segments, unsigned int trace_points) -> void
{
    try {
        std::ofstream ofs (filename);
        for (auto segment : face_segments) {
            ofs << segment.p0 << " 120 50 50 0.3 0.3 0.3\n";

            for (int index = 0; index < static_cast<int>(trace_points); ++index) {
                auto p2 = (segment.p1 - segment.p0) * static_cast<float>(index+1) / static_cast<float>(trace_points + 1) + segment.p0;
                ofs << p2 << " 60 30 30 0.3 0.3 0.3\n";
            }

            ofs << segment.p1 << " 120 50 120 0.3 0.3 0.3\n";
        }
    } catch(std::exception& error) {
        std::cerr << error.what();
    }
}


// =====================================================================================================================
