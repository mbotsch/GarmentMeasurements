// =====================================================================================================================

#pragma once

#include <map>

#include <nlohmann/json.hpp>
#include <utility>
#include <pmp/surface_mesh.h>

#include "defs.h"

// =====================================================================================================================

using json = nlohmann::json;
using SurfaceMesh = pmp::SurfaceMesh;
using Selection = std::vector<int>;

// =====================================================================================================================
// types

struct Landmark
{
    size_t      vertex_id;
    pmp::Point  position;
};

struct Plane
{
    pmp::Point p = {};
    pmp::vec3  n = {};

    Plane() : p(), n() {}
    Plane(const pmp::Point point, const pmp::vec3 normal) : p(point), n(normal) {}

    [[nodiscard]] std::string to_string() const {
        std::stringstream ss;
        ss << "Plane[point="<< p << ", normal=" << n << "]";
        return ss.str();
    }
};

struct Halfspace : public Plane
{
    // origin, axis_0: two landmarks, axis_1: axis_aligned orientiation
    // uses a right-handed coordinate system
    Halfspace(pmp::Point origin, Landmark axis_0_start, Landmark axis_0_end, pmp::vec3 axis_1) : Plane() {
        auto axis0 = pmp::normalize(axis_0_end.position - axis_0_start.position);
        auto axis1 = pmp::normalize(axis_1);

        p = origin;
        n = pmp::cross(axis0, axis1);
   }

   // origin in plane and two axis in plane as vectors
   // uses a right-handed coordinate system
   Halfspace(pmp::Point origin, const pmp::vec3& axis_0, const pmp::vec3& axis_1) : Plane() {
       auto axis0 = pmp::normalize(axis_0);
       auto axis1 = pmp::normalize(axis_1);

       p = origin;
       n = pmp::cross(axis0, axis1);
    }

   // given plane initialization
   Halfspace(pmp::Point point, pmp::vec3 normal) : Plane(point, normal) {};
};

// forward declaration
struct HalfspaceRecipe;

//enum struct Axis
//{
//    X, Y, Z, X_NEG, Y_NEG, Z_NEG,
//};

class Axis
{
public:
    enum Value : uint8_t
    {
        X,
        Y,
        Z,
        X_NEG,
        Y_NEG,
        Z_NEG,
    };

    Axis() = default;
    constexpr Axis(Value axis) : value(axis) { }

    // create from string
    Axis(const std::string& name) {
        if (name == "X") { (*this) = X; return; }
        if (name == "Y") { (*this) = Y; return; }
        if (name == "Z") { (*this) = Z; return; }
        if (name == "-X") { (*this) = X_NEG; return; }
        if (name == "-Y") { (*this) = Y_NEG; return; }
        if (name == "-Z") { (*this) = Z_NEG; return; }
        throw std::runtime_error("Unknown axis name " + name);
    }

    // allow switch and comparison (switch case)
    constexpr operator Value() const { return value; }

    // prevent usage if (axis)
    explicit operator bool() const = delete;

    // comparisons
    constexpr bool operator==(Axis a) const { return value == a.value; }
    constexpr bool operator!=(Axis a) const { return value != a.value; }

    // suppress warnings: ambiguous overload for ‘operator==’ (operand types are ‘const Axis’ and ‘Axis::Value’
    constexpr bool operator==(Axis::Value a) const { return value == a; }
    constexpr bool operator!=(Axis::Value a) const { return value != a; }

    // convert to string
    [[nodiscard]] std::string to_string() const {
        switch (this->value) {
            case X:
                return "X";
            case Y:
                return "Y";
            case Z:
                return "Z";
            case X_NEG:
                return "-X";
            case Y_NEG:
                return "-Y";
            case Z_NEG:
                return "-Z";
        }
        return "";
    }

    [[nodiscard]] auto as_normal() const -> pmp::vec3 {
        switch (value) {
            case Axis::X:
                return pmp::vec3{ 1.0, 0.0, 0.0 };
            case Axis::Y:
                return pmp::vec3{ 0.0, 1.0, 0.0 };
            case Axis::Z:
                return pmp::vec3{ 0.0, 0.0, 1.0 };
            case Axis::X_NEG:
                return pmp::vec3{ -1.0, 0.0, 0.0 };
            case Axis::Y_NEG:
                return pmp::vec3{ 0.0, -1.0, 0.0 };
            case Axis::Z_NEG:
                return pmp::vec3{ 0.0, 0.0, -1.0 };
        }

        // error!
        return pmp::vec3{ 0.0, 0.0, 0.0 };
    }

    [[nodiscard]] auto axis_index() const -> size_t {
        switch (value) {
            case X:
            case X_NEG:
                return 0;
            case Y:
            case Y_NEG:
                return 1;
            default:
                return 2;
        }
    }

    [[nodiscard]] auto axis_sign() const -> long {
        switch (value) {
            case X:
            case Y:
            case Z:
                return 1L;
            case X_NEG:
            case Y_NEG:
            case Z_NEG:
                return -1L;
            default:
                return 0L;
        }
    }

private:
    Value value;
};


enum struct MeasurementType
{
    // define plane from vertex property
    AxisAlignedPlane,

    // axis aligned plane must define one or more half-space plane which separate the curve
    AxisAlignedPlanePart,

    // angle between axis (measurement_plane_axis) and two landmarks
    AxisInclination,

    // geodesic must define two landmarks, result is geodesic distance
    Geodesic,
    VertexTrace,

    // distance between two landmarks
    LandmarkDistance,

    // distance between two planes or a plane and a landmark
    LandmarkDistanceAxisAligned,

    // Distance for the axis aligned bounding box
    // used ONLY for height
    AxisAlignedBoundingBox,

    // Head length special case measurement
    HeadLength,

    // None value
    NONE,
};

enum struct MeasurementPlaneOptimization
{
    NONE,
    MIN,
    MAX,
    DOWN,
    UP,
};

enum struct MeasurementPlaneType
{
    NONE,
    CENTROID,
    LANDMARKS,
    NORMAL_AXIS,
};

struct MeasurementOptions
{
    // alignment axis for intersection plane
    // e.g. a ground plane normal / or an axis for inclination
    Axis measurement_plane_normal_axis = Axis::Y;

    // how to construct the measurement plane
    MeasurementPlaneType measurement_plane_type = MeasurementPlaneType::NONE;
    // optional: search minimum or maximum in neighbourhood of plane
    MeasurementPlaneOptimization measurement_plane_optimization = MeasurementPlaneOptimization::NONE;
    // start and end landmark for part measurements (zero or two landmarks)
    std::vector<std::string> measurement_plane_landmarks;
    Axis measurement_plane_landmarks_second_axis = Axis::Z;

    // include distance between first and last point
    bool is_ring = true;
    // do only add faces from candidate list to intersection
    bool enforce_candidate_list = true;

    // number of face rings around selection to search for intersections
    unsigned int num_face_rings = 6;

    // halfspaces
    std::vector<HalfspaceRecipe> halfspace_recipes {};

    // use convex hull of intersection points for measurement
    bool convex_hull = false;
    // convex hull options
    pmp::vec3 convex_hull_normal {};
    float convex_hull_normal_deviance_min_cos = 0.0F;

    // if true, this value is not saved in final yaml file.
    bool internal_value = false;
};

enum struct MeasurementUnit
{
    METERS,
    DEGREES,
};

struct FaceSegment
{
    pmp::Point p0;
    pmp::Point p1;

    FaceSegment(const pmp::Point& point0, const pmp::Point& point1) : p0 {point0}, p1 {point1} { }
};

struct Measurement
{
    std::string name;
    MeasurementUnit unit = MeasurementUnit::METERS;
    MeasurementType measurement_type = MeasurementType::NONE;

    MeasurementOptions options = {};
    Selection selection = {};

    double measured_value = 0.0;
    Plane  measurement_plane = {};

    std::vector<std::string> landmarks;

    // all intersection points of plane with mesh (may contain duplicates)
    std::vector<FaceSegment> intersection_segments {};
    std::vector<FaceSegment> intersection_segments_convex {};
};

struct MeasurementContextOptions
{
    bool print_stdout = false;
    bool write_debug_files = false;
    std::filesystem::path debug_base_dir = {""};
};

struct MeasurementContext
{
    pmp::SurfaceMesh mesh;
    MeasurementContextOptions options {};

    std::map<std::string, Measurement> measurements;
    std::map<std::string, Landmark> landmarks;

    auto get_landmark(const std::string& name) -> Landmark&;
    [[nodiscard]] auto get_landmark(const std::string& name) const -> const Landmark&;
    auto get_measurement(const std::string& name) -> Measurement&;
};


struct HalfspaceRecipe : json
{
    HalfspaceRecipe(json p_recipe) : recipe(std::move(p_recipe)) {

    }

    json recipe;

    [[nodiscard]] auto to_halfspace(MeasurementContext& context) const -> Halfspace;
};

// =====================================================================================================================

