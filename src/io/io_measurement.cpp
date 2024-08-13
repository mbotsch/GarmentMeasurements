#include "io_measurement.h"

#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

// =====================================================================================================================

using json = nlohmann::json;

// =====================================================================================================================
// helper methods

// ---------------------------------------------------------------------------------------------------------------------

auto dump_measurement_type(const MeasurementType& value) -> std::string
{
    switch (value) {
        case MeasurementType::AxisAlignedPlane:
            return "AxisAlignedPlane";
        case MeasurementType::AxisAlignedPlanePart:
            return "AxisAlignedPlanePart";
        case MeasurementType::AxisInclination:
            return "AxisInclination";
        case MeasurementType::Geodesic:
            return "Geodesic";
        case MeasurementType::VertexTrace:
            return "VertexTrace";
        case MeasurementType::LandmarkDistance:
            return "LandmarkDistance";
        case MeasurementType::LandmarkDistanceAxisAligned:
            return "LandmarkDistanceAxisAligned";
        case MeasurementType::AxisAlignedBoundingBox:
            return "AxisAlignedBoundingBox";
        case MeasurementType::HeadLength:
            return "HeadLength";
        default:
            std::cout << "Could not convert measurement_type to string\n";
            return "NONE";
    }
    throw std::runtime_error("IO: No conversion defined for measurement type.");
}

// ---------------------------------------------------------------------------------------------------------------------

auto parse_measurement_type(const std::string& value) -> MeasurementType
{
    if (value == "AxisAlignedPlane") {
        return MeasurementType::AxisAlignedPlane;
    }
    if (value == "AxisAlignedPlanePart") {
        return MeasurementType::AxisAlignedPlanePart;
    }
    if (value == "AxisInclination") {
        return MeasurementType::AxisInclination;
    }
    if (value == "VertexTrace") {
        return MeasurementType::VertexTrace;
    }
    if (value == "Geodesic") {
        return MeasurementType::Geodesic;
    }
    if (value == "LandmarkDistance") {
        return MeasurementType::LandmarkDistance;
    }
    if (value == "LandmarkDistanceAxisAligned") {
        return MeasurementType::LandmarkDistanceAxisAligned;
    }
    if (value == "AxisAlignedBoundingBox") {
        return MeasurementType::AxisAlignedBoundingBox;
    }
    if (value == "HeadLength") {
        return MeasurementType::HeadLength;
    }
    if (value == "NONE") {
        return MeasurementType::NONE;
    }

    throw std::runtime_error("IO: Unknown measurement type " + value);
}

// ---------------------------------------------------------------------------------------------------------------------

auto dump_measurement_plane_optimization_type(MeasurementPlaneOptimization value) -> std::string
{
    switch (value) {

        case MeasurementPlaneOptimization::NONE:
            return "NONE";
        case MeasurementPlaneOptimization::MIN:
            return "MIN";
        case MeasurementPlaneOptimization::MAX:
            return "MAX";
        case MeasurementPlaneOptimization::DOWN:
            return "DOWN";
        case MeasurementPlaneOptimization::UP:
            return "UP";
    }
    throw std::runtime_error("IO: No conversion defined for measurement plane optimization.");
}

// ---------------------------------------------------------------------------------------------------------------------

auto dump_measurement_plane_type(const Measurement& measurement) -> std::string
{
    if (measurement.options.measurement_plane_type == MeasurementPlaneType::CENTROID) {
        return "CENTROID";
    }
    if (measurement.options.measurement_plane_type == MeasurementPlaneType::LANDMARKS) {
        return "LANDMARKS";
    }
    if (measurement.options.measurement_plane_type == MeasurementPlaneType::NORMAL_AXIS) {
        return "NORMAL_AXIS";
    }
    return "NONE";
}

// ---------------------------------------------------------------------------------------------------------------------

auto parse_measurement_plane_optimization_type(const std::string& value) -> MeasurementPlaneOptimization
{
    if (value == "MAX") {
        return MeasurementPlaneOptimization::MAX;
    }
    if (value == "MIN") {
        return MeasurementPlaneOptimization::MIN;
    }
    if (value == "DOWN") {
        return MeasurementPlaneOptimization::DOWN;
    }
    if (value == "UP") {
        return MeasurementPlaneOptimization::UP;
    }
    return MeasurementPlaneOptimization::NONE;
}

// ---------------------------------------------------------------------------------------------------------------------

auto parse_measurement_unit(const std::string& value) -> MeasurementUnit
{
    if (value == "DEGREES") {
        return MeasurementUnit::DEGREES;
    }
    return MeasurementUnit::METERS;
}

// ---------------------------------------------------------------------------------------------------------------------

auto parse_normal(const json& value) -> pmp::vec3
{
    if (not value.is_array()) {
        std::cerr << "[ERROR] parse_normal: Non-array type given.\n";
        return {};
    }

    std::vector<float> coordinates = value.get<std::vector<float>>();

    if (coordinates.size() != 3) {
        std::cerr << "[ERROR] parse_normal: Normal entry must contain exactly 3 values.\n";
    }

    return pmp::normalize(pmp::vec3{coordinates[0], coordinates[1], coordinates[2]});
}

// ---------------------------------------------------------------------------------------------------------------------

auto parse_measurement_plane(const json& plane_data, Measurement& measurement) -> void
{
    // type
    if (plane_data.contains("type")) {
        auto plane_type = plane_data.value<std::string>("type", "undefined");
        if (plane_type == "CENTROID") {
            measurement.options.measurement_plane_type = MeasurementPlaneType::CENTROID;
        }
        else if (plane_type == "LANDMARKS") {
            measurement.options.measurement_plane_type = MeasurementPlaneType::LANDMARKS;
        }
        else if (plane_type == "NORMAL_AXIS") {
            measurement.options.measurement_plane_type = MeasurementPlaneType::NORMAL_AXIS;
        }
        else if (plane_type == "NONE") {
            measurement.options.measurement_plane_type = MeasurementPlaneType::NONE;
        }
        else {
            std::cerr << "[ERROR] Unknown measurement plane type: " << plane_type << '\n';
        }
    } else {
        std::cerr << "[ERROR] parse_measurement_plane: measurement_plane.type is not defined.\n";
    }

    // normal axis?
    if (plane_data.contains("normal_axis")) {
        measurement.options.measurement_plane_normal_axis = Axis(plane_data.value<std::string>("normal_axis", "None"));
    }

    // load landmark axis
    if (plane_data.contains("landmarks")) {
        measurement.options.measurement_plane_landmarks = plane_data.value<std::vector<std::string>>("landmarks", {});
    }

    if (plane_data.contains("landmarks_second_axis")) {
        measurement.options.measurement_plane_landmarks_second_axis = Axis(plane_data.value<std::string>("landmarks_second_axis", "None"));
    }

    // plane optimization
    measurement.options.measurement_plane_optimization = parse_measurement_plane_optimization_type(plane_data.value("optimization", "NONE"));
}

// =====================================================================================================================

bool read_measurement(const std::string& fn_measurement, Measurement& measurement)
{
    try {
        std::ifstream ifs(fn_measurement);
        json measurement_json = json::parse(ifs);

        // read name
        measurement.name = measurement_json["name"];

        // read ordered selection
        measurement.selection.clear();
        auto selection = measurement_json.value("selection", std::vector<int> {});
        measurement.selection.insert(measurement.selection.cend(), selection.cbegin(), selection.cend());

        // reset plane (to origin facing z-up)
        measurement.measurement_plane = Plane{pmp::vec3{0.0, 0.0, 0.0}, pmp::vec3{0.0, 0.0, 1.0}};
        measurement.intersection_segments.clear();

        // load options
        measurement.options.convex_hull = measurement_json.value("is_convex_hull", false);
        measurement.options.enforce_candidate_list = measurement_json.value("enforce_candidate_list", true);
        measurement.options.is_ring = measurement_json.value("is_ring", false);
        measurement.options.num_face_rings = measurement_json.value("num_face_rings", 6);
        measurement.options.internal_value = measurement_json.value("internal_value", false);

        measurement.measurement_type = parse_measurement_type(measurement_json.value("type", "None"));

        // load measurement plane
        if (measurement_json.contains("measurement_plane")) {
            parse_measurement_plane(measurement_json["measurement_plane"], measurement);
        }

        // read landmarks (for geodesics)
        if (measurement_json.contains("landmarks")) {
            measurement.landmarks = measurement_json.value<std::vector<std::string>>("landmarks", {});
        }

        // read half spaces
        if (measurement_json.contains("halfspaces")) {
            measurement.options.halfspace_recipes.clear();
            for (const json& entry : measurement_json.value<std::vector<json>>("halfspaces", {})) {
                measurement.options.halfspace_recipes.emplace_back(entry);
            }
        }

        // convex hull normal
        if (measurement_json.contains("convex_hull_normal")) {
            measurement.options.convex_hull_normal = parse_normal(measurement_json["convex_hull_normal"]);
        }

        // minimum cos distance for normal of convex hull
        measurement.options.convex_hull_normal_deviance_min_cos = measurement_json.value<float>("convex_hull_normal_deviance_min_cos", 0.0F);

        // read measurement unit
        measurement.unit = parse_measurement_unit(measurement_json.value<std::string>("unit", "undefined"));

        // reset values
        measurement.measured_value = 0.0;

    } catch (std::exception& error) {
        std::cerr << error.what() << std::endl;
        return false;
    }

    return true;
}

bool write_measurement(const std::string& fn_measurement, const Measurement& measurement)
{
    json json_measurement;
    json_measurement["name"] = measurement.name;
    json_measurement["selection"] = measurement.selection;

    // placeholder in measurement configuration
    json_measurement["num_face_rings"] = measurement.options.num_face_rings;
    json_measurement["type"] = dump_measurement_type(measurement.measurement_type);

    // landmarks
    if (not measurement.landmarks.empty()) {
        json_measurement["landmarks"] = measurement.landmarks;
    }

    // connect first and last vertex of vertex trace
    if (measurement.options.is_ring) {
        json_measurement["is_ring"] = measurement.options.is_ring;
    }


    // use "2d convex hull" of mesh-plane intersection
    if (measurement.options.convex_hull) {
        json_measurement["is_convex_hull"] = measurement.options.convex_hull;
    }

    // write measurement plane (and optimization)
    if (measurement.options.measurement_plane_optimization != MeasurementPlaneOptimization::NONE) {
        json measurement_plane {};
        measurement_plane["type"] = dump_measurement_plane_type(measurement);
        measurement_plane["optimization"] = dump_measurement_plane_optimization_type(
                measurement.options.measurement_plane_optimization);
        measurement_plane["normal_axis"] = measurement.options.measurement_plane_normal_axis.to_string();
        json_measurement["measurement_plane"] = measurement_plane;
    }

    // halfspaces
    if (not measurement.options.halfspace_recipes.empty()) {
        json_measurement["halfspaces"] = json::array();
        for (auto& hs_recipe : measurement.options.halfspace_recipes) {
            json_measurement["halfspaces"].push_back(hs_recipe.recipe);
        }
    }

    if (measurement.options.internal_value) {
        json_measurement["internal_value"] = true;
    }


    try {
        std::ofstream ofs(fn_measurement);
        ofs << std::setw(2) << json_measurement << std::endl;
    } catch (std::exception& error) {
        std::cerr << error.what();
    }

    std::cout << "Saved measurement " << fn_measurement << std::endl;

    return true;
}

bool write_measurement_yaml(const std::string &fn_output, const MeasurementContext& context) {
    try {
        std::ofstream ofs(fn_output);
        ofs << "body:\n";

        for (auto& it : context.measurements)
        {
            // do not save internal values
            if (it.second.options.internal_value) {
                continue;
            }

            auto value = it.second.measured_value;
            if (it.second.unit == MeasurementUnit::METERS && it.second.measured_value != -1.0) {
                value *= 100.0;
            }

            ofs << "  " <<  it.second.name << ": "
                // convert meters to centimeters
                // std::scientific, std::fixed
                // std::setw(5)
                << std::setprecision(6) << value << '\n';
        }
    } catch (const std::exception& exception) {
        std::cout << exception.what() << std::endl;
        return false;
    }

    return true;
}
