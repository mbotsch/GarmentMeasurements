// =====================================================================================================================

#include <fstream>
#include <filesystem>
#include <ios>

#include <argparse/argparse.hpp>
#include <pmp/surface_mesh.h>
#include <pmp/io/io.h>

#include <io/io_measurement.h>

#include "measure_body.h"

#include "measurement_utils.h"

// =====================================================================================================================

auto print_measurement(const MeasurementContext& context, const Measurement& measurement)
{
    if (context.options.print_stdout) {
        std::cout << std::setw(22) << std::left << measurement.name << ": " << measurement.measured_value << '\n';
    }
}

// =====================================================================================================================

auto auto_measure(MeasurementContext& context, const std::string& measurement_name) -> void
{
    auto& measurement = context.get_measurement(measurement_name);
    switch (measurement.measurement_type) {
        case MeasurementType::AxisAlignedPlane:
        case MeasurementType::AxisAlignedPlanePart:
            measure_axis_aligned_circumference(context, measurement_name);
            break;
        case MeasurementType::AxisInclination:
            measure_axis_inclination(context, measurement_name);
            break;
        case MeasurementType::Geodesic:
            measure_geodesic_distance(context, measurement_name);
            break;
        case MeasurementType::VertexTrace:
            measure_vertex_trace(context, measurement_name);
            break;
        case MeasurementType::LandmarkDistance:
            measure_landmark_distance_euclidean(context, measurement_name);
            break;
        case MeasurementType::LandmarkDistanceAxisAligned:
            measure_landmark_distance_axis_aligned(context, measurement_name);
            break;
        case MeasurementType::AxisAlignedBoundingBox:
            measure_bounding_box(context, measurement_name);
            break;
        case MeasurementType::HeadLength:
            measure_head_l(context, measurement_name);
            break;
        default:
            std::cerr << "Could not auto measure " << measurement_name << '\n';
            break;
    }
}

// =====================================================================================================================

int main(int argc, char* argv[])
{
    argparse::ArgumentParser program("measurements");

    // positional argument
    program.add_argument("mesh")
        .help("Input mesh for measurement.");

    program.add_argument("output")
        .help("Output yaml file.")
        .default_value("measurements.yaml");

    program.add_argument("--data_dir")
        .help("Path to data directory of this project. Contains measurement files.")
        .default_value("../data");

    program.add_argument("--debug_dir")
        .help("Path for debugging files.")
        .default_value("");

    program.add_argument("--debug")
        .help("Write debug information when measure.")
        .default_value(false)
        .implicit_value(true);

    program.add_argument("--stdout")
        .help("Print measurements to stdout.")
        .flag();

    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    // data directory
    std::filesystem::path data_dir (program.get("data_dir"));
    data_dir = std::filesystem::absolute(data_dir);

    // read mesh
    pmp::SurfaceMesh mesh;
    pmp::read(mesh, program.get("mesh"));

    // context object stores landmarks and measured values and mesh
    MeasurementContext context = {};
    context.mesh = mesh;

    // options
    context.options.print_stdout = program.get<bool>("stdout");

    // debug mode
    context.options.write_debug_files = program.get<bool>("debug");
    context.options.debug_base_dir = std::filesystem::path(program.get<std::string>("debug_dir"));

    // Load landmarks
    if (!load_landmarks(data_dir / "landmarks_female.json", context.mesh, context.landmarks)) { return -1; }

    if (context.options.write_debug_files)
    {
        std::ofstream ofs(context.options.debug_base_dir / "landmarks_before.txt");
        for (auto& it : context.landmarks)
        {
            it.second.position = context.mesh.position(pmp::Vertex(it.second.vertex_id));
            ofs << it.second.position << " 200 35 35 0.3 0.3 0.3" << std::endl;
        }
    }

    // Load measurements
    Measurement& waist = context.measurements["waist"];
    read_measurement(data_dir / "measurements/waist.mes", waist);

    Measurement& bust = context.measurements["bust"];
    read_measurement(data_dir / "measurements/bust.mes", bust);

    Measurement& underbust = context.measurements["underbust"];
    read_measurement(data_dir / "measurements/underbust.mes", underbust);

    Measurement& hips = context.measurements["hips"];
    read_measurement(data_dir / "measurements/hips.mes", hips);

    Measurement& leg_circ = context.measurements["leg_circ"];
    leg_circ.name = "leg_circ";
    // note: leg circ is the minimum value of left and right leg circ measurement

    Measurement& leg_circ_left = context.measurements["leg_circ_left"];
    read_measurement(data_dir / "measurements/leg_circ_left.mes", leg_circ_left);

    Measurement& leg_circ_right = context.measurements["leg_circ_right"];
    read_measurement(data_dir / "measurements/leg_circ_right.mes", leg_circ_right);

    Measurement& waist_line = context.measurements["waist_line"];
    read_measurement(data_dir / "measurements/waist_line.mes", waist_line);

    Measurement& waist_over_bust_line = context.measurements["waist_over_bust_line"];
    read_measurement(data_dir / "measurements/waist_over_bust_line.mes", waist_over_bust_line);

    Measurement& waist_back_width = context.measurements["waist_back_width"];
    read_measurement(data_dir / "measurements/waist_back_width.mes", waist_back_width);

    Measurement& back_width = context.measurements["back_width"];
    read_measurement(data_dir / "measurements/back_width.mes", back_width);

    Measurement& hip_back_width = context.measurements["hip_back_width"];
    read_measurement(data_dir / "measurements/hip_back_width.mes", hip_back_width);

    Measurement& bust_line = context.measurements["bust_line"];
    read_measurement(data_dir / "measurements/bust_line.mes", bust_line);

    Measurement& wrist = context.measurements["wrist"];
    read_measurement(data_dir / "measurements/wrist.mes", wrist);

    Measurement& neck_w = context.measurements["neck_w"];
    read_measurement(data_dir / "measurements/neck_w.mes", neck_w);

    Measurement& arm_length = context.measurements["arm_length"];
    read_measurement(data_dir / "measurements/arm_length.mes", arm_length);

    Measurement& height = context.measurements["height"];
    read_measurement("../data/measurements/height.mes", height);

    Measurement& head_l = context.measurements["head_l"];
    read_measurement(data_dir / "measurements/head_l.mes", head_l);

    Measurement& shoulder_w = context.measurements["shoulder_w"];
    read_measurement(data_dir / "measurements/shoulder_w.mes", shoulder_w);

    Measurement& armscye_depth = context.measurements["armscye_depth"];
    read_measurement(data_dir / "measurements/armscye_depth.mes", armscye_depth);

    Measurement& bust_points = context.measurements["bust_points"];
    read_measurement(data_dir / "measurements/bust_points.mes", bust_points);

    Measurement& hips_line = context.measurements["hips_line"];
    read_measurement(data_dir / "measurements/hips_line.mes", hips_line);

    Measurement& bum_points = context.measurements["bum_points"];
    read_measurement(data_dir / "measurements/bum_points.mes", bum_points);

    Measurement& crotch_hip_diff = context.measurements["crotch_hip_diff"];
    read_measurement(data_dir / "measurements/crotch_hip_diff.mes", crotch_hip_diff);

    Measurement& vert_bust_line = context.measurements["vert_bust_line"];
    read_measurement(data_dir / "measurements/vert_bust_line.mes", vert_bust_line);

    Measurement& hip_inclination = context.measurements["hip_inclination"];
    read_measurement(data_dir / "measurements/hip_inclination.mes", hip_inclination);

    Measurement& shoulder_incl = context.measurements["shoulder_incl"];
    read_measurement(data_dir / "measurements/shoulder_incl.mes", shoulder_incl);

    Measurement& arm_pose_angle = context.measurements["arm_pose_angle"];
    read_measurement(data_dir / "measurements/arm_pose_angle.mes", arm_pose_angle);


    // Compute measurements
    auto_measure(context, "waist");
    recompute_landmark_position(context, "waist_side_left", "waist");
    recompute_landmark_position(context, "waist_side_right", "waist");
    recompute_landmark_position(context, "mid_waist_back", "waist");
    print_measurement(context, waist);

    auto_measure(context, "bust");
    recompute_landmark_position(context, "bust_right", "bust");
    recompute_landmark_position(context, "bust_left", "bust");
    recompute_landmark_position(context, "bust_side_left", "bust");
    recompute_landmark_position(context, "bust_side_right", "bust");
    print_measurement(context, bust);

    auto_measure(context, "underbust");
    print_measurement(context, underbust);

    auto_measure(context, "hips");
    recompute_landmark_position(context, "hips_side_left", "hips");
    recompute_landmark_position(context, "hips_side_right", "hips");
    recompute_landmark_position(context, "bum_left", "hips");
    recompute_landmark_position(context, "bum_right", "hips");
    print_measurement(context, hips);

    auto_measure(context, "leg_circ_left");
    auto_measure(context, "leg_circ_right");
    print_measurement(context, leg_circ_left);
    print_measurement(context, leg_circ_right);

    // REDUCE measurement to leg_circ measurement
    leg_circ.measured_value = std::min(leg_circ_left.measured_value, leg_circ_right.measured_value);
    print_measurement(context, leg_circ);

    // ----- distances (convex hull)
    auto_measure(context, "waist_line");
    print_measurement(context, waist_line);

    auto_measure(context, "waist_over_bust_line");
    print_measurement(context, waist_over_bust_line);

    // ----- selections (part of other paths)
    auto_measure(context, "waist_back_width");
    print_measurement(context, waist_back_width);

    auto_measure(context, "back_width");
    print_measurement(context, back_width);

    auto_measure(context, "hip_back_width");
    print_measurement(context, hip_back_width);

    auto_measure(context, "bust_line");
    print_measurement(context, bust_line);

    // ----- circumferences (as is)
    auto_measure(context, "wrist");
    print_measurement(context, wrist);
    auto_measure(context, "neck_w");
    print_measurement(context, neck_w);

    // ----- geodesics
    auto_measure(context, "arm_length");
    print_measurement(context, arm_length);

    // ----- distances euclidean
    auto_measure(context, "height");
    print_measurement(context, height);

    // only landmarks
    auto_measure(context, "head_l");
    print_measurement(context, head_l);

    auto_measure(context, "shoulder_w");
    print_measurement(context, shoulder_w);

    auto_measure(context, "armscye_depth");
    print_measurement(context, armscye_depth);

    auto_measure(context, "bust_points");
    print_measurement(context, bust_points);

    auto_measure(context, "hips_line");
    print_measurement(context, hips_line);

    auto_measure(context, "bum_points");
    print_measurement(context, bum_points);

    auto_measure(context, "crotch_hip_diff");
    print_measurement(context, crotch_hip_diff);

    auto_measure(context, "vert_bust_line");
    print_measurement(context, vert_bust_line);


    // ----- angles
    // measure against X plane (if using right side of body)
    // positive X is on the right body side, values > 0
    auto_measure(context, "hip_inclination");
    print_measurement(context, hip_inclination);

    auto_measure(context, "shoulder_incl");
    print_measurement(context, shoulder_incl);

    auto_measure(context, "arm_pose_angle");
    print_measurement(context, arm_pose_angle);

    // print everything
    std::cout << std::flush;

    if (context.options.write_debug_files)
    {
        std::ofstream ofs(context.options.debug_base_dir / "landmarks_after.txt");
        for (auto& it : context.landmarks)
        {
            ofs << it.second.position << " 35 200 35 0.3 0.3 0.3" << '\n';
        }

        ofs = std::ofstream(context.options.debug_base_dir / "landmarks.csv");
        ofs << "name,x,y,z\n";
        for (const auto& it : context.landmarks)
        {
            const auto& position = it.second.position;
            ofs << it.first << ',' << position[0] << ',' << position[1] << ',' << position[2]  << '\n';
        }
    }

    // yaml export
    write_measurement_yaml(program.get("output"), context);


    return 0;
}

// =====================================================================================================================
