// =====================================================================================================================

#include "measurement.h"

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto MeasurementContext::get_landmark(const std::string& name) -> Landmark&
{
    auto entry = landmarks.find(name);
    if (entry == landmarks.end())
    {
        throw std::runtime_error("[ERROR] Landmark " + name + " not defined.");
    }
    return entry->second;
}

// ---------------------------------------------------------------------------------------------------------------------

auto MeasurementContext::get_landmark(const std::string& name) const -> const Landmark&
{
    const auto entry = landmarks.find(name);
    if (entry == landmarks.end())
    {
        throw std::runtime_error("[ERROR] Landmark " + name + " not defined.");
    }
    return entry->second;
}

// ---------------------------------------------------------------------------------------------------------------------

auto MeasurementContext::get_measurement(const std::string& name) -> Measurement&
{
    auto entry = measurements.find(name);
    if (entry == measurements.end())
    {
        throw std::runtime_error("[ERROR] Measurement " + name + " not defined.");
    }
    return entry->second;
}

// ---------------------------------------------------------------------------------------------------------------------

auto HalfspaceRecipe::to_halfspace(MeasurementContext& context) const -> Halfspace
{
    // landmark_axis or axis_axis
    auto origin = recipe.value<std::string>("origin", "undefined");

    auto lm_origin = context.get_landmark(origin);
    auto axes = recipe.value<std::vector<json>>("axes", {});

    if (axes.size() == 1) {
        auto halfspace_type = axes[0].value<std::string>("type", "undefined");
        if (halfspace_type != "normal") {
            throw std::runtime_error("When defining only one axis, this axis must be a normal.");
        }
        auto normal_axis = Axis(axes[0].value<std::string>("axis", "undefined"));
        return { lm_origin.position, normal_axis.as_normal() };
    }

    if (axes.size() == 2) {
        auto axis_0_type = axes[0].value<std::string>("type", "undefined");
        auto axis_1_type = axes[1].value<std::string>("type", "undefined");

        if (axis_0_type == "landmarks" && axis_1_type == "axis") {
            auto landmarks = axes[0].value<std::vector<std::string>>("landmarks", {});

            auto lm_0 = context.get_landmark(landmarks[0]);
            auto lm_1 = context.get_landmark(landmarks[1]);
            auto axis_1 = Axis(axes[1].value<std::string>("axis", "undefined"));

            return { lm_origin.position, lm_0, lm_1, axis_1.as_normal() };
        }

        if (axis_0_type == "axis" && axis_1_type == "axis") {
            auto axis_0 = Axis(axes[0].value<std::string>("axis", "undefined"));
            auto axis_1 = Axis(axes[1].value<std::string>("axis", "undefined"));

            return { lm_origin.position, axis_0.as_normal(), axis_1.as_normal() };
        }
    }

    throw std::runtime_error("Could not convert recipe into halfspace. Unsupported axis format.");
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

