//
// Created by Fabian Kemper on 11/29/23.
//

#ifndef SHAPESFORGARMENTS_IO_MEASUREMENT_H
#define SHAPESFORGARMENTS_IO_MEASUREMENT_H

#include <string>

#include "defs.h"

#include "../apps/measurements/measurement.h"

// =====================================================================================================================

bool read_measurement(const std::string& fn_measurement, Measurement& measurement);

bool write_measurement(const std::string& fn_measurement, const Measurement& measurement);

bool write_measurement_yaml(const std::string& fn_output, const MeasurementContext& context);

#endif //SHAPESFORGARMENTS_IO_MEASUREMENT_H
