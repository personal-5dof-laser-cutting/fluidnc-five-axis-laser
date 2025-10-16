//
// Created by Tom Haas on 15.10.25.
//

#include "RotatingTableFiveAxis.h"

#include "../Machine/MachineConfig.h"

namespace Kinematics {

    void RotatingTableFiveAxis::group(Configuration::HandlerBase& handler) {
        //handler.item("roll_offset", _roll_offset, -360.0f, 360.0f);
    }

    void RotatingTableFiveAxis::init() {
        //log_info("Kinematic system:" << name() << " roll_offset:" << _roll_offset);
    }
    // ANYTHING BElOW IS WRONG.
    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors, float* cartesian) {
        return true;
    }

    void RotatingTableFiveAxis::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
    }

    bool RotatingTableFiveAxis::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        return true;
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<RotatingTableFiveAxis> registration("rotating_table_five_axis");
    }

}  // namespace Kinematics