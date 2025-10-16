//
// Created by Tom Haas on 15.10.25.
// Kinematics of a 5 axis machine (gantry with a rotating table and a tilting head).
// X, Y are the gantry axes
// Z is the vertical axis of the gantry
// ROLL_AXIS is the tilt axis of the head
// TABLE_AXIS is the rotation axis of the table
//

#ifndef INC_5AXISFLUIDNC_ROTATINGTABLEFIVEAXIS_H
#define INC_5AXISFLUIDNC_ROTATINGTABLEFIVEAXIS_H

#include "Kinematics.h"

// Axis aliases for clarity
#ifndef TABLE_AXIS
#define TABLE_AXIS B_AXIS
#endif
#ifndef ROLL_AXIS
#define ROLL_AXIS A_AXIS
#endif

namespace Kinematics {

    class RotatingTableFiveAxis : public KinematicSystem {
    public:
        explicit RotatingTableFiveAxis(const char* name) : KinematicSystem(name) {}

        // Configurable parameters
        void group(Configuration::HandlerBase& handler) override;

        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void init() override;
        void init_position() override;
        void motors_to_cartesian(float* cartesian, float* motors, int n_axis) override;
        bool transform_cartesian_to_motors(float* motors, float* cartesian) override;

    private:
        // Offset to align the reported TABLE_AXIS 0° to the physical home switch position.
        // Units: degrees (same units as the rotary axis is configured to use).
        float _roll_offset = 0.0f;
    };

}  // namespace Kinematics

#endif  // INC_5AXISFLUIDNC_ROTATINGTABLEFIVEAXIS_H
