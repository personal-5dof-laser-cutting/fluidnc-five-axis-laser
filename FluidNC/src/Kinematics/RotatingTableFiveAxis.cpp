//
// Created by Tom Haas on 15.10.25.
//

#include "RotatingTableFiveAxis.h"

#include "../Machine/MachineConfig.h"

#include <cmath>

namespace Kinematics {

    void RotatingTableFiveAxis::group(Configuration::HandlerBase& handler) {
        handler.item("max_z_mm", _max_z);
        handler.item("focus_offset", _focus_offset);
        handler.item("max_angle", _max_angle);
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm);
        handler.item("center_x", _center_x);
        handler.item("center_y", _center_y);
        handler.item("rotation_offset", _rotation_offset);
        handler.item("z_height", _z_height);
    }

    void RotatingTableFiveAxis::init() {
        //log_info("Kinematic system:" << name() << " roll_offset:" << _roll_offset);
    }

    void RotatingTableFiveAxis::init_position() {
        float angles[MAX_N_AXIS]    = { 0.0, 0.0, 0.0 };
        float cartesian[MAX_N_AXIS] = { 0.0, 0.0, 0.0 };
        // Calculate the Z offset at the arm zero angles ...
        // Z offset is the z distance from the motor axes to the end effector axes at zero angle
        motors_to_cartesian(cartesian, angles, 3);  // Sets the cartesian values
        log_info("  Z Offset:" << cartesian[Z_AXIS]);
    }


    // ANYTHING BElOW IS WRONG.
    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors, float* cartesian) {
        motors[0] = motors[1] = motors[2] = motors[3] = motors[4] = 0;

        constexpr float to_radians = M_PI / 180;
        constexpr float to_degrees = 180 / M_PI;

        // bool calc_ok = false;

        float dx = tan(cartesian[B_AXIS] * to_radians) * cartesian[Z_AXIS], dy = tan(cartesian[A_AXIS] * to_radians) * cartesian[Z_AXIS] ;
        float leng = sqrt(dx * dx + dy * dy);

        // laser head tilt
        float beta = atan2(leng, cartesian[Z_AXIS]); //atan(leng / cartesian[2]);
        // table rotation
        float alpha = atan2(dy, dx); // atan(dy / dx);
        motors[A_AXIS] = -alpha * to_degrees;
        motors[B_AXIS] = beta * to_degrees;

        float orx = cartesian[X_AXIS] - _center_x, ry = cartesian[Y_AXIS] - _center_y;
        float rx = cos(alpha) * orx - sin(alpha) * ry;
        ry = sin(alpha) * orx + cos(alpha) * ry;

        float x_offset = tan(beta) * (_focus_offset + _rotation_offset);

        //float dz = _rotation_offset - cos(beta) * _rotation_offset;
        float dz = (_rotation_offset + _focus_offset) / cos(beta) - (_rotation_offset + _focus_offset);

        rx += x_offset;

        motors[X_AXIS] = rx + _center_x;
        motors[Y_AXIS] = ry + _center_y;

        // motors[2] = -(_rotation_offset - cos(alpha) * _rotation_offset);
        motors[Z_AXIS] = _z_height + dz - _focus_offset - cartesian[Z_AXIS] / 2;
        //motors[Z_AXIS] = _offset_z + (cartesian[Z_AXIS] / 2) - dz;



        return true;
    }

    void RotatingTableFiveAxis::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        constexpr float to_radians = M_PI / 180;
        constexpr float to_degrees = 180 / M_PI;

        float alpha = motors[A_AXIS] * to_radians, beta = motors[B_AXIS] * to_radians;

        //float dz = _rotation_offset - cos(beta) * _rotation_offset;
        float dz = (_rotation_offset + _focus_offset) / cos(beta) - (_rotation_offset + _focus_offset);

        float z = motors[Z_AXIS] - _z_height - dz + _focus_offset;
        z*=-2;

        cartesian[Z_AXIS] = z;

        float x = motors[X_AXIS];
        float y = motors[Y_AXIS];

        float x_offset = tan(beta) * (_focus_offset + _rotation_offset);

        x -= x_offset;
        x -= _center_x;
        y -= _center_y;

        float rx = cos(alpha) * x + sin(alpha) * y;
        float ry = -sin(alpha) * x + cos(alpha) * y;

        cartesian[X_AXIS] = rx + _center_x;
        cartesian[Y_AXIS] = ry + _center_y;

        float leng = tan(beta) * z;

        float dy = sin(alpha) * leng;
        float dx = cos(alpha) * leng;

        cartesian[A_AXIS] = atan2(dy, z) * to_degrees;
        cartesian[B_AXIS] = atan2(dx, z) * to_degrees;

        if( z == 0) {
            cartesian[A_AXIS] = 0;
            cartesian[B_AXIS] = 0;
        }
    }

    bool RotatingTableFiveAxis::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        float dx, dy, dz, da, db;
        float motor_angles[5];

        float seq_target[5];
        float feed_rate = pl_data->feed_rate;
        bool show_error = true;

        bool calc_ok = true;

        dx = target[X_AXIS] - position[X_AXIS];
        dy = target[Y_AXIS] - position[Y_AXIS];
        dz = target[Z_AXIS] - position[Z_AXIS];
        da = target[A_AXIS] - position[A_AXIS];
        db = target[B_AXIS] - position[B_AXIS];

        float motors[5] = {0};

        if (!transform_cartesian_to_motors(motors, target)) {
            return false;
        }

        return mc_move_motors(motors, pl_data);
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<RotatingTableFiveAxis> registration("rotating_table_five_axis");
    }

}  // namespace Kinematics