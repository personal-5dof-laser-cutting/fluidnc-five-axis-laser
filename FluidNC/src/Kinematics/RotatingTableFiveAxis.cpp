//
// Created by Tom Haas on 15.10.25.
//

#include "RotatingTableFiveAxis.h"

#include "../Machine/MachineConfig.h"

#include <cmath>

namespace Kinematics {

    static float last_mpos[5] = { 0.0 };

    void RotatingTableFiveAxis::group(Configuration::HandlerBase& handler) {
        Cartesian::group(handler);
        handler.item("max_z_mm", _max_z);
        handler.item("focus_offset", _focus_offset);
        handler.item("max_angle", _max_angle);
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm);
        handler.item("center_x", _center_x);
        handler.item("center_y", _center_y);
        handler.item("rotation_offset", _rotation_offset);
        handler.item("z_height", _z_height);
        handler.item("max_x", _max_x);
        handler.item("max_y", _max_y);
    }

    void RotatingTableFiveAxis::init() {
        this->init_position();
    }

    void RotatingTableFiveAxis::init_position() {
        auto n_axis = Axes::_numberAxis;
        for (size_t axis = 0; axis < n_axis; axis++) {
            set_motor_steps(axis, 0);  // Set to zeros
        }
    }

    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors_left, float* motors_right, float* cartesian) {
        log_info("RotatingTableFiveAxis::transform_cartesian_to_motors");
        log_info("cartesian: X:" << cartesian[X_AXIS] << " Y:" << cartesian[Y_AXIS] << " Z:" << cartesian[Z_AXIS]
                                 << " A:" << cartesian[A_AXIS] << " B:" << cartesian[B_AXIS]);
        memset(motors_right, 0, 5 * sizeof(float));
        memset(motors_left, 0, 5 * sizeof(float));

        constexpr float to_radians = M_PI / 180;
        constexpr float to_degrees = 180 / M_PI;

        // bool calc_ok = false;
        int negate = 1;

        if (cartesian[B_AXIS] < 0) {
            negate = -1;
            cartesian[B_AXIS] *= -1;
        }

        float dx = tan(cartesian[B_AXIS] * to_radians) * cartesian[Z_AXIS], dy = tan(cartesian[A_AXIS] * to_radians) * cartesian[Z_AXIS];
        float leng = sqrt(dx * dx + dy * dy);

        // laser head tilt
        float beta = negate * atan2(leng, cartesian[Z_AXIS]);  //atan(leng / cartesian[2]);
        // table rotation
        float alpha = negate * atan2(dy, dx);  // atan(dy / dx);

        if (alpha > M_PI) {
            alpha = 2 * M_PI - alpha;
        }
        motors_left[A_AXIS]  = -alpha * to_degrees;
        motors_right[A_AXIS] = motors_left[A_AXIS] - 180;

        motors_left[B_AXIS]  = -beta * to_degrees;
        motors_right[B_AXIS] = beta * to_degrees;

        float orx = cartesian[X_AXIS] - _center_x, ry = cartesian[Y_AXIS] - _center_y;
        float rx = cos(alpha) * orx - sin(alpha) * ry;
        ry       = sin(alpha) * orx + cos(alpha) * ry;

        float x_offset = tan(-beta) * (_focus_offset + _rotation_offset);

        //float dz = _rotation_offset - cos(beta) * _rotation_offset;
        float dz = (_rotation_offset + _focus_offset) / cos(beta) - (_rotation_offset + _focus_offset);

        rx += x_offset;

        motors_left[X_AXIS] = rx + _center_x;
        motors_left[Y_AXIS] = ry + _center_y;

        // motors[2] = -(_rotation_offset - cos(alpha) * _rotation_offset);
        motors_left[Z_AXIS]  = _z_height + dz - _focus_offset - cartesian[Z_AXIS] / 2;
        motors_right[Z_AXIS] = motors_left[Z_AXIS];
        //motors[Z_AXIS] = _offset_z + (cartesian[Z_AXIS] / 2) - dz;

        rx = -rx;
        ry = -ry;

        motors_right[X_AXIS] = rx + _center_x;
        motors_right[Y_AXIS] = ry + _center_y;

        return true;
    }
    float angle_dist(float pos, float dest){
        pos = fmodf(pos, 360.0f);
        dest = fmodf(dest, 360.0f);
        float diff = dest - pos;
        if (diff > 180.0f) diff -= 360.0f;
        else if (diff < -180.0f) diff += 360.0f;

        return diff;
    }


    void shortest_move(float* pos, float* left, float* right){
        while (pos[A_AXIS] < 0) pos[A_AXIS] += 360.0f;
        while (left[A_AXIS] < 0) left[A_AXIS] += 360.0f;
        while (right[A_AXIS] < 0) right[A_AXIS] += 360.0f;

        float adl = angle_dist(pos[A_AXIS], left[A_AXIS]);
        float adr = angle_dist(pos[A_AXIS], right[A_AXIS]);
        log_info("left: " << left[A_AXIS] << " right:" << right[A_AXIS]);
        log_info("pos: " << pos[A_AXIS]);
        log_info("adl: " << adl << " adr:" << adr);
        if (abs(adl) < abs(adr)){
            log_info("Went for left")
            left[A_AXIS] = pos[A_AXIS] + adl;
            memcpy(pos, left, 5 * sizeof(pos[0]));
        }else{
            log_info("Went for right")
            right[A_AXIS] = pos[A_AXIS] + adr;
            memcpy(pos, right, 5 * sizeof(pos[0]));
        }
    }


    // ANYTHING BElOW IS WRONG.
    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors, float* cartesian) {
        if (state_is(State::Homing)) {
            return Cartesian::transform_cartesian_to_motors(motors, cartesian);
        }
        auto* motors_left  = new float[5];
        auto* motors_right = new float[5];
        transform_cartesian_to_motors(motors_left, motors_right, cartesian);
        shortest_move(last_mpos, motors_left, motors_right);
        delete[] motors_left;
        delete[] motors_right;
        memcpy(motors, last_mpos, 5 * sizeof(motors[0]));
        return true;
    }

    void RotatingTableFiveAxis::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        if (state_is(State::Homing)) {
            return Cartesian::motors_to_cartesian(cartesian, motors, n_axis);
        }

        constexpr float to_radians = M_PI / 180;
        constexpr float to_degrees = 180 / M_PI;

        float alpha = motors[A_AXIS] * to_radians, beta = -motors[B_AXIS] * to_radians;

        //float dz = _rotation_offset - cos(beta) * _rotation_offset;
        float dz = (_rotation_offset + _focus_offset) / cos(beta) - (_rotation_offset + _focus_offset);

        float z = motors[Z_AXIS] - _z_height - dz + _focus_offset;
        z *= -2;

        cartesian[Z_AXIS] = z;

        float x = motors[X_AXIS];
        float y = motors[Y_AXIS];

        float x_offset = tan(-beta) * (_focus_offset + _rotation_offset);

        x -= x_offset;
        x -= _center_x;
        y -= _center_y;

        float rx = cos(alpha) * x + sin(alpha) * y;
        float ry = -sin(alpha) * x + cos(alpha) * y;

        cartesian[X_AXIS] = rx + _center_x;
        cartesian[Y_AXIS] = round((ry + _center_y) * 100) / 100;

        float leng = tan(beta) * z;

        float dy = sin(alpha) * leng;
        float dx = cos(alpha) * leng;

        cartesian[A_AXIS] = atan2(dy, z) * to_degrees;
        cartesian[B_AXIS] = atan2(dx, z) * to_degrees;

        if (z == 0) {
            cartesian[A_AXIS] = 0;
            cartesian[B_AXIS] = 0;
        }
    }

    bool RotatingTableFiveAxis::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        if (state_is(State::Homing)) {
            return Cartesian::cartesian_to_motors(target, pl_data, position);
        }
        log_info("cartesian_to_motors");
        log_info("target: " << " X:" << target[X_AXIS] << " Y:" << target[Y_AXIS] << " Z:" << target[Z_AXIS] << " A:" << target[A_AXIS]
                            << " B:" << target[B_AXIS]);
        float dx, dy, dz, da, db;
        float motor_angles[5];

        float seq_target[5];
        memcpy(seq_target, position, 5 * sizeof(float));
        float feed_rate  = pl_data->feed_rate;
        bool  show_error = true;

        bool calc_ok = true;

        dx = target[X_AXIS] - position[X_AXIS];
        dy = target[Y_AXIS] - position[Y_AXIS];
        dz = target[Z_AXIS] - position[Z_AXIS];
        da = target[A_AXIS] - position[A_AXIS];
        db = target[B_AXIS] - position[B_AXIS];

        float segment_count = sqrt(dx * dx + dy * dy) / _kinematic_segment_len_mm;

        float motors[5] = { 0 };

        for (int i = 1; i <= segment_count; i++) {
            seq_target[X_AXIS] += dx / segment_count;
            seq_target[Y_AXIS] += dy / segment_count;
            seq_target[Z_AXIS] += dz / segment_count;
            seq_target[A_AXIS] += da / segment_count;
            seq_target[B_AXIS] += db / segment_count;

            if (!transform_cartesian_to_motors(motors, seq_target)) {
                return false;
            }

            mc_move_motors(motors, pl_data);
        }

        if (!transform_cartesian_to_motors(motors, target)) {
            return false;
        }



        memcpy(last_mpos, motors, 5 * sizeof(motors[0]));
        log_info("-> motors:" << " X:" << motors[X_AXIS] << " Y:" << motors[Y_AXIS] << " Z:" << motors[Z_AXIS] << " A:" << motors[A_AXIS]
                              << " B:" << motors[B_AXIS]);
        return mc_move_motors(motors, pl_data);
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<RotatingTableFiveAxis> registration("rotating_table_five_axis");
    }

    bool RotatingTableFiveAxis::kinematics_homing(AxisMask& axisMask) {
        this->init_position();
        return false;
    }
}  // namespace Kinematics