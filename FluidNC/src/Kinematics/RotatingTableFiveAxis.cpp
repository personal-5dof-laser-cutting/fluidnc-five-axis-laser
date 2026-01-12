//
// Created by Tom Haas on 15.10.25.
//

#include "RotatingTableFiveAxis.h"

#include "../Machine/MachineConfig.h"
#include <cmath>
#include <ArduinoEigen.h>
using namespace Eigen;
namespace Kinematics {

    static float       last_mpos[5] = { 0.0 };
    static const float to_rad       = M_PI / 180.0f;
    static const float to_deg       = 180.0f / M_PI;

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
        this->_center = Vector3f({ _center_x, _center_y, 0 });
    }

    void RotatingTableFiveAxis::init_position() {
        auto n_axis = Axes::_numberAxis;
        for (size_t axis = 0; axis < n_axis; axis++) {
            set_motor_steps(axis, 0);  // Set to zeros
        }
    }

    bool RotatingTableFiveAxis::check_bounds(float* motors) {
        if (abs(motors[B_AXIS]) > _max_angle) {
            log_info("Out of bounds");
            return false;
        }
        if (motors[Z_AXIS] < 0 || motors[Z_AXIS] > _max_z) {
            log_info("Out of bounds");
            return false;
        }
        if (motors[X_AXIS] < 0 || motors[X_AXIS] > _max_x) {
            log_info("Out of bounds");
            return false;
        }
        if (motors[Y_AXIS] < 0 || motors[Y_AXIS] > _max_y) {
            log_info("Out of bounds");
            return false;
        }

        return true;
    }

    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors_left, float* motors_right, float* cartesian) {
        memset(motors_right, 0, 5 * sizeof(float));
        memset(motors_left, 0, 5 * sizeof(float));

        constexpr float to_radians = M_PI / 180;
        constexpr float to_degrees = 180 / M_PI;

        // distance from rotation center of laser head to surface of work piece
        float height_above_surface = _z_height + _rotation_offset - cartesian[Z_AXIS];

        // stores direction and distance of laser head rotation center to surface
        Vector3f direction({
            tan(cartesian[B_AXIS] * to_radians) * height_above_surface,
            tan(cartesian[A_AXIS] * to_radians) * height_above_surface,
            -height_above_surface
        });


        float table_angle = atan2(direction[1], direction[0]);

        // when the cut isn't angled we don't need a specific table rotation
        // choosing the last table angle saves time
        if (abs(cartesian[A_AXIS]) < 1e-6 && abs(cartesian[B_AXIS]) < 1e-6) {
            table_angle = last_mpos[A_AXIS] * to_radians;
        }

        auto rotation_matrix = AngleAxisf(table_angle, -Vector3f::UnitZ());

        // rotating the direction vector translates A and B angles into a single angle
        Vector3f rotated_direction = rotation_matrix * direction;

        // direction has exact length to hit surface, x value stores exact offset
        float x_offset = rotated_direction[0];

        Vector3f position({cartesian[X_AXIS], cartesian[Y_AXIS], 0});
        Vector3f center_position({_center_x, _center_y, 0});

        Vector3f rotated_position = rotation_matrix * (position - center_position) + center_position;
        auto rotation_right = AngleAxisf(M_PI, -Vector3f::UnitZ());

        motors_left[X_AXIS] = rotated_position[0] - x_offset;
        motors_left[Y_AXIS] = rotated_position[1];

        // rotated_direction.norm() would move laser head rotation center directly onto the surface
        // subtracting _rotation_offset, we'd have the nozzle directly on the surface
        // also subtracting the _focus_offset puts the focus point on the surface
        motors_left[Z_AXIS] = (rotated_direction.norm() - (_focus_offset + _rotation_offset));
        motors_left[A_AXIS] = table_angle * to_degrees;
        motors_left[B_AXIS] = -atan2(rotated_direction[0], -rotated_direction[2]) * to_degrees;

        // rotating calculated motors positions by 180° to get second option
        Vector3f rotated_position_right = rotation_right * (rotated_position - center_position) + center_position;

        motors_right[X_AXIS] = rotated_position_right[0] + x_offset;
        motors_right[Y_AXIS] = rotated_position_right[1];

        motors_right[Z_AXIS] = motors_left[Z_AXIS];
        motors_right[A_AXIS] = (table_angle) * to_degrees + 180;
        if (motors_right[A_AXIS] >= 360)
            motors_right[A_AXIS] = motors_right[A_AXIS] - 360;
        motors_right[B_AXIS] = -motors_left[B_AXIS];

        return true;
    }

    float angle_dist(float pos, float dest) {
        pos        = fmodf(pos, 360.0f);
        dest       = fmodf(dest, 360.0f);
        float diff = dest - pos;
        if (diff > 180.0f)
            diff -= 360.0f;
        else if (diff < -180.0f)
            diff += 360.0f;

        return diff;
    }

    bool RotatingTableFiveAxis::shortest_move(float* pos, float* left, float* right) {
        // normalize to positive angles [0, 360]
        while (left[A_AXIS] < 0)
            left[A_AXIS] += 360.0f;
        while (right[A_AXIS] < 0)
            right[A_AXIS] += 360.0f;

        log_info("Input pos: X" << pos[X_AXIS] << " Y: " << pos[Y_AXIS] << " A: " << pos[A_AXIS] << " B:" << pos[B_AXIS]);
        log_info("Input left: X" << left[X_AXIS] << " Y: " << left[Y_AXIS] << " A: " << left[A_AXIS] << " B:" << left[B_AXIS] << " bounds: " << check_bounds(left));
        log_info("Input right: X" << right[X_AXIS] << " Y: " << right[Y_AXIS] << " A: " << right[A_AXIS] << " B:" << right[B_AXIS] << " bounds: " << check_bounds(right));

        float adl = angle_dist(pos[A_AXIS], left[A_AXIS]);
        float adr = angle_dist(pos[A_AXIS], right[A_AXIS]);

        // left more optimal or right not possible
        if (abs(adl) < abs(adr) || !check_bounds(right)) {
            if (check_bounds(left)) {
                // adding angle distance to turn in the right direction
                // from 355° to 5° should rotate 10° to '365°' and not -350° to 5°
                left[A_AXIS] = pos[A_AXIS] + adl;
                log_info("Took left with turn dist:" << adl)
                memcpy(pos, left, 5 * sizeof(pos[0]));
                return true;
            }
        }
        if (check_bounds(right)) {
            right[A_AXIS] = pos[A_AXIS] + adr;
            log_info("Took right with turn dist:" << adr)
            memcpy(pos, right, 5 * sizeof(pos[0]));
            return true;
        }
        log_info("All out of bounds... Cannot move to target.");
        return false;
    }

    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors, float* cartesian) {
        if (state_is(State::Homing)) {
            return Cartesian::transform_cartesian_to_motors(motors, cartesian);
        }

        // 2 possible motorpositions (180° rotated)
        auto* motors_left  = new float[5];
        auto* motors_right = new float[5];
        transform_cartesian_to_motors(motors_left, motors_right, cartesian);
        if (!shortest_move(last_mpos, motors_left, motors_right)) {
            return false;
        }
        delete[] motors_left;
        delete[] motors_right;
        log_info("Motor target: A:" << last_mpos[A_AXIS] << ", B: " << last_mpos[B_AXIS]);
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

        float motors[5] = { 0 };

        if (!transform_cartesian_to_motors(motors, target)) {
            return false;
        }
        // storing last motor positions for shortest move calculations
        memcpy(last_mpos, motors, 5 * sizeof(motors[0]));
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