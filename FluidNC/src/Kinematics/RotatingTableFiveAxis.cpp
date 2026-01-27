//
// Created by Tom Haas on 15.10.25.
//

#include "RotatingTableFiveAxis.h"

#include "../Machine/MachineConfig.h"
#include <cmath>
#include <ArduinoEigen.h>

constexpr float to_radians = M_PI / 180;
constexpr float to_degrees = 180 / M_PI;

using namespace Eigen;
namespace Kinematics {

    static float last_mpos[5]  = {};
    static bool  is_homing_pos = false;

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
        handler.item("angle_threshold", _angle_threshold);
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

    bool RotatingTableFiveAxis::check_and_clamp_bounds(float* motors) const {
        // z-clamping
        if (motors[Z_AXIS] < 0)
            motors[Z_AXIS] = 0;
        if (motors[Z_AXIS] > _max_z)
            motors[Z_AXIS] = _max_z;
        const bool out_of_bounds = fabsf(motors[B_AXIS]) > _max_angle || motors[X_AXIS] < 0 || motors[X_AXIS] > _max_x ||
                                   motors[Y_AXIS] < 0 || motors[Y_AXIS] > _max_y;
        return !out_of_bounds;
    }

    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors_left, float* motors_right, const float* cartesian) const {
        // distance from rotation center of laser head to surface of work piece
        float height_above_surface = _z_height + _rotation_offset - cartesian[Z_AXIS];

        // stores direction and distance of laser head rotation center to surface
        Vector3f direction({ tanf(cartesian[B_AXIS] * to_radians) * height_above_surface,
                             tanf(cartesian[A_AXIS] * to_radians) * height_above_surface,
                             -height_above_surface });

        float table_angle = atan2f(direction[1], direction[0]);

        // when the cut isn't angled we don't need a specific table rotation
        // choosing the last table angle saves time
        if (fabsf(cartesian[A_AXIS]) < 1e-6 && fabsf(cartesian[B_AXIS]) < 1e-6) {
            table_angle = last_mpos[A_AXIS] * to_radians;
        }

        auto rotation_matrix = AngleAxisf(table_angle, -Vector3f::UnitZ());

        // rotating the direction vector translates A and B angles into a single angle
        Vector3f rotated_direction = rotation_matrix * direction;

        // direction has exact length to hit surface, x value stores exact offset
        float x_offset = rotated_direction[0];

        Vector3f position({ cartesian[X_AXIS], cartesian[Y_AXIS], 0 });
        Vector3f center_position({ _center_x, _center_y, 0 });

        Vector3f rotated_position = rotation_matrix * (position - center_position) + center_position;
        auto     rotation_right   = AngleAxisf(M_PI, -Vector3f::UnitZ());

        motors_left[X_AXIS] = rotated_position[0] - x_offset;
        motors_left[Y_AXIS] = rotated_position[1];

        // rotated_direction.norm() would move laser head rotation center directly onto the surface
        // subtracting _rotation_offset, we'd have the nozzle directly on the surface
        // also subtracting the _focus_offset puts the focus point on the surface
        motors_left[Z_AXIS] = (rotated_direction.norm() - (_focus_offset + _rotation_offset));
        motors_left[A_AXIS] = table_angle * to_degrees;
        motors_left[B_AXIS] = -atan2f(rotated_direction[0], -rotated_direction[2]) * to_degrees;

        // rotating calculated motors positions by 180° to get second option
        Vector3f rotated_position_right = rotation_right * (rotated_position - center_position) + center_position;

        motors_right[X_AXIS] = rotated_position_right[0] + x_offset;
        motors_right[Y_AXIS] = rotated_position_right[1];

        motors_right[Z_AXIS] = motors_left[Z_AXIS];
        motors_right[A_AXIS] = (table_angle)*to_degrees + 180;
        if (motors_right[A_AXIS] >= 360)
            motors_right[A_AXIS] = motors_right[A_AXIS] - 360;
        motors_right[B_AXIS] = -motors_left[B_AXIS];

        return true;
    }

    float angle_dist(float pos, float dest) {
        pos        = fmodf(fmodf(pos, 360.0f) + 360.0f, 360.0f);
        dest       = fmodf(fmodf(dest, 360.0f) + 360.0f, 360.0f);
        float diff = dest - pos;
        if (diff > 180.0f)
            diff -= 360.0f;
        else if (diff <= -180.0f)
            diff += 360.0f;
        return diff;
    }

    bool RotatingTableFiveAxis::shortest_move(float* pos, float* left, float* right) {
        float adl = angle_dist(pos[A_AXIS], left[A_AXIS]);
        float adr = angle_dist(pos[A_AXIS], right[A_AXIS]);
        // left more optimal or right not possible
        if (fabsf(adl) < fabsf(adr) || !check_and_clamp_bounds(right)) {
            if (check_and_clamp_bounds(left)) {
                // adding angle distance to turn in the right direction
                // from 355° to 5° should rotate 10° to '365°' and not -350° to 5°
                left[A_AXIS] = pos[A_AXIS] + adl;
                memcpy(pos, left, 5 * sizeof(pos[0]));
                return true;
            }
        }
        if (check_and_clamp_bounds(right)) {
            right[A_AXIS] = pos[A_AXIS] + adr;
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

        float motors_left[5]  = {};
        float motors_right[5] = {};
        transform_cartesian_to_motors(motors_left, motors_right, cartesian);
        if (!shortest_move(motors, motors_left, motors_right)) {
            return false;
        }
        return true;
    }

    void RotatingTableFiveAxis::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        if (state_is(State::Homing)) {
            return Cartesian::motors_to_cartesian(cartesian, motors, n_axis);
        }
        float table_angle = motors[A_AXIS] * to_radians;
        float b_angle_rad = -motors[B_AXIS] * to_radians;

        // The length of the direction vector was: motors[Z] + _focus_offset + _rotation_offset
        float length = motors[Z_AXIS] + _focus_offset + _rotation_offset;

        // Recover the rotated direction vector
        // In forward: motors[B] = -atan2(rotated_direction[0], -rotated_direction[2])
        // And length = sqrt(x^2 + y^2 + z^2). Since it's a "rotated" vector in a 2D tilt plane:
        Vector3f rotated_direction;
        rotated_direction[0] = length * sinf(b_angle_rad);
        rotated_direction[1] = 0;  // The rotation matrix * direction collapsed tilt into XZ plane
        rotated_direction[2] = -length * cosf(b_angle_rad);

        float x_offset = rotated_direction[0];

        // Undo the table rotation on the direction vector to find original A and B angles
        auto     inv_rotation_matrix = AngleAxisf(table_angle, Vector3f::UnitZ());
        Vector3f direction           = inv_rotation_matrix * rotated_direction;

        // Recover A and B cartesian angles
        // direction = { tan(B)*h, tan(A)*h, -h }
        float height_above_surface = -direction[2];
        cartesian[B_AXIS]          = atan2f(direction[0], height_above_surface) * to_degrees;
        cartesian[A_AXIS]          = atan2f(direction[1], height_above_surface) * to_degrees;
        cartesian[Z_AXIS]          = _z_height + _rotation_offset - height_above_surface;

        // Recover X and Y
        Vector3f center_position({ _center_x, _center_y, 0 });
        Vector3f rotated_pos_no_offset({ motors[X_AXIS] + x_offset, motors[Y_AXIS], 0 });

        Vector3f original_pos = inv_rotation_matrix * (rotated_pos_no_offset - center_position) + center_position;

        cartesian[X_AXIS] = original_pos[0];
        cartesian[Y_AXIS] = original_pos[1];
    }

    bool RotatingTableFiveAxis::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        if (state_is(State::Homing)) {
            // home in cartesian
            is_homing_pos = true;
            return Cartesian::cartesian_to_motors(target, pl_data, position);
        }
        // for the first call after homing, convert position to cartesian space. It will always be in cartesian space for later calls
        if (is_homing_pos) {
            motors_to_cartesian(position, position, 5);
            is_homing_pos = false;
        }

        // initialize a motors array at last_mpos
        float motors[5] = {};
        memcpy(motors, last_mpos, 5 * sizeof(motors[0]));

        // segment count calculated from xy-move distance, and angles, so that resolution is at least 1-per-degree and _kinematic_segment_len_mm for xy.
        float segment_count =
            sqrtf(powf(target[X_AXIS] - position[X_AXIS], 2) + powf(target[Y_AXIS] - position[Y_AXIS], 2)) / _kinematic_segment_len_mm;

        segment_count = fmaxf(segment_count, fmaxf(fabsf(target[B_AXIS] - position[B_AXIS]), fabsf(target[A_AXIS] - position[A_AXIS])));
        if (segment_count > 1) {
            // array storing distance of each segment per axis
            float move[] = {};

            // array we will actually move to
            float segment_cart[5] = {};

            move[X_AXIS] = target[X_AXIS] - position[X_AXIS];
            move[Y_AXIS] = target[Y_AXIS] - position[Y_AXIS];
            move[Z_AXIS] = target[Z_AXIS] - position[Z_AXIS];
            move[A_AXIS] = target[A_AXIS] - position[A_AXIS];
            move[B_AXIS] = target[B_AXIS] - position[B_AXIS];

            for (float i = 1; i < segment_count; i++) {
                // adding segment length to current position
                const float p        = i / segment_count;
                segment_cart[X_AXIS] = position[X_AXIS] + move[X_AXIS] * p;
                segment_cart[Y_AXIS] = position[Y_AXIS] + move[Y_AXIS] * p;
                segment_cart[Z_AXIS] = position[Z_AXIS] + move[Z_AXIS] * p;
                segment_cart[A_AXIS] = position[A_AXIS] + move[A_AXIS] * p;
                segment_cart[B_AXIS] = position[B_AXIS] + move[B_AXIS] * p;
                if (!transform_cartesian_to_motors(motors, segment_cart)) {
                    return false;
                }
                // move current segment
                memcpy(last_mpos, motors, 5 * sizeof(motors[0]));
                mc_move_motors(motors, pl_data);
            }
        }

        // kinematics for final move (to target directly to avoid floating point error)
        if (!transform_cartesian_to_motors(motors, target)) {
            return false;
        }
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