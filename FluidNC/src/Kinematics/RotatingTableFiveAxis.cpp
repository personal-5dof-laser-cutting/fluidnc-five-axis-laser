//
// Created by Tom Haas on 15.10.25.
//

#include "RotatingTableFiveAxis.h"

#include "../Machine/MachineConfig.h"
constexpr float to_radians = M_PI / 180.0;
constexpr float to_degrees = 180.0 / M_PI;

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

        // rotating the direction vector translates A and B angles into a single angle
        Vector3f rotated_direction = direction.rotateZ(table_angle);

        // direction has exact length to hit surface, x value stores exact offset
        float x_offset = rotated_direction[0];

        Vector3f position({ cartesian[X_AXIS], cartesian[Y_AXIS], 0 });
        Vector3f center_position({ _center_x, _center_y, 0 });

        Vector3f rotated_position = (position).rotateZ(table_angle, center_position);

        // rotated_direction.norm() would move laser head rotation center directly onto the surface
        // subtracting _rotation_offset, we'd have the nozzle directly on the surface
        // also subtracting the _focus_offset puts the focus point on the surface
        motors_left[Z_AXIS] = (rotated_direction.norm() - (_focus_offset + _rotation_offset));
        motors_left[A_AXIS] = table_angle * to_degrees;
        motors_left[B_AXIS] = -atan2f(rotated_direction[0], -rotated_direction[2]) * to_degrees;

        float y_offset    = 0.5f * (motors_left[B_AXIS] / 45.0f);
        float x_off_at_45 = 0.5f * fabsf(motors_left[B_AXIS] / 45.0f);

        motors_left[X_AXIS] = rotated_position[0] - x_offset + x_off_at_45;
        motors_left[Y_AXIS] = rotated_position[1] + y_offset;

        // rotating calculated motors positions by 180° to get second option
        Vector3f rotated_position_right = position.rotateZ(M_PI + table_angle, center_position);

        motors_right[X_AXIS] = rotated_position_right[0] + x_offset + x_off_at_45;
        motors_right[Y_AXIS] = rotated_position_right[1] + y_offset;

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
        set_state(State::Alarm);
        return false;
    }

    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors, float* cartesian) {
        if (state_is(State::Homing)) {
            return Cartesian::transform_cartesian_to_motors(motors, cartesian);
        }

        float motors_left[5]  = {};
        float motors_right[5] = {};
        transform_cartesian_to_motors(motors_left, motors_right, cartesian);
        return shortest_move(motors, motors_left, motors_right);
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
        Vector3f direction = rotated_direction.rotateZ(-table_angle);

        // Recover A and B cartesian angles
        // direction = { tan(B)*h, tan(A)*h, -h }
        float height_above_surface = -direction[2];
        cartesian[B_AXIS]          = atan2f(direction[0], height_above_surface) * to_degrees;
        cartesian[A_AXIS]          = atan2f(direction[1], height_above_surface) * to_degrees;
        cartesian[Z_AXIS]          = _z_height + _rotation_offset - height_above_surface;

        // Recover X and Y
        Vector3f center_position({ _center_x, _center_y, 0 });
        Vector3f rotated_pos_no_offset({ motors[X_AXIS] + x_offset, motors[Y_AXIS], 0 });

        Vector3f original_pos = rotated_pos_no_offset.rotateZ(-table_angle, center_position);

        cartesian[X_AXIS] = original_pos[0];
        cartesian[Y_AXIS] = original_pos[1];
    }

    bool move_motors(float* target, plan_line_data_t* pl_data) {
        memcpy(last_mpos, target, 5 * sizeof(last_mpos[0]));
        //log_info("Moving: " << target[X_AXIS] << ", " << target[Y_AXIS] << ", " << target[Z_AXIS] << ", " << target[A_AXIS] << ", " << target[B_AXIS]);
        plan_line_data_t pl_copy = *pl_data;
        return mc_move_motors(target, &pl_copy);
    }

    bool RotatingTableFiveAxis::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        if (state_is(State::Homing)) {
            // home in cartesian
            is_homing_pos = true;
            return Cartesian::cartesian_to_motors(target, pl_data, position);
        }
        // initialize a motors array at last_mpos
        float motors[5] = {};
        memcpy(motors, last_mpos, 5 * sizeof(motors[0]));

        // for the first call after homing, move to a sane position before continuing
        if (is_homing_pos) {
            position[X_AXIS] = _center_x;
            position[Y_AXIS] = _center_y;
            position[A_AXIS] = position[B_AXIS] = 0;
            position[Z_AXIS]                    = target[Z_AXIS];
            is_homing_pos                       = false;
            if (!transform_cartesian_to_motors(motors, position)) {
                return false;
            }
            move_motors(motors, pl_data);
        }

        // segment count calculated from xy-move distance, and angles, so that resolution is at least 1-per-degree and _kinematic_segment_len_mm for xy.
        float segment_count =
            sqrtf(powf(target[X_AXIS] - position[X_AXIS], 2) + powf(target[Y_AXIS] - position[Y_AXIS], 2)) / _kinematic_segment_len_mm;

        segment_count = fmaxf(segment_count, fmaxf(fabsf(target[B_AXIS] - position[B_AXIS]), fabsf(target[A_AXIS] - position[A_AXIS])));
        //log_info("Cartesian goal: " << target[X_AXIS] << ", " <<  target[Y_AXIS] << ", " << target[Z_AXIS] << ", " << target[A_AXIS] << ", " << target[B_AXIS]);

        if (segment_count > 1) {
            // array storing distance of each segment per axis
            float move[5] = {};

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
                //log_info("Cartesian segment: " << segment_cart[X_AXIS] << ", " <<  segment_cart[Y_AXIS] << ", " << segment_cart[Z_AXIS] << ", " << segment_cart[A_AXIS] << ", " << segment_cart[B_AXIS]);
                if (!transform_cartesian_to_motors(motors, segment_cart)) {
                    return false;
                }

                // segments shouldn't input more than 1 degree changes per axis
                // output angle change shouldn't exceed 3 degrees
                // (changes from 60/60 to 61/61 would be 2,975 which is already an 83° angle -> not possible)
                // more than 3 degrees difference can only happen if left/right change
                // -> causes b to change sign from one segment to the next
                // -> z stays same -> nozzle moves into table
                if (fabsf(motors[B_AXIS] - last_mpos[B_AXIS]) > 3) {
                    log_info("Using jonas cool function");
                    // movement wouldn't be intended by user -> travel move?
                    // travel move -> segment would essentially be skipped -> cut move?
                    // last_mpos = right -> travel to last_mpos_left
                    // <->

                    // turn on travel move
                    SpindleState st  = pl_data->spindle;
                    auto         sm  = pl_data->motion;
                    pl_data->spindle = SpindleState::Disable;
                    pl_data->motion.rapidMotion = true;

                    float cart_before[5] = {};
                    motors_to_cartesian(cart_before, last_mpos, 5);

                    float ctrTarget[5] = {_center_x, _center_y, target[Z_AXIS], 0, 0};

                    // move to center
                    cartesian_to_motors(ctrTarget, pl_data, cart_before);

                    // move there (on the other side of  the table)
                    move_motors(motors, pl_data);

                    // move back to our cartesian starting position, but this time this will be on the other side
                    cartesian_to_motors(cart_before, pl_data, segment_cart);

                    // restore laser state
                    pl_data->spindle = st;
                    pl_data->motion  = sm;
                }

                // move current segment
                move_motors(motors, pl_data);
            }
        }

        // kinematics for final move (to target directly to avoid floating point error)
        if (!transform_cartesian_to_motors(motors, target)) {
            return false;
        }
        return move_motors(motors, pl_data);
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