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
        // the point on the cutbed where the laser is supposed to enter
        Vector3f cut_start({ cartesian[X_AXIS], cartesian[Y_AXIS], cartesian[Z_AXIS] });

        // Direction vector of the laser in world space
        const float a_rad = cartesian[A_AXIS] * to_rad;
        const float b_rad = cartesian[B_AXIS] * to_rad;
        Vector3f    cut_dir({ tan(b_rad), tan(a_rad), -1.0f });
        cut_dir.normalize();

        // Solution 1: atan2 gives the primary angle for the table (A_AXIS)
        float table_angle_1;
        if (abs(cut_dir[0]) < 1e-6f && abs(cut_dir[1]) < 1e-6f) {
            // if the direction is nearly vertical, use the current table angle instead since we could use all table angles
            table_angle_1 = last_mpos[A_AXIS] * to_rad;
        } else {
            // atan2(x, y) corresponds to the rotation around Z to align the tilt
            table_angle_1 = atan2(cut_dir[0], cut_dir[1]);
        }

        // Solution 2: The alternative is 180 degrees away
        float table_angle_2 = table_angle_1 + M_PI;
        if (table_angle_2 > M_PI)
            table_angle_2 -= 2.0f * M_PI;

        auto calculate_motors = [&](float table_rad, float* m_out) {
            // Rotate coordinate system so table rotation is accounted for
            auto rotation_matrix = AngleAxisf(table_rad, Vector3f::UnitZ());
            // Rotate the position and direction back into the "machine" frame
            // because the table moves, not the gantry (relatively)
            Vector3f local_dir = rotation_matrix * cut_dir;

            // Laser head tilt (B_AXIS). Use atan2 to handle the vertical case safely.
            // local_dir[1] is the new Y offset, local_dir[2] is Z (-1)
            float head_tilt = atan2(local_dir[1], -local_dir[2]);

            // Calculate focus position
            Vector3f laser_pos = cut_start - cut_dir * _focus_offset;
            // rotate the laser position around center
            Vector3f local_pos = (rotation_matrix * (laser_pos - _center)) + _center;

            m_out[X_AXIS] = local_pos[0];
            m_out[Y_AXIS] = local_pos[1];
            // z_height is the height of laser at mpos 0. Higher mpos z: lower laser head in world position. Hence:
            m_out[Z_AXIS] = _z_height - local_pos[2];
            m_out[A_AXIS] = table_rad * to_deg;
            m_out[B_AXIS] = head_tilt * to_deg;
        };

        calculate_motors(table_angle_1, motors_left);
        calculate_motors(table_angle_2, motors_right);
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
        while (pos[A_AXIS] < 0)
            pos[A_AXIS] += 360.0f;
        while (left[A_AXIS] < 0)
            left[A_AXIS] += 360.0f;
        while (right[A_AXIS] < 0)
            right[A_AXIS] += 360.0f;

        float adl = angle_dist(pos[A_AXIS], left[A_AXIS]);
        float adr = angle_dist(pos[A_AXIS], right[A_AXIS]);
        if (abs(adl) < abs(adr) && check_bounds(left)) {
            left[A_AXIS] = pos[A_AXIS] + adl;
            memcpy(pos, left, 5 * sizeof(pos[0]));
            return true;
        }
        if (check_bounds(right)) {
            right[A_AXIS] = pos[A_AXIS] + adr;
            memcpy(pos, right, 5 * sizeof(pos[0]));
            return true;
        }
        return false;
    }

    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors, float* cartesian) {
        if (state_is(State::Homing)) {
            return Cartesian::transform_cartesian_to_motors(motors, cartesian);
        }
        auto* motors_left  = new float[5];
        auto* motors_right = new float[5];
        transform_cartesian_to_motors(motors_left, motors_right, cartesian);
        if (!shortest_move(last_mpos, motors_left, motors_right)) {
            return false;
        }
        delete[] motors_left;
        delete[] motors_right;
        memcpy(motors, last_mpos, 5 * sizeof(motors[0]));
        return true;
    }

    void RotatingTableFiveAxis::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        if (state_is(State::Homing)) {
            return Cartesian::motors_to_cartesian(cartesian, motors, n_axis);
        }
        // the current position of the head, in rotated world space
        Vector3f rotated_pos({ motors[X_AXIS], motors[Y_AXIS], _z_height - motors[Z_AXIS] });
        // rotation matrix to rotate back to real world space
        auto rotation_matrix = AngleAxisf(-motors[A_AXIS] * to_rad, Vector3f::UnitZ());

        // rotate to real positon, around table center
        auto world_laser_pos = (rotation_matrix * (rotated_pos - _center)) + _center;
        // direction vector of the laser in world space
        auto world_laser_dir = (rotation_matrix * Vector3f({ 0, motors[A_AXIS] * to_rad, -1.0f })).normalized();
        // cut entry in world space
        auto cut_start = world_laser_pos + world_laser_dir * _focus_offset;

        cartesian[X_AXIS] = cut_start[0];
        cartesian[Y_AXIS] = cut_start[1];
        cartesian[Z_AXIS] = cut_start[2];
        cartesian[A_AXIS] = world_laser_dir[1] * to_deg;
        cartesian[B_AXIS] = world_laser_dir[0] * to_deg;
    }

    bool RotatingTableFiveAxis::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        if (state_is(State::Homing)) {
            return Cartesian::cartesian_to_motors(target, pl_data, position);
        }

        float motors[5] = { 0 };

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