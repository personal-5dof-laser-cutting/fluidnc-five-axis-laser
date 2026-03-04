
#include "RotatingTableFiveAxis.h"

#include "../Machine/MachineConfig.h"
constexpr float to_radians = M_PI / 180.0;
constexpr float to_degrees = 180.0 / M_PI;

namespace Kinematics {
    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<RotatingTableFiveAxis> registration("rotating_table_five_axis");
    }

    bool RotatingTableFiveAxis::kinematics_homing(AxisMask& axisMask) {
        this->init_position();
        return false;
    }

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
        handler.item("angle_xoff_x4", _angle_xoff_x4);
        handler.item("angle_xoff_x3", _angle_xoff_x3);
        handler.item("angle_xoff_x2", _angle_xoff_x2);
        handler.item("angle_xoff_x1", _angle_xoff_x1);

        handler.item("angle_yoff_x4", _angle_yoff_x4);
        handler.item("angle_yoff_x3", _angle_yoff_x3);
        handler.item("angle_yoff_x2", _angle_yoff_x2);
        handler.item("angle_yoff_x1", _angle_yoff_x1);
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

    bool RotatingTableFiveAxis::transform_cartesian_to_motors(float* motors, float* cartesian) {
        if (state_is(State::Homing)) {
            return Cartesian::transform_cartesian_to_motors(motors, cartesian);
        }

        Mpos closest, furthest;
        cartesian_to_closest_furthest(Cpos(cartesian), closest, furthest);
        Mpos& selected = closest;
        if (!is_in_bounds(closest)) {
            if (is_in_bounds(furthest)) {
                selected = furthest;
            } else {
                log_error("Kinematic calculation error. Target unreachable (" << cartesian[0] << "," << cartesian[1] << "," << cartesian[2]
                                                                              << ")");
                return false;
            }
        }
        motors[X_AXIS] = selected[A_AXIS];
        motors[Y_AXIS] = selected[B_AXIS];
        motors[Z_AXIS] = selected[Z_AXIS];
        motors[A_AXIS] = selected[A_AXIS];
        motors[B_AXIS] = selected[B_AXIS];
        return true;
    }
    bool RotatingTableFiveAxis::cartesian_to_motors(float* _target, plan_line_data_t* pl_data, float* _position) {
        if (state_is(State::Homing)) {
            // home in cartesian
            _homing = true;
            return Cartesian::cartesian_to_motors(_target, pl_data, _position);
        }

        auto position = Cpos(_position);
        auto target   = Cpos(_target);
        // for the first call after homing, move to a sane position before continuing
        if (_homing) {
            _homing  = false;
            position = Cpos(_center_x, _center_y, target[Z_AXIS], 0, 0);
            move_motors(position, pl_data);
        }
        segmented_move(position, target, pl_data);
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
        Vector3f    rotated_direction = { length * sinf(b_angle_rad), 0, -length * cosf(b_angle_rad) };
        const float x_offset          = rotated_direction[0];

        // Undo the table rotation on the direction vector to find original A and B angles
        Vector3f direction = rotated_direction.rotateZ(-table_angle);

        // Recover A and B cartesian angles
        // direction = { tan(B)*h, tan(A)*h, -h }
        const float height_above_surface = -direction[2];
        cartesian[B_AXIS]                = atan2f(direction[0], height_above_surface) * to_degrees;
        cartesian[A_AXIS]                = atan2f(direction[1], height_above_surface) * to_degrees;
        cartesian[Z_AXIS]                = _z_height + _rotation_offset - height_above_surface;

        // Recover X and Y
        const Vector3f center_position({ _center_x, _center_y, 0 });
        const Vector3f rotated_pos_no_offset({ motors[X_AXIS] + x_offset, motors[Y_AXIS], 0 });

        Vector3f original_pos = rotated_pos_no_offset.rotateZ(-table_angle, center_position);

        cartesian[X_AXIS] = original_pos[0];
        cartesian[Y_AXIS] = original_pos[1];

        cartesian[X_AXIS] += this->estimate_x_offset(Cpos(cartesian));
        cartesian[Y_AXIS] += this->estimate_y_offset(Cpos(cartesian));
    }

    float RotatingTableFiveAxis::angle_distance(const float a1, const float a2) {
        const float diff = fmodf((a2 - a1 + 180.0f), 360.0f) - 180.0f;
        return diff < -180.0f ? diff + 360.0f : diff;
    }

    bool RotatingTableFiveAxis::is_in_bounds(const Mpos& mpos) const {
        const bool out_of_bounds = fabsf(mpos[B_AXIS]) > _max_angle || mpos[X_AXIS] < 0 || mpos[X_AXIS] > _max_x || mpos[Y_AXIS] < 0 ||
                                   mpos[Y_AXIS] > _max_y;
        return !out_of_bounds;
    }

    void RotatingTableFiveAxis::clamp_bounds(Mpos& mpos) const {
        if (mpos[Z_AXIS] < 0)
            mpos[Z_AXIS] = 0;
        if (mpos[Z_AXIS] > _max_z)
            mpos[Z_AXIS] = _max_z;
    }

    float RotatingTableFiveAxis::estimate_x_offset(const Mpos& mpos) {
        return _angle_xoff_x4 * powf(mpos[B_AXIS], 4.0) + _angle_xoff_x3 * powf(mpos[B_AXIS], 3.0) +
               _angle_xoff_x2 * powf(mpos[B_AXIS], 2.0) + _angle_xoff_x1 * mpos[B_AXIS];
    }

    float RotatingTableFiveAxis::estimate_y_offset(const Mpos& mpos) {
        return _angle_yoff_x4 * powf(mpos[B_AXIS], 4.0) + _angle_yoff_x3 * powf(mpos[B_AXIS], 3.0) +
               _angle_yoff_x2 * powf(mpos[B_AXIS], 2.0) + _angle_yoff_x1 * mpos[B_AXIS];
    }

    void RotatingTableFiveAxis::cartesian_to_possible_motors(const Cpos& cartesian, Mpos& m1, Mpos& m2) {
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
            table_angle = _last_mpos[A_AXIS] * to_radians;
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
        m1[Z_AXIS] = (rotated_direction.norm() - (_focus_offset + _rotation_offset));
        m1[A_AXIS] = table_angle * to_degrees;
        m1[B_AXIS] = -atan2f(rotated_direction[0], -rotated_direction[2]) * to_degrees;

        m1[X_AXIS] = rotated_position[0] - x_offset;
        m1[Y_AXIS] = rotated_position[1];

        // rotating calculated motors positions by 180° to get second option
        Vector3f rotated_position_right = position.rotateZ(M_PI + table_angle, center_position);

        m2[X_AXIS] = rotated_position_right[0] + x_offset;
        m2[Y_AXIS] = rotated_position_right[1];

        m2[Z_AXIS] = m1[Z_AXIS];
        m2[A_AXIS] = (table_angle)*to_degrees + 180;
        if (m2[A_AXIS] >= 360)
            m2[A_AXIS] = m2[A_AXIS] - 360;
        m2[B_AXIS] = -m1[B_AXIS];

        m1[X_AXIS] -= this->estimate_x_offset(m1);
        m1[Y_AXIS] -= this->estimate_y_offset(m1);
        m2[X_AXIS] -= this->estimate_x_offset(m2);
        m2[Y_AXIS] -= this->estimate_y_offset(m2);
    }

    void RotatingTableFiveAxis::cartesian_to_closest_furthest(const Cpos& cartesian, Mpos& closest, Mpos& furthest) {
        Mpos m1, m2;
        cartesian_to_possible_motors(cartesian, m1, m2);

        clamp_bounds(m1);
        clamp_bounds(m2);

        const float d1 = angle_distance(_last_mpos.a, m1.a);
        const float d2 = angle_distance(_last_mpos.a, m2.a);

        // Convert table motor positions to cumulative table motor positions
        m1 = Mpos { m1.x, m1.y, m1.z, _last_mpos.a + d1, m1.b };
        m2 = Mpos { m2.x, m2.y, m2.z, _last_mpos.a + d2, m2.b };

        // Pick the one that's closer to the current position
        if (fabsf(d1) < fabsf(d2)) {
            closest  = m1;
            furthest = m2;
        } else {
            closest  = m2;
            furthest = m1;
        }
    }

    void RotatingTableFiveAxis::direct_move(const Cpos& start, const Cpos& end, plan_line_data_t* pl_data) {
        Mpos closest, furthest;
        cartesian_to_closest_furthest(end, closest, furthest);

        if (is_in_bounds(closest)) {
            move_motors(closest, pl_data);
        } else {
            if (is_in_bounds(furthest)) {
                // If the closest is out of bounds, we have to first travel to the other table side
                safe_fast_move_motors(_last_mpos, furthest, pl_data);
                // We are now at the cut end point. From here, perform the cut if needed
                if (pl_data->spindle != SpindleState::Disable) {
                    auto st                     = pl_data->spindle;
                    auto sm                     = pl_data->motion;
                    pl_data->spindle            = SpindleState::Disable;
                    pl_data->motion.rapidMotion = true;
                    direct_move(end, start, pl_data);  // move to start point of cut without cutting

                    pl_data->spindle = st;
                    pl_data->motion  = sm;

                    direct_move(start, end, pl_data);  // perform the cut
                }
            } else {
                // Put machine in alarm state. Both positions not reachable.
                log_info("Out Of Bounds: Cannot move to target.");
                set_state(State::Alarm);
            }
        }
    }

    void RotatingTableFiveAxis::safe_fast_move_motors(const Mpos& start, const Mpos& goal, plan_line_data_t* pl_data) {
        // TODO: make better
        auto st                     = pl_data->spindle;
        auto sm                     = pl_data->motion;
        pl_data->spindle            = SpindleState::Disable;
        pl_data->motion.rapidMotion = true;

        auto start_higher    = start;
        auto goal_higher     = goal;
        start_higher[Z_AXIS] = 0;
        goal_higher[Z_AXIS]  = 0;

        move_motors(start_higher, pl_data);
        move_motors(goal_higher, pl_data);
        move_motors(goal, pl_data);

        pl_data->spindle = st;
        pl_data->motion  = sm;
    }

    void RotatingTableFiveAxis::segmented_move(const Cpos& start, const Cpos& end, plan_line_data_t* pl_data) {
        const float height_above_surface = _z_height + _rotation_offset - start[Z_AXIS];

        const float tan_a   = fabsf(tanf(end[A_AXIS] * to_radians) - tanf(start[A_AXIS] * to_radians)) * height_above_surface;
        const float tan_b   = fabsf(tanf(end[B_AXIS] * to_radians) - tanf(start[B_AXIS] * to_radians)) * height_above_surface;
        const float max_tan = fmaxf(tan_a, tan_b);

        const int segment_count = std::max(1, static_cast<int>(std::ceil(max_tan / 2.0f)));

        const float inv  = 1.0f / static_cast<float>(segment_count);
        const auto  diff = end - start;

        Cpos last = start;

        for (int i = 1; i <= segment_count; ++i) {
            const float p            = static_cast<float>(i) * inv;
            const auto  segment_cart = start + diff * p;

            direct_move(last, segment_cart, pl_data);
            last = segment_cart;
        }
    }

    void RotatingTableFiveAxis::move_motors(const Mpos& pos, plan_line_data_t* pl_data) {
        auto copy = pos;
        if (mc_move_motors(copy.asFloatArray(), pl_data)) {
            _last_mpos = pos;
        }
    }
}