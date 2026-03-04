//
// Created by Tom Haas on 15.10.25.
// Kinematics of a 5 axis machine (gantry with a rotating table and a tilting head).
// X, Y are the gantry axes
// Z is the vertical axis of the gantry
// ROLL_AXIS is the tilt axis of the head
// TABLE_AXIS is the rotation axis of the table
//

#ifndef INC_5AXISFLUIDNC_ROTATINGTABLEFIVEAXISNEW_H
#define INC_5AXISFLUIDNC_ROTATINGTABLEFIVEAXISNEW_H

#include "Cartesian.h"
#include "Kinematics.h"

// Axis aliases for clarity
#ifndef TABLE_AXIS
#    define TABLE_AXIS B_AXIS
#endif
#ifndef ROLL_AXIS
#    define ROLL_AXIS A_AXIS
#endif
#include <cmath>
namespace Kinematics {

    struct FiveTuple {
        float x, y, z, a, b, c;  //c is here for memory safety
        explicit FiveTuple() : x(0), y(0), z(0), a(0), b(0), c(0) {}
        explicit FiveTuple(const float* arr) : x(arr[0]), y(arr[1]), z(arr[2]), a(arr[3]), b(arr[4]), c(0) {}
        explicit FiveTuple(const float& x, const float& y, const float& z, const float& a, const float& b) :
            x(x), y(y), z(z), a(a), b(b), c(0) {}

        float& operator[](size_t index) {
            switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                case 3:
                    return a;
                case 4:
                    return b;
                default:
                    throw std::out_of_range("Index out of range");
            }
        }
        const float& operator[](size_t index) const {
            switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                case 3:
                    return a;
                case 4:
                    return b;
                default:
                    throw std::out_of_range("Index out of range");
            }
        }
        float* asFloatArray() { return &x; }

        FiveTuple operator+(const FiveTuple& rhs) const { return FiveTuple(x + rhs.x, y + rhs.y, z + rhs.z, a + rhs.a, b + rhs.b); }

        FiveTuple operator-(const FiveTuple& rhs) const { return FiveTuple(x - rhs.x, y - rhs.y, z - rhs.z, a - rhs.a, b - rhs.b); }

        FiveTuple operator*(const float scalar) const { return FiveTuple(x * scalar, y * scalar, z * scalar, a * scalar, b * scalar); }

        FiveTuple operator/(const float scalar) const { return FiveTuple(x / scalar, y / scalar, z / scalar, a / scalar, b / scalar); }

        FiveTuple& operator+=(const FiveTuple& rhs) {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            a += rhs.a;
            b += rhs.b;
            return *this;
        }

        FiveTuple& operator-=(const FiveTuple& rhs) {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            a -= rhs.a;
            b -= rhs.b;
            return *this;
        }

        FiveTuple& operator*=(const float scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            a *= scalar;
            b *= scalar;
            return *this;
        }

        FiveTuple& operator/=(const float scalar) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            a /= scalar;
            b /= scalar;
            return *this;
        }
    };

    // Type aliases
    using Mpos = FiveTuple;  // Actual raw motor positions. A is table angle, B is miter angle
    using Cpos = FiveTuple;  // Cartesian position on the table

    struct Vector3f {
        float    x, y, z;
        Vector3f operator+(const Vector3f& rhs) const { return { x + rhs.x, y + rhs.y, z + rhs.z }; }

        Vector3f operator-(const Vector3f& rhs) const { return { x - rhs.x, y - rhs.y, z - rhs.z }; }

        Vector3f& operator+=(const Vector3f& rhs) {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        Vector3f& operator-=(const Vector3f& rhs) {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        Vector3f operator-() const {  // unary minus
            return { -x, -y, -z };
        }

        float& operator[](const size_t index) {
            switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                default:
                    throw std::out_of_range("Index out of range");
            }
        }

        [[nodiscard]] Vector3f rotateZ(const float angle, Vector3f pivot) const { return (*this - pivot).rotateZ(angle) + pivot; }
        [[nodiscard]] Vector3f rotateZ(const float angle) const {
            const float angle_ = -angle;  // rotate clockwise
            const float s      = sinf(angle_);
            const float c      = cosf(angle_);
            return { x * c - y * s, x * s + y * c, z };
        }
        [[nodiscard]] float norm() const { return sqrtf(x * x + y * y + z * z); }
    };

    class RotatingTableFiveAxis : public Cartesian {
    public:
        explicit RotatingTableFiveAxis(const char* name) : Cartesian(name) {}

        // FiveTuple struct - actual raw motor positions or cartesian positions

        // Configurable parameters
        void group(Configuration::HandlerBase& handler) override;
        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void init() override;
        void init_position() override;
        void motors_to_cartesian(float* cartesian, float* motors, int n_axis) override;
        bool transform_cartesian_to_motors(float* motors, float* cartesian) override;
        bool kinematics_homing(AxisMask& axisMask) override;

    private:  // Configuration parameters
        float _max_z                    = 0.0f;
        float _focus_offset             = 0.0f;
        float _max_angle                = 0.0f;
        float _kinematic_segment_len_mm = 1.0;
        float _center_x                 = 0;
        float _center_y                 = 0;
        float _rotation_offset          = 0;
        float _z_height                 = 0;
        float _max_x                    = 0.0f;
        float _max_y                    = 0.0f;
        float _angle_threshold          = 0.0f;
        float _angle_xoff_x4            = 0.0f;
        float _angle_xoff_x3            = 0.0f;
        float _angle_xoff_x2            = 0.0f;
        float _angle_xoff_x1            = 0.0f;
        float _angle_yoff_x4            = 0.0f;
        float _angle_yoff_x3            = 0.0f;
        float _angle_yoff_x2            = 0.0f;
        float _angle_yoff_x1            = 0.0f;

        // state parameters
        Vector3f _center = { 0, 0, 0 };
        Mpos     _last_mpos;
        bool     _homing = true;

        // Returns the distance between two angles, accounting for wraparound
        static float angle_distance(float a1, float a2);

        // Returns false if the motor position is out of bounds for the machine
        [[nodiscard]] bool is_in_bounds(const Mpos& mpos) const;

        // Clamps the motor position to bounds if needed
        void clamp_bounds(Mpos& mpos) const;

        float estimate_x_offset(const Mpos& mpos);

        float estimate_y_offset(const Mpos& mpos);

        // The machine has up to two different valid configurations for each cartesian position.
        // This should perform kinematics and return them. They may be equal.
        void cartesian_to_possible_motors(const Cpos& cartesian, Mpos& m1, Mpos& m2);

        // Move from cartesian to cartesian without segmenting (unless for jumps)
        void direct_move(const Cpos& start, const Cpos& end, plan_line_data_t* pl_data);

        // Move from start position to goal, without bumping the laser into the table. No cutting.
        void safe_fast_move_motors(const Mpos& start, const Mpos& goal, plan_line_data_t* pl_data);

        // Segmented move from start to end
        void segmented_move(const Cpos& start, const Cpos& end, plan_line_data_t* pl_data);

        // Move to motor position and update last_mpos
        void move_motors(const Mpos& pos, plan_line_data_t* pl_data);

        void cartesian_to_closest_furthest(const Cpos& cartesian, Mpos& closest, Mpos& furthest);
    };

}  // namespace Kinematics

#endif  // INC_5AXISFLUIDNC_ROTATINGTABLEFIVEAXISNEW_H
