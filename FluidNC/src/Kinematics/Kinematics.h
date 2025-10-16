#pragma once

// Kinematics interface.
#include "src/Configuration/Configurable.h"
#include "src/Configuration/GenericFactory.h"
#include "src/MotionControl.h"
#include "src/Planner.h"
#include "src/Types.h"
#include "src/Machine/Homing.h"

/*
How to add a new kinematic system (overview)

You can add your own kinematics by adding two files to the Kinematics folder,
for example MyKinematics.h and MyKinematics.cpp. Use ParallelDelta as a
reference implementation (see ParallelDelta.h/.cpp) and follow these steps:

1) Subclass Kinematics::KinematicSystem (or an appropriate helper like
   Kinematics::Cartesian if your machine is mostly Cartesian).

2) Implement the core virtual methods:
   - init()                               : Read configuration, validate, set up state.
   - init_position()                      : Establish machine position at startup.
   - cartesian_to_motors(...)             : Break a Cartesian move into one or more
                                            motor-space moves and send them to the planner.
   - motors_to_cartesian(...)             : Forward-kinematics: convert motor positions
                                            (e.g., angles or belt lengths) to Cartesian XYZ.
   - transform_cartesian_to_motors(...)   : Inverse-kinematics: compute motor-space target
                                            from a Cartesian target (no planning output).
   Optionally override:
   - constrain_jog(...), invalid_line(...), invalid_arc(...), canHome(...),
     kinematics_homing(...), releaseMotors(...), limitReached(...)

3) Register your implementation with the factory in your .cpp file, for example:

   namespace {
       KinematicsFactory::InstanceBuilder<MyKinematics> registration("my_kinematics_name");
   }

   The string must match the name used in the configuration file.

4) Expose any configurable parameters by overriding group(handler) to declare
   items, and optionally afterParse()/validate() for post-processing.

See ParallelDelta for concrete examples of segmentation of moves, feed-rate
conversion between Cartesian distance and motor-angle distance, soft-limit
checking, homing strategy, and factory registration.
*/

namespace Kinematics {
    class KinematicSystem;

    // Kinematics: Thin wrapper that owns a KinematicSystem implementation selected
    // by configuration and exposes a uniform interface to the rest of the motion
    // system. It also participates in the configuration lifecycle.
    class Kinematics : public Configuration::Configurable {
    public:
        Kinematics() {}
        ~Kinematics();

        // Configuration system helpers:
        // Declare configurable items for selecting/constructing the concrete
        // KinematicSystem and its parameters.
        void group(Configuration::HandlerBase& handler) override;

        // Called after configuration is parsed; typically used to finalize
        // construction of the concrete KinematicSystem via the factory.
        void afterParse() override;

        // Initialize the kinematic system (delegates to the concrete implementation),
        // may log helpful information and validate settings.
        void init();

        // Initialize the machine position as reported to the planner/controller.
        // Implementations usually compute forward kinematics at a known motor
        // reference to determine offsets (see ParallelDelta::init_position()).
        void init_position();

        // Convert a Cartesian target (in workspace units) into one or more motor-space
        // moves and enqueue them to the motion planner. Implementations generally:
        //  - compute inverse kinematics for start and target points,
        //  - segment the move (if kinematics are non-linear),
        //  - adjust feed rate based on motor-space distance per segment,
        //  - call mc_move_motors() for each segment.
        // Returns true on success; false if the target is unreachable or aborted.
        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position);

        // Forward kinematics: convert an array of motor positions (e.g., angles) to
        // Cartesian coordinates. n_axis is the number of involved axes to consider.
        void motors_to_cartesian(float* cartesian, float* motors, int n_axis);

        // Inverse kinematics transform only: compute the motor-space coordinates for
        // a Cartesian point without sending anything to the planner. Returns false if
        // the point is unreachable or violates limits.
        bool transform_cartesian_to_motors(float* motors, float* cartesian);

        // Constrain a jog move in-place. Implementations can modify target to keep it
        // within reachable/soft-limit space. See ParallelDelta::constrain_jog().
        void constrain_jog(float* target, plan_line_data_t* pl_data, float* position);

        // Quick per-line validity check. Return true if the line should be rejected
        // (e.g., outside of soft limits). See ParallelDelta::invalid_line().
        bool invalid_line(float* target);

        // Validate an arc move for reachability and correctness in this kinematic system.
        // Most implementations return false (accept) unless special handling is needed for
        // non-Cartesian arcs. See ParallelDelta::invalid_arc().
        bool invalid_arc(
            float* target, plan_line_data_t* pl_data, float* position, float center[3], float radius, size_t caxes[3], bool is_clockwise_arc, int pword_rotations);

        // Query if this kinematics supports homing for the given axis set.
        bool canHome(AxisMask axisMask);

        // Optional custom homing flow handled entirely by the kinematics. Return true if
        // homing was fully performed here (so the core should not do default homing).
        bool kinematics_homing(AxisMask axisMask);

        // Allow the kinematics to release/disable motors that are not needed for a given
        // axisMask and motor bitmask.
        void releaseMotors(AxisMask axisMask, MotorMask motors);

        // Called by limit handling to let the kinematics map axis limits to motors. Return
        // true if any limit was asserted and consumed by the kinematics; update masks.
        bool limitReached(AxisMask& axisMask, MotorMask& motors, MotorMask limited);

    private:
        // Pointer to the concrete implementation selected from the factory.
        ::Kinematics::KinematicSystem* _system = nullptr;
    };

    // KinematicSystem: Abstract interface every concrete kinematics must implement.
    // Implementations map between Cartesian space (what G-code describes) and motor space
    // (what the motion planner actuates), and may impose soft limits or special homing.
    class KinematicSystem : public Configuration::Configurable {
        const char* _name;

    public:
        KinematicSystem(const char* name) : _name(name) {}

        KinematicSystem(const KinematicSystem&)            = delete;
        KinematicSystem(KinematicSystem&&)                 = delete;
        KinematicSystem& operator=(const KinematicSystem&) = delete;
        KinematicSystem& operator=(KinematicSystem&&)      = delete;

        // Convert a Cartesian move to one or more motor-space moves and push them to the
        // planner. Return false if the move is rejected/unreachable.
        virtual bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) = 0;

        // Initialize kinematic-specific state, validate configuration, log details, etc.
        virtual void init() = 0;

        // Set the initial machine position as seen by the controller (e.g., compute
        // offsets using forward kinematics at a known reference).
        virtual void init_position() = 0;  // used to set the machine position at init

        // Optionally constrain a jog target for reachability or soft limits.
        virtual void constrain_jog(float* cartesian, plan_line_data_t* pl_data, float* position) {}

        // Return true if a planned straight line should be considered invalid.
        virtual bool invalid_line(float* cartesian) { return false; }

        // Return true if an arc is invalid for this kinematics. Default: accept.
        virtual bool invalid_arc(
            float* target, plan_line_data_t* pl_data, float* position, float center[3], float radius, size_t caxes[3], bool is_clockwise_arc, int pword_rotations) {
            return false;
        }

        // Forward kinematics: motor positions to Cartesian coordinates.
        virtual void motors_to_cartesian(float* cartesian, float* motors, int n_axis) = 0;

        // Inverse kinematics: Cartesian coordinates to motor positions; no planner output.
        virtual bool transform_cartesian_to_motors(float* motors, float* cartesian) = 0;

        // Optional capabilities and hooks for homing/limits and motor control.
        virtual bool canHome(AxisMask axisMask) { return false; }
        virtual void releaseMotors(AxisMask axisMask, MotorMask motors) {}
        virtual bool limitReached(AxisMask& axisMask, MotorMask& motors, MotorMask limited) { return false; }
        virtual bool kinematics_homing(AxisMask& axisMask) { return false; }

        // Configuration interface.
        // Populate configuration items (override in concrete classes if needed).
        void afterParse() override {}
        void group(Configuration::HandlerBase& handler) override {}
        void validate() override {}

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() { return _name; }

        // Virtual base classes require a virtual destructor.
        virtual ~KinematicSystem() {}
    };

    using KinematicsFactory = Configuration::GenericFactory<KinematicSystem>;
};
