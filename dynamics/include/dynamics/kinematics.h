#pragma once

#include "dynamics/model_params.h"
#include "dynamics/vehicle_state.h"

class Kinematics
{
    public:
        static State update(
                const State start,
                double acc,
                double steer_angle_vel,
                CarParams p,
                double dt);

        static State update_low_vel(
                const State start,
                double acc,
                double steer_angle_vel,
                CarParams p,
                double dt);
};
