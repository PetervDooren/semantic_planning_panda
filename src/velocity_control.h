#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/model.h>

class VelocityController {
public:
    VelocityController(franka::Model* model)
    {
        robot_model = model;
    }

    std::array<double, 7> controlLaw(const franka::RobotState& robot_state, franka::Duration period, Eigen::Vector3d desired_velocity);

private:
    double maximum_end_effector_velocity;

    franka::Model* robot_model;
};
