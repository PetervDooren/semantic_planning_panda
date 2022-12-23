#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/model.h>

#include <franka_hw/franka_model_interface.h>

class CornerPlaceController {
public:
    CornerPlaceController(){}

    explicit CornerPlaceController(franka_hw::FrankaModelHandle *model_handle)
    {
        robot_model = model_handle;
    }

    std::array<double, 7> controlLaw(const franka::RobotState& robot_state, const ros::Duration& period);

private:
    Eigen::Vector3d desired_direction= Eigen::Vector3d(0, -1, -1);
    Eigen::Quaterniond orientation_d = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0); // w, x, y, z

    // position velocity control
    double max_velocity = 0.2;
    double velocity_gain = 300;
    double max_force = 10;

    // orientation position control
    double rotation_gain = 2.0;
    double rotational_damping = 2.0;
    double max_torque = 5;

    // contact monitor
    double velocity_threshold = 0.002;
    franka_hw::FrankaModelHandle *robot_model = nullptr;
};
