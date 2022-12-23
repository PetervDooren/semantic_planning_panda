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

class ModelPredictiveController {
public:
    ModelPredictiveController(){}

    explicit ModelPredictiveController(franka_hw::FrankaModelHandle *model_handle)
    {
        robot_model = model_handle;
    }

    std::array<double, 7> controlLaw(const franka::RobotState& robot_state, const ros::Duration& period);

private:

    franka_hw::FrankaModelHandle *robot_model = nullptr;
};
