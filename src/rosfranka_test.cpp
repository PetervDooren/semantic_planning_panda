// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "rosfranka_test.h"

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace semantic_planning_panda {

    bool MyController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("MyController: Could not get parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("MyController: Could not parse joint names");
        }
        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("MyController: Wrong number of joint names, got "
                                     << joint_names.size() << " instead of 7 names!");
            return false;
        }

        auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM("MyController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "MyController: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM("MyController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM("MyController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("MyController: Could not get state interface from hardware");
            return false;
        }

        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    state_interface->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                    "MyController: Exception getting state handle: " << e.what());
            return false;
        }

        // configure controller
        MPCControl = CornerPlaceController(model_handle_.get());

        trigger_service_ = node_handle.advertiseService("trigger", &MyController::trigger_callback, this);

        return true;
    }

    void MyController::starting(const ros::Time& /* time */) {
        elapsed_time_ = ros::Duration(0.0);
    }

    void MyController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
        if (active) {
            elapsed_time_ += period;
            franka::RobotState robot_state = state_handle_->getRobotState();

            std::array<double, 7> tau_d_input = MPCControl.controlLaw(robot_state, period);

            for (int i = 0; i < joint_handles_.size(); i++) {
                joint_handles_[i].setCommand(tau_d_input[i]);
            }
        }
        else
        {
            for (auto joint_handle: joint_handles_) {
                joint_handle.setCommand(0.0);
            }
        }
    }

    void MyController::stopping(const ros::Time& /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    }

    bool MyController::trigger_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
    {
        active = !active;
        res.success = true;
        if (active)
            res.message = "MyController active has been set to True";
        else
            res.message = "MyController active has been set to False";
        return true;
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(semantic_planning_panda::MyController,
                       controller_interface::ControllerBase)
