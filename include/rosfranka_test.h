// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>

#include "cornerplace_skill.h"

namespace semantic_planning_panda {

    class MyController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void starting(const ros::Time&) override;
        void stopping(const ros::Time&) override;

    private:
        //hardware_interface::VelocityJointInterface* velocity_joint_interface_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        ros::Duration elapsed_time_;

        // controller
        CornerPlaceController MPCControl;


        bool active = false;
        ros::ServiceServer trigger_service_;
        bool trigger_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    };

}  // namespace franka_example_controllers