#pragma once

#include <franka/robot.h>
#include <franka/model.h>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <vector>

class DataSaver {
public:
    // write data to file.
    void writeData();

    // set internal data structure.
    bool setData(franka::RobotState state,
                 int fsmState_in,
                 std::array<double, 7> tau_input_in,
                 std::array<double, 7> gravity,
                 KDL::Jacobian alpha_in,
                 KDL::JntArray beta_in,
                 std::vector<KDL::Wrench> f_ext_in,
                 KDL::JntArray ff_torques_in);

    // print internal data to screen in human readable format.
    void printData();

    bool has_data = false;

private:

        std::mutex mutex;

        //#TODO make data to be
        franka::RobotState robot_state;
        std::array<double, 7> gravity;

        // control info
        int fsmState;
        std::array<double, 7> tau_input;

        // popov-vereshchagin motion drivers
        KDL::Jacobian alpha;
        KDL::JntArray beta;
        std::vector<KDL::Wrench> f_ext;
        KDL::JntArray ff_torques;
};
