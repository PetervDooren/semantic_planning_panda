#include "data_saver.h"

#include <iostream>

void DataSaver::writeData()
{
    //#TODO implement writing to file
}

bool DataSaver::setData(franka::RobotState state,
                   int fsmState_in,
                   std::array<double, 7> tau_input_in,
                   KDL::Jacobian alpha_in,
                   KDL::JntArray beta_in,
                   std::vector<KDL::Wrench> f_ext_in,
                   KDL::JntArray ff_torques_in
                   )
{
    // Update data to print.
    if (mutex.try_lock()) {
        has_data = true;

        robot_state = state;
        //gravity = model.gravity(state);

        fsmState = fsmState_in;
        tau_input = tau_input_in;
        alpha = alpha_in;
        beta = beta_in;
        f_ext = f_ext_in;
        ff_torques = ff_torques_in;

        mutex.unlock();
        return true;
    }
    return false;
}

void DataSaver::printData()
{
    std::cout << "robot state: " << fsmState << std::endl
              << "popov-vershchagin motion drivers:" << std::endl
              << "alpha" << alpha << std::endl
              << "beta" << beta << std::endl
              << "external forces" << f_ext << std::endl
              << "feedforward torques" << ff_torques << std::endl
              << "input torque: "  << tau_input << std::endl;
}
