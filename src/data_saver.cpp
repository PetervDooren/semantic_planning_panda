#include "data_saver.h"

#include <iostream>
#include <iterator>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
}
}  // anonymous namespace

void printJntArray(KDL::JntArray& q)
{
    for (uint i=0; i < q.rows(); i++)
        std::cout << "   " << q(i) << std::endl;
}

void printWrench(KDL::Wrench& w)
{
    std::cout << "force [" << w.force.x() << ", " << w.force.y() << ", "<< w.force.z() << "]"
    << " torque [" << w.torque.x() << ", " << w.torque.y() << ", " << w.torque.z() << "]"
    << std::endl;
}

void printWrenches(std::vector<KDL::Wrench>& w)
{
    for (auto it= w.begin(); it != w.end(); it++)
       printWrench(*it);
}

void printJacobian(KDL::Jacobian& J)
{
    for (uint i=0; i < J.rows(); i++)
    {
        for (uint j=0; j < J.columns(); j++)
            std::cout << J(i,j) << ", ";
        std::cout << std::endl;
    }
}


void printTwist(KDL::Twist& t)
{
    std::cout << "vel [" << t.vel.x() << ", " << t.vel.y() << ", "<< t.vel.z() << "]"
    << " rot [" << t.rot.x() << ", " << t.rot.y() << ", " << t.rot.z() << "]"
    << std::endl;
}

void printTwists(std::vector<KDL::Twist>& t)
{
    for (auto it= t.begin(); it != t.end(); it++)
       printTwist(*it);
}

void DataSaver::writeData()
{
    //#TODO implement writing to file
}

bool DataSaver::setData(franka::RobotState state,
                   int fsmState_in,
                   std::array<double, 7> tau_input_in,
                   std::array<double, 7> gravity_in,//TODO use an internal model
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
        gravity = gravity_in;

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
    std::cout << "robot state: " << fsmState << std::endl;
    std::cout << "q: " << robot_state.q << std::endl;

    std::cout << "q_dot: " << robot_state.dq << std::endl;

    std::cout << "popov-vershchagin motion drivers:" << std::endl;
    std::cout << "alpha" << std::endl;
    printJacobian(alpha);

    std::cout << "beta" << std::endl;
    printJntArray(beta);

    std::cout << "external forces" << std::endl;
    printWrenches(f_ext);

    std::cout << "feedforward torques" << std::endl;
    printJntArray(ff_torques);

    std::cout << "desired torques" << tau_input << std::endl;
    std::cout << "gravity torques" << gravity << std::endl;
}
