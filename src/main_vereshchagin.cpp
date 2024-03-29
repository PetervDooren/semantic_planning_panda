#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainhdsolver_vereshchagin.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <ros/ros.h>

#include "examples_common.h"
#include "semantic_mpc_controller.h"
#include "position_control.h"
#include "velocity_control.h"
#include "compliant_control.h"

#include "data_saver.h"

void velocityControlLaw(Eigen::Vector3d velocity, Eigen::Vector3d reference, KDL::Jacobian& alpha, KDL::JntArray& beta)
{
    // Define axes
    alpha.setColumn(0, KDL::Twist(KDL::Vector(1, 0, 0), KDL::Vector(0,0,0)));
    alpha.setColumn(1, KDL::Twist(KDL::Vector(0, 1, 0), KDL::Vector(0,0,0)));
    alpha.setColumn(2, KDL::Twist(KDL::Vector(0, 0, 1), KDL::Vector(0,0,0)));
    alpha.setColumn(3, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(1,0,0)));
    alpha.setColumn(4, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0,1,0)));
    alpha.setColumn(5, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0,0,1)));

    // x direction
    double gain = 1.0;
    beta(0) = gain * (reference[0] - velocity[0]);
    beta(1) = gain * (reference[1] - velocity[1]);
    beta(2) = gain * (reference[2] - velocity[2]);
}

void jointlimitLaw(KDL::JntArray& q, KDL::JntArray& ff_torques)
{
    KDL::JntArray q_lower_limit(7);
    q_lower_limit(0) = -2.8973;
    q_lower_limit(1) = -1.7628;
    q_lower_limit(2) = -2.8973;
    q_lower_limit(3) = -3.0718;
    q_lower_limit(4) = -2.8973;
    q_lower_limit(5) = -0.0175;
    q_lower_limit(6) = -2.8973;

    KDL::JntArray q_upper_limit(7);
    q_upper_limit(0) = 2.8973;
    q_upper_limit(1) = 1.7628;
    q_upper_limit(2) = 2.8973;
    q_upper_limit(3) = -0.069;
    q_upper_limit(4) = 2.8973;
    q_upper_limit(5) = 3.7525;
    q_upper_limit(6) = 2.8973;

    double eps = 1.0; // [rad] how close we may get to the joint limits
    double tau_max = 10.0;
    for (int i=0; i<7; i++)
    {
        if (q(i) < (q_lower_limit(i) + eps))
            ff_torques(i) = std::min(tau_max, tau_max/eps * (-q(i)+q_lower_limit(i)+eps));
        else if (q(i) > (q_upper_limit(i) - eps))
            ff_torques(i) = - std::min(tau_max, tau_max/eps * (q(i)-q_upper_limit(i)+eps));
        else
            ff_torques(i) = 0.0;
        //std::cout << "i: " << i << ", q: " << q(i) << ", ff: " << ff_torques(i) << ", qL: " << (q_lower_limit(i) + eps) << ", qU: " << (q_upper_limit(i) - eps) << std::endl;
    }
}

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

enum State{
    FREE_SPACE = 0,
    MAKING_CONTACT,
    COMPLIANT_MOTION,
    FINISHED
};

/**
 * Panda experiment code.
 */

int main(int argc, char** argv) {
    // Check whether the required arguments were passed.
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // start node
    ros::init(argc, argv, "main_node");
    ros::NodeHandle node;

    // load panda model
    KDL::Tree my_tree;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return 0;
    }

    std::cout << "Constructed kdl tree" << std::endl;
    KDL::SegmentMap segmap = my_tree.getSegments();
    for (auto it=segmap.begin(); it != segmap.end(); it++)
    {
        std::cout << "   -" <<  it->first << std::endl;
        std::cout << "   mass: " <<  it->second.segment.getInertia().getMass() << std::endl;
    }

    std::cout << "getting chain" << std::endl;
    KDL::Chain my_chain;
    my_tree.getChain("panda_link0", "panda_link7", my_chain);

    std::cout << "created chain with " << my_chain.getNrOfSegments()
              << " segments and " << my_chain.getNrOfJoints() << " joint." << std::endl;

    std::cout << "getting solver" << std::endl;
    uint nj = my_chain.getNrOfJoints(); //number of joints
    uint ns = my_chain.getNrOfSegments(); //number of joints
    uint nc = 6;

    KDL::Twist root_acc = KDL::Twist::Zero();
    root_acc[2] = 0.0;//9.81; // set gravity
    KDL::ChainHdSolver_Vereshchagin my_solver(my_chain, root_acc, nc);

    // Set print rate for comparing commanded vs. measured torques.
    const double print_rate = 10.0;

    // Initialize data fields for the print thread.
    DataSaver data_saver;
    std::atomic_bool running{true}; // flag indicating the status of the hardware

    // Start print thread.
    std::thread print_thread([print_rate, &data_saver, &running]() {
        while (running) {
            // Sleep to achieve the desired print rate.
            std::this_thread::sleep_for(
                        std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

            // Print data to screen
            if (data_saver.has_data)
                data_saver.printData();
        }
    });

    try {
        // Connect to robot.
        franka::Robot robot(argv[1]);
        setDefaultBehavior(robot);

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
                    {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
                    {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
                    {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
                    {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});

        // Load the kinematics and dynamics model.
        franka::Model model = robot.loadModel();

        State fsmState = FREE_SPACE;

        double time = 0.0;

        // Define callback for the joint torque control loop.
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                hardware_control_callback =
                [&time, &data_saver, &model, &fsmState, &my_solver](
                const franka::RobotState& state, franka::Duration period) -> franka::Torques {

            time += period.toSec();

            // get state variables
            std::array<double, 7> coriolis_array = model.coriolis(state);
            std::array<double, 42> jacobian_array =
                model.zeroJacobian(franka::Frame::kEndEffector, state);

            // convert to Eigen
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.linear());
            Eigen::Matrix<double, 6, 1> twist = jacobian * dq;
            Eigen::Vector3d velocity(twist(0), twist(2), twist(3));

            // hardcoded stuff for now
            uint nj = 7;
            uint ns = 7;
            uint nc = 6;

            // instantiating solver inputs
            KDL::JntArray q(nj);
            for (uint i=0; i<q.rows(); i++)
                    q(i) = state.q[i];

            KDL::JntArray q_dot(nj);
            for (uint i=0; i<q_dot.rows(); i++)
                    q_dot(i) = state.dq[i];

            KDL::Jacobian alpha(nc);
            KDL::JntArray beta(nc);
            KDL::Wrenches f_ext(ns);
            KDL::JntArray ff_torques(nj);

            // set solver constraints
            Eigen::Vector3d desired_velocity(0.0, 0.0, 0.5);
            //velocityControlLaw(velocity, desired_velocity, alpha, beta);

            jointlimitLaw(q, ff_torques);

            // instantiate solver output
            KDL::JntArray q_dotdot(nj);
            KDL::JntArray constraint_torques(nj);
            std::vector<KDL::Twist> x_dotdot(ns);
            KDL::JntArray total_torques(nj);

            int result = my_solver.CartToJnt(q, q_dot, q_dotdot, alpha, beta, f_ext, ff_torques, constraint_torques);
            //my_solver.getTransformedLinkAcceleration(x_dotdot);
            my_solver.getTotalTorque(total_torques);

            std::array<double, 7> tau_d_input = {0, 0, 0, 0, 0, 0, 0};
            std::array<double, 7> tau_gravity = model.gravity(state); // gravity is already compensated in the panda interface
            if (result == 0){
                for (uint i=0; i<tau_d_input.size(); i++)
                    tau_d_input[i] = total_torques(i);// -tau_gravity[i];
            }
            std::array<double, 7> tau_d_sub = {0, 0, 0, 0, 0, 0, 0}; // substitute input

            // Update data to print.
            data_saver.setData(state, fsmState, tau_d_input, tau_gravity, alpha, beta, f_ext, ff_torques);

            // Send torque command.
            return tau_d_input;
        };

        // Start real-time control loop.
        robot.control(hardware_control_callback);

    } catch (const franka::Exception& ex) {
        std::cerr << ex.what() << std::endl;
    }
    running = false;

    if (print_thread.joinable()) {
        print_thread.join();
    }
    return 0;
}
