#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
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
    root_acc[2] = 9.81; // set gravity
    KDL::ChainHdSolver_Vereshchagin my_solver(my_chain, root_acc, nc);

    // Set print rate for comparing commanded vs. measured torques.
    const double print_rate = 10.0;

    // Initialize data fields for the print thread.
    struct {
        std::mutex mutex;
        bool has_data;
        std::array<double, 7> tau_d_last;
        franka::RobotState robot_state;
        std::array<double, 7> gravity;
        State fsmState;
    } print_data{};
    std::atomic_bool running{true}; // flag indicating the status of the hardware

    // Start print thread.
    std::thread print_thread([print_rate, &print_data, &running]() {
        while (running) {
            // Sleep to achieve the desired print rate.
            std::this_thread::sleep_for(
                        std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

            // Try to lock data to avoid read write collisions.
            if (print_data.mutex.try_lock()) {
                if (print_data.has_data) {
                    std::array<double, 7> tau_d_actual{};
                    for (size_t i = 0; i < 7; ++i) {
                        tau_d_actual[i] = print_data.tau_d_last[i] + print_data.gravity[i];
                    }

                    // Print data to console
                    std::cout << "state: " << print_data.fsmState << std::endl
                              << "-----------------------" << std::endl;
                    print_data.has_data = false;
                }
                print_data.mutex.unlock();
            }
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
        VelocityController velocityControl(&model);
        PositionController positionControl(&model);
        CompliantController compliantControl(&model);

        State fsmState = FREE_SPACE;

        double time = 0.0;

        // Define callback for the joint torque control loop.
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                hardware_control_callback =
                [&time, &print_data, &model, &fsmState, &my_solver](
                const franka::RobotState& state, franka::Duration period) -> franka::Torques {

            time += period.toSec();

            // hardcoded stuff for now
            uint nj = 7;
            uint ns = 8;
            uint nc = 6;

            // instantiating solver inputs
            KDL::JntArray q(nj);
            for (uint i=0; i<q.columns(); i++)
                    q(i) = state.q[i];

            KDL::JntArray q_dot(nj);
            for (uint i=0; i<q_dot.columns(); i++)
                    q_dot(i) = state.dq[i];

            KDL::Jacobian alpha(nc);
            KDL::JntArray beta(nc);
            KDL::Wrenches f_ext(ns);
            KDL::JntArray ff_torques(nj);

            // instantiate solver output
            KDL::JntArray q_dotdot(nj);
            KDL::JntArray constraint_torques(nj);
            std::vector<KDL::Twist> x_dotdot(ns);
            KDL::JntArray total_torques(nj);

            std::cout << "calling the solver" << std::endl;
            int result = my_solver.CartToJnt(q, q_dot, q_dotdot, alpha, beta, f_ext, ff_torques, constraint_torques);
            std::cout << "solver returned: " << result << std::endl;
            //my_solver.getTransformedLinkAcceleration(x_dotdot);
            my_solver.getTotalTorque(total_torques);

            std::array<double, 7> tau_d_input;
            std::array<double, 7> tau_gravity = model.gravity(state); // gravity is already compensated in the panda interface
            for (uint i=0; i<tau_d_input.size(); i++)
                tau_d_input[i] = total_torques(i) -tau_gravity[i];

            // Update data to print.
            if (print_data.mutex.try_lock()) {
                print_data.has_data = true;
                print_data.robot_state = state;
                print_data.gravity = model.gravity(state);
                print_data.fsmState = fsmState;
                print_data.mutex.unlock();
            }

            // Send torque command.
            return tau_d_input;
        };

        // Start real-time control loop.
        robot.control(hardware_control_callback);

    } catch (const franka::Exception& ex) {
        running = false;
        std::cerr << ex.what() << std::endl;
    }

    if (print_thread.joinable()) {
        print_thread.join();
    }
    return 0;
}
