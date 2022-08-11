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

#include "examples_common.h"
#include "semantic_mpc_controller.h"
#include "position_control.h"
#include "velocity_control.h"
#include "compliant_control.h"
#include "tube_control.h"


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
    TUBE_CONTROL,
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

              Eigen::Affine3d transform(Eigen::Matrix4d::Map(print_data.robot_state.O_T_EE.data()));
              Eigen::Vector3d position(transform.translation());

              // Print data to console
              std::cout << "state: " << print_data.fsmState << std::endl
                        << "-----------------------" << std::endl;
              std::cout << "Position [m]: " << position << std::endl;
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
    TubeController tubeControl(&model);

    double desired_velocity = 0.2;
    tubeControl.setvelocity(desired_velocity);

    State fsmState = FREE_SPACE;

    double time = 0.0;

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        hardware_control_callback =
            [&time, &print_data, &model, &fsmState, &velocityControl, &positionControl, &compliantControl, &tubeControl, &desired_velocity](
                const franka::RobotState& state, franka::Duration period) -> franka::Torques {

        time += period.toSec();

        if (time < 2.0)
            fsmState = FREE_SPACE;
        else if (time < 6.0)
            fsmState = TUBE_CONTROL;
        else
            fsmState = FINISHED;

        std::array<double, 7> tau_d_input = {0, 0, 0, 0, 0, 0, 0};

        Eigen::Vector3d desired_position = {0.65, -0.3, 0.15};

        switch(fsmState){
        case FREE_SPACE:
            tau_d_input = positionControl.controlLaw(state, period, desired_position);
            break;
        case TUBE_CONTROL:
            tau_d_input = tubeControl.controlLaw(state, period);
            break;
        case FINISHED:
            desired_velocity = -desired_velocity;
            tubeControl.setvelocity(desired_velocity);
            time = 2.0; // reset
            break;

        }

      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_input, state.tau_J_d);

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = state;
        print_data.tau_d_last = tau_d_rate_limited;
        print_data.gravity = model.gravity(state);
        print_data.fsmState = fsmState;
        print_data.mutex.unlock();
      }

      // Send torque command.
      return tau_d_rate_limited;
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
