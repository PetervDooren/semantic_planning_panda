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
          std::array<double, 7> tau_error{};
          double error_rms(0.0);
          std::array<double, 7> tau_d_actual{};
          for (size_t i = 0; i < 7; ++i) {
            tau_d_actual[i] = print_data.tau_d_last[i] + print_data.gravity[i];
            tau_error[i] = tau_d_actual[i] - print_data.robot_state.tau_J[i];
            error_rms += std::pow(tau_error[i], 2.0) / tau_error.size();
          }
          error_rms = std::sqrt(error_rms);

          // Print data to console
          std::cout << "tau_error [Nm]: " << tau_error << std::endl
                    << "tau_commanded [Nm]: " << tau_d_actual << std::endl
                    << "tau_measured [Nm]: " << print_data.robot_state.tau_J << std::endl
                    << "root mean square of tau_error [Nm]: " << error_rms << std::endl
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
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();

    std::array<double, 7> tau_d_input = {0, 0, 0, 0, 0, 0, 0};

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        hardware_control_callback =
            [&print_data, &model, &tau_d_input](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
      // Read current coriolis terms from model.

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be
      // adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_input, state.tau_J_d);

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = state;
        print_data.tau_d_last = tau_d_rate_limited;
        print_data.gravity = model.gravity(state);
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
