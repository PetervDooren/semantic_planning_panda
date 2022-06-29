#include "position_control.h"

#include <iostream>
#include <math.h>


void limitForce(Eigen::Matrix<double, 6, 1> force)
{
    double max_F = 10.0;
    double max_torque = 10.0;
    for(int i = 0; i < 3; i++)
        force(i,0) = std::min(max_F, std::max(-max_F, force(i,0)));
    for(int i = 3; i < 6; i++)
        force(i,0) = std::min(max_torque, std::max(-max_torque, force(i,0)));
}

std::array<double, 7> PositionController::controlLaw(const franka::RobotState& robot_state, franka::Duration period, Eigen::Vector3d desired_position)
{
    // aggregate robot state

    // get state variables
    std::array<double, 7> coriolis_array = robot_model->coriolis(robot_state);
    std::array<double, 42> jacobian_array =
        robot_model->zeroJacobian(franka::Frame::kEndEffector, robot_state);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;

    // calculate applied force
    Eigen::Matrix<double, 6, 1> force_applied;
    Eigen::Vector3d orientation_error;

    // position control
    double position_gain = 150;
    double damping = 2*sqrt(position_gain);
    force_applied.head(3) << position_gain * (desired_position - position.head(3)) - damping * velocity.head(3);

    // orientation position control
    // orientation error
    Eigen::Quaterniond orientation_d(0.0, 1.0, 0.0, 0.0); // w, x, y, z
    const double rotational_stiffness = 15.0;
    const double rotational_damping = 2.0 * sqrt(rotational_stiffness);

    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    orientation_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    orientation_error.tail(3) << -transform.linear() * orientation_error.tail(3);
    // compute control
    Eigen::VectorXd tau_task(7), tau_d(7);

    force_applied.tail(3) << -rotational_stiffness * orientation_error - rotational_damping * velocity.tail(3);

    limitForce(force_applied);
    //std::cout << "Force applied: " << force_applied << std::endl;

    // apply the computed force
    tau_task << jacobian.transpose() * force_applied;
    tau_d << tau_task + coriolis;
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
    return tau_d_array;
}
