#include "putdown_controller.h"

#include <iostream>


void limitForce(Eigen::Matrix<double, 6, 1> force)
{
    double max_F = 10.0;
    double max_torque = 10.0;
    for(int i = 0; i < 3; i++)
        force(i,0) = std::min(max_F, std::max(-max_F, force(i,0)));
    for(int i = 3; i < 6; i++)
        force(i,0) = std::min(max_torque, std::max(-max_torque, force(i,0)));
}

std::array<double, 7> ModelPredictiveController::controlLaw(const franka::RobotState& robot_state, const ros::Duration& period)
{
    // get state variables
    std::array<double, 7> coriolis_array = robot_model->getCoriolis();
    std::array<double, 42> jacobian_array =
            robot_model->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;

    std::array< double, 6 > F_ext = robot_state.O_F_ext_hat_K;

    Eigen::Vector3d desired_force;

    bool use_force_control = true;
    bool use_velocity_control = false;
    desired_force[0] = 0.0;
    desired_force[1] = 0.0;
    desired_force[2] = -300.0 * std::min(std::max(velocity[2]+0.2, -0.1), 0.1); // low resolution velocity feedback

    // calculate applied force
    Eigen::Matrix<double, 6, 1> force_applied;
    Eigen::Vector3d orientation_error;

    // position velocity control
    if(use_force_control)
    {
        //std::cout << "desired force: " << desired_force << std::endl;
        force_applied.head(3) << desired_force;
    }
    else
    {
        std::cout << "Unknown control method specified: default to no force applied" << std::endl;
        force_applied[0] = 0.0;
        force_applied[1] = 0.0;
        force_applied[2] = 0.0;
    }

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
    Eigen::VectorXd tau_task(7);
    Eigen::VectorXd tau_d(7);

    force_applied.tail(3) << -rotational_stiffness * orientation_error - rotational_damping * velocity.tail(3);

    limitForce(force_applied);
    std::cout << "Force applied: " << force_applied << std::endl;

    // apply the computed force
    tau_task << jacobian.transpose() * force_applied;
    tau_d << tau_task + coriolis;
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    std::array<double, 7> tau_d_zero{0, 0, 0, 0, 0, 0, 0};
    return tau_d_array;
}
