#include "cornerplace_skill.h"

#include <iostream>


Eigen::Vector3d saturateForce(Eigen::Vector3d force_in, double max_force)
{
    if (force_in.squaredNorm() <= max_force*max_force)
        return force_in;
    return max_force * force_in.normalized();
}

std::array<double, 7> CornerPlaceController::controlLaw(const franka::RobotState& robot_state, const ros::Duration& period)
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
    // monitor contact
    Eigen::Vector3d position_velocity = velocity.head(3);
    std::cout << "Velocity [m/s]: " << position_velocity.norm() << std::endl;
    if (position_velocity.norm() < velocity_threshold)
    {
        std::cout << "-----------------------" << std::endl;
        std::cout << "contact detected" << std::endl;
        std::cout << "-----------------------" << std::endl;
    }


    // Position velocity control
    Eigen::Vector3d desired_force;

    Eigen::Vector3d desired_velocity = max_velocity * desired_direction.normalized();

    desired_force = velocity_gain * (desired_velocity-velocity.head(3));
    Eigen::Vector3d saturated_force = saturateForce(desired_force, max_force);

    // orientation position control
    // orientation error
    Eigen::Vector3d orientation_error;
    Eigen::Quaterniond orientation_d(0.0, 1.0, 0.0, 0.0); // w, x, y, z
    const double rotational_stiffness = 2.0;
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

    // calculate applied force
    Eigen::Matrix<double, 6, 1> force_applied;
    force_applied.head(3) << saturated_force;
    force_applied.tail(3) << -rotational_stiffness * orientation_error - rotational_damping * velocity.tail(3);

    std::cout << "Force applied: " << force_applied << std::endl;

    // apply the computed force
    tau_task << jacobian.transpose() * force_applied;
    tau_d << tau_task + coriolis;
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    std::array<double, 7> tau_d_zero{0, 0, 0, 0, 0, 0, 0};
    return tau_d_array;
}
