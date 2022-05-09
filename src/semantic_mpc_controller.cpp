#include "semantic_mpc_controller.h"

#include <iostream>

enum HorizontalTopology{outside, over, inside};
enum VerticalTopology{above, level, below};

HorizontalTopology determineHorizontalTopology(double x, double y)
{
    // box dimensions
    double box_x_pos = 0.5;
    double box_y_pos = 0.0;
    double box_z_pos = 0.0;
    double box_length = 0.4; // x direction
    double box_width = 0.6; // y direction
    double wall_width = 0.01; // vertical wall width
    double floor_height = 0.01;

    x = std::abs(x-box_x_pos); // relative potition + symmetry
    y = std::abs(y-box_y_pos); // relative position + symmetry
    if (x > 0.5*box_length + wall_width || y > 0.5*box_width + wall_width)
        return outside;
    if (x < 0.5*box_length || y < 0.5*box_width)
        return inside;
    return over;
}

VerticalTopology determineVerticalTopology(double z)
{
    // box dimensions
    double box_z_pos = 0.0;
    double box_height = 0.3;
    double floor_height = 0.01;

    z = z-box_z_pos; // relative potition
    if (z > box_height)
        return above;
    if (z > floor_height)
        return level;
    return below;
}

Eigen::Vector2d out(double x, double y)
{
    // TODO dirty copy of box parameters
    double box_x_pos = 0.5;
    double box_y_pos = 0.0;
    Eigen::Vector2d outputvel;
    outputvel[0] = 2.0* std::signbit(x - box_x_pos) -1.0;
    outputvel[1] = 2.0* std::signbit(y - box_y_pos) -1.0;
    return outputvel;
}

Eigen::Vector2d in(double x, double y)
{
    // TODO dirty copy of box parameters
    double box_x_pos = 0.5;
    double box_y_pos = 0.0;
    Eigen::Vector2d outputvel;
    outputvel[0] = 2.0* std::signbit(-x + box_x_pos) -1.0;
    outputvel[1] = 2.0* std::signbit(-y + box_y_pos) -1.0;
    return outputvel;
}

std::array<double, 7> ModelPredictiveController::controlLaw(const franka::RobotState& robot_state, franka::Duration period)
{
    // aggregate robot state

    // get state variables
    std::array<double, 7> coriolis_array = robot_model.coriolis(robot_state);
    std::array<double, 42> jacobian_array =
        robot_model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;

    // determine the semantic state of the robot
    HorizontalTopology htop = determineHorizontalTopology(position[0], position[1]);
    VerticalTopology vtop = determineVerticalTopology(position[2]);

    // control law based on this state
    Eigen::Vector3d desired_velocity;
    if(vtop == below)
    {
        if (htop != outside) // htop == inside || htop == over
        {
            desired_velocity.head(2) << out(position[0], position[1]);
            desired_velocity[2] = 0.0;
        }
        else // htop == outside
        {
            desired_velocity[0] = 0.0;
            desired_velocity[1] = 0.0;
            desired_velocity[2] = 1.0; //up
        }
    }
    else if(vtop == above)
    {
        desired_velocity.head(2) << in(position[0], position[1]);
        if (htop == inside)
        {
            desired_velocity[2] = -1.0; //down
        }
        else
        {
            desired_velocity[2] = 0.0;
        }
    }
    else if (vtop == level)
    {
        if (htop == inside)
        {
            desired_velocity[2] = -0.2; //down slowly
        }
        else if (htop == over)
        {
            std::cerr << "Topology is both level and over, this cannot be true as it would imply collision!";
            desired_velocity[0] = 0.0;
            desired_velocity[1] = 0.0;
            desired_velocity[2] = 0.0;
        }
        else // htop == outside
        {
            // #TODO equal to vtop == below && htop == outside
            desired_velocity[0] = 0.0;
            desired_velocity[1] = 0.0;
            desired_velocity[2] = 1.0;
        }
    }

    // calculate applied force
    Eigen::Matrix<double, 6, 1> force_applied;
    Eigen::Vector3d orientation_error;

    // position velocity control
    double velocity_gain = 100;
    force_applied.head(3) << velocity_gain * (desired_velocity - velocity.head(3));

    // orientation position control
    // orientation error
    Eigen::Quaterniond orientation_d(1.0, 0.0, 0.0, 0.0);
    const double rotational_stiffness = 10.0;
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

    force_applied.tail(3) << rotational_stiffness * orientation_error;

    // apply the computed force
    tau_task << jacobian.transpose() * force_applied;
    tau_d << tau_task + coriolis;
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
    return tau_d_array;
}
