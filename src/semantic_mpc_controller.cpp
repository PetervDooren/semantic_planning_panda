#include "semantic_mpc_controller.h"

#include <iostream>

// box dimensions
#define BOX_X_POS 0.65
#define BOX_Y_POS 0.0
#define BOX_Z_POS 0.0
#define BOX_WIDTH 0.4 //   Y Direction
#define BOX_LENGTH 0.3 // x direction
#define BOX_HEIGHT 0.25
#define WALL_WIDTH 0.05
#define FLOOR_HEIGHT 0.01

enum HorizontalTopology{outside, over, inside};
enum VerticalTopology{above, level, below};

HorizontalTopology determineHorizontalTopology(double x, double y)
{
    x = std::abs(x-BOX_X_POS); // relative potition + symmetry
    y = std::abs(y-BOX_Y_POS); // relative position + symmetry
    if (x > 0.5*BOX_LENGTH + WALL_WIDTH || y > 0.5*BOX_WIDTH + WALL_WIDTH)
        return outside;
    if (x < 0.5*BOX_LENGTH || y < 0.5*BOX_WIDTH)
        return inside;
    return over;
}

VerticalTopology determineVerticalTopology(double z)
{
    z = z-BOX_Z_POS; // relative potition
    if (z > BOX_HEIGHT)
        return above;
    if (z > FLOOR_HEIGHT)
        return level;
    return below;
}

Eigen::Vector2d out(double x, double y)
{
    double dx = x-BOX_X_POS;
    double dy = y-BOX_Y_POS;
    double dist = sqrt(dx*dx + dy*dy);

    Eigen::Vector2d outputvel;
    outputvel[0] = dx/dist;
    outputvel[1] = dy/dist;
    return outputvel;
}

Eigen::Vector2d in(double x, double y)
{
    double dx = x-BOX_X_POS;
    double dy = y-BOX_Y_POS;
    double dist = sqrt(dx*dx + dy*dy);

    Eigen::Vector2d outputvel;
    outputvel[0] = -dx/dist;
    outputvel[1] = -dy/dist;
    return outputvel;
}

void limitForce(Eigen::Matrix<double, 6, 1> force)
{
    double max_F = 10.0;
    double max_torque = 10.0;
    for(int i = 0; i < 3; i++)
        force(i,0) = std::min(max_F, std::max(-max_F, force(i,0)));
    for(int i = 3; i < 6; i++)
        force(i,0) = std::min(max_torque, std::max(-max_torque, force(i,0)));
}

std::array<double, 7> ModelPredictiveController::controlLaw(const franka::RobotState& robot_state, franka::Duration period)
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

    // determine the semantic state of the robot
    HorizontalTopology htop = determineHorizontalTopology(position[0], position[1]);
    VerticalTopology vtop = determineVerticalTopology(position[2]);

    std::cout << "htop: " << htop << " , vtop: " << vtop << std::endl;

    // control law based on this state
    Eigen::Vector3d desired_velocity;
    if(vtop == below)
    {
        if (htop != outside) // htop == inside || htop == over
        {
            std::cout << "Topology is underneath the box" << std::endl;
            desired_velocity.head(2) << out(position[0], position[1]);
            desired_velocity[2] = 0.0;
        }
        else // htop == outside
        {
            std::cout << "Topology is underneath and outside the box" << std::endl;
            desired_velocity[0] = 0.0;
            desired_velocity[1] = 0.0;
            desired_velocity[2] = 1.0; //up
        }
    }
    else if(vtop == above)
    {
        if (htop == inside)
        {
            std::cout << "Topology is over the box" << std::endl;
            desired_velocity[0] = 0.0;
            desired_velocity[1] = 0.0;
            desired_velocity[2] = -1.0; //down
        }
        else
        {
            std::cout << "Topology is above the box" << std::endl;
            desired_velocity.head(2) << in(position[0], position[1]);
            desired_velocity[2] = 0.0;
        }
    }
    else if (vtop == level)
    {
        if (htop == inside)
        {
            std::cout << "Topology is inside the box" << std::endl;
            desired_velocity[0] = 0.0;
            desired_velocity[1] = 0.0;
            desired_velocity[2] = -0.2; //down slowly
        }
        else if (htop == over)
        {
            std::cerr << "Topology is both level and over, this cannot be true as it would imply collision!" << std::endl;
            desired_velocity.head(2) << in(position[0], position[1]);
            desired_velocity[2] = -0.2;
        }
        else // htop == outside
        {
            // #TODO equal to vtop == below && htop == outside
            std::cout << "Topology is besides the box" << std::endl;
            desired_velocity[0] = 0.0;
            desired_velocity[1] = 0.0;
            desired_velocity[2] = 1.0;
        }
    }

    std::cout << "desired velocity: " << desired_velocity << std::endl;

    // calculate applied force
    Eigen::Matrix<double, 6, 1> force_applied;
    Eigen::Vector3d orientation_error;

    // position velocity control
    double velocity_gain = 10;
    force_applied.head(3) << velocity_gain * (desired_velocity - velocity.head(3));

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
