#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

enum Topology{in_box, over_box, above_box, next_to_box, below_box, near_wall};

enum HorizontalTopology{outside, over, inside};
enum VerticalTopology{above, level, target, below};

class WorldModel {
public:
    WorldModel();

    double max_position[3]; // x, y, z
    double min_position[3];

    double width; // y direction
    double length; // x direction
    double height; // z direction

    HorizontalTopology determineHorizontalTopology(double x, double y);
    VerticalTopology determineVerticalTopology(double z);
    Topology determineTopology(double x, double y, double z, Topology prev);

    Eigen::Vector2d out(double x, double y);
    Eigen::Vector2d in(double x, double y);

    void visualise();
};
