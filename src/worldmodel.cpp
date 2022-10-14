#include "worldmodel.h"

#include <iostream>

WorldModel::WorldModel()
{
    double pose_uncertainty = 0.1;
    double pringles_offset = 0.25;
    max_position[0] = 0.65 + pose_uncertainty;
    max_position[1] = 0.0 + pose_uncertainty;
    max_position[2] = 0.05 + pringles_offset; // things are on the floor

    min_position[0] = 0.65 - pose_uncertainty;
    min_position[1] = 0.0 - pose_uncertainty;
    min_position[2] = 0.0 + pringles_offset; // things are on the floor

    width = 0.43;
    length = 0.34;
    height = 0.25;

    // box dimensions
    /*
    #define WALL_WIDTH 0.15
    #define FLOOR_HEIGHT 0.01
    #define TARGET_HEIGHT 0.15
    */
}

HorizontalTopology WorldModel::determineHorizontalTopology(double x, double y)
{
    // maximum value of the highest x coordinate on the box
    double max_max_x = max_position[0] + length/2;
    double min_max_x = min_position[0] + length/2;
    double max_min_x = max_position[0] - length/2;
    double min_min_x = min_position[0] - length/2;

    double max_max_y = max_position[1] + width/2;
    double min_max_y = min_position[1] + width/2;
    double max_min_y = max_position[1] - width/2;
    double min_min_y = min_position[1] - width/2;

    if (x > max_max_x || x < min_min_x || y > max_max_y || y < min_min_y)
        return outside;
    if (x > max_min_x && x < min_max_x && y > max_min_y && y < min_max_y)
        return inside;
    return over;
}

VerticalTopology WorldModel::determineVerticalTopology(double z)
{
    double max_top_z = max_position[2] + height;
    double max_floor_z = max_position[2];
    double min_top_z = min_position[2] + height;
    double min_floor_z = min_position[2];
    if (z > max_top_z)
        return above;
    if (z > min_top_z)
        return level;
    if (z > max_floor_z)
        return target;
    return below;
}

Topology WorldModel::determineTopology(double x, double y, double z, Topology prev)
{
    HorizontalTopology htop = determineHorizontalTopology(x, y);
    VerticalTopology vtop = determineVerticalTopology(z);
    //std::cout << "htop: " << htop << "vtop: " << vtop << std::endl;

    // maximum value of the highest x coordinate on the box
    double max_max_x = max_position[0] + length/2;
    double min_max_x = min_position[0] + length/2;
    double max_min_x = max_position[0] - length/2;
    double min_min_x = min_position[0] - length/2;

    double max_max_y = max_position[1] + width/2;
    double min_max_y = min_position[1] + width/2;
    double max_min_y = max_position[1] - width/2;
    double min_min_y = min_position[1] - width/2;

    double max_top_z = max_position[2] + height;
    double max_floor_z = max_position[2];
    double min_top_z = min_position[2] + height;
    double min_floor_z = min_position[2];

    //FSM
    switch(prev)
    {
    case next_to_box:
        if (z > max_top_z)
            return above_box;
        break;
    case above_box:
        if (htop == inside)
            return over_box;
        if (z < max_top_z)
            return next_to_box;
        break;
    case over_box:
        if (z < max_floor_z)
            return in_box;
        if (htop == outside)
            return above_box;
        break;
    case in_box:
        if (htop == outside) // TODO this would reflect a change in belief state only
            return next_to_box;
        if (z > min_top_z)
            return over_box;
        break;
    }
    return prev;
}

Eigen::Vector2d WorldModel::out(double x, double y)
{
    double dx = x-(max_position[0] + min_position[0])/2;
    double dy = y-(max_position[1] + min_position[1])/2;
    double dist = sqrt(dx*dx + dy*dy);

    Eigen::Vector2d outputvel;
    outputvel[0] = dx/dist;
    outputvel[1] = dy/dist;
    return outputvel;
}

Eigen::Vector2d WorldModel::in(double x, double y)
{
    double dx = x-(max_position[0] + min_position[0])/2;
    double dy = y-(max_position[1] + min_position[1])/2;
    double dist = sqrt(dx*dx + dy*dy);

    Eigen::Vector2d outputvel;
    outputvel[0] = -dx/dist;
    outputvel[1] = -dy/dist;
    return outputvel;
}

void WorldModel::visualise()
{
    std::cout << "sorry use your imagination" << std::endl;
}
