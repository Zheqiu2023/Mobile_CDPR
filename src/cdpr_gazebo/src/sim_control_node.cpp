#include "sim_control.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdpr_gazebo_node");

    sim_control::SimControl sim_ctrl;
    sim_ctrl.moveAround();

    return 0;
}