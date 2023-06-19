#include "tension_sensors.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tension_sensors_node");

    tension_sensors::TensionSensors tension_sensors;
    tension_sensors.start_read();

    return 0;
}