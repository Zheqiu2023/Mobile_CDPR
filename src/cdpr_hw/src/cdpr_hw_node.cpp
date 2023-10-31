#include "cdpr_hw/control_loop.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdpr_hw");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    try
    {
        // Create the hardware interface specific to your robot
        std::shared_ptr<cdpr_hw::CdprHW> cdpr_hw_interface = std::make_shared<cdpr_hw::CdprHW>(nh);

        // Start the control loop
        cdpr_hw::ControlLoop control_loop(nh, cdpr_hw_interface);
        control_loop.run();  // Blocks until shutdown signal recieved
    }
    catch (const ros::Exception& e)
    {
        ROS_FATAL_STREAM("Error while starting control loop: " << e.what());
        return EXIT_FAILURE;
    }

    return 0;
}