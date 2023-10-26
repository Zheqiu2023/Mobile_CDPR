#include "generic_hw_control_loop.hpp"
#include "cdpr_hw.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdpr_chassis_hw");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(3);
    spinner.start();

    try
    {
        // Create the hardware interface specific to your robot
        std::shared_ptr<cdpr_hw::CdprHWInterface> chassis_hw_interface = std::make_shared<cdpr_hw::CdprHWInterface>(nh);
        chassis_hw_interface->init();

        // Start the control loop
        generic_hw_control_loop::GenericHWControlLoop control_loop(nh, chassis_hw_interface);
        control_loop.run();  // Blocks until shutdown signal recieved
    }
    catch (const ros::Exception& e)
    {
        ROS_FATAL_STREAM("Error while starting control loop: " << e.what());
        return EXIT_FAILURE;
    }

    return 0;
}