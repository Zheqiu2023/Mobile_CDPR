#pragma once

#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <chrono>

#include "hw/cdpr_hw.hpp"

namespace cdpr_hw
{
using namespace std::chrono;
using clock = high_resolution_clock;
/**
 * \brief The control loop - repeatidly calls read() and write() to the hardware interface at a
 * specified frequency
 *        We use MONOTONIC time to ensure robustness in the event of system time updates/change.
 *        See
 * http://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
 */
class ControlLoop
{
  public:
    /**
     * \brief Constructor
     * \param NodeHandle
     * \param hardware_interface - the robot-specific hardware interface to be use with cdpr
     */
    ControlLoop(ros::NodeHandle& nh, std::shared_ptr<CdprHW> hardware_interface);

    // Run the control loop (blocking)
    void run();

  protected:
    // Update funcion called with loop_hz_ rate
    void update();

    // Startup and shutdown of the internal node inside a roscpp program
    ros::NodeHandle nh_;

    // Name of this class
    std::string name_ = "ControlLoop";

    // Settings
    double cycle_time_error_threshold_ = 0.0;

    // Timing
    ros::Duration elapsed_time_;
    double loop_hz_ = 0.0;
    clock::time_point last_time_;
    clock::time_point current_time_;

    /** \brief ROS Controller Manager and Runner
     *
     * This class advertises a ROS interface for loading, unloading, starting, and
     * stopping ros_control-based controllers. It also serializes execution of all
     * running controllers in \ref update.
     */
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    /** \brief Abstract Hardware Interface for cdpr */
    std::shared_ptr<CdprHW> hardware_interface_;

};  // end class

}  // namespace cdpr_hw