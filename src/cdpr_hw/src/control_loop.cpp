#include "cdpr_hw/control_loop.hpp"

using namespace cdpr_hw;

ControlLoop::ControlLoop(ros::NodeHandle& nh, std::shared_ptr<CdprHW> hardware_interface)
  : nh_(nh), hardware_interface_(std::move(hardware_interface))
{
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

    // Load rosparams
    ros::NodeHandle clnh("~");
    std::size_t error = 0;
    error += !clnh.getParam("loop_frequency", loop_hz_);
    error += !clnh.getParam("cycle_time_error_threshold", cycle_time_error_threshold_);
    if (error > 0)
    {
        std::string error_message = "could not load parameters loop_hz or cycle_time_error_threshold";
        ROS_ERROR_STREAM(error_message.c_str());
        throw std::runtime_error(error_message);
    }

    hardware_interface_->init(nh, clnh);

    // Get current time for use with first update
    last_time_ = clock::now();
}

/**
 * @brief Run the control loop (blocking)
 */
void ControlLoop::run()
{
    ros::Rate rate(loop_hz_);
    while (ros::ok())
    {
        update();
        rate.sleep();
    }
}

/**
 * @brief Update funcion called with loop_hz_ rate
 */
void ControlLoop::update()
{
    // Get change in time
    current_time_ = clock::now();
    const duration<double> desired_duration(1.0 / loop_hz_);
    duration<double> time_span = duration_cast<duration<double>>(current_time_ - last_time_);
    elapsed_time_ = ros::Duration(time_span.count());
    last_time_ = current_time_;
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "hw_main", "Sampled update loop with elapsed time " << elapsed_time_.toSec());

    // Error check cycle time
    const double cycle_time_error = (elapsed_time_ - ros::Duration(desired_duration.count())).toSec();
    if (cycle_time_error > cycle_time_error_threshold_)
    {
        ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                         << cycle_time_error << ", cycle time: " << elapsed_time_
                                         << ", threshold: " << cycle_time_error_threshold_);
    }

    // Input
    hardware_interface_->read(ros::Time::now(), elapsed_time_);

    // Control
    controller_manager_->update(ros::Time::now(), elapsed_time_);

    // Output
    hardware_interface_->write(ros::Time::now(), elapsed_time_);
}
