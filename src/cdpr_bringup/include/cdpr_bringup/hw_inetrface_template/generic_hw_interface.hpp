#pragma once

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

namespace generic_hardware_interface
{
/// \brief Hardware interface for a robot
class GenericHWInterface : public hardware_interface::RobotHW
{
  public:
    /**
     * \brief Constructor
     * \param nh - Node handle for topics.
     * \param urdf - optional pointer to a parsed robot model
     */
    GenericHWInterface(const ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

    /** \brief Destructor */
    virtual ~GenericHWInterface()
    {
    }

    /** \brief Initialize the hardware interface */
    virtual void init();

    /** \brief Read the state from the robot hardware. */
    virtual void read(ros::Duration& elapsed_time) = 0;

    /** \brief Read the state from the robot hardware
     *
     * \note This delegates RobotHW::read() calls to read()
     *
     * \param time The current time, currently unused
     * \param period The time passed since the last call
     */
    virtual void read(const ros::Time& /*time*/, const ros::Duration& period) override
    {
        ros::Duration elapsed_time = period;
        read(elapsed_time);
    }

    /** \brief Write the command to the robot hardware. */
    virtual void write(ros::Duration& elapsed_time) = 0;

    /** \brief Write the command to the robot hardware
     *
     * \note This delegates RobotHW::write() calls to \ref write()
     *
     * \param time The current time, currently unused
     * \param period The time passed since the last call
     */
    virtual void write(const ros::Time& /*time*/, const ros::Duration& period) override
    {
        ros::Duration elapsed_time = period;
        write(elapsed_time);
    }

    /** \brief Set all members to default values */
    virtual void reset();

    /**
     * \brief Check (in non-realtime) if given controllers could be started and stopped from the
     * current state of the RobotHW
     * with regard to necessary hardware interface switches. Start and stop list are disjoint.
     * This is just a check, the actual switch is done in doSwitch()
     */
    virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                           const std::list<hardware_interface::ControllerInfo>& stop_list) const
    {
        return true;
    }

    /**
     * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
     * and stop the given controllers.
     * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
     */
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& stop_list)
    {
    }

    /**
     * \brief Register the limits of the joint specified by joint_id and joint_handle. The limits
     * are retrieved from the urdf_model.
     *
     * \return the joint's type, lower position limit, upper position limit, and effort limit.
     */
    virtual void registerJointLimits(const hardware_interface::JointHandle& joint_handle_position,
                                     const hardware_interface::JointHandle& joint_handle_velocity,
                                     const hardware_interface::JointHandle& joint_handle_effort, std::size_t joint_id);

    /** \breif Enforce limits for all values before writing */
    virtual void enforceLimits(ros::Duration& period) = 0;

    /** \brief Helper for debugging a joint's state */
    virtual void printState();
    std::string printStateHelper();

    /** \brief Helper for debugging a joint's command */
    std::string printCommandHelper();

  protected:
    /** \brief Get the URDF XML from the parameter server */
    virtual void loadURDF(const ros::NodeHandle& nh, std::string param_name);

    // Short name of this class
    std::string name_;

    // Startup and shutdown of the internal node inside a roscpp program
    ros::NodeHandle nh_;

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    // Joint limits interfaces - Saturation
    joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;
    joint_limits_interface::VelocityJointSaturationInterface vel_jnt_sat_interface_;
    joint_limits_interface::EffortJointSaturationInterface eff_jnt_sat_interface_;

    // Joint limits interfaces - Soft limits
    joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_jnt_soft_limits_;
    joint_limits_interface::EffortJointSoftLimitsInterface eff_jnt_soft_limits_;

    // Custom or available transmissions
    // transmission_interface::ROBOTTransmission ROBOT_trans_;
    // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

    // Configuration
    std::vector<std::string> joint_names_;
    int joint_mode_;  // position, velocity, or effort
    std::size_t num_joints_;
    urdf::Model* urdf_model_;

    // Modes
    bool use_rosparam_joint_limits_;
    bool use_soft_limits_if_available_;

    // States
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Commands
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;

    // Copy of limits, in case we need them later in our control stack
    std::vector<double> joint_position_lower_limits_;
    std::vector<double> joint_position_upper_limits_;
    std::vector<double> joint_velocity_limits_;
    std::vector<double> joint_effort_limits_;

};  // class

}  // namespace generic_hardware_interface