#pragma once

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "general_file/ActuatorState.h"
#include "comm_protocol.hpp"
#include "types.hpp"

namespace cdpr_hw
{
/// \brief Hardware interface for cdpr chassis
class CdprHW : public hardware_interface::RobotHW
{
  public:
    /**
     * \brief Constructor
     *
     * Load urdf of robot.
     *
     * \param nh - Node handle for topics.
     * \param urdf - optional pointer to a parsed robot model
     */
    CdprHW(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

    /** \brief Get necessary params from param server. Init hardware_interface.
     *
     * Get params from param server and check whether these params are set. Set up transmission and
     * joint limit.
     *
     * @param root_nh Root node-handle of a ROS node.
     * @param robot_hw_nh Node-handle for robot hardware.
     * @return True when init successful, False when failed.
     */
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

    /** \brief Read the state from the robot hardware
     * \param time The current time, currently unused
     * \param period The time passed since the last call
     */
    void read(const ros::Time& time, const ros::Duration& period) override;

    /** \brief Write the command to the robot hardware
     * \param time The current time, currently unused
     * \param period The time passed since the last call
     */
    void write(const ros::Time& time, const ros::Duration& period) override;

  private:
    /** \brief Get the URDF XML from the parameter server */
    bool loadURDF(const ros::NodeHandle& nh, std::string param_name);

    /** \brief Check whether some coefficients that are related to actuator are set up and load these coefficients.
     * @param act_coeffs Coefficients you want to check and load.
     * @return True if all coefficients are set up.
     */
    bool parseActCoeffs(XmlRpc::XmlRpcValue& act_coeffs);

    /** \brief Check whether actuator is specified and load specified params.
     * @param act_datas Params you want to check and load.
     * @param robot_hw_nh Root node-handle of a ROS node.
     * @return True if all params are set up.
     */
    bool parseActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh);

    /** \brief Set up transmission.
     * @param root_nh Root node-handle of a ROS node.
     * @return True if successful.
     */
    bool setupTransmission(ros::NodeHandle& root_nh);

    /** \brief Set up joint limit.
     * @param root_nh Root node-handle of a ROS node.
     * @return True if successful.
     */
    bool setupJointLimit(ros::NodeHandle& root_nh);

    /** \brief Publish actuator's state to a topic named "/actuator_states".
     * @param time Current time
     */
    void publishActuatorState(const ros::Time& time);

    // Short name of this class
    std::string name_;

    // Configuration
    std::string urdf_string_{};
    urdf::Model* urdf_model_;

    // Modes
    bool use_rosparam_joint_limits_;
    bool use_soft_limits_if_available_;

    hardware_interface::ActuatorStateInterface act_state_interface_;
    hardware_interface::EffortActuatorInterface effort_act_interface_;

    std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_{};
    transmission_interface::RobotTransmissions robot_transmissions_;
    transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_{};
    transmission_interface::JointToActuatorEffortInterface* jnt_to_act_effort_{};
    joint_limits_interface::EffortJointSaturationInterface effort_jnt_saturation_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface effort_jnt_soft_limits_interface_;
    std::vector<hardware_interface::JointHandle> effort_joint_handles_{};

    // Actuator
    bool is_actuator_specified_ = false;
    std::vector<CanBus*> can_buses_{};
    std::vector<ModBus*> mod_buses_{};
    std::unordered_map<std::string, ActCoeff> type2act_coeffs_{};
    std::unordered_map<std::vector<int>, std::unordered_map<int, ActData>, VectorHasher> comm_id2act_data_{};

    ros::Time last_publish_time_;
    std::shared_ptr<realtime_tools::RealtimePublisher<general_file::ActuatorState>> actuator_state_pub_;
};

}  // namespace cdpr_hw
