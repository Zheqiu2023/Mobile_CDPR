#include "cdpr_hw/hw/cdpr_hw.hpp"

using namespace cdpr_hw;

CdprHW::CdprHW(ros::NodeHandle& nh, urdf::Model* urdf_model)
{
    use_rosparam_joint_limits_ = false;
    use_soft_limits_if_available_ = false;

    // Check if the URDF model needs to be loaded
    if (urdf_model == nullptr)
        loadURDF(nh, "robot_description");
    else
        urdf_model_ = urdf_model;
}

bool CdprHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    XmlRpc::XmlRpcValue xml_rpc_value;
    // Parse actuator coefficient specified by user (stored on ROS parameter server)
    if (!robot_hw_nh.getParam("actuator_coefficient", xml_rpc_value))
        ROS_WARN("No actuator coefficient specified");
    else if (!parseActCoeffs(xml_rpc_value))
        return false;

    // Parse actuator specified by user (stored on ROS parameter server)
    if (!robot_hw_nh.getParam("actuators", xml_rpc_value))
        ROS_WARN("No actuator specified");
    else if (!parseActData(xml_rpc_value, robot_hw_nh))
        return false;

    // CAN Bus
    if (!robot_hw_nh.getParam("can_bus", xml_rpc_value))
        ROS_WARN("No can bus specified");
    else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        ROS_ASSERT(xml_rpc_value.hasMember("dev_ind"));
        ROS_ASSERT(xml_rpc_value.hasMember("can_ind"));
        for (int i = 0; i < xml_rpc_value["dev_ind"].size(); ++i)
        {
            int dev_ind = xml_rpc_value["dev_ind"][i];
            for (int j = 0; j < xml_rpc_value["can_ind"].size(); ++j)
            {
                int can_ind = xml_rpc_value["can_ind"][j][0];
                int baud_rate = xml_rpc_value["can_ind"][j][1];
                std::vector<int> dev_can{ dev_ind, can_ind };
                can_buses_.push_back(new CanBus(
                    dev_ind, can_ind, baud_rate,
                    ActDataPtr{ .type2act_coeffs_ = &type2act_coeffs_, .id2act_data_ = &comm_id2act_data_[dev_can] }));
            }
        }
    }

    // RS485
    if (!robot_hw_nh.getParam("usb_port", xml_rpc_value))
        ROS_WARN("No usb port specified");
    else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0; i < xml_rpc_value.size(); ++i)
        {
            ROS_ASSERT(xml_rpc_value[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string port_name = xml_rpc_value[i];
            if (port_name.find("/dev/ttyUSB") != std::string::npos)
            {
                std::vector<int> port_num{ port_name.back() - '0' };
                mod_buses_.push_back(new ModBus(port_name, ActDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
                                                                       .id2act_data_ = &comm_id2act_data_[port_num] }));
            }
        }
    }

    // Initialize transmission
    if (!setupTransmission(root_nh))
    {
        ROS_ERROR("Error occurred while setting up transmission");
        return false;
    }
    // Initialize joint limit
    if (!setupJointLimit(root_nh))
    {
        ROS_ERROR("Error occurred while setting up joint limit");
        return false;
    }

    // Other Interface
    // registerInterface(&robot_state_interface_);/

    actuator_state_pub_.reset(
        new realtime_tools::RealtimePublisher<general_file::ActuatorState>(root_nh, "/actuator_states", 100));

    return true;
}

// Read the joint states from hardware
void CdprHW::read(const ros::Time& time, const ros::Duration& period)
{
    // read state from CAN
    for (auto can_bus : can_buses_)
        can_bus->read(time);
    // read state from RS485
    for (auto mod_bus : mod_buses_)
        mod_bus->read(time);

    for (auto& comm_id2act_data : comm_id2act_data_)
        for (auto& id2act_data : comm_id2act_data.second)
        {
            try
            {  // Duration will be out of dual 32-bit range while motor failure
                id2act_data.second.halted =
                    (time - id2act_data.second.stamp).toSec() > 0.1 || id2act_data.second.temp > 99;
            }
            catch (std::runtime_error& ex)
            {
            }
            if (id2act_data.second.halted)
            {
                id2act_data.second.seq = 0;
                id2act_data.second.vel = 0;
                id2act_data.second.effort = 0;
            }
        }

    if (is_actuator_specified_)
        act_to_jnt_state_->propagate();
    // Set all cmd to zero to avoid crazy soft limit oscillation when no controller loaded
    for (auto effort_joint_handle : effort_joint_handles_)
        effort_joint_handle.setCommand(0.);
}

void CdprHW::write(const ros::Time& time, const ros::Duration& period)
{
    if (is_actuator_specified_)
    {
        // enforceLimits will limit cmd_effort into suitable value
        // https://github.com/ros-controls/ros_control/wiki/joint_limits_interface
        effort_jnt_saturation_interface_.enforceLimits(period);
        effort_jnt_soft_limits_interface_.enforceLimits(period);
        // Propagate with joint limits
        jnt_to_act_effort_->propagate();
    }
    // write cmd through CAN
    for (auto can_bus : can_buses_)
        can_bus->write();
    // write cmd through RS485
    for (auto mod_bus : mod_buses_)
        mod_bus->write();
    publishActuatorState(time);
}

void CdprHW::publishActuatorState(const ros::Time& time)
{
    if (!is_actuator_specified_)
        return;
    if (last_publish_time_ + ros::Duration(1.0 / 100.0) < time)
    {
        if (actuator_state_pub_->trylock())
        {
            general_file::ActuatorState actuator_state;
            for (const auto& comm_id2act_data : comm_id2act_data_)
                for (const auto& id2act_data : comm_id2act_data.second)
                {
                    actuator_state.stamp.push_back(id2act_data.second.stamp);
                    actuator_state.name.push_back(id2act_data.second.name);
                    actuator_state.type.push_back(id2act_data.second.type);
                    actuator_state.comm.push_back(comm_id2act_data.first[0]);
                    actuator_state.id.push_back(id2act_data.first);
                    actuator_state.halted.push_back(id2act_data.second.halted);
                    actuator_state.position_raw.push_back(id2act_data.second.q_raw);
                    actuator_state.velocity_raw.push_back(id2act_data.second.qd_raw);
                    actuator_state.temperature.push_back(id2act_data.second.temp);
                    actuator_state.circle.push_back(id2act_data.second.q_circle);
                    actuator_state.last_position_raw.push_back(id2act_data.second.q_last);
                    actuator_state.frequency.push_back(id2act_data.second.frequency);
                    actuator_state.position.push_back(id2act_data.second.pos);
                    actuator_state.velocity.push_back(id2act_data.second.vel);
                    actuator_state.effort.push_back(id2act_data.second.effort);
                    actuator_state.executed_effort.push_back(id2act_data.second.exe_effort);
                    actuator_state.offset.push_back(id2act_data.second.offset);
                }
            actuator_state_pub_->msg_ = actuator_state;
            actuator_state_pub_->unlockAndPublish();
            last_publish_time_ = time;
        }
    }
}