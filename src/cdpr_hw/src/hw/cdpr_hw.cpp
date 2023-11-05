#include "cdpr_hw/hw/cdpr_hw.hpp"

using namespace cdpr_hw;

CdprHW::CdprHW(ros::NodeHandle& nh, urdf::Model* urdf_model) : nh_(nh), name_("CdprHW")
{
    use_rosparam_joint_limits_ = false;
    use_soft_limits_if_available_ = true;

    // Check if the URDF model needs to be loaded
    if (urdf_model == nullptr)
    {
        loadURDF(nh, "robot_description");
    }
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
    // registerInterface(&joint_state_interface_);

    actuator_state_pub_.reset(
        new realtime_tools::RealtimePublisher<cdpr_bringup::ActuatorState>(root_nh, "/actuator_states", 100));

    // CAN Bus
    if (!robot_hw_nh.getParam("can_bus", xml_rpc_value))
        ROS_WARN("No can bus specified");
    else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        for (auto iter = xml_rpc_value.begin(); iter != xml_rpc_value.end(); ++iter)
        {
            // if (iter->first.find("dev_ind") == std::string::npos)
            // {
            //     ROS_WARN("Invalid CAN device index in .yaml file");
            //     continue;
            // }
            // std::string dev_ind = iter->first;
            // int dev_ind_num = dev_ind.back() - '0';
            // if (VCI_OpenDevice(VCI_USBCAN2, dev_ind_num, 0) != 1)  // open usb_can device：each device can only be
            // open
            //                                                        // once
            //     ROS_ERROR("Failed to open USB_CAN[%d].", dev_ind);
            // for (auto it = iter->second.begin(); it != iter->second.end(); ++it)
            // {
            //     if (it->first.find("can_ind") != std::string::npos)
            //     {
            //         std::string can_ind = it->first;
            //         int can_ind_num = can_ind.back() - '0';
            //         int baud_rate = static_cast<int>(it->second);
            //         std::vector<int> dev_can{ dev_ind_num, can_ind_num };
            //         can_buses_.push_back(new CanBus(dev_ind_num, can_ind_num, baud_rate,
            //                                         ActDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
            //                                                     .id2act_data_ = &comm_id2act_data_[dev_can] }));
            //     }
            // }
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
                for (auto& id2act_data : comm_id2act_data_[port_num])
                {
                    if (id2act_data.second.type.find("a1") != std::string::npos)
                    {
                        unitree_a1_.push_back(
                            new UnitreeMotorA1(port_name, ActDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
                                                                      .id2act_data_ = &comm_id2act_data_[port_num] }));
                    }
                    // else if (id2act_data.second.type.find("go") != std::string::npos)
                    // {
                    //     unitree_go_.push_back(
                    //         new UnitreeMotorGo(port_name, ActDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
                    //                                                   .id2act_data_ = &comm_id2act_data_[port_num]
                    //                                                   }));
                    // }
                    else
                        ROS_WARN_STREAM("Invalid motor type: " << id2act_data.second.type);
                    break;
                }
            }
            else
                ROS_WARN_STREAM("Invalid port name: " << port_name);
        }
    }

    return true;
}

// Read the joint states from hardware
void CdprHW::read(const ros::Time& time, const ros::Duration& period)
{
    // read state from CAN
    // for (auto can_bus : can_buses_)
    //     can_bus->read(time);
    // read state from RS485
    for (auto unitree_a1 : unitree_a1_)
        unitree_a1->read(time);
    // for (auto unitree_go : unitree_go_)
    //     unitree_go->read(time);

    for (auto& comm_id2act_data : comm_id2act_data_)
        for (auto& id2act_data : comm_id2act_data.second)
        {
            // try
            // {  // Duration will be out of dual 32-bit range while motor failure
            //     id2act_data.second.halted =
            //         (time - id2act_data.second.stamp).toSec() > 0.5 || id2act_data.second.temp > 99;
            // }
            // catch (std::runtime_error& ex)
            // {
            // }
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
    // for (auto can_bus : can_buses_)
    //     can_bus->write();
    // write cmd through RS485
    for (auto unitree_a1 : unitree_a1_)
        unitree_a1->write();
    // for (auto unitree_go : unitree_go_)
    //     unitree_go->write();
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
            cdpr_bringup::ActuatorState actuator_state;
            for (const auto& comm_id2act_data : comm_id2act_data_)
                for (const auto& id2act_data : comm_id2act_data.second)
                {
                    actuator_state.stamp.push_back(id2act_data.second.stamp);
                    actuator_state.name.push_back(id2act_data.second.name);
                    actuator_state.type.push_back(id2act_data.second.type);
                    actuator_state.comm.push_back(comm_id2act_data.first[0]);
                    actuator_state.id.push_back(id2act_data.first);
                    actuator_state.halted.push_back(id2act_data.second.halted);
                    actuator_state.temperature.push_back(id2act_data.second.temp);
                    actuator_state.frequency.push_back(id2act_data.second.frequency);
                    actuator_state.position.push_back(id2act_data.second.pos);
                    actuator_state.velocity.push_back(id2act_data.second.vel);
                    actuator_state.effort.push_back(id2act_data.second.effort);
                    actuator_state.executed_effort.push_back(id2act_data.second.exe_effort);
                    actuator_state.offset.push_back(id2act_data.second.zero_point);
                }
            actuator_state_pub_->msg_ = actuator_state;
            actuator_state_pub_->unlockAndPublish();
            last_publish_time_ = time;
        }
    }
}