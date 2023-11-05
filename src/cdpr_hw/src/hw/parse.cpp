#include <transmission_interface/transmission_interface_loader.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include "cdpr_hw/hw/cdpr_hw.hpp"
#include "cdpr_bringup/ros_utilities.hpp"

using namespace cdpr_hw;

bool CdprHW::parseActCoeffs(XmlRpc::XmlRpcValue& act_coeffs)
{
    ROS_ASSERT(act_coeffs.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    try
    {
        for (auto it = act_coeffs.begin(); it != act_coeffs.end(); ++it)
        {
            ActCoeff act_coeff{};

            // All motor
            if (it->second.hasMember("act2pos"))
                act_coeff.act2pos = xmlRpcGetDouble(act_coeffs[it->first], "act2pos", 0.);
            else
                ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated act2pos.");
            if (it->second.hasMember("act2vel"))
                act_coeff.act2vel = xmlRpcGetDouble(act_coeffs[it->first], "act2vel", 0.);
            else
                ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated act2vel.");
            if (it->second.hasMember("act2effort"))
                act_coeff.act2effort = xmlRpcGetDouble(act_coeffs[it->first], "act2effort", 0.);
            else
                ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated act2effort.");
            if (it->second.hasMember("pos2act"))
                act_coeff.pos2act = xmlRpcGetDouble(act_coeffs[it->first], "pos2act", 0.);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated pos2act.");
            if (it->second.hasMember("vel2act"))
                act_coeff.vel2act = xmlRpcGetDouble(act_coeffs[it->first], "vel2act", 0.);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated vel2act.");
            if (it->second.hasMember("effort2act"))
                act_coeff.effort2act = xmlRpcGetDouble(act_coeffs[it->first], "effort2act", 0.0);
            else
                ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated effort2act.");
            if (it->second.hasMember("max_out"))
                act_coeff.max_out = xmlRpcGetDouble(act_coeffs[it->first], "max_out", 0.0);
            else
                ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated max_out.");

            // MIT Cheetah Motor
            if (it->second.hasMember("act2pos_offset"))
                act_coeff.act2pos_offset = xmlRpcGetDouble(act_coeffs[it->first], "act2pos_offset", -12.5);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated act2pos_offset.");
            if (it->second.hasMember("act2vel_offset"))
                act_coeff.act2vel_offset = xmlRpcGetDouble(act_coeffs[it->first], "act2vel_offset", -65.0);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated act2vel_offset.");
            if (it->second.hasMember("act2effort_offset"))
                act_coeff.act2effort_offset = xmlRpcGetDouble(act_coeffs[it->first], "act2effort_offset", -18.0);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated act2effort_offset.");
            if (it->second.hasMember("kp"))
                act_coeff.kp = xmlRpcGetDouble(act_coeffs[it->first], "kp", 0.1);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated kp.");
            if (it->second.hasMember("kw"))
                act_coeff.kw = xmlRpcGetDouble(act_coeffs[it->first], "kw", 0.1);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated kw.");
            if (it->second.hasMember("mode"))
                act_coeff.mode = xmlRpcGetDouble(act_coeffs[it->first], "mode", 0);
            else
                ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated mode.");

            std::string type = it->first;
            if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
                type2act_coeffs_.insert(std::make_pair(type, act_coeff));
            else
                ROS_ERROR_STREAM("Repeat actuator coefficient of type: " << type);
        }
    }
    catch (XmlRpc::XmlRpcException& e)
    {
        ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                         << "configuration: " << e.getMessage() << ".\n"
                         << "Please check the configuration, particularly parameter types.");
        return false;
    }
    return true;
}

bool CdprHW::parseActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh)
{
    ROS_ASSERT(act_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    try
    {
        for (auto it = act_datas.begin(); it != act_datas.end(); ++it)
        {
            if (!it->second.hasMember("id"))
            {
                ROS_ERROR_STREAM("Actuator " << it->first << " has no associated ID.");
                continue;
            }
            else if (!it->second.hasMember("type"))
            {
                ROS_ERROR_STREAM("Actuator " << it->first << " has no associated type.");
                continue;
            }
            else if (!it->second.hasMember("comm_protocol"))
            {
                ROS_ERROR_STREAM("Actuator " << it->first << " has no associated conmmunication protocol.");
                continue;
            }
            else
            {
                if (it->second["comm_protocol"] == "CAN")
                {
                    if (!(it->second.hasMember("can_ind") && it->second.hasMember("dev_ind")))
                    {
                        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated can bus.");
                        continue;
                    }
                }
                else if (it->second["comm_protocol"] == "RS485")
                {
                    if (!it->second.hasMember("usb_port"))
                    {
                        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated usb port.");
                        continue;
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Actuator " << it->first << " has undefined conmmunication protocol.");
                    continue;
                }
            }

            std::string type = it->second["type"], comm_protocol = it->second["comm_protocol"];
            int id = static_cast<int>(it->second["id"]);

            // check define of act_coeffs
            if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
            {
                ROS_ERROR_STREAM("Actuator type " << type << " has no associated coefficient.");
                return false;
            }

            int dev_ind = 0, can_ind = 0, port_num = 0;
            std::vector<int> comm{};
            if (comm_protocol == "CAN")
            {
                ROS_ASSERT(it->second["dev_ind"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(it->second["can_ind"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                dev_ind = static_cast<int>(it->second["dev_ind"]);
                can_ind = static_cast<int>(it->second["can_ind"]);
                comm = { dev_ind, can_ind };
            }
            else if (comm_protocol == "RS485")
            {
                ROS_ASSERT(it->second["usb_port"].getType() == XmlRpc::XmlRpcValue::TypeString);
                std::string usb_port = it->second["usb_port"];
                port_num = usb_port.back() - '0';
                comm = { port_num };
            }

            if (comm_id2act_data_.find(comm) == comm_id2act_data_.end())
            {
                comm_id2act_data_.insert(std::make_pair(comm, std::unordered_map<int, ActData>()));
            }
            if (!(comm_id2act_data_[comm].find(id) == comm_id2act_data_[comm].end()))
            {
                ROS_ERROR_STREAM("Repeat actuator on comm_protocol " << comm_protocol << " device index " << comm[0]
                                                                     << " and ID " << id);
                return false;
            }
            else
            {
                comm_id2act_data_[comm].insert(std::make_pair(id, ActData{ .name = it->first,
                                                                           .type = type,
                                                                           .stamp = ros::Time::now(),
                                                                           .halted = false,
                                                                           .seq = 0,
                                                                           .temp = 0,
                                                                           .frequency = 0,
                                                                           .pos = 0,
                                                                           .vel = 0,
                                                                           .effort = 0,
                                                                           .cmd_pos = 0,
                                                                           .cmd_vel = 0,
                                                                           .cmd_effort = 0,
                                                                           .exe_effort = 0,
                                                                           .zero_point = 0 }));
            }

            // for ros_control interface
            hardware_interface::ActuatorStateHandle act_state(
                comm_id2act_data_[comm][id].name, &comm_id2act_data_[comm][id].pos, &comm_id2act_data_[comm][id].vel,
                &comm_id2act_data_[comm][id].effort);
            act_state_interface_.registerHandle(act_state);
            effort_act_interface_.registerHandle(
                hardware_interface::ActuatorHandle(act_state, &comm_id2act_data_[comm][id].exe_effort));
        }

        registerInterface(&act_state_interface_);
        registerInterface(&effort_act_interface_);
        is_actuator_specified_ = true;
    }
    catch (XmlRpc::XmlRpcException& e)
    {
        ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                         << "configuration: " << e.getMessage() << ".\n"
                         << "Please check the configuration, particularly parameter types.");
        return false;
    }
    return true;
}

bool CdprHW::loadURDF(const ros::NodeHandle& nh, std::string param_name)
{
    urdf_model_ = new urdf::Model();

    // search and wait for robot_description on param server
    while (urdf_string_.empty() && ros::ok())
    {
        std::string search_param_name;
        if (nh.searchParam(param_name, search_param_name))
        {
            ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: "
                                             << nh.getNamespace() << search_param_name);
            nh.getParam(search_param_name, urdf_string_);
        }
        else
        {
            ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: "
                                             << nh.getNamespace() << param_name);
            nh.getParam(param_name, urdf_string_);
        }

        usleep(100000);
    }

    if (!urdf_model_->initString(urdf_string_))
    {
        ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        return false;
    }
    else
        ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");

    return true;
}

bool CdprHW::setupTransmission(ros::NodeHandle& root_nh)
{
    if (!is_actuator_specified_)
        return true;
    try
    {
        transmission_loader_ =
            std::make_unique<transmission_interface::TransmissionInterfaceLoader>(this, &robot_transmissions_);
    }
    catch (const std::invalid_argument& ex)
    {
        ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
        return false;
    }
    catch (const pluginlib::LibraryLoadException& ex)
    {
        ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
        return false;
    }
    catch (...)
    {
        ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
        return false;
    }

    // Perform transmission loading
    if (!transmission_loader_->load(urdf_string_))
    {
        return false;
    }
    act_to_jnt_state_ = robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>();
    jnt_to_act_effort_ = robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>();

    auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
    std::vector<std::string> names = effort_joint_interface->getNames();
    for (const auto& name : names)
        effort_joint_handles_.push_back(effort_joint_interface->getHandle(name));

    return true;
}

bool CdprHW::setupJointLimit(ros::NodeHandle& root_nh)
{
    if (!is_actuator_specified_)
        return true;

    joint_limits_interface::JointLimits joint_limits;     // Position
    joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position

    for (const auto& joint_handle : effort_joint_handles_)
    {
        bool has_joint_limits = false;
        bool has_soft_limits = false;
        std::string name = joint_handle.getName();

        // Check if URDF is loaded
        if (urdf_model_ == nullptr)
        {
            ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to get joint limits");
            return false;
        }
        // Check if joints match
        urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_handle.getName());
        if (urdf_joint == nullptr)
        {
            ROS_WARN_STREAM_NAMED(name_, "URDF joint " << name << " not found ");
            return false;
        }

        // Get limits from URDF
        if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
        {
            has_joint_limits = true;
            ROS_DEBUG_STREAM_NAMED(name_, "Joint " << name << " has URDF position limits [" << joint_limits.min_position
                                                   << ", " << joint_limits.max_position << "]");
            if (joint_limits.has_velocity_limits)
                ROS_DEBUG_STREAM_NAMED(name_, "Joint " << name << " has URDF velocity limit ["
                                                       << joint_limits.max_velocity << "]");
        }
        else if (urdf_joint->type != urdf::Joint::CONTINUOUS)
            ROS_DEBUG_STREAM("Joint " << name << " does not have a URDF limit.");
        // Get limits from ROS param
        if (use_rosparam_joint_limits_)
        {
            if (joint_limits_interface::getJointLimits(joint_handle.getName(), root_nh, joint_limits))
            {
                has_joint_limits = true;
                ROS_DEBUG_STREAM_NAMED(name_, "Joint " << name << " has rosparam position limits ["
                                                       << joint_limits.min_position << ", " << joint_limits.max_position
                                                       << "]");
                if (joint_limits.has_velocity_limits)
                    ROS_DEBUG_STREAM_NAMED(name_, "Joint " << name << " has rosparam velocity limit ["
                                                           << joint_limits.max_velocity << "]");
            }
        }  // the else debug message provided internally by joint_limits_interface

        // Get soft limits from URDF
        if (use_soft_limits_if_available_)
        {
            if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
            {
                has_soft_limits = true;
                ROS_DEBUG_STREAM_NAMED(name_, "Joint " << name << " has soft joint limits.");
            }
            else
                ROS_DEBUG_STREAM_NAMED(name_, "Joint " << name
                                                       << " does not have soft joint "
                                                          "limits");
        }

        // Slightly reduce the joint limits to prevent floating point errors
        if (joint_limits.has_position_limits)
        {
            joint_limits.min_position += std::numeric_limits<double>::epsilon();
            joint_limits.max_position -= std::numeric_limits<double>::epsilon();
        }

        if (has_soft_limits)
        {  // Use soft limits
            ROS_DEBUG_STREAM("Using soft saturation limits");
            effort_jnt_soft_limits_interface_.registerHandle(
                joint_limits_interface::EffortJointSoftLimitsHandle(joint_handle, joint_limits, soft_limits));
        }
        else if (has_joint_limits)
        {  // Use saturation limits
            ROS_DEBUG_STREAM("Using saturation limits (not soft limits)");
            effort_jnt_saturation_interface_.registerHandle(
                joint_limits_interface::EffortJointSaturationHandle(joint_handle, joint_limits));
        }
    }
    return true;
}