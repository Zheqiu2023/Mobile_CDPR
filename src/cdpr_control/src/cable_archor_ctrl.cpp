#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <std_msgs/Bool.h>

namespace cable_archor_ctrl
{
class CableArchorCtrl
{
  public:
    CableArchorCtrl(ros::NodeHandle& nh) : nh_(nh)
    {
        pubs_.emplace_back(nh.advertise<std_msgs::Float32MultiArray>("/archor_coor_z", 100));
        pubs_.emplace_back(nh.advertise<std_msgs::Float32MultiArray>("/cable_length", 100));
        pubs_.emplace_back(nh.advertise<std_msgs::Float32MultiArray>("/cable_force", 100));
        subs_.emplace_back(nh.subscribe("/motor_re35/reset_flag", 10, &CableArchorCtrl::re35ResetCallback, this));
        subs_.emplace_back(nh.subscribe("/motor_57/reset_flag", 10, &CableArchorCtrl::stepper57ResetCallback, this));
    }

    void readPublishTraj()
    {
        std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/test.csv", std::ios::in);
        if (!f_in.is_open())
            ROS_ERROR("Failed to open .csv file");

        std::string line, data;
        std::istringstream s;
        // read title
        getline(f_in, line);
        // wait for motors reset to complete
        while (ros::ok() && !(is_re35_reset_ && is_stepper57_reset_))
            ros::spinOnce();
        // read data
        while (getline(f_in, line))
        {
            s.clear();
            archor_coor_z_.data.clear();
            cable_length_.data.clear();
            cable_force_.data.clear();
            // used for breaking words
            s.str(line);
            int cnt = 0;

            // read every column data of a row and store it in a string variable 'data'
            while (std::getline(s, data, ','))
            {
                if (cnt >= 0 && cnt <= 3)
                {
                    // add all the column data of a row to member variables
                    archor_coor_z_.data.emplace_back(stof(data));
                }
                else if (cnt >= 4 && cnt <= 7)
                {
                    cable_length_.data.emplace_back(stof(data));
                }
                else if (cnt >= 8 && cnt <= 11)
                {
                    cable_force_.data.emplace_back(stof(data));
                }
                ++cnt;
            }
            pubs_[0].publish(archor_coor_z_);
            pubs_[1].publish(cable_length_);
            pubs_[2].publish(cable_force_);
            ros::Duration(0.5).sleep();
        }

        f_in.close();
    }

  private:
    void re35ResetCallback(const std_msgs::Bool::ConstPtr& is_reset)
    {
        is_re35_reset_ = is_reset->data;
    }
    void stepper57ResetCallback(const std_msgs::Bool::ConstPtr& is_reset)
    {
        is_stepper57_reset_ = is_reset->data;
    }

    std_msgs::Float32MultiArray archor_coor_z_{}, cable_length_{}, cable_force_{};
    bool is_re35_reset_, is_stepper57_reset_;

    ros::NodeHandle nh_;
    ros::V_Publisher pubs_;
    ros::V_Subscriber subs_;
};
}  // namespace cable_archor_ctrl

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cable_archor_ctrl");
    ros::NodeHandle nh("~");

    cable_archor_ctrl::CableArchorCtrl ctrl(nh);
    ctrl.readPublishTraj();

    return 0;
}
