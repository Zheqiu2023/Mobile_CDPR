#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "cdpr_bringup/TrajCmd.h"

namespace cable_archor_ctrl
{
class CableArchorCtrl
{
  public:
    CableArchorCtrl(ros::NodeHandle& nh) : nh_(nh), is_re35_reset_(false), is_stepper57_reset_(true)
    {
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/stepper_57/archor_coor_z", 100));
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/maxon_re35/cable_length", 100));
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/maxon_re35/cable_force", 100));
        subs_.emplace_back(nh.subscribe("/maxon_re35/reset_flag", 10, &CableArchorCtrl::re35ResetCallback, this));
        subs_.emplace_back(nh.subscribe("/stepper_57/reset_flag", 10, &CableArchorCtrl::stepper57ResetCallback, this));

        archor_coor_z_.is_traj_end = false;
        cable_length_.is_traj_end = false;
        cable_force_.is_traj_end = false;
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
        while (!is_re35_reset_ || !is_stepper57_reset_)
            ros::spinOnce();
        // read data
        while (getline(f_in, line))
        {
            s.clear();
            archor_coor_z_.target.clear();
            cable_length_.target.clear();
            cable_force_.target.clear();
            // used for breaking words
            s.str(line);
            int cnt = 0;

            // read every column data of a row and store it in a string variable 'data'
            while (std::getline(s, data, ','))
            {
                if (cnt >= 0 && cnt <= 3)
                {
                    // add all the column data of a row to member variables
                    archor_coor_z_.target.emplace_back(stod(data));
                }
                else if (cnt >= 4 && cnt <= 7)
                {
                    cable_length_.target.emplace_back(stod(data));
                }
                else if (cnt >= 8 && cnt <= 11)
                {
                    cable_force_.target.emplace_back(stod(data));
                }
                ++cnt;
            }
            pubs_[0].publish(archor_coor_z_);
            pubs_[1].publish(cable_length_);
            // pubs_[2].publish(cable_force_);
            ros::Duration(0.3).sleep();
        }
        // End of trajectory, all motors move back to zero position then stop
        archor_coor_z_.is_traj_end = true;
        cable_length_.is_traj_end = true;
        cable_force_.is_traj_end = true;
        std::fill(archor_coor_z_.target.begin(), archor_coor_z_.target.end(), 0.0);
        std::fill(cable_length_.target.begin(), cable_length_.target.end(), 0.0);
        std::fill(cable_force_.target.begin(), cable_force_.target.end(), 0.0);
        pubs_[0].publish(archor_coor_z_);
        pubs_[1].publish(cable_length_);
        // pubs_[2].publish(cable_force_);

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

    cdpr_bringup::TrajCmd archor_coor_z_{}, cable_length_{}, cable_force_{};
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

    ros::spin();
    return 0;
}
