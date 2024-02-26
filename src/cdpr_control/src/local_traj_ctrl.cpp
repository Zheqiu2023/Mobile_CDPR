#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cdpr_bringup/TrajCmd.h"

namespace local_traj_ctrl {
class LocalTrajCtrl {
   public:
    LocalTrajCtrl(ros::NodeHandle& nh) : nh_(nh), is_cable_ready_(false), is_archor_ready_(false) {
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/archor_coor_z", 1));
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/cable_length", 1));
        subs_.emplace_back(nh.subscribe("/maxon_re35/ready_state", 10, &LocalTrajCtrl::cableResetCallback, this));
        subs_.emplace_back(nh.subscribe("/movable_archor/ready_state", 10, &LocalTrajCtrl::archorResetCallback, this));

        archor_coor_z_.is_traj_end = false;
        cable_length_.is_traj_end = false;
    }

    void readPublishTraj() {
        std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/updown.csv", std::ios::in);
        // std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/line.csv", std::ios::in);
        // std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/circle.csv", std::ios::in);
        // std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/hit.csv", std::ios::in);
        if (!f_in.is_open()) ROS_ERROR("Failed to open .csv file");

        std::string line, data;
        std::istringstream s;
        double traj_period = nh_.param("traj_period", 0.3);

        // wait for motors reset
        while (ros::ok() && !(is_cable_ready_ && is_archor_ready_)) ros::spinOnce();
        // read data
        while (ros::ok() && getline(f_in, line)) {
            s.clear();
            archor_coor_z_.target.clear();
            cable_length_.target.clear();
            // used for breaking words
            s.str(line);
            int cnt = 0;

            // read every column data of a row and store it in a string variable 'data'
            while (std::getline(s, data, ',')) {
                if (cnt >= 0 && cnt <= 3) {
                    // add all the column data of a row to member variables
                    archor_coor_z_.target.emplace_back(stod(data));
                } else if (cnt >= 4 && cnt <= 7) {
                    cable_length_.target.emplace_back(stod(data));
                }
                ++cnt;
            }
            pubs_[0].publish(archor_coor_z_);
            pubs_[1].publish(cable_length_);
            ROS_INFO("Publish time is: %16f", ros::Time::now().toSec());  //打印，%16f表示的是16位宽度的float类型的数字;

            ros::Duration(traj_period).sleep();
        }
        // End of trajectory, all motors move back to zero position then stop
        archor_coor_z_.is_traj_end = true;
        cable_length_.is_traj_end = true;
        std::fill(archor_coor_z_.target.begin(), archor_coor_z_.target.end(), 0.0);
        std::fill(cable_length_.target.begin(), cable_length_.target.end(), 0.0);
        pubs_[0].publish(archor_coor_z_);
        pubs_[1].publish(cable_length_);

        f_in.close();
    }

   private:
    void cableResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_cable_ready_ = is_ready->data; }
    void archorResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_archor_ready_ = is_ready->data; }

    cdpr_bringup::TrajCmd archor_coor_z_{}, cable_length_{};
    bool is_cable_ready_, is_archor_ready_;

    ros::NodeHandle nh_;
    ros::V_Publisher pubs_;
    ros::V_Subscriber subs_;
};
}  // namespace local_traj_ctrl

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "local_traj_ctrl");
    ros::NodeHandle nh("~");

    local_traj_ctrl::LocalTrajCtrl local_traj_ctrl(nh);
    local_traj_ctrl.readPublishTraj();

    return 0;
}
