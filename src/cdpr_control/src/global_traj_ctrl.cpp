#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cdpr_bringup/TrajCmd.h"
#include "cdpr_bringup/math_utilities.hpp"

namespace global_traj_ctrl {
class GlobalTrajCtrl {
   public:
    GlobalTrajCtrl(ros::NodeHandle& nh)
        : nh_(nh), is_re35_ready_(false), is_stepper57_ready_(false), is_a1_ready_(false), is_go_ready_(false) {
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/archor_coor_z", 100));
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/cable_length", 100));
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/go/target_angle", 100));
        pubs_.emplace_back(nh.advertise<cdpr_bringup::TrajCmd>("/a1/target_pos", 100));
        subs_.emplace_back(nh.subscribe("/maxon_re35/ready_state", 10, &GlobalTrajCtrl::re35ResetCallback, this));
        subs_.emplace_back(nh.subscribe("/stepper_57/ready_state", 10, &GlobalTrajCtrl::stepper57ResetCallback, this));
        subs_.emplace_back(nh.subscribe("/go/ready_state", 10, &GlobalTrajCtrl::goResetCallback, this));
        subs_.emplace_back(nh.subscribe("/a1/ready_state", 10, &GlobalTrajCtrl::a1ResetCallback, this));

        archor_coor_z_.is_traj_end = false;
        cable_length_.is_traj_end = false;
        target_angle_.is_traj_end = false;
        target_pos_.is_traj_end = false;
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
        while (ros::ok() && !(is_re35_ready_ && is_stepper57_ready_ && is_a1_ready_ && is_go_ready_)) ros::spinOnce();
        // read data
        while (ros::ok() && getline(f_in, line)) {
            s.clear();
            archor_coor_z_.target.clear();
            cable_length_.target.clear();
            target_angle_.target.clear();
            target_pos_.target.clear();

            // used for breaking words
            s.str(line);
            int cnt = 0;
            double x = 0.0, y = 0.0, last_x = 0.0, last_y = 0.0;

            // read every column data of a row and store it in a string variable 'data'
            while (std::getline(s, data, ',')) {
                if (cnt >= 0 && cnt <= 3) {
                    // add all the column data of a row to member variables
                    archor_coor_z_.target.emplace_back(stod(data));
                } else if (cnt >= 4 && cnt <= 7) {
                    cable_length_.target.emplace_back(stod(data));
                } else if (cnt == 8) {
                    last_x = x;
                    x = stod(data);
                } else if (cnt == 9) {
                    last_y = y;
                    y = stod(data);
                }
                ++cnt;
            }
            std::fill(target_angle_.target.begin(), target_angle_.target.end(), std::atan((y - last_y) / (x - last_x)));
            std::fill(target_pos_.target.begin(), target_pos_.target.end(),
                      std::sqrt(square(x - last_x) + square(y - last_y)));

            pubs_[0].publish(archor_coor_z_);
            pubs_[1].publish(cable_length_);
            pubs_[2].publish(target_angle_);
            pubs_[3].publish(target_pos_);
            ros::Duration(traj_period).sleep();
        }
        // End of trajectory, all motors move back to zero position then stop
        ros::Duration(2.0).sleep();
        archor_coor_z_.is_traj_end = true;
        cable_length_.is_traj_end = true;
        target_angle_.is_traj_end = true;
        target_pos_.is_traj_end = true;
        std::fill(archor_coor_z_.target.begin(), archor_coor_z_.target.end(), 0.0);
        std::fill(cable_length_.target.begin(), cable_length_.target.end(), 0.0);
        pubs_[0].publish(archor_coor_z_);
        pubs_[1].publish(cable_length_);
        pubs_[2].publish(target_angle_);
        pubs_[3].publish(target_pos_);

        f_in.close();
    }

   private:
    void re35ResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_re35_ready_ = is_ready->data; }
    void stepper57ResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_stepper57_ready_ = is_ready->data; }
    void a1ResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_a1_ready_ = is_ready->data; }
    void goResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_go_ready_ = is_ready->data; }

    cdpr_bringup::TrajCmd archor_coor_z_{}, cable_length_{}, target_angle_{}, target_pos_{};
    bool is_re35_ready_, is_stepper57_ready_, is_a1_ready_, is_go_ready_;

    ros::NodeHandle nh_;
    ros::V_Publisher pubs_;
    ros::V_Subscriber subs_;
};
}  // namespace global_traj_ctrl

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "global_traj_ctrl");
    ros::NodeHandle nh("~");

    global_traj_ctrl::GlobalTrajCtrl global_traj_ctrl(nh);
    global_traj_ctrl.readPublishTraj();

    ros::waitForShutdown();
    return 0;
}
