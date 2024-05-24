#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace global_traj_ctrl {
class GlobalTrajCtrl {
   public:
    GlobalTrajCtrl(ros::NodeHandle& nh)
        : nh_(nh), is_cable_ready_(true), is_archor_ready_(true), is_a1_ready_(false), is_go_ready_(false) {
        pubs_.emplace_back(nh.advertise<std_msgs::Float64MultiArray>("/archor_coor_z", 10));
        pubs_.emplace_back(nh.advertise<std_msgs::Float64MultiArray>("/cable_length", 10));
        pubs_.emplace_back(nh.advertise<std_msgs::Float64MultiArray>("/traj_steer_angle", 10));
        pubs_.emplace_back(nh.advertise<std_msgs::Float64MultiArray>("/traj_roll_vel", 10));
        pubs_.emplace_back(nh.advertise<std_msgs::Bool>("/start_traj_tracking", 10));

        subs_.emplace_back(nh.subscribe("/maxon_re35/ready_state", 10, &GlobalTrajCtrl::cableResetCallback, this));
        subs_.emplace_back(nh.subscribe("/movable_archor/ready_state", 10, &GlobalTrajCtrl::archorResetCallback, this));
        subs_.emplace_back(nh.subscribe("/go/ready_state", 10, &GlobalTrajCtrl::goResetCallback, this));
        subs_.emplace_back(nh.subscribe("/a1/ready_state", 10, &GlobalTrajCtrl::a1ResetCallback, this));
    }

    void readPublishTraj() {
        // read cable、archor trajectory
        std::ifstream file1(ros::package::getPath("cdpr_control") + "/csv/global/updown.csv", std::ios::in);

        // read chassis trajectory
        // std::ifstream file2(ros::package::getPath("cdpr_control") + "/csv/global/no_obs.csv", std::ios::in);
        std::ifstream file2(ros::package::getPath("cdpr_control") + "/csv/global/obs.csv", std::ios::in);

        if (!file1.is_open() || !file2.is_open()) ROS_ERROR("Failed to open .csv file");

        std::string line, data;
        std::istringstream s;

        // read data
        while (ros::ok() && getline(file1, line)) {
            s.clear();
            // used for breaking words
            s.str(line);
            int cnt = 0;

            // read every column data of a row and store it in a string variable 'data'
            while (std::getline(s, data, ',')) {
                if (cnt >= 0 && cnt <= 3)
                    // add all the column data of a row to member variables
                    archor_coor_z_.data.emplace_back(stod(data));
                else if (cnt >= 4 && cnt <= 7)
                    cable_length_.data.emplace_back(stod(data));

                ++cnt;
            }
        }
        file1.close();

        while (ros::ok() && getline(file2, line)) {
            s.clear();
            // used for breaking words
            s.str(line);
            int cnt = 0;

            // read every column data of a row and store it in a string variable 'data'
            while (std::getline(s, data, ',')) {
                if (cnt >= 0 && cnt <= 3)
                    // add all the column data of a row to member variables
                    steer_angle_.data.emplace_back(stod(data));
                else if (cnt >= 4 && cnt <= 7)
                    roll_vel_.data.emplace_back(stod(data));

                ++cnt;
            }
        }
        file2.close();

        // wait for motors reset
        while (ros::ok() && !(is_cable_ready_ && is_archor_ready_ && is_a1_ready_ && is_go_ready_)) ros::spinOnce();

        pubs_[0].publish(archor_coor_z_);
        pubs_[1].publish(cable_length_);
        pubs_[2].publish(steer_angle_);
        pubs_[3].publish(roll_vel_);

        std_msgs::Bool start_traj_tracking;
        start_traj_tracking.data = true;
        pubs_[4].publish(start_traj_tracking);

        std::string is_stall{};
        while (ros::ok()) {
            getline(std::cin, is_stall);
            if (is_stall == "p" || is_stall == "P") {
                start_traj_tracking.data = false;
                pubs_[4].publish(start_traj_tracking);
                break;
            }
        }
    }

   private:
    void cableResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_cable_ready_ = is_ready->data; }
    void archorResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_archor_ready_ = is_ready->data; }
    void a1ResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_a1_ready_ = is_ready->data; }
    void goResetCallback(const std_msgs::Bool::ConstPtr& is_ready) { is_go_ready_ = is_ready->data; }

    std_msgs::Float64MultiArray archor_coor_z_{}, cable_length_{}, steer_angle_{}, roll_vel_{};
    bool is_cable_ready_, is_archor_ready_, is_a1_ready_, is_go_ready_;

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

    return 0;
}
