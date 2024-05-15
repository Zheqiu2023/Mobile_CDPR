#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace local_traj_ctrl
{
class LocalTrajCtrl
{
  public:
    LocalTrajCtrl(ros::NodeHandle& nh) : nh_(nh), is_cable_ready_(false), is_archor_ready_(false)
    {
        pubs_.emplace_back(nh.advertise<std_msgs::Float64MultiArray>("/archor_coor_z", 1));
        pubs_.emplace_back(nh.advertise<std_msgs::Float64MultiArray>("/cable_length", 1));
        pubs_.emplace_back(nh.advertise<std_msgs::Bool>("/start_traj_tracking", 1));
        subs_.emplace_back(nh.subscribe("/maxon_re35/ready_state", 10, &LocalTrajCtrl::cableResetCallback, this));
        subs_.emplace_back(nh.subscribe("/movable_archor/ready_state", 10, &LocalTrajCtrl::archorResetCallback, this));
    }

    void readPublishTraj()
    {
        std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/local/updown.csv", std::ios::in);
        // std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/local/line.csv", std::ios::in);
        // std::ifstream f_in(ros::package::getPath("cdpr_control") + "/csv/local/circle.csv", std::ios::in);
        if (!f_in.is_open())
            ROS_ERROR("Failed to open .csv file");

        std::string line, data;
        std::istringstream s;

        // read trajectory
        while (ros::ok() && getline(f_in, line))
        {
            s.clear();
            // used for breaking words
            s.str(line);
            int cnt = 0;

            // read every column data of a row and store it in a string variable 'data'
            while (std::getline(s, data, ','))
            {
                if (cnt >= 0 && cnt <= 3)
                    // add all the column data of a row to member variables
                    archor_coor_z_.data.emplace_back(stod(data));
                else if (cnt >= 4 && cnt <= 7)
                    cable_length_.data.emplace_back(stod(data));

                ++cnt;
            }
        }
        f_in.close();

        // wait for motors reset
        while (ros::ok() && !(is_cable_ready_ && is_archor_ready_))
            ros::spinOnce();

        pubs_[0].publish(archor_coor_z_);
        pubs_[1].publish(cable_length_);

        std_msgs::Bool start_traj_tracking;
        start_traj_tracking.data = true;
        pubs_[3].publish(start_traj_tracking);
    }

  private:
    void cableResetCallback(const std_msgs::Bool::ConstPtr& is_ready)
    {
        is_cable_ready_ = is_ready->data;
    }
    void archorResetCallback(const std_msgs::Bool::ConstPtr& is_ready)
    {
        is_archor_ready_ = is_ready->data;
    }

    std_msgs::Float64MultiArray archor_coor_z_{}, cable_length_{};
    bool is_cable_ready_, is_archor_ready_;

    ros::NodeHandle nh_;
    ros::V_Publisher pubs_;
    ros::V_Subscriber subs_;
};
}  // namespace local_traj_ctrl

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_traj_ctrl");
    ros::NodeHandle nh("~");

    local_traj_ctrl::LocalTrajCtrl local_traj_ctrl(nh);
    local_traj_ctrl.readPublishTraj();

    return 0;
}
