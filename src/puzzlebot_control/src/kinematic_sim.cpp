#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class KineamticModel{
public:
    KineamticModel() : 
        nh{},
        leftPub(nh.advertise<std_msgs::Float64>("wl", 10)),
        rightPub(nh.advertise<std_msgs::Float64>("wr", 10))
    {
        cmd_vel.linear.x = 0; // u
        cmd_vel.angular.z = 0; // r

        velSub = nh.subscribe("cmd_vel", 1, &KineamticModel::vel_cb, this);
        timer = nh.createTimer(ros::Duration(dt), &KineamticModel::update, this);
        timer.start();
    }
protected:
    void vel_cb(const geometry_msgs::Twist &msg) {
        cmd_vel = msg;
    }

    void update(const ros::TimerEvent &e) {
        double u = cmd_vel.linear.x;
        double omega = cmd_vel.angular.z;

        // Publish wheel velocities from cmd_vel
        std_msgs::Float64 leftMsg, rightMsg;
        leftMsg.data = (u - (l * omega) / 2.0) * (1.0 / r);
        rightMsg.data = (u + (l * omega) / 2.0) * (1.0 / r);

        leftPub.publish(leftMsg);
        rightPub.publish(rightMsg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher leftPub, rightPub;
    ros::Subscriber velSub;
    ros::Timer timer;

    geometry_msgs::Twist cmd_vel;

    // Simulation constants
    static constexpr double dt{0.01};
    static constexpr double r{0.05}; // wheel radius
    static constexpr double l{0.19}; // wheel base
};

int main(int argc, char **argv){
    ros::init(argc, argv, "puzzlebot_kinematic_model");
    KineamticModel node;
    ros::spin();
    return 0;    
}