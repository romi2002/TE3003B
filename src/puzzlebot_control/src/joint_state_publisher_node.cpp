#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class JointStatePubNode{
public:
    JointStatePubNode() : 
        nh{},
        jointPub(nh.advertise<sensor_msgs::JointState>("joint_states", 10))
    {
        leftSub = nh.subscribe<std_msgs::Float64, const std_msgs::Float64&>("wl", 1, [&](const std_msgs::Float64 &msg){
            this->wl = msg.data;
        });
        rightSub = nh.subscribe<std_msgs::Float64, const std_msgs::Float64&>("wr", 1, [&](const std_msgs::Float64 &msg){
            this->wr = msg.data;
        });

        timer = nh.createTimer(ros::Duration(dt), &JointStatePubNode::update, this);
        timer.start();
    }
protected:
    void update(const ros::TimerEvent &e) {
        left_wheel += wl * dt;
        right_wheel += wr * dt;

        sensor_msgs::JointState joint_msg;
        joint_msg.name = {"leftWheel", "rightWheel"};
        joint_msg.position = {left_wheel, right_wheel};

        joint_msg.header.stamp = ros::Time::now();
        jointPub.publish(joint_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher jointPub;
    ros::Subscriber leftSub, rightSub;
    ros::Timer timer;

    double wl{0}, wr{0};
    double left_wheel{0}, right_wheel{0};

    // Simulation constants
    static constexpr double dt{0.01};
    static constexpr double r{0.05}; // wheel radius
    static constexpr double l{0.19}; // wheel base
};

int main(int argc, char **argv){
    ros::init(argc, argv, "puzzlebot_joint_state_node");
    JointStatePubNode node;
    ros::spin();
    return 0;    
}