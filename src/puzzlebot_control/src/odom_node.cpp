#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "puzzlebot_control/ResetPosition.h"

class OdometryNode{
public:
    OdometryNode() : 
        nh{},
        odomPub(nh.advertise<nav_msgs::Odometry>("odom", 10))
    {
        currentOdom.pose.pose.position.x = 0;
        currentOdom.pose.pose.position.y = 0;
        currentOdom.pose.pose.position.z = 0;

        leftSub = nh.subscribe<std_msgs::Float64, const std_msgs::Float64&>("wl", 1, [&](const std_msgs::Float64 &msg){
            this->wl = msg.data;
        });
        rightSub = nh.subscribe<std_msgs::Float64, const std_msgs::Float64&>("wr", 1, [&](const std_msgs::Float64 &msg){
            this->wr = msg.data;
        });

        resetService = nh.advertiseService("reset_position", &OdometryNode::reset_position, this);

        timer = nh.createTimer(ros::Duration(dt), &OdometryNode::update, this);
        timer.start();
    }
protected:
    void vel_cb(const geometry_msgs::Twist &msg) {
        cmd_vel = msg;
    }

    void update(const ros::TimerEvent &e) {
        double v = r * ((wr + wl) / 2.0); // X Vel
        double omega = r * ((wr - wl) / l); // Angular vel

        currentOdom.pose.pose.position.x += std::cos(yaw) * v * dt;
        currentOdom.pose.pose.position.y += std::sin(yaw) * v * dt;
        yaw += omega * dt;

        // Publish odom.
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        currentOdom.pose.pose.orientation = tf2::toMsg(quat);

        currentOdom.twist.twist.linear.x = v;
        currentOdom.twist.twist.angular.z = omega;

        currentOdom.header.stamp = ros::Time::now();
        odomPub.publish(currentOdom);

        // Publish broadcast.
        geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = currentOdom.pose.pose.position.x;
        transformStamped.transform.translation.y = currentOdom.pose.pose.position.y;
        transformStamped.transform.translation.z = currentOdom.pose.pose.position.z;
        transformStamped.transform.rotation = tf2::toMsg(quat);
        br.sendTransform(transformStamped);
    }

    bool reset_position(
        puzzlebot_control::ResetPosition::Request &req,
        puzzlebot_control::ResetPosition::Response &res){
        this->currentOdom.pose.pose = req.initialPose;
        ROS_INFO("Resetting position to x=%f, y=%f", req.initialPose.position.x , req.initialPose.position.y);
        return true;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher odomPub;
    ros::Subscriber leftSub, rightSub;
    ros::Timer timer;

    nav_msgs::Odometry currentOdom;
    double yaw{0};
    geometry_msgs::Twist cmd_vel;

    ros::ServiceServer resetService;
    tf2_ros::TransformBroadcaster br;

    // Simulation constants
    static constexpr double dt{0.01};
    static constexpr double r{0.05}; // wheel radius
    static constexpr double l{0.19}; // wheel base

    double wl{0}, wr{0};
};

int main(int argc, char **argv){
    ros::init(argc, argv, "puzzlebot_odometry_node");
    OdometryNode node;
    ros::spin();
    return 0;    
}