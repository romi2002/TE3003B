#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node{
    public:
    OdometryNode()
    : Node("odometry_node")
    {
        auto odomPub = this->create_publisher<nav_msgs::msg::Odometry>("odometry_topic", 10);
        auto leftSub = this->create_subscription<std_msgs::msg::Float64>("wl", 1, [this](const std_msgs::msg::Float64 &msg){
            this->wl = msg.data;
        });
        auto rightSub = this->create_subscription<std_msgs::msg::Float64>("wr", 1, [this](const std_msgs::msg::Float64 &msg){
            this->wr = msg.data;
        });

        auto timer = this->create_wall_timer(10ms, std::bind(&OdometryNode::update, this));

        currentOdom.pose.pose.position.x = 0;
        currentOdom.pose.pose.position.y = 0;
        currentOdom.pose.pose.position.z = 0;
    }

    protected:
        void update() {
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

        currentOdom.header.stamp = this->get_clock()->now();
        odomPub->publish(currentOdom);

        // Publish broadcast.
        geometry_msgs::msg::TransformStamped transformStamped;
  
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = currentOdom.pose.pose.position.x;
        transformStamped.transform.translation.y = currentOdom.pose.pose.position.y;
        transformStamped.transform.translation.z = currentOdom.pose.pose.position.z;
        transformStamped.transform.rotation = tf2::toMsg(quat);
        br->sendTransform(transformStamped);
    }

    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftSub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightSub;
        rclcpp::TimerBase::SharedPtr timer;

        nav_msgs::msg::Odometry currentOdom;
        double yaw{0};
        geometry_msgs::msg::Twist cmd_vel; 
        std::unique_ptr<tf2_ros::TransformBroadcaster> br;

        // Simulation constants
        static constexpr double dt{0.01};
        static constexpr double r{0.05}; // wheel radius
        static constexpr double l{0.19}; // wheel base

        double wl{0}, wr{0};
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;  
}