#include <algorithm>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/PoseStamped.hpp"
#include "geometry_msgs/msg/Pose.hpp"
#include "geometry_msgs/msg/Twist.hpp"
#include "std_msgs/msg/Float64.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class KinematicModel : public rclcpp::Node 
{
    public:
        KinematicModel() : Node("puzzlebot_kinematic_model"),
            leftPub(this->create_publisher<std_msgs::msg::Float64>("wl", 10)),
            rightPub(this->create_publisher<std_msgs::msg::Float64>("wr", 10))
        {
            cmd_vel.linear.x = 0; // u
            cmd_vel.angular.z = 0; // r

            velSub = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 1, std::bind(&KinematicModel::vel_cb, this, std::placeholders::_1));
            
            timer = this->create_wall_timer(std::chrono::duration<double>(dt),
                std::bind(&KinematicModel::update, this));
        }
    protected:
        void vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
            cmd_vel = *msg;
        }

        void update() {
            double u = cmd_vel.linear.x;
            double omega = cmd_vel.angular.z;

            // Publish wheel velocities from cmd_vel
            auto leftMsg = std_msgs::msg::Float64();
            auto rightMsg = std_msgs::msg::Float64();
            leftMsg.data = (u - (l * omega) / 2.0) * (1.0 / r);
            rightMsg.data = (u + (l * omega) / 2.0) * (1.0 / r);

            leftPub->publish(leftMsg);
            rightPub->publish(rightMsg);
        }

    private:
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leftPub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rightPub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSub;
        rclcpp::TimerBase::SharedPtr timer;

        geometry_msgs::msg::Twist cmd_vel;

        // Simulation constants
        static constexpr double dt{0.01};
        static constexpr double r{0.05}; // wheel radius
        static constexpr double l{0.19}; // wheel base
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<KinematicModel>();
    // rclcpp::spin(node);
    rclcpp::spin(std::make_shared<KinematicModel>());
    rclcpp::shutdown();
    return 0;    
}
