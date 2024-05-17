#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/float32.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"

class KinematicModel : public rclcpp::Node 
{
    public:
        KinematicModel() : Node("puzzlebot_kinematic_model"),
            leftPub(this->create_publisher<std_msgs::msg::Float32>("VelocityEncL", 10)),
            rightPub(this->create_publisher<std_msgs::msg::Float32>("VelocityEncR", 10))
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
            auto leftMsg = std_msgs::msg::Float32();
            auto rightMsg = std_msgs::msg::Float32();
            leftMsg.data = (u - (l * omega) / 2.0) * (1.0 / r);
            rightMsg.data = (u + (l * omega) / 2.0) * (1.0 / r);

            leftPub->publish(leftMsg);
            rightPub->publish(rightMsg);
        }

    private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightPub;
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
