#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class JointStatePubNode : public rclcpp::Node {
public:
    JointStatePubNode() : Node("joint_state_publisher") {
        double dt = 0.01;
        jointPub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        leftSub = this->create_subscription<std_msgs::msg::Float32>(
            "VelocityEncL", 1, [&](const std_msgs::msg::Float32::SharedPtr msg) {
                this->wl = msg->data;
            });

        rightSub = this->create_subscription<std_msgs::msg::Float32>(
            "VelocityEncR", 1, [&](const std_msgs::msg::Float32::SharedPtr msg) {
                this->wr = msg->data;
            });

        timer = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&JointStatePubNode::update, this));
    }

protected:
    void update() {
        // Hardcode for now
        double dt = 0.01;

        left_wheel += wl * dt;
        right_wheel += wr * dt;

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.name = {"leftWheel", "rightWheel"};
        joint_msg.position = {left_wheel, right_wheel};

        joint_msg.header.stamp = this->now();
        jointPub->publish(joint_msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr leftSub, rightSub;
    rclcpp::TimerBase::SharedPtr timer;

    double wl{0}, wr{0};
    double left_wheel{0}, right_wheel{0};

    // Simulation constants
    static constexpr double dt{0.01};
    static constexpr double r{0.05}; // wheel radius
    static constexpr double l{0.19}; // wheel base
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePubNode>());
    rclcpp::shutdown();
    return 0;
}
