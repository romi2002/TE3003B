#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class PathController : public rclcpp::Node {
 public:
  PathController() : Node("PathController") {
    // Setup publisher / subscribers
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", 10, [this](const geometry_msgs::msg::PoseStamped &msg) {
            // TODO Target yaw
          this->target_x = msg.pose.position.x;
          this->target_y = msg.pose.position.y;
        }
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    last_time_ = this->get_clock()->now();

    // Controller constants
    this->declare_parameter("max_u", 0.6);
    max_u_ = this->get_parameter("max_u").as_double();
    
    this->declare_parameter("max_r", 1.25);
    max_r_ = this->get_parameter("max_r").as_double();
    
    this->declare_parameter("k_u", 0.25);
    k_u = this->get_parameter("k_u").as_double();
    
    this->declare_parameter("k_r", 0.85);
    k_r = this->get_parameter("k_r").as_double();

    timer = this->create_wall_timer(
        10ms, std::bind(&PathController::update, this)
    );
  }

 protected:
  void sendVelocity(double u, double r) {
    // Clamp velocities to max
    if (std::abs(u) > max_u_) u = std::copysign(max_u_, u);
    if (std::abs(r) > max_r_) r = std::copysign(max_r_, r);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = u;
    msg.angular.z = r;
    last_u_ = u;
    last_r_ = r;
    cmd_vel_pub->publish(msg);
  }

  double wrapAngle(double angle) {
    // Wraps an angle to [-pi, pi]
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void update() {
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();

    geometry_msgs::msg::TransformStamped transformStamped;
    try{
      transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(1s);
      return;
    }

    // /// Get RPY from transform quaternion.
    tf2::Quaternion q;
    tf2::fromMsg(transformStamped.transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Compute error distance and angle to target
    double error_x = target_x - transformStamped.transform.translation.x;
    double error_y = target_y - transformStamped.transform.translation.y;
    double distance = std::hypot(error_x, error_y);
    double angle = wrapAngle(std::atan2(error_y, error_x) - yaw);

    // Compute velocities with proportional control
    //ki_r = 0;
    //ki_u = 0;
    double u = distance * std::cos(angle) * k_u;
    double r = angle * k_r;
    //RCLCPP_INFO(this->get_logger(), "U: %f R: %f error_x: %f error_y: %f yaw: %f angle: %f", u, r, error_x, error_y, yaw, angle);

    // Stop when close
    if (distance < 0.05) {
      u = 0;
      r = 0;
    }

    dist_integral += distance * std::cos(angle) * dt;
    angle_integral += angle * dt;
    sendVelocity(u, r);

    last_time_ = this->get_clock()->now();
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double last_u_{0}, last_r_{0};
  rclcpp::Time last_time_;

  double max_u_{1}, max_r_{1};
  double k_u{1}, k_r{1};
  double ki_u{0.0}, ki_r{0.0};

  double dist_integral{0}, angle_integral{0};

  double target_x{0}, target_y{0}, target_yaw{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathController>());
  rclcpp::shutdown();
  return 0;
}