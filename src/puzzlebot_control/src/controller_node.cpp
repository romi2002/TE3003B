#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    last_time = this->get_clock()->now();

    // Setup robot constant params
    this->declare_parameter("wheel_base", 0.18);
    l = this->get_parameter("wheel_base").as_double();

    this->declare_parameter("wheel_radius", 0.05);
    r = this->get_parameter("wheel_radius").as_double();

    // Controller constants
    this->declare_parameter("max_u", 0.6);
    max_u = this->get_parameter("max_u").as_double();
    
    this->declare_parameter("max_r", 1.25);
    max_r = this->get_parameter("max_r").as_double();
    
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
    if (std::abs(u) > max_u) u = std::copysign(max_u, u);
    if (std::abs(r) > max_r) r = std::copysign(max_r, r);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = u;
    msg.angular.z = r;
    last_u = u;
    last_r = r;
    cmd_vel_pub->publish(msg);
  }

  double wrapAngle(double angle) {
    // Wraps an angle to [-pi, pi]
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void update() {
    // Hardcoded for now :)
    double dt = 0.01;

    geometry_msgs::msg::TransformStamped transformStamped;
    try{
      transformStamped = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(1s);
      return;
    }

    /// Get RPY from transform quaternion.
    tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Compute error distance and angle to target
    double error_x = target_x - transformStamped.transform.translation.x;
    double error_y = target_y - transformStamped.transform.translation.y;
    double distance = std::hypot(error_x, error_y);
    double angle = wrapAngle(std::atan2(error_y, error_x) - yaw);

    // Compute velocities with proportional control
    double u = distance * std::cos(angle) * k_u + dist_integral * ki_u;
    double r = -angle * k_r - angle_integral * ki_r;

    // Stop when close
    if (distance < 0.1) {
      u = 0;
      r = 0;
    }

    dist_integral += distance * std::cos(angle) * dt;
    angle_integral += angle * dt;
    sendVelocity(u, r);

    last_time = this->get_clock()->now();
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  // Estimated positions
  double leftWheel{0}, rightWheel{0};
  double last_u{0}, last_r{0};
  rclcpp::Time last_time;

  // TODO make these params
  double l{1}, r{1};
  double max_u{1}, max_r{1};

  double k_u{1}, k_r{1};
  double ki_u{0.0}, ki_r{0.0};

  double dist_integral{0}, angle_integral{0};

  double target_x{0}, target_y{0};
  bool closedLoop{true};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathController>());
  rclcpp::shutdown();
  return 0;
}