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

#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class PotentialFieldController : public rclcpp::Node {
 public:
  PotentialFieldController() : Node("PotentialFieldController") {
    // Setup publisher / subscribers
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", 10, [this](const geometry_msgs::msg::PoseStamped &msg) {
            // TODO Target yaw
          this->target_x = msg.pose.position.x;
          this->target_y = msg.pose.position.y;
        }
    );

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, [this](const sensor_msgs::msg::LaserScan &msg){
        this->scan_msg_ = msg;
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

    this->declare_parameter("max_range", 0.5);
    max_range_ = this->get_parameter("max_range").as_double();

    this->declare_parameter("goal_gain", 1.0);
    goal_gain_ = this->get_parameter("goal_gain").as_double();

    this->declare_parameter("obstacle_gain", 10.0);
    obstacle_gain_ = this->get_parameter("obstacle_gain").as_double();

    timer = this->create_wall_timer(
        10ms, std::bind(&PotentialFieldController::update, this)
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
    // Hardcoded for now :)
    double dt = 0.01;

    geometry_msgs::msg::TransformStamped transformStamped;
    // try{
    //   transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    // }
    // catch (tf2::TransformException &ex) {
    //   RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    //   rclcpp::sleep_for(1s);
    //   return;
    // }

    // /// Get RPY from transform quaternion.
    // tf2::Quaternion q;
    // tf2::fromMsg(transformStamped.transform.rotation, q);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // TMP Variables
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    double yaw = 0;
    double target_x = 1;
    double target_y = 0;

    // Compute error distance and angle to target
    double error_x = target_x - transformStamped.transform.translation.x;
    double error_y = target_y - transformStamped.transform.translation.y;


    // Goal attraction. Transform error from world frame to robot frame.
    double goal_x = error_x * std::cos(yaw) - error_y * std::sin(yaw);
    double goal_y = error_x * std::sin(yaw) + error_y * std::cos(yaw);

    int range_count = 0;
    double min_det_range = max_range_;

    double obstacle_x{0}, obstacle_y{0};
    // Compute forces from LIDAR data.
    if(scan_msg_.has_value()){
      for(int i = 0; i < scan_msg_->ranges.size(); i++){
        // Clamp and normalize.
        double range = std::clamp(static_cast<double>(scan_msg_->ranges[i]), 0.0, max_range_) / max_range_;
        double angle = wrapAngle(static_cast<double>(i) * scan_msg_->angle_increment + scan_msg_->angle_min + M_PI);
        double intensity = scan_msg_->intensities[i];
        
        
        // Only process forward facing ranges.
        double min_angle = -M_PI / 2.0;
        double max_angle = M_PI / 3.0;
        if(angle < min_angle || angle > max_angle){
          continue;
        }

        // Check for valid intensity.
        if(intensity == 0){
          continue;
        }

        // Invert such that obstacles create a repelling force.
        double x = (1.0 - range) * std::cos(angle);
        double y = (1.0 - range) * std::sin(angle);
        min_det_range = std::min(min_det_range, range);

        obstacle_x -= x;
        obstacle_y -= y;
        range_count += 1;
      }
    }

    if(range_count == 0){
      RCLCPP_INFO(this->get_logger(), "No valid range from lidar.");
      return;
    }

    obstacle_x /= range_count;
    obstacle_y /= range_count;

    double force_x = goal_x * goal_gain_ + obstacle_x * obstacle_gain_;
    double force_y = goal_y * goal_gain_ + obstacle_y * obstacle_gain_;

    double force_mag = std::hypot(force_x, force_y);
    double force_angle = std::atan2(force_y, force_x);

    RCLCPP_INFO(this->get_logger(), "Count %d Obstacle force: (%f, %f) min range: %f. Goal force: (%f, %f). Final Force: (%f, %f)", range_count, obstacle_x, obstacle_y, min_det_range, goal_x, goal_y, force_x, force_y);

    double u = force_mag * std::cos(force_angle) * k_u;
    double r = force_angle * k_r;
    RCLCPP_INFO(this->get_logger(), "U: %f R: %f", u, r);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::optional<sensor_msgs::msg::LaserScan> scan_msg_;

  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double last_u_{0}, last_r_{0};
  rclcpp::Time last_time_;

  double max_u_{1}, max_r_{1};
  double k_u{1}, k_r{1};
  double ki_u{0.0}, ki_r{0.0};

  double max_range_{1};
  double goal_gain_{1}, obstacle_gain_{1};

  double dist_integral{0}, angle_integral{0};

  double target_x{0}, target_y{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldController>());
  rclcpp::shutdown();
  return 0;
}