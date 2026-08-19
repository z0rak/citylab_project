#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol(const std::string &node_name = "patrol_node") : Node(node_name) {
    
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Patrol::control_callback, this), callback_group_);

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1), sub_options);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", qos);

    RCLCPP_INFO(this->get_logger(), "Patrol node started");
  }

private:
  const double front_min_angle = -M_PI / 2.0; // -90° (right)
  const double front_max_angle = M_PI / 2.0;  // +90° (left)
  double max_range = 0.0;
  double best_angle = 0.0;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    bool found = false;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double angle = msg->angle_min + i * msg->angle_increment;
      double range = msg->ranges[i];
      // is forward blocked?
      if (std::abs(angle) < 1.8) {
         if (range >= 35) {
            // there are no objects closer than 35mm, keep going forward
            return;
         } 
      }
      // Keep only front 180°
      if (angle < front_min_angle || angle > front_max_angle) {
        continue;
      }
      if (!std::isfinite(range)) {
        continue;
      }
      if (range < msg->range_min || range > msg->range_max) {
        continue;
      }
      if (!found || range > max_range) {
        max_range = range;
        best_angle = angle;
        found = true;
      }
    }

    if (found) {
         std::lock_guard<std::mutex> lock(scan_mutex_);
         direction_ = best_angle;
    }

  }
  
  void control_callback()
  {
    double my_angle = 0.0;

    std::lock_guard<std::mutex> lock(scan_mutex_);
         my_angle = direction_;

    auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.1;  
        message.angular.z = my_angle / 2;
        cmd_vel_pub_->publish(message);
  }
  std::mutex scan_mutex_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  double direction_{0.0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
