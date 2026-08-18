#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
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
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    front_ranges_.clear();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double angle = msg->angle_min + i * msg->angle_increment;
      // Keep only front 180°
      if (angle < front_min_angle || angle > front_max_angle) {
        continue;
      }
      double range = msg->ranges[i];

      if (range < msg->range_min || range > msg->range_max) {
        continue;
      }
      front_ranges_.push_back(range);
    }
    RCLCPP_INFO(this->get_logger(), "front readings: %zu",
                front_ranges_.size());
  }
  
  void control_callback()
  {
    RCLCPP_INFO(this->get_logger(), "100ms callback executed.");
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::vector<double> front_ranges_;
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
