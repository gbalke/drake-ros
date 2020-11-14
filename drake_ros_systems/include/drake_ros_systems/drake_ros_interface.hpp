#ifndef DRAKE_ROS_SYSTEMS__DRAKE_ROS_INTERFACE_HPP_
#define DRAKE_ROS_SYSTEMS__DRAKE_ROS_INTERFACE_HPP_

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>

namespace drake_ros_systems
{
// Forward declarations for non-public-API classes
class Publisher;
class Subscription;

/// System that abstracts working with ROS
class DrakeRosInterface
{
public:
  virtual
  std::unique_ptr<Publisher>
  create_publisher(
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    const rclcpp::QoS & qos) = 0;

  virtual
  std::shared_ptr<Subscription>
  create_subscription(
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback) = 0;

  virtual
  void
  spin(
    int timeout_millis) = 0;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__DRAKE_ROS_INTERFACE_HPP_
