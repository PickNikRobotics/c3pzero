#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

/**
 * @brief Simple class that broadcasts tf from the odometry message
 *
 */
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

/**
 * @brief Simple class that broadcasts tf from the odometry message
 *
 */
class Odom2TfBroadcaster : public rclcpp::Node
{
public:
  Odom2TfBroadcaster() : Node("tf_broadcaster")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Odom2TfBroadcaster::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

    transform_stamped_.header = msg->header;
    transform_stamped_.child_frame_id = msg->child_frame_id;
    transform_stamped_.transform.translation.x = msg->pose.pose.position.x;
    transform_stamped_.transform.translation.y = msg->pose.pose.position.y;
    transform_stamped_.transform.translation.z = msg->pose.pose.position.z;
    transform_stamped_.transform.rotation.x = msg->pose.pose.orientation.x;
    transform_stamped_.transform.rotation.y = msg->pose.pose.orientation.y;
    transform_stamped_.transform.rotation.z = msg->pose.pose.orientation.z;
    transform_stamped_.transform.rotation.w = msg->pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(transform_stamped_);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom2TfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
