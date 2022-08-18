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
struct Odom2TfBroadcaster : public rclcpp::Node
{
  Odom2TfBroadcaster()
  : Node("odom2tf_broadcaster"), 
    tf_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this())},
    subscription_{this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
          geometry_msgs::msg::TransformStamped transform_stamped;
          transform_stamped.header = msg->header;
          transform_stamped.child_frame_id = msg->child_frame_id;
          transform_stamped.transform.translation.x = msg->pose.pose.position.x;
          transform_stamped.transform.translation.y = msg->pose.pose.position.y;
          transform_stamped.transform.translation.z = msg->pose.pose.position.z;
          transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
          transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
          transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
          transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;
          tf_broadcaster_->sendTransform(transform_stamped);
        })}
  {}
private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom2TfBroadcaster>());
  rclcpp::shutdown();
}
