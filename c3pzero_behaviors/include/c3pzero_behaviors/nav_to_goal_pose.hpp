#pragma once

#include <behaviortree_cpp_v3/action_node.h>

// This header includes the SharedResourcesNode type
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ClientGoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
}

namespace c3pzero_behaviors
{
/**
 * @brief TODO(...)
 */
class NavToGoalPose : public moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  /**
   * @brief Constructor for the NavToGoalPose behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  NavToGoalPose(const std::string& name, const BT::NodeConfiguration& config, const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  ~NavToGoalPose();

  /**
   * @brief Implementation of the required providedPorts() function for the NavToGoalPose Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return NavToGoalPose does not use expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of onStart(). Runs when the Behavior is ticked for the first time.
   * @return Always returns BT::NodeStatus::RUNNING, since the success of Behavior's initialization is checked in @ref
   * onRunning().
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Implementation of onRunning(). Checks the status of the Behavior when it is ticked after it starts running.
   * @return TODO(...)
   */
  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  void execute(const NavigateToPose::Goal& goal);

  fp::Result<ClientGoalHandleNavigateToPose::WrappedResult> sendActionGoal(const NavigateToPose::Goal& goal,
                                                                            const double result_timeout);

  std::atomic<BT::NodeStatus> process_status_{ BT::NodeStatus::IDLE };

  std::shared_ptr<rclcpp_action::Client<NavigateToPose>> nav_to_pose_client_;

  std::shared_ptr<ClientGoalHandleNavigateToPose> goal_handle_;

  std::mutex goal_handle_mutex_;

  std::thread thread_;

};
}  // namespace c3pzero_behaviors
