#include <c3pzero_behaviors/nav_to_goal_pose.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <rclcpp_action/exceptions.hpp>

namespace
{
constexpr double kTimeoutServiceRequest = 5.0;
constexpr double kTimeoutServiceResultDefault = 10.0;

// constexpr auto kPortIDGripperCommandActionName = "gripper_command_action_name";
// constexpr auto kPortIDPosition = "position";

constexpr auto kLoggerName = "NavToGoalPose";
const auto kLogger = rclcpp::get_logger(kLoggerName);

using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

}  // namespace

namespace c3pzero_behaviors
{
NavToGoalPose::NavToGoalPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

NavToGoalPose::~NavToGoalPose()
{
  if (thread_.joinable())
  {
    thread_.join();
  }
}


BT::PortsList NavToGoalPose::providedPorts()
{
  // TODO(...)
  return {
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<double>("theta"),

  };
}

BT::NodeStatus NavToGoalPose::onStart()
{
  process_status_ = BT::NodeStatus::IDLE;

  const auto target_x = getInput<double>("x");
  const auto target_y = getInput<double>("y");
  const auto target_theta = getInput<double>("theta");
  
  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(target_x, target_y, target_theta); error)
  {
    shared_resources_->failure_logger->publishFailureMessage(
        name(),
        MoveItStudioErrorCode{ MoveItErrorCodes::FAILURE, "Failed to get required value from input data port." },
        error.value());
    return BT::NodeStatus::FAILURE;
  }

  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
      shared_resources_->node, "/navigate_to_pose", shared_resources_->callback_group_mutually_exclusive);

  if (thread_.joinable())
  {
    thread_.join();
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = target_x.value();
  goal.pose.pose.position.y = target_y.value();
  goal.pose.pose.orientation.z = target_theta.value();
  // goal.command.position = target_position.value();

  thread_ = std::thread{ [=]() { execute(goal); } };

  process_status_ = BT::NodeStatus::RUNNING;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavToGoalPose::onRunning()
{
  return process_status_;
}

void NavToGoalPose::onHalted()
{
  // Acquire lock to check state of goal handle and use it to cancel the action
  std::scoped_lock lock(goal_handle_mutex_);

  if (goal_handle_ == nullptr || nav_to_pose_client_ == nullptr)
  {
    return;
  }
  try
  {
    nav_to_pose_client_->async_cancel_goal(goal_handle_);
  }
  catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
  {
    // This exception can occur if the Behavior is halted after the action goal has completed.
    // It isn't detrimental to the operation of the Behavior or the action server, so we log a warning
    // and then continue.
    RCLCPP_WARN_STREAM(kLogger, "Caught exception while canceling gripper command goal: " << e.what());
  }
}

void NavToGoalPose::execute(const NavigateToPose::Goal& goal)
{
  const auto wrapped_result = sendActionGoal(goal, 60.0);

  if (!wrapped_result)
  {
    shared_resources_->failure_logger->publishFailureMessage(
        kLoggerName,
        MoveItStudioErrorCode{ MoveItErrorCodes::COMMUNICATION_FAILURE,
                               "Failed to send trajectory execution action goal to server." },
        wrapped_result.error().what);
    process_status_ = BT::NodeStatus::FAILURE;
    return;
  }

  if (wrapped_result.value().code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    const auto& result = wrapped_result.value().result;
    std::stringstream details;
    // if (!result->reached_goal)
    // {
    //   details << "Gripper did not reach goal. Final position was " << result->position << ". ";
    // }
    // if (result->stalled)
    // {
    //   details << "Gripper reports that it stalled. ";
    // }

    shared_resources_->failure_logger->publishFailureMessage(
        kLoggerName, MoveItStudioErrorCode{ MoveItErrorCodes::FAILURE, "Gripper motion did not succeed." },
        details.str());
    process_status_ = BT::NodeStatus::FAILURE;
    return;
  }
  process_status_ = BT::NodeStatus::SUCCESS;
}

fp::Result<ClientGoalHandleNavigateToPose::WrappedResult>
NavToGoalPose::sendActionGoal(const NavigateToPose::Goal& goal,
                                  const double result_timeout = kTimeoutServiceResultDefault)
{
  // Check validity of input parameters. Return failure if client wasn't initialized.
  if (nav_to_pose_client_ == nullptr)
  {
    return tl::make_unexpected(fp::FailedPrecondition("Action client was not initialized."));
  }

  if (!nav_to_pose_client_->action_server_is_ready())
  {
    return tl::make_unexpected(fp::NotFound("Action server not available."));
  }

  {
    // Acquire lock within this scope until we get a new goal_handle corresponding to the new action goal request.
    std::scoped_lock lock(goal_handle_mutex_);

    // Asynchronously send the goal to the server and block until the future is satisfied.
    auto goal_handle_future = nav_to_pose_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::duration<double>(kTimeoutServiceRequest)) ==
        std::future_status::timeout)
    {
      // Return failure if we timed out waiting for the goal handle.
      return tl::make_unexpected(fp::Timeout("Timed out waiting for goal response after " +
                                             std::to_string(kTimeoutServiceRequest) + " seconds."));
    }

    goal_handle_ = goal_handle_future.get();
    if (goal_handle_ == nullptr)
    {
      // Return failure if the goal_handle is a nullptr, which means that the action was rejected by the server.
      return tl::make_unexpected(fp::InvalidArgument("Action goal was rejected by server."));
    }
  }

  auto wrapped_result_future = nav_to_pose_client_->async_get_result(goal_handle_);
  if (wrapped_result_future.wait_for(std::chrono::duration<double>(result_timeout)) == std::future_status::timeout)
  {
    // Return failure if we timed out waiting for the result.
    return tl::make_unexpected(
        fp::Timeout("Timed out waiting for result after " + std::to_string(result_timeout) + " seconds."));
  }

  return wrapped_result_future.get();
}

}  // namespace c3pzero_behaviors
