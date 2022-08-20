#include <behaviortree_cpp_v3/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <c3pzero_behaviors/nav_to_goal_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace c3pzero_behaviors
{
class C3PzeroBehaviorsBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<NavToGoalPose>(factory, "NavToGoalPose", shared_resources);
    
  }
};
}  // namespace c3pzero_behaviors

PLUGINLIB_EXPORT_CLASS(c3pzero_behaviors::C3PzeroBehaviorsBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
