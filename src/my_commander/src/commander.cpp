#include "my_commander/commander.h"
#include <cmath>
#include <functional>
#include <thread>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <set>

using namespace std::placeholders;

// Commander 不是一个 Node 子类，而是使用外部传入的 ROS 2 节点。
// 它主要完成两件事：
// 1. 创建 MoveIt 的机械臂控制接口；
// 2. 创建 goToPoseTarget Action 服务端。
Commander::Commander(const std::shared_ptr<rclcpp::Node> &node)
{
    // 保存节点，后面创建 Action、获取时间和输出日志都要使用它。
    node_ = node;

    // 创建 MoveIt 规划组。
    arm_ = std::make_shared<MoveGroupInterface>(node, "arm");
    arm_->setMaxVelocityScalingFactor(0.1);      // 使用最大速度的 10%
    arm_->setMaxAccelerationScalingFactor(0.1); // 使用最大加速度的 10%
    gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

    // Reentrant 表示该回调组允许多个回调并发执行。
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 创建 Action 服务端。
    // Action 收到请求后，会依次进入以下回调：
    // pose_target_handle_goal() -> pose_target_handle_accepted() -> execute()
    // 如果客户端请求取消，则会进入 pose_target_handle_cancel()。
    action_pose_target_server_ = rclcpp_action::create_server<PoseTarget>(
        node_, 
        "goToPoseTarget",  // Action 名称
        std::bind(&Commander::pose_target_handle_goal, this, _1, _2),
        std::bind(&Commander::pose_target_handle_cancel, this, _1),
        std::bind(&Commander::pose_target_handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        callback_group_
    );
    
    action_joint_target_server_ = rclcpp_action::create_server<JointTarget>(
        node_, 
        "goToJointTarget",  // Action 名称
        std::bind(&Commander::joint_target_handle_goal, this, _1, _2),
        std::bind(&Commander::joint_target_handle_cancel, this, _1),
        std::bind(&Commander::joint_target_handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        callback_group_
    );
    
}