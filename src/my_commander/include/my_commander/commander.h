#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <my_robot_interfaces/action/pose_target.hpp>
#include <my_robot_interfaces/action/joint_target.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>

#include <rclcpp/callback_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>



using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PoseTarget = my_robot_interfaces::action::PoseTarget;
using JointTarget = my_robot_interfaces::action::JointTarget;
using GoalHandlePoseTarget = rclcpp_action::ServerGoalHandle<PoseTarget>;
using GoalHandleJointTarget = rclcpp_action::ServerGoalHandle<JointTarget>;

// Commander 不是一个 Node 子类，而是使用外部传入的 ROS 2 节点。
// 它主要完成两件事：
// 1. 创建 MoveIt 的机械臂控制接口；
// 2. 创建 goToPoseTarget Action 服务端。
class Commander
{
public:
    explicit Commander(const std::shared_ptr<rclcpp::Node> &node);

    // ---------------------  PoseTarget ---------------------
    // 第一步：收到客户端目标，决定接受还是拒绝。
    rclcpp_action::GoalResponse pose_target_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PoseTarget::Goal> goal);


    // 客户端请求取消时调用。
    rclcpp_action::CancelResponse pose_target_handle_cancel(
        const std::shared_ptr<GoalHandlePoseTarget> goal_handle);

    // 第二步：目标被接受后调用。
    void pose_target_handle_accepted(
        const std::shared_ptr<GoalHandlePoseTarget> goal_handle);

    // 第三步：真正执行这个 Action 目标。
    void pose_target_execute(const std::shared_ptr<GoalHandlePoseTarget> goal_handle);
  

    // 目标位姿规划或笛卡尔路径规划
    std::pair<bool, std::string> goToPoseTarget(
        const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
        const geometry_msgs::msg::PoseStamped &goal_target_pose,
        bool cartesian_path = false, bool is_execute = true);

    // 普通目标位姿的“规划 + 可选执行”流程。
    std::pair<bool, std::string> planAndExecutePoseTarget(
        const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
        const std::shared_ptr<MoveGroupInterface> &interface,
        bool is_execute);

    // 发布机器可判断的固定阶段码。
    void publishPoseTargetStage(
        const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
        uint8_t stage,
        const std::string &message) const;

    // 只查询 Action 是否已经进入 CANCELING 状态。
    // 本函数不停止机械臂、不修改 Result，也不改变 Action 的最终状态。
    bool isCancelPoseTargetRequested(
        const std::shared_ptr<GoalHandlePoseTarget> &goal_handle) const;


    // 只负责完成 Action 协议层的取消：
    // 填写 Result，并将目标从 CANCELING 转换为最终的 CANCELED 状态。
    // 实际机械臂的停止请求已经由 pose_target_handle_cancel() 中的 arm_->stop() 发出。
    void finishCanceledPoseTargetGoal(
        const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
        const std::shared_ptr<PoseTarget::Result> &result);




    // ---------------------  JointTarget ---------------------
    rclcpp_action::GoalResponse joint_target_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const JointTarget::Goal> goal);
    
    rclcpp_action::CancelResponse joint_target_handle_cancel(
        const std::shared_ptr<GoalHandleJointTarget> goal_handle);
    

    void joint_target_handle_accepted(
        const std::shared_ptr<GoalHandleJointTarget> goal_handle);

    void joint_target_execute(const std::shared_ptr<GoalHandleJointTarget> goal_handle);
 
    std::pair<bool, std::string> goToJointTarget(
    const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
        const std::vector<std::string> &joint_names,
        const std::vector<double> &positions,
        bool is_execute);

    std::pair<bool, std::string> planAndExecuteJointTarget(
        const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
        const std::shared_ptr<MoveGroupInterface> &interface,
        bool is_execute);

    void publishJointTargetStage(
        const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
        uint8_t stage,
        const std::string &message) const;


    // 只查询 Action 是否已经进入 CANCELING 状态。
    bool isCancelJointTargetRequested(
        const std::shared_ptr<GoalHandleJointTarget> &goal_handle) const;
    

    // 只负责完成 Action 协议层的取消
    void finishCanceledJointTargetGoal(
        const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
        const std::shared_ptr<JointTarget::Result> &result);
    

private:
    // 外部传入的 ROS 2 节点。
    std::shared_ptr<rclcpp::Node> node_;

    // MoveIt 的机械臂和夹爪规划接口。
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    // Action 使用的回调组和服务端对象。
    std::shared_ptr<rclcpp::CallbackGroup> callback_group_;
    std::shared_ptr<rclcpp_action::Server<PoseTarget>> action_pose_target_server_;
    std::shared_ptr<rclcpp_action::Server<JointTarget>> action_joint_target_server_;
    static constexpr double kBoundTol = 1e-3;  // rad/m 容差，按需调

    // true 表示当前已有目标正在执行。
    // atomic_bool 可以安全地被 ROS 回调线程和后台执行线程同时访问。
    std::atomic_bool goal_active_{false};



};


