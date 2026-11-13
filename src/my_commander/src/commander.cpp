#include "my_commander/commander.h"
#include <cmath>
#include <functional>
#include <stdexcept>
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
    gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

    // joint_limits.yaml 定义绝对上限，下面的参数决定本节点使用速度上限的比例。
    // NodeOptions 可能已经从 launch 自动声明参数，因此先检查再声明。
    if (!node_->has_parameter("velocity_scaling_factor"))
    {
        node_->declare_parameter<double>("velocity_scaling_factor", 0.05);
    }
    const double velocity_scaling_factor =
        node_->get_parameter("velocity_scaling_factor").as_double();

    if (!std::isfinite(velocity_scaling_factor) ||
        velocity_scaling_factor <= 0.0 || velocity_scaling_factor > 1.0)
    {
        throw std::invalid_argument(
            "velocity_scaling_factor must be finite and in the range (0.0, 1.0]");
    }

    arm_->setMaxVelocityScalingFactor(velocity_scaling_factor);
    gripper_->setMaxVelocityScalingFactor(velocity_scaling_factor);

    RCLCPP_INFO(
        node_->get_logger(),
        "Velocity scaling configured: %.3f",
        velocity_scaling_factor);

    // Reentrant 表示该回调组允许多个回调并发执行。
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 创建 Action 服务端。
    // Action 收到请求后，会依次进入以下回调：
    // pose_target_handle_goal() -> pose_target_handle_accepted() -> execute()
    // 如果客户端请求取消，则会进入 pose_target_handle_cancel()。
    action_pose_target_server_ = rclcpp_action::create_server<PoseTarget>(
        node_,
        "goToPoseTarget", // Action 名称
        std::bind(&Commander::pose_target_handle_goal, this, _1, _2),
        std::bind(&Commander::pose_target_handle_cancel, this, _1),
        std::bind(&Commander::pose_target_handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        callback_group_);

    action_joint_target_server_ = rclcpp_action::create_server<JointTarget>(
        node_,
        "goToJointTarget", // Action 名称
        std::bind(&Commander::joint_target_handle_goal, this, _1, _2),
        std::bind(&Commander::joint_target_handle_cancel, this, _1),
        std::bind(&Commander::joint_target_handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        callback_group_);

    action_gripper_target_server_ = rclcpp_action::create_server<GripperTarget>(
        node_,
        "goToGripperTarget", // Action 名称
        std::bind(&Commander::gripper_target_handle_goal, this, _1, _2),
        std::bind(&Commander::gripper_target_handle_cancel, this, _1),
        std::bind(&Commander::gripper_target_handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        callback_group_);
}

// ------------------------------------ 以下是夹爪部分 ------------------------------------
rclcpp_action::GoalResponse Commander::gripper_target_handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const GripperTarget::Goal> goal)
{
    (void)uuid;

    if (!goal)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 检查1：目标位置不能是 NaN 或 Inf
    if (!std::isfinite(goal->position))
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "GripperTarget rejected: position is not finite");

        return rclcpp_action::GoalResponse::REJECT;
    }

    // 检查2：获取夹爪 MoveIt RobotModel
    const auto robot_model = gripper_->getRobotModel();
    if (!robot_model)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GripperTarget rejected: robot model is unavailable");

        return rclcpp_action::GoalResponse::REJECT;
    }

    // 检查5：动态获取 Gripper 关节的上下限
    const auto &bounds = robot_model->getVariableBounds("Gripper");

    if (bounds.position_bounded_)
    {
        const double lower = bounds.min_position_;
        const double upper = bounds.max_position_;
        RCLCPP_ERROR(
            node_->get_logger(),
            "GripperTarget rejected: target %.4f out of range [%.4f, %.4f]",
            goal->position,
            lower,
            upper);

        if (goal->position < lower - kBoundTol ||
            goal->position > upper + kBoundTol)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "GripperTarget rejected: target %.4f out of range [%.4f, %.4f]",
                goal->position,
                lower,
                upper);

            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    bool expected = false;
    if (!gripper_goal_active_.compare_exchange_strong(expected, true))
    {
        RCLCPP_WARN(node_->get_logger(), "A pose target is already running");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(node_->get_logger(), "Received goToPoseTarget goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Commander::gripper_target_handle_cancel(
    const std::shared_ptr<GoalHandleGripperTarget> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(node_->get_logger(), "Received cancel request for goToGripperTarget");
    // 请求 MoveIt 停止当前运动。
    // execute() 稍后还会调用 canceled()，正式结束这个 Action 目标。
    gripper_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Commander::gripper_target_handle_accepted(
    const std::shared_ptr<GoalHandleGripperTarget> goal_handle)
{
    std::thread(&Commander::gripper_target_execute, this, goal_handle).detach();
}

void Commander::gripper_target_execute(const std::shared_ptr<GoalHandleGripperTarget> goal_handle)
{
    // Goal 是客户端发送过来的目标参数。
    auto goal = goal_handle->get_goal();

    // Result 是 Action 最终返回给客户端的结果。
    auto result = std::make_shared<GripperTarget::Result>();

    try
    {
        publishGripperFeedback(
            goal_handle,
            GripperTarget::Feedback::STAGE_PREPARING,
            "Preparing target pose");

        // 检查点 1：如果任务开始前已经收到取消请求，就直接结束目标。
        if (isCancelGripperRequested(goal_handle))
        {
            finishCanceledGripperGoal(goal_handle, result);
            gripper_goal_active_ = false;
            return;
        }

        const auto motion_result = goToGripperTarget(
            goal_handle,
            goal->position,
            goal->is_execute);

        // 检查点 3：MoveIt 的 plan()/execute() 是阻塞调用。
        // 取消是否发生在规划过程中
        if (isCancelGripperRequested(goal_handle))
        {
            finishCanceledGripperGoal(goal_handle, result);
            gripper_goal_active_ = false;
            return;
        }

        // goToPoseTarget() 返回 pair<bool, string>：
        // first  表示规划/执行是否成功；
        // second 是给客户端看的说明文字。
        result->is_finish = motion_result.first;
        result->message = motion_result.second;

        if (motion_result.first)
        {
            // succeed() 表示 Action 状态为成功。
            goal_handle->succeed(result);
        }
        else
        {
            // abort() 表示 Action 因规划或执行失败而终止。
            goal_handle->abort(result);
        }
    }
    catch (const std::exception &exception)
    {
        // 捕获带错误信息的标准 C++ 异常，避免后台线程直接退出。
        result->is_finish = false;
        result->message = std::string("Exception: ") + exception.what();
        RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
        if (goal_handle->is_active())
        {
            goal_handle->abort(result);
        }
    }
    catch (...)
    {
        // 捕获其他未知类型的异常。
        result->is_finish = false;
        result->message = "Unknown exception while moving to gripper target";
        RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
        if (goal_handle->is_active())
        {
            goal_handle->abort(result);
        }
    }

    // 本次目标已经结束，允许接收下一个目标。
    gripper_goal_active_ = false;
}

std::pair<bool, std::string> Commander::goToGripperTarget(
    const std::shared_ptr<GoalHandleGripperTarget> &goal_handle,
    const double &position,
    bool is_execute)

{
    // 每次规划都从机械臂当前状态开始。
    gripper_->setStartStateToCurrentState();
    publishGripperFeedback(
        goal_handle,
        PoseTarget::Feedback::STAGE_PLANNING,
        "Planning gripper_target trajectory");

    // 开始规划
    gripper_->setJointValueTarget("Gripper", position);
    MoveGroupInterface::Plan plan;
    const auto plan_code = gripper_->plan(plan);
    // plan() 返回错误码，必须主动判断是否成功。
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return {false, "Failed to plan trajectory to gripper target"};
    }

    if (!is_execute)
    {
        // 只规划、不执行时，到这里就可以返回成功。
        return {true, "Trajectory planned successfully; execution skipped"};
    }

    if (isCancelGripperRequested(goal_handle))
    {
        return {false, "Goal canceled before trajectory execution"};
    }

    publishGripperFeedback(
        goal_handle,
        PoseTarget::Feedback::STAGE_EXECUTING,
        "Executing planned trajectory");

    // 开始执行
    const auto execute_code = gripper_->execute(plan);

    // 清除旧目标，避免影响下一次规划。
    gripper_->clearPoseTargets();

    if (execute_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return {false, "Failed to execute gripper_target planned trajectory"};
    }

    return {true, "Trajectory executed gripper_target successfully"};
}

void Commander::publishGripperFeedback(
    const std::shared_ptr<GoalHandleGripperTarget> &goal_handle,
    uint8_t stage,
    const std::string &message) const
{
    if (!goal_handle->is_active())
    {
        return;
    }

    auto feedback = std::make_shared<GripperTarget::Feedback>();
    feedback->stage = stage;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
}

// 只查询 Action 是否已经进入 CANCELING 状态。
bool Commander::isCancelGripperRequested(
    const std::shared_ptr<GoalHandleGripperTarget> &goal_handle) const
{
    return goal_handle->is_canceling();
}

// 只负责完成 Action 协议层的取消
void Commander::finishCanceledGripperGoal(
    const std::shared_ptr<GoalHandleGripperTarget> &goal_handle,
    const std::shared_ptr<GripperTarget::Result> &result)
{
    result->is_finish = false;
    result->message = "Goal canceled";
    goal_handle->canceled(result);
}
