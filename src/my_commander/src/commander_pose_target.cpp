#include "my_commander/commander.h"
#include <cmath>
#include <functional>
#include <thread>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <set>

using namespace std::placeholders;


// 第一步：收到客户端目标，决定接受还是拒绝。
rclcpp_action::GoalResponse Commander::pose_target_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PoseTarget::Goal> goal)
{
    (void)uuid;

    if (!goal)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 正式接口要求客户端明确给出目标坐标系，避免服务端猜测坐标含义。
    if (goal->target_pose.header.frame_id.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Rejected goal: target_pose.frame_id is empty");
        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto &position = goal->target_pose.pose.position;
    if (!std::isfinite(position.x) ||
        !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
        RCLCPP_WARN(node_->get_logger(), "Rejected goal: target position contains NaN or Inf");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 四元数不能为零，也不能包含 NaN/Inf。
    const auto &orientation = goal->target_pose.pose.orientation;
    const double quaternion_norm_squared =
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w;
    if (!std::isfinite(quaternion_norm_squared) ||
        quaternion_norm_squared < 1e-12)
    {
        RCLCPP_WARN(node_->get_logger(), "Rejected goal: invalid orientation quaternion");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // goal_active_ 用来限制同一时间只能执行一个机械臂目标。
    // compare_exchange_strong 的意思是：
    // 如果 goal_active_ 当前为 false，就把它改成 true，并返回成功；
    // 如果它已经为 true，说明前一个目标还没结束，因此拒绝新目标。
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true))
    {
        RCLCPP_WARN(node_->get_logger(), "A pose target is already running");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(node_->get_logger(), "Received goToPoseTarget goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


// 客户端请求取消时调用。
rclcpp_action::CancelResponse Commander::pose_target_handle_cancel(
        const std::shared_ptr<GoalHandlePoseTarget> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(node_->get_logger(), "Received cancel request for goToPoseTarget");

    // 请求 MoveIt 停止当前运动。
    // execute() 稍后还会调用 canceled()，正式结束这个 Action 目标。
    arm_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 真正执行这个 Action 目标。
void Commander::pose_target_handle_accepted(
        const std::shared_ptr<GoalHandlePoseTarget> goal_handle)
{
    // MoveIt 的规划和执行可能耗时较长，所以放到新线程中执行。
    // 这样 handle_accepted() 可以快速返回，不阻塞 ROS 2 的其他回调。
    std::thread(&Commander::pose_target_execute, this, goal_handle).detach();
}



void Commander::pose_target_execute(const std::shared_ptr<GoalHandlePoseTarget> goal_handle)
{
    // Goal 是客户端发送过来的目标参数。
    auto goal = goal_handle->get_goal();

    // Result 是 Action 最终返回给客户端的结果。
    auto result = std::make_shared<PoseTarget::Result>();

    try
    {
        publishPoseFeedback(
            goal_handle,
            PoseTarget::Feedback::STAGE_PREPARING,
            "Preparing target pose");

        // 检查点 1：如果任务开始前已经收到取消请求，就直接结束目标。
        if (isCancelPoseTargetRequested(goal_handle))
        {
            finishCanceledPoseTargetGoal(goal_handle, result);
            goal_active_ = false;
            return;
        }

        const auto motion_result = goToPoseTarget(
            goal_handle,
            goal->target_pose,
            goal->cartesian_path, goal->is_execute);

        // 检查点 3：MoveIt 的 plan()/execute() 是阻塞调用。
        // 取消是否发生在规划过程中
        if (isCancelPoseTargetRequested(goal_handle))
        {
            finishCanceledPoseTargetGoal(goal_handle, result);
            goal_active_ = false;
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
        result->message = "Unknown exception while moving to pose target";
        RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
        if (goal_handle->is_active())
        {
            goal_handle->abort(result);
        }
    }

    // 本次目标已经结束，允许接收下一个目标。
    goal_active_ = false;
}


// 普通目标位姿规划或笛卡尔路径规划。
std::pair<bool, std::string> Commander::goToPoseTarget(
    const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
    const geometry_msgs::msg::PoseStamped &goal_target_pose,
    bool cartesian_path, 
    bool is_execute)
{
    // 使用副本，服务端可以在不修改原始 Goal 的情况下规范化四元数。
    auto target_pose = goal_target_pose;
    tf2::Quaternion quaternion(
        target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z,
        target_pose.pose.orientation.w);
    quaternion.normalize();
    target_pose.pose.orientation.x = quaternion.x();
    target_pose.pose.orientation.y = quaternion.y();
    target_pose.pose.orientation.z = quaternion.z();
    target_pose.pose.orientation.w = quaternion.w();

    // 普通规划和笛卡尔路径都按照客户端声明的 frame_id 解释目标位姿。
    // 该坐标系必须存在于当前 TF 树中。
    arm_->setPoseReferenceFrame(target_pose.header.frame_id);

    // 每次规划都从机械臂当前状态开始。
    arm_->setStartStateToCurrentState();
    publishPoseFeedback(
        goal_handle,
        PoseTarget::Feedback::STAGE_PLANNING,
        "Planning pose_target trajectory");

    if (!cartesian_path)
    {
        // 普通规划：只给 MoveIt 一个末端目标位姿，
        // 具体关节如何运动由 MoveIt 的规划器决定。
        arm_->setPoseTarget(target_pose);
        const auto motion_result = planAndExecutePoseTarget(goal_handle, arm_, is_execute);

        // 清除旧目标，避免影响下一次规划。
        arm_->clearPoseTargets();
        return motion_result;
    }

    // 笛卡尔路径规划：末端执行器尽量沿直线到达目标。
    // waypoints 可以包含多个途经点，这里只有最终目标这一个点。
    std::vector<geometry_msgs::msg::Pose> waypoints{target_pose.pose};
    moveit_msgs::msg::RobotTrajectory trajectory;

    // eef_step=0.01 表示每隔约 1 cm 插值计算一次末端位姿。
    // jump_threshold=0.0 表示关闭关节跳变阈值检查。
    const double fraction = arm_->computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory);

    // fraction 是成功规划出的笛卡尔路径比例，不是机械臂执行进度。

    // 要求至少成功规划出 95% 的路径，否则认为规划失败。
    if (fraction < 0.95)
    {
        return {false, "Cartesian path planning incomplete, fraction=" +
                            std::to_string(fraction)};
    }

    if (!is_execute)
    {
        // 客户端只要求规划，不要求机械臂真正运动。
        return {true, "Cartesian path planned successfully; execution skipped"};
    }

    // 检查点 2：规划完成后、执行前检查取消。
    // 这里只阻止下一步执行；Action 的最终状态由外层 execute() 统一设置。
    if (isCancelPoseTargetRequested(goal_handle))
    {
        return {false, "Goal canceled before trajectory execution"};
    }

    publishPoseFeedback(
        goal_handle,
        PoseTarget::Feedback::STAGE_EXECUTING,
        "Executing Cartesian trajectory");
    const auto execute_code = arm_->execute(trajectory);

    // MoveIt 通常通过错误码表示失败，而不是抛出异常。
    if (execute_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return {false, "Failed to execute pose_target Cartesian trajectory"};
    }

    return {true, "Cartesian trajectory executed pose_target successfully"};
}


// 普通目标位姿的“规划 + 可选执行”流程。
std::pair<bool, std::string> Commander::planAndExecutePoseTarget(
    const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
    const std::shared_ptr<MoveGroupInterface> &interface,
    bool is_execute)
{
    // Plan 中保存 MoveIt 规划出的轨迹。
    MoveGroupInterface::Plan plan;
    const auto plan_code = interface->plan(plan);

    // plan() 返回错误码，必须主动判断是否成功。
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return {false, "Failed to plan trajectory to pose target"};
    }

    if (!is_execute)
    {
        // 只规划、不执行时，到这里就可以返回成功。
        return {true, "Trajectory planned successfully; execution skipped"};
    }

    // 检查点 2：规划完成后、执行前检查取消。
    // 这里只阻止下一步执行；Action 的最终状态由外层 execute() 统一设置。
    if (isCancelPoseTargetRequested(goal_handle))
    {
        return {false, "Goal canceled before trajectory execution"};
    }

    publishPoseFeedback(
        goal_handle,
        PoseTarget::Feedback::STAGE_EXECUTING,
        "Executing planned trajectory");
    const auto execute_code = interface->execute(plan);
    if (execute_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return {false, "Failed to execute pose_target planned trajectory"};
    }

    return {true, "Trajectory executed pose_target successfully"};
}


// 发布机器可判断的固定阶段码。
// message 只用于日志或界面显示，客户端业务逻辑不应依赖文字内容。
void Commander::publishPoseFeedback(
    const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
    uint8_t stage,
    const std::string &message) const
{
    if (!goal_handle->is_active())
    {
        return;
    }

    // 根据 PoseTarget.action 中的 Feedback 定义创建消息。
    auto feedback = std::make_shared<PoseTarget::Feedback>();
    feedback->stage = stage;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
}


// 只查询 Action 是否已经进入 CANCELING 状态。
bool Commander::isCancelPoseTargetRequested(
    const std::shared_ptr<GoalHandlePoseTarget> &goal_handle) const
{
    return goal_handle->is_canceling();
}

// 只负责完成 Action 协议层的取消
void Commander::finishCanceledPoseTargetGoal(
    const std::shared_ptr<GoalHandlePoseTarget> &goal_handle,
    const std::shared_ptr<PoseTarget::Result> &result)
{
    result->is_finish = false;
    result->message = "Goal canceled";
    goal_handle->canceled(result);
}
