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
rclcpp_action::GoalResponse Commander::joint_target_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const JointTarget::Goal> goal)
{
    (void)uuid;

    if (!goal)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto & names = goal->joint_names;
    const auto & positions = goal->positions;

    // 校验1：两个数组都不为空
    if (names.empty() || positions.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
            "JointTarget rejected: joint_names or positions is empty");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 校验2：两个数组长度必须相同（一一对应）
    if (names.size() != positions.size())
    {
        RCLCPP_WARN(node_->get_logger(),
            "JointTarget rejected: size mismatch, names=%zu positions=%zu",
            names.size(), positions.size());
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 校验3：关节名称不能重复
    std::set<std::string> unique_names(names.begin(), names.end());
    if (unique_names.size() != names.size())
    {
        RCLCPP_WARN(node_->get_logger(),
            "JointTarget rejected: duplicate joint names");
        return rclcpp_action::GoalResponse::REJECT;
    }


    // 校验4：关节名称属于 arm 规划组

    const auto robot_model = arm_->getRobotModel();
    if (!robot_model)
    {
        RCLCPP_ERROR(node_->get_logger(),
            "JointTarget rejected: robot model is unavailable");
        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto *joint_model_group =
        robot_model->getJointModelGroup(arm_->getName());

    if (!joint_model_group)
    {
        RCLCPP_ERROR(node_->get_logger(),
            "JointTarget rejected: planning group '%s' does not exist",
            arm_->getName().c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    //         用 MoveGroupInterface 动态拿到本组关节，避免硬编码
    const auto &arm_variables = joint_model_group->getVariableNames();

    const std::set<std::string> arm_variable_set(
    arm_variables.begin(), arm_variables.end());

    for (const auto &name : names)
    {
        if (arm_variable_set.find(name) == arm_variable_set.end())
        {
            RCLCPP_WARN(node_->get_logger(),
                "JointTarget rejected: joint variable '%s' not in group '%s'",
                name.c_str(), arm_->getName().c_str());

            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // 校验5：目标没有超出 URDF 上下限
    // 用 getVariableBounds 动态读取，URDF 改了不用改代码
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        if (!std::isfinite(positions[i]))
        {
            RCLCPP_WARN(node_->get_logger(),
                "JointTarget rejected: non-finite position for joint '%s'",
                names[i].c_str());

            return rclcpp_action::GoalResponse::REJECT;
        }

        const moveit::core::VariableBounds &bounds =
            robot_model->getVariableBounds(names[i]);

        // 连续旋转关节可能没有位置上下限，只有 bounded 时才检查。
        if (bounds.position_bounded_ &&
            (positions[i] < bounds.min_position_ - kBoundTol ||
            positions[i] > bounds.max_position_ + kBoundTol))
        {
            RCLCPP_WARN(node_->get_logger(),
                "JointTarget rejected: joint '%s' target %.4f "
                "out of range [%.4f, %.4f]",
                names[i].c_str(),
                positions[i],
                bounds.min_position_,
                bounds.max_position_);

            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // 校验6：NaN 与上下限比较时两个条件都会返回 false，因此当前的范围检查会放过 NaN。
    for (size_t i = 0; i < positions.size(); ++i)
    {
        if (!std::isfinite(positions[i]))
        {
            RCLCPP_WARN(node_->get_logger(),
                "JointTarget rejected: non-finite position for joint '%s'",
                names[i].c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true))
    {
        RCLCPP_WARN(node_->get_logger(), "A pose target is already running");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(node_->get_logger(), "Received goToJointTarget goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


// 客户端请求取消时调用。
rclcpp_action::CancelResponse Commander::joint_target_handle_cancel(
        const std::shared_ptr<GoalHandleJointTarget> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(node_->get_logger(), "Received cancel request for goToJointTarget");

    // 请求 MoveIt 停止当前运动。
    // execute() 稍后还会调用 canceled()，正式结束这个 Action 目标。
    arm_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 真正执行这个 Action 目标。
void Commander::joint_target_handle_accepted(
        const std::shared_ptr<GoalHandleJointTarget> goal_handle)
{
    // MoveIt 的规划和执行可能耗时较长，所以放到新线程中执行。
    // 这样 handle_accepted() 可以快速返回，不阻塞 ROS 2 的其他回调。
    std::thread(&Commander::joint_target_execute, this, goal_handle).detach();
}


void Commander::joint_target_execute(const std::shared_ptr<GoalHandleJointTarget> goal_handle)
{
    // Goal 是客户端发送过来的目标参数。
    auto goal = goal_handle->get_goal();

    // Result 是 Action 最终返回给客户端的结果。
    auto result = std::make_shared<JointTarget::Result>();

    try
    {
        publishJointTargetStage(
            goal_handle,
            JointTarget::Feedback::STAGE_PREPARING,
            "Preparing target joint");

        // 检查点 1：如果任务开始前已经收到取消请求，就直接结束目标。
        if (isCancelJointTargetRequested(goal_handle))
        {
            finishCanceledJointTargetGoal(goal_handle, result);
            goal_active_ = false;
            return;
        }

        const auto motion_result = goToJointTarget(
            goal_handle,
            goal->joint_names,
            goal->positions,
            goal->is_execute);

        // 检查点 3：MoveIt 的 plan()/execute() 是阻塞调用。
        // 取消是否发生在规划过程中
        if (isCancelJointTargetRequested(goal_handle))
        {
            finishCanceledJointTargetGoal(goal_handle, result);
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


std::pair<bool, std::string> Commander::goToJointTarget(
    const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &positions,
    bool is_execute)
{
    arm_->setStartStateToCurrentState();
    publishJointTargetStage(
        goal_handle,
        JointTarget::Feedback::STAGE_PLANNING,
        "Planning pose_target trajectory");
    
    
    std::map<std::string, double> joint_targets;
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        joint_targets[joint_names[i]] = positions[i];
    }

    // setJointValueTarget() 只负责把关节目标交给 MoveIt。
    // 如果目标无效或 MoveIt 无法接受该目标，就不要继续进行轨迹规划。
    if (!arm_->setJointValueTarget(joint_targets))
    {
        return {false, "MoveIt rejected the joint target"};
    }

    const auto motion_result = planAndExecuteJointTarget(goal_handle, arm_, is_execute);

    // 清除旧目标，避免影响下一次规划。
    arm_->clearPoseTargets();
    return motion_result;
}



// 普通目标位姿的“规划 + 可选执行”流程。
std::pair<bool, std::string> Commander::planAndExecuteJointTarget(
    const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
    const std::shared_ptr<MoveGroupInterface> &interface,
    bool is_execute)
{
    // Plan 中保存 MoveIt 规划出的轨迹。
    MoveGroupInterface::Plan plan;
    const auto plan_code = interface->plan(plan);

    // plan() 返回错误码，必须主动判断是否成功。
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return {false, "Failed to plan trajectory to joint target"};
    }

    if (!is_execute)
    {
        // 只规划、不执行时，到这里就可以返回成功。
        return {true, "Trajectory planned successfully; execution skipped"};
    }

    // 检查点 2：规划完成后、执行前检查取消。
    // 这里只阻止下一步执行；Action 的最终状态由外层 execute() 统一设置。
    if (isCancelJointTargetRequested(goal_handle))
    {
        return {false, "Goal canceled before trajectory execution"};
    }

    publishJointTargetStage(
        goal_handle,
        JointTarget::Feedback::STAGE_EXECUTING,
        "Executing planned trajectory");
    const auto execute_code = interface->execute(plan);
    if (execute_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return {false, "Failed to execute joint_target planned trajectory"};
    }

    return {true, "Trajectory executed joint_target successfully"};
}





// 发布机器可判断的固定阶段码。
// message 只用于日志或界面显示，客户端业务逻辑不应依赖文字内容。
void Commander::publishJointTargetStage(
    const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
    uint8_t stage,
    const std::string &message) const
{
    if (!goal_handle->is_active())
    {
        return;
    }

    // 根据 JointTarget.action 中的 Feedback 定义创建消息。
    auto feedback = std::make_shared<JointTarget::Feedback>();
    feedback->stage = stage;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
}


// 只查询 Action 是否已经进入 CANCELING 状态。
bool Commander::isCancelJointTargetRequested(
    const std::shared_ptr<GoalHandleJointTarget> &goal_handle) const
{
    return goal_handle->is_canceling();
}

// 只负责完成 Action 协议层的取消
void Commander::finishCanceledJointTargetGoal(
    const std::shared_ptr<GoalHandleJointTarget> &goal_handle,
    const std::shared_ptr<JointTarget::Result> &result)
{
    result->is_finish = false;
    result->message = "Goal canceled";
    goal_handle->canceled(result);
}
