#include <memory>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "my_commander/commander.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // MoveIt 参数通常由 launch 文件传入。这个选项允许节点自动声明这些参数。
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("commander", node_options);

    // Commander 必须在 executor.spin() 期间保持存活，
    // 否则 Action Server 和 MoveGroupInterface 会随之销毁。
    auto commander = std::make_shared<Commander>(node);
    (void)commander;

    // Action 执行期间仍需处理取消等回调，因此使用多线程执行器。
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Commander node started");
    executor.spin();

    executor.remove_node(node);
    commander.reset();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
