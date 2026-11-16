#include "my_commander/commander.h"

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>


bool Commander::addWall(const WallConfig& config)
{
    moveit_msgs::msg::CollisionObject wall;

    wall.header.frame_id = arm_->getPlanningFrame();
    wall.id = config.id;

    // 设置BOX 几何体
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

    primitive.dimensions = {
        config.size_x,
        config.size_y,
        config.size_z
    };

    // 设置位置
    geometry_msgs::msg::Pose pose;
    pose.position.x = config.x;
    pose.position.y = config.y;
    pose.position.z = config.z;

    // 3. 欧拉角转四元数
    tf2::Quaternion q;
    q.setRPY(
        config.roll,
        config.pitch,
        config.yaw
    );

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // 4. 添加几何体和位姿
    wall.primitives.push_back(primitive);
    wall.primitive_poses.push_back(pose);

    wall.operation = moveit_msgs::msg::CollisionObject::ADD;

    // 5. 添加到 Planning Scene
    bool success =
        planning_scene_interface_->applyCollisionObject(wall);

    RCLCPP_INFO(
        node_->get_logger(),
        "Add wall [%s]: %s",
        config.id.c_str(),
        success ? "success" : "failed"
    );

    return success;
}



bool Commander::addTestWalls()
{
    std::vector<WallConfig> walls = {

        // 墙 1：原来的正面墙
        {
            "table",
            0.705, 0.705, 0.04,
            0.25499, 0.0475, -0.02,
            0.0, 0.0, 0.0
        },
        {
            "computer_table",
            1.0, 0.705, 0.08,
            // 后方为 -X：原桌后沿 - 间隙 - 电脑桌半深度。
            // 0.25499 - 0.705 / 2 - 0.02 - 1 / 2 = -0.61751
            -0.61751, 0.0475, 0.04,
            0.0, 0.0, 0.0
        },
        {
            "computer",
            0.2, 0.705, 0.5,
            // 后方为 -X：原桌后沿 - 间隙 - 电脑桌半深度。
            // 0.25499 - 0.705 / 2 - 0.08 - 0.2 / 2 = -0.27751
            -0.27751, 0.0475, 0.33,
            0.0, 0.0, 0.0
        },
        {
            "wall",  // x_max: 0.60749，x_min: -1.11751
            // X 长度 = x_max - x_min。
            1.725, 0.1, 1,
            // X 中心 = (x_max + x_min) / 2。
            -0.25501, 0.45, 0.5,
            0.0, 0.0, 0.0
        },
        {
            "wardrobe",  
            0.05, 0.705, 1,
            // X 中心 = (x_max + x_min) / 2。
            0.63249, 0.0475, 0.5,
            0.0, 0.0, 0.0
        },

    };

    bool all_success = true;

    for (const auto& wall : walls)
    {
        if (!addWall(wall))
        {
            all_success = false;
        }
    }

    return all_success;
}
