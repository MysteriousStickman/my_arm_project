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
            "wall_1",
            1.0, 1.0, 0.1,
            0.42249999940395355, 0.0, -0.05,
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