#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import JointTarget, PoseTarget, GripperTarget
from base import TestNode


class PoseTargetNode(TestNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._target_client = ActionClient(self, PoseTarget, "goToPoseTarget")


    def get_goal_msg(self):
        goal_msg = PoseTarget.Goal()
        goal_msg.target_pose.header.frame_id = "base_link"
        goal_msg.target_pose.pose.position.x = -0.053
        goal_msg.target_pose.pose.position.y = 0.470
        goal_msg.target_pose.pose.position.z = 0.151

        goal_msg.target_pose.pose.orientation.x = -0.379
        goal_msg.target_pose.pose.orientation.y = 0.856
        goal_msg.target_pose.pose.orientation.z = 0.275
        goal_msg.target_pose.pose.orientation.w = 0.219
        goal_msg.cartesian_path = False
        goal_msg.is_execute = True
        return goal_msg



class JointTargetNode(TestNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._target_client = ActionClient(self, JointTarget, "goToJointTarget")
        self.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]

    def get_goal_msg(self):
        goal_msg = JointTarget.Goal()
        goal_msg.joint_names = self.joint_names
        goal_msg.positions = [-1.501, -0.229, 0.534, -0.015, -1.248, 0.899]
        goal_msg.is_execute = True
        return goal_msg


class GripperTargetNode(TestNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._target_client = ActionClient(self, GripperTarget, "goToGripperTarget")

    def get_goal_msg(self):
        goal_msg = GripperTarget.Goal()
        goal_msg.position = 0.0
        goal_msg.is_execute = True
        return goal_msg
    


def main(args=None):
    rclpy.init(args=args)
    # node = PoseTargetNode("action_client")  
    # node = JointTargetNode("action_client")  
    node = GripperTargetNode("action_client")  
    node.send_pose()
    node.get_logger().info(f"启动脚本")
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._key_listener.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()