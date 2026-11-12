#!/usr/bin/env python3
import threading
from pynput import keyboard
from action_msgs.msg import GoalStatus
from rclpy.node import Node


class TestNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        self._target_client = None
        self._goal_handle = None
        self._cancel_lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._cancel_timer = self.create_timer(0.05, self._process_cancel_request)

        self._key_listener = keyboard.Listener(on_press=self._on_key_press)
        self._key_listener.daemon = True
        self._key_listener.start()
        self.get_logger().info("按键监听已启动，按 ESC 可取消当前目标")

    
    def _on_key_press(self, key):
        """键盘回调运行在独立线程中，只通知 ROS Timer 处理取消。"""
        if key == keyboard.Key.esc:
            self._cancel_event.set()

    def _process_cancel_request(self):
        if self._cancel_event.is_set():
            self._cancel_event.clear()
            self.get_logger().warn("检测到 ESC 键，请求取消目标...")
            self.cancel_goal()
    
    def cancel_goal(self):
        """必须在 ROS2 主线程中调用"""
        with self._cancel_lock:
            if self._goal_handle is None:
                self.get_logger().warn("没有可取消的目标")
                return
            handle = self._goal_handle

        self.get_logger().info("正在发送取消请求...")
        future = handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_callback)


    def _cancel_response_callback(self, future):
        try:
            cancel_response = future.result()
        except Exception as exc:
            self.get_logger().error(f"发送取消请求失败: {exc}")
            return

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("服务端已接受取消请求，等待目标真正停止")
        else:
            self.get_logger().warn("服务端拒绝了取消请求，或目标已经结束")
    

    def send_pose(self):
        if not self._target_client:
            raise NotImplemented("请创建self._target_client")
        
        goal_msg = self.get_goal_msg()
        if not self._target_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server  未就绪，请检查服务端是否启动")
            return

        self._send_goal_future = self._target_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # 反馈回调函数
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)  # 结果返回回调函数
    

    def get_goal_msg(self):
        raise NotImplementedError("请实现该方法")
    
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"发送目标失败: {exc}")
            return

        if not goal_handle.accepted:  # 如果动作被拒绝执行
            self.get_logger().info("Goal rejected:(")
            return
        self.get_logger().info("Goal accepted :)")

        with self._cancel_lock:
            self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()  # 异步获取动作最终执行的结果反馈
        self._get_result_future.add_done_callback(self.get_result_callback)  # 设置一个收到最终结果的回调函数
    

    def get_result_callback(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"获取最终结果失败: {exc}")
            return
        finally:
            with self._cancel_lock:
                self._goal_handle = None

        result = response.result
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"目标执行成功，结果: {result.message}")
        elif response.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"目标已取消，结果: {result.message}")
        elif response.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"目标执行失败，结果: {result.message}")
        else:
            self.get_logger().warn(
                f"目标以未知状态结束，status={response.status}，结果: {result.message}"
            )
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"收到反馈:{feedback}")



