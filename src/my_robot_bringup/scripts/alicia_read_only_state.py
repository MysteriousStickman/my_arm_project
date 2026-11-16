#!/usr/bin/python3
"""Publish fresh measured joint states; never send robot control commands."""

import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


PREFIX = "ALICIA_READ_ONLY "


def measured_positions(data, now, max_age, gripper_type):
    timestamp = float(data["received_monotonic"])
    if not math.isfinite(timestamp) or not 0 <= now - timestamp <= max_age:
        raise ValueError("Expired or invalid measurement timestamp")
    angles = [float(x) for x in data["angles"]]
    gripper = float(data["gripper"])
    if len(angles) != 6 or not all(math.isfinite(x) and abs(x) <= math.pi for x in angles):
        raise ValueError("Expected six finite measured joint angles in [-pi, pi]")
    if not math.isfinite(gripper) or not 0 <= gripper <= 1000:
        raise ValueError("Invalid measured gripper position")
    stroke = {"50mm": 0.025, "100mm": 0.05}[gripper_type]
    return angles + [(1.0 - gripper / 1000.0) * stroke]


class ReadOnlyState(Node):
    def __init__(self):
        super().__init__("alicia_read_only_state")
        defaults = {
            "sdk_python": str(Path.home() / "miniconda3/envs/alicia_d_sdk/bin/python"),
            "sdk_path": "",
            "port": "/dev/ttyACM0",
            "expected_serial": "",
            "gripper_type": "50mm",
            "rate": 10.0,
            "timeout": 1.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        params = {key: self.get_parameter(key).value for key in defaults}
        self.gripper_type = params["gripper_type"]
        if self.gripper_type not in ("50mm", "100mm"):
            raise ValueError("gripper_type must be 50mm or 100mm")
        self.max_age = float(params["timeout"])
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError("timeout must be positive")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.pending = queue.Queue(maxsize=1)
        self.last_sequence = 0
        self.published_count = 0
        self.last_received = time.monotonic()
        self.last_report = self.last_received
        self.worker_error = None
        self.log_dir = tempfile.TemporaryDirectory(prefix="alicia-read-only-")
        command = [
            str(Path(params["sdk_python"]).expanduser()), "-u",
            str(Path(__file__).resolve().with_name("alicia_read_only_worker.py")),
            "--sdk-path", params["sdk_path"], "--port", params["port"],
            "--expected-serial", params["expected_serial"],
            "--rate", str(params["rate"]), "--timeout", str(self.max_age),
        ]
        try:
            worker_env = dict(os.environ)
            worker_env.pop("PYTHONPATH", None)
            worker_env.pop("PYTHONHOME", None)
            self.worker = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, cwd=self.log_dir.name, env=worker_env,
            )
        except Exception:
            self.log_dir.cleanup()
            raise
        self.reader = threading.Thread(target=self.read_worker, daemon=True)
        self.reader.start()
        self.timer = self.create_timer(
            0.02, self.publish_measurement, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info("Read-only feedback: only identity and position queries are allowed")

    def read_worker(self):
        for line in self.worker.stdout:
            if not line.startswith(PREFIX):
                if line.strip():
                    self.get_logger().info("SDK: " + line.strip())
                continue
            try:
                data = json.loads(line[len(PREFIX):])
                if data["type"] == "error":
                    self.worker_error = data["message"]
                elif data["type"] == "identity":
                    self.get_logger().info("Robot serial: " + data["serial_number"])
                elif data["type"] == "state":
                    try:
                        self.pending.get_nowait()
                    except queue.Empty:
                        pass
                    self.pending.put_nowait(data)
            except (ValueError, KeyError) as exc:
                self.worker_error = f"Invalid SDK response: {exc}"

    def publish_measurement(self):
        if self.worker_error is not None or self.worker.poll() is not None:
            self.get_logger().error(self.worker_error or "SDK reader exited; stopping feedback")
            self.timer.cancel()
            rclpy.try_shutdown(context=self.context)
            return
        try:
            data = self.pending.get_nowait()
        except queue.Empty:
            limit = 15.0 if self.published_count == 0 else 2.0 * self.max_age
            if time.monotonic() - self.last_received > limit:
                self.worker_error = "No fresh SDK sample received; stopping read-only feedback"
            return
        try:
            now = time.monotonic()
            positions = measured_positions(data, now, self.max_age, self.gripper_type)
            if data["sequence"] <= self.last_sequence:
                raise ValueError("Repeated or out-of-order feedback")
            msg = JointState()
            # Stamp only a validated fresh frame. Wall-clock adjustments must
            # not make the polling timer or freshness checks stall.
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f"Joint{i}" for i in range(1, 7)] + ["Gripper"]
            msg.position = positions
            # No invented velocity/effort values; right_finger is a URDF mimic joint.
            self.publisher.publish(msg)
            self.last_sequence = data["sequence"]
            self.last_received = now
            self.published_count += 1
            if self.published_count == 1:
                self.get_logger().info("First measured joint state published on /joint_states")
            if now - self.last_report >= 5.0:
                self.get_logger().info(
                    f"Feedback active: {self.published_count} states published; "
                    f"receipt age={now - data['received_monotonic']:.3f}s"
                )
                self.last_report = now
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(str(exc))

    def close(self):
        if self.worker.poll() is None:
            self.worker.send_signal(signal.SIGINT)
            try:
                self.worker.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.worker.kill()
                self.worker.wait()
        self.reader.join(timeout=1)
        self.worker.stdout.close()
        self.log_dir.cleanup()


def main():
    rclpy.init()
    node = None
    try:
        node = ReadOnlyState()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
