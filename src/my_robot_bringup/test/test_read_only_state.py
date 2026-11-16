"""Offline checks: no ROS graph, serial port, or robot is opened."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


worker = load("alicia_read_only_worker")
bridge = load("alicia_read_only_state")


class ReadOnlyChecks(unittest.TestCase):
    def test_query_guard_blocks_control_packets_before_transport(self):
        comm = SimpleNamespace(send_data=Mock(return_value=True))
        transport = comm.send_data
        worker.restrict_queries(comm, "/missing-alicia-device")
        for packet in worker.QUERY_PACKETS:
            self.assertTrue(comm.send_data(list(packet)))
        self.assertEqual(transport.call_count, 2)
        for packet in [
            "AA 03 00 01 FE A8 FF",  # Zero calibration
            "AA 05 00 01 01 F9 FF",  # Torque on
            "AA 05 00 01 00 6F FF",  # Torque off
            "AA 06 03 1C 00 FF",     # Joint/gripper control function
        ]:
            with self.assertRaises(RuntimeError):
                comm.send_data(bytes.fromhex(packet))
        self.assertEqual(transport.call_count, 2)

    def test_missing_explicit_port_does_not_fall_back(self):
        comm = SimpleNamespace(send_data=Mock())
        worker.restrict_queries(comm, "/missing-alicia-device")
        with self.assertRaisesRegex(RuntimeError, "does not exist"):
            comm.find_serial_port()

    def test_permission_error_identifies_the_device_group(self):
        comm = SimpleNamespace(send_data=Mock())
        worker.restrict_queries(comm, "/dev/fake-alicia")
        with patch.object(worker.os.path, "exists", return_value=True), \
                patch.object(worker.os, "access", return_value=False), \
                patch.object(worker.os, "stat", return_value=SimpleNamespace(st_gid=20)), \
                patch.object(worker.grp, "getgrgid", return_value=SimpleNamespace(gr_name="dialout")):
            with self.assertRaisesRegex(RuntimeError, "No read/write permission.*dialout"):
                comm.find_serial_port()

    def test_wrong_robot_is_rejected_before_joint_polling(self):
        driver = SimpleNamespace(
            acquire_info=Mock(return_value=True),
            data_parser=SimpleNamespace(get_info=Mock(return_value={"serial_number": "other"})),
        )
        with self.assertRaisesRegex(RuntimeError, "Unexpected robot"):
            worker.read_states(driver, "expected", 10, 1, Mock())
        self.assertEqual(driver.acquire_info.call_count, 1)

    def test_repeated_measurement_is_not_reissued_as_fresh(self):
        state = SimpleNamespace(timestamp=99.9, angles=[0.0] * 6, gripper=997)
        driver = SimpleNamespace(
            acquire_info=Mock(return_value=True),
            data_parser=SimpleNamespace(
                get_info=Mock(return_value={"serial_number": "expected"}),
                get_joint_state=Mock(return_value=state),
            ),
        )
        monitor = SimpleNamespace(snapshot=Mock(return_value=(state, 1, 99.9)))
        with patch.object(worker.time, "monotonic", return_value=100.0), \
                patch.object(worker.time, "sleep"), patch.object(worker, "emit") as emit:
            with self.assertRaisesRegex(RuntimeError, "stale"):
                worker.read_states(driver, "expected", 10, 1, monitor)
        states = [call.args[0] for call in emit.call_args_list if call.args[0]["type"] == "state"]
        self.assertEqual(len(states), 1)

    def test_feedback_timeout_fails_without_a_state(self):
        driver = SimpleNamespace(
            acquire_info=Mock(side_effect=[True, False]),
            data_parser=SimpleNamespace(get_info=Mock(return_value={"serial_number": "expected"})),
        )
        with patch.object(worker, "emit") as emit:
            with self.assertRaisesRegex(RuntimeError, "timed out"):
                worker.read_states(driver, "expected", 10, 1, Mock())
        self.assertEqual(emit.call_count, 1)  # Identity only; no fabricated joint sample.

    def test_gripper_units_and_zero_joint_angles(self):
        for gripper_type, stroke in [("50mm", 0.025), ("100mm", 0.05)]:
            for raw, expected in [(0, stroke), (1000, 0.0), (997, stroke * 0.003)]:
                data = {"received_monotonic": 100.0, "angles": [0.0] * 6, "gripper": raw}
                positions = bridge.measured_positions(data, 100.1, 1.0, gripper_type)
                self.assertEqual(positions[:6], [0.0] * 6)
                self.assertAlmostEqual(positions[6], expected)

    def test_invalid_or_old_feedback_is_rejected(self):
        base = {"received_monotonic": 100.0, "angles": [0.0] * 6, "gripper": 997}
        for change in [
            {"received_monotonic": 90.0}, {"received_monotonic": 110.0},
            {"angles": [float("nan")] * 6}, {"angles": [0.0] * 5},
            {"gripper": 1001}, {"gripper": float("inf")},
        ]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                bridge.measured_positions({**base, **change}, 100.1, 1.0, "50mm")

    def test_wall_clock_jump_does_not_reject_fresh_frames(self):
        before = SimpleNamespace(timestamp=100.0, angles=[0.0] * 6, gripper=997)
        after = SimpleNamespace(timestamp=50.0, angles=[0.0] * 6, gripper=997)
        driver = SimpleNamespace(
            acquire_info=Mock(return_value=True),
            data_parser=SimpleNamespace(get_info=Mock(return_value={"serial_number": "expected"})),
        )
        monitor = SimpleNamespace(snapshot=Mock(side_effect=[
            (before, 1, 99.9), (after, 2, 99.95), (after, 2, 99.95),
        ]))
        with patch.object(worker.time, "monotonic", return_value=100.0), \
                patch.object(worker.time, "sleep"), patch.object(worker, "emit") as emit:
            with self.assertRaisesRegex(RuntimeError, "stale"):
                worker.read_states(driver, "expected", 10, 1, monitor)
        states = [call.args[0] for call in emit.call_args_list if call.args[0]["type"] == "state"]
        self.assertEqual([state["sequence"] for state in states], [1, 2])

    def test_monitor_counts_only_parsed_joint_frames(self):
        parser = SimpleNamespace(parse_frame=Mock(side_effect=[
            {"type": "version"}, {"type": "joint_data"}, None,
        ]), get_joint_state=Mock(return_value="sample"))
        monitor = worker.FeedbackMonitor(parser)
        parser.parse_frame([])
        self.assertEqual(monitor.snapshot()[1], 0)
        parser.parse_frame([])
        state, sequence, received = monitor.snapshot()
        self.assertEqual((state, sequence), ("sample", 1))
        self.assertGreater(received, 0)
        parser.parse_frame([])
        self.assertEqual(monitor.snapshot()[1], 1)


if __name__ == "__main__":
    unittest.main()
