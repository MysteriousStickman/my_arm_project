#!/usr/bin/env python3
"""Read telemetry with the SDK's Python; emit JSON for the ROS process."""

import argparse
import grp
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time


PREFIX = "ALICIA_READ_ONLY "
# Exact version and joint/gripper query packets from the local manufacturer SDK.
QUERY_PACKETS = frozenset(
    [bytes.fromhex("AA 01 00 01 FE 23 FF"), bytes.fromhex("AA 06 00 01 FE 9A FF")]
)


def restrict_queries(communicator, port):
    """Install the packet allowlist and disable the SDK's port auto-selection."""
    send_data = communicator.send_data

    def send_query(data):
        if bytes(data) not in QUERY_PACKETS:
            raise RuntimeError("Read-only connection blocked a non-query packet")
        return send_data(data)

    def exact_port():
        if not os.path.exists(port):
            raise RuntimeError(
                f"Serial device does not exist: {port}. Check the USB device path; "
                "the reader will not select a different port automatically."
            )
        if not os.access(port, os.R_OK | os.W_OK):
            device_gid = os.stat(port).st_gid
            try:
                device_group = grp.getgrgid(device_gid).gr_name
            except KeyError:
                device_group = str(device_gid)
            raise RuntimeError(
                f"No read/write permission for {port}; device group={device_group} "
                f"({device_gid}), process gid={os.getegid()}, groups={os.getgroups()}. "
                "Check id -nG in the launch terminal. Group changes require a "
                "new login or newgrp in that same terminal."
            )
        return port

    communicator.send_data = send_query
    communicator.find_serial_port = exact_port


def emit(data):
    print(PREFIX + json.dumps(data, allow_nan=False), flush=True)


class FeedbackMonitor:
    """Track successfully parsed frames using a clock unaffected by wall-time jumps."""

    def __init__(self, parser):
        self.parser = parser
        self.lock = threading.Lock()
        self.sequence = 0
        self.received_monotonic = 0.0
        parse_frame = parser.parse_frame

        def monitored_parse(frame):
            # The SDK sets its response event inside parse_frame(). Hold this
            # lock until receipt metadata is updated, before snapshot() can run.
            with self.lock:
                result = parse_frame(frame)
                if isinstance(result, dict) and result.get("type") == "joint_data":
                    self.sequence += 1
                    self.received_monotonic = time.monotonic()
                return result

        parser.parse_frame = monitored_parse

    def snapshot(self):
        with self.lock:
            return self.parser.get_joint_state(), self.sequence, self.received_monotonic


def read_states(driver, expected_serial, rate, timeout, monitor):
    if not driver.acquire_info("version", wait=True, timeout=timeout):
        raise RuntimeError("No firmware identity response")
    version = driver.data_parser.get_info("version") or {}
    actual_serial = version.get("serial_number")
    if not actual_serial or (expected_serial and actual_serial != expected_serial):
        raise RuntimeError(f"Unexpected robot serial: {actual_serial!r}")
    emit({"type": "identity", "serial_number": actual_serial})
    previous_sequence = 0
    while True:
        started = time.monotonic()
        if not driver.acquire_info("joint_gripper", wait=True, timeout=timeout):
            raise RuntimeError("Joint feedback timed out; no further states will be sent")
        state, sequence, received_monotonic = monitor.snapshot()
        if state is None or not math.isfinite(state.timestamp):
            raise RuntimeError("Missing joint feedback timestamp")
        age = time.monotonic() - received_monotonic
        if sequence <= previous_sequence or not 0 <= age <= timeout:
            raise RuntimeError(
                f"Joint feedback is stale: frame={sequence}, previous={previous_sequence}, "
                f"receipt_age={age:.3f}s"
            )
        angles = list(state.angles)
        if len(angles) != 6 or not all(math.isfinite(x) for x in angles):
            raise RuntimeError("Invalid joint feedback")
        emit(
            {
                "type": "state",
                "timestamp": state.timestamp,
                "sequence": sequence,
                "received_monotonic": received_monotonic,
                "angles": angles,
                "gripper": state.gripper,
            }
        )
        previous_sequence = sequence
        time.sleep(max(0.0, 1.0 / rate - (time.monotonic() - started)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-path", required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--expected-serial", default="")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not 0 < args.rate <= 20:
        parser.error("rate must be in (0, 20] Hz")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be positive")
    if not Path(args.port).is_absolute():
        parser.error("port must be an absolute device path")
    sdk_path = Path(args.sdk_path).expanduser().resolve()
    if not (sdk_path / "alicia_d_sdk" / "hardware" / "servo_driver.py").is_file():
        parser.error("sdk-path must contain the manufacturer's alicia_d_sdk package")
    sys.path.insert(0, str(sdk_path))
    driver = None
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        # Importing the SDK does not connect; use its hardware class, not an adapter
        # whose connect() could invoke set_home(). Install the guard before connect.
        from alicia_d_sdk.hardware import ServoDriver

        driver = ServoDriver(port=args.port, debug_mode=False)
        restrict_queries(driver.serial_comm, args.port)
        monitor = FeedbackMonitor(driver.data_parser)
        if not driver.connect():
            raise RuntimeError("Could not connect to the specified serial port")
        read_states(driver, args.expected_serial, args.rate, args.timeout, monitor)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        emit({"type": "error", "message": str(exc)})
        return 1
    finally:
        if driver is not None:
            driver.disconnect()


if __name__ == "__main__":
    sys.exit(main())
