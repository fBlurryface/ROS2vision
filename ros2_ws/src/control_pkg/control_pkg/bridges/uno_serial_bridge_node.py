from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    import serial
    from serial import SerialException
except Exception as exc:  # pragma: no cover
    serial = None
    SerialException = Exception
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


class UnoSerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("uno_serial_bridge_node")

        self.declare_parameter("angle_command_topic", "/control/angle_command")
        self.declare_parameter(
            "serial_port",
            "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        )
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.05)
        self.declare_parameter("min_send_interval_sec", 0.05)
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("startup_reset_wait_sec", 2.0)

        self.declare_parameter("require_handshake", True)
        self.declare_parameter("handshake_command", "PING")
        self.declare_parameter("handshake_expect", "OK PONG")
        self.declare_parameter("handshake_timeout_sec", 3.0)

        self.declare_parameter("command_mode", "angle")
        self.declare_parameter("steps_per_revolution", 4096.0)
        self.declare_parameter("min_effective_step", 1)

        self.declare_parameter("drop_commands_while_busy", True)
        self.declare_parameter("busy_hold_sec", 0.20)
        self.declare_parameter("use_done_feedback", True)

        self.declare_parameter("enable_raw_command_topic", True)
        self.declare_parameter("raw_command_topic", "/control/raw_serial_command")

        self.declare_parameter("log_serial_tx", True)
        self.declare_parameter("log_serial_rx", True)

        self._load_params()

        if serial is None:
            raise RuntimeError(f"pyserial is not available: {SERIAL_IMPORT_ERROR}")

        self._serial: Optional[serial.Serial] = None
        self._connected = False
        self._last_connect_attempt_monotonic = 0.0
        self._last_send_monotonic = 0.0
        self._busy = False
        self._busy_until_monotonic = 0.0

        self._angle_subscription = self.create_subscription(
            Float32,
            self.angle_command_topic,
            self._angle_command_callback,
            10,
        )

        self._raw_subscription = None
        if self.enable_raw_command_topic:
            self._raw_subscription = self.create_subscription(
                String,
                self.raw_command_topic,
                self._raw_command_callback,
                10,
            )

        self._serial_timer = self.create_timer(0.02, self._poll_serial)
        self._connection_timer = self.create_timer(0.25, self._connection_timer_callback)

        self.get_logger().info(
            "Started uno_serial_bridge_node | "
            f"serial_port={self.serial_port} | command_mode={self.command_mode} | "
            f"angle_command_topic={self.angle_command_topic} | "
            f"raw_command_topic={self.raw_command_topic if self.enable_raw_command_topic else 'disabled'}"
        )

        self._try_connect(force=True)

    def _load_params(self) -> None:
        self.angle_command_topic = str(self.get_parameter("angle_command_topic").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.serial_timeout_sec = float(self.get_parameter("serial_timeout_sec").value)
        self.min_send_interval_sec = float(self.get_parameter("min_send_interval_sec").value)
        self.reconnect_interval_sec = float(self.get_parameter("reconnect_interval_sec").value)
        self.startup_reset_wait_sec = float(self.get_parameter("startup_reset_wait_sec").value)

        self.require_handshake = bool(self.get_parameter("require_handshake").value)
        self.handshake_command = str(self.get_parameter("handshake_command").value).strip()
        self.handshake_expect = str(self.get_parameter("handshake_expect").value).strip()
        self.handshake_timeout_sec = float(self.get_parameter("handshake_timeout_sec").value)

        self.command_mode = str(self.get_parameter("command_mode").value).strip().lower()
        self.steps_per_revolution = float(self.get_parameter("steps_per_revolution").value)
        self.min_effective_step = max(1, int(self.get_parameter("min_effective_step").value))

        self.drop_commands_while_busy = bool(self.get_parameter("drop_commands_while_busy").value)
        self.busy_hold_sec = float(self.get_parameter("busy_hold_sec").value)
        self.use_done_feedback = bool(self.get_parameter("use_done_feedback").value)

        self.enable_raw_command_topic = bool(self.get_parameter("enable_raw_command_topic").value)
        self.raw_command_topic = str(self.get_parameter("raw_command_topic").value)

        self.log_serial_tx = bool(self.get_parameter("log_serial_tx").value)
        self.log_serial_rx = bool(self.get_parameter("log_serial_rx").value)

    def _connection_timer_callback(self) -> None:
        if self._connected:
            return
        self._try_connect()

    def _try_connect(self, *, force: bool = False) -> None:
        now_mono = time.monotonic()
        if not force and (now_mono - self._last_connect_attempt_monotonic) < self.reconnect_interval_sec:
            return

        self._last_connect_attempt_monotonic = now_mono
        self.get_logger().info(
            f"Attempting UNO serial connection on {self.serial_port} at {self.baud_rate} baud"
        )

        self._close_serial()

        try:
            candidate = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout_sec,
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to open serial port {self.serial_port}: {exc}")
            return

        self._serial = candidate
        self._connected = False
        self._busy = False
        self._busy_until_monotonic = 0.0

        if self.startup_reset_wait_sec > 0.0:
            time.sleep(self.startup_reset_wait_sec)

        try:
            if self._serial is not None:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
        except Exception as exc:
            self._mark_disconnected(f"Failed to reset serial buffers: {exc}")
            return

        if self.require_handshake and not self._perform_handshake():
            self._mark_disconnected(
                f"Handshake failed on {self.serial_port}; expected '{self.handshake_expect}'"
            )
            return

        self._connected = True
        self.get_logger().info(f"UNO serial connected on {self.serial_port}")

    def _perform_handshake(self) -> bool:
        if not self.handshake_command:
            self.get_logger().warn("Handshake requested but handshake_command is empty")
            return False

        try:
            self._send_line(self.handshake_command)
        except Exception as exc:
            self.get_logger().warn(f"Failed to send handshake command: {exc}")
            return False

        deadline = time.monotonic() + self.handshake_timeout_sec
        while time.monotonic() < deadline:
            line = self._read_line()
            if line is None:
                continue

            if self.log_serial_rx:
                self.get_logger().info(f"[uno rx] {line}")

            if line == self.handshake_expect:
                return True

        return False

    def _angle_command_callback(self, msg: Float32) -> None:
        angle_delta_deg = float(msg.data)
        if abs(angle_delta_deg) < 1e-6:
            return

        if not self._connected:
            self.get_logger().warn("Ignoring angle command because UNO serial is disconnected")
            return

        now_mono = time.monotonic()

        if self.drop_commands_while_busy and self._busy:
            return

        if (now_mono - self._last_send_monotonic) < self.min_send_interval_sec:
            return

        command = self._format_command(angle_delta_deg)
        if command is None:
            return

        if not self._send_command(command):
            return

        self._last_send_monotonic = now_mono

        if self.use_done_feedback or self.busy_hold_sec > 0.0:
            self._busy = True
            self._busy_until_monotonic = now_mono + self.busy_hold_sec

    def _raw_command_callback(self, msg: String) -> None:
        command = str(msg.data).strip()
        if not command:
            return

        if not self._connected:
            self.get_logger().warn(f"Ignoring raw command while disconnected: {command}")
            return

        self._send_command(command)

    def _format_command(self, angle_delta_deg: float) -> str | None:
        if self.command_mode == "step":
            step_count = int(round((angle_delta_deg / 360.0) * self.steps_per_revolution))
            if step_count == 0:
                step_count = self.min_effective_step if angle_delta_deg > 0.0 else -self.min_effective_step
            return f"STEP {step_count}"

        return f"ANG {angle_delta_deg:.3f}"

    def _send_command(self, command: str) -> bool:
        try:
            self._send_line(command)
        except Exception as exc:
            self._mark_disconnected(f"Serial write failed for '{command}': {exc}")
            return False

        if self.log_serial_tx:
            self.get_logger().info(f"[uno tx] {command}")
        return True

    def _send_line(self, command: str) -> None:
        if self._serial is None or not self._serial.is_open:
            raise SerialException("serial port is not open")

        line = f"{command}\n".encode("ascii", errors="ignore")
        self._serial.write(line)
        self._serial.flush()

    def _read_line(self) -> str | None:
        if self._serial is None or not self._serial.is_open:
            raise SerialException("serial port is not open")

        raw = self._serial.readline()
        if not raw:
            return None

        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            return None
        return line

    def _poll_serial(self) -> None:
        if not self._connected or self._serial is None:
            return

        if self._busy and self.busy_hold_sec > 0.0 and time.monotonic() >= self._busy_until_monotonic:
            self._busy = False

        try:
            while self._serial.in_waiting > 0:
                line = self._read_line()
                if line is None:
                    break

                if self.log_serial_rx:
                    self.get_logger().info(f"[uno rx] {line}")

                if self.use_done_feedback and (
                    line.startswith("DONE")
                    or line.startswith("STOPPED")
                    or line.startswith("OK STOP")
                ):
                    self._busy = False
        except Exception as exc:
            self._mark_disconnected(f"Serial polling error: {exc}")

    def _mark_disconnected(self, reason: str) -> None:
        if self._connected:
            self.get_logger().warn(reason)
        else:
            self.get_logger().warn(reason)

        self._connected = False
        self._busy = False
        self._busy_until_monotonic = 0.0
        self._close_serial()

    def _close_serial(self) -> None:
        try:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        finally:
            self._serial = None

    def destroy_node(self):
        try:
            self._close_serial()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnoSerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()