from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import serial
except Exception as exc:  # pragma: no cover
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


class UnoSerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("uno_serial_bridge_node")

        self.declare_parameter("angle_command_topic", "/control/angle_command")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.02)
        self.declare_parameter("min_send_interval_sec", 0.05)

        self.declare_parameter("command_mode", "angle")
        self.declare_parameter("steps_per_revolution", 4096.0)
        self.declare_parameter("min_effective_step", 1)

        self.declare_parameter("drop_commands_while_busy", True)
        self.declare_parameter("busy_hold_sec", 0.20)
        self.declare_parameter("use_done_feedback", True)

        self.declare_parameter("log_serial_tx", True)
        self.declare_parameter("log_serial_rx", True)

        self._load_params()

        if serial is None:
            raise RuntimeError(f"pyserial is not available: {SERIAL_IMPORT_ERROR}")

        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout_sec,
            )
            time.sleep(0.2)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as exc:
            raise RuntimeError(
                f"Failed to open serial port {self.serial_port} at {self.baud_rate} baud: {exc}"
            ) from exc

        self._last_send_monotonic = 0.0
        self._busy = False
        self._busy_until_monotonic = 0.0

        self._angle_subscription = self.create_subscription(
            Float32,
            self.angle_command_topic,
            self._angle_command_callback,
            10,
        )
        self._serial_timer = self.create_timer(0.02, self._poll_serial)

        self.get_logger().info(
            f"Started uno_serial_bridge_node | serial_port={self.serial_port} | "
            f"command_mode={self.command_mode} | angle_command_topic={self.angle_command_topic}"
        )

    def _load_params(self) -> None:
        self.angle_command_topic = str(self.get_parameter("angle_command_topic").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.serial_timeout_sec = float(self.get_parameter("serial_timeout_sec").value)
        self.min_send_interval_sec = float(self.get_parameter("min_send_interval_sec").value)

        self.command_mode = str(self.get_parameter("command_mode").value).strip().lower()
        self.steps_per_revolution = float(self.get_parameter("steps_per_revolution").value)
        self.min_effective_step = max(1, int(self.get_parameter("min_effective_step").value))

        self.drop_commands_while_busy = bool(self.get_parameter("drop_commands_while_busy").value)
        self.busy_hold_sec = float(self.get_parameter("busy_hold_sec").value)
        self.use_done_feedback = bool(self.get_parameter("use_done_feedback").value)

        self.log_serial_tx = bool(self.get_parameter("log_serial_tx").value)
        self.log_serial_rx = bool(self.get_parameter("log_serial_rx").value)

    def _angle_command_callback(self, msg: Float32) -> None:
        angle_delta_deg = float(msg.data)
        if abs(angle_delta_deg) < 1e-6:
            return

        now_mono = time.monotonic()

        if self.drop_commands_while_busy and self._busy:
            return

        if (now_mono - self._last_send_monotonic) < self.min_send_interval_sec:
            return

        command = self._format_command(angle_delta_deg)
        if command is None:
            return

        self._send_command(command)
        self._last_send_monotonic = now_mono

        if self.use_done_feedback or self.busy_hold_sec > 0.0:
            self._busy = True
            self._busy_until_monotonic = now_mono + self.busy_hold_sec

    def _format_command(self, angle_delta_deg: float) -> str | None:
        if self.command_mode == "step":
            step_count = int(round((angle_delta_deg / 360.0) * self.steps_per_revolution))
            if step_count == 0:
                step_count = self.min_effective_step if angle_delta_deg > 0.0 else -self.min_effective_step
            return f"STEP {step_count}"

        return f"ANG {angle_delta_deg:.3f}"

    def _send_command(self, command: str) -> None:
        line = f"{command}\n".encode("ascii", errors="ignore")
        self._serial.write(line)
        self._serial.flush()

        if self.log_serial_tx:
            self.get_logger().info(f"[uno tx] {command}")

    def _poll_serial(self) -> None:
        if self._busy and self.busy_hold_sec > 0.0 and time.monotonic() >= self._busy_until_monotonic:
            self._busy = False

        try:
            while self._serial.in_waiting > 0:
                raw = self._serial.readline()
                if not raw:
                    break

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if self.log_serial_rx:
                    self.get_logger().info(f"[uno rx] {line}")

                if self.use_done_feedback and (
                    line.startswith("DONE")
                    or line.startswith("STOPPED")
                    or line.startswith("OK STOP")
                ):
                    self._busy = False
        except Exception as exc:
            self.get_logger().error(f"Serial polling error: {exc}")

    def destroy_node(self):
        try:
            if hasattr(self, "_serial") and self._serial is not None and self._serial.is_open:
                self._serial.close()
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