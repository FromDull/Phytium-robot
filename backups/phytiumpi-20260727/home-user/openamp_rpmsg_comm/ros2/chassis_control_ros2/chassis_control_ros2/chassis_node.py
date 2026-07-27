import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .protocol import (
    BrokerClient,
    CMD_BALANCE_DISABLE,
    CMD_BALANCE_ENABLE,
    CMD_CHASSIS_SET_TRACK_WIDTH,
    CMD_CHASSIS_SET_VELOCITY,
    CMD_CHASSIS_STATUS,
    decode_chassis_telemetry,
    track_width_payload,
    velocity_payload,
)


BALANCE_DISABLED = 0
BALANCE_ACTIVE = 2


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ChassisNode(Node):
    def __init__(self) -> None:
        super().__init__("openamp_chassis")
        self.declare_parameter("broker_socket", "/run/rpmsg-broker/rpmsg.sock")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("command_timeout_ms", 300)
        self.declare_parameter("cmd_vel_timeout_s", 0.25)
        self.declare_parameter("max_linear_m_s", 0.20)
        self.declare_parameter("max_angular_rad_s", 0.50)
        self.declare_parameter("wheel_track_m", 0.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        socket_path = str(self.get_parameter("broker_socket").value)
        rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.command_timeout_ms = int(
            self.get_parameter("command_timeout_ms").value)
        self.cmd_vel_timeout_s = float(
            self.get_parameter("cmd_vel_timeout_s").value)
        self.max_linear_m_s = float(
            self.get_parameter("max_linear_m_s").value)
        self.max_angular_rad_s = float(
            self.get_parameter("max_angular_rad_s").value)
        self.configured_track_m = float(
            self.get_parameter("wheel_track_m").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        if not 100 <= self.command_timeout_ms <= 1000:
            raise ValueError("command_timeout_ms must be 100..1000")
        if self.configured_track_m != 0.0 and not (
                0.08 <= self.configured_track_m <= 0.50):
            raise ValueError("wheel_track_m must be zero or 0.08..0.50")

        self.broker = BrokerClient(socket_path)
        self.latest_command = Twist()
        self.latest_command_time = 0.0
        self.telemetry = None
        self.track_configured = False
        self.last_update_time = time.monotonic()
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.previous_balance_state = BALANCE_DISABLED
        self.last_error = ""

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_service(Trigger, "chassis/enable", self._on_enable)
        self.create_service(Trigger, "chassis/disable", self._on_disable)
        self.timer = self.create_timer(1.0 / rate_hz, self._update)
        self.get_logger().info(
            f"OpenAMP chassis bridge started: socket={socket_path}, rate={rate_hz:.1f} Hz")
        if self.configured_track_m <= 0.0:
            self.get_logger().warning(
                "wheel_track_m is unset; angular.z is forced to zero")

    def destroy_node(self):
        try:
            if self.telemetry is not None and self.telemetry.balance_state == BALANCE_ACTIVE:
                payload = velocity_payload(0.0, 0.0, 100)
                self.broker.request(CMD_CHASSIS_SET_VELOCITY, payload)
        except (OSError, ValueError):
            pass
        self.broker.close()
        return super().destroy_node()

    def _on_cmd_vel(self, message: Twist) -> None:
        self.latest_command = message
        self.latest_command_time = time.monotonic()

    def _trigger(self, command: int, response: Trigger.Response) -> Trigger.Response:
        try:
            self.broker.request(command)
            response.success = True
            response.message = "command accepted by remote core"
        except (OSError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _on_enable(self, _request, response):
        return self._trigger(CMD_BALANCE_ENABLE, response)

    def _on_disable(self, _request, response):
        return self._trigger(CMD_BALANCE_DISABLE, response)

    def _command(self) -> tuple[float, float]:
        age = time.monotonic() - self.latest_command_time
        if age > self.cmd_vel_timeout_s:
            return 0.0, 0.0
        linear = clamp(self.latest_command.linear.x, self.max_linear_m_s)
        angular = clamp(self.latest_command.angular.z, self.max_angular_rad_s)
        if not self.track_configured:
            angular = 0.0
        return linear, angular

    def _request_telemetry(self):
        if self.telemetry is not None and self.telemetry.balance_state == BALANCE_ACTIVE:
            linear, angular = self._command()
            command = CMD_CHASSIS_SET_VELOCITY
            payload = velocity_payload(linear, angular, self.command_timeout_ms)
        elif (self.configured_track_m > 0.0 and not self.track_configured and
              (self.telemetry is None or
               self.telemetry.balance_state == BALANCE_DISABLED)):
            command = CMD_CHASSIS_SET_TRACK_WIDTH
            payload = track_width_payload(self.configured_track_m)
        else:
            command = CMD_CHASSIS_STATUS
            payload = b""

        reply_command, reply_payload = self.broker.request(command, payload)
        if reply_command != command:
            raise ValueError(
                f"unexpected reply type {reply_command}, expected {command}")
        telemetry = decode_chassis_telemetry(reply_payload)
        if command == CMD_CHASSIS_SET_TRACK_WIDTH and telemetry.status == 0:
            self.track_configured = True
            self.get_logger().info(
                f"remote wheel track configured: {telemetry.wheel_track_m:.4f} m")
        else:
            self.track_configured = telemetry.wheel_track_m >= 0.08
        return telemetry

    def _update(self) -> None:
        now_monotonic = time.monotonic()
        try:
            telemetry = self._request_telemetry()
            self.last_error = ""
        except (OSError, ValueError) as error:
            self.last_error = str(error)
            self._publish_diagnostics(None)
            return

        dt = max(0.0, min(0.2, now_monotonic - self.last_update_time))
        self.last_update_time = now_monotonic
        if telemetry.balance_state != self.previous_balance_state:
            if telemetry.balance_state == BALANCE_ACTIVE:
                self.x_m = 0.0
                self.y_m = 0.0
                self.yaw_rad = 0.0
            self.previous_balance_state = telemetry.balance_state
        if telemetry.balance_state == BALANCE_ACTIVE:
            self.x_m += telemetry.measured_linear_m_s * math.cos(self.yaw_rad) * dt
            self.y_m += telemetry.measured_linear_m_s * math.sin(self.yaw_rad) * dt
            self.yaw_rad += telemetry.measured_angular_rad_s * dt
        self.telemetry = telemetry
        self._publish_odometry(telemetry)
        self._publish_diagnostics(telemetry)

    def _publish_odometry(self, telemetry) -> None:
        stamp = self.get_clock().now().to_msg()
        quaternion = yaw_quaternion(self.yaw_rad)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x_m
        message.pose.pose.position.y = self.y_m
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        message.twist.twist.linear.x = telemetry.measured_linear_m_s
        message.twist.twist.angular.z = telemetry.measured_angular_rad_s
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.05
        message.twist.covariance[0] = 0.03
        message.twist.covariance[35] = 0.08
        self.odom_publisher.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x_m
            transform.transform.translation.y = self.y_m
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _publish_diagnostics(self, telemetry) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "openamp_chassis"
        status.hardware_id = "phytium-openamp"
        if telemetry is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self.last_error or "telemetry unavailable"
        elif telemetry.fault:
            status.level = DiagnosticStatus.ERROR
            status.message = f"balance fault 0x{telemetry.fault:02x}"
        elif telemetry.balance_state != BALANCE_ACTIVE:
            status.level = DiagnosticStatus.WARN
            status.message = f"balance state {telemetry.balance_state}"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "active"
        if telemetry is not None:
            status.values = [
                KeyValue(key="balance_state", value=str(telemetry.balance_state)),
                KeyValue(key="fault", value=f"0x{telemetry.fault:02x}"),
                KeyValue(key="linear_m_s", value=f"{telemetry.measured_linear_m_s:.4f}"),
                KeyValue(key="angular_rad_s", value=f"{telemetry.measured_angular_rad_s:.4f}"),
                KeyValue(key="wheel_track_m", value=f"{telemetry.wheel_track_m:.4f}"),
            ]
        array.status.append(status)
        self.diagnostics_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
