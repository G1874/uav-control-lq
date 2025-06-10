#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
import numpy as np
from geometry_msgs.msg import Point
from px4_msgs.msg import ActuatorMotors
from scipy.spatial.transform import Rotation as R


K = np.loadtxt("/home/developer/ros2_ws/src/math/K.txt", dtype=np.float32, delimiter=',')

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.motor_publisher = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)

        # Create subscribers
        self.odometry_subscriber = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)

        # Initialize variables
        x_d = 0.0
        y_d = 0.0
        z_d = 0.0
        yaw_d = 0.0

        self.x_ref = np.array([x_d, y_d, z_d, 0, 0, 0, 0, 0, yaw_d, 0, 0, 0])

        self.offboard_setpoint_counter = 0
        self.x = np.zeros(12)
        self.position = np.zeros(3)
        self.q = np.zeros(4)
        self.lin_vel = np.zeros(3)
        self.ang_vel = np.zeros(3)
        self.angle = np.zeros(3)

        self.sensor_active = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(100e-6, self.timer_callback)

    def odometry_callback(self, msg:VehicleOdometry):
        self.position[0] = msg.position[0]
        self.position[1] = msg.position[1]
        self.position[2] = msg.position[2]
        self.q[0] = msg.q[0]
        self.q[1] = msg.q[1]
        self.q[2] = msg.q[2]
        self.q[3] = msg.q[3]
        self.ang_vel[0] = msg.angular_velocity[0]
        self.ang_vel[1] = msg.angular_velocity[1]   # This axis is inverted
        self.ang_vel[2] = msg.angular_velocity[2]
        self.lin_vel[0] = msg.velocity[0]
        self.lin_vel[1] = msg.velocity[1]
        self.lin_vel[2] = msg.velocity[2]

        rot = R.from_quat([self.q[1], self.q[2], self.q[3], self.q[0]])  # scipy: [x, y, z, w]
        euler = rot.as_euler('xyz', degrees=False)  # zwraca: roll, pitch, yaw

        self.angle[0] = euler[0]
        self.angle[1] = euler[1]           # This axis is inverted
        self.angle[2] = euler[2] - np.pi/2  # Yaw is 90 deg in starting position

        self.x = np.concatenate([self.position, self.lin_vel, self.angle, self.ang_vel])

        if self.sensor_active == 0:
            self.x_ref = self.x
            self.x_ref[2] = -10
        self.sensor_active = 1

        # self.get_logger().info(f'MACIERZ X : {self.x}')
        # self.get_logger().info(f'roll={np.degrees(self.angle[0]):.1f}, pitch={np.degrees(self.angle[1]):.1f}, yaw={np.degrees(self.angle[2]):.1f}')
        # self.get_logger().info(f'Pozycja: x={self.position[0]:.2f}, y={self.position[1]:.2f}, z={self.position[2]:.2f}')

    def control_loop(self):
        msg = ActuatorMotors()
        now = self.get_clock().now().nanoseconds // 1000  # w mikrosekundach
        msg.timestamp = now
        msg.timestamp_sample = now

        msg.reversible_flags = 0  # jeśli nie masz silników z odwracalnym ciągiem

        x_error = self.x - self.x_ref

        u = -K @ x_error
        T = u / 1e6
        T[T > 1] = 1
        T[T < 0] = 0

        if self.sensor_active == 1:
            self.get_logger().info(f'x: {self.x}')
            self.get_logger().info(f'x_error: {x_error}')
            self.get_logger().info(f'u: {u}')
            self.get_logger().info(f'T: {T}')
            self.get_logger().info(f'x_ref: {self.x_ref}')
            msg.control = [T[0], T[1], T[2], T[3]] + [float('nan')] * 8
            # msg.control = [1.0, 0.5, 1.0, 0.5] + [float('nan')] * 8

        self.motor_publisher.publish(msg)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        self.control_loop()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)