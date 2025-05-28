#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
import numpy as np
from geometry_msgs.msg import Point
from px4_msgs.msg import ActuatorMotors
from scipy.spatial.transform import Rotation as R


# K = np.array([
#     [0.707106781186554,-4.04575826675283e-14,0.500000000000021,9.65124390198225,-4.69753598794366e-13,241.846017472289,-2.62020819062859e-11,-642.662661463822,-0.499999999999972,-7.46061552958981e-11,-2552.52854352887,-11.1469112684321],
#     [-1.09267289350883e-15,0.707106781186479,0.499999999999985,-6.81929670246177e-15,9.65124390198137,241.846017472264,642.662661463773,7.05172982938684e-14,0.500000000000038,2552.52854352876,-1.64107995953571e-12,11.1469112684335],
#     [-0.70710678118655,4.67505910481469e-14,0.499999999999975,-9.6512439019822,5.28277997181896e-13,241.84601747226,2.8791758547635e-11,642.662661463819,-0.500000000000025,7.95444915872978e-11,2552.52854352886,-11.1469112684332],
#     [-4.85195299088364e-15,-0.707106781186485,0.500000000000024,-5.97915182018812e-14,-9.65124390198143,241.846017472287,-642.662661463776,3.56791840481906e-12,0.499999999999961,-2552.52854352876,1.15949102685787e-11,11.1469112684318],
# ])


K = np.array([
    [0.499999999999838,-0.500000000000147,0.500000000000005,6.82446000997512,-6.82446000997795,241.846017472279,-454.431125936495,-454.431125936371,-0.500000000000009,-1804.91024230154,-1804.91024230129,-11.1469112684323],
    [-0.499999999999836,0.500000000000145,0.5,-6.82446000997511,6.82446000997793,241.846017472265,454.431125936495,454.43112593637,-0.499999999999988,1804.91024230154,1804.91024230129,-11.1469112684329],
    [0.49999999999984,0.500000000000178,0.500000000000017,6.82446000997515,6.82446000997835,241.84601747227,454.43112593652,-454.431125936372,0.500000000000017,1804.91024230162,-1804.91024230129,11.1469112684325],
    [-0.499999999999841,-0.500000000000174,0.500000000000062,-6.82446000997515,-6.82446000997831,241.846017472296,-454.431125936518,454.431125936372,0.49999999999998,-1804.91024230161,1804.91024230129,11.1469112684327],
])

x_d = 0.0
y_d = 0.0
z_d = 1.0
yaw_d = 0.0

x_ref = np.array([x_d, y_d, z_d, yaw_d, 0, 0, 0, 0, 0, 0, 0, 0])


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

        self.x = np.zeros(12)
        self.position = np.zeros(3)
        self.q = np.zeros(4)
        self.lin_vel = np.zeros(3)
        self.ang_vel = np.zeros(3)
        self.angle = np.zeros(3)

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # self.trajectory_setpoint_publisher = self.create_publisher(
        #     TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # self.vel_setpoint_publisher = self.create_publisher() # TODO

        self.motor_publisher = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)



        # Create subscribers
        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        # self.vehicle_status_subscriber = self.create_subscription(
        #     VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.odometry_subscriber = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile) # TODO

        # Initialize variables
        self.offboard_setpoint_counter = 0
        # TODO: Add initialization of vehicle position

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odometry_callback(self, msg:VehicleOdometry):
        self.position[0] = msg.position[0]
        self.position[1] = msg.position[1]
        self.position[2] = msg.position[2]
        self.q[0] = msg.q[0]
        self.q[1] = msg.q[1]
        self.q[2] = msg.q[2]
        self.q[3] = msg.q[3]
        self.ang_vel[0] = msg.angular_velocity[0]
        self.ang_vel[1] = msg.angular_velocity[1]
        self.ang_vel[2] = msg.angular_velocity[2]
        self.lin_vel[0] = msg.velocity[0]
        self.lin_vel[1] = msg.velocity[1]
        self.lin_vel[2] = msg.velocity[2]

        rot = R.from_quat([self.q[1], self.q[2], self.q[3], self.q[0]])  # scipy: [x, y, z, w]
        euler = rot.as_euler('xyz', degrees=False)  # zwraca: roll, pitch, yaw

        self.angle[0] = euler[0]
        self.angle[1] = euler[1]
        self.angle[2] = euler[2]

        self.x = np.concatenate([self.position, self.lin_vel, self.angle, self.ang_vel])
        # self.get_logger().info(f'MACIERZ X : {self.x}')

        # self.get_logger().info(f'roll={np.degrees(self.angle[0]):.1f}, pitch={np.degrees(self.angle[1]):.1f}, yaw={np.degrees(self.angle[2]):.1f}')
        # self.get_logger().info(f'Pozycja: x={self.position[0]:.2f}, y={self.position[1]:.2f}, z={self.position[2]:.2f}')

    def control_loop(self):
        msg = ActuatorMotors()
        now = self.get_clock().now().nanoseconds // 1000  # w mikrosekundach
        msg.timestamp = now
        msg.timestamp_sample = now

        msg.reversible_flags = 0  # jeśli nie masz silników z odwracalnym ciągiem

        x_error = self.x - x_ref

        u = K @ x_error
        T = u / 100
        # T = u

        self.get_logger().info(f'macierz thrustow: {T}')
        # Przykład: 4 silniki, wartości od 0.1 do 0.1 (mały ciąg, do testu)
        msg.control = [T[0], T[1], T[2], T[3]] + [float('nan')] * 8

        self.motor_publisher.publish(msg)
        # self.get_logger().info(f'Siłanie silników: {[round(x, 2) if not np.isnan(x) else "NaN" for x in msg.control[:4]]}')

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