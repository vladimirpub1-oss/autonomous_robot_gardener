#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTargetExperiment(Node):
    """Move the xArm base joint and verify the measured result."""

    def __init__(self):
        super().__init__('joint_target_experiment')

        self.declare_parameter('target_angle', 0.35)
        self.declare_parameter('tolerance', 0.02)
        self.declare_parameter('velocity_tolerance', 0.01)
        self.declare_parameter('hold_time', 0.5)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('movement_time', 3.0)

        self.target_angle = float(
            self.get_parameter('target_angle').value
        )
        self.tolerance = float(
            self.get_parameter('tolerance').value
        )

        self.velocity_tolerance = float(
            self.get_parameter('velocity_tolerance').value
        )

        self.hold_time = float(
            self.get_parameter('hold_time').value
        )
        self.timeout = float(
            self.get_parameter('timeout').value
        )
        self.movement_time = float(
            self.get_parameter('movement_time').value
        )

        self.controlled_joint = 'xarm_6_joint'
        self.joint_names = [
            'xarm_1_joint',
            'xarm_2_joint',
            'xarm_3_joint',
            'xarm_4_joint',
            'xarm_5_joint',
            'xarm_6_joint',
        ]

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/hiwonder_xarm_controller/joint_trajectory',
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.created_at = time.monotonic()
        self.command_sent_at = None
        self.hold_started_at = None
        self.done = False
        self.success = False

        self.get_logger().info(
            f'Ready: target {self.controlled_joint} = '
            f'{self.target_angle:.3f} rad'
        )
        self.get_logger().info('Waiting for /joint_states...')

    def joint_state_callback(self, msg):
        if self.done:
            return

        positions_by_name = dict(zip(msg.name, msg.position))

        velocities_by_name = dict(zip(msg.name, msg.velocity))

        missing = [
            name
            for name in self.joint_names
            if name not in positions_by_name
        ]
        if missing:
            return

        if self.command_sent_at is None:
            positions = [
                positions_by_name[name]
                for name in self.joint_names
            ]
            target_index = self.joint_names.index(
                self.controlled_joint
            )
            positions[target_index] = self.target_angle

            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(self.movement_time)
            point.time_from_start.nanosec = int(
                (self.movement_time % 1.0) * 1_000_000_000
            )
            trajectory.points = [point]

            self.publisher.publish(trajectory)
            self.command_sent_at = time.monotonic()

            self.get_logger().info(
                f'Command sent: {self.controlled_joint} -> '
                f'{self.target_angle:.3f} rad in '
                f'{self.movement_time:.1f} s'
            )
            return

        actual_angle = positions_by_name[self.controlled_joint]
        actual_velocity = velocities_by_name.get(
            self.controlled_joint,
            float('inf'),
        )
        error = abs(self.target_angle - actual_angle)

        position_ok = error <= self.tolerance
        velocity_ok = abs(actual_velocity) <= self.velocity_tolerance

        if position_ok and velocity_ok:
            if self.hold_started_at is None:
                self.hold_started_at = time.monotonic()

            held_for = time.monotonic() - self.hold_started_at
            if held_for >= self.hold_time:
                self.finish(
                    True,
                    f'target={self.target_angle:.3f}, '
                    f'actual={actual_angle:.3f}, '
                    f'error={error:.4f} rad'
                    f'velocity={actual_velocity:.4f} rad/s',
                )
        else:
            self.hold_started_at = None

    def timer_callback(self):
        if self.done:
            return

        now = time.monotonic()

        if self.command_sent_at is None:
            if now - self.created_at > 5.0:
                self.finish(
                    False,
                    'no complete /joint_states received',
                )
            return

        if now - self.command_sent_at > self.timeout:
            self.finish(
                False,
                'target was not reached before timeout',
            )

    def finish(self, success, message):
        self.done = True
        self.success = success

        if success:
            self.get_logger().info(f'SUCCESS: {message}')
        else:
            self.get_logger().error(f'FAILURE: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = JointTargetExperiment()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Experiment interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()