#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class LiftGazeboController(Node):

    def __init__(self):
        super().__init__("lift_gazebo_controller")

        # ================================================================
        # MULTI-ROBOT TOPIC PREFIX PARAMETER
        #
        # Example:
        #   ""       -> /lift_command, /lift_position, ...
        #   "/amr_1" -> /amr_1/lift_command, /amr_1/lift_position, ...
        #   "/amr_2" -> /amr_2/lift_command, /amr_2/lift_position, ...
        # ================================================================

        self.declare_parameter("topic_prefix", "")
        raw_prefix = (
            self.get_parameter("topic_prefix")
            .get_parameter_value()
            .string_value
            .strip()
        )
        if raw_prefix and not raw_prefix.startswith("/"):
            raw_prefix = "/" + raw_prefix
        self.prefix = raw_prefix.rstrip("/")

        self.max_lift_travel = 0.290

        cmd_topic = f"{self.prefix}/lift_command" if self.prefix else "/lift_command"
        lift_pos_topic = f"{self.prefix}/lift_position" if self.prefix else "/lift_position"

        # ================================================================
        # LIFT COMMAND SUBSCRIBER
        #
        # Accepted range:
        # 0.000 m to 0.290 m (Level 1 = 0.133 m, Level 2 = 0.268 m)
        # ================================================================

        self.command_subscription = self.create_subscription(
            Float64,
            cmd_topic,
            self.command_callback,
            10,
        )

        # Legacy fallback subscription on /lift_command for amr_1
        self.legacy_subscription = None
        if self.prefix == "/amr_1":
            self.legacy_subscription = self.create_subscription(
                Float64,
                "/lift_command",
                self.command_callback,
                10,
            )

        # ================================================================
        # MAIN VERTICAL LIFT PUBLISHER
        # ================================================================

        self.lift_publisher = self.create_publisher(
            Float64,
            lift_pos_topic,
            10,
        )

        # ================================================================
        # FOUR SCISSOR-JOINT PUBLISHERS
        # ================================================================

        scissor_suffixes = [
            "scissor_left_a_position",
            "scissor_left_b_position",
            "scissor_right_a_position",
            "scissor_right_b_position",
        ]

        self.scissor_publishers = [
            self.create_publisher(
                Float64,
                f"{self.prefix}/{suffix}" if self.prefix else f"/{suffix}",
                10,
            )
            for suffix in scissor_suffixes
        ]


        # ================================================================
        # CURRENT TARGET
        # ================================================================

        self.target_lift = 0.0
        self.last_reported_lift = None


        # ================================================================
        # SCISSOR GEOMETRY
        #
        # OpenSCAD geometry converted from millimeters to meters.
        #
        # Horizontal pivot distance:
        # 270 mm = 0.270 m
        #
        # Initial vertical difference:
        # 42 mm = 0.042 m
        # ================================================================

        self.horizontal_distance = 0.270
        self.initial_vertical_distance = 0.042


        # ================================================================
        # CONTINUOUS COMMAND TIMER
        #
        # Keep publishing targets at 30 Hz so Gazebo controllers continue
        # holding the lift and scissor-joint positions.
        # ================================================================

        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_commands,
        )

        self.get_logger().info(
            f"Synchronized Gazebo lift controller started for '{self.prefix or '/'}'."
        )

        self.get_logger().info(
            f"Input topic: {cmd_topic}"
        )

        self.get_logger().info(
            f"Lift range: 0.000 m to {self.max_lift_travel:.3f} m"
        )


    # ====================================================================
    # LIFT COMMAND CALLBACK
    # ====================================================================

    def command_callback(self, message):
        if not math.isfinite(message.data):
            self.get_logger().warning(
                "Ignored non-finite lift command."
            )
            return

        # Clamp the command to the physical lift range.
        self.target_lift = max(
            0.0,
            min(
                self.max_lift_travel,
                message.data,
            ),
        )

        rounded_lift = round(
            self.target_lift,
            3,
        )

        if rounded_lift != self.last_reported_lift:
            scissor_angle = self.calculate_scissor_angle(
                self.target_lift
            )

            self.get_logger().info(
                f"Lift target = {self.target_lift:.3f} m, "
                f"scissor target = {scissor_angle:.3f} rad"
            )

            self.last_reported_lift = rounded_lift


    # ====================================================================
    # CALCULATE SCISSOR ANGLE
    #
    # angle = atan2(vertical distance, horizontal distance)
    #
    # At lift_position = 0.000 m:
    # angle = atan2(0.042, 0.270)
    # angle is approximately 0.154 rad.
    #
    # At lift_position = 0.180 m:
    # angle = atan2(0.222, 0.270)
    # angle is approximately 0.688 rad.
    # ====================================================================

    def calculate_scissor_angle(
        self,
        lift_position,
    ):
        vertical_distance = (
            self.initial_vertical_distance
            + lift_position
        )

        return math.atan2(
            vertical_distance,
            self.horizontal_distance,
        )


    # ====================================================================
    # PUBLISH SYNCHRONIZED COMMANDS
    # ====================================================================

    def publish_commands(self):
        # Main lift platform command.
        lift_message = Float64()
        lift_message.data = self.target_lift

        self.lift_publisher.publish(
            lift_message
        )

        # Actual nonlinear angle for four scissor bars.
        scissor_angle = self.calculate_scissor_angle(
            self.target_lift
        )

        angle_message = Float64()
        angle_message.data = scissor_angle

        for publisher in self.scissor_publishers:
            publisher.publish(
                angle_message
            )


def main(args=None):
    rclpy.init(args=args)

    node = LiftGazeboController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()