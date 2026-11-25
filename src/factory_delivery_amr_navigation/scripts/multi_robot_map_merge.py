#!/usr/bin/env python3
"""
Multi-Robot OccupancyGrid Map Merger & Static TF Broadcaster.

Broadcasts static transforms from the global `map` frame to each robot's
local SLAM map frame (`amr_1/map`, `amr_2/map`) using their known initial
spawn poses, and merges `/amr_1/map` and `/amr_2/map` into a unified `/map`
and `/map_metadata` in real time.
"""

import math
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster


class MultiRobotMapMerge(Node):
    """Merge per-robot SLAM OccupancyGrids into a shared global /map."""

    def __init__(self) -> None:
        super().__init__("multi_robot_map_merge")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("merge_rate_hz", 2.0)

        self.declare_parameter("amr_1_init_x", 0.0)
        self.declare_parameter("amr_1_init_y", 0.80)
        self.declare_parameter("amr_1_init_yaw", 0.0)

        self.declare_parameter("amr_2_init_x", 0.0)
        self.declare_parameter("amr_2_init_y", -0.80)
        self.declare_parameter("amr_2_init_yaw", 0.0)

        self.global_frame: str = self.get_parameter("global_frame").value
        merge_rate = float(self.get_parameter("merge_rate_hz").value)

        self.robot_poses: Dict[str, Tuple[float, float, float]] = {
            "amr_1": (
                float(self.get_parameter("amr_1_init_x").value),
                float(self.get_parameter("amr_1_init_y").value),
                float(self.get_parameter("amr_1_init_yaw").value),
            ),
            "amr_2": (
                float(self.get_parameter("amr_2_init_x").value),
                float(self.get_parameter("amr_2_init_y").value),
                float(self.get_parameter("amr_2_init_yaw").value),
            ),
        }

        self.maps: Dict[str, Optional[OccupancyGrid]] = {ns: None for ns in self.robot_poses}
        self._dirty = False

        # Publish static transforms: map -> <robot_ns>/map
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_static_tfs()

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.merged_map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.merged_meta_pub = self.create_publisher(MapMetaData, "/map_metadata", map_qos)

        for ns in self.robot_poses:
            self.create_subscription(
                OccupancyGrid,
                f"/{ns}/map",
                lambda msg, robot=ns: self._on_map(robot, msg),
                map_qos,
            )

        self.create_timer(1.0 / max(merge_rate, 0.2), self._merge_and_publish)
        self.get_logger().info(
            "MultiRobotMapMerge ready: merging /amr_1/map + /amr_2/map -> /map"
        )

    def _broadcast_static_tfs(self) -> None:
        transforms = []
        now = self.get_clock().now().to_msg()
        for ns, (x, y, yaw) in self.robot_poses.items():
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.global_frame
            tf_msg.child_frame_id = f"{ns}/map"
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = math.sin(yaw / 2.0)
            tf_msg.transform.rotation.w = math.cos(yaw / 2.0)
            transforms.append(tf_msg)
        self.tf_broadcaster.sendTransform(transforms)

    def _on_map(self, robot: str, msg: OccupancyGrid) -> None:
        self.maps[robot] = msg
        self._dirty = True

    @staticmethod
    def _grid_bounds_in_global(
        msg: OccupancyGrid, pose: Tuple[float, float, float]
    ) -> Tuple[float, float, float, float]:
        tx, ty, tyaw = pose
        res = msg.info.resolution
        w = msg.info.width
        h = msg.info.height
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # Four corners of the local map in <ns>/map frame
        corners_local = np.array(
            [
                [ox, oy],
                [ox + w * res, oy],
                [ox, oy + h * res],
                [ox + w * res, oy + h * res],
            ],
            dtype=np.float64,
        )
        c, s = math.cos(tyaw), math.sin(tyaw)
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        corners_global = corners_local @ rot.T + np.array([tx, ty], dtype=np.float64)
        return (
            float(np.min(corners_global[:, 0])),
            float(np.min(corners_global[:, 1])),
            float(np.max(corners_global[:, 0])),
            float(np.max(corners_global[:, 1])),
        )

    def _merge_and_publish(self) -> None:
        if not self._dirty:
            return

        active = [(ns, m) for ns, m in self.maps.items() if m is not None and m.info.width > 0 and m.info.height > 0]
        if not active:
            return

        self._dirty = False
        res = active[0][1].info.resolution

        min_x = float("inf")
        min_y = float("inf")
        max_x = float("-inf")
        max_y = float("-inf")

        for ns, msg in active:
            bx0, by0, bx1, by1 = self._grid_bounds_in_global(msg, self.robot_poses[ns])
            min_x = min(min_x, bx0)
            min_y = min(min_y, by0)
            max_x = max(max_x, bx1)
            max_y = max(max_y, by1)

        # Snap origin to resolution grid
        origin_x = math.floor(min_x / res) * res
        origin_y = math.floor(min_y / res) * res
        gw = max(1, int(math.ceil((max_x - origin_x) / res)))
        gh = max(1, int(math.ceil((max_y - origin_y) / res)))

        merged = np.full((gh, gw), -1, dtype=np.int16)

        for ns, msg in active:
            tx, ty, tyaw = self.robot_poses[ns]
            w = msg.info.width
            h = msg.info.height
            r_res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y

            data = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
            known_y, known_x = np.nonzero(data >= 0)
            if known_x.size == 0:
                continue

            vals = data[known_y, known_x]
            lx = ox + (known_x.astype(np.float64) + 0.5) * r_res
            ly = oy + (known_y.astype(np.float64) + 0.5) * r_res

            c, s = math.cos(tyaw), math.sin(tyaw)
            gx = tx + c * lx - s * ly
            gy = ty + s * lx + c * ly

            gix = np.floor((gx - origin_x) / res).astype(np.int32)
            giy = np.floor((gy - origin_y) / res).astype(np.int32)

            valid = (gix >= 0) & (gix < gw) & (giy >= 0) & (giy < gh)
            gix = gix[valid]
            giy = giy[valid]
            vals = vals[valid]

            np.maximum.at(merged, (giy, gix), vals)

        now = self.get_clock().now().to_msg()
        out = OccupancyGrid()
        out.header.stamp = now
        out.header.frame_id = self.global_frame
        out.info.map_load_time = now
        out.info.resolution = res
        out.info.width = gw
        out.info.height = gh
        out.info.origin.position.x = origin_x
        out.info.origin.position.y = origin_y
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = merged.astype(np.int8).ravel().tolist()

        self.merged_map_pub.publish(out)
        self.merged_meta_pub.publish(out.info)


def main() -> None:
    rclpy.init()
    node = MultiRobotMapMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
