#!/usr/bin/env python3
"""Publish mock ROS data for exercising the AVT-341 RViz plugin.

The RViz plugin normally subscribes to topics produced by the full autonomy
stack, which is heavy to bring up. This node fabricates plausible, randomized
data for a handful of vehicles so the plugin's panels can be developed and
tested in isolation.

A single master timer ticks at ``TICK_RATE_HZ``. Each topic declares its own
intended publish rate (up to the tick rate) via a ``TopicSpec``; the rate is
realized by decimating the master tick, so e.g. a 10 Hz topic publishes on
every 3rd tick and a 0.2 Hz topic publishes once every 150 ticks.

Run it (after sourcing the workspace so ``avt_341_msgs`` is importable) with::

    python3 publish_mock_ui_data.py
"""

from __future__ import annotations

import math
import random
import string
from dataclasses import dataclass
from functools import partial
from typing import Any, Callable, Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

from avt_341_msgs.msg import (MapMarker, MapMarkerList, MissionTaskStatus,
                              NavGoal, NavGoalSequence, NavState, TrackerStatus,
                              TrackerModuleStatus)

# Vehicles to publish for. Every topic is published once per vehicle, namespaced
# as ``/<vehicle_id>/<topic>``.
VEHICLE_IDS: List[str] = ["mrzr2", "mrzr4", "fed"]

# Shared world frame for map-aligned data (goals, markers, mission state).
MAP_FRAME_ID: str = "map"

# Master timer frequency. No topic can publish faster than this.
TICK_RATE_HZ: float = 30.0

# Depth of the publisher queues.
QOS_DEPTH: int = 10

# "Latched" QoS for the global topics, which carry shared, static-ish map data:
# a depth-1, transient-local profile so a subscriber that joins after the last
# publish still receives the most recent sample. Subscribers must also be
# transient-local to get the latched sample (a volatile subscriber still
# connects, it just won't receive the retained message on join).
LATCHED_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# Nav goals are scattered uniformly within this radius (meters) of the origin.
GOAL_RADIUS_M: float = 10.0

# Map markers are placed exactly this far (meters) from the origin.
MARKER_RADIUS_M: float = 10.0

# Number of goals carried in a mock NavGoalSequence.
GOALS_PER_SEQUENCE: int = 3


@dataclass(frozen=True)
class TopicSpec:
    """Declares a topic to mock, its message type and intended publish rate.

    Attributes:
        topic: Topic name suffix (the ``/<vehicle_id>/`` prefix is added later).
        msg_type: ROS message class published on the topic.
        rate_hz: Desired publish rate; clamped to ``TICK_RATE_HZ``.
        factory: Builds a fresh, randomly-populated message for a vehicle id.
        qos: Publisher QoS -- a queue depth (int) for a volatile stream, or a
            ``QoSProfile``. Map data uses ``LATCHED_QOS`` so a late-joining panel
            still receives the most recent sample.
    """

    topic: str
    msg_type: type
    rate_hz: float
    factory: Callable[["MockUiDataPublisher", str], Any]
    qos: Any = QOS_DEPTH


@dataclass
class PublishJob:
    """A concrete publisher with a precomputed decimation and message builder."""

    publisher: Any
    build: Callable[[], Any]
    decimation: int


class MockUiDataPublisher(Node):
    """Publishes randomized mock data on the topics the RViz plugin consumes."""

    def __init__(self) -> None:
        super().__init__("mock_ui_data_publisher")

        self._specs = self._build_topic_specs()
        self._jobs = self._create_jobs()
        self._tick = 0

        # NavGoals are randomized once here and then republished unchanged, so
        # goals stay put in RViz instead of teleporting on every tick.
        self._current_goals: Dict[str, NavGoal] = {
            vehicle_id: self._make_nav_goal(MAP_FRAME_ID)
            for vehicle_id in VEHICLE_IDS
        }
        self._goal_sequences: Dict[str, NavGoalSequence] = {
            vehicle_id: self._build_goal_sequence()
            for vehicle_id in VEHICLE_IDS
        }

        # A stable set of tracked-target ids per vehicle, chosen once so the
        # tracker sub-groups in the UI stay put and only their values update.
        self._tracker_targets: Dict[str, List[str]] = {
            vehicle_id: self._build_tracker_targets(vehicle_id)
            for vehicle_id in VEHICLE_IDS
        }

        self._timer = self.create_timer(1.0 / TICK_RATE_HZ, self._on_tick)

        self.get_logger().info(
            f"Publishing {len(self._specs)} topic(s) per vehicle for vehicles "
            f"{VEHICLE_IDS} at a {TICK_RATE_HZ:g} Hz master tick.")

    # ------------------------------------------------------------------ setup

    def _build_topic_specs(self) -> List[TopicSpec]:
        """Single source of truth for which topics are mocked and how fast."""
        return [
            TopicSpec("avt_341/odometry", Odometry, 10.0,
                      MockUiDataPublisher._make_odometry),
            TopicSpec("avt_341/cmd_vel", Twist, 10.0,
                      MockUiDataPublisher._make_cmd_vel),
            TopicSpec("avt_341/desired_speed", Float64, 10.0,
                      MockUiDataPublisher._make_desired_speed),
            TopicSpec("avt_341/task_status", MissionTaskStatus, 0.2,
                      MockUiDataPublisher._make_task_status),
            TopicSpec("avt_341/state", NavState, 10.0,
                      MockUiDataPublisher._make_nav_state),
            TopicSpec("avt_341/tracker/state", TrackerModuleStatus, 10.0,
                      MockUiDataPublisher._make_tracker_module_status),
            TopicSpec("avt_341/current_goal", NavGoal, 10.0,
                      MockUiDataPublisher._make_current_goal),
            TopicSpec("avt_341/waypoints", NavGoalSequence, 10.0,
                      MockUiDataPublisher._make_goal_sequence),
            # Map data is latched (transient-local) so a late-joining panel still
            # receives the most recent sample.
            TopicSpec("avt_341/map_marker", MapMarker, 0.1,
                      MockUiDataPublisher._make_map_marker, qos=LATCHED_QOS),
            TopicSpec("avt_341/map_markers_changed", MapMarkerList, 0.1,
                      MockUiDataPublisher._make_map_marker_list, qos=LATCHED_QOS),
        ]

    def _create_jobs(self) -> List[PublishJob]:
        """Create one publisher per (vehicle, topic), each with a precomputed
        decimation and a zero-arg message builder."""
        jobs: List[PublishJob] = []
        for vehicle_id in VEHICLE_IDS:
            for spec in self._specs:
                topic = f"/{vehicle_id}/{spec.topic}"
                publisher = self.create_publisher(spec.msg_type, topic, spec.qos)
                jobs.append(
                    PublishJob(publisher=publisher,
                               build=partial(spec.factory, self, vehicle_id),
                               decimation=self._decimation(spec.rate_hz)))
        return jobs

    @staticmethod
    def _decimation(rate_hz: float) -> int:
        """Number of master ticks between publishes for the given rate.

        Rates at or above the tick rate decimate to 1 (publish every tick),
        which caps the effective rate at ``TICK_RATE_HZ``.
        """
        return max(1, round(TICK_RATE_HZ / rate_hz))

    # ------------------------------------------------------------------- loop

    def _on_tick(self) -> None:
        for job in self._jobs:
            if self._tick % job.decimation == 0:
                job.publisher.publish(job.build())
        self._tick += 1

    # -------------------------------------------------------------- factories

    def _make_odometry(self, vehicle_id: str) -> Odometry:
        """Odometry with only the pose and linear velocity populated."""
        return self._fill_odometry(Odometry(), vehicle_id)

    def _make_cmd_vel(self, vehicle_id: str) -> Twist:
        """Drive command. nav_state_component reads linear.x as throttle,
        linear.y as brake and angular.z as steering (the t/s/b command row);
        the other components are unused by the UI and left at zero."""
        cmd = Twist()
        cmd.linear.x = random.uniform(0.0, 1.0)    # throttle
        cmd.linear.y = random.uniform(0.0, 1.0)    # brake
        cmd.angular.z = random.uniform(-1.0, 1.0)  # steering
        return cmd

    def _make_desired_speed(self, vehicle_id: str) -> Float64:
        """Desired speed (m/s); nav_state_component shows it as the x-hat term
        of the velocity row."""
        return Float64(data=random.uniform(0.0, 10.0))

    def _make_task_status(self, vehicle_id: str) -> MissionTaskStatus:
        msg = MissionTaskStatus()
        msg.header = self._header(MAP_FRAME_ID)
        # -1 sentinel (no task) about a third of the time, else a real id.
        msg.task_id = random.choice([-1, random.randint(0, 50)])
        msg.task_description = random.choice([
            "Idle", "Move to goal", "Holding position", "Form wedge",
            "Encircle target", "Follow leader"
        ])
        others = [v for v in VEHICLE_IDS if v != vehicle_id]
        # Empty tracked_vehicle => this vehicle is the leader / not following.
        msg.tracked_vehicle = random.choice([""] + others)
        msg.formation_type = random.choice(
            ["", "wedge", "column", "line", "encircle"])
        msg.target_pose = self._random_pose()
        # A random subset of the other vehicles participate in the formation.
        msg.formation_vehicles = random.sample(others,
                                               random.randint(0, len(others)))
        return msg

    def _make_nav_state(self, vehicle_id: str) -> NavState:
        msg = NavState()
        msg.header = self._header(MAP_FRAME_ID)
        msg.goal = self._current_goals[vehicle_id]
        msg.run_state = random.choice([-1, 0, 1, 2, 3])
        msg.goal_duration = random.uniform(0.0, 300.0)
        msg.goal_distance = random.uniform(0.0, 100.0)
        msg.goal_yaw_difference = random.uniform(-math.pi, math.pi)
        return msg

    def _make_current_goal(self, vehicle_id: str) -> NavGoal:
        """The vehicle's current navigation goal, randomized once at startup."""
        return self._current_goals[vehicle_id]

    def _make_goal_sequence(self, vehicle_id: str) -> NavGoalSequence:
        """The vehicle's ordered run of goals, randomized once at startup."""
        return self._goal_sequences[vehicle_id]

    def _build_goal_sequence(self) -> NavGoalSequence:
        """Build an ordered run of GOALS_PER_SEQUENCE goals, each near the
        origin. Called once per vehicle at startup."""
        msg = NavGoalSequence()
        msg.header = self._header(MAP_FRAME_ID)
        msg.goals = [
            self._make_nav_goal(MAP_FRAME_ID) for _ in range(GOALS_PER_SEQUENCE)
        ]
        return msg

    def _build_tracker_targets(self, vehicle_id: str) -> List[str]:
        """A stable set of 0-2 distinct tracked-target ids for this vehicle,
        chosen once at startup."""
        others = [v for v in VEHICLE_IDS if v != vehicle_id]
        pool = others + ["target_1", "target_2"]
        return random.sample(pool, random.randint(0, 2))

    def _make_tracker_module_status(self, vehicle_id: str) -> TrackerModuleStatus:
        """Aggregate tracker-module status: a coarse module state plus a list of
        per-target TrackerStatus, one per stable tracked target (so the module is
        uninitialized when the vehicle happens to have no targets)."""
        msg = TrackerModuleStatus()
        msg.header = self._header(f"{vehicle_id}/base_link")
        targets = self._tracker_targets[vehicle_id]
        if not targets:
            msg.module_state = TrackerModuleStatus.MODULE_STATE_UNINITIALIZED
        else:
            msg.module_state = TrackerModuleStatus.MODULE_STATE_ACTIVE
            msg.trackers = [self._make_tracker_status(vehicle_id, object_id)
                            for object_id in targets]
        return msg

    def _make_tracker_status(self, vehicle_id: str, object_id: str) -> TrackerStatus:
        msg = TrackerStatus()
        msg.header = self._header(f"{vehicle_id}/base_link")
        msg.state = random.randint(TrackerStatus.STATE_UNINITIALIZED,
                                   TrackerStatus.STATE_FULL_TRACKING)
        msg.tracked_object_id = object_id
        msg.odom_estimate = self._make_tracker_odometry(vehicle_id)
        msg.tracked_cloud_size = random.randint(0, 5000)
        msg.clustering_success = random.random() < 0.8
        msg.clusters_found = random.randint(0, 20)
        msg.ground_segmentation_success = random.random() < 0.8
        msg.time_since_valid_detection = random.uniform(0.0, 5000.0)
        msg.execution_time = random.uniform(0.0, 100.0)
        return msg

    def _make_tracker_odometry(self, vehicle_id: str) -> Odometry:
        """Tracked-target odometry with pose + a populated x/y/yaw covariance
        block. pose.covariance is a 6x6 row-major [x, y, z, roll, pitch, yaw]
        array; the tracker_component shows the x/y/yaw sub-matrix, so fill those
        six unique (symmetric) entries."""
        odom = self._fill_odometry(Odometry(), vehicle_id)
        cov = list(odom.pose.covariance)
        # Diagonal variances for x, y, yaw (span the matrix_field thresholds so
        # the cells show a mix of green / orange / red).
        cov[0 * 6 + 0] = random.uniform(0.0, 15.0)   # var(x)
        cov[1 * 6 + 1] = random.uniform(0.0, 15.0)   # var(y)
        cov[5 * 6 + 5] = random.uniform(0.0, 15.0)   # var(yaw)
        # Symmetric off-diagonal covariances.
        cov_xy = random.uniform(-3.0, 3.0)
        cov_xyaw = random.uniform(-3.0, 3.0)
        cov_yyaw = random.uniform(-3.0, 3.0)
        cov[0 * 6 + 1] = cov[1 * 6 + 0] = cov_xy     # cov(x, y)
        cov[0 * 6 + 5] = cov[5 * 6 + 0] = cov_xyaw   # cov(x, yaw)
        cov[1 * 6 + 5] = cov[5 * 6 + 1] = cov_yyaw   # cov(y, yaw)
        odom.pose.covariance = cov
        return odom

    def _make_map_marker(self, vehicle_id: str) -> MapMarker:
        """A single mission-point marker at a random 10 m position and yaw."""
        return self._make_marker(vehicle_id, MapMarker.MISSION_POINT)

    def _make_map_marker_list(self, vehicle_id: str) -> MapMarkerList:
        """Three markers (two mission points, one target of interest), each at a
        random 10 m position with a random yaw."""
        msg = MapMarkerList()
        msg.header = self._header(MAP_FRAME_ID)
        msg.markers = [
            self._make_marker(vehicle_id, MapMarker.MISSION_POINT),
            self._make_marker(vehicle_id, MapMarker.MISSION_POINT),
            self._make_marker(vehicle_id, MapMarker.TARGET_OF_INTEREST),
        ]
        return msg

    def _make_marker(self, vehicle_id: str, marker_type: int) -> MapMarker:
        """A MapMarker of the given type at a random 10 m position and yaw."""
        marker = MapMarker()
        marker.header = self._header(MAP_FRAME_ID)
        # Mission-point id: "MP_<letter>" with a random uppercase letter, which
        # the map-marker visual renders as "MP" stacked over the letter.
        marker.marker_id = f"MP_{random.choice(string.ascii_uppercase)}"
        marker.label = ("Mission Point"
                        if marker_type == MapMarker.MISSION_POINT
                        else "Target of Interest")
        marker.type = marker_type
        marker.pose = self._random_marker_pose()
        return marker

    # ------------------------------------------------------------- mock fields

    def _make_nav_goal(self, frame_id: str) -> NavGoal:
        """A NavGoal whose pose lands within GOAL_RADIUS_M of the origin."""
        goal = NavGoal()
        goal.header = self._header(frame_id)
        goal.pose = self._random_goal_pose()
        goal.dist_threshold = random.uniform(0.5, 5.0)
        goal.yaw_threshold = random.uniform(0.05, math.pi)
        return goal

    def _fill_odometry(self, odom: Odometry, vehicle_id: str) -> Odometry:
        """Populate only the pose and linear velocity of an odometry message.

        Reused wherever an odometry (sub-)message is encountered so the rule
        "fill only pose and linear velocity" is applied consistently.
        """
        odom.header = self._header(f"{vehicle_id}/odom")
        odom.child_frame_id = f"{vehicle_id}/base_link"
        odom.pose.pose = self._random_pose()
        odom.twist.twist.linear = Vector3(x=random.uniform(0.0, 10.0),
                                          y=random.uniform(-1.0, 1.0),
                                          z=random.uniform(-1.0, 1.0))
        return odom

    def _header(self, frame_id: str) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _random_pose(self) -> Pose:
        pose = Pose()
        pose.position = Point(x=random.uniform(-50.0, 50.0),
                              y=random.uniform(-50.0, 50.0),
                              z=random.uniform(-2.0, 2.0))
        pose.orientation = self._random_quaternion()
        return pose

    def _random_goal_pose(self) -> Pose:
        """A ground-level pose (z = 0) drawn uniformly from the disk of radius
        GOAL_RADIUS_M, oriented to a random yaw about +Z (no roll or pitch).

        ``sqrt`` of a uniform sample gives a radius that is uniform over area,
        so goals do not bunch up near the origin.
        """
        radius = GOAL_RADIUS_M * math.sqrt(random.random())
        angle = random.uniform(0.0, 2.0 * math.pi)
        pose = Pose()
        pose.position = Point(x=radius * math.cos(angle),
                              y=radius * math.sin(angle),
                              z=0.0)
        pose.orientation = self._yaw_quaternion(
            random.uniform(-math.pi, math.pi))
        return pose

    @staticmethod
    def _random_quaternion() -> Quaternion:
        """A uniformly-distributed, unit-norm random orientation (Shoemake)."""
        u1, u2, u3 = random.random(), random.random(), random.random()
        return Quaternion(
            x=math.sqrt(1.0 - u1) * math.sin(2.0 * math.pi * u2),
            y=math.sqrt(1.0 - u1) * math.cos(2.0 * math.pi * u2),
            z=math.sqrt(u1) * math.sin(2.0 * math.pi * u3),
            w=math.sqrt(u1) * math.cos(2.0 * math.pi * u3))

    def _random_marker_pose(self) -> Pose:
        """A pose exactly MARKER_RADIUS_M from the origin at a random bearing,
        oriented to a random yaw about +Z."""
        bearing = random.uniform(0.0, 2.0 * math.pi)
        pose = Pose()
        pose.position = Point(x=MARKER_RADIUS_M * math.cos(bearing),
                              y=MARKER_RADIUS_M * math.sin(bearing),
                              z=0.0)
        pose.orientation = self._yaw_quaternion(
            random.uniform(-math.pi, math.pi))
        return pose

    @staticmethod
    def _yaw_quaternion(yaw: float) -> Quaternion:
        """A quaternion for a rotation of `yaw` radians about +Z (no roll/pitch)."""
        return Quaternion(x=0.0, y=0.0,
                          z=math.sin(yaw / 2.0),
                          w=math.cos(yaw / 2.0))


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = MockUiDataPublisher()
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
