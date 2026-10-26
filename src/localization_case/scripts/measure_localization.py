#!/usr/bin/env python3
"""PR08 measurement node: AMCL convergence on the PR07 recording.

Run BEFORE `ros2 bag play` with use_sim_time:=true so that the FIRST
/clock of the recording defines t0. All intervals are in SIMULATION
time from /clock, never wall time and never absolute 10.0:

  warmup:  [t0, t0 + window_start)            (skipped, default 10 s)
  window:  [t0 + window_start, t0 + window_start + window_len)
                                              (default [10, 30) -> 20 s)

Every query period (default 0.1 s of sim time inside the window) it
performs ONE lookup_transform('map', 'odom', Time()) — the zero stamp
means "latest available". A tf2 exception counts as an unsuccessful
query; no retries against a frozen buffer.

It also collects /amcl_pose messages whose header.stamp falls inside
the window (>= 10 required) and computes the median of
covariance[0] + covariance[7].

When the window is done AND the player has stopped feeding /clock
(wall-clock silence), it writes the result JSON: map file SHA-256s, bag
path and duration (first/last /clock), initial pose, window duration,
query counts and fraction, amcl_pose count, covariance median, and the
base_frame_id read from the params file.

Pure helpers live module-level so test_measures.py can unit-test them
without a ROS graph.
"""

import argparse
import hashlib
import json
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

WINDOW_START_DEFAULT = 10.0
WINDOW_LEN_DEFAULT = 20.0
QUERY_PERIOD_DEFAULT = 0.1
PLAYER_SILENCE_SEC = 4.0   # wall seconds without /clock -> player finished


def stamp_to_sec(stamp) -> float:
    """ROS Time / builtin msg stamp -> float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def in_window(stamp_sec: float, win_start: float, win_end: float) -> bool:
    """Inclusive start, exclusive end — [win_start, win_end)."""
    return win_start <= stamp_sec < win_end


def covariance_sum(cov) -> float:
    """covariance[0] (x, x) + covariance[7] (y, y)."""
    return float(cov[0]) + float(cov[7])


def median_covariance(messages) -> float:
    """Median of cov[0]+cov[7] over PoseWithCovarianceStamped messages."""
    return float(statistics.median(covariance_sum(m.pose.covariance)
                                   for m in messages))


def read_profile_fields(params_file: str):
    """Extract (base_frame_id, initial_pose dict) from the params YAML."""
    import yaml
    with open(params_file, encoding='utf-8') as f:
        profile = yaml.safe_load(f)
    amcl = profile.get('amcl', {}).get('ros__parameters', {})
    base_frame = amcl.get('base_frame_id', '')
    ip = amcl.get('initial_pose', {}) or {}
    initial_pose = {
        'x': float(ip.get('x', 0.0)),
        'y': float(ip.get('y', 0.0)),
        'yaw': float(ip.get('yaw', 0.0)),
    }
    return base_frame, initial_pose


def sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


class LocalizationMeasurer(Node):

    def __init__(self, args):
        super().__init__('localization_measurer')
        self.win_start_off = args.window_start
        self.win_len = args.window_len
        self.query_period = args.query_period
        self.out_path = args.out
        self.bag_path = args.bag_path
        self.params_file = args.params_file
        self.base_frame_id, self.initial_pose = \
            read_profile_fields(args.params_file)
        self.map_yaml_sha = sha256_of(args.map_yaml)
        self.map_pgm_sha = sha256_of(args.map_pgm)

        self.t0 = None                 # first /clock stamp (sim time)
        self.win_end_abs = None
        self.window_done = False
        self.last_clock_wall = None

        self.queries_total = 0
        self.queries_ok = 0
        self.amcl_poses = []
        self.first_clock = None
        self.last_clock = None

        # sensor-data QoS would drop messages; AMCL pose is reliable-ish
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=200)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose,
            pose_qos)
        self.create_subscription(Clock, 'clock', self._on_clock, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.query_timer = self.create_timer(self.query_period,
                                             self._on_query_tick)
        # wall-time watchdog: finalize after the player goes silent
        self.create_timer(1.0, self._on_wall_tick)
        self.get_logger().info(
            'waiting for the first /clock to define t0 '
            f'(window [t0+{self.win_start_off}, '
            f't0+{self.win_start_off + self.win_len}))')

    # ---------- callbacks ----------

    def _on_clock(self, msg: Clock):
        sec = stamp_to_sec(msg.clock)
        if self.first_clock is None:
            self.first_clock = sec
            self.t0 = sec
            self.win_end_abs = self.t0 + self.win_start_off + self.win_len
            self.get_logger().info(f't0 = {sec:.3f} (first /clock)')
        self.last_clock = sec
        self.last_clock_wall = time.monotonic()

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        if self.t0 is None or self.window_done:
            return
        stamp = stamp_to_sec(msg.header.stamp)
        if in_window(stamp, self.t0 + self.win_start_off, self.win_end_abs):
            self.amcl_poses.append(msg)

    def _on_query_tick(self):
        if self.t0 is None or self.window_done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        win_start = self.t0 + self.win_start_off
        if now < win_start:
            return
        if now >= self.win_end_abs:
            self._on_window_done()
            return
        # exactly one lookup per tick, zero stamp = latest available
        self.queries_total += 1
        try:
            self.tf_buffer.lookup_transform('map', 'odom', Time())
            self.queries_ok += 1
        except Exception as e:   # tf2 LookupException / Connectivity / Extrapolation
            self.get_logger().warning(
                f'lookup map->odom failed: {type(e).__name__}: {e}')

    def _on_window_done(self):
        self.window_done = True
        self.query_timer.cancel()
        self.get_logger().info(
            f'window done: queries {self.queries_ok}/{self.queries_total}, '
            f'amcl_pose in window: {len(self.amcl_poses)}')

    def _on_wall_tick(self):
        # finalize only when the window is measured and the player
        # stopped publishing /clock (so bag duration is complete)
        if not self.window_done or self.last_clock_wall is None:
            return
        if time.monotonic() - self.last_clock_wall < PLAYER_SILENCE_SEC:
            return
        self.finalize()

    # ---------- result ----------

    def finalize(self):
        bag_duration = None
        if self.first_clock is not None and self.last_clock is not None:
            bag_duration = round(self.last_clock - self.first_clock, 3)

        result = {
            '_note': ('Field names follow the PR08 assignment text; verify '
                      'against localization-result-v1.schema.json from the '
                      'course kit (schema unavailable in this environment).'),
            'map_yaml_sha256': self.map_yaml_sha,
            'map_pgm_sha256': self.map_pgm_sha,
            'bag_path': self.bag_path,
            'bag_first_clock_sec': self.first_clock,
            'bag_last_clock_sec': self.last_clock,
            'bag_duration_sec': bag_duration,
            'initial_pose': self.initial_pose,
            'window_start_offset_sec': self.win_start_off,
            'window_duration_sec': self.win_len,
            'tf_queries_total': self.queries_total,
            'tf_queries_successful': self.queries_ok,
            'tf_success_fraction': (
                round(self.queries_ok / self.queries_total, 4)
                if self.queries_total else None),
            'amcl_pose_count': len(self.amcl_poses),
            'covariance_median': (
                round(median_covariance(self.amcl_poses), 4)
                if len(self.amcl_poses) >= 1 else None),
            'base_frame_id': self.base_frame_id,
        }
        with open(self.out_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
            f.write('\n')
        self.get_logger().info(f'result written to {self.out_path}')
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--map-pgm', required=True)
    parser.add_argument('--bag-path', required=True)
    parser.add_argument('--params-file', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--window-start', type=float,
                        default=WINDOW_START_DEFAULT)
    parser.add_argument('--window-len', type=float, default=WINDOW_LEN_DEFAULT)
    parser.add_argument('--query-period', type=float,
                        default=QUERY_PERIOD_DEFAULT)
    script_args = []
    if '--' in sys.argv:
        script_args = sys.argv[sys.argv.index('--') + 1:]
    args = parser.parse_args(script_args)

    rclpy.init()
    node = LocalizationMeasurer(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
