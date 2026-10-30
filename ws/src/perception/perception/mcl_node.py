#!/usr/bin/env python3
"""Monte Carlo localization on the saved map: a drop-in for Cartographer localization + localization_gate.

Started by mcl.launch.py. Same outputs as localization.launch.py:
  TF map -> odom      the filter's pose in the map composed with the inverse of odom -> base_link
  /localized_pose     PoseStamped in 'map' (base_link = rear axle), published at /odom rate, and only while
                      localized, so pure pursuit does not drive before that (as with localization_gate)
Extra: /mcl/particles (PoseArray), /map (OccupancyGrid, latched) for Foxglove.

  find:      the car finds itself with a global scan-matching search (ScanMatcher), in a worker thread (the pose
             and TF keep flowing): at the start, after a sim reset, and when lost. Hybrid: first only near the
             track's raceline, heading along it either way (~5x faster); if that fits well (hybrid_min_fit) and
             is decided, the car may go at once while the search over the whole drivable area at every heading
             runs in the background to verify it (moving the car if it is clearly elsewhere, keeping places that
             fit as well as alternatives). Otherwise, or with no raceline.csv, the whole-area search decides. No ground truth is used, so the sim behaves like the real car. If several places fit the
             scan about as well, choose_pose refines from the last known pose (or the map origin, where race.sh
             puts the car). If that cannot decide, the search retries (backing off to search_period_max); after
             force_after tries it takes its best guess (and a relaxed_min_fit floor) and keeps the other places
             as alternatives, scored against the scans as the car drives: one that fits clearly better
             (validate_evidence, summed over the updates) is switched to, one that fits clearly worse is ruled
             out, and ones that keep fitting the same (a featureless stretch) are kept for validate_distance m
             (about a lap), not taken as ruled out.
  converge:  the cloud starts around the pose found; until it is tight (converged_*), scans are used even
             though the car stands still, then the car may drive. Afterwards a scan is used after
             update_min_d / update_min_a of odometry motion.
  thrown off: each update scores how well the scan fits the map at the estimate. When the fit stays below
             fit_drop x its running level for fit_bad_updates updates (wheel slip, a stalled sensor stream),
             a local search (within local_radius, and local_yaw: slip moves the car but barely turns it)
             re-anchors the cloud if it finds a clearly better pose and the next search finds the same offset;
             every recheck_updates updates a small local search catches what the fit does not show (along a
             straight). The car keeps driving through both. Either re-anchors only to a pose that fits well
             (reanchor_min_fit, or reanchor_ratio x the fit on good stretches if that is lower, so a lidar or
             map that fits less everywhere scales the gate down): where the map itself is wrong (iros2026_v2
             has a stretch where no pose within 1 m fits better than ~0.6), a "better" pose is just another
             wrong one, and the filter coasts on the odometry instead. A burst of re-anchors (slip_reanchors
             in 2 s, where the map fits badly) pauses them for slip_hold s and starts a background global search.
  doubt:     the fit below doubt_fit x the good-fit level (or the estimate off the drivable area) for
             doubt_updates updates in a row (beyond the local search's reach, or something the map does not
             have): a global search in the background, re-anchoring only
             if it finds the car clearly elsewhere, and only when the place fits very well (relocate_min_fit, or
             relocate_ratio x the good-fit level) and the next search agrees (the same place, carried by the
             odometry): where the map is wrong the true place fits worse than a look-alike elsewhere. A low fit
             alone never stops the car.
  odom jump: a step between two odometry messages bigger than max_jump that the car's speed does not explain
             (an odometry restart): the pose is kept in the new frame and a background search checks it. A big
             step the speed explains (the streams stalled while the car drove on) is driven.
  unsure:    after a guess, until a whole-area search finds the pick and no other place fitting as well, one
             runs every unsure_period m in the background (the true place may not be among the alternatives).
  lost:      only the cloud spreading beyond lost_std_xy, or the estimate in a wall / outside the map for
             doubt_updates updates, withdraws the pose (the car stops) and starts a search.
  sim reset: fast_bridge's /autodrive/roboracer_1/sim_reset event: the pose is withheld at once; after
             debounce s with the car still, a global search.
  manual:    a PoseWithCovarianceStamped on /initialpose (RViz, Foxglove) restarts the cloud there.
"""
import math
import os
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from perception.map_origin import pose2d
from perception.mcl import (Alternatives, LikelihoodMap, ParticleFilter, ScanMatcher, choose_pose, compose,
                            inverse, pose_distance, relative, select_beams, wrap)
from perception.tracks import track_dir

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MAX_PARTICLES_SHOWN = 200
MIN_BEAMS = 10
MAP_ORIGIN = (0.0, 0.0, 0.0)   # where race.sh puts the car: the pose mapping started from
LOG_PERIOD = 2.0               # s between re-anchor log lines
FIND_LOG_PERIOD = 5.0          # s between repeats of the same "cannot find the car" warning
RELOCATE_TRIES = 4             # check searches in a row looking for two that agree, then wait for a trigger
LATENCY_LOG_PERIOD = 60.0      # s between scan latency log lines
MAX_SCAN_LATENCY = 0.2         # s: a scan stamped longer before the newest /odom is matched to the newest
RELOCATE_EXPIRY = 10.0         # s after which a relocation candidate no longer counts as the one before


def yaw_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)   # z, w


class MclNode(Node):
    def __init__(self):
        super().__init__('mcl')
        p = self.declare_parameter
        track = p('track', '').value
        self.beams = p('beams', 60).value
        self.update_min_d = p('update_min_d', 0.05).value           # m of odometry between scan updates
        self.update_min_a = p('update_min_a', 0.05).value           # rad
        self.max_jump = p('max_jump', 1.0).value                    # m: a bigger step between two odometry
        #                                         messages that the car's speed does not explain: a frame jump
        self.init_std_xy = p('init_std_xy', 0.15).value
        self.init_std_yaw = p('init_std_yaw', 0.10).value
        self.conv_xy = p('converged_std_xy', 0.10).value
        self.conv_yaw = p('converged_std_yaw', 0.08).value
        self.conv_updates = p('converged_updates', 3).value
        self.lost_xy = p('lost_std_xy', 0.60).value
        self.debounce = p('debounce', 0.5).value                    # s after the last reset event
        self.still_speed = p('still_speed', 0.1).value              # m/s: the car has stopped after a reset
        self.timeout = p('timeout', 10.0).value
        self.publish_particles = p('publish_particles', True).value
        self.min_fit = p('min_fit', 0.5).value                      # a search result fitting worse is no pose
        self.ambiguity = p('ambiguity', 0.97).value                 # 2nd place fitting this x the best: a tie
        self.prior_radius = p('prior_radius', 3.0).value            # m around the last known pose: tie-break
        self.search_period = p('search_period', 0.5).value          # s between global search attempts
        self.search_period_max = p('search_period_max', 2.0).value  # s, backing off while they fail
        self.force_after = p('force_after', 4).value                # failed attempts before a best guess
        self.relaxed_min_fit = p('relaxed_min_fit', 0.3).value      # min_fit once guessing
        self.yaw_scale = p('yaw_scale', 1.0).value                  # m per rad when comparing poses to priors
        self.validate_distance = p('validate_distance', 200.0).value  # m an unresolved alternative is kept
        self.validate_evidence = p('validate_evidence', 1.5).value  # summed fit difference that decides one
        self.unsure_period = p('unsure_period', 10.0).value         # m between whole-area searches after a guess
        self.unsure_searches = p('unsure_searches', 20).value       # ... at most this many, until one confirms
        hybrid = p('hybrid', True).value                            # raceline-first search, verified after
        self.hybrid_min_fit = p('hybrid_min_fit', 0.8).value        # a raceline-only result fitting worse: full
        self.hybrid_seeds = p('hybrid_seeds', 15).value
        self.fit_drop = p('fit_drop', 0.75).value                   # x the fit's level: try a local search
        self.fit_bad_updates = p('fit_bad_updates', 3).value
        self.doubt_fit = p('doubt_fit', 0.4).value                  # x the fit's level for doubt_updates
        self.doubt_updates = p('doubt_updates', 10).value           # updates in a row: a background search
        self.local_radius = p('local_radius', 0.8).value            # m
        self.local_yaw = p('local_yaw', 0.07).value                 # rad: slip moves the car, barely turns it
        self.snap_gain = p('snap_gain', 0.1).value                  # fit a re-anchored pose must gain
        self.reanchor_min_fit = p('reanchor_min_fit', 0.8).value    # ... and at least this fit (or this x the
        self.reanchor_ratio = p('reanchor_ratio', 0.85).value       # good-fit level, if lower): not a bad map
        self.relocate_min_fit = p('relocate_min_fit', 0.9).value    # a background search moves the car only to
        self.relocate_ratio = p('relocate_ratio', 0.93).value       # a place fitting this well (or ratio x level),
        self.relocate_agree = p('relocate_agree', 0.5).value        # found twice (m)
        self.relocate_margin = p('relocate_margin', 0.05).value     # and fitting this much better than the pose
        self.snap_agree_yaw = p('snap_agree_yaw', 0.035).value      # rad between two searches' offsets
        self.slip_reanchors = p('slip_reanchors', 3).value          # re-anchors in LOG_PERIOD s: a burst
        self.slip_hold = p('slip_hold', 2.0).value                  # s without local re-anchors after a burst
        self.recheck_updates = p('recheck_updates', 5).value        # updates between small local searches
        self.recheck_radius = p('recheck_radius', 0.5).value        # m
        self.recheck_gain = p('recheck_gain', 0.03).value           # fit gain that counts as better
        self.recheck_min_shift = p('recheck_min_shift', 0.08).value  # m, smaller offsets: the filter's job
        self.recheck_agree = p('recheck_agree', 0.1).value          # m between two checks' offsets
        self.frame = 'map'

        self.lmap = LikelihoodMap.from_yaml(os.path.join(track_dir(track), 'map.yaml'),
                                            max_dist=p('max_dist', 2.0).value)
        self.pf = ParticleFilter(
            n=p('particles', 1000).value, seed=p('seed', -1).value, sigma_hit=p('sigma_hit', 0.15).value,
            z_hit=p('z_hit', 0.8).value, z_rand=p('z_rand', 0.2).value,
            likelihood_scale=p('likelihood_scale', 0.4).value,
            noise_x=tuple(p('noise_x', [0.10, 0.02]).value), noise_y=tuple(p('noise_y', [0.05, 0.05]).value),
            noise_yaw=tuple(p('noise_yaw', [0.05, 0.10]).value))
        self.matcher = ScanMatcher(self.lmap)
        self.hybrid = False
        raceline = os.path.join(track_dir(track), 'raceline.csv')
        if hybrid and os.path.isfile(raceline):
            line = np.loadtxt(raceline, delimiter=';', comments='#', ndmin=2)[:, 1:3]   # TUM: s; x; y; ...
            self.matcher.set_track(line, radius=p('hybrid_radius', 0.6).value,
                                   yaw_window=p('hybrid_yaw', math.radians(25.0)).value)
            self.hybrid = True

        self.odom = None              # (pose, stamp msg, speed)
        self.last_odom_pose = None    # odom pose of the last filter update
        self.prev_odom = None         # the previous /odom message: (pose, stamp [s], speed), for frame jumps
        self.odom_hist = deque(maxlen=200)   # (stamp [s], pose) of recent /odom: the odometry at a scan's stamp
        self.interpolate_odom = p('interpolate_odom', True).value
        self.scan_odom = None         # the odometry at the scan being processed
        self.latency = []             # s, newest /odom stamp minus scan stamp, since the last latency log
        self.latency_log_t = time.monotonic()
        self.jumped = None            # size [m] of an odometry frame jump the next scan has to check
        self.map_T_odom = None
        self.sensor_pose = {}         # scan frame -> pose in base_link
        self.initialized = False
        self.localized = False
        self.good = 0
        self.pending = None           # monotonic time of the last reset signal
        self.search = 'start'         # why a global search is due, None while tracking
        self.priors = [MAP_ORIGIN]    # poses the car is likely near: tie-break for an ambiguous search
        self.last_search = 0.0
        self.search_wait = self.search_period
        self.attempts = 0             # failed global searches since the search started
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.job = None               # (future, kind, odom pose at the scan, generation, start time)
        self.generation = 0           # bumped whenever a running search's result would be stale
        self.alternatives = None      # Alternatives after a guessed pick
        self.unsure = False           # a guessed pick not yet confirmed by a whole-area search
        self.unsure_left = 0          # whole-area searches left for it (per find)
        self.unsure_travel = 0.0      # m driven since the last of them
        self.offmap = 0               # updates in a row with the estimate in a wall or outside the map
        self.find_log = ('', 0.0)     # last "cannot find" warning and when
        self.fit_level = None         # running level of the fit while tracking
        self.good_level = None        # the fit on stretches where the map fits: scales the re-anchor floors
        self.fit_bad = 0
        self.doubt = 0
        self.rechecks = 0
        self.recheck_offset = None    # offset (in the car's frame) the last recheck found, waiting to agree
        self.snap_offset = None       # the same for the low-fit local search
        self.relocation = None        # (pose, odom pose, searches, time) a check search would move the car to
        self.anchors = []             # monotonic times of recent re-anchors
        self.anchor_log_t = 0.0
        self.hold_until = 0.0         # monotonic time until which local searches do not re-anchor (a burst)
        self.burst = False            # a burst of re-anchors: a background global search is due
        self.updates = 0
        self.warn_t = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/localized_pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/mcl/particles', 1)
        map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.publish_map(map_pub)

        # first: rclpy runs ready subscriptions in creation order, so a reset closes the relay before a
        # pose that arrived with it is relayed
        self.create_subscription(String, '/autodrive/roboracer_1/sim_reset', self.reset_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'{self.pf.n} particles on {self.lmap.w}x{self.lmap.h} map '
                               f'({self.lmap.res:.3f} m), {len(self.matcher.candidates)} search positions'
                               + (f', {self.matcher.track_poses} raceline-first poses' if self.hybrid else
                                  ', no raceline-first search') + ', waiting for /odom and /scan')

    # ---------------- setup ----------------
    def publish_map(self, pub):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.lmap.res
        msg.info.width, msg.info.height = self.lmap.w, self.lmap.h
        msg.info.origin.position.x, msg.info.origin.position.y = self.lmap.ox, self.lmap.oy
        msg.info.origin.orientation.w = 1.0
        msg.data = self.lmap.occupancy_grid().tolist()
        pub.publish(msg)

    def init_filter(self, pose, reason, keep_localized=False, quiet=False, matched=False, odom=None):
        """Cloud around pose. matched: a scan-matched pose, known to a few cm, so a tighter cloud (a loose one
        never tightens along a featureless straight while the car stands, and the car would wait forever).
        keep_localized: a re-anchor while driving (always matched), the car does not stop for it. odom: the
        odometry pose that pose belongs to (default the newest): a pose matched to a scan belongs to the
        odometry at the scan's stamp, up to the lidar latency older (tagged with the newest, every re-anchor put
        the filter speed x latency behind the car)."""
        odom = odom if odom is not None else self.odom[0]
        k = 0.3 if keep_localized or matched else 1.0
        self.pf.init_gaussian(pose, k * self.init_std_xy, k * self.init_std_yaw)
        if not keep_localized:
            self.localized = False
            self.good = 0
        self.pending = None
        self.search = None
        self.fit_bad = 0
        self.doubt = 0
        self.rechecks = 0
        self.recheck_offset = None
        self.snap_offset = None
        self.relocation = None
        self.offmap = 0
        if reason != 're-anchor':   # a re-anchor only nudges the pose: the places still to check stay
            self.alternatives = None
        # any other pose change makes a running search stale. A re-anchor only nudges the pose, and the verify
        # behind a start (which may find the start was the wrong place) is carried by the odometry: it survives.
        if not (reason == 're-anchor' and self.job is not None and self.job[1] in ('verify', 'reverify')):
            self.generation += 1
        if not keep_localized:   # a new start: a burst before it says nothing about this pose
            self.unsure = False
            self.unsure_left = self.unsure_searches
            self.hold_until = 0.0
            self.burst = False
            self.anchors = []
        if not quiet:
            self.get_logger().info(f'{reason}: particles at x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}')
        self.initialized = True
        self.last_odom_pose = odom
        self.jumped = None
        self.map_T_odom = compose(pose, inverse(odom))

    def start_search(self, reason):
        """Withdraw the pose and find the car again, preferring places near where it was."""
        if self.initialized:
            self.priors = [self.pf.estimate(), MAP_ORIGIN]
        self.localized = False
        self.good = 0
        self.search = reason
        self.last_search = 0.0
        self.search_wait = self.search_period
        self.attempts = 0
        self.alternatives = None
        self.generation += 1

    # ---------------- odometry: TF and pose at the odom rate ----------------
    def odom_cb(self, msg):
        pose = pose2d(msg.pose.pose)
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom = (pose, msg.header.stamp, speed)
        prev, self.prev_odom = self.prev_odom, (pose, stamp, speed)
        if self.odom_hist and stamp < self.odom_hist[-1][0]:
            self.odom_hist.clear()   # time went back (a restarted bag or sim): start over
        self.odom_hist.append((stamp, pose))
        if self.map_T_odom is None:
            return
        if prev is not None:
            self.check_frame_jump(prev, pose, stamp, speed)
        self.broadcast(msg.header.stamp)
        if self.localized:
            x, y, yaw = compose(self.map_T_odom, pose)
            out = PoseStamped()
            out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame
            out.pose.position.x, out.pose.position.y = x, y
            out.pose.orientation.z, out.pose.orientation.w = yaw_quaternion(yaw)
            self.pose_pub.publish(out)

    def check_frame_jump(self, prev, pose, stamp, speed):
        """A step from the previous odometry message bigger than max_jump and than the car's speed explains over
        the time between the two messages is the odometry frame jumping (an odometry restart, a respawn without a
        reset event): the pose is kept, re-expressed in the new frame, and the next scan checks it. A big step the
        speed does explain is driving (the streams stalled: 2.6 m at 3.8 m/s in 0.69 s on iros2026_v2), and the
        filter moves the car by it as by any other step: dropping it left the pose 2.6 m behind, and the car
        crashed before a search could fix that. Per message, so standing still for a while (no filter updates)
        does not make a jump look like driving."""
        step = math.hypot(pose[0] - prev[0][0], pose[1] - prev[0][1])
        dt = max(0.0, stamp - prev[1])
        if step <= self.max_jump or step <= dt * 1.5 * max(prev[2], speed) + 0.3:
            return
        jump = compose(pose, inverse(prev[0]))           # new frame = jump * old frame (the car stood still)
        self.map_T_odom = compose(self.map_T_odom, inverse(jump))
        if self.last_odom_pose is not None:
            self.last_odom_pose = compose(jump, self.last_odom_pose)
        if self.alternatives is not None:   # carried by the odometry: into the new frame
            self.alternatives.odom0 = compose(jump, self.alternatives.odom0)
            self.alternatives.last_odom = compose(jump, self.alternatives.last_odom)
        # a search running or finished but not yet polled is carried by relative(odometry then, now): across the
        # jump that is the jump itself (a verify moved the car 2.7 m). It is dropped; the next scan checks.
        self.generation += 1
        self.relocation = None   # a candidate from before is carried through the old frame: it never agrees
        self.odom_hist.clear()   # the old frame's poses
        self.odom_hist.append((stamp, pose))
        self.jumped = step

    def broadcast(self, stamp):
        x, y, yaw = self.map_T_odom
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.frame, 'odom'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = yaw_quaternion(yaw)
        self.tf_broadcaster.sendTransform(t)

    # ---------------- scans: the filter update ----------------
    def get_sensor_pose(self, frame):
        if frame not in self.sensor_pose:
            try:
                t = self.tf_buffer.lookup_transform('base_link', frame, Time())
            except TransformException as e:
                if time.monotonic() - self.warn_t > 2.0:
                    self.warn_t = time.monotonic()
                    self.get_logger().warning(f'waiting for TF base_link -> {frame}: {e}')
                return None
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.sensor_pose[frame] = (t.transform.translation.x, t.transform.translation.y, yaw)
        return self.sensor_pose[frame]

    def scan_cb(self, msg):
        if self.odom is None or self.pending is not None:
            return
        sensor = self.get_sensor_pose(msg.header.frame_id)
        if sensor is None:
            return
        ranges, angles = select_beams(msg.ranges, msg.angle_min, msg.angle_increment, self.beams,
                                      max(msg.range_min, 0.05), msg.range_max)
        if len(ranges) < MIN_BEAMS:
            return
        self.scan_odom = self.odom_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.log_latency(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.poll_search()
        if self.search is not None:
            now = time.monotonic()
            if self.job is None and now - self.last_search >= self.search_wait:
                self.last_search = now
                force = self.attempts >= self.force_after
                self.submit_search('find', ranges, angles, sensor, self.priors, force,
                                   self.relaxed_min_fit if force else self.min_fit)
            return
        if self.jumped is not None:   # the odometry frame jumped (restart, respawn): pose kept, now check it
            self.get_logger().warning(f'odometry jumped {self.jumped:.2f} m (reset or respawn?): keeping the pose, '
                                      'checking it against the map')
            self.jumped = None
            self.generation += 1   # a running search's result is carried by the odometry: the jump spoils it
            self.job = None
            if self.localized:
                self.submit_search('check', ranges, angles, sensor, [self.pf.estimate()], False,
                                   self.relocate_floor())
        odom_pose = self.scan_odom = self.odom_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        delta = relative(self.last_odom_pose, odom_pose)
        trans = math.hypot(delta[0], delta[1])
        moved = trans >= self.update_min_d or abs(delta[2]) >= self.update_min_a
        if not moved and self.localized:
            return
        if moved:   # before localizing, scans are used while standing still: the start pose is known
            self.pf.predict(delta)
            self.last_odom_pose = odom_pose
        self.pf.update_sensor(ranges, angles, sensor, self.lmap)
        self.pf.resample()
        self.map_T_odom = compose(self.pf.estimate(), inverse(odom_pose))
        self.check_localized(*self.pf.spread())
        if self.localized and self.search is None:
            self.check_fit(ranges, angles, sensor, odom_pose)
        if self.localized and self.alternatives is not None and self.search is None:
            self.validate(ranges, angles, sensor, odom_pose)
        if self.localized and self.search is None:
            self.recheck_guess(ranges, angles, sensor, trans)
        self.updates += 1
        if self.publish_particles and self.updates % 5 == 0:
            self.publish_cloud(msg.header.stamp)

    def log_latency(self, stamp):
        """Every LATENCY_LOG_PERIOD s: how much older the scans are than the newest /odom (what odom_at makes up
        for; 0 in the sim), to check on a real car's bag."""
        if self.odom_hist and stamp > 0.0:
            self.latency.append(self.odom_hist[-1][0] - stamp)
        now = time.monotonic()
        if now - self.latency_log_t >= LATENCY_LOG_PERIOD and self.latency:
            lat = np.array(self.latency) * 1000.0
            self.get_logger().info(f'scan stamps vs the newest /odom: {np.mean(lat):.0f} ms mean, '
                                   f'{np.max(lat):.0f} ms max over {len(lat)} scans')
            self.latency, self.latency_log_t = [], now

    def odom_at(self, stamp):
        """The odometry pose at a scan's stamp, interpolated between the two /odom messages around it: the
        newest /odom is up to the lidar's latency later than the scan (30-50 ms on a real car: 0.25-0.4 m at
        8 m/s), and the filter would put the scan there. Newer than every /odom: the newest (in the sim the scan
        shares its stamp with the /odom just before it). The newest too without history, with interpolate_odom
        false, and for a stamp more than MAX_SCAN_LATENCY older than the newest /odom or before the history (an
        unset stamp, a lidar on another clock): better a little lag than a pose from the wrong time."""
        hist = self.odom_hist
        if (not self.interpolate_odom or len(hist) < 2 or stamp >= hist[-1][0] or stamp < hist[0][0]
                or hist[-1][0] - stamp > MAX_SCAN_LATENCY):
            return self.odom[0]
        for k in range(len(hist) - 1, 0, -1):
            (t0, a), (t1, b) = hist[k - 1], hist[k]
            if t0 <= stamp <= t1:
                f = (stamp - t0) / (t1 - t0) if t1 > t0 else 1.0
                yaw = float(wrap(a[2] + f * wrap(b[2] - a[2])))
                return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), yaw
        return self.odom[0]

    # ---------------- global search, in the worker thread ----------------
    def submit_search(self, kind, ranges, angles, sensor, priors, force, floor, odom_then=None):
        """kind 'find': the car is not localized; 'check': localized, but the scan has not fit for a while;
        'verify': the whole-area search behind a raceline-first answer. odom_then: the odometry pose at the
        scan (default: now)."""
        hybrid = self.hybrid and kind == 'find'
        self.job = (self.pool.submit(self.search_job, ranges, angles, sensor, list(priors), force, floor, hybrid,
                                     kind != 'check'),
                    kind, odom_then or self.scan_odom or self.odom[0], self.generation, time.monotonic(),
                    (ranges, angles, sensor, list(priors)))

    def search_job(self, ranges, angles, sensor, priors, force, floor, hybrid, score_priors):
        """Worker thread: search_global (raceline first if hybrid), then choose_pose. Returns (stage, hypotheses,
        choice, its fit). Uses no node state but the (read-only) matcher.

        score_priors: where the car is expected (the map origin race.sh puts it at, the last known pose) is
        scored directly with a local search and joins the places found: the raceline-first search only looks
        near the raceline, and a start further from it lost to a look-alike elsewhere (iros2026). Fitting about
        as well, the expected pose wins the tie (choose_pose)."""
        def refine(q):
            return self.matcher.search_local(q, ranges, angles, sensor, radius=1.0, yaw_range=math.radians(10.0))

        expected = []

        def scored():   # once, and only when a search needs it
            if score_priors and not expected:
                expected.extend(self.local_search(q, ranges, angles, sensor, 1.0, math.radians(10.0)) for q in priors)
            return expected

        def best_fit(hyps):
            return max([h[1] for h in hyps[:1]] + [e[1] for e in scored()], default=0.0)

        def distinct(a, b):   # as search_global keeps its places apart
            return math.hypot(a[0] - b[0], a[1] - b[1]) > 1.0 or abs(wrap(a[2] - b[2])) > math.radians(30.0)

        def choose(hyps, force):
            hyps = sorted(hyps + [e for e in scored() if all(distinct(e[0], h[0]) for h in hyps)],
                          key=lambda h: -h[1])
            return choose_pose(hyps, priors, self.ambiguity, self.prior_radius, self.yaw_scale, force, refine)

        def fitted(choice):   # the choice's own fit (a refined or tied pick is not hyps[0])
            return choice, (self.matcher.fit(choice[0], ranges, angles, sensor) if choice is not None else 0.0)

        if hybrid:
            hyps = self.matcher.search_global(ranges, angles, sensor, seeds=self.hybrid_seeds, track_only=True)
            if best_fit(hyps) >= max(floor, self.hybrid_min_fit):
                choice = choose(hyps, False)
                if choice is not None:
                    return ('raceline', hyps, *fitted(choice))
        hyps = self.matcher.search_global(ranges, angles, sensor)
        if best_fit(hyps) < floor:
            return 'full', hyps, None, 0.0
        return ('full', hyps, *fitted(choose(hyps, force)))

    def poll_search(self):
        if self.job is None or not self.job[0].done():
            return
        future, kind, odom_then, generation, t0, scan = self.job
        self.job = None
        if generation != self.generation:
            return   # a reset, /initialpose or new search since it started
        try:
            stage, hyps, choice, fit = future.result()
        except Exception as e:   # never let a search failure kill the node: it is retried
            self.get_logger().error(f'global search failed: {e!r}')
            return
        ms = (time.monotonic() - t0) * 1000.0
        moved = relative(odom_then, self.odom[0])   # the car may have moved during the search
        if kind in ('check', 'verify', 'reverify'):
            # where the filter has the car at the scan (what the same scan says about it) and now: the filter's
            # pose is at its last update's odometry, neither the scan's nor the newest
            est_fit = self.matcher.fit(compose(self.map_T_odom, odom_then), *scan[:3])
            est_now = compose(self.map_T_odom, self.odom[0])
        if kind == 'check':
            self.on_check(choice, moved, ms, fit, est_fit, est_now)
        elif kind in ('verify', 'reverify'):
            self.on_verify(hyps, choice, moved, ms, fit, est_fit, est_now, may_move=kind == 'verify')
        else:
            self.on_find(hyps, choice, moved, ms, stage)
            if stage == 'raceline' and self.search is None:   # found: verify with the whole-area search
                self.submit_search('verify', *scan, False, self.min_fit, odom_then=odom_then)

    def on_find(self, hyps, choice, moved, ms, stage):
        if choice is None:
            self.attempts += 1
            self.search_wait = min(self.search_wait * 1.5, self.search_period_max)
            best = ', '.join(f'({h[0][0]:.1f}, {h[0][1]:.1f}, {math.degrees(h[0][2]):.0f} deg) fit {h[1]:.2f}'
                             for h in hyps[:3])
            why = ('no pose fits the scan' if not hyps or hyps[0][1] < self.min_fit else
                   f'{sum(h[1] >= self.ambiguity * hyps[0][1] for h in hyps)} places fit about as well and the '
                   'last known pose cannot tell')
            self.warn_find(f'{self.search}: {why} (best: {best or "none"}); retrying, a best guess after '
                           f'{self.force_after} tries (or /initialpose)')
            return
        pose, alternatives, how = choice
        self.init_filter(compose(pose, moved), f'{self.search}: found the car in {ms:.0f} ms ({how}'
                         + (', near the raceline; verifying' if stage == 'raceline' else '') + ')', matched=True)
        if alternatives:
            self.alternatives = Alternatives([compose(a, moved) for a in alternatives], self.odom[0],
                                             evidence=self.validate_evidence, distance=self.validate_distance)
            self.make_unsure()

    def on_verify(self, hyps, choice, moved, ms, fit, est_fit, est, may_move=True):
        """The whole-area search behind a raceline-first answer: confirm it, move the car if it is clearly
        elsewhere, or keep the places that fit as well as alternatives for driving to settle. Clearly elsewhere:
        the place fits well (relocate_floor) and better than the scan fits where the filter has the car
        (relocate_margin): the whole-area search keeps 40 seeds and can miss the true pose (3 in 40 scans with
        15 % of the beams shortened on iros2026_v2), and its 0.5 floor alone moved a driving car on one search.
        A place that does not pass is kept as an alternative instead, and so is every place a reverify (the
        searches after a guess, while driving) finds: there the alternatives' evidence decides, never one search.
        est: the filter's pose now."""
        far = (choice is not None
               and pose_distance(compose(choice[0], moved), est, self.yaw_scale) > self.local_radius)
        if may_move and far and fit >= max(self.relocate_floor(), est_fit + self.relocate_margin):
            pose = compose(choice[0], moved)
            self.get_logger().warning(f'verify: the whole-area search puts the car '
                                      f'{math.hypot(pose[0] - est[0], pose[1] - est[1]):.2f} m away ({choice[2]}, '
                                      f'{ms:.0f} ms): moving it there')
            self.init_filter(pose, 'verify', keep_localized=self.localized, quiet=True, matched=True)
            others = [est] + [compose(a, moved) for a in choice[1]]
        else:
            tied = [h[0] for h in hyps if hyps and h[1] >= self.ambiguity * hyps[0][1]]
            if far:
                tied.append(choice[0])
            others = [q for q in (compose(t, moved) for t in tied)
                      if pose_distance(q, est, self.yaw_scale) > self.local_radius]
            if others:
                self.get_logger().warning(f'verify: {len(others)} other place(s) fit as well ({ms:.0f} ms): '
                                          'checking them while driving')
            elif choice is not None:
                if self.unsure:   # the whole map has no other place fitting as well now: settled
                    self.get_logger().info(f'verify: the guess is confirmed by the whole-area search ({ms:.0f} ms)'
                                           + ('; alternatives dropped' if self.alternatives is not None else ''))
                    self.alternatives = None
                else:
                    self.get_logger().info(f'verify: confirmed by the whole-area search ({ms:.0f} ms)')
                self.unsure = False
        if others:
            if self.alternatives is None:
                self.alternatives = Alternatives(others, self.odom[0], evidence=self.validate_evidence,
                                                 distance=self.validate_distance)
            else:
                self.alternatives.add(others, self.odom[0])
            self.make_unsure()

    def on_check(self, choice, moved, ms, fit, est_fit, est):
        """A background global search while localized (a burst, a long low fit, a frame jump), submitted with the
        relocate_floor() floor, so a choice fits at least ~that well: move the car only if it is clearly elsewhere
        and the next search finds the same place (carried by the odometry). Where the map is wrong the true place
        fits only ~0.5 and look-alikes win: on iros2026_v2 the corridor across the wall, 2.1 m away, fits up to
        0.84 every lap, and two searches in a row found it; a place the car really is at fits 0.93-1.0 there."""
        if choice is None or not self.localized:
            self.relocation = None
            return
        pose = compose(choice[0], moved)
        if pose_distance(pose, est, self.yaw_scale) < self.local_radius or fit < est_fit + self.relocate_margin:
            self.relocation = None   # the car is where the filter has it (something the map does not have), or
            return                   # the place found fits no better than the filter's
        prev = self.relocation
        if prev is not None and time.monotonic() - prev[3] > RELOCATE_EXPIRY:
            prev = None   # too old to confirm anything
        if prev is None or pose_distance(compose(prev[0], relative(prev[1], self.odom[0])), pose,
                                         self.yaw_scale) > self.relocate_agree:
            # a first finding, or one that disagrees with the last: check_fit asks again, at most
            # RELOCATE_TRIES searches in a row (where the map is wrong the findings can keep disagreeing, and
            # each whole-area search costs ~1-2 s of the worker); the candidate waits for the next trigger then
            tries = 1 if prev is None else prev[2] + 1
            self.relocation = (pose, self.odom[0], tries, time.monotonic())
            if tries < RELOCATE_TRIES:
                self.burst = True
            return
        self.relocation = None
        self.get_logger().warning(f'relocated {math.hypot(pose[0] - est[0], pose[1] - est[1]):.2f} m by a global '
                                  f'search, confirmed by the one before ({ms:.0f} ms, {choice[2]})')
        self.init_filter(pose, 'relocate', keep_localized=True, quiet=True)

    def make_unsure(self):
        """After a guess (or a switch, or a verify that found other places): until a whole-area search finds the
        pick and no other place fitting as well, one runs every unsure_period m in the background (at most
        unsure_searches). The alternatives only settle the places already found: a look-alike picked while the
        true place was not among them (a guess with most of the view blocked) was otherwise never corrected.
        The budget (unsure_left) is per find, not per call: a persistent look-alike makes every search find
        it again, and re-arming on each would keep a whole-area search running every unsure_period m forever."""
        if not self.unsure:
            self.unsure_travel = 0.0
        self.unsure = True

    def recheck_guess(self, ranges, angles, sensor, trans):
        if not self.unsure or self.job is not None:
            return
        if self.unsure_left <= 0:
            self.unsure = False
            self.get_logger().warning(f'{self.unsure_searches} whole-area searches could not confirm the guess: '
                                      'keeping it (and the places still to check)')
            return
        self.unsure_travel += trans
        if self.unsure_travel < self.unsure_period:
            return
        self.unsure_travel = 0.0
        self.unsure_left -= 1
        self.submit_search('reverify', ranges, angles, sensor, [self.pf.estimate()], False, self.min_fit)

    def warn_find(self, text):
        now = time.monotonic()
        if text != self.find_log[0] or now - self.find_log[1] > FIND_LOG_PERIOD:
            self.find_log = (text, now)
            self.get_logger().warning(text)

    def validate(self, ranges, angles, sensor, odom_pose):
        """After a guessed pick: switch to an alternative that keeps fitting clearly better."""
        alt = self.alternatives
        est = self.pf.estimate()
        switch = alt.update(self.matcher, est, odom_pose, ranges, angles, sensor)
        if switch is not None:
            pose, evidence, _ = switch
            moved = relative(alt.odom0, odom_pose)
            others = [q for q in (compose(a, moved) for a in alt.poses)
                      if pose_distance(q, pose, self.yaw_scale) > 1.0] + [est]
            self.get_logger().warning(f'the guess was wrong: switching {math.hypot(pose[0] - est[0], pose[1] - est[1]):.2f} m '
                                      f'to another place that fits clearly better (by {evidence:.1f} fit x updates '
                                      f'over {alt.travelled:.1f} m)')
            self.init_filter(pose, 'switch', keep_localized=True, quiet=True, odom=odom_pose)
            self.alternatives = Alternatives(others, odom_pose, evidence=self.validate_evidence,
                                             distance=self.validate_distance)
            self.make_unsure()
        elif alt.done:
            if alt.poses:   # nothing on the way told them apart: keep the pick, say so
                self.get_logger().warning(f'guess kept: {alt.travelled:.0f} m of driving could not tell it from '
                                          f'{len(alt.poses)} other place(s) (ruled out {alt.ruled_out})')
            else:
                self.get_logger().info(f'guess confirmed: the scans ruled out {alt.ruled_out} other place(s) in '
                                       f'{alt.travelled:.1f} m of driving')
            self.alternatives = None

    def check_fit(self, ranges, angles, sensor, odom_pose):
        """Re-anchor the cloud when the scan stops fitting where it is (see the module docstring)."""
        est = self.pf.estimate()
        fit = self.matcher.fit(est, ranges, angles, sensor)
        if self.fit_level is None:
            self.fit_level = fit
        self.track_good_level(fit)
        # doubt against the good-fit level: fit_level follows the fit down after every failed local search
        # (0.97 -> 0.04 in 500 updates while the pose was wrong), so against it doubt never fired. An estimate
        # off the drivable area (outside the map, in a wall, on unseen ground) is doubtful whatever it fits.
        cell = int(self.lmap.cell(est[0], est[1]))
        low = fit < self.doubt_fit * self.good_level
        self.doubt = self.doubt + 1 if cell == 2 or low else 0
        # in a wall or outside the map, and not fitting, for as long: not a pose (the cloud drifted where no scan
        # contradicts it, 15 m off a 12 m map, still "localized"; it fits ~0); withdraw it like a spread cloud.
        # A pose that fits stays, even in an occupied blob (a person or box captured in the map): no false stop.
        self.offmap = self.offmap + 1 if cell == 2 and low else 0
        if self.offmap >= self.doubt_updates:
            self.offmap = 0
            self.get_logger().error(f'lost: the pose ({est[0]:.1f}, {est[1]:.1f}) is in a wall or off the map, '
                                    'not driving')
            self.start_search('lost')
            return
        if (self.burst or self.doubt >= self.doubt_updates) and self.job is None:
            self.doubt = 0
            self.burst = False
            self.submit_search('check', ranges, angles, sensor, [est], False, self.relocate_floor())
        holding = time.monotonic() < self.hold_until
        if fit >= self.fit_drop * self.fit_level:
            self.fit_bad = 0
            self.fit_level += 0.02 * (fit - self.fit_level)
            if not holding:
                self.recheck(est, fit, ranges, angles, sensor, odom_pose)
            return
        self.fit_bad += 1
        if self.fit_bad < self.fit_bad_updates or holding:
            return
        self.fit_bad = 0
        t0 = time.monotonic()
        best, best_fit = self.local_search(est, ranges, angles, sensor, self.local_radius, self.local_yaw)
        ms = (time.monotonic() - t0) * 1000.0
        if best_fit < max(fit + self.snap_gain, self.fit_drop * self.fit_level, self.reanchor_floor()):
            # nothing fits clearly better nearby: something the map does not have, or a stretch where the map
            # itself is wrong (no pose fits well, so the best of them is no better than the filter's)
            self.snap_offset = None
            self.fit_level += 0.02 * (fit - self.fit_level)
            return
        # one search can be fooled where the map fits badly: re-anchor only when the next one (fit_bad_updates
        # later) finds the same offset in the car's frame, as a slip or a stalled stream leaves it
        offset = relative(est, best)
        prev, self.snap_offset = self.snap_offset, offset
        if prev is None or (math.hypot(offset[0] - prev[0], offset[1] - prev[1]) > self.recheck_agree
                            or abs(wrap(offset[2] - prev[2])) > self.snap_agree_yaw):
            return
        self.snap_offset = None
        self.reanchor(best, odom_pose, f'{math.hypot(offset[0], offset[1]):.2f} m, '
                                       f'{math.degrees(wrap(offset[2])):+.1f} deg: fit {fit:.2f} -> {best_fit:.2f} '
                                       f'({ms:.0f} ms)')

    def recheck(self, est, fit, ranges, angles, sensor, odom_pose):
        """Every recheck_updates updates, a small local search: catches the error a low fit does not show, along
        a straight (the scan fits almost as well 0.4 m along it). Re-anchors only when two checks in a row find
        a better fit at the same offset, so noise and obstacles do not make the pose jump."""
        self.rechecks += 1
        if self.rechecks % self.recheck_updates:
            return
        best, best_fit = self.matcher.search_local(est, ranges, angles, sensor, radius=self.recheck_radius,
                                                   yaw_range=math.radians(4.0), yaw_step=math.radians(1.0))
        offset = relative(est, best)
        if (best_fit < max(fit + self.recheck_gain, self.reanchor_floor())
                or math.hypot(offset[0], offset[1]) < self.recheck_min_shift):
            self.recheck_offset = None
            return
        prev, self.recheck_offset = self.recheck_offset, offset
        if prev is None or math.hypot(offset[0] - prev[0], offset[1] - prev[1]) > self.recheck_agree:
            return
        self.recheck_offset = None
        self.reanchor(best, odom_pose, f'{math.hypot(offset[0], offset[1]):.2f} m ({offset[0]:+.2f} along, '
                                       f'{offset[1]:+.2f} across), fit {fit:.2f} -> {best_fit:.2f}')

    def track_good_level(self, fit):
        """The level of a good fit, for reanchor_floor / relocate_floor: a slow high percentile of the fit at the
        estimate. It rises with any better fit, follows fits within 5 % of it, and sinks only very slowly below that
        (0.05 % of the gap per update: a ~10 m stretch of bad map, 200 updates, takes ~0.04 off it, and the next
        good stretch puts that back). Unlike fit_level it is not dragged down where nothing fits. A lidar or map
        that fits less everywhere still brings it down: it starts at the first fit, and a uniformly lower fit
        pulls it there within ~1400 updates (~70 m of driving)."""
        if self.good_level is None:
            self.good_level = fit
        elif fit > self.good_level:
            self.good_level += 0.05 * (fit - self.good_level)
        elif fit >= 0.95 * self.good_level:
            self.good_level += 0.02 * (fit - self.good_level)
        else:
            self.good_level += 0.0005 * (fit - self.good_level)

    def reanchor_floor(self):
        """Fit a re-anchor must reach: reanchor_min_fit, or less on a map/lidar where even good stretches fit less
        (reanchor_ratio x the good-fit level), so the gate scales instead of silently disabling re-anchors."""
        level = self.good_level if self.good_level is not None else 1.0
        return max(self.min_fit, min(self.reanchor_min_fit, self.reanchor_ratio * level))

    def relocate_floor(self):
        """The same for a background global search moving the car (relocate_min_fit, relocate_ratio)."""
        level = self.good_level if self.good_level is not None else 1.0
        return max(self.min_fit, min(self.relocate_min_fit, self.relocate_ratio * level))

    def local_search(self, pose, ranges, angles, sensor, radius, yaw_range):
        """Best (pose, fit) near pose: coarse (0.1 m) within radius / yaw_range, then fine around that; a few ms."""
        coarse, _ = self.matcher.search_local(pose, ranges, angles, sensor, radius=radius, yaw_range=yaw_range,
                                              step=0.1, yaw_step=min(math.radians(3.0), yaw_range / 2.0))
        return self.matcher.search_local(coarse, ranges, angles, sensor, radius=0.1, yaw_range=math.radians(2.0),
                                         yaw_step=math.radians(1.0))

    def reanchor(self, pose, odom_pose, what):
        """Move the cloud to a scan-matched pose without stopping the car; log at most every LOG_PERIOD s.
        slip_reanchors in LOG_PERIOD s is a burst: where the map fits badly, each local search can pull the pose
        a little further off. Local re-anchors pause for slip_hold s and a global search in the background
        (never holding the car) decides whether the car is clearly elsewhere."""
        self.init_filter(pose, 're-anchor', keep_localized=True, quiet=True, odom=odom_pose)
        now = time.monotonic()
        self.anchors = [t for t in self.anchors if now - t < LOG_PERIOD] + [now]
        n = len(self.anchors)
        if n >= self.slip_reanchors:
            self.anchors = []
            self.hold_until = now + self.slip_hold
            self.burst = True
            self.anchor_log_t = now
            self.get_logger().warning(f're-anchored {n} times in {LOG_PERIOD:.0f} s, last {what}: the scan keeps '
                                      'contradicting the odometry (wheels slipping, the car blocked, or the map '
                                      f'fits badly here): no local re-anchors for {self.slip_hold:.0f} s, checking '
                                      'with a background search')
            return
        if now - self.anchor_log_t < LOG_PERIOD:
            return
        self.anchor_log_t = now
        self.get_logger().warning(f're-anchored {what}')

    def check_localized(self, pos_std, yaw_std):
        if self.localized:
            if pos_std > self.lost_xy:
                self.get_logger().error(f'lost: particles spread {pos_std:.2f} m, not driving')
                self.start_search('lost')
        else:
            self.good = self.good + 1 if pos_std <= self.conv_xy and yaw_std <= self.conv_yaw else 0
            if self.good >= self.conv_updates:
                self.localized = True
                self.get_logger().info(f'localized (spread {pos_std:.3f} m, {yaw_std:.3f} rad), driving')

    def publish_cloud(self, stamp):
        msg = PoseArray()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame
        step = max(1, self.pf.n // MAX_PARTICLES_SHOWN)
        for x, y, yaw in self.pf.p[::step]:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.z, pose.orientation.w = yaw_quaternion(float(yaw))
            msg.poses.append(pose)
        self.particles_pub.publish(msg)

    # ---------------- resets and manual initial poses ----------------
    def reset_cb(self, msg):
        if self.localized or self.pending is None:
            self.get_logger().warning(f'sim reset ({msg.data}): relocalizing, not driving')
        self.localized = False
        self.good = 0
        self.alternatives = None
        self.generation += 1
        self.pending = time.monotonic()

    def initialpose_cb(self, msg):
        if msg.header.frame_id not in ('', self.frame):
            self.get_logger().warning(f"/initialpose in frame '{msg.header.frame_id}', needs '{self.frame}'")
            return
        if self.odom is None:
            self.get_logger().warning('/initialpose before /odom: ignored')
            return
        self.init_filter(pose2d(msg.pose.pose), '/initialpose')

    def tick(self):
        """After a reset: search for the car once it has settled."""
        if self.pending is None or self.odom is None:
            return
        waited = time.monotonic() - self.pending
        if waited < self.debounce or (self.odom[2] >= self.still_speed and waited < self.timeout):
            return
        self.pending = None
        self.start_search('sim reset')   # the cloud was not updated since the reset: priors = the pose before it


def main():
    rclpy.init()
    node = MclNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
