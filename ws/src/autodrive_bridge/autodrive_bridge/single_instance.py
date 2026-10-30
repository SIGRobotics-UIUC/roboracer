"""Keeps a bridge restart from running two copies of a node side by side.

ros2 launch forwards nothing to its nodes on a SIGINT sent to the launch process alone (Humble's
execute_local: send_sigint=not due_to_sigint); they get SIGTERM after 5 s and SIGKILL after 10 s.
A bridge restarted within that window, or after the launch itself died, ran a second adapter
and wheel_odom beside the old ones (the new fast_bridge could not bind :4567), and the two
wheel_odoms' duplicate /odom stamps aborted Cartographer.

  exit_with_parent()   the node gets SIGTERM when its parent (ros2 launch, ros2 run) dies
  DuplicateGuard       holds a node's output while another copy of it runs
"""
import ctypes
import os
import signal
import sys
import time

PR_SET_PDEATHSIG = 1   # linux/prctl.h


def exit_with_parent(sig=signal.SIGTERM):
    """Ask the kernel for `sig` when the parent process dies (PR_SET_PDEATHSIG). Call first in main(),
    before rclpy.init: a parent already gone sends it at once, still with the default action. Linux
    only (the sim container, the car's Jetson); a no-op elsewhere."""
    if not sys.platform.startswith('linux'):
        return False
    ppid = os.getppid()
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(PR_SET_PDEATHSIG, int(sig), 0, 0, 0) != 0:
            return False
    except (OSError, AttributeError):
        return False
    if os.getppid() != ppid:   # the parent died before the prctl
        os.kill(os.getpid(), sig)
    return True


def topic_publishers(node, topic, own=1):
    """For DuplicateGuard: the publishers of `topic` ('/ns/node' each) if there are more than this
    node's `own`, else []."""
    infos = node.get_publishers_info_by_topic(topic)
    if len(infos) <= own:
        return []
    return sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in infos)


def same_name_nodes(node):
    """For DuplicateGuard: the nodes with this node's name and namespace if there is more than one, else []."""
    me = (node.get_name(), node.get_namespace())
    found = [f'{ns.rstrip("/")}/{n}' for n, ns in node.get_node_names_and_namespaces() if (n, ns) == me]
    return found if len(found) > 1 else []


class DuplicateGuard:
    """`ok` once no other copy of the node is seen: check `others()` (a list naming what was found,
    empty when alone) every `period` s from `delay` s after start (discovery).

    A copy that finds another when it starts holds its output (ok stays False) until that one is
    gone: the newer copy yields to the running stack. A copy that is already running keeps going
    and logs an error, the newer one holds. A crashed node can stay in the graph until its DDS lease
    runs out, so a restart after a crash may hold for a while; it says so.
    """

    def __init__(self, node, what, others, delay=2.0, period=1.0):
        self.node = node
        self.what = what               # the held output, for the log
        self.others = others
        self.delay = delay
        self.ok = False
        self.held = False              # held after the discovery delay, for the release message
        self.t0 = time.monotonic()
        node.create_timer(period, self.check)

    def check(self):
        if time.monotonic() - self.t0 < self.delay:
            return
        found = self.others()
        log = self.node.get_logger()
        if not found:
            if self.held:
                log.info(f'the other copy is gone; publishing {self.what}')
            self.ok, self.held = True, False
            return
        if self.ok:
            log.error(f'another copy publishes beside this one: {", ".join(found)} (a second bridge launch, '
                      f'or a bag replay?); this one keeps publishing {self.what}', throttle_duration_sec=10.0)
        else:
            self.held = True
            log.warn(f'another copy is running: {", ".join(found)}; holding {self.what} until it stops '
                     f'(stop the old bridge: Ctrl-C in its terminal)', throttle_duration_sec=5.0)
