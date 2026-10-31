from autodrive_bridge.steady_time import SteadyEpoch


def test_epoch_clock_ignores_forward_and_backward_wall_steps(monkeypatch):
    clock=SteadyEpoch(wall_ns=1700000000000000000,monotonic_ns=1000000000)
    for offset in (2500000000,-5000000000):
        monkeypatch.setattr('autodrive_bridge.steady_time.time.time_ns',lambda:1700000000000000000+offset)
        assert clock.now_ns(1068000000) == 1700000000068000000
        assert clock.now_ns(1136000000) == 1700000000136000000


def test_clock_publishes_without_sensor_frames_and_survives_wall_step(monkeypatch):
    import time
    import rclpy
    from rclpy.parameter import Parameter
    from autodrive_bridge.fast_bridge import FastBridge
    original = rclpy.create_node
    def node(name):
        return original(name, parameter_overrides=[Parameter('publish_clock',value=True),
                                                  Parameter('use_sim_time',value=True)])
    monkeypatch.setattr(rclpy,'create_node',node)
    bridge=FastBridge()
    try:
        deadline=time.monotonic()+2
        while bridge.node.get_clock().now().nanoseconds == 0 and time.monotonic()<deadline:
            rclpy.spin_once(bridge.node,timeout_sec=.02)
        before=bridge.node.get_clock().now().nanoseconds
        assert before>0
        wall=time.time_ns()
        monkeypatch.setattr('autodrive_bridge.steady_time.time.time_ns',lambda:wall+2500000000)
        deadline=time.monotonic()+.1
        while time.monotonic()<deadline:
            rclpy.spin_once(bridge.node,timeout_sec=.02)
        after=bridge.node.get_clock().now().nanoseconds
        assert .05 < (after-before)*1e-9 < .3
    finally:
        bridge.node.destroy_node()


def test_inactive_clock_does_not_publish_before_socket_ownership(monkeypatch):
    import rclpy
    from rclpy.parameter import Parameter
    from autodrive_bridge.fast_bridge import FastBridge
    original=rclpy.create_node
    monkeypatch.setattr(rclpy,'create_node',lambda name:original(name,parameter_overrides=[Parameter('publish_clock',value=True)]))
    bridge=FastBridge(clock_active=False)
    class Recorder:
        def __init__(self): self.messages=[]
        def publish(self,msg): self.messages.append(msg)
    recorder=Recorder();bridge.clock_pub=recorder
    try:
        bridge.publish_clock()
        assert not recorder.messages
        bridge.clock_active=True
        bridge.publish_clock()
        assert len(recorder.messages)==1
    finally:
        bridge.node.destroy_node()
