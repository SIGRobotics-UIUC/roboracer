# Every obstacle perception believes in on this cycle.
#
# An empty array is meaningful: it asserts that nothing is currently seen, which
# is different from publishing nothing at all. Publishers should emit on every
# cycle so that consumers can distinguish "clear track" from "perception died".
#
# header.frame_id is the frame the Frenet coordinates are measured against,
# i.e. the global raceline, not a TF frame.

std_msgs/Header header
Obstacle[] obstacles
