import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import json

class UDPTeleop(Node):
    def __init__(self)
        super().__init__('udp_teleop')
        self.steer_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', 10)
        
        # Listen on the UDP port we opened
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5005))
        self.sock.setblocking(False)
        
        self.timer = self.create_timer(0.05, self.receive_data)

    def receive_data(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            cmds = json.loads(data.decode())
            
            s_msg, t_msg = Float32(), Float32()
            s_msg.data = float(cmds['steering'])
            t_msg.data = float(cmds['throttle'])
            
            self.steer_pub.publish(s_msg)
            self.throttle_pub.publish(t_msg)
        except BlockingIOError:
            pass

def main():
    rclpy.init()
    node = UDPTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()