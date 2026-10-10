import pygame
import socket
import json
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected! Connect your PS5 controller via Bluetooth.")
    exit()

joy = pygame.joystick.Joystick(0)
joy.init()
print(f"Connected to: {joy.get_name()}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    pygame.event.pump()
    
    # Axis 0: Left Stick X (Steering)
    # Axis 3: Right Stick Y (Throttle). Up is negative, so we invert it.
    steering = -joy.get_axis(0) 
    throttle = -joy.get_axis(3)

    # Deadzones to stop drifting
    if abs(steering) < 0.1: steering = 0.0
    if abs(throttle) < 0.1: throttle = 0.0

    # Scale throttle to 30% max so you get a nice, slow, smooth mapping speed!
    data = {"steering": steering, "throttle": throttle * 0.3} 
    sock.sendto(json.dumps(data).encode(), ("127.0.0.1", 5005))
    
    time.sleep(0.05) # 20 Hz
    