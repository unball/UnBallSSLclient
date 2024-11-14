import socket
import time
import math
from protocols.grsim.ssl_simulation_robot_control_pb2 import (
    RobotControl,
    RobotCommand,
    RobotMoveCommand,
    MoveWheelVelocity,
    MoveGlobalVelocity,
    MoveLocalVelocity,
)

from simulation_grsim.RobotControlClient import RobotControlClient


def circular_movement():
    client = RobotControlClient(team_port=10301)  # Blue team

    # Parameters for circular movement
    robot_id = 0
    radius = 0.5  # Radius of circle in meters
    angular_speed = 1.0  # Radians per second
    linear_speed = radius * angular_speed  # Linear velocity at circle edge

    try:
        start_time = time.time()
        while True:
            # Calculate current angle
            t = time.time() - start_time
            angle = angular_speed * t

            # Calculate velocities for circular motion
            vx = -linear_speed * math.sin(angle)  # X component
            vy = linear_speed * math.cos(angle)  # Y component

            # Send command
            client.send_global_velocity(
                robot_id=robot_id,
                vx=vx,
                vy=vy,
                angular=angular_speed,  # Keep robot oriented tangent to circle
            )

            # Control rate
            time.sleep(1 / 60)  # 60Hz update rate

    except KeyboardInterrupt:
        # Stop robot when Ctrl+C is pressed
        client.send_global_velocity(robot_id, 0, 0, 0)
        print("Stopped")


def figure_eight_movement():
    client = RobotControlClient(team_port=10301)  # Blue team

    # Parameters
    robot_id = 0
    radius = 1.0  # Radius of each circle in meters
    angular_speed = 1.0  # Radians per second
    linear_speed = radius * angular_speed

    try:
        start_time = time.time()
        while True:
            # Calculate current angle
            t = time.time() - start_time
            angle = angular_speed * t

            # Figure-8 pattern using parametric equations
            vx = linear_speed * math.cos(angle)
            vy = linear_speed * math.sin(2 * angle) / 2

            # Calculate desired angular velocity to face movement direction
            desired_angle = math.atan2(vy, vx)

            client.send_global_velocity(
                robot_id=robot_id, vx=vx, vy=vy, angular=desired_angle
            )

            time.sleep(1 / 60)

    except KeyboardInterrupt:
        client.send_global_velocity(robot_id, 0, 0, 0)
        print("Stopped")


def spiral_movement():
    client = RobotControlClient(team_port=10301)  # Blue team

    # Parameters
    robot_id = 0
    initial_radius = 0.01  # Starting radius in meters
    max_radius = 2.0  # Maximum radius
    angular_speed = 2.0  # Radians per second
    growth_rate = 0.1  # How fast spiral grows

    try:
        start_time = time.time()
        while True:
            # Calculate current time and angle
            t = time.time() - start_time
            angle = angular_speed * t

            # Calculate growing radius
            current_radius = min(initial_radius + growth_rate * t, max_radius)
            linear_speed = current_radius * angular_speed

            # Calculate velocities
            vx = linear_speed * math.cos(angle)
            vy = linear_speed * math.sin(angle)

            client.send_global_velocity(
                robot_id=robot_id, vx=vx, vy=vy, angular=angular_speed
            )

            time.sleep(1 / 60)

    except KeyboardInterrupt:
        client.send_global_velocity(robot_id, 0, 0, 0)
        print("Stopped")


if __name__ == "__main__":
    print("Select movement pattern:")
    print("1. Circle")
    print("2. Figure-8")
    print("3. Spiral")

    choice = input("Choice (1-3): ")

    if choice == "1":
        circular_movement()
    elif choice == "2":
        figure_eight_movement()
    elif choice == "3":
        spiral_movement()
    else:
        print("Invalid choice!")
