import socket
import time
import math
import sys
import tty
import termios
import os
from protocols.grsim.ssl_simulation_robot_control_pb2 import (
    RobotControl,
    RobotCommand,
    RobotMoveCommand,
    MoveWheelVelocity,
    MoveGlobalVelocity,
    MoveLocalVelocity,
)

from simulation_grsim.RobotControlClient import RobotControlClient


class RobotController:
    def __init__(self, ip: str = "localhost"):
        self.blue_client = RobotControlClient(team_port=10301)
        self.yellow_client = RobotControlClient(team_port=10302)
        self.current_client = self.blue_client
        self.current_robot_id = 0
        self.speed = 20.0
        self.control_rate = 220
        self.last_update_time = 0
        self.control_mode = "global"  # Default to global velocity control

    def _getch(self):
        """Get a single character from stdin"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def _clear_screen(self):
        """Clear the screen"""
        os.system("cls" if os.name == "nt" else "clear")

    def _display_controls(self):
        """Display the current controls to the user"""
        self._clear_screen()
        print(
            f"Current team: {'Blue' if self.current_client == self.blue_client else 'Yellow'}"
        )
        print(f"Current robot: {self.current_robot_id}")
        print(f"Control mode: {self.control_mode}")
        print("Controls:")
        print("- WASD: Move robot (global velocity)")
        print("- QE: Rotate robot (global velocity)")
        print("- IJKL: Move robot (wheel velocity)")
        print("- b/y: Switch team")
        print("- n: Select robot")
        print("- m: Switch control mode")
        print("- q: Quit")

    def _handle_key_press(self):
        """Handle key presses to control the robot"""
        while True:
            self._display_controls()
            key = self._getch()
            current_time = time.time()
            if current_time - self.last_update_time >= 1 / self.control_rate:
                if self.control_mode == "global":
                    if key == "w":
                        self.current_client.send_global_velocity(
                            self.current_robot_id, self.speed, 0, 0
                        )
                    elif key == "s":
                        self.current_client.send_global_velocity(
                            self.current_robot_id, -self.speed, 0, 0
                        )
                    elif key == "a":
                        self.current_client.send_global_velocity(
                            self.current_robot_id, 0, -self.speed, 0
                        )
                    elif key == "d":
                        self.current_client.send_global_velocity(
                            self.current_robot_id, 0, self.speed, 0
                        )
                    elif key == "q":
                        self.current_client.send_global_velocity(
                            self.current_robot_id, 0, 0, -self.speed
                        )
                    elif key == "e":
                        self.current_client.send_global_velocity(
                            self.current_robot_id, 0, 0, self.speed
                        )
                elif self.control_mode == "wheel":
                    if key == "i":
                        self.current_client.send_wheel_velocity(
                            self.current_robot_id,
                            self.speed,
                            self.speed,
                            self.speed,
                            self.speed,
                        )
                    elif key == "k":
                        self.current_client.send_wheel_velocity(
                            self.current_robot_id,
                            -self.speed,
                            -self.speed,
                            -self.speed,
                            -self.speed,
                        )
                    elif key == "j":
                        self.current_client.send_wheel_velocity(
                            self.current_robot_id,
                            -self.speed,
                            self.speed,
                            self.speed,
                            -self.speed,
                        )
                    elif key == "l":
                        self.current_client.send_wheel_velocity(
                            self.current_robot_id,
                            self.speed,
                            -self.speed,
                            -self.speed,
                            self.speed,
                        )
                if key == "b":
                    self.current_client = self.blue_client
                    self.current_robot_id = 0
                elif key == "y":
                    self.current_client = self.yellow_client
                    self.current_robot_id = 0
                elif key == "n":
                    self._select_robot()
                elif key == "m":
                    self.control_mode = (
                        "wheel" if self.control_mode == "global" else "global"
                    )
                    print(f"Switched to {self.control_mode} control mode")
                    time.sleep(1)
                elif key == "q":
                    self.current_client.send_global_velocity(
                        self.current_robot_id, 0, 0, 0
                    )
                    break
                self.last_update_time = current_time

    def _select_robot(self):
        """Allow user to select the robot to control"""
        self._display_controls()
        robot_id = input("Enter robot ID (0-2): ")
        try:
            self.current_robot_id = int(robot_id)
            print(f"Controlling robot {self.current_robot_id}")
            time.sleep(2)
        except ValueError:
            print("Invalid robot ID")
            time.sleep(2)


def main():
    controller = RobotController()
    controller._handle_key_press()


if __name__ == "__main__":
    main()
