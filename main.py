#!/usr/bin/env python3
"""
Sistema Cliente SSL para RoboCup - UnBall (Universidade de Brasília)
"""

import sys
import warnings
import os

warnings.filterwarnings("ignore", category=DeprecationWarning)
os.environ["GTK_MODULES"] = ""

import json
import threading
import time
import math
import signal
import traceback
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QVBoxLayout,
)
from PyQt5.QtCore import Qt, QTimer

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Project imports
from PyQt.main_window import MainWindow
from GameController.GameController import GameController
from VisionClient.Vision import Vision
from SimulationGrSim.RobotControlClient_threaded import ThreadedRobotControlClient
from PyQt.field_visualization import FieldVisualization


class Game:
    def __init__(self) -> None:
        # Load Config
        self.config = get_config()

        self.debug = {
            "vision": False,
            "referee": False,
            "threads": False,
            "timing": False,
            "all": False,
        }
        self.current_command = None
        self.command_thread = None

        self.field_bounds = {"x_min": 0, "x_max": 0, "y_min": 0, "y_max": 0}

        print("Initializing Game...")

        # Initialize robot control clients
        self.blue_robots = ThreadedRobotControlClient(team_port=10301)
        self.yellow_robots = ThreadedRobotControlClient(team_port=10302)
        print("Robot control clients initialized")

        # Input Layer
        self.vision = Vision(self)
        self.referee = GameController(self)
        print("Vision and referee systems initialized")

        # State tracking
        self.running = False
        self._update_thread = None
        self._fps = 60
        self.last_vision_data = None
        self.last_referee_data = None

        # UI tracking
        self.window = None
        self.visualization_timer = None

    def start(self):
        """Start all components"""
        print("Starting game systems...")

        # Start robot control
        self.blue_robots.start()
        self.yellow_robots.start()
        print("Robot control clients started")

        # Start vision and referee
        self.vision.start()
        self.referee.start()
        print("Vision and referee systems started")

        self.running = True
        self._update_thread = threading.Thread(target=self.update_loop)
        self._update_thread.daemon = True
        self._update_thread.start()
        print("Main update loop started")

    def check_bounds(self, x, y):
        """Check if position is within field bounds"""
        return (
            self.field_bounds["x_min"] <= x <= self.field_bounds["x_max"]
            and self.field_bounds["y_min"] <= y <= self.field_bounds["y_max"]
        )

    def update_field_bounds(self):
        """Update field bounds from vision geometry"""
        if self.vision and self.vision.raw_geometry:
            field_length = self.vision.raw_geometry["fieldLength"]
            field_width = self.vision.raw_geometry["fieldWidth"]

            # Field dimensions are centered at 0,0
            self.field_bounds = {
                "x_min": -field_length / 2,
                "x_max": field_length / 2,
                "y_min": -field_width / 2,
                "y_max": field_width / 2,
            }
            # print(f"Updated field bounds: {self.field_bounds}")  # Debug print

    def set_game_controller_enabled(self, enabled):
        """Enable/disable game controller"""
        self.game_controller_enabled = enabled
        print(f"Game controller {'enabled' if enabled else 'disabled'}")

    def execute_movement(self):
        """Continuous movement execution"""
        while self.running and self.current_command:
            try:
                if self.current_command == "FORCE_START":
                    # Triangle formation movement with boundary checking
                    for i, (dx, dy) in enumerate(
                        [
                            (0.5, 0.5),  # Robot 0: diagonal right
                            (-0.5, 0.5),  # Robot 1: diagonal left
                            (0, -0.5),  # Robot 2: backward
                        ]
                    ):
                        robot = self.blue_robots.get_position(i)
                        new_x = robot["x"] + dx * 0.1  # Scale movement
                        new_y = robot["y"] + dy * 0.1

                        if self.check_bounds(new_x, new_y):
                            self.blue_robots.send_global_velocity(i, dx, dy, 0)
                        else:
                            # Reverse direction if hitting boundary
                            self.blue_robots.send_global_velocity(i, -dx, -dy, 0)

                    # Similar for yellow team
                    for i, dx in enumerate([-0.5, 0, 0.5]):
                        robot = self.yellow_robots.get_position(i)
                        new_x = robot["x"] + dx * 0.1

                        if self.check_bounds(new_x, robot["y"]):
                            self.yellow_robots.send_global_velocity(i, dx, 0, 0)
                        else:
                            self.yellow_robots.send_global_velocity(i, -dx, 0, 0)

                elif self.current_command == "NORMAL_START":
                    # Circular movement with boundary checking
                    t = time.time()
                    for i in range(3):
                        radius = 0.5 + i * 0.2
                        x = radius * math.cos(t)
                        y = radius * math.sin(t)

                        if self.check_bounds(x, y):
                            self.blue_robots.send_global_velocity(i, x, y, math.pi / 2)
                        else:
                            # Reduce radius if hitting boundary
                            self.blue_robots.send_global_velocity(
                                i, x / 2, y / 2, math.pi / 2
                            )

                time.sleep(1.0 / self._fps)

            except Exception as e:
                print(f"Error in movement execution: {str(e)}")

    def test_movement(self):
        """Test random movements for robots"""
        import random
        import math

        # Simple test patterns
        def circle_position(robot_id, time, radius=1.0, frequency=0.5):
            angle = time * frequency * 2 * math.pi
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            return x, y, angle

        def oscillate_position(robot_id, time, amplitude=1.0, frequency=0.5):
            x = amplitude * math.sin(time * frequency * 2 * math.pi)
            y = 0.5 * math.cos(time * frequency * 2 * math.pi)
            return x, y, math.pi / 4

        start_time = time.time()

        while self.running:
            try:
                current_time = time.time() - start_time

                # Move blue robots in a circle
                for i in range(3):
                    x, y, theta = circle_position(i, current_time, radius=1.0 + i * 0.2)
                    self.blue_robots.send_global_velocity(i, x, y, theta)

                # Move yellow robots in oscillation
                for i in range(3):
                    x, y, theta = oscillate_position(
                        i, current_time, amplitude=1.0 + i * 0.2
                    )
                    self.yellow_robots.send_global_velocity(i, x, y, theta)

                time.sleep(1.0 / self._fps)

            except Exception as e:
                print(f"Error in test movement: {str(e)}")

    def update_loop(self):
        """Main game logic update loop"""
        while self.running:
            try:
                # Update vision data if available
                if self.vision.new_data:
                    self.last_vision_data = self.vision.get_last_frame()
                    # Update field bounds when we get new geometry
                    self.update_field_bounds()

                # Update referee data independently
                if self.referee.new_data or self.referee.any_referee:
                    self.last_referee_data = self.referee.get_state()

                # Handle robot control based on referee state
                if (
                    self.last_referee_data
                    and self.last_referee_data["command"] == "HALT"
                ):
                    self.blue_robots.send_global_velocity(0, 0, 0, 0)
                    self.yellow_robots.send_global_velocity(0, 0, 0, 0)
                elif self.last_vision_data:
                    pass  # Robot control logic here

                time.sleep(1.0 / self._fps)

            except Exception as e:
                print(f"Error in update loop: {str(e)}")

    def stop(self):
        """Stop all running threads and components"""
        print("Stopping all systems...")
        self.running = False

        # Stop robot control clients
        if hasattr(self, "blue_robots"):
            self.blue_robots.stop()
        if hasattr(self, "yellow_robots"):
            self.yellow_robots.stop()

        # Stop vision and referee
        if hasattr(self, "vision"):
            self.vision.stop()
        if hasattr(self, "referee"):
            self.referee.stop()

        # Stop visualization timer
        if self.visualization_timer:
            self.visualization_timer.stop()

        # Wait for update thread
        if self._update_thread and self._update_thread.is_alive():
            try:
                self._update_thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error joining update thread: {e}")

        print("All systems stopped")

    def get_vision_data(self):
        """Get latest vision data"""
        return self.last_vision_data

    def get_referee_data(self):
        """Get latest referee data"""
        return self.last_referee_data

    def send_command(self, command):
        """Send command to game controller and robots"""
        if not self.running:
            return

        print(f"Executing command: {command}")

        # Stop previous movement thread if exists
        if self.command_thread and self.command_thread.is_alive():
            self.current_command = None
            self.command_thread.join()

        if command == "HALT" or command == "STOP":
            # Stop all robots
            self.current_command = None
            for i in range(3):
                self.blue_robots.send_global_velocity(i, 0, 0, 0)
                self.yellow_robots.send_global_velocity(i, 0, 0, 0)
        else:
            # Start new movement pattern
            self.current_command = command
            self.command_thread = threading.Thread(target=self.execute_movement)
            self.command_thread.daemon = True
            self.command_thread.start()

    def stop(self):
        """Stop all components"""
        print("Stopping all systems...")
        self.running = False
        self.current_command = None  # Stop movement thread

        # Stop existing threads
        if self.command_thread and self.command_thread.is_alive():
            try:
                self.command_thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error stopping command thread: {e}")

    def get_unball_data_vision(self):
        """Get vision data specific to UnBall team's robots"""
        if not self.last_vision_data:
            return None

        # Determine which team color UnBall is using from config
        is_blue = self.config["match"]["team_color"] == "blue"
        robot_data = (
            self.last_vision_data["robotsBlue"]
            if is_blue
            else self.last_vision_data["robotsYellow"]
        )

        # Filter for robots that are actually detected on field
        active_robots = {
            robot_id: {
                "position": {"x": data["x"], "y": data["y"]},
                "orientation": data["theta"],
                "timestamp": data["tCapture"],
                "camera_id": data["cCapture"],
            }
            for robot_id, data in robot_data.items()
            if data["x"] is not None
        }

        return {
            "team_color": "blue" if is_blue else "yellow",
            "robots": active_robots,
            "total_robots": len(active_robots),
            "ball_position": (
                {
                    "x": self.last_vision_data["ball"]["x"],
                    "y": self.last_vision_data["ball"]["y"],
                }
                if self.last_vision_data["ball"]
                else None
            ),
        }

    def get_unball_data_referee(self):
        """Get referee data specific to UnBall team"""
        if not self.last_referee_data:
            return None

        team_color = self.config["match"]["team_color"]
        team_data = self.last_referee_data[team_color]

        return {
            "side": team_color,
            "score": team_data["score"],
            "red_cards": team_data["red_cards"],
            "yellow_cards": team_data["yellow_cards"],
            "timeouts": team_data["timeouts"],
            "timeout_time": team_data["timeout_time"],
            "goalkeeper": team_data["goalkeeper"],
        }


def get_config(config_file=None):
    """Load configuration from file"""
    try:
        if config_file:
            with open(config_file, "r") as f:
                config = json.load(f)
        else:
            with open("config.json", "r") as f:
                config = json.load(f)

        required_fields = [
            ("network", "multicast_ip"),
            ("network", "vision_port"),
            ("network", "referee_ip"),
            ("network", "referee_port"),
            ("match", "team_side"),
            ("match", "team_color"),
        ]

        for section, field in required_fields:
            if section not in config or field not in config[section]:
                raise ValueError(f"Missing required config field: {section}.{field}")

        return config

    except FileNotFoundError:
        raise FileNotFoundError("Config file not found")
    except json.JSONDecodeError:
        raise ValueError("Invalid JSON in config file")


def main():
    try:
        # Create Qt application
        app = QApplication(sys.argv)

        # Create and start game instance
        game = Game()

        # Create and show main window
        window = MainWindow(game)
        window.show()

        # Start game systems
        game.start()

        # Handle Ctrl+C gracefully
        def signal_handler(sig, frame):
            print("\nEncerrando sistemas...")
            game.stop()
            app.quit()

        signal.signal(signal.SIGINT, signal_handler)

        # Keep Qt responsive to Ctrl+C
        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(100)

        sys.exit(app.exec_())

    except Exception as e:
        print(f"Erro fatal: {str(e)}")
        traceback.print_exc()
        if "game" in locals():
            game.stop()


if __name__ == "__main__":
    main()
