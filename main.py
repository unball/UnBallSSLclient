import sys
import warnings
import os
import json
import threading
import time
import math
import signal
import traceback
from typing import Optional, Dict, Any
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt, QTimer

warnings.filterwarnings("ignore", category=DeprecationWarning)
os.environ["GTK_MODULES"] = ""

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Project imports
from PyQt.ssl_client import SSLClientWindow
from GameController.GameController import GameController
from VisionClient.Vision import Vision
from SimulationGrSim.RobotControlClient_threaded import ThreadedRobotControlClient
from RobotBehavior.grsim_client import GrSimController
from RobotBehavior.irl_client import IRLController
from PathPlanning.path_planner import PathPlanner
from RobotBehavior.robot_state_machine import (
    GoalkeeperStateMachine,
    DefenderStateMachine,
    AttackerStateMachine,
)
from RobotBehavior.robot_states import RobotRole

# Global debug flag
GAME_DEBUG = True

# Forcing robot_state_machine debug from here if needed:
import RobotBehavior.robot_state_machine as rsm

rsm.DEBUG_ROBOT_BEHAVIOR = True  # Set to True for development

# Field dimensions by division (central place for this info)
FIELD_DIMENSIONS_BY_DIVISION = {
    "Division A": {
        "width": 12.0,
        "height": 9.0,
        "max_robots": 11,
        "roles": {
            0: RobotRole.GOALKEEPER,
            1: RobotRole.DEFENDER,
            2: RobotRole.DEFENDER,
            3: RobotRole.ATTACKER,
            4: RobotRole.ATTACKER,
        },
    },
    "Division B": {
        "width": 9.0,
        "height": 6.0,
        "max_robots": 6,
        "roles": {
            0: RobotRole.GOALKEEPER,
            1: RobotRole.DEFENDER,
            2: RobotRole.ATTACKER,
            3: RobotRole.ATTACKER,
        },
    },
    "Entry Level": {
        "width": 4.5,
        "height": 3.0,
        "max_robots": 3,
        "roles": {
            0: RobotRole.GOALKEEPER,
            1: RobotRole.DEFENDER,
            2: RobotRole.ATTACKER,
        },
    },
}


class Game:
    def __init__(self) -> None:
        # Load Config
        self.config = get_config()
        if GAME_DEBUG:
            print(f"Game: Config loaded: {json.dumps(self.config, indent=2)}")

        # Debug flags from config
        self.debug = self.config.get(
            "debug_flags",
            {
                "vision": False,
                "referee": False,
                "threads": False,
                "timing": False,
                "path_planning": True,
                "all": False,
            },
        )
        rsm.DEBUG_ROBOT_BEHAVIOR = self.debug.get("robot_behavior", True)

        # Game state
        self.current_command: Optional[str] = "HALT"  # Initial state
        self.team_color_for_current_command: Optional[str] = None
        self.command_thread = None

        if GAME_DEBUG:
            print("Game: Initializing...")

        # Initialize robot control clients using config ports
        self.blue_robots = ThreadedRobotControlClient(
            team_port=self.config["network"].get("blue_control_port", 10301)
        )
        self.yellow_robots = ThreadedRobotControlClient(
            team_port=self.config["network"].get("yellow_control_port", 10302)
        )
        if GAME_DEBUG:
            print("Game: Robot control clients initialized")

        # Input Layer - Initialize BEFORE using them
        self.vision = Vision(self)
        self.referee = GameController(self)
        if GAME_DEBUG:
            print("Game: Vision and referee systems initialized")

        # Field bounds - Initialize AFTER vision is created, with default values first
        self.field_bounds = {"x_min": -2.25, "x_max": 2.25, "y_min": -1.5, "y_max": 1.5}
        # Set proper bounds based on division (will use defaults since vision not started yet)
        self.update_field_bounds_for_division(
            self.config.get("match", {}).get("division", "Entry Level")
        )

        # Initialize controllers with config ports
        self.controllers = {
            "grSim": {
                "blue": GrSimController(
                    team_port=self.config["network"].get("blue_control_port", 10301)
                ),
                "yellow": GrSimController(
                    team_port=self.config["network"].get("yellow_control_port", 10302)
                ),
            },
            "IRL": {"blue": IRLController(), "yellow": IRLController()},
        }

        # Set active controller based on config
        initial_team_color = self.config.get("match", {}).get("team_color", "blue")
        initial_control_mode = self.config.get("match", {}).get("control_mode", "grSim")
        self.active_controller = self.controllers[initial_control_mode][
            initial_team_color
        ]

        if GAME_DEBUG:
            print(
                f"Game: Active controller set to {initial_control_mode} for {initial_team_color}"
            )

        # Path Planner
        self.path_planner = PathPlanner(self)

        # State tracking
        self.running = False
        self._update_thread: Optional[threading.Thread] = None
        self._fps = self.config.get("match", {}).get("fps", 60)
        self.last_vision_data: Optional[Dict] = None
        self.last_referee_data: Optional[Dict] = None

        # Robot state machines
        self.robot_state_machines: Dict[int, Any] = {}

        # UI tracking
        self.window: Optional[SSLClientWindow] = None
        self.visualization_timer = None

    def start(self):
        """Start all components"""
        if self.running:
            if GAME_DEBUG:
                print("Game: Start called, but already running.")
            return

        if GAME_DEBUG:
            print("Game: Starting all systems...")

        # Start robot control
        self.blue_robots.start()
        self.yellow_robots.start()
        if GAME_DEBUG:
            print("Game: Robot control clients started")

        # Start vision and referee
        self.vision.start()
        self.referee.start()
        if GAME_DEBUG:
            print("Game: Vision and referee systems started")

        # Wait for vision geometry to set correct field bounds
        max_wait_geom = 5.0  # seconds
        start_wait_geom = time.time()
        while not self.vision.any_geometry and (
            time.time() - start_wait_geom < max_wait_geom
        ):
            time.sleep(0.1)
        if self.vision.any_geometry:
            self.update_field_bounds_for_division(
                self.config.get("match", {}).get("division", "Entry Level")
            )
            if GAME_DEBUG:
                print(f"Game: Field bounds set from vision: {self.field_bounds}")

        # Start path planner
        self.path_planner.start()
        if GAME_DEBUG:
            print("Game: Path planner started")

        # Start active controller
        self.active_controller.start()
        if GAME_DEBUG:
            print("Game: Active controller started")

        # Initialize robot state machines
        self._initialize_robot_state_machines()

        self.running = True
        self._update_thread = threading.Thread(
            target=self.update_loop, name="GameUpdateLoop"
        )
        self._update_thread.daemon = True
        self._update_thread.start()
        if GAME_DEBUG:
            print("Game: Main update loop started")

    def update_field_bounds_for_division(self, division_name: str):
        """Update field bounds based on division and vision data"""
        # Try to get from vision first (only if vision exists and has geometry)
        if (
            hasattr(self, "vision")
            and self.vision
            and hasattr(self.vision, "any_geometry")
            and self.vision.any_geometry
            and hasattr(self.vision, "raw_geometry")
            and self.vision.raw_geometry.get("fieldLength", 0) > 0
        ):

            field_length = self.vision.raw_geometry["fieldLength"]  # Already in meters
            field_width = self.vision.raw_geometry["fieldWidth"]  # Already in meters
            if GAME_DEBUG:
                print(f"Game: Using vision geometry: L={field_length}, W={field_width}")
        else:
            # Fallback to FIELD_DIMENSIONS_BY_DIVISION
            dims = FIELD_DIMENSIONS_BY_DIVISION.get(
                division_name, FIELD_DIMENSIONS_BY_DIVISION["Entry Level"]
            )
            field_length = dims["width"]
            field_width = dims["height"]
            if GAME_DEBUG:
                print(
                    f"Game: Using default dimensions for {division_name}: L={field_length}, W={field_width}"
                )

        self.field_bounds = {
            "x_min": -field_length / 2.0,
            "x_max": field_length / 2.0,
            "y_min": -field_width / 2.0,
            "y_max": field_width / 2.0,
        }
        if GAME_DEBUG:
            print(f"Game: Field bounds updated to: {self.field_bounds}")

        self.field_bounds = {
            "x_min": -field_length / 2.0,
            "x_max": field_length / 2.0,
            "y_min": -field_width / 2.0,
            "y_max": field_width / 2.0,
        }
        if GAME_DEBUG:
            print(f"Game: Field bounds updated to: {self.field_bounds}")

    def update_field_bounds(self):
        """Update field bounds from vision geometry (legacy method)"""
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

    def check_bounds(self, x, y):
        """Check if position is within field bounds"""
        return (
            self.field_bounds["x_min"] <= x <= self.field_bounds["x_max"]
            and self.field_bounds["y_min"] <= y <= self.field_bounds["y_max"]
        )

    def get_team_for_command(self, command_str: Optional[str]) -> str:
        """Extract team color from referee command string"""
        if not command_str:
            return ""
        if command_str.endswith("_BLUE"):
            return "blue"
        if command_str.endswith("_YELLOW"):
            return "yellow"
        return ""

    def set_game_state(self, state):
        """Process game state commands and update robot behavior accordingly"""
        if GAME_DEBUG:
            print(f"Game: set_game_state received: '{state}'")

        self.current_command = state
        self.team_color_for_current_command = self.get_team_for_command(state)

        # Parse team-specific commands
        team_commands = [
            "FREE_KICK",
            "KICK_OFF",
            "PENALTY",
            "GOAL_KICK",
            "CORNER_KICK",
            "BALL_PLACEMENT",
        ]

        team_color = None
        base_command = state

        # Check if this is a team-specific command
        for cmd in team_commands:
            if state.startswith(cmd + "_"):
                parts = state.split("_", 1)
                if len(parts) > 1:
                    base_command = parts[0]
                    team_color = parts[1].lower()
                    if GAME_DEBUG:
                        print(
                            f"Game: Extracted base command: {base_command}, team: {team_color}"
                        )
                    break

        # Update referee data with this command
        if hasattr(self, "last_referee_data") and self.last_referee_data:
            self.last_referee_data["command"] = base_command
            self.last_referee_data["team_color_for_command"] = team_color
            self.last_referee_data["can_play"] = base_command in [
                "FORCE_START",
                "NORMAL_START",
            ]

        # Take immediate action based on command
        if state == "HALT":
            if GAME_DEBUG:
                print("Game: HALT command - stopping all robots")
            num_robots = FIELD_DIMENSIONS_BY_DIVISION[self.config["match"]["division"]][
                "max_robots"
            ]
            for i in range(num_robots):
                self.blue_robots.send_global_velocity(i, 0, 0, 0)
                self.yellow_robots.send_global_velocity(i, 0, 0, 0)

        elif state == "STOP":
            if GAME_DEBUG:
                print("Game: STOP command - robots must stay away from ball")

        elif state.startswith("FORCE_START") or state.startswith("NORMAL_START"):
            if GAME_DEBUG:
                print(f"Game: {state} - game starting, robots can move freely")

        # Force immediate update of all robot state machines
        vision_data = self.get_vision_data()
        if vision_data and hasattr(self, "robot_state_machines"):
            if GAME_DEBUG:
                print("Game: Updating all robot state machines with new command")
            for robot_id, state_machine in self.robot_state_machines.items():
                if hasattr(state_machine, "update"):
                    state_machine.update(vision_data)

    def update_loop(self):
        """Main game logic update loop"""
        if GAME_DEBUG:
            print("Game: Update loop starting")
        loop_counter = 0

        while self.running:
            try:
                loop_start_time = time.perf_counter()

                # 1. Get Vision Data
                if self.vision.new_data or loop_counter == 0:
                    self.last_vision_data = self.vision.get_last_frame()
                    self.vision.new_data = False
                    if self.vision.any_geometry:
                        self.update_field_bounds_for_division(
                            self.config["match"]["division"]
                        )

                # 2. Get Referee Data
                current_ref_command = self.referee.raw_referee.get("command")
                current_ref_stage = self.referee.raw_referee.get("stage")
                team_for_gc_command = self.get_team_for_command(current_ref_command)

                self.last_referee_data = {
                    "command": current_ref_command,
                    "stage": current_ref_stage,
                    "team_color_for_command": team_for_gc_command,
                    "can_play": not (current_ref_command in ["HALT", "STOP"]),
                    "designated_position": (
                        self.referee.raw_referee.get("position")
                        if self.referee.raw_referee.get("meta", {}).get(
                            "has_designated_position"
                        )
                        else None
                    ),
                    "blue_team_details": self.referee.raw_referee.get("blue"),
                    "yellow_team_details": self.referee.raw_referee.get("yellow"),
                }

                # Override with UI/Test command if active
                if self.current_command is not None:
                    self.last_referee_data["command"] = self.current_command
                    self.last_referee_data["team_color_for_command"] = (
                        self.team_color_for_current_command
                    )
                    self.last_referee_data["can_play"] = not (
                        self.current_command in ["HALT", "STOP"]
                    )

                # 3. Update Robot State Machines
                if self.last_vision_data and self.robot_state_machines:
                    # Only update if not in direct control mode
                    if not self.current_command or self.current_command in [
                        "FORCE_START",
                        "NORMAL_START",
                    ]:
                        for (
                            robot_id,
                            state_machine,
                        ) in self.robot_state_machines.items():
                            state_machine.update(self.last_vision_data)

                loop_processing_time = time.perf_counter() - loop_start_time
                sleep_time = (1.0 / self._fps) - loop_processing_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

                loop_counter += 1

            except Exception as e:
                print(f"Game ERROR in update_loop: {e}")
                traceback.print_exc()
                time.sleep(0.1)

        if GAME_DEBUG:
            print("Game: Update loop finished")

    def _initialize_robot_state_machines(self):
        """Initialize state machines for different robot roles based on division"""
        self.robot_state_machines.clear()
        team_color = self.config.get("match", {}).get("team_color", "blue")
        current_division_name = self.config.get("match", {}).get(
            "division", "Entry Level"
        )

        division_info = FIELD_DIMENSIONS_BY_DIVISION.get(
            current_division_name, FIELD_DIMENSIONS_BY_DIVISION["Entry Level"]
        )
        num_robots = division_info["max_robots"]
        role_assignments = division_info["roles"]

        if GAME_DEBUG:
            print(
                f"Game: Initializing {num_robots} SMs for '{current_division_name}', team '{team_color}'"
            )
            print(f"Game: Role assignments: {role_assignments}")

        for robot_id_int in range(num_robots):
            role_to_assign = role_assignments.get(robot_id_int, RobotRole.ATTACKER)

            sm_instance = None
            if role_to_assign == RobotRole.GOALKEEPER:
                sm_instance = GoalkeeperStateMachine(robot_id_int, team_color, self)
            elif role_to_assign == RobotRole.DEFENDER:
                sm_instance = DefenderStateMachine(robot_id_int, team_color, self)
            elif role_to_assign == RobotRole.ATTACKER:
                sm_instance = AttackerStateMachine(robot_id_int, team_color, self)

            if sm_instance:
                self.robot_state_machines[robot_id_int] = sm_instance
                if GAME_DEBUG:
                    print(
                        f"Game: Initialized Robot {robot_id_int} as {sm_instance.role.value}"
                    )

    def switch_team_color(self, new_team_color: str):
        """Switch team color and reinitialize robots"""
        if new_team_color not in ["blue", "yellow"]:
            if GAME_DEBUG:
                print(f"Game: Invalid team color '{new_team_color}'")
            return

        old_color = self.config["match"]["team_color"]
        if old_color == new_team_color:
            if GAME_DEBUG:
                print(f"Game: Team color already {new_team_color}")
            return

        if GAME_DEBUG:
            print(f"Game: Switching team color from {old_color} to {new_team_color}")

        # Stop robots before switching
        if self.active_controller:
            num_robots = FIELD_DIMENSIONS_BY_DIVISION[self.config["match"]["division"]][
                "max_robots"
            ]
            for i in range(num_robots):
                self.active_controller.send_global_velocity(i, 0, 0, 0)
            time.sleep(0.1)

        # Update configuration
        self.config["match"]["team_color"] = new_team_color

        # Switch controller
        current_mode = self.config.get("match", {}).get("control_mode", "grSim")
        if self.active_controller:
            self.active_controller.stop()
        self.active_controller = self.controllers[current_mode][new_team_color]
        self.active_controller.start()

        # Reinitialize robot state machines
        self._initialize_robot_state_machines()

        if GAME_DEBUG:
            print(f"Game: Team switch complete to {new_team_color}")

    def set_control_mode(self, mode: str, team_color: str):
        """Switch between grSim and IRL control"""
        if mode not in self.controllers:
            if GAME_DEBUG:
                print(f"Game: Invalid control mode '{mode}'")
            return

        current_team_color = self.config["match"]["team_color"]

        if self.active_controller == self.controllers[mode][current_team_color]:
            if GAME_DEBUG:
                print(f"Game: Already in {mode} for team {current_team_color}")
            return

        if GAME_DEBUG:
            print(
                f"Game: Switching control mode to {mode} for team {current_team_color}"
            )

        if self.active_controller:
            self.active_controller.stop()

        self.config["match"]["control_mode"] = mode
        self.active_controller = self.controllers[mode][current_team_color]
        self.active_controller.start()

    def get_vision_data(self):
        """Get latest vision data"""
        return self.last_vision_data

    def get_referee_data(self):
        """Get latest referee data"""
        return self.last_referee_data

    def send_command(self, command):
        """Legacy method - use set_game_state instead"""
        self.set_game_state(command)

    def stop(self):
        """Stop all components"""
        if not self.running:
            return

        if GAME_DEBUG:
            print("Game: Stopping all systems...")
        self.running = False
        self.current_command = None

        # Stop path planner first
        if hasattr(self, "path_planner"):
            self.path_planner.stop()

        # Stop robots
        if self.active_controller:
            num_robots = FIELD_DIMENSIONS_BY_DIVISION[self.config["match"]["division"]][
                "max_robots"
            ]
            for i in range(num_robots):
                self.active_controller.send_global_velocity(i, 0, 0, 0)

        # Stop update thread
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=1.0)

        # Stop components
        components = ["vision", "referee"]
        for comp in components:
            if hasattr(self, comp):
                try:
                    getattr(self, comp).stop()
                except Exception as e:
                    if GAME_DEBUG:
                        print(f"Error stopping {comp}: {e}")

        # Stop all controllers
        for mode_key in self.controllers:
            for color_key in self.controllers[mode_key]:
                try:
                    self.controllers[mode_key][color_key].stop()
                except Exception as e:
                    if GAME_DEBUG:
                        print(f"Error stopping controller {mode_key}/{color_key}: {e}")

        # Stop robot control clients
        if hasattr(self, "blue_robots"):
            self.blue_robots.stop()
        if hasattr(self, "yellow_robots"):
            self.yellow_robots.stop()

        if GAME_DEBUG:
            print("Game: All systems stopped")

    def update_network_settings(self, network_config):
        """Update network settings for vision and referee"""
        try:
            if GAME_DEBUG:
                print(f"Game: Updating network settings: {network_config}")

            # Stop existing vision and referee threads
            if self.vision:
                self.vision.stop()
            if self.referee:
                self.referee.stop()

            # Update configuration
            self.config["network"].update(network_config)

            # Reinitialize vision and referee with new settings
            self.vision = Vision(self)
            self.referee = GameController(self)

            # Restart vision and referee
            self.vision.start()
            self.referee.start()

            if GAME_DEBUG:
                print("Game: Network settings updated successfully")
            return True

        except Exception as e:
            if GAME_DEBUG:
                print(f"Game ERROR updating network settings: {e}")
            traceback.print_exc()
            return False

    # Legacy methods for UI compatibility
    def get_unball_data_vision(self):
        """Get vision data specific to UnBall team's robots"""
        if not self.last_vision_data:
            return None

        is_blue = self.config["match"]["team_color"] == "blue"
        robot_data = (
            self.last_vision_data["robotsBlue"]
            if is_blue
            else self.last_vision_data["robotsYellow"]
        )

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
        team_data = self.last_referee_data.get(team_color, {})

        return {
            "side": team_color,
            "score": team_data.get("score", 0),
            "red_cards": team_data.get("red_cards", 0),
            "yellow_cards": team_data.get("yellow_cards", 0),
            "timeouts": team_data.get("timeouts", 0),
            "timeout_time": team_data.get("timeout_time", 0),
            "goalkeeper": team_data.get("goalkeeper", 0),
        }


def get_config(config_file=None) -> dict:
    """Load configuration from file with proper defaults"""
    config_path = config_file or os.path.join(SCRIPT_DIR, "config.json")

    # Create default configuration
    default_cfg_data = {
        "network": {
            "multicast_ip": "224.5.23.2",
            "vision_port": 10020,
            "referee_ip": "224.5.23.1",
            "referee_port": 10003,
            "blue_control_port": 10301,
            "yellow_control_port": 10302,
            "simulation_control_port": 10300,
        },
        "match": {
            "team_1_name": "UnBall",
            "team_2_name": "Opponent",
            "event_name": "RoboCup SSL",
            "team_side": "left",
            "team_color": "blue",
            "division": "Entry Level",
            "control_mode": "grSim",
            "fps": 60,
            "time_logging": False,
        },
        "debug_flags": {
            "vision": False,
            "referee": False,
            "threads": False,
            "timing": False,
            "path_planning": True,
            "robot_behavior": True,
            "all": False,
        },
    }

    if os.path.exists(config_path):
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            # Merge with defaults
            for section, section_defaults in default_cfg_data.items():
                if section not in config:
                    config[section] = section_defaults
                else:
                    for key, value in section_defaults.items():
                        config[section].setdefault(key, value)
            return config
        except json.JSONDecodeError:
            print(f"WARNING: Invalid JSON in {config_path}. Using defaults.")
        except Exception as e:
            print(f"WARNING: Error loading {config_path}: {e}. Using defaults.")

    # Save default config
    try:
        with open(config_path, "w") as f:
            json.dump(default_cfg_data, f, indent=4)
        print(f"Created default config at {config_path}")
    except Exception as e:
        print(f"WARNING: Could not save config: {e}")

    return default_cfg_data


# Global game instance for signal handler
_game_instance_for_signal_handler: Optional[Game] = None


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n! Received interrupt signal. Shutting down systems gracefully...")
    rsm.DEBUG_ROBOT_BEHAVIOR = False

    if _game_instance_for_signal_handler is not None:
        _game_instance_for_signal_handler.stop()

    QApplication.quit()
    time.sleep(0.5)
    print("Shutdown complete. Exiting.")
    sys.exit(0)


def main():
    global _game_instance_for_signal_handler

    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        app = QApplication(sys.argv)
        if GAME_DEBUG:
            print("Main: QApplication created")

        game = Game()
        _game_instance_for_signal_handler = game
        if GAME_DEBUG:
            print("Main: Game instance created")

        window = SSLClientWindow(game=game)
        game.window = window
        window.show()
        if GAME_DEBUG:
            print("Main: SSLClientWindow shown")

        game.start()
        if GAME_DEBUG:
            print("Main: Game systems started")

        # Keep Qt responsive to Ctrl+C
        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(100)

        sys.exit(app.exec_())

    except Exception as e:
        print(f"Main FATAL ERROR: {e}")
        traceback.print_exc()
        if (
            "_game_instance_for_signal_handler" in globals()
            and _game_instance_for_signal_handler
        ):
            _game_instance_for_signal_handler.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()
