import sys
import warnings
import os
import json
import threading
import time
import math
import signal
import traceback
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

        # Initialize controllers
        self.controllers = {
            "grSim": {
                "blue": GrSimController(team_port=10301),
                "yellow": GrSimController(team_port=10302),
            },
            "IRL": {"blue": IRLController(), "yellow": IRLController()},
        }
        self.active_controller = self.controllers["grSim"]["blue"]

        # Path Planner
        self.path_planner = PathPlanner(self)

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

        # Start path planner
        self.path_planner.start()
        print("Path planner started")

        # Initialize robot state machines
        self._initialize_robot_state_machines()

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

    def set_game_state(self, state):
        """
        Process game state commands and update robot behavior accordingly

        Args:
            state: Game state command string
        """
        print(f"Setting game state to: {state}")
        self.current_command = state

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
                    print(f"Extracted base command: {base_command}, team: {team_color}")
                    break

        # Update referee data with this command to ensure robot state machines respect it
        if hasattr(self, "last_referee_data") and self.last_referee_data:
            self.last_referee_data["command"] = base_command
            self.last_referee_data["team"] = team_color
            self.last_referee_data["can_play"] = base_command in [
                "FORCE_START",
                "NORMAL_START",
            ]
            print(
                f"Updated referee data: command={base_command}, team={team_color}, can_play={self.last_referee_data['can_play']}"
            )

        # Take immediate action based on command
        if state == "HALT":
            # Stop all robots immediately
            for i in range(3):  # Assuming 3 robots per team
                self.blue_robots.send_global_velocity(i, 0, 0, 0)
                self.yellow_robots.send_global_velocity(i, 0, 0, 0)
            print("HALT command: All robots stopped")

        elif state == "STOP":
            # Robots can move but must stay away from ball
            print("STOP command: Robots must stay away from ball")
            # Let state machines handle this in their next update

        elif state.startswith("FORCE_START") or state.startswith("NORMAL_START"):
            print(f"{state}: Game starting, robots can move freely")
            # Game is now in play, robots will follow their state machines

        # Special handling for team-specific commands
        elif team_color:
            print(f"Team-specific command {base_command} for team {team_color}")
            # Force update of robot states for the active team
            if team_color == self.config["match"]["team_color"]:
                print("Command applies to our team - preparing for action")
            else:
                print(
                    "Command applies to opponent team - preparing defensive positions"
                )

        # Force immediate update of all robot state machines
        vision_data = self.get_vision_data()
        if vision_data and hasattr(self, "robot_state_machines"):
            print("Updating all robot state machines with new command")
            for robot_id, state_machine in self.robot_state_machines.items():
                # Pass the current command to the state machine update
                if hasattr(state_machine, "update"):
                    state_machine.update(vision_data)
                    print(
                        f"Robot {robot_id} state updated to: {state_machine.current_state.value if hasattr(state_machine, 'current_state') else 'Unknown'}"
                    )

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

    def switch_team_color(self, new_team_color):
        """
        Switch the team color and reinitialize robots with proper state

        Args:
            new_team_color: New team color ("blue" or "yellow")
        """
        if new_team_color not in ["blue", "yellow"]:
            print(f"Invalid team color: {new_team_color}")
            return

        old_color = self.config["match"]["team_color"]
        if new_team_color == old_color:
            print(f"Team color already set to {new_team_color}")
            return

        print(f"Switching team color from {old_color} to {new_team_color}")

        # Stop all robots before switching
        for i in range(3):
            self.blue_robots.send_global_velocity(i, 0, 0, 0)
            self.yellow_robots.send_global_velocity(i, 0, 0, 0)

        # Update configuration
        self.config["match"]["team_color"] = new_team_color

        # Get current game state
        current_game_state = "HALT"  # Default to HALT when switching teams
        if hasattr(self, "current_command") and self.current_command:
            current_game_state = self.current_command

        # Get current controller mode
        current_mode = "grSim"  # Default
        for mode in self.controllers:
            for color in self.controllers[mode]:
                if self.active_controller == self.controllers[mode][color]:
                    current_mode = mode
                    break

        # Switch to appropriate controller
        self.set_control_mode(current_mode, new_team_color)

        # Reinitialize robot state machines with new team color
        self._initialize_robot_state_machines()

        # Immediately set robots to halt state
        self.set_game_state(current_game_state)

        print(
            f"Team color switched to {new_team_color}, robots initialized in {current_game_state} state"
        )

    def set_control_mode(self, mode: str, team_color: str):
        """
        Switch between grSim and IRL control and update team color

        Args:
            mode: Control mode ('grSim' or 'IRL')
            team_color: Team color ('blue' or 'yellow')
        """
        if mode not in self.controllers or team_color not in self.controllers[mode]:
            print(f"Invalid mode '{mode}' or team color '{team_color}'")
            return

        # Update the team color in configuration
        if team_color != self.config["match"]["team_color"]:
            print(
                f"Changing team color from {self.config['match']['team_color']} to {team_color}"
            )
            self.config["match"]["team_color"] = team_color

            # Reinitialize robot state machines with new team color
            if hasattr(self, "robot_state_machines"):
                self._initialize_robot_state_machines()
                print("Reinitialized robot state machines with new team color")

        # Only change the controller if it's different
        if self.active_controller != self.controllers[mode][team_color]:
            # Stop the old controller
            if self.active_controller:
                self.active_controller.stop()

            # Set and start the new controller
            self.active_controller = self.controllers[mode][team_color]
            self.active_controller.start()
            print(f"Switched to {mode} controller for {team_color} team")
        else:
            print(f"Already using {mode} controller for {team_color} team")

    def handle_sim_state_change(self, state):
        if not self.game:
            return
        team_color = (
            "yellow" if self.team_select.currentText() == "Time Amarelo" else "blue"
        )
        self.game.set_control_mode(state, team_color)

    def handle_team_selection(self, team):
        """Handle team selection changes"""
        if self.game:
            is_yellow = team == "Time Amarelo"
            team_color = "yellow" if is_yellow else "blue"
            print(f"Selected team: {team_color}")

            # Update game configuration
            self.game.config["match"]["team_color"] = team_color

            # Get current control mode
            mode = self.state_game.currentText()

            # Switch to the appropriate controller
            self.game.set_control_mode(mode, team_color)

            # We need to reinitialize the robot state machines with the new team color
            # First stop any existing state machines
            for robot_id in self.game.robot_state_machines:
                self.game.robot_state_machines[robot_id].current_state = RobotState.IDLE

            # Reinitialize state machines with new team color
            self.game._initialize_robot_state_machines()

            # Update UI to reflect team change
            self.update_team_display(team_color)

            print(f"Successfully switched to {team_color} team")

    def update_team_display(self, team_color):
        """Update the UI to reflect the selected team"""
        # This function updates any UI elements that need to change when the team changes
        # For example, changing colors, labels, etc.

        # You could add code here to update team labels, colors, etc.
        self.lblCurrentTeam.setText(
            f"Current Team: {'Yellow' if team_color == 'yellow' else 'Blue'}"
        )

        # Force update of the field visualization if it exists
        if hasattr(self, "field_viz") and self.field_viz:
            self.field_viz.update()

    def test_movement(self):
        """Test random movements for robots"""
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
                    self.update_field_bounds()

                    # Update robot state machines if not in direct control mode
                    if not self.current_command:
                        for (
                            robot_id,
                            state_machine,
                        ) in self.robot_state_machines.items():
                            state_machine.update(self.last_vision_data)

                # Update referee data independently
                if self.referee.new_data or self.referee.any_referee:
                    self.last_referee_data = self.referee.get_state()

                # Sleep for frame rate
                time.sleep(1.0 / self._fps)

            except Exception as e:
                print(f"Error in update loop: {str(e)}")

    def stop(self):
        """Stop all components"""
        if not self.running:
            return

        print("Stopping all systems...")
        self.running = False

        # Stop threads
        if self.command_thread and self.command_thread.is_alive():
            self.current_command = None
            self.command_thread.join(timeout=1.0)

        # Stop components
        components = [
            "blue_robots",
            "yellow_robots",
            "vision",
            "referee",
            "path_planner",
        ]
        for comp in components:
            if hasattr(self, comp):
                try:
                    getattr(self, comp).stop()
                except Exception as e:
                    print(f"Error stopping {comp}: {e}")

        # Stop visualization
        if self.visualization_timer:
            self.visualization_timer.stop()

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
            self.command_thread.join(timeout=1.0)

        if command == "HALT" or command == "STOP":
            # Stop all robots and disable state machines temporarily
            self.current_command = command
            for i in range(3):
                self.blue_robots.send_global_velocity(i, 0, 0, 0)
                self.yellow_robots.send_global_velocity(i, 0, 0, 0)
        elif command == "FORCE_START":
            # Enable state machines - they'll take over control
            self.current_command = None
        elif command == "NORMAL_START":
            # Enable state machines with normal start behavior
            self.current_command = None
        elif command == "KICK-OFF":
            # Reset robots to kick-off positions
            # This could trigger a special state in the state machines
            self.current_command = command
            # Additional kick-off logic
        elif command in ["FREE-KICK", "PENALTY", "GOAL_KICK", "CORNER_KICK"]:
            # Set up for special plays
            self.current_command = command
            # Additional special play logic
        else:
            # Other commands
            self.current_command = command

        # For testing: Force update of state machines immediately
        if self.current_command is None and self.last_vision_data:
            for robot_id, state_machine in self.robot_state_machines.items():
                state_machine.update(self.last_vision_data)

    def stop(self):
        """Stop all components"""
        if not self.running:
            return

        print("Stopping all systems...")
        self.running = False
        self.current_command = None  # Stop movement thread

        # Signal path planner to stop before other components
        if hasattr(self, "path_planner"):
            try:
                self.path_planner.running = False
                time.sleep(0.1)  # Brief pause to let threads recognize stop signal
            except Exception as e:
                print(f"Error stopping path planner: {e}")

        # Stop existing command thread
        if (
            hasattr(self, "command_thread")
            and self.command_thread
            and self.command_thread.is_alive()
        ):
            try:
                self.command_thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error stopping command thread: {e}")

        # Stop components
        components = [
            "blue_robots",
            "yellow_robots",
            "vision",
            "referee",
            "path_planner",
        ]
        for comp in components:
            if hasattr(self, comp):
                try:
                    getattr(self, comp).stop()
                except Exception as e:
                    print(f"Error stopping {comp}: {e}")

        # Stop visualization
        if hasattr(self, "visualization_timer") and self.visualization_timer:
            try:
                self.visualization_timer.stop()
            except Exception as e:
                print(f"Error stopping visualization timer: {e}")

        # Stop update thread
        if (
            hasattr(self, "_update_thread")
            and self._update_thread
            and self._update_thread.is_alive()
        ):
            try:
                self._update_thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error joining update thread: {e}")

        # Final cleanup
        try:
            # Reset any global state that might be needed
            if hasattr(self, "controllers"):
                for mode in self.controllers:
                    for color in self.controllers[mode]:
                        try:
                            self.controllers[mode][color].stop()
                        except Exception as e:
                            print(f"Error stopping controller {mode}/{color}: {e}")
        except Exception as e:
            print(f"Error in final cleanup: {e}")

        print("All systems stopped")

    def update_network_settings(self, network_config):
        """
        Update network settings for vision and referee

        :param network_config: Dictionary containing network configuration
        """
        try:
            # Stop existing vision and referee threads
            self.vision.stop()
            self.referee.stop()

            # Update configuration
            self.config["network"].update(network_config)

            # Reinitialize vision and referee with new settings
            self.vision = Vision(
                self,
                multicast_ip=network_config.get(
                    "multicast_ip", self.config["network"]["multicast_ip"]
                ),
                port=network_config.get(
                    "vision_port", self.config["network"]["vision_port"]
                ),
            )
            self.referee = GameController(
                self,
                referee_ip=network_config.get(
                    "referee_ip", self.config["network"]["referee_ip"]
                ),
                referee_port=network_config.get(
                    "referee_port", self.config["network"]["referee_port"]
                ),
            )

            # Restart vision and referee
            self.vision.start()
            self.referee.start()

            print("Network settings updated successfully")
            return True

        except Exception as e:
            print(f"Error updating network settings: {e}")
            return False

    def _initialize_robot_state_machines(self):
        """Initialize state machines for different robot roles"""
        self.robot_state_machines = {}
        team_color = self.config["match"]["team_color"]

        # Initialize goalkeeper (robot ID 0)
        self.robot_state_machines[0] = GoalkeeperStateMachine(
            robot_id=0, team_color=team_color, game=self
        )

        # Initialize defender (robot ID 1)
        self.robot_state_machines[1] = DefenderStateMachine(
            robot_id=1, team_color=team_color, game=self
        )

        # Initialize attacker (robot ID 2)
        self.robot_state_machines[2] = AttackerStateMachine(
            robot_id=2, team_color=team_color, game=self
        )

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
        # Try to load from specified or default config file
        config_path = config_file or "config.json"

        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = json.load(f)
        else:
            # Create a basic config file if it doesn't exist
            config = {
                "default_network": {
                    "multicast_ip": "224.5.23.2",
                    "vision_port": 10020,
                    "referee_ip": "224.5.23.1",
                    "referee_port": 10003,
                    "yellow_port": 10004,
                    "blue_port": 10005,
                },
                "network": {},
                "match": {
                    "team_1": "UnBall",
                    "team_2": "Unknown",
                    "event": "Unknown",
                    "team_side": "left",
                    "team_color": "blue",
                    "time_logging": False,
                },
            }

            # Save the default configuration
            with open(config_path, "w") as f:
                json.dump(config, f, indent=4)

        # Merge default_network into network if network is empty
        if not config.get("network", {}):
            config["network"] = config.get("default_network", {}).copy()

        # Validate required fields
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
        window = SSLClientWindow(game)
        window.show()

        # Start game systems
        game.start()

        def signal_handler(sig, frame):
            print("\nShutting down systems gracefully...")

            # Disable DEBUG_ROBOT_BEHAVIOR to prevent more debug printing
            import RobotBehavior.robot_state_machine as rsm

            rsm.DEBUG_ROBOT_BEHAVIOR = False

            # Stop all printing in threads
            sys.stdout.flush()

            # Stop the game if it exists
            if "game" in globals():
                game.stop()

            # Give a short time for threads to clean up
            time.sleep(0.5)
            print("Shutdown complete")
            sys.exit(0)

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
