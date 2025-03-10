import os
import json
import math
import copy
import sys

from PyQt5.QtCore import Qt, QTimer, QCoreApplication
from PyQt5.QtWidgets import (
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QDialog,
    QPushButton,
    QLineEdit,
    QMessageBox,
    QAction,
    QFrame,
    QCheckBox,
    QComboBox,
)
from PyQt5 import uic
from PyQt.field_visualization import FieldVisualization
from PyQt.main_ui import Ui_MainWindow
from .settings_ui import Ui_Dialog
from .field_visualization import FieldVisualization
from .debug_window import DebugWindow, DebugStreamRedirector

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(ROOT_DIR, "config.json")
DEFAULT_CONFIG_PATH = os.path.join(ROOT_DIR, "default_config.json")


class SSLClientWindow(QMainWindow):
    def __init__(self, game=None):
        super().__init__()
        self.game = game  # Store game reference
        ui_file = os.path.join(SCRIPT_DIR, "main.ui")
        uic.loadUi(ui_file, self)

        # Set up stdout redirection
        self.stdout_redirector = DebugStreamRedirector(sys.stdout)
        sys.stdout = self.stdout_redirector

        # Connect signal to log capture
        self.stdout_redirector.text_written.connect(self.capture_log)

        # Store logs
        self.path_planning_logs = []
        self.vision_logs = []
        self.sim_logs = []

        # Reference to debug window
        self.debug_window = None

        # Set up field visualization
        self.field_widget = FieldVisualization()
        self.field_frame = self.findChild(QFrame, "field_frame")

        # Create layout for field
        layout = QVBoxLayout(self.field_frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.field_widget)

        # Connect UI elements
        self.setup_menu_actions()
        self.setup_control_buttons()
        self.setup_checkboxes()
        self.setup_division_selector()

        # Connect A* visualization checkbox
        self.aestrela_checkbox = self.findChild(QCheckBox, "aestrela_checkbox")
        if self.aestrela_checkbox:
            self.aestrela_checkbox.toggled.connect(self.update_visualization_settings)

        # Connect the debug action in menu
        self.actionPathing_Debug.triggered.connect(self.show_debug_window)

        # Setup timer for regular updates if game is provided
        if self.game:
            self.update_timer = QTimer()
            self.update_timer.timeout.connect(self.update_display)
            self.update_timer.start(16)  # ~60 FPS

        # Store original stdout for restoration
        self.original_stdout = sys.stdout

        # Set up stdout redirection
        try:
            self.stdout_redirector = DebugStreamRedirector(
                self.original_stdout
            )
            sys.stdout = self.stdout_redirector
            self.stdout_redirector.text_written.connect(self.capture_log)
        except Exception as e:
            print(f"Warning: Could not set up stdout redirection: {e}")
            self.stdout_redirector = None

    # Add this method to your class
    def update_visualization_settings(self):
        """Update visualization settings based on checkboxes"""
        if hasattr(self, "field_widget"):
            self.field_widget.set_show_paths(self.aestrela_checkbox.isChecked())

    def update_display(self):
        """Update field display with latest game data"""
        if not self.game:
            return

        # Get latest vision data
        vision_data = self.game.get_vision_data()

        if not vision_data:
            self.field_widget.clear_safely()
            return

        # Detailed data validation
        def is_valid_coordinate(coord):
            return coord is not None and not math.isnan(coord)

        # Ball validation and update
        ball = vision_data.get("ball", {})
        if is_valid_coordinate(ball.get("x")) and is_valid_coordinate(ball.get("y")):
            self.field_widget.update_ball(ball["x"], ball["y"])

        # Blue robots validation and update
        blue_robots = vision_data.get("robotsBlue", {})
        for robot_id, robot in blue_robots.items():
            if (
                is_valid_coordinate(robot.get("x"))
                and is_valid_coordinate(robot.get("y"))
                and is_valid_coordinate(robot.get("theta"))
            ):
                self.field_widget.update_robot(
                    robot["x"], robot["y"], robot["theta"], Qt.blue, int(robot_id)
                )

        # Yellow robots validation and update
        yellow_robots = vision_data.get("robotsYellow", {})
        for robot_id, robot in yellow_robots.items():
            if (
                is_valid_coordinate(robot.get("x"))
                and is_valid_coordinate(robot.get("y"))
                and is_valid_coordinate(robot.get("theta"))
            ):
                self.field_widget.update_robot(
                    robot["x"], robot["y"], robot["theta"], Qt.yellow, int(robot_id)
                )

        # Update path visualization if A* checkbox is checked
        if hasattr(self, "astar_checkbox") and self.astar_checkbox.isChecked():
            if hasattr(self.game, "path_planner"):
                # Get paths for each robot
                for robot_id in range(3):
                    path = self.game.path_planner.get_path(int(robot_id))
                    if path:
                        self.field_widget.update_path(int(robot_id), path)

        # Update status labels if available
        if hasattr(self, "robot_status_labels") and hasattr(
            self.game, "robot_state_machines"
        ):
            for robot_id, state_machine in self.game.robot_state_machines.items():
                if robot_id in self.robot_status_labels:
                    role = state_machine.role.value if state_machine.role else "Unknown"
                    state = (
                        state_machine.current_state.value
                        if state_machine.current_state
                        else "IDLE"
                    )
                    self.robot_status_labels[robot_id].setText(
                        f"Robot {robot_id} ({role}): {state}"
                    )

        # Update robot status indicators
        if hasattr(self.game, "robot_state_machines"):
            for robot_id, state_machine in self.game.robot_state_machines.items():
                status_label = getattr(self, f"robot{robot_id}_status", None)
                if status_label:
                    role = state_machine.role.value if state_machine.role else "Unknown"
                    state = (
                        state_machine.current_state.value
                        if state_machine.current_state
                        else "IDLE"
                    )
                    status_label.setText(f"Robot {robot_id} ({role}): {state}")

    def closeEvent(self, event):
        """Handle window close event"""
        # Restore original stdout
        if hasattr(self, "stdout_redirector") and self.stdout_redirector:
            self.stdout_redirector.deactivate()
            if hasattr(self, "original_stdout"):
                sys.stdout = self.original_stdout

        # Close debug window if it exists
        if self.debug_window and self.debug_window.isVisible():
            self.debug_window.close()

        # Accept the close event
        event.accept()

    def setup_menu_actions(self):
        self.actionAbrir = self.findChild(QAction, "actionAbrir")
        if self.actionAbrir:
            self.actionAbrir.triggered.connect(self.open_settings)

    def setup_control_buttons(self):
        control_map = {
            "halt_button": ("HALT", "HALT"),
            "stop_button": ("STOP", "STOP"),
            "force_start_button": ("FORCE_START", "FORCE START"),
            "normal_start_button": ("NORMAL_START", "NORMAL START"),
            "timeout_button": ("TIMEOUT", "ASK TIMEOUT"),
            "substitution_button": ("SUBSTITUTION", "SUBSTITUTION"),
            "free_kick_button": ("FREE_KICK", "FREE-KICK POSITION"),
            "kick_off_button": ("KICK_OFF", "KICK-OFF"),
            "penalty_button": ("PENALTY", "PENALTY"),
            "goal_kick_button": ("GOAL_KICK", "GOAL KICK"),
            "corner_kick_button": ("CORNER_KICK", "CORNER KICK"),
            "center_circle_button": ("CENTER_CIRCLE", "CENTER CIRCLE"),
            "ball_placement_button": ("BALL_PLACEMENT", "BALL PLACEMENT"),
            "penalty_mark_button": ("PENALTY_MARK", "PENALTY MARK"),
        }

        for btn_name, (action, text) in control_map.items():
            button = self.findChild(QPushButton, btn_name)
            if button:
                if btn_name == "force_start_button":
                    # Special handling for force start button
                    button.clicked.connect(self.handle_force_start)
                else:
                    button.clicked.connect(
                        lambda checked, a=action: self.handle_control_action(a)
                    )

        # Setup simulation state selector
        self.state_game = self.findChild(QComboBox, "state_game")
        if self.state_game:
            self.state_game.currentTextChanged.connect(self.handle_sim_state_change)

        # Setup robot selector
        self.box_all_robots = self.findChild(QComboBox, "box_all_robots")
        if self.box_all_robots:
            self.box_all_robots.clear()
            for i in range(3):  # SSL-EL uses 3 robots
                self.box_all_robots.addItem(f"Robot {i}")

    def handle_force_start(self):
        """Handle FORCE START button - start game and initialize behavior"""
        if self.game:
            # Start the game if not already started
            self.start_game()

            # Then send the FORCE_START command
            self.handle_control_action("FORCE_START")

    def start_game(self):
        """Initialize and start the game"""
        if not self.game or not hasattr(self.game, "start"):
            return

        # Check if game is already running
        if hasattr(self.game, "running") and self.game.running:
            # Game is already running, just initialize robot state machines
            if not hasattr(self.game, "robot_state_machines"):
                self.game._initialize_robot_state_machines()
        else:
            # First start the game systems if not running
            self.game.start()

            # Then initialize robot state machines
            if not hasattr(self.game, "robot_state_machines"):
                self.game._initialize_robot_state_machines()

        # Update the UI once to show initial states
        self.update_display()

    def handle_sim_state_change(self, state):
        """Handle simulation state changes between grSim and real robots"""
        print(f"Switching to {state} mode")
        # Implement simulation state switching logic

    def setup_checkboxes(self):
        # Team selection combobox
        self.team_select = self.findChild(QComboBox, "comboBox")
        if self.team_select:
            self.team_select.currentTextChanged.connect(self.handle_team_selection)

        # Game Controller checkbox
        self.game_controller_checkbox = self.findChild(QCheckBox, "gc_checkbox")
        if self.game_controller_checkbox:
            self.game_controller_checkbox.toggled.connect(self.handle_game_controller)

        # A* visualization checkbox
        self.astar_checkbox = self.findChild(QCheckBox, "aestrela_checkbox")
        if self.astar_checkbox:
            self.astar_checkbox.toggled.connect(self.update_visualization)

        # Team visibility checkboxes
        self.show_blue = self.findChild(QCheckBox, "show_blue")
        self.show_yellow = self.findChild(QCheckBox, "show_yellow")
        if self.show_blue:
            self.show_blue.toggled.connect(self.update_team_visibility)
        if self.show_yellow:
            self.show_yellow.toggled.connect(self.update_team_visibility)

    def handle_team_selection(self, team):
        """Handle team selection changes"""
        if self.game:
            is_yellow = team == "Time Amarelo"
            print(f"Selected team: {'Yellow' if is_yellow else 'Blue'}")
            # Update game configuration
            self.game.config["match"]["team_color"] = "yellow" if is_yellow else "blue"

    def setup_division_selector(self):
        self.division_combo = self.findChild(QComboBox, "division_combo")
        if self.division_combo:
            self.division_combo.currentTextChanged.connect(self.handle_division_change)

    def handle_game_controller(self, checked):
        print(f"Game Controller: {'enabled' if checked else 'disabled'}")
        # Implement game controller logic

    def handle_control_action(self, action):
        """Handle control button clicks"""
        print(f"Control action triggered: {action}")
        if not self.game:
            return

        if action == "TIMEOUT":
            # Request timeout
            pass
        elif action == "SUBSTITUTION":
            # Request substitution
            pass
        else:
            # Send command to game
            self.game.send_command(action)

    def update_visualization(self, checked):
        print(f"A* visualization: {'enabled' if checked else 'disabled'}")
        # Implement visualization update logic

    def update_team_visibility(self):
        if hasattr(self, "field_widget"):
            show_blue = self.show_blue.isChecked() if self.show_blue else True
            show_yellow = self.show_yellow.isChecked() if self.show_yellow else True
            self.field_widget.set_team_visibility(show_blue, show_yellow)

    def handle_division_change(self, division):
        """Handle division selection changes"""
        if hasattr(self, "field_widget"):
            self.field_widget.set_division(division)
            max_robots = self.field_widget.divisions[division]["max_robots"]
            print(f"Division changed to {division} (max {max_robots} robots)")

    def open_settings(self):
        try:
            # Create dialog and setup UI
            self.settings_dialog = QDialog()
            self.settings_ui = Ui_Dialog()
            self.settings_ui.setupUi(self.settings_dialog)

            # Determine config path
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "config.json",
            )
            print(f"Config path: {config_path}")
            print(f"Config file exists: {os.path.exists(config_path)}")

            # Load configuration
            try:
                with open(config_path, "r") as f:
                    self.current_config = json.load(f)
                print("Loaded config:", json.dumps(self.current_config, indent=2))
            except Exception as e:
                print(f"Error loading config: {e}")
                # Use a default configuration if loading fails
                self.current_config = {
                    "network": {
                        "multicast_ip": "224.5.23.2",
                        "vision_port": 10020,
                        "referee_ip": "224.5.23.1",
                        "referee_port": 10003,
                        "yellow_port": 10004,
                        "blue_port": 10005,
                    }
                }
                QMessageBox.warning(
                    self.settings_dialog,
                    "Error",
                    "Could not load configuration. Using default values.",
                )

            # Prioritize network section
            network_config = self.current_config.get("network", {})
            print("Network config:", json.dumps(network_config, indent=2))

            # Populate line edits with explicit type conversion and error checking
            def safe_set(line_edit, value):
                try:
                    line_edit.setText(str(value) if value is not None else "")
                    print(f"Setting {line_edit.objectName()} to {value}")
                except Exception as e:
                    print(f"Error setting {line_edit.objectName()}: {e}")

            # Update widget names here
            safe_set(
                self.settings_ui.multicast_ip_input, network_config.get("multicast_ip")
            )
            safe_set(
                self.settings_ui.vision_port_input, network_config.get("vision_port")
            )
            safe_set(
                self.settings_ui.referee_ip_input, network_config.get("referee_ip")
            )
            safe_set(
                self.settings_ui.referee_port_input, network_config.get("referee_port")
            )
            safe_set(
                self.settings_ui.yellow_port_input, network_config.get("yellow_port")
            )
            safe_set(self.settings_ui.blue_port_input, network_config.get("blue_port"))

            # Connect buttons (update button names here)
            self.settings_ui.confirm_button.clicked.connect(self.save_settings)
            self.settings_ui.reset_button.clicked.connect(self.reset_settings)

            # Show dialog
            self.settings_dialog.show()

        except Exception as e:
            print(f"Unexpected error in open_settings: {e}")
            QMessageBox.critical(
                None, "Critical Error", f"Could not open settings: {str(e)}"
            )

    def save_settings(self):
        try:
            # Validate and collect new settings
            updated_config = copy.deepcopy(self.current_config)

            # Update network section with new widget names
            network_settings = {
                "multicast_ip": self.settings_ui.multicast_ip_input.text(),
                "vision_port": int(self.settings_ui.vision_port_input.text()),
                "referee_ip": self.settings_ui.referee_ip_input.text(),
                "referee_port": int(self.settings_ui.referee_port_input.text()),
                "yellow_port": int(self.settings_ui.yellow_port_input.text()),
                "blue_port": int(self.settings_ui.blue_port_input.text()),
            }

            # Validate settings
            self.validate_settings({"network": network_settings})

            # Update network settings in the game instance
            if self.game:
                result = self.game.update_network_settings(network_settings)
                print(f"Network settings update result: {result}")

                if result:
                    # Determine config path
                    config_path = os.path.join(
                        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config.json",
                    )

                    # Save to config file
                    updated_config["network"] = network_settings
                    with open(config_path, "w") as f:
                        json.dump(updated_config, f, indent=4)

                    # Update current configuration
                    self.current_config = updated_config

                    QMessageBox.information(
                        self.settings_dialog,
                        "Settings",
                        "Network settings saved and applied successfully!",
                    )
                else:
                    QMessageBox.warning(
                        self.settings_dialog,
                        "Error",
                        "Could not apply network settings",
                    )
            else:
                QMessageBox.warning(
                    self.settings_dialog, "Error", "Game instance not available"
                )

        except ValueError as ve:
            QMessageBox.warning(self.settings_dialog, "Invalid Input", str(ve))
        except Exception as e:
            QMessageBox.warning(
                self.settings_dialog, "Error", f"Could not save settings: {str(e)}"
            )

    def validate_settings(self, config):
        """Validate network settings"""
        network = config.get("network", {})

        # Basic validation
        required_keys = [
            "multicast_ip",
            "vision_port",
            "referee_ip",
            "referee_port",
            "yellow_port",
            "blue_port",
        ]

        for key in required_keys:
            if key not in network or not network[key]:
                raise ValueError(f"Missing or invalid setting for {key}")

    def reset_settings(self):
        """Reset settings to default"""
        try:

            # Load default configuration
            default_config = self.load_default_config()

            # Update current configuration
            self.current_config = copy.deepcopy(default_config)

            # Repopulate settings dialog with new widget names
            self.settings_ui.multicast_ip_input.setText(
                default_config["network"]["multicast_ip"]
            )
            self.settings_ui.vision_port_input.setText(
                str(default_config["network"]["vision_port"])
            )
            self.settings_ui.referee_ip_input.setText(
                default_config["network"]["referee_ip"]
            )
            self.settings_ui.referee_port_input.setText(
                str(default_config["network"]["referee_port"])
            )
            self.settings_ui.yellow_port_input.setText(
                str(default_config["network"]["yellow_port"])
            )
            self.settings_ui.blue_port_input.setText(
                str(default_config["network"]["blue_port"])
            )

            QMessageBox.information(
                self.settings_dialog, "Reset", "Settings have been reset to default!"
            )

        except Exception as e:
            QMessageBox.warning(
                self.settings_dialog, "Error", f"Could not reset settings: {str(e)}"
            )

    def load_default_config(self):
        """Load default configuration"""
        default_config = {
            "network": {
                "multicast_ip": "224.5.23.2",
                "vision_port": 10020,
                "referee_ip": "224.5.23.1",
                "referee_port": 10003,
                "yellow_port": 10004,
                "blue_port": 10005,
            },
            "match": {
                "team_1": "UnBall",
                "team_2": "Unknown",
                "event": "Unknown",
                "team_side": "left",
                "team_color": "blue",
                "time_logging": False,
            },
        }
        return default_config

    def setup_division_selector(self):
        self.division_combo = self.findChild(QComboBox, "division_combo")
        if self.division_combo:
            self.division_combo.currentTextChanged.connect(self.handle_division_change)

            # Force initial division setup after the window is fully loaded
            QTimer.singleShot(100, self.initial_division_setup)

    def initial_division_setup(self):
        if hasattr(self, "field_widget"):
            initial_division = self.division_combo.currentText()
            self.handle_division_change(initial_division)

    def capture_log(self, text):
        """Capture logs and categorize them"""
        if (
            "Path" in text
            or "path" in text
            or "obstacles" in text
            or "waypoint" in text
        ):
            self.path_planning_logs.append(text)
            # Limit log size
            if len(self.path_planning_logs) > 500:
                self.path_planning_logs = self.path_planning_logs[-500:]

        elif "Vision" in text or "vision" in text or "Ball" in text or "robot" in text:
            self.vision_logs.append(text)
            if len(self.vision_logs) > 500:
                self.vision_logs = self.vision_logs[-500:]

        elif "Sim" in text or "controller" in text or "command" in text:
            self.sim_logs.append(text)
            if len(self.sim_logs) > 500:
                self.sim_logs = self.sim_logs[-500:]

    def show_debug_window(self):
        """Show the debug window when the debug action is triggered"""
        if self.debug_window is None or not self.debug_window.isVisible():
            self.debug_window = DebugWindow(parent=self, game=self.game)

        # Show the window and bring to front
        self.debug_window.show()
        self.debug_window.raise_()
        self.debug_window.activateWindow()

    def update_visualization_settings(self, checked):
        """Update visualization settings based on A* checkbox"""
        if hasattr(self.field_widget, "set_show_paths"):
            self.field_widget.set_show_paths(checked)
        else:
            print("Field widget does not have set_show_paths method")

    def show_debug_window(self):
        """Show the debug window as a separate window"""
        if self.debug_window is None or not self.debug_window.isVisible():
            # Create as a standalone window (no parent)
            self.debug_window = DebugWindow(parent=None, game=self.game)

        # Show the window and bring to front
        self.debug_window.show()
        self.debug_window.raise_()
        self.debug_window.activateWindow()
