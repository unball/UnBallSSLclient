from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QDialog,
    QPushButton,
    QLineEdit,
    QMessageBox,
    QAction,
)
from PyQt5 import QtWidgets, uic


from PyQt.field_visualization import FieldVisualization
from PyQt.main_ui import Ui_MainWindow

import json
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, game_instance):
        super().__init__()
        self.game = game_instance

        # Set up the UI generated from Qt Designer
        self.setupUi(self)

        # Setup field visualization
        self.setup_field_visualization()

        # Connect control buttons
        self.connect_controls()

        # Set up update timer for visualization
        self.visualization_timer = QTimer()
        self.visualization_timer.timeout.connect(self.update_visualization)
        self.visualization_timer.start(16)  # ~60 FPS

        self.actionAbrir = self.findChild(QAction, "actionAbrir")
        self.actionAbrir.triggered.connect(self.open_settings)

    def setup_field_visualization(self):
        """Setup field visualization widget"""
        self.field_widget = FieldVisualization()

        # Get the field frame from the UI
        if hasattr(self, "field_frame"):
            layout = QVBoxLayout(self.field_frame)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.field_widget)

    def handle_game_controller_toggle(self, checked):
        """Handle game controller checkbox separately from commands"""
        if self.game:
            print(f"Game controller {'enabled' if checked else 'disabled'}")
            # Call appropriate game method
            self.game.set_game_controller_enabled(checked)

    def connect_controls(self):
        """Connect UI controls to their handlers"""
        # Game Controller checkbox - this should use a different handler
        if hasattr(self, "gc_checkbox"):
            self.gc_checkbox.toggled.connect(self.handle_game_controller_toggle)

        # Team visibility toggles
        if hasattr(self, "show_blue"):
            self.show_blue.toggled.connect(self.update_team_visibility)
        if hasattr(self, "show_yellow"):
            self.show_yellow.toggled.connect(self.update_team_visibility)

        # Connect control buttons using a dictionary of commands
        button_commands = {
            "pushButton_2": "HALT",
            "pushButton": "FORCE_START",
            "pushButton_4": "NORMAL_START",
            "pushButton_3": "STOP",
            "pushButton_9": "FREE_KICK",
            "pushButton_8": "KICK_OFF",
            "pushButton_10": "PENALTY",
        }

        for button_name, command in button_commands.items():
            if hasattr(self, button_name):
                button = getattr(self, button_name)
                button.clicked.connect(
                    lambda checked, cmd=command: self.handle_command(cmd)
                )

    def open_settings(self):
        self.settings_dialog = SettingsDialog(self)
        self.settings_dialog.exec_()

    def update_visualization(self):
        """Update field visualization with latest data"""
        if not self.game:
            print("No game instance")
            return

        vision_data = self.game.get_vision_data()
        # print("Vision data:", vision_data)  # Debug print
        if not vision_data:
            return

        # Update ball position
        if "ball" in vision_data and vision_data["ball"]["x"] is not None:
            ball = vision_data["ball"]
            # print(f"Ball position: x={ball['x']}, y={ball['y']}")  # Debug print
            self.field_widget.update_ball(ball["x"], ball["y"])

        # Update blue robots
        if "robotsBlue" in vision_data:
            for robot_id, robot in vision_data["robotsBlue"].items():
                if robot["x"] is not None:
                    # print(
                    #    f"Blue robot {robot_id}: x={robot['x']}, y={robot['y']}, θ={robot['theta']}"
                    # )  # Debug print
                    self.field_widget.update_robot(
                        robot["x"], robot["y"], robot["theta"], Qt.blue, robot_id
                    )

        # Update yellow robots
        if "robotsYellow" in vision_data:
            for robot_id, robot in vision_data["robotsYellow"].items():
                if robot["x"] is not None:
                    # print(
                    #    f"Yellow robot {robot_id}: x={robot['x']}, y={robot['y']}, θ={robot['theta']}"
                    # )  # Debug print
                    self.field_widget.update_robot(
                        robot["x"], robot["y"], robot["theta"], Qt.yellow, robot_id
                    )

    def update_team_visibility(self):
        """Update team visibility based on checkboxes"""
        if hasattr(self, "show_blue") and hasattr(self, "show_yellow"):
            self.field_widget.set_team_visibility(
                self.show_blue.isChecked(), self.show_yellow.isChecked()
            )

    def handle_game_controller(self, checked):
        """Handle game controller toggle"""
        if self.game:
            self.game.set_game_controller_enabled(checked)

    def handle_command(self, command):
        """Handle button commands only"""
        if self.game and isinstance(command, str):  # Make sure command is a string
            print(f"Sending command: {command}")
            self.game.send_command(command)

    def closeEvent(self, event):
        """Handle window close event"""
        if self.game:
            self.game.stop()
        event.accept()


def get_config(config_file=None):
    """Load configuration from file"""
    try:
        if config_file:
            config_path = config_file
        else:
            config_path = os.path.join("..", "config.json")

        with open(config_path, "r") as f:
            config = json.load(f)

        return config

    except FileNotFoundError:
        raise FileNotFoundError("Config file not found")
    except json.JSONDecodeError:
        raise ValueError("Invalid JSON in config file")


class SettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        ui_file = os.path.join(SCRIPT_DIR, "settings.ui")
        uic.loadUi(ui_file, self)

        # Get references to UI elements
        self.confirm_button = self.findChild(QPushButton, "pushButton")
        self.reset_button = self.findChild(QPushButton, "pushButton_2")

        self.multicast_ip = self.findChild(QLineEdit, "multicast_ip")
        self.vision_port = self.findChild(QLineEdit, "vision_port")
        self.referee_ip = self.findChild(QLineEdit, "referee_ip")
        self.referee_port = self.findChild(QLineEdit, "referee_port")
        self.yellow_port = self.findChild(QLineEdit, "yellow_port")
        self.blue_port = self.findChild(QLineEdit, "blue_port")

        # Connect buttons
        self.confirm_button.clicked.connect(self.apply_settings)
        self.reset_button.clicked.connect(self.reset_default)

        # Load saved settings
        self.load_settings()

    def apply_settings(self):
        try:
            # Save the settings to the config file
            config = get_config()
            config["network"]["multicast_ip"] = self.multicast_ip.text()
            config["network"]["vision_port"] = int(self.vision_port.text())
            config["network"]["referee_ip"] = self.referee_ip.text()
            config["network"]["referee_port"] = int(self.referee_port.text())
            config["network"]["yellow_port"] = int(self.yellow_port.text())
            config["network"]["blue_port"] = int(self.blue_port.text())

            with open(os.path.join("..", "config.json"), "w") as f:
                json.dump(config, f, indent=4)

            # Apply the settings to your application
            # For example, updating network connections with new values
            QMessageBox.information(self, "Success", "Settings applied successfully")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to apply settings: {str(e)}")

    def reset_default(self):
        config = get_config()
        self.multicast_ip.setText(config["network"]["multicast_ip"])
        self.vision_port.setText(str(config["network"]["vision_port"]))
        self.referee_ip.setText(config["network"]["referee_ip"])
        self.referee_port.setText(str(config["network"]["referee_port"]))
        self.yellow_port.setText(str(config["network"]["yellow_port"]))
        self.blue_port.setText(str(config["network"]["blue_port"]))

    def load_settings(self):
        try:
            config = get_config()
            self.multicast_ip.setText(config["network"]["multicast_ip"])
            self.vision_port.setText(str(config["network"]["vision_port"]))
            self.referee_ip.setText(config["network"]["referee_ip"])
            self.referee_port.setText(str(config["network"]["referee_port"]))
            self.yellow_port.setText(str(config["network"]["yellow_port"]))
            self.blue_port.setText(str(config["network"]["blue_port"]))
        except Exception as e:
            print(f"Error loading settings: {str(e)}")
            self.reset_default()
