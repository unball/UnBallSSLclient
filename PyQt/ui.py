# flake8: noqa

import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)

import signal
import json
import os
import sys
import math
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import (
    QMainWindow,
    QDialog,
    QApplication,
    QFrame,
    QGraphicsScene,
    QGraphicsView,
    QVBoxLayout,
    QPushButton,
    QLineEdit,
    QMessageBox,
    QCheckBox,
)
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QAction  # Add this line

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_config():
    config_path = os.path.join(SCRIPT_DIR, "../config.json")
    try:
        with open(config_path, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Config file not found at {config_path}")
        return None
    except json.JSONDecodeError:
        print(f"Error decoding config file at {config_path}")
        return None


class SettingsDialog(QDialog):
    def __init__(self):
        super().__init__()
        ui_file = os.path.join(SCRIPT_DIR, "settings.ui")
        uic.loadUi(ui_file, self)

        # Get references to UI elements
        self.confirm_button = self.findChild(QPushButton, "pushButton")
        self.reset_button = self.findChild(QPushButton, "pushButton_2")

        # Get references to line edits
        self.multicast_ip = self.findChild(QLineEdit, "lineEdit")
        self.vision_port = self.findChild(QLineEdit, "lineEdit_2")
        self.referee_ip = self.findChild(QLineEdit, "lineEdit_3")
        self.referee_port = self.findChild(QLineEdit, "lineEdit_4")
        self.yellow_port = self.findChild(QLineEdit, "lineEdit_5")
        self.blue_port = self.findChild(QLineEdit, "lineEdit_6")

        # Connect buttons
        self.confirm_button.clicked.connect(self.apply_settings)
        self.reset_button.clicked.connect(self.reset_default)

        # Load saved settings
        self.load_settings()

    def apply_settings(self):
        # Just apply the settings without saving to file or closing
        try:
            # Here you would apply the settings to your application
            # For example, updating network connections with new values
            QMessageBox.information(self, "Success", "Settings applied successfully")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to apply settings: {str(e)}")

    def reset_default(self):
        default_values = {
            "multicast_ip": "224.5.23.2",
            "vision_port": "10002",
            "referee_ip": "224.5.23.1",
            "referee_port": "10003",
            "yellow_port": "20011",
            "blue_port": "20012",
        }

        self.multicast_ip.setText(default_values["multicast_ip"])
        self.vision_port.setText(default_values["vision_port"])
        self.referee_ip.setText(default_values["referee_ip"])
        self.referee_port.setText(default_values["referee_port"])
        self.yellow_port.setText(default_values["yellow_port"])
        self.blue_port.setText(default_values["blue_port"])

    def load_settings(self):
        self.reset_default()  # Just load default values for now


class FieldVisualization(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)

        # Set a fixed size slightly smaller than original
        self.view.setFixedSize(700, 500)

        # Disable scrollbars
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Create layout and center the view
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        layout.setAlignment(Qt.AlignCenter)  # Center the view
        layout.addWidget(self.view)

        # Initial field setup
        self.setup_field()

    def setup_field(self):
        field_color = QColor(0, 150, 0)
        field_pen = QPen(Qt.white, 2)

        # Draw main field rectangle
        self.scene.addRect(QRectF(0, 0, 700, 500), field_pen, QBrush(field_color))

        # Draw center line
        self.scene.addLine(QLineF(350, 0, 350, 500), field_pen)

        # Draw center circle
        center_circle = QRectF(300, 200, 100, 100)
        self.scene.addEllipse(center_circle, field_pen, QBrush(Qt.transparent))

        # Draw penalty areas
        left_penalty = QRectF(0, 150, 100, 200)
        right_penalty = QRectF(600, 150, 100, 200)
        self.scene.addRect(left_penalty, field_pen, QBrush(Qt.transparent))
        self.scene.addRect(right_penalty, field_pen, QBrush(Qt.transparent))

    def add_robot(self, x, y, orientation, team_color, robot_id):
        robot_size = 30
        robot_rect = QRectF(
            x - robot_size / 2, y - robot_size / 2, robot_size, robot_size
        )

        # Draw robot body
        brush = QBrush(team_color)
        self.scene.addEllipse(robot_rect, QPen(Qt.black), brush)

        # Draw orientation line
        line_length = robot_size / 2
        end_x = x + line_length * math.cos(orientation)
        end_y = y + line_length * math.sin(orientation)
        self.scene.addLine(QLineF(x, y, end_x, end_y), QPen(Qt.black, 2))

        # Draw ID
        text = self.scene.addText(str(robot_id))
        text.setDefaultTextColor(Qt.black)
        text.setPos(
            x - text.boundingRect().width() / 2, y - text.boundingRect().height() / 2
        )

    def add_ball(self, x, y):
        ball_size = 15
        ball_rect = QRectF(x - ball_size / 2, y - ball_size / 2, ball_size, ball_size)
        self.scene.addEllipse(ball_rect, QPen(Qt.black), QBrush(QColor(255, 165, 0)))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        ui_file = os.path.join(SCRIPT_DIR, "main.ui")
        uic.loadUi(ui_file, self)

        # Set up field visualization
        self.field_widget = FieldVisualization()
        self.field_frame = self.findChild(QFrame, "field_frame")

        # Create layout that centers the field
        layout = QVBoxLayout(self.field_frame)
        layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        layout.setAlignment(Qt.AlignCenter)  # Center alignment
        layout.addWidget(self.field_widget)

        # Connect menu action
        self.actionAbrir = self.findChild(QAction, "actionAbrir")
        self.actionAbrir.triggered.connect(self.open_settings)

        # Connect game controller checkbox
        self.game_controller_checkbox = self.findChild(QCheckBox, "gc_checkbox")
        if self.game_controller_checkbox:
            self.game_controller_checkbox.toggled.connect(self.handle_game_controller)

        # Connect control buttons
        self.setup_control_buttons()

        # Connect checkbox for visualization
        self.setup_visualization_checkbox()

    def setup_control_buttons(self):
        control_buttons = {
            "HALT": "pushButton_2",
            "FORCE_START": "pushButton",
            "NORMAL_START": "pushButton_4",
            "STOP": "pushButton_3",
            "FREE_KICK": "pushButton_9",
            "KICK_OFF": "pushButton_8",
            "PENALTI": "pushButton_10",
            "POSICIONAMENTO": "pushButton_11",
            "POSICIONAMENTO_2": "pushButton_12",
            "POSICIONAMENTO_3": "pushButton_13",
        }

        for action, button_name in control_buttons.items():
            button = self.findChild(QPushButton, button_name)
            if button:
                button.clicked.connect(
                    lambda checked, a=action: self.handle_control_action(a)
                )

    def setup_visualization_checkbox(self):
        self.astar_checkbox = self.findChild(QCheckBox, "aestrela_checkbox")
        if self.astar_checkbox:
            self.astar_checkbox.toggled.connect(self.update_visualization)

    def handle_game_controller(self, checked):
        print(f"Game Controller: {'enabled' if checked else 'disabled'}")

    def handle_control_action(self, action):
        print(f"Control action triggered: {action}")
        # Implement your control logic here

    def update_visualization(self, checked):
        print(f"A* visualization: {'enabled' if checked else 'disabled'}")
        # Implement your visualization update logic here

    def open_settings(self):
        self.settings_dialog = SettingsDialog()
        self.settings_dialog.show()  # Using show() instead of exec_() to keep it non-modal


if __name__ == "__main__":
    # Enable clean Ctrl+C handling
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
