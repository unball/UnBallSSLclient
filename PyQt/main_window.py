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
    QFrame,
    QCheckBox,
    QComboBox,
)
from PyQt5 import QtWidgets, uic


from PyQt.field_visualization import FieldVisualization
from PyQt.main_ui import Ui_MainWindow

import json
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class MainWindow(QMainWindow):
    def __init__(self, game=None):  # Add game parameter with default None
        super().__init__()
        self.game = game  # Store game reference
        ui_file = os.path.join(SCRIPT_DIR, "main.ui")
        uic.loadUi(ui_file, self)

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

        # Setup timer for regular updates if game is provided
        if self.game:
            self.update_timer = QTimer()
            self.update_timer.timeout.connect(self.update_display)
            self.update_timer.start(16)  # ~60 FPS

    def update_display(self):
        """Update field display with latest game data"""
        if not self.game:
            return

        # Get latest vision data
        vision_data = self.game.get_vision_data()
        if vision_data:
            # Update ball
            if "ball" in vision_data:
                ball = vision_data["ball"]
                if ball.get("x") is not None:
                    self.field_widget.update_ball(ball["x"], ball["y"])

            # Update blue robots
            if "robotsBlue" in vision_data:
                for robot_id, robot in vision_data["robotsBlue"].items():
                    if robot["x"] is not None:
                        self.field_widget.update_robot(
                            robot["x"], robot["y"], robot["theta"], Qt.blue, robot_id
                        )

            # Update yellow robots
            if "robotsYellow" in vision_data:
                for robot_id, robot in vision_data["robotsYellow"].items():
                    if robot["x"] is not None:
                        self.field_widget.update_robot(
                            robot["x"], robot["y"], robot["theta"], Qt.yellow, robot_id
                        )

    def setup_menu_actions(self):
        self.actionAbrir = self.findChild(QAction, "actionAbrir")
        if self.actionAbrir:
            self.actionAbrir.triggered.connect(self.open_settings)

    def setup_control_buttons(self):
        # Map button names to actions
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

        # Connect all buttons
        for action, button_name in control_buttons.items():
            button = self.findChild(QPushButton, button_name)
            if button:
                button.clicked.connect(
                    lambda checked, a=action: self.handle_control_action(a)
                )

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
        print(f"Control action triggered: {action}")
        # Implement control action logic

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
        self.settings_dialog = SettingsDialog()
        self.settings_dialog.show()
