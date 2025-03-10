from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import io
from .debug_ui import Ui_debug_selector  # Import the generated UI class


# In PyQt/debug_window.py
class DebugStreamRedirector(QtCore.QObject):
    """Redirects print statements to both console and debug window"""

    text_written = QtCore.pyqtSignal(str)

    def __init__(self, stream=None):
        super().__init__()
        self.stream = stream
        self._buffer = io.StringIO()
        self._active = True

    def write(self, text):
        if not self._active:
            # If we're no longer active, just write to the original stream
            if self.stream:
                self.stream.write(text)
            return

        try:
            if self.stream:
                self.stream.write(text)
            self._buffer.write(text)
            self.text_written.emit(text)
        except RuntimeError:
            # If we get a C++ object deleted error, deactivate
            self._active = False
            if self.stream:
                self.stream.write(text)

    def flush(self):
        if self.stream:
            self.stream.flush()

    def get_buffer_contents(self):
        return self._buffer.getvalue()

    def clear_buffer(self):
        self._buffer = io.StringIO()

    def deactivate(self):
        """Deactivate the redirector to prevent errors when window closes"""
        self._active = False


class DebugWindow(QtWidgets.QWidget):
    def __init__(self, parent=None, game=None):
        super().__init__(parent)

        # Set window flags to make it a standalone window
        if parent is None:
            self.setWindowFlags(QtCore.Qt.Window)

        self.ui = Ui_debug_selector()
        self.ui.setupUi(self)
        self.game = game

        # Set up text area for logs in the frame
        self.log_text = QtWidgets.QTextEdit(self.ui.frame)
        self.log_text.setGeometry(10, 10, 491, 311)
        self.log_text.setReadOnly(True)

        # Connect signals
        self.ui.box_all_robots.currentIndexChanged.connect(self.update_debug_info)

        # Initial update
        self.update_debug_info()

    def update_debug_info(self):
        """Update debug information based on selected option"""
        selected = self.ui.box_all_robots.currentText().strip()
        self.log_text.clear()

        if "Path Planning" in selected:
            self.show_path_planning_info()
        elif "Vision" in selected:
            self.show_vision_info()
        elif "Sim" in selected:
            self.show_sim_info()

    def show_path_planning_info(self):
        """Display path planning related information"""
        if not self.game:
            self.log_text.append("Game instance not available")
            return

        # Display path planning settings
        self.log_text.append("Path Planning Settings:")
        self.log_text.append("-----------------------")
        self.log_text.append(f"Field bounds: {self.game.field_bounds}")

        # Display robot paths if available
        self.log_text.append("\nCurrent Robot Paths:")
        if hasattr(self.game, "robot_state_machines"):
            for robot_id, state_machine in self.game.robot_state_machines.items():
                self.log_text.append(f"Robot {robot_id} ({state_machine.role}):")
                self.log_text.append(f"  Current state: {state_machine.current_state}")
                self.log_text.append(
                    f"  Target position: {state_machine.target_position}"
                )

                # Get path for this robot
                path = self.game.path_planner.get_path(robot_id)
                if path:
                    self.log_text.append(f"  Path with {len(path)} waypoints")
                    # Optionally show first few waypoints
                    if len(path) > 0:
                        self.log_text.append(f"  First waypoint: {path[0]}")
                    if len(path) > 1:
                        self.log_text.append(f"  Last waypoint: {path[-1]}")
                else:
                    self.log_text.append("  No path available")

                self.log_text.append("")

        # Add captured logs
        if (
            hasattr(self.parent(), "path_planning_logs")
            and self.parent().path_planning_logs
        ):
            self.log_text.append("\nRecent Path Planning Logs:")
            for log in self.parent().path_planning_logs[-30:]:  # Show last 30 logs
                self.log_text.append(log.rstrip())

    def show_vision_info(self):
        """Display vision system information"""
        if not self.game:
            self.log_text.append("Game instance not available")
            return

        self.log_text.append("Vision System Information:")
        self.log_text.append("-------------------------")
        self.log_text.append(
            f"Vision IP: {self.game.config['network']['multicast_ip']}"
        )
        self.log_text.append(
            f"Vision Port: {self.game.config['network']['vision_port']}"
        )

        # Show ball data
        if self.game.last_vision_data and "ball" in self.game.last_vision_data:
            ball = self.game.last_vision_data["ball"]
            self.log_text.append(f"\nBall Position: ({ball['x']:.2f}, {ball['y']:.2f})")

        # Show robot data
        self.log_text.append("\nDetected Robots:")
        if self.game.last_vision_data:
            for team, color in [("robotsBlue", "Blue"), ("robotsYellow", "Yellow")]:
                if team in self.game.last_vision_data:
                    visible_robots = [
                        id
                        for id, robot in self.game.last_vision_data[team].items()
                        if robot["x"] is not None
                    ]
                    self.log_text.append(
                        f"{color} Team: {len(visible_robots)} robots visible"
                    )
                    for robot_id in visible_robots:
                        robot = self.game.last_vision_data[team][robot_id]
                        self.log_text.append(
                            f"  Robot {robot_id}: ({robot['x']:.2f}, {robot['y']:.2f}), θ: {robot['theta']:.2f}"
                        )

        # Add captured logs
        if hasattr(self.parent(), "vision_logs") and self.parent().vision_logs:
            self.log_text.append("\nRecent Vision Logs:")
            for log in self.parent().vision_logs[-30:]:  # Show last 30 logs
                self.log_text.append(log.rstrip())

    def show_sim_info(self):
        """Display simulator information"""
        if not self.game:
            self.log_text.append("Game instance not available")
            return

        self.log_text.append("Simulator Information:")
        self.log_text.append("---------------------")

        # Get controller information
        self.log_text.append(f"Active controller: {self.game.active_controller.mode}")

        # Team colors
        self.log_text.append(f"Team color: {self.game.config['match']['team_color']}")

        # Show team ports
        self.log_text.append("\nTeam Ports:")
        self.log_text.append(f"Blue team port: 10301")
        self.log_text.append(f"Yellow team port: 10302")

        # Show robot stats
        self.log_text.append("\nRobot Commands:")
        if hasattr(self.game.blue_robots, "get_stats"):
            stats = self.game.blue_robots.get_stats()
            self.log_text.append(f"Blue team commands sent: {stats['sent_commands']}")
            self.log_text.append(
                f"Blue team success rate: {stats['success_rate']:.1f}%"
            )

        if hasattr(self.game.yellow_robots, "get_stats"):
            stats = self.game.yellow_robots.get_stats()
            self.log_text.append(f"Yellow team commands sent: {stats['sent_commands']}")
            self.log_text.append(
                f"Yellow team success rate: {stats['success_rate']:.1f}%"
            )

        # Add captured logs
        if hasattr(self.parent(), "sim_logs") and self.parent().sim_logs:
            self.log_text.append("\nRecent Simulator Logs:")
            for log in self.parent().sim_logs[-30:]:  # Show last 30 logs
                self.log_text.append(log.rstrip())
