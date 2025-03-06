# test_visualization.py
import sys
import time
import math
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer, Qt
from PyQt.field_visualization import FieldVisualization
from SimulationGrSim.RobotControlClient_threaded import ThreadedRobotControlClient
from RobotBehavior.robot_state_machine import (
    GoalkeeperStateMachine,
    DefenderStateMachine,
    AttackerStateMachine,
)
from VisionClient.Vision import Vision
from PathPlanning.path_planner import PathPlanner


class TestGame:
    """Simple game class for testing with visualization"""

    def __init__(self):
        self.config = {
            "network": {"multicast_ip": "224.5.23.2", "vision_port": 10020},
            "match": {"team_side": "left", "team_color": "blue"},
        }

        # Initialize controllers
        self.blue_robots = ThreadedRobotControlClient(team_port=10301)
        self.blue_robots.start()

        # Set active controller
        self.active_controller = self.blue_robots

        # Initialize vision
        self.vision = Vision(self)
        self.vision.start()

        # Initialize path planner
        self.path_planner = PathPlanner(self)
        self.path_planner.start()

        # Set field bounds
        self.field_bounds = {"x_min": -2.25, "x_max": 2.25, "y_min": -1.5, "y_max": 1.5}

        # Initialize robots
        self.state_machines = {}
        self.state_machines[0] = GoalkeeperStateMachine(
            robot_id=0, team_color="blue", game=self
        )
        self.state_machines[1] = DefenderStateMachine(
            robot_id=1, team_color="blue", game=self
        )
        self.state_machines[2] = AttackerStateMachine(
            robot_id=2, team_color="blue", game=self
        )

    def get_vision_data(self):
        """Get latest vision data"""
        return self.vision.get_last_frame()

    def stop(self):
        """Stop all components"""
        self.blue_robots.stop()
        self.vision.stop()
        self.path_planner.stop()


def main():
    # Create Qt application
    app = QApplication(sys.argv)

    # Create field visualization
    field_viz = FieldVisualization()
    field_viz.set_division("Entry Level")  # SSL-EL field dimensions
    field_viz.resize(800, 600)
    field_viz.show()

    # Create game instance
    game = TestGame()

    # Enable path visualization
    field_viz.set_show_paths(True)

    # Timer for updates
    update_timer = QTimer()

    def update():
        # Get vision data
        vision_data = game.get_vision_data()
        if not vision_data:
            return

        # Update robot state machines
        for robot_id, state_machine in game.state_machines.items():
            state_machine.update(vision_data)

        # Update field visualization with ball position
        if "ball" in vision_data and vision_data["ball"]["x"] is not None:
            field_viz.update_ball(vision_data["ball"]["x"], vision_data["ball"]["y"])

        # Update robot positions
        for team in ["robotsBlue", "robotsYellow"]:
            team_color = Qt.blue if team == "robotsBlue" else Qt.yellow
            for robot_id, robot in vision_data[team].items():
                if robot["x"] is not None:
                    field_viz.update_robot(
                        robot["x"],
                        robot["y"],
                        robot["theta"],
                        team_color,
                        int(robot_id),
                    )

        # Update path visualization
        for robot_id, state_machine in game.state_machines.items():
            path = game.path_planner.get_path(robot_id)
            field_viz.update_path(robot_id, path)

    update_timer.timeout.connect(update)
    update_timer.start(16)  # ~60Hz update rate

    try:
        sys.exit(app.exec_())
    finally:
        game.stop()


if __name__ == "__main__":
    main()
