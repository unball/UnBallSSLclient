# test_behavior.py
import time
import math
from SimulationGrSim.RobotControlClient_threaded import ThreadedRobotControlClient
from RobotBehavior.robot_state_machine import (
    GoalkeeperStateMachine,
    DefenderStateMachine,
    AttackerStateMachine,
)
from RobotBehavior.robot_states import RobotRole
from VisionClient.Vision import Vision
from PathPlanning.path_planner import PathPlanner


class TestGame:
    """Simple game class for testing"""

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

    def get_vision_data(self):
        """Get latest vision data"""
        return self.vision.get_last_frame()

    def stop(self):
        """Stop all components"""
        self.blue_robots.stop()
        self.vision.stop()
        self.path_planner.stop()


def test_goalkeeper():
    """Test goalkeeper behavior"""
    game = TestGame()

    try:
        # Create goalkeeper state machine
        goalkeeper = GoalkeeperStateMachine(robot_id=0, team_color="blue", game=game)

        # Main test loop
        start_time = time.time()
        while time.time() - start_time < 30:  # Run for 30 seconds
            vision_data = game.get_vision_data()
            if vision_data:
                # Update goalkeeper
                goalkeeper.update(vision_data)

                # Print current state
                print(f"Goalkeeper State: {goalkeeper.current_state}")
                print(
                    f"Ball Position: {vision_data.get('ball', {}).get('x')}, {vision_data.get('ball', {}).get('y')}"
                )

            time.sleep(0.016)  # ~60Hz update rate

    finally:
        # Clean up
        game.stop()


def test_multiple_robots():
    """Test multiple robots with different roles"""
    game = TestGame()

    try:
        # Create state machines for different roles
        game.state_machines[0] = GoalkeeperStateMachine(
            robot_id=0, team_color="blue", game=game
        )
        game.state_machines[1] = DefenderStateMachine(
            robot_id=1, team_color="blue", game=game
        )
        game.state_machines[2] = AttackerStateMachine(
            robot_id=2, team_color="blue", game=game
        )

        # Main test loop
        start_time = time.time()
        while time.time() - start_time < 60:  # Run for 60 seconds
            vision_data = game.get_vision_data()
            if vision_data:
                # Update all robots
                for robot_id, state_machine in game.state_machines.items():
                    state_machine.update(vision_data)

                # Print current states
                for robot_id, state_machine in game.state_machines.items():
                    print(
                        f"Robot {robot_id} ({state_machine.role.value}): {state_machine.current_state.value}"
                    )

                # Print ball position
                if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                    print(
                        f"Ball: ({vision_data['ball']['x']:.2f}, {vision_data['ball']['y']:.2f})"
                    )

            time.sleep(0.016)  # ~60Hz update rate

    finally:
        # Clean up
        game.stop()


if __name__ == "__main__":
    # Choose which test to run
    # test_goalkeeper()
    test_multiple_robots()
