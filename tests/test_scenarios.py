# test_scenarios.py
import time
from test_behavior import TestGame
from RobotBehavior.robot_state_machine import (
    GoalkeeperStateMachine,
    DefenderStateMachine,
    AttackerStateMachine,
)


def test_ball_defense_scenario():
    """Test scenario with ball threatening goal"""
    game = TestGame()

    try:
        # Create robots
        goalkeeper = GoalkeeperStateMachine(robot_id=0, team_color="blue", game=game)
        defender = DefenderStateMachine(robot_id=1, team_color="blue", game=game)

        # Override positions in vision data for testing
        def mock_get_vision_data():
            """Create mock vision data with ball near goal"""
            vision_data = {
                "ball": {"x": -1.5, "y": 0.2},
                "robotsBlue": {
                    0: {"x": -2.0, "y": 0.0, "theta": 0.0},
                    1: {"x": -1.0, "y": 0.0, "theta": 0.0},
                },
                "robotsYellow": {},
            }
            return vision_data

        # Replace vision data method
        game.get_vision_data = mock_get_vision_data

        # Run test for 10 seconds
        start_time = time.time()
        while time.time() - start_time < 10:
            vision_data = game.get_vision_data()

            # Update robots
            goalkeeper.update(vision_data)
            defender.update(vision_data)

            # Print states
            print(
                f"Goalkeeper: {goalkeeper.current_state.value}, Target: {goalkeeper.target_position}"
            )
            print(
                f"Defender: {defender.current_state.value}, Target: {defender.target_position}"
            )

            time.sleep(0.1)

    finally:
        game.stop()


def test_attacker_ball_pursuit():
    """Test attacker pursuing the ball"""
    game = TestGame()

    try:
        # Create attacker
        attacker = AttackerStateMachine(robot_id=2, team_color="blue", game=game)

        # Create position sequence for ball
        ball_positions = [
            {"x": 0.0, "y": 0.0},
            {"x": 0.5, "y": 0.5},
            {"x": 1.0, "y": 0.0},
            {"x": 1.5, "y": -0.5},
        ]

        # Override vision data
        def get_mock_vision_data(time_index):
            """Create mock vision data with moving ball"""
            index = min(int(time_index), len(ball_positions) - 1)
            vision_data = {
                "ball": ball_positions[index],
                "robotsBlue": {2: {"x": 0.0, "y": 0.0, "theta": 0.0}},
                "robotsYellow": {},
            }
            return vision_data

        # Run test for each ball position
        for i in range(len(ball_positions)):
            # Get vision data for this time step
            vision_data = get_mock_vision_data(i)

            # Update attacker
            attacker.update(vision_data)

            # Print state
            print(f"Ball at {ball_positions[i]}")
            print(
                f"Attacker: {attacker.current_state.value}, Target: {attacker.target_position}"
            )

            time.sleep(1.0)  # Wait to see behavior change

    finally:
        game.stop()


if __name__ == "__main__":
    # Choose which scenario to test
    test_ball_defense_scenario()
    # test_attacker_ball_pursuit()
# The test_ball_defense_scenario() function creates a goalkeeper and defender robot, then simulates a scenario where the ball is near the goal. The test_attacker_ball_pursuit() function creates an attacker robot and simulates a scenario where the attacker pursues a moving ball. Both scenarios run for a fixed duration and print the state of the robots at each time step. The test functions use a mock vision data function to simulate the robot positions and ball positions for testing purposes. The test functions demonstrate how to test specific robot behaviors and scenarios using the robot state machine classes.

