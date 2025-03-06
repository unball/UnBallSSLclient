# test_cli.py
import time
import argparse
from test_behavior import TestGame
from RobotBehavior.robot_state_machine import (
    GoalkeeperStateMachine,
    DefenderStateMachine,
    AttackerStateMachine,
)


def main():
    parser = argparse.ArgumentParser(description="Test robot behaviors")
    parser.add_argument(
        "--role",
        choices=["goalkeeper", "defender", "attacker", "all"],
        default="all",
        help="Robot role to test",
    )
    parser.add_argument(
        "--duration", type=int, default=30, help="Test duration in seconds"
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Print detailed state information"
    )

    args = parser.parse_args()

    # Create game instance
    game = TestGame()

    try:
        # Create robots based on args
        state_machines = {}

        if args.role == "goalkeeper" or args.role == "all":
            state_machines[0] = GoalkeeperStateMachine(
                robot_id=0, team_color="blue", game=game
            )

        if args.role == "defender" or args.role == "all":
            state_machines[1] = DefenderStateMachine(
                robot_id=1, team_color="blue", game=game
            )

        if args.role == "attacker" or args.role == "all":
            state_machines[2] = AttackerStateMachine(
                robot_id=2, team_color="blue", game=game
            )

        # Run test for specified duration
        start_time = time.time()
        while time.time() - start_time < args.duration:
            vision_data = game.get_vision_data()
            if vision_data:
                # Update robots
                for robot_id, state_machine in state_machines.items():
                    state_machine.update(vision_data)

                # Print states
                if args.verbose:
                    for robot_id, state_machine in state_machines.items():
                        print(f"Robot {robot_id} ({state_machine.role.value}):")
                        print(f"  State: {state_machine.current_state.value}")
                        print(f"  Target: {state_machine.target_position}")
                        print(f"  Current: {state_machine._get_current_pos()}")

                        # Get path for robot
                        path = game.path_planner.get_path(robot_id)
                        print(f"  Path: {path[:2]}{'...' if len(path) > 2 else ''}")
                        print()
                else:
                    # Simple state reporting
                    states = [
                        f"{robot_id}:{state_machine.current_state.value}"
                        for robot_id, state_machine in state_machines.items()
                    ]
                    print(f"States: {', '.join(states)}")

            time.sleep(0.1)

    finally:
        game.stop()


if __name__ == "__main__":
    main()
