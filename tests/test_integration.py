# test_integration.py
import sys
import time
import os

# Adjust the path to import from the root of your project
# This assumes test_integration.py is in a 'tests/system' or similar directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "..")
)  # Adjust '..' based on your structure
sys.path.append(PROJECT_ROOT)

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

# Assuming your main Game class is in 'main.py' or a similar top-level script
# If it's elsewhere, adjust the import
try:
    from main import Game  # Or from your_project_core.game import Game
except ImportError:
    print("ERROR: Could not import 'Game' class. Make sure it's accessible.")
    print(f"PROJECT_ROOT: {PROJECT_ROOT}")
    print(f"sys.path: {sys.path}")
    sys.exit(1)

from PyQt.ssl_client import SSLClientWindow
from RobotBehavior.robot_states import (
    RobotState,
    RobotRole,
)  # For type checking if needed

# Configuration for the test
TEST_DURATION_SECONDS = 60  # How long the test should run automatically
AUTO_TEST_COMMAND_INTERVAL = 10  # Seconds between automated game state changes
INITIAL_TEAM_COLOR = "blue"
INITIAL_DIVISION = "Entry Level"  # Or "Division B", "Division A"

# --- Global variable for Game instance ---
# This is often discouraged, but for a single test script like this, it can simplify things
# if you need to access `game` from multiple functions or atexit.
# Alternatively, pass `game` around.
game_instance = None


def run_automated_test_sequence(game: Game, window: SSLClientWindow):
    """
    Runs a sequence of automated commands to test various game states.
    """
    print("\n--- STARTING AUTOMATED TEST SEQUENCE ---")
    start_time = time.time()
    last_command_time = start_time
    command_index = 0

    # Example sequence of game states to test
    # Values should match what your GameController/Referee sends or what set_game_state expects
    test_commands = [
        "STOP",
        "NORMAL_START",
        f"PREPARE_KICKOFF_{INITIAL_TEAM_COLOR.upper()}",
        f"KICK_OFF_{INITIAL_TEAM_COLOR.upper()}",
        "NORMAL_START",
        f"DIRECT_FREE_{'YELLOW' if INITIAL_TEAM_COLOR == 'blue' else 'BLUE'}",  # Opponent's free kick
        "NORMAL_START",
        "HALT",
        "STOP",
        f"BALL_PLACEMENT_{INITIAL_TEAM_COLOR.upper()}",
        "NORMAL_START",
    ]
    # Add more states: PENALTY, CORNER_KICK, GOAL_KICK, etc.

    try:
        while time.time() - start_time < TEST_DURATION_SECONDS:
            current_time = time.time()

            # --- Main Game Update Logic (simulates the core loop if not driven by UI timer directly) ---
            # In a real application, this might be part of the Game class's own loop or UI's timer
            vision_data = game.get_vision_data()
            referee_data = (
                game.get_referee_data()
            )  # Important to process referee commands

            if vision_data and game.robot_state_machines:  # Ensure SMs are initialized
                for robot_id, sm in game.robot_state_machines.items():
                    sm.update(
                        vision_data
                    )  # Pass vision_data (SMs will get ref_data from game)

            # Update UI (this is already handled by SSLClientWindow's QTimer)
            # window.update_display() # No need to call directly if UI has its own timer

            QApplication.processEvents()  # Keep UI responsive

            # --- Automated Command Logic ---
            if current_time - last_command_time > AUTO_TEST_COMMAND_INTERVAL:
                if command_index < len(test_commands):
                    command_to_send = test_commands[command_index]
                    print(
                        f"\nINTEGRATION TEST: Sending command -> {command_to_send} (Time: {current_time - start_time:.1f}s)"
                    )
                    game.set_game_state(
                        command_to_send
                    )  # Assumes game.set_game_state handles it

                    # Example: Switch team color mid-test
                    if command_index == len(test_commands) // 2:  # Halfway through
                        new_color = (
                            "yellow"
                            if game.config["match"]["team_color"] == "blue"
                            else "blue"
                        )
                        print(f"INTEGRATION TEST: Switching team color to {new_color}")
                        game.switch_team_color(new_color)
                        # Update UI team selector if possible (or UI should observe game.config)
                        if new_color == "yellow":
                            window.team_select.setCurrentIndex(
                                1
                            )  # Assuming 0=Blue, 1=Yellow
                        else:
                            window.team_select.setCurrentIndex(0)

                    command_index += 1
                    last_command_time = current_time
                else:
                    print("INTEGRATION TEST: Automated command sequence finished.")
                    break  # End the automated sequence

            time.sleep(0.016)  # ~60Hz, adjust as needed for your system's performance

    except KeyboardInterrupt:
        print("INTEGRATION TEST: Interrupted by user.")
    finally:
        print("--- AUTOMATED TEST SEQUENCE FINISHED ---")


def main():
    global game_instance
    app = QApplication(sys.argv)

    print("--- INITIALIZING INTEGRATION TEST ---")

    # 1. Initialize the Game
    # This should set up Vision, Controllers, PathPlanner, GameController
    try:
        game_instance = Game()  # Your main Game class
        print("Game instance created.")
    except Exception as e:
        print(f"FATAL: Could not create Game instance: {e}")
        import traceback

        traceback.print_exc()
        return

    # Apply initial test configurations to the game instance
    game_instance.config["match"]["team_color"] = INITIAL_TEAM_COLOR
    # If your Game class doesn't automatically pick up division for field_bounds, set it:
    # game_instance.set_division(INITIAL_DIVISION) # Assuming Game class has this

    # 2. Start Game Systems (Vision, Robot Control, Path Planning, GC)
    # This should be handled by game_instance.start()
    try:
        print("Starting game systems...")
        game_instance.start()  # This should also call _initialize_robot_state_machines
        print("Game systems started.")

        # Brief pause to allow systems (especially vision) to connect and get first data
        print("Waiting for initial data acquisition (e.g., vision)...")
        time.sleep(2)  # Adjust as needed

        # Verify state machines are initialized
        if not game_instance.robot_state_machines:
            print(
                "WARNING: Robot state machines not initialized after game.start(). Forcing init."
            )
            game_instance._initialize_robot_state_machines()
        if not game_instance.robot_state_machines:
            print("FATAL: Robot state machines are still empty. Aborting.")
            game_instance.stop()
            return
        else:
            print(
                f"Initialized {len(game_instance.robot_state_machines)} robot state machines."
            )

    except Exception as e:
        print(f"FATAL: Error during game system startup: {e}")
        import traceback

        traceback.print_exc()
        if game_instance and hasattr(game_instance, "stop"):
            game_instance.stop()
        return

    # 3. Create and Show UI
    try:
        print("Creating UI window...")
        window = SSLClientWindow(game=game_instance)
        window.show()
        print("UI window shown.")

        # Set initial UI elements based on test config
        window.division_combo.setCurrentText(INITIAL_DIVISION)  # Triggers field redraw
        if INITIAL_TEAM_COLOR == "blue":
            window.team_select.setCurrentIndex(0)  # Assuming 0 is Blue
        else:
            window.team_select.setCurrentIndex(1)  # Assuming 1 is Yellow
        window.update_team_display(INITIAL_TEAM_COLOR)

    except Exception as e:
        print(f"FATAL: Error creating UI: {e}")
        import traceback

        traceback.print_exc()
        game_instance.stop()
        return

    # 4. Run Automated Test Sequence (optional) or just let user interact
    # If you want user interaction primarily, you can comment out run_automated_test_sequence
    # The UI's own QTimer will call window.update_display() which updates visualization
    # and robot status labels based on game_instance.

    # Option A: Run automated tests
    # run_automated_test_sequence(game_instance, window)

    # Option B: Let the UI run and user interacts manually
    # The UI's QTimer will keep things updated.
    print(
        f"\n--- INTEGRATION TEST RUNNING for {TEST_DURATION_SECONDS}s (or Ctrl+C) ---"
    )
    print("Interact with the UI or observe automated sequence if enabled.")
    print(f"Initial Team: {INITIAL_TEAM_COLOR}, Division: {INITIAL_DIVISION}")

    # --- PyQt Application Event Loop ---
    # This will block until the UI is closed or app.exit() is called.
    # For a timed test without full UI interaction loop, you'd manage time differently.
    # However, since SSLClientWindow has its own QTimer for updates, starting the app loop is correct.

    exit_code = 0
    try:
        # To make the test run for a fixed duration even with the UI event loop:
        # We can use another QTimer to call app.quit after TEST_DURATION_SECONDS
        # if run_automated_test_sequence is NOT used or completes early.

        # If you want the test to end automatically after TEST_DURATION_SECONDS:
        # QTimer.singleShot(TEST_DURATION_SECONDS * 1000, app.quit)
        # print(f"Test will automatically close in {TEST_DURATION_SECONDS} seconds.")

        # Start the automated sequence in a QTimer to allow UI to fully show first
        QTimer.singleShot(
            1000, lambda: run_automated_test_sequence(game_instance, window)
        )

        exit_code = app.exec_()
    except Exception as e:
        print(f"Error during app execution: {e}")
        import traceback

        traceback.print_exc()
        exit_code = 1
    finally:
        print("\n--- INTEGRATION TEST SHUTDOWN ---")
        if game_instance:
            print("Stopping game systems...")
            game_instance.stop()
            print("Game systems stopped.")
        print(f"Test finished with exit code: {exit_code}")
        sys.exit(exit_code)


if __name__ == "__main__":
    # Ensure DEBUG_ROBOT_BEHAVIOR is True for detailed logs from state machines
    # You might need to set this in robot_state_machine.py directly or through a config
    # Forcing it here for test purposes if possible (depends on how it's defined)
    try:
        import RobotBehavior.robot_state_machine

        RobotBehavior.robot_state_machine.DEBUG_ROBOT_BEHAVIOR = True
        print("DEBUG_ROBOT_BEHAVIOR set to True for this test run.")
    except Exception as e:
        print(f"Could not set DEBUG_ROBOT_BEHAVIOR: {e}")

    main()
