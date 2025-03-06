# test_integration.py
import time
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from main import Game
from PyQt.ssl_client import SSLClientWindow


def main():
    # Create Qt application
    app = QApplication(sys.argv)

    # Create and start game instance
    game = Game()

    # Initialize robot state machines
    game._initialize_robot_state_machines()

    # Create and show UI
    window = SSLClientWindow(game)
    window.show()

    # Start game systems
    game.start()

    # Timer for update logic
    update_timer = QTimer()

    def update_robots():
        # Get vision data
        vision_data = game.get_vision_data()
        if not vision_data:
            return

        # Update robot state machines
        for robot_id, state_machine in game.robot_state_machines.items():
            state_machine.update(vision_data)

    update_timer.timeout.connect(update_robots)
    update_timer.start(16)  # ~60Hz update rate

    try:
        sys.exit(app.exec_())
    finally:
        game.stop()


if __name__ == "__main__":
    main()