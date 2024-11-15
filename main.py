import json
import threading
import time
from GameController.GameController import GameController
from VisionClient.Vision import Vision
from SimulationGrSim.RobotControlClient_threaded import (
    ThreadedRobotControlClient,
)


class Game:
    def __init__(self) -> None:
        # Load Config
        self.config = get_config()

        print("Initializing Game...")

        # Initialize robot control clients
        self.blue_robots = ThreadedRobotControlClient(
            team_port=10301
        )  # Blue team port
        self.yellow_robots = ThreadedRobotControlClient(
            team_port=10302
        )  # Yellow team port
        print("Robot control clients initialized")

        # Input Layer
        self.vision = Vision(self)  # Vision system
        self.referee = GameController(self)  # Game controller/referee
        print("Vision and referee systems initialized")

        # State tracking
        self.running = False
        self._update_thread = None
        self._fps = 60
        self.last_vision_data = None
        self.last_referee_data = None

    def start(self):
        print("Starting game systems...")

        # Start robot control clients
        self.blue_robots.start()
        self.yellow_robots.start()
        print("Robot control clients started")

        # Start vision and referee
        self.vision.start()
        self.referee.start()
        print("Vision and referee systems started")

        self.running = True
        self._update_thread = threading.Thread(target=self.update_loop)
        self._update_thread.daemon = True
        self._update_thread.start()
        print("Main update loop started")

    def update_loop(self):
        """Main update loop running at specified FPS"""
        while self.running:
            try:
                if self.vision.new_data:
                    t_start = time.time()

                    # Update vision data
                    self.last_vision_data = self.vision.get_last_frame()

                    # Update referee data
                    self.last_referee_data = self.referee.get_state()

                    # Handle robot control based on referee state
                    if self.referee.raw_referee["command"] == "HALT":
                        # Stop all robots when halted
                        self.blue_robots.send_global_velocity(0, 0, 0, 0)
                        self.yellow_robots.send_global_velocity(0, 0, 0, 0)
                    else:
                        # Here you would implement your robot control logic
                        # Example: Make blue robot 0 move in a circle
                        self.blue_robots.move_robot_circle(
                            0, radius=1.0, angular_speed=1.0
                        )

                    t_end = time.time()
                    print(f"Update loop time: {(t_end - t_start)*1000:.1f}ms")

            except Exception as e:
                print(f"Error in update loop: {e}")

            time.sleep(1.0 / self._fps)

    def stop(self):
        """Stop all running threads and components"""
        print("Stopping all systems...")
        self.running = False

        # Stop robot control clients
        self.blue_robots.stop()
        self.yellow_robots.stop()

        # Stop vision and referee
        self.vision.stop()
        self.referee.stop()

        if self._update_thread:
            self._update_thread.join()

        print("All systems stopped")

    def get_vision_data(self):
        """Get latest vision data"""
        return self.last_vision_data

    def get_referee_data(self):
        """Get latest referee data"""
        return self.last_referee_data


def get_config(config_file=None):
    """Load configuration from file"""
    if config_file:
        config = json.loads(open(config_file, "r").read())
    else:
        config = json.loads(open("config.json", "r").read())
    return config


if __name__ == "__main__":
    game = Game()
    try:
        game.start()

        # Print some status info periodically
        while True:
            time.sleep(1)
            vision_data = game.get_vision_data()
            referee_data = game.get_referee_data()

            print("\n=== Status Update ===")
            if vision_data:
                ball_pos = vision_data["ball"]
                print(
                    f"Ball position: x={ball_pos['x']:.2f}, "
                    f"y={ball_pos['y']:.2f}"
                )

            if referee_data:
                print(f"Referee command: {referee_data['command']}")
                print(
                    (
                        f"Score - Blue: {referee_data['blue']['score']} "
                        f"Yellow: {referee_data['yellow']['score']}"
                    )
                )

    except KeyboardInterrupt:
        print("\nShutting down...")
        game.stop()
