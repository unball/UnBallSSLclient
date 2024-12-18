import json
import threading
import time
import os
from GameController.GameController import GameController
from VisionClient.Vision import Vision
from SimulationGrSim.RobotControlClient_threaded import ThreadedRobotControlClient


class Game:
    def __init__(self) -> None:
        # Load Config
        self.config = get_config()

        self.debug = {
            "vision": False,
            "referee": False,
            "threads": False,
            "timing": False,
            "all": False,  # Master debug flag
        }

        print("Initializing Game...")

        # Initialize robot control clients
        self.blue_robots = ThreadedRobotControlClient(team_port=10301)
        self.yellow_robots = ThreadedRobotControlClient(team_port=10302)
        print("Robot control clients initialized")

        # Input Layer
        self.vision = Vision(self)
        self.referee = GameController(self)
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
                # Update vision data if available
                if self.vision.new_data:
                    self.last_vision_data = self.vision.get_last_frame()

                # Update referee data independently
                if self.referee.new_data or self.referee.any_referee:
                    self.last_referee_data = self.referee.get_state()

                # Handle robot control based on referee state
                if (
                    self.last_referee_data
                    and self.last_referee_data["command"] == "HALT"
                ):
                    # Stop all robots when halted
                    self.blue_robots.send_global_velocity(0, 0, 0, 0)
                    self.yellow_robots.send_global_velocity(0, 0, 0, 0)
                elif (
                    self.last_vision_data
                ):  # Only control robots if we have vision data
                    # Robot control logic here
                    pass

                time.sleep(1.0 / self._fps)

            except Exception as e:
                print(f"Error in update loop: {str(e)}")

    def stop(self):
        """Stop all running threads and components"""
        print("Stopping all systems...")
        self.running = False

        # Stop robot control clients
        if hasattr(self, "blue_robots"):
            self.blue_robots.stop()
        if hasattr(self, "yellow_robots"):
            self.yellow_robots.stop()

        # Stop vision and referee
        if hasattr(self, "vision"):
            self.vision.stop()
        if hasattr(self, "referee"):
            self.referee.stop()

        # Wait for update thread
        if self._update_thread and self._update_thread.is_alive():
            try:
                self._update_thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error joining update thread: {e}")

        print("All systems stopped")

    def get_vision_data(self):
        """Get latest vision data"""
        return self.last_vision_data

    def get_referee_data(self):
        """Get latest referee data"""
        return self.last_referee_data

    def get_unball_data_vision(self):
        """Get vision data specific to UnBall team's robots"""
        if not self.last_vision_data:
            return None

        # Determine which team color UnBall is using from config
        is_blue = self.config["match"]["team_color"] == "blue"
        robot_data = (
            self.last_vision_data["robotsBlue"]
            if is_blue
            else self.last_vision_data["robotsYellow"]
        )

        # Filter for robots that are actually detected on field
        active_robots = {
            robot_id: {
                "position": {"x": data["x"], "y": data["y"]},
                "orientation": data["theta"],
                "timestamp": data["tCapture"],
                "camera_id": data["cCapture"],
            }
            for robot_id, data in robot_data.items()
            if data["x"] is not None  # Only include detected robots
        }

        return {
            "team_color": "blue" if is_blue else "yellow",
            "robots": active_robots,
            "total_robots": len(active_robots),
            "ball_position": (
                {
                    "x": self.last_vision_data["ball"]["x"],
                    "y": self.last_vision_data["ball"]["y"],
                }
                if self.last_vision_data["ball"]
                else None
            ),
        }

    def get_unball_data_referee(self):
        """Get referee data specific to UnBall team"""
        if not self.last_referee_data:
            return None

        team_color = self.config["match"]["team_color"]
        team_data = self.last_referee_data[team_color]

        return {
            "side": team_color,
            "score": team_data["score"],
            "red_cards": team_data["red_cards"],
            "yellow_cards": team_data["yellow_cards"],
            "timeouts": team_data["timeouts"],
            "timeout_time": team_data["timeout_time"],
            "goalkeeper": team_data["goalkeeper"],
        }


def get_config(config_file=None):
    """Load configuration from file"""
    try:
        if config_file:
            with open(config_file, "r") as f:
                config = json.load(f)
        else:
            with open("config.json", "r") as f:
                config = json.load(f)

        # Validate required fields
        required_fields = [
            ("network", "multicast_ip"),
            ("network", "vision_port"),
            ("network", "referee_ip"),
            ("network", "referee_port"),
            ("match", "team_side"),
            ("match", "team_color"),
        ]

        for section, field in required_fields:
            if section not in config or field not in config[section]:
                raise ValueError(f"Missing required config field: {section}.{field}")

        return config

    except FileNotFoundError:
        raise FileNotFoundError("Config file not found")
    except json.JSONDecodeError:
        raise ValueError("Invalid JSON in config file")


if __name__ == "__main__":
    game = None

    # Set debug flags as needed
    # game.debug["referee"] = False  # Enable only referee debugging
    # game.debug["all"] = True    # Enable all debugging

    # try:
    # game.start()
    # game.test_referee_connection()

    # while True:
    #     vision_data = game.get_vision_data()
    #     referee_data = game.get_referee_data()

    #     # First clear screen
    #     os.system("cls" if os.name == "nt" else "clear")

    #     # Then print all status info with a small delay to ensure visibility
    #     print("\n=== Status Update ===")
    #     time.sleep(0.1)  # Small delay to ensure print is visible

    #     print(referee_data)

    #     if vision_data:
    #         ball_pos = vision_data["ball"]
    #         print(
    #             f"Ball position: x={ball_pos['x']:.2f}, " f"y={ball_pos['y']:.2f}"
    #         )

    #     if referee_data:
    #         print(f"Referee command: {referee_data['command']}")
    #         print(
    #             f"Score - Blue: {referee_data['blue']['score']} "
    #             f"Yellow: {referee_data['yellow']['score']}"
    #         )
    #     else:
    #         print("No referee data available")

    #     # Main loop delay
    #     time.sleep(0.9)  # Combined with the 0.1s above makes 1 second total

    try:
        game = Game()
        game.start()

        while True:
            vision_data = game.get_unball_data_vision()
            referee_data = game.get_unball_data_referee()

            os.system("cls" if os.name == "nt" else "clear")
            print("\n=== Status Update ===")

            if vision_data:
                print("\n=== UnBall Vision Status ===")
                print(f"Team Color: {vision_data['team_color']}")
                print(f"Active Robots: {vision_data['total_robots']}")
                print("\nRobot Positions:")
                for robot_id, data in vision_data["robots"].items():
                    pos = data["position"]
                    print(
                        f"Robot {robot_id}: x={pos['x']:.2f}, y={pos['y']:.2f}, θ={data['orientation']:.2f}"
                    )

                if vision_data["ball_position"]:
                    ball = vision_data["ball_position"]
                    print(f"\nBall Position: x={ball['x']:.2f}, y={ball['y']:.2f}")

            if referee_data:
                print("\n=== UnBall Team Status ===")
                print(f"Playing as: {referee_data['side']}")
                print(f"Score: {referee_data['score']}")
                print(f"Red cards: {referee_data['red_cards']}")
                print(f"Yellow cards: {referee_data['yellow_cards']}")
                print(f"Timeouts remaining: {referee_data['timeouts']}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Fatal error: {str(e)}")
        import traceback

        traceback.print_exc()
    finally:
        if game:
            game.stop()
