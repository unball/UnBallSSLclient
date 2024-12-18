import json
import socket
import struct
import threading
import time
import os
from google.protobuf.json_format import MessageToJson
from protocols.GameController.ssl_gc_referee_message_pb2 import Referee

class GameController(threading.Thread):
    def __init__(self, game) -> None:
        super(GameController, self).__init__()

        self.game = game
        self.config = self.game.config
        self.daemon = True
        self.running = False
        self._fps = 60
        self.new_data = False
        self.any_referee = False

        # Initialize referee data structure
        self.raw_referee = {
            "command": "",
            "team": "",
            "position": None,
            "stage": "",
            "can_play": False,
            "meta": {
                "has_designated_position": False,
                "cameras": {i: {"last_capture": -1} for i in range(0, 4)},
            },
            "yellow": {
                "name": "",
                "score": 0,
                "red_cards": 0,
                "yellow_cards": 0,
                "timeouts": 0,
                "timeout_time": 0,
                "goalkeeper": 0,
            },
            "blue": {
                "name": "",
                "score": 0,
                "red_cards": 0,
                "yellow_cards": 0,
                "timeouts": 0,
                "timeout_time": 0,
                "goalkeeper": 0,
            },
        }

        self.referee_port = self.config["network"]["referee_port"]
        self.host = self.config["network"]["referee_ip"]
        self.referee_sock = None

    def run(self):
        """Main thread loop to receive and process referee data"""
        print("Starting referee module...")
        print(
            f"Creating socket with address: {self.host} and port: {self.referee_port}"
        )

        self.referee_sock = self._create_socket()
        self._wait_to_connect()

        self.running = True
        print("Referee module started!")

        while self.running:
            referee = Referee()
            try:
                data = self.referee_sock.recv(2048)
                referee.ParseFromString(data)
                last_frame = json.loads(MessageToJson(referee))
                self.new_data = self.update_referee(last_frame)
            except Exception as e:
                print(f"Error processing referee message: {e}")
        self.stop()

    def stop(self):
        """Stop the game controller thread"""
        self.running = False
        if self.referee_sock:
            self.referee_sock.close()
            self.referee_sock = None
        print("Referee module stopped!")

    def update_referee(self, last_frame):
        """Update referee data from new frame"""
        if not last_frame:
            return False

        self.any_referee = True

        # Update command and stage
        self.raw_referee["command"] = last_frame.get("command", "")
        self.raw_referee["stage"] = last_frame.get("stage", "")

        # Update team info
        if yellow := last_frame.get("yellow"):
            self.update_team_info(yellow, "yellow")
        if blue := last_frame.get("blue"):
            self.update_team_info(blue, "blue")

        # Update designated position
        if pos := last_frame.get("designatedPosition"):
            self.raw_referee["position"] = (pos.get("x"), pos.get("y"))
            self.raw_referee["meta"]["has_designated_position"] = True
        else:
            self.raw_referee["position"] = None
            self.raw_referee["meta"]["has_designated_position"] = False

        # Update play state
        self.raw_referee["can_play"] = not (
            self.raw_referee["command"] in ["HALT", "STOP"]
        )

        return True

    def update_team_info(self, team_data, color):
        """Update team information"""
        self.raw_referee[color].update(
            {
                "name": team_data.get("name", ""),
                "score": team_data.get("score", 0),
                "red_cards": team_data.get("redCards", 0),
                "yellow_cards": team_data.get("yellowCards", 0),
                "timeouts": team_data.get("timeouts", 0),
                "timeout_time": team_data.get("timeoutTime", 0),
                "goalkeeper": team_data.get("goalkeeper", 0),
            }
        )

    def get_state(self):
        """Get current referee state"""
        self.new_data = False
        return self.raw_referee

    def _wait_to_connect(self):
        """Wait for initial connection"""
        if self.referee_sock:
            self.referee_sock.recv(1024)

    def _create_socket(self):
        """Create and configure multicast UDP socket"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Changed binding to empty string to match Vision module
        sock.bind(("", self.referee_port))
        mreq = struct.pack("4sl", socket.inet_aton(self.host), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return sock

    def print_formatted_referee_data(self):
        """Print referee data in formatted output"""
        referee_state = self.get_state()

        print(
            "-----Received Referee Packet---------------------------------------------"
        )
        print("-[Referee Data]-------")

        # Print command and stage
        print(f"Command: {referee_state['command']}")
        print(f"Stage: {referee_state['stage']}")
        print(f"Can Play: {referee_state['can_play']}")

        # Print designated position if available
        if referee_state["position"]:
            x, y = referee_state["position"]
            print(f"Designated Position: <{x:9.2f},{y:9.2f}>")

        # Print yellow team info
        yellow = referee_state["yellow"]
        print("\n-[Yellow Team]-------")
        print(f"Name: {yellow['name']}")
        print(f"Score: {yellow['score']}")
        print(f"Red Cards: {yellow['red_cards']}")
        print(f"Yellow Cards: {yellow['yellow_cards']}")
        print(f"Timeouts: {yellow['timeouts']}")
        print(f"Timeout Time: {yellow['timeout_time']}")
        print(f"Goalkeeper: {yellow['goalkeeper']}")

        # Print blue team info
        blue = referee_state["blue"]
        print("\n-[Blue Team]-------")
        print(f"Name: {blue['name']}")
        print(f"Score: {blue['score']}")
        print(f"Red Cards: {blue['red_cards']}")
        print(f"Yellow Cards: {blue['yellow_cards']}")
        print(f"Timeouts: {blue['timeouts']}")
        print(f"Timeout Time: {blue['timeout_time']}")
        print(f"Goalkeeper: {blue['goalkeeper']}")


def main():
    # Create test environment
    class Game:
        def __init__(self):
            self.config = {
                "network": {
                    "referee_port": 10003,
                    "referee_ip": "224.5.23.1",
                }
            }

    # Create and start game controller
    game = Game()
    game_controller = GameController(game)
    game_controller.start()

    try:
        while True:
            # Clear screen for better readability
            os.system("cls" if os.name == "nt" else "clear")

            # Use the formatted print instead of raw state
            game_controller.print_formatted_referee_data()

            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        game_controller.stop()


if __name__ == "__main__":
    main()
