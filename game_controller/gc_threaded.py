# Version 1: Threaded Game Controller
import json
import socket
import struct
import threading
from google.protobuf.json_format import MessageToJson
from protocols.game_controller.ssl_gc_referee_message_pb2 import Referee


class ThreadedGameController(threading.Thread):
    """Threaded implementation of SSL Game Controller referee interface"""

    def __init__(self, multicast_ip="224.5.23.1", port=10003):
        super().__init__()
        self.referee_port = port
        self.host = multicast_ip
        self.running = False
        self.daemon = True
        self._referee_message = {}

    def run(self):
        """Main thread loop that receives and processes referee messages"""

        self.referee_sock = self._create_socket()
        self.running = True
        print("Threaded GC started")

        while self.running:
            referee = Referee()
            try:
                data = self.referee_sock.recv(1024)
                referee.ParseFromString(data)
                self._referee_message = json.loads(MessageToJson(referee))
            except Exception as e:
                print(f"Error processing referee message: {e}")

        self.stop()

    def stop(self):
        """Stop the game controller thread"""
        self.running = False
        if hasattr(self, "referee_sock"):
            self.referee_sock.close()
        print("GC module stopped!")

    def get_state(self):
        """Get current game state"""
        if not self._referee_message:
            return {
                "command": "",
                "team": "",
                "position": None,
                "stage": "",
                "can_play": False,
            }

        return {
            "command": self.get_command(),
            "team": self.get_team(),
            "position": self.get_designated_position(),
            "stage": self.get_stage(),
            "can_play": self.can_play(),
        }

    def can_play(self):
        """Check if game is in playable state"""
        if not self._referee_message:
            return False
        is_halted = self._referee_message.get("command") == "HALT"
        is_stopped = self._referee_message.get("command") == "STOP"
        return not (is_halted or is_stopped)

    def get_command(self):
        """Get current referee command"""
        return self._referee_message.get("command", "")

    def get_team(self):
        """Get current team color"""
        return (
            "blue"
            if self._referee_message.get("command", "").endswith("BLUE")
            else "yellow"
        )

    def get_designated_position(self):
        """Get designated ball position if available"""
        if pos := self._referee_message.get("designatedPosition", None):
            return (pos["x"], pos["y"])
        return None

    def get_stage(self):
        """Get current game stage"""
        return self._referee_message.get("stage", "")

    def _create_socket(self):
        """Create and configure multicast UDP socket"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.referee_port))
        mreq = struct.pack("4sl", socket.inet_aton(self.host), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return sock
