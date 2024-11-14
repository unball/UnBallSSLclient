import json
import socket
import struct
import logging
from google.protobuf.json_format import MessageToJson
from protocols.game_controller.ssl_gc_referee_message_pb2 import Referee


class SyncGameController:
    """Synchronous implementation of SSL Game Controller referee interface"""

    def __init__(self, multicast_ip="224.5.23.1", port=10003):
        self.referee_port = port
        self.host = multicast_ip
        self._referee_message = {}
        self.referee_sock = None

    def start(self):
        """Initialize the game controller"""
        self.referee_sock = self._create_socket()
        self.referee_sock.setblocking(False)
        print("Referee module started!")

    def update(self):
        """Receive and process new referee messages (non-blocking)"""
        if not self.referee_sock:
            return

        try:
            data = self.referee_sock.recv(1024)
            referee = Referee()
            referee.ParseFromString(data)
            self._referee_message = json.loads(MessageToJson(referee))
            return True
        except BlockingIOError:
            return False
        except Exception as e:
            print(f"Error processing referee message: {e}")
            return False

    def stop(self):
        """Stop the game controller"""
        if self.referee_sock:
            self.referee_sock.close()
            self.referee_sock = None
        print("Referee module stopped!")

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
