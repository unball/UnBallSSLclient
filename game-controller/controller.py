import socket
import threading
import time
from typing import Optional, Tuple
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor

# Assuming these are implemented elsewhere or imported from other modules
from referee_base import RefereeBase
from referee_parser import RefereeParser
from shared_data import SharedData, SharedOptional
from global_timer import GlobalTimer
from utils import get_network_interface
from protobuf_utils import from_byte_array
from robocup_ssl_referee import RoboCupSSL_Referee

# Simulating Qt's Signal mechanism
class Signal:
    def __init__(self):
        self.callbacks = []
    
    def connect(self, callback):
        self.callbacks.append(callback)
    
    def emit(self, *args, **kwargs):
        for callback in self.callbacks:
            callback(*args, **kwargs)

class Parameters:
    class Arg:
        def __init__(self, default_value):
            self.value = default_value
            self._updated = False

        def set(self, new_value):
            self.value = new_value
            self._updated = True

        def updated(self):
            return self._updated

@dataclass
class Args:
    ip: Parameters.Arg = Parameters.Arg("224.5.23.1")
    port: Parameters.Arg = Parameters.Arg(10003)

class Shared:
    def __init__(self):
        self.packet = SharedOptional()

class RefereeWorld(RefereeBase):
    def __init__(self, thread_pool: ThreadPoolExecutor):
        super().__init__(thread_pool)
        self.args = Args()
        self.shared = Shared()
        self.parser = RefereeParser(self.frame, self.field, self.has_is_yellow)
        self.socket = None
        self.sequence_number = 0
        self.packet: Optional[Tuple[int, RoboCupSSL_Referee]] = None
        self.send_referee_signal = Signal()

    def update(self):
        if self.args.ip.updated() or self.args.port.updated():
            self.setup_client()
            time.sleep(0.0006)  # Equivalent to QThread::usleep(600)
            self.setup_client()
        
        self.packet = self.shared.packet.extract()

    def exec(self):
        if not self.packet:
            return

        parsed_referee = self.parser.parse(self.packet[1])
        parsed_referee.sequence_number = self.sequence_number
        parsed_referee.timestamp = GlobalTimer.nsecs_elapsed()
        
        self.send_referee_signal.emit(parsed_referee)
        self.sequence_number += 1
        self.packet = None

    def setup_client(self):
        def setup():
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.socket.bind(('', self.args.port.value))
                print(f"RefereeWorld::socket connected on port {self.args.port.value}")
            except Exception as e:
                print(f"Error binding to port: {e}")
                return

            try:
                self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                       socket.inet_aton(self.args.ip.value) + socket.inet_aton('0.0.0.0'))
                print(f"RefereeWorld::socket connected with ip {self.args.ip.value} "
                      f"and interface {get_network_interface()}")
            except Exception as e:
                print(f"Error joining multicast group: {e}")

            self.socket.setblocking(False)
            threading.Thread(target=self.receive_datagrams, daemon=True).start()

        self.thread_pool.submit(setup)

    def receive_datagrams(self):
        while True:
            try:
                data, _ = self.socket.recvfrom(4096)
                packet = from_byte_array(RoboCupSSL_Referee, data)
                if packet:
                    received = (GlobalTimer.nsecs_elapsed(), packet)
                    self.shared.packet.set(received)
                    self.run_in_parallel()
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"Error receiving datagram: {e}")
            time.sleep(0.001)  # Small sleep to prevent busy-waiting

    def run_in_parallel(self):
        self.thread_pool.submit(self.exec)

# Equivalent to static_block in C++
def register_referee_world():
    # Assuming Factory is a global object or imported
    Factory.referee.insert(RefereeWorld)

register_referee_world()