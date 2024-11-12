import socket
import os
import time
from proto.ssl_vision_wrapper_pb2 import SSL_WrapperPacket
from proto.grSim_Packet_pb2 import grSim_Packet
from Vision import Vision

import sys
import os

# Add debug prints to trace execution
print("1. Script starting")
print(f"2. Python path: {sys.path}")
print(f"3. Current directory: {os.getcwd()}")

# Add parent directory to Python path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
print(f"4. Added to path: {parent_dir}")

try:
    print("5. Attempting to import Vision")
    from VisionClient.Vision import Vision

    print("6. Vision import successful")
except ImportError as e:
    print(f"Error importing Vision: {e}")
    print("Make sure VisionClient directory exists and contains Vision.py")
    sys.exit(1)

try:
    print("7. Attempting to import proto files")
    from proto.ssl_vision_wrapper_pb2 import SSL_WrapperPacket
    from proto.grSim_Packet_pb2 import grSim_Packet

    print("8. Proto imports successful")
except ImportError as e:
    print(f"Error importing proto files: {e}")
    print("Make sure proto directory exists with required pb2 files")
    sys.exit(1)

print("9. Setting up GrSimClient")


class GrSimClient:
    """Client to send commands to grSim"""

    def __init__(self, address: str = "127.0.0.1", port: int = 20011):
        print(f"10. Initializing GrSimClient with {address}:{port}")
        self.address = address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(
        self,
        robot_id: int,
        yellow: bool,
        veltangent: float = 0,
        velnormal: float = 0,
        velangular: float = 0,
        kickspeedx: float = 0,
        kickspeedz: float = 0,
        spinner: bool = False,
    ) -> bool:
        """Send command to robot"""
        print(f"11. Sending command to robot {robot_id} (yellow={yellow})")
        packet = grSim_Packet()

        commands = packet.commands
        commands.timestamp = 0
        commands.isteamyellow = yellow

        robot_command = commands.robot_commands.add()
        robot_command.id = robot_id
        robot_command.kickspeedx = kickspeedx
        robot_command.kickspeedz = kickspeedz
        robot_command.veltangent = veltangent
        robot_command.velnormal = velnormal
        robot_command.velangular = velangular
        robot_command.spinner = spinner
        robot_command.wheelsspeed = False

        try:
            self.socket.sendto(packet.SerializeToString(), (self.address, self.port))
            print("12. Command sent successfully")
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False


def main():
    print("13. Starting main function")

    # Create config class
    class Config:
        def __init__(self):
            print("14. Initializing Config")
            self.config = {
                "network": {"vision_port": 10020, "multicast_ip": "224.5.23.4"},
                "match": {"team_side": "left"},
            }

    print("15. Creating Vision and GrSim clients")
    config = Config()
    vision_client = Vision(config)
    sim = GrSimClient("127.0.0.1", 20011)

    print("16. Setting up vision socket")
    vision_client.vision_sock = vision_client._create_socket()

    try:
        print("17. Entering main loop")
        while True:
            print("18. Receiving frame")
            env = SSL_WrapperPacket()
            data = vision_client.vision_sock.recv(2048)
            env.ParseFromString(data)

            print("19. Updating vision data")
            vision_client.new_data = vision_client.update_detection(env)
            vision_client.new_geometry = vision_client.update_geometry(env)

            print("20. Printing formatted data")
            vision_client.print_formatted_vision_data()

            print("21. Processing robot controls")
            frame = vision_client.get_last_frame()
            for robot_id, robot in frame["robotsBlue"].items():
                if robot["x"] is not None:
                    if robot["x"] <= 0:
                        sim.send_command(int(robot_id), False, veltangent=1.0)
                    else:
                        sim.send_command(int(robot_id), False, veltangent=-1.0)

            time.sleep(0)

    except KeyboardInterrupt:
        print("22. Shutting down...")
        vision_client.vision_sock.close()


if __name__ == "__main__":
    main()
