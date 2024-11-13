import time
import socket
from VisionClient.Vision import Vision
from protocols.grsim.grSim_Packet_pb2 import grSim_Packet


class GrSimClient:
    """Client to send commands to grSim"""

    def __init__(self, address: str = "127.0.0.1", port: int = 20011):
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
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False


def main():
    # Create config object needed by Vision class
    class Config:
        def __init__(self):
            self.config = {
                "network": {"vision_port": 10006, "multicast_ip": "224.5.23.2"},
                "match": {"team_side": "left"},
            }

    # Create Vision client
    vision = Vision(Config())
    vision.start()

    # Create simulator client
    sim = GrSimClient("127.0.0.1", 20011)

    try:
        while True:
            # Get latest frame data
            frame = vision.get_last_frame()

            # Print vision data
            vision.print_formatted_vision_data()

            # Control robots based on their position
            for robot_id, robot_data in frame["robotsBlue"].items():
                if robot_data["x"] is not None:  # Check if robot is detected
                    if robot_data["x"] <= 0:
                        sim.send_command(robot_id, False, veltangent=1.0)
                    else:
                        sim.send_command(robot_id, False, veltangent=-1.0)

            time.sleep(0.016)  # ~60Hz update rate

    except KeyboardInterrupt:
        print("Shutting down...")
        vision.stop()
        vision.join()


if __name__ == "__main__":
    main()
