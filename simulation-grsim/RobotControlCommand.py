# flake8: noqa

import socket
import struct
import time
from typing import Optional, Tuple, List
from dataclasses import dataclass
from proto.ssl_vision_wrapper_pb2 import SSL_WrapperPacket
from proto.grSim_Packet_pb2 import grSim_Packet

def print_robot_info(robot) -> None:
    """Print detailed information about a robot
    
    Args:
        robot: SSL_DetectionRobot instance
    """
    print(f"CONF={robot.confidence:4.2f} ", end="")
    
    if robot.HasField('robot_id'):
        print(f"ID={robot.robot_id:3d} ", end="")
    else:
        print("ID=N/A ", end="")
    
    print(f" HEIGHT={robot.height:6.2f} POS=<{robot.x:9.2f},{robot.y:9.2f}> ", end="")
    
    if robot.HasField('orientation'):
        print(f"ANGLE={robot.orientation:6.3f} ", end="")
    else:
        print("ANGLE=N/A    ", end="")
    
    print(f"RAW=<{robot.pixel_x:8.2f},{robot.pixel_y:8.2f}>")

@dataclass
class DetectionBall:
    confidence: float
    x: float
    y: float
    z: Optional[float]
    pixel_x: float
    pixel_y: float

@dataclass
class DetectionRobot:
    confidence: float
    robot_id: Optional[int]
    x: float
    y: float
    orientation: Optional[float]
    pixel_x: float
    pixel_y: float
    height: Optional[float]

class SSLVisionClient:
    """Client to receive SSL vision data"""
    
    def __init__(self, port: int = 10020, address: str = "224.5.23.4", interface: str = ""):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to the port
        self.socket.bind(('', port))
        
        # Setup multicast
        mreq = struct.pack("4s4s", socket.inet_aton(address),
                          socket.inet_aton(interface) if interface else struct.pack("4s", b'\0' * 4))
        self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        self.packet = SSL_WrapperPacket()
    
    def receive(self) -> bool:
        """Receive and parse vision packet"""
        try:
            data = self.socket.recv(2048)
            self.packet.ParseFromString(data)
            return True
        except Exception as e:
            print(f"Error receiving vision data: {e}")
            return False
    
    def get_field_data(self) -> Tuple[int, int, int, int]:
        """Get field dimensions if geometry packet is available"""
        if self.packet.HasField('geometry'):
            field = self.packet.geometry.field
            

            
            return (field.field_length, field.field_width,
                   field.goal_width, field.goal_depth)
        return (0, 0, 0, 0)
    
    def print_all_robot_info(self) -> None:
        """Print information about all detected robots"""
        if self.packet.HasField('detection'):
            detection = self.packet.detection
            
            # Print blue robots
            for i, robot in enumerate(detection.robots_blue):
                print(f"-Robot(B) ({i+1:2d}/{len(detection.robots_blue):2d}): ", end="")
                print_robot_info(robot)
            
            # Print yellow robots
            for i, robot in enumerate(detection.robots_yellow):
                print(f"-Robot(Y) ({i+1:2d}/{len(detection.robots_yellow):2d}): ", end="")
                print_robot_info(robot)

class GrSimClient:
    """Client to send commands to grSim"""
    
    def __init__(self, address: str = "127.0.0.1", port: int = 20011):
        self.address = address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def send_command(self, robot_id: int, yellow: bool,
                    veltangent: float = 0, velnormal: float = 0, velangular: float = 0,
                    kickspeedx: float = 0, kickspeedz: float = 0,
                    spinner: bool = False) -> bool:
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
    # Create vision client
    vision = SSLVisionClient(10020, "224.5.23.4")
    
    
    # Create simulator client
    sim = GrSimClient("127.0.0.1", 20011)
    
    
    try:
        while True:
            if vision.receive():
                print("-----Received Wrapper Packet---------------------------------------------")
                
                if vision.packet.HasField('detection'):
                    t_now = time.time()  # Current time in seconds
                    detection = vision.packet.detection
                    
                    print("-[Detection Data]-------")
                    print(f"Camera ID={detection.camera_id} FRAME={detection.frame_number} T_CAPTURE={detection.t_capture:.4f}")
                    print(f"SSL-Vision Processing Latency                   {(detection.t_sent - detection.t_capture) * 1000.0:7.3f}ms")
                    print(f"Network Latency (assuming synched system clock) {(t_now - detection.t_sent) * 1000.0:7.3f}ms")
                    print(f"Total Latency   (assuming synched system clock) {(t_now - detection.t_capture) * 1000.0:7.3f}ms")
                    
                    # Print robot information
                    vision.print_all_robot_info()
                
                # Control robots based on their position
                if vision.packet.HasField('detection'):
                    for robot in vision.packet.detection.robots_blue:
                        if robot.HasField('robot_id'):
                            if robot.x <= 0:
                                sim.send_command(robot.robot_id, False, veltangent=1.0)
                            else:
                                sim.send_command(robot.robot_id, False, veltangent=-1.0)
            
            time.sleep(0)  
            #time.sleep(0.016) # ~60Hz update rate
            
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == "__main__":
    main()