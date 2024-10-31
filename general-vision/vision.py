import socket
import struct
import json
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, Tuple
import math
from proto.ssl_vision_wrapper_pb2 import SSL_WrapperPacket

def format_robot_info(robot_data: Dict[str, Any]) -> str:
    """Format robot information into a readable string"""
    orientation_str = f"{robot_data['orientation']:.3f}" if robot_data['orientation'] is not None else "N/A"
    confidence_str = f"{robot_data['confidence']:.2f}" if robot_data['confidence'] is not None else "N/A"
    height_str = f"{robot_data['height']:.2f}" if robot_data['height'] is not None else "N/A"
    
    return (f"pos=({robot_data['x']:7.3f}, {robot_data['y']:7.3f}), "
            f"orientation={orientation_str:>7}, "
            f"confidence={confidence_str:>5}, "
            f"height={height_str:>6}")

def format_value(value, format_spec: str, default: str = "N/A") -> str:
    """Format a value with error handling"""
    if value is None:
        return default.rjust(len(format_spec.replace(':', '')))
    try:
        return f"{value:{format_spec}}"
    except (TypeError, ValueError):
        return default.rjust(len(format_spec.replace(':', '')))

@dataclass
class VisionFrame:
    """Data class for holding vision frame information"""
    frame_number: int
    t_capture: float
    t_sent: float
    camera_id: int
    ball: Optional[Dict[str, float]]
    robots_blue: Dict[int, Dict[str, Any]]
    robots_yellow: Dict[int, Dict[str, Any]]

class SSLVisionClient(threading.Thread):
    def __init__(self, config_file: str = 'config.json'):
        super().__init__()
        self.config = self._load_config(config_file)
        self._setup_variables()
        self._setup_socket()
        
    def _load_config(self, config_file: str) -> dict:
        with open(config_file, 'r') as f:
            return json.load(f)
            
    def _setup_variables(self):
        """Initialize internal variables"""
        self.frame = {}
        self.last_frame = {}
        self._frame_times = deque(maxlen=60)
        self._fps = 0
        self.running = True
        self.packet = SSL_WrapperPacket()
        
        network_config = self.config['network']
        self.vision_port = network_config['vision_port']
        self.multicast_ip = network_config['multicast_ip']
        self.buffer_size = network_config['buffer_size']
        
        field_config = self.config['field']
        self.field_size = (field_config['length'], field_config['width'])
        self.team_side = field_config['team_side']

    def _setup_socket(self):
        """Setup multicast socket"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.vision_port))
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        
        mreq = struct.pack("4s4s", 
                          socket.inet_aton(self.multicast_ip),
                          socket.inet_aton("0.0.0.0"))
        self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def normalize_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """Convert SSL coordinates to normalized field coordinates"""
        x_meters = x / 1000.0
        y_meters = y / 1000.0
        
        if self.team_side == 'right':
            x_meters = -x_meters
            y_meters = -y_meters
            
        x_norm = x_meters + self.field_size[0] / 2
        y_norm = y_meters + self.field_size[1] / 2
        
        return x_norm, y_norm
    def print_frame_info(self, frame: dict):
        """Print detailed frame information with robust error handling"""
        if not frame:
            print("\nNo frame data available")
            return

        print("\n=============== Vision Frame Information ===============")
        print(f"FPS: {format_value(self._fps, '.1f')}")
        print(f"Frame Number: {format_value(frame.get('frame_number'), 'd')}")
        print(f"Camera ID: {format_value(frame.get('camera_id'), 'd')}")
        print(f"Capture Time: {format_value(frame.get('t_capture'), '.6f')}")
        print(f"Sent Time: {format_value(frame.get('t_sent'), '.6f')}")
        
        print("\n--- Ball Information ---")
        ball = frame.get('ball')
        if ball:
            print(f"Position: ("
                  f"{format_value(ball.get('x'), '7.3f')}, "
                  f"{format_value(ball.get('y'), '7.3f')}, "
                  f"{format_value(ball.get('z'), '7.3f')})")
            print(f"Confidence: {format_value(ball.get('confidence'), '.2f')}")
            print(f"Pixel coordinates: ("
                  f"{format_value(ball.get('pixel_x'), '7.2f')}, "
                  f"{format_value(ball.get('pixel_y'), '7.2f')})")
        else:
            print("No ball detected")

        for color in ['blue', 'yellow']:
            robots = frame.get(f'robots_{color}', {})
            print(f"\n--- {color.upper()} Team Robots ({len(robots)}) ---")
            if robots:
                print("ID   |  Position (x, y)    | Orientation | Confidence | Height | Pixel (x, y)")
                print("-" * 75)
                
                for robot_id, data in sorted(robots.items()):
                    try:
                        # Handle potential None or invalid robot_id
                        robot_id_str = format_value(robot_id, '3')
                        
                        # Format position
                        x_str = format_value(data.get('x'), '7.3f')
                        y_str = format_value(data.get('y'), '7.3f')
                        
                        # Format orientation
                        orientation_str = format_value(data.get('orientation'), '10.3f')
                        
                        # Format confidence and height
                        confidence_str = format_value(data.get('confidence'), '9.2f')
                        height_str = format_value(data.get('height'), '6.2f')
                        
                        # Format pixel coordinates
                        pixel_x_str = format_value(data.get('pixel_x'), '7.2f')
                        pixel_y_str = format_value(data.get('pixel_y'), '7.2f')
                        
                        print(f"{robot_id_str} | "
                              f"({x_str}, {y_str}) | "
                              f"{orientation_str} | "
                              f"{confidence_str} | "
                              f"{height_str} | "
                              f"({pixel_x_str}, {pixel_y_str})")
                    except Exception as e:
                        print(f"Error printing robot {robot_id}: {e}")
                        continue
            else:
                print(f"No {color} robots detected")
        print("====================================================\n")

    def process_frame(self, packet: SSL_WrapperPacket) -> dict:
        """Process vision packet into structured data with error handling"""
        if not packet.HasField('detection'):
            return self.last_frame

        detection = packet.detection
        frame_data = {}

        # Process ball
        frame_data['ball'] = None
        if detection.balls:
            ball = detection.balls[0]
            try:
                x, y = self.normalize_coordinates(ball.x, ball.y)
                frame_data['ball'] = {
                    'x': x,
                    'y': y,
                    'z': ball.z if ball.HasField('z') else None,
                    'confidence': ball.confidence if hasattr(ball, 'confidence') else None,
                    'pixel_x': ball.pixel_x if hasattr(ball, 'pixel_x') else None,
                    'pixel_y': ball.pixel_y if hasattr(ball, 'pixel_y') else None
                }
            except Exception as e:
                print(f"Error processing ball data: {e}")

        # Process robots
        frame_data['robots_blue'] = {}
        frame_data['robots_yellow'] = {}

        for color in ['blue', 'yellow']:
            robot_list = detection.robots_blue if color == 'blue' else detection.robots_yellow
            target_dict = frame_data['robots_blue'] if color == 'blue' else frame_data['robots_yellow']

            for robot in robot_list:
                try:
                    robot_id = robot.robot_id if robot.HasField('robot_id') else None
                    if robot_id is None:
                        continue

                    x, y = self.normalize_coordinates(robot.x, robot.y)
                    orientation = robot.orientation if robot.HasField('orientation') else None
                    
                    if orientation is not None and self.team_side == 'right':
                        orientation += math.pi

                    target_dict[robot_id] = {
                        'x': x,
                        'y': y,
                        'orientation': orientation,
                        'confidence': robot.confidence if hasattr(robot, 'confidence') else None,
                        'height': robot.height if robot.HasField('height') else None,
                        'pixel_x': robot.pixel_x if hasattr(robot, 'pixel_x') else None,
                        'pixel_y': robot.pixel_y if hasattr(robot, 'pixel_y') else None
                    }
                except Exception as e:
                    print(f"Error processing robot data: {e}")
                    continue

        frame_data['frame_number'] = detection.frame_number
        frame_data['t_capture'] = detection.t_capture
        frame_data['t_sent'] = detection.t_sent
        frame_data['camera_id'] = detection.camera_id

        self.last_frame = frame_data
        return frame_data
    
    def update_fps(self):
        """Update FPS calculation"""
        self._frame_times.append(time.time())
        if len(self._frame_times) <= 3:
            return
        
        intervals = [b - a for a, b in zip(self._frame_times, list(self._frame_times)[1:])]
        if intervals:
            self._fps = len(intervals) / sum(intervals)

    def run(self):
        """Main thread loop"""
        print("Starting vision client...")
        
        while self.running:
            try:
                data = self.socket.recv(self.buffer_size)
                self.packet.ParseFromString(data)
                self.frame = self.process_frame(self.packet)
                self.update_fps()
            except Exception as e:
                print(f"Error processing vision data: {e}")
            
    def get_frame(self) -> dict:
        """Get the latest processed frame"""
        return self.frame

    def get_fps(self) -> float:
        """Get current FPS"""
        return self._fps

    def stop(self):
        """Stop the vision thread"""
        self.running = False
        self.join()

def main():
    # Create and start vision client
    vision = SSLVisionClient('config.json')
    vision.start()

    try:
        while True:
            frame = vision.get_frame()
            if frame:
                vision.print_frame_info(frame)
            time.sleep(0.016)  # ~60Hz refresh rate
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        vision.stop()

if __name__ == "__main__":
    main()