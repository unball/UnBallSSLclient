import socket
import struct
import json
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple
import math
from proto import ssl_vision_wrapper_pb2 as ssl_wrapper_pb2

def format_value(value, format_spec: str, default: str = "N/A") -> str:
    """Format a value with error handling"""
    if value is None:
        return default.rjust(len(format_spec.replace(':', '')))
    try:
        return f"{value:{format_spec}}"
    except (TypeError, ValueError):
        return default.rjust(len(format_spec.replace(':', '')))

class EnhancedVisionClient(threading.Thread):

    def __init__(self, config_file: str = 'config.json', is_simulation: bool = False):
        super().__init__()
        self.config = self._load_config(config_file)
        self.is_simulation = is_simulation
        self._setup_variables()
        self._setup_socket()
        
        print(f"Initializing Vision Client in {'Simulation' if is_simulation else 'Physical SSL-Vision'} mode")

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
        self.callback = None  # Added callback variable
        
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

    def assign_callback(self, callback):
        """Assign callback function to be called after each frame processing"""
        self.callback = callback

    def update_fps(self):
        """Update FPS calculation"""
        self._frame_times.append(time.time())
        if len(self._frame_times) <= 3:
            return
        
        intervals = [b - a for a, b in zip(self._frame_times, list(self._frame_times)[1:])]
        if intervals:
            self._fps = len(intervals) / sum(intervals)

    def normalize_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """Convert SSL coordinates to normalized field coordinates"""
        # Convert from millimeters to meters
        x_meters = x / 1000.0
        y_meters = y / 1000.0
        
        # Debug print to see raw coordinates
        print(f"Raw coordinates (m): ({x_meters:.3f}, {y_meters:.3f})")
        print(f"Field size (m): ({self.field_size[0]}, {self.field_size[1]})")
        
        # Apply team side transformation if needed
        if self.team_side == 'right':
            x_meters = -x_meters
            y_meters = -y_meters
        
        # Center the coordinates on the field
        # Note: SSL Vision uses center of field as (0,0)
        # We want to transform this to have (0,0) at one corner
        x_norm = x_meters + (self.field_size[0] / 2)
        y_norm = y_meters + (self.field_size[1] / 2)
        
        # Debug print transformed coordinates
        print(f"Normalized coordinates (m): ({x_norm:.3f}, {y_norm:.3f})")
        
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
                        robot_id_str = format_value(robot_id, '3')
                        x_str = format_value(data.get('x'), '7.3f')
                        y_str = format_value(data.get('y'), '7.3f')
                        orientation_str = format_value(data.get('orientation'), '10.3f')
                        confidence_str = format_value(data.get('confidence'), '9.2f')
                        height_str = format_value(data.get('height'), '6.2f')
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
        
    def process_frame(self, detection) -> dict:
        """Process vision frame into structured data with error handling"""
        if not detection:
            return self.last_frame

        frame_data = {
            'frame_number': detection.frame_number,
            't_capture': detection.t_capture,
            't_sent': detection.t_sent,
            'camera_id': detection.camera_id,
            'ball': None,
            'robots_blue': {},
            'robots_yellow': {}
        }

        # For simulation mode, process coordinates directly without additional transformations
        if self.is_simulation:
            # Process blue robots
            for robot in detection.robots_blue:
                try:
                    if robot.HasField('robot_id'):
                        frame_data['robots_blue'][robot.robot_id] = {
                            'x': robot.x / 1000.0,  # Convert to meters
                            'y': robot.y / 1000.0,
                            'orientation': robot.orientation if robot.HasField('orientation') else 0,
                            'confidence': robot.confidence,
                            'height': robot.height if robot.HasField('height') else 0,
                            'pixel_x': robot.pixel_x,
                            'pixel_y': robot.pixel_y
                        }
                except Exception as e:
                    print(f"Error processing blue robot in simulation: {e}")

            # Process yellow robots
            for robot in detection.robots_yellow:
                try:
                    if robot.HasField('robot_id'):
                        frame_data['robots_yellow'][robot.robot_id] = {
                            'x': robot.x / 1000.0,  # Convert to meters
                            'y': robot.y / 1000.0,
                            'orientation': robot.orientation if robot.HasField('orientation') else 0,
                            'confidence': robot.confidence,
                            'height': robot.height if robot.HasField('height') else 0,
                            'pixel_x': robot.pixel_x,
                            'pixel_y': robot.pixel_y
                        }
                except Exception as e:
                    print(f"Error processing yellow robot in simulation: {e}")

            # Process ball
            if detection.balls:
                ball = detection.balls[0]
                try:
                    frame_data['ball'] = {
                        'x': ball.x / 1000.0,  # Convert to meters
                        'y': ball.y / 1000.0,
                        'z': ball.z if ball.HasField('z') else None,
                        'confidence': ball.confidence if hasattr(ball, 'confidence') else None,
                        'pixel_x': ball.pixel_x if hasattr(ball, 'pixel_x') else None,
                        'pixel_y': ball.pixel_y if hasattr(ball, 'pixel_y') else None
                    }
                except Exception as e:
                    print(f"Error processing ball in simulation: {e}")

        else:
            # Original SSL-Vision processing with field transformations
            # Process blue robots
            for robot in detection.robots_blue:
                try:
                    if robot.HasField('robot_id'):
                        x = robot.x / 1000.0  # Convert to meters
                        y = robot.y / 1000.0
                        
                        if self.team_side == 'right':
                            x = -x
                            y = -y
                            
                        x += self.field_size[0] / 2
                        y += self.field_size[1] / 2
                        
                        frame_data['robots_blue'][robot.robot_id] = {
                            'x': x,
                            'y': y,
                            'orientation': robot.orientation if robot.HasField('orientation') else 0,
                            'confidence': robot.confidence,
                            'height': robot.height if robot.HasField('height') else 0,
                            'pixel_x': robot.pixel_x,
                            'pixel_y': robot.pixel_y
                        }
                except Exception as e:
                    print(f"Error processing blue robot in SSL-Vision: {e}")

            # Process yellow robots
            for robot in detection.robots_yellow:
                try:
                    if robot.HasField('robot_id'):
                        x = robot.x / 1000.0  # Convert to meters
                        y = robot.y / 1000.0
                        
                        if self.team_side == 'right':
                            x = -x
                            y = -y
                            
                        x += self.field_size[0] / 2
                        y += self.field_size[1] / 2
                        
                        frame_data['robots_yellow'][robot.robot_id] = {
                            'x': x,
                            'y': y,
                            'orientation': robot.orientation if robot.HasField('orientation') else 0,
                            'confidence': robot.confidence,
                            'height': robot.height if robot.HasField('height') else 0,
                            'pixel_x': robot.pixel_x,
                            'pixel_y': robot.pixel_y
                        }
                except Exception as e:
                    print(f"Error processing yellow robot in SSL-Vision: {e}")

            # Process ball
            if detection.balls:
                ball = detection.balls[0]
                try:
                    x = ball.x / 1000.0  # Convert to meters
                    y = ball.y / 1000.0
                    
                    if self.team_side == 'right':
                        x = -x
                        y = -y
                        
                    x += self.field_size[0] / 2
                    y += self.field_size[1] / 2
                    
                    frame_data['ball'] = {
                        'x': x,
                        'y': y,
                        'z': ball.z if ball.HasField('z') else None,
                        'confidence': ball.confidence if hasattr(ball, 'confidence') else None,
                        'pixel_x': ball.pixel_x if hasattr(ball, 'pixel_x') else None,
                        'pixel_y': ball.pixel_y if hasattr(ball, 'pixel_y') else None
                    }
                except Exception as e:
                    print(f"Error processing ball in SSL-Vision: {e}")

        return frame_data

    def run(self):
        """Main thread loop"""
        print("Starting vision client...")
        print(f"Field size: {self.field_size}")
        print(f"Team side: {self.team_side}")
        
        while self.running:
            try:
                data = self.socket.recv(self.buffer_size)
                packet = ssl_wrapper_pb2.SSL_WrapperPacket()
                packet.ParseFromString(data)
                
                if packet.HasField('detection'):
                    # Process the frame
                    self.frame = self.process_frame(packet.detection)
                    self.update_fps()
                    
                    # Call callback if assigned
                    if self.callback:
                        self.callback()
                    
            except Exception as e:
                print(f"Error in run loop: {e}")
                continue

    def stop(self):
        """Stop the vision thread"""
        self.running = False
        if hasattr(self, 'socket'):
            self.socket.close()
        self.join()

    # [Previous process_frame method remains the same]            
    def get_frame(self) -> dict:
        """Get the latest processed frame"""
        return self.frame

    def get_fps(self) -> float:
        """Get current FPS"""
        return self._fps

def main():
    # Create and start vision client
    vision = EnhancedVisionClient('config.json', is_simulation=True)
    
    def debug_callback():
        print("\n--- Vision Frame Info ---")
        print(f"Mode: {'Simulation' if vision.is_simulation else 'Physical SSL-Vision'}")
        print(f"Camera ID: {vision.frame.get('camera_id')}")
        print(f"Frame Number: {vision.frame.get('frame_number')}")
        print(f"FPS: {vision._fps:.2f}")
        vision.print_frame_info(vision.frame)
        print("----------------------\n")
    
    vision.assign_callback(debug_callback)
    vision.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down vision client...")
        vision.stop()

if __name__ == "__main__":
    main()