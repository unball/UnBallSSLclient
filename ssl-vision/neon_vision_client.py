
import json
import socket
import struct
import logging
import threading
import math
import time
from google.protobuf.json_format import MessageToJson
from proto import messages_robocup_ssl_wrapper_pb2
import yaml

class GrSimVision(threading.Thread):
    def __init__(self, game) -> None:
        super(GrSimVision, self).__init__()

        self.game = game
        self.config = self.game.config
        self.daemon = True
        self.running = False
        self._fps = 60
        self.new_data = False
        self.any_geometry = False

        # Initialize geometry data structure
        self.raw_geometry = {
            "fieldLength": 0,
            "fieldWidth": 0,
            "goalWidth": 0,
            "fieldLines": {
                "LeftGoalLine": {"p1": {"x": 0, "y": 0}},
                "RightGoalLine": {"p1": {"x": 0, "y": 0}},
                "HalfwayLine": {"p1": {"x": 0, "y": 0}},
                "LeftPenaltyStretch": {"p1": {"x": 0, "y": 0}},
                "RightPenaltyStretch": {"p1": {"x": 0, "y": 0}},
                "RightGoalBottomLine": {"p1": {"x": 0, "y": 0}},
                "LeftGoalBottomLine": {"p1": {"x": 0, "y": 0}},
            },
        }

        # Initialize detection data structure
        self.raw_detection = {
            "ball": {"x": 0, "y": 0, "tCapture": -1, "cCapture": -1},
            "robotsBlue": {
                i: {"x": None, "y": None, "theta": None, "tCapture": -1}
                for i in range(0, 16)
            },
            "robotsYellow": {
                i: {"x": None, "y": None, "theta": None, "tCapture": -1}
                for i in range(0, 16)
            },
            "meta": {
                "has_speed": True,
                "has_height": True,
                "cameras": {i: {"last_capture": -1} for i in range(0, 4)},
            },
        }

        self.side_factor = 1
        self.angle_factor = 0
        self.vision_port = self.config["network"]["vision_port"]
        self.host = self.config["network"]["multicast_ip"]
        self.logger = logging.getLogger("input")

    def run(self):
        """Main thread loop to receive and process vision data"""
        self.logger.info(f"Starting SSL-Vision module...")
        self.logger.info(
            f"Creating socket with address: {self.host} and port: {self.vision_port}"
        )
        self.vision_sock = self._create_socket()
        self._wait_to_connect()
        self.logger.info(f"SSL-Vision module started!")

        self.running = True
        while self.running:
            env = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
            data = self.vision_sock.recv(2048)
            env.ParseFromString(data)
            last_frame = json.loads(MessageToJson(env))
            self.new_data = self.update_detection(last_frame)
            self.new_geometry = self.update_geometry(last_frame)
        self.stop()

    def stop(self):
        """Stop the vision thread"""
        self.running = False
        self.vision_sock.close()
        self.logger.info(f"SSL-Vision module stopped!")

    def update_detection(self, last_frame):
        """Update detection data from new frame"""
        frame = last_frame.get("detection")
        if not frame:
            return False

        t_capture = frame.get("tCapture")
        camera_id = frame.get("cameraId")
        self.update_camera_capture_number(camera_id, t_capture)

        self.side_factor = 1 if self.config["match"]["team_side"] == "left" else -1
        self.angle_factor = 0 if self.config["match"]["team_side"] == "left" else math.pi

        # Update ball data
        balls = frame.get("balls", [])
        self.update_ball_detection(balls, camera_id)

        # Update robot data
        robots_blue = frame.get("robotsBlue")
        if robots_blue:
            for robot in robots_blue:
                self.update_robot_detection(robot, t_capture, camera_id, color="Blue")

        robots_yellow = frame.get("robotsYellow")
        if robots_yellow:
            for robot in robots_yellow:
                self.update_robot_detection(robot, t_capture, camera_id, color="Yellow")

        return True

    def update_geometry(self, last_frame):
        """Update geometry data from new frame"""
        frame = last_frame.get("geometry")
        if not frame:
            return False

        self.any_geometry = True
        frame = frame.get("field")

        # Update field dimensions
        self.raw_geometry["fieldLength"] = frame.get("fieldLength") / 1000
        self.raw_geometry["fieldWidth"] = frame.get("fieldWidth") / 1000
        self.raw_geometry["goalWidth"] = frame.get("goalWidth") / 1000
        self.raw_geometry["penaltyAreaDepth"] = frame.get("penaltyAreaDepth", 1000) / 1000
        self.raw_geometry["penaltyAreaWidth"] = frame.get("penaltyAreaWidth", 2000) / 1000

        # Update field lines
        field_lines = frame.get("fieldLines", [])
        if len(field_lines) >= 10:  # Ensure we have enough lines
            self.raw_geometry["fieldLines"].update({
                "LeftGoalLine": {"p1": {
                    "x": field_lines[2]["p1"]["x"] / 1000,
                    "y": field_lines[2]["p1"]["y"] / 1000
                }},
                "RightGoalLine": {"p1": {
                    "x": field_lines[3]["p1"]["x"] / 1000,
                    "y": field_lines[3]["p1"]["y"] / 1000
                }},
                "HalfwayLine": {"p1": {
                    "x": field_lines[4]["p1"]["x"] / 1000,
                    "y": field_lines[4]["p1"]["y"] / 1000
                }},
                "LeftPenaltyStretch": {"p1": {
                    "x": field_lines[6]["p1"]["x"] / 1000,
                    "y": field_lines[6]["p1"]["y"] / 1000
                }},
                "RightPenaltyStretch": {"p1": {
                    "x": field_lines[7]["p1"]["x"] / 1000,
                    "y": field_lines[7]["p1"]["y"] / 1000
                }},
                "RightGoalBottomLine": {"p1": {
                    "x": field_lines[9]["p1"]["x"] / 1000,
                    "y": field_lines[9]["p1"]["y"] / 1000
                }}
            })

        return True

    def update_camera_capture_number(self, camera_id, t_capture):
        """Update camera capture timestamp"""
        last_camera_data = self.raw_detection["meta"]["cameras"][camera_id]
        if last_camera_data["last_capture"] > t_capture:
            return
        self.raw_detection["meta"]["cameras"][camera_id] = {"last_capture": t_capture}

    def update_ball_detection(self, balls, camera_id):
        """Update ball detection data"""
        if len(balls) > 0:
            ball = balls[0]
            self.raw_detection["ball"] = {
                "x": self.side_factor * ball.get("x") / 1000,
                "y": self.side_factor * ball.get("y") / 1000,
                "tCapture": ball.get("tCapture"),
                "cCapture": camera_id,
            }

    def update_robot_detection(self, robot, _timestamp, camera_id, color="Blue"):
        """Update robot detection data"""
        robot_id = robot.get("robotId")
        last_robot_data = self.raw_detection["robots" + color][robot_id]
        if last_robot_data.get("tCapture") > _timestamp:
            return

        self.raw_detection["robots" + color][robot_id] = {
            "x": self.side_factor * robot["x"] / 1000,
            "y": self.side_factor * robot["y"] / 1000,
            "theta": robot["orientation"] + self.angle_factor,
            "tCapture": _timestamp,
            "cCapture": camera_id,
        }

    def get_geometry(self):
        """Get current geometry data"""
        self.new_data = False
        return self.raw_geometry

    def get_last_frame(self):
        """Get latest frame data"""
        self.new_data = False
        return self.raw_detection

    def _wait_to_connect(self):
        """Wait for initial connection"""
        self.vision_sock.recv(1024)

    def _create_socket(self):
        """Create and configure UDP socket"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.vision_port))
        mreq = struct.pack("4sl", socket.inet_aton(self.host), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return sock

    def print_formatted_vision_data(self):
        """Print vision data in formatted output"""
        last_frame = self.get_last_frame()
        geometry = self.get_geometry()
        
        print("-----Received Wrapper Packet---------------------------------------------")
        print("-[Detection Data]-------")
        
        t_now = time.time()
        
        # Get camera metadata
        camera_id = -1
        t_capture = -1
        for robot_dict in last_frame["robotsBlue"].values():
            if robot_dict["tCapture"] > 0:
                t_capture = robot_dict["tCapture"]
                camera_id = robot_dict.get("cCapture", -1)
                break
        
        if t_capture > 0:
            print(f"Camera ID={camera_id} T_CAPTURE={t_capture:.4f}")
            print(f"Total Latency (assuming synched system clock) {(t_now - t_capture) * 1000.0:7.3f}ms")
        
        # Print ball data
        if last_frame["ball"]["tCapture"] is not None and last_frame["ball"]["tCapture"] > 0:
            ball = last_frame["ball"]
            print(f"-Ball: POS=<{ball['x']:9.2f},{ball['y']:9.2f}>")
        
        # Print blue robots
        blue_robots = [(id, robot) for id, robot in last_frame["robotsBlue"].items() 
                      if robot["x"] is not None]
        for i, (robot_id, robot) in enumerate(blue_robots, 1):
            print(f"-Robot(B) ({i:2d}/{len(blue_robots):2d}): ", end="")
            print(f"ID={robot_id:3d} ", end="")
            print(f"POS=<{robot['x']:9.2f},{robot['y']:9.2f}> ", end="")
            if robot['theta'] is not None:
                print(f"ANGLE={robot['theta']:6.3f}")
            else:
                print("ANGLE=N/A")
        
        # Print yellow robots
        yellow_robots = [(id, robot) for id, robot in last_frame["robotsYellow"].items() 
                        if robot["x"] is not None]
        for i, (robot_id, robot) in enumerate(yellow_robots, 1):
            print(f"-Robot(Y) ({i:2d}/{len(yellow_robots):2d}): ", end="")
            print(f"ID={robot_id:3d} ", end="")
            print(f"POS=<{robot['x']:9.2f},{robot['y']:9.2f}> ", end="")
            if robot['theta'] is not None:
                print(f"ANGLE={robot['theta']:6.3f}")
            else:
                print("ANGLE=N/A")
        
        # Print geometry data
        print("-[Geometry Data]-------")
        print(f"Field Dimensions: {int(geometry['fieldLength']*1000)}x{int(geometry['fieldWidth']*1000)} mm")
        print(f"Goal Width: {int(geometry['goalWidth']*1000)} mm")
        print(f"Penalty Area: {int(geometry['penaltyAreaWidth']*1000)}x{int(geometry['penaltyAreaDepth']*1000)} mm")


def main():
    """Main function for testing"""
    # Set up logging
    logging.basicConfig(level=logging.INFO)
    
    # Create test environment
    class MockGame:
        def __init__(self):
            self.config = {
                "network": {"vision_port": 10006, "multicast_ip": "224.5.23.2"},
                "match": {"team_side": "left"},
            }
    
    # Create and start vision client
    
    mock_game = MockGame()
    vision_client = GrSimVision(mock_game)
    
    vision_client.start()
    
    # Wait for data
    time.sleep(2)
    
    # Print formatted data
    
    #formatacao bonita
    vision_client.print_formatted_vision_data()
    
    #json feio
    print("Last Frame:", vision_client.get_last_frame())
    print("Geometry:", vision_client.get_geometry())
    
    # Cleanup
    vision_client.stop()
    vision_client.join()

if __name__ == "__main__":
    main()

