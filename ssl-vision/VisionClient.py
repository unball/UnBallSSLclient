# flake8: noqa

import socket
import struct
import json
import threading
import time
from collections import deque
from google.protobuf.json_format import MessageToJson
from proto import messages_robocup_ssl_wrapper_pb2
import os
import math

#Esse funciona mto bem na visão

class VisionClient(threading.Thread):
    def __init__(self, config_file=None):
        super().__init__()
        self.config = self.get_config(config_file)
        self.frame = {}
        self.last_frame = {}
        self.vision_port = int(os.environ.get('VISION_PORT', self.config['network']['vision_port']))
        self.host = os.environ.get('MULTICAST_IP', self.config['network']['multicast_ip'])
        self._fps = 0
        self._frame_times = deque(maxlen=60)
        self.callback = None
        self.kill_received = False

    @staticmethod
    def get_config(config_file=None):
        config_file = config_file or 'config.json'
        with open(config_file, 'r') as file:
            return json.load(file)

    def assign_callback(self, callback):
        self.callback = callback

    def connect(self):
        print(f"Creating socket with address: {self.host} and port: {self.vision_port}")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.vision_port))
        mreq = struct.pack("4sl", socket.inet_aton(self.host), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def run(self):
        print("Starting vision...")
        self.connect()
        self.sock.recv(1024)  # Wait for initial connection
        print("Vision connected!")

        while not self.kill_received:
            data = self.sock.recv(1024)
            self.set_fps()
            env = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
            env.ParseFromString(data)
            self.frame = json.loads(MessageToJson(env))

            if self.callback:
                self.callback()

    def set_fps(self):
        self._frame_times.append(time.time())
        if len(self._frame_times) <= 3:
            return
        fps_frame_by_frame = [
            (v - i) for i, v in zip(self._frame_times, list(self._frame_times)[1:])
        ]
        self._fps = len(fps_frame_by_frame) / sum(fps_frame_by_frame)

    def stop(self):
        self.kill_received = True

    @staticmethod
    def process_frame(raw_frame, field_size, team_side, last_frame=None, ball_lp=False):
        """Process vision frame placing the origin at the right edge of the field relative to your defending goal"""
        if raw_frame.get('detection') is None:
            return last_frame

        frame = raw_frame.get('detection')
        w, h = field_size

        frame['ball'] = {}
        if frame.get('balls'):
            ball = frame['balls'][0]
            frame['ball']['x'] = (-ball.get('x', 0) if team_side == 'right' else ball.get('x', 0)) / 1000 + w / 2
            frame['ball']['y'] = (-ball.get('y', 0) if team_side == 'right' else ball.get('y', 0)) / 1000 + h / 2
        else:
            frame['ball'] = last_frame['ball'] if ball_lp else {'x': -1, 'y': -1}

        for color in ['Yellow', 'Blue']:
            for robot in frame.get(f"robots{color}", []):
                if team_side == 'right':
                    robot['x'] = -robot.get('x', 0)
                    robot['y'] = -robot.get('y', 0)
                    robot['orientation'] = robot.get('orientation', 0) + math.pi

                robot['x'] = robot['x'] / 1000 + w / 2
                robot['y'] = robot['y'] / 1000 + h / 2
                robot['robotId'] = robot.get('robotId', 0)

        return frame
    
if __name__ == "__main__":
    client = VisionClient()
    client.start()

    try:
        while True:
            time.sleep(1)
            print(client.frame)
    except KeyboardInterrupt:
        client.stop()
        client.join()