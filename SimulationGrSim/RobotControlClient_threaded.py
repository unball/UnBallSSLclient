import socket
import time
import math
import threading
from queue import Queue
from dataclasses import dataclass
from protocols.grsim.ssl_simulation_robot_control_pb2 import (
    RobotControl,
    RobotCommand,
    RobotMoveCommand,
    MoveWheelVelocity,
    MoveGlobalVelocity,
    MoveLocalVelocity,
)


@dataclass
class RobotCommand:
    """Data class for robot commands"""

    robot_id: int
    command_type: str  # 'wheel', 'global', 'local'
    values: tuple  # (w1,w2,w3,w4) or (vx,vy,w)


class ThreadedRobotControlClient:
    """Threaded client for controlling SSL robots in simulation"""

    def __init__(self, ip: str = "localhost", team_port: int = 10301):
        """Initialize threaded robot control client

        Args:
            ip: Server IP address
            team_port: Team control port (10301 for blue, 10302 for yellow)
        """
        self.ip = ip
        self.team_port = team_port
        self.command_queue = Queue()
        self.running = False
        self.thread = None
        self.sent_commands = 0
        self.failed_commands = 0
        self.last_command_time = None

    def start(self):
        """Start the command processing thread"""
        self.running = True
        self.thread = threading.Thread(target=self._process_commands)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        """Stop the command processing thread"""
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None

    def _process_commands(self):
        """Process commands from queue and send to simulator"""
        while self.running:
            try:
                if not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()
                    if cmd.command_type == "wheel":
                        self._send_wheel_velocity(cmd.robot_id, *cmd.values)
                    elif cmd.command_type == "global":
                        self._send_global_velocity(cmd.robot_id, *cmd.values)
                    elif cmd.command_type == "local":
                        self._send_local_velocity(cmd.robot_id, *cmd.values)
                time.sleep(1 / 300)  # Process at 300Hz
            except Exception as e:
                print(f"Error processing command: {e}")

    def _send_command(self, control_message: RobotControl) -> bool:
        """Send a control message to the simulator"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                bytes_sent = sock.sendto(
                    control_message.SerializeToString(), (self.ip, self.team_port)
                )
                self.sent_commands += 1
                self.last_command_time = time.time()
                return True
        except Exception as e:
            print(f"Error sending command: {e}")
            self.failed_commands += 1
            return False

    def _get_success_rate(self):
        """Calculate success rate of commands"""
        if self.sent_commands == 0:
            return 100.0
        return ((self.sent_commands - self.failed_commands) / self.sent_commands) * 100

    def get_stats(self):
        """Get command statistics"""
        return {
            "sent_commands": self.sent_commands,
            "failed_commands": self.failed_commands,
            "success_rate": self._get_success_rate(),
            "last_command_time": self.last_command_time,
        }

    def _send_wheel_velocity(
        self, robot_id: int, w1: float, w2: float, w3: float, w4: float
    ):
        """Internal method to send wheel velocities"""
        control = RobotControl()
        robot_command = control.robot_commands.add()
        robot_command.id = robot_id

        wheel_vel = MoveWheelVelocity()
        wheel_vel.front_right = w1
        wheel_vel.back_right = w2
        wheel_vel.back_left = w3
        wheel_vel.front_left = w4

        robot_command.move_command.wheel_velocity.CopyFrom(wheel_vel)
        return self._send_command(control)

    def _send_global_velocity(
        self, robot_id: int, vx: float, vy: float, angular: float
    ):
        """Internal method to send global velocities"""
        control = RobotControl()
        robot_command = control.robot_commands.add()
        robot_command.id = robot_id

        global_vel = MoveGlobalVelocity()
        global_vel.x = vx
        global_vel.y = vy
        global_vel.angular = angular

        robot_command.move_command.global_velocity.CopyFrom(global_vel)
        return self._send_command(control)

    def _send_local_velocity(
        self, robot_id: int, forward: float, left: float, angular: float
    ):
        """Internal method to send local velocities"""
        control = RobotControl()
        robot_command = control.robot_commands.add()
        robot_command.id = robot_id

        local_vel = MoveLocalVelocity()
        local_vel.forward = forward
        local_vel.left = left
        local_vel.angular = angular

        robot_command.move_command.local_velocity.CopyFrom(local_vel)
        return self._send_command(control)

    # Public interface methods
    def send_wheel_velocity(
        self, robot_id: int, w1: float, w2: float, w3: float, w4: float
    ):
        """Queue wheel velocity command"""
        self.command_queue.put(RobotCommand(robot_id, "wheel", (w1, w2, w3, w4)))

    def send_global_velocity(self, robot_id: int, vx: float, vy: float, angular: float):
        """Queue global velocity command"""
        self.command_queue.put(RobotCommand(robot_id, "global", (vx, vy, angular)))

    def send_local_velocity(
        self, robot_id: int, forward: float, left: float, angular: float
    ):
        """Queue local velocity command"""
        self.command_queue.put(
            RobotCommand(robot_id, "local", (forward, left, angular))
        )

    def move_robot_circle(
        self, robot_id: int, radius: float = 1.0, angular_speed: float = 1.0
    ):
        """Move robot in circle pattern"""
        linear_speed = radius * angular_speed
        t0 = time.time()

        try:
            while self.running:
                t = time.time() - t0
                angle = angular_speed * t

                vx = -linear_speed * math.sin(angle)
                vy = linear_speed * math.cos(angle)

                self.send_global_velocity(robot_id, vx, vy, angular_speed)
                time.sleep(1 / 60)
        except KeyboardInterrupt:
            self.send_global_velocity(robot_id, 0, 0, 0)


def main():
    debug = True

    try:
        # Test clients
        blue_client = ThreadedRobotControlClient(team_port=10301)
        yellow_client = ThreadedRobotControlClient(team_port=10302)

        blue_client.start()
        yellow_client.start()

        if debug:
            print("[DEBUG] Starting robot control test...")
            last_stats_time = time.time()

        while True:
            # Test some commands
            blue_client.send_wheel_velocity(0, 15, 1, 15, 1)
            blue_client.send_global_velocity(1, 1, 0, 0)
            blue_client.send_local_velocity(2, 0, 1, 0)

            yellow_client.send_wheel_velocity(0, 15, 1, 15, 1)
            yellow_client.send_global_velocity(1, -1, 0, 0)
            yellow_client.send_local_velocity(2, 0, -1, 0)

            # Print stats every 5 seconds
            current_time = time.time()
            if current_time - last_stats_time >= 5:
                print("\n[DEBUG] === 5 Second Statistics ===")
                print("Blue Team:", blue_client.get_stats())
                print("Yellow Team:", yellow_client.get_stats())
                print("================================\n")
                last_stats_time = current_time

            time.sleep(1 / 60)

    except KeyboardInterrupt:
        print("\n[DEBUG] === Final Statistics ===")
        blue_client.stop()
        yellow_client.stop()
        print("Blue Team:", blue_client.get_stats())
        print("Yellow Team:", yellow_client.get_stats())
        print("============================")
        print("[DEBUG] Stopping robot control test")


if __name__ == "__main__":
    main()
