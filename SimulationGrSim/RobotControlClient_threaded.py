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

# Debug flags - set these to control debug output
DEBUG_COMMAND_QUEUE = False  # Print when commands are queued
DEBUG_COMMAND_SEND = False  # Print when commands are sent
DEBUG_COMMAND_PROCESS = False  # Print during command processing
DEBUG_STATS = False  # Print statistics
DEBUG_ERRORS = True  # Always print errors (keep this True)


@dataclass
class CommandData:
    """Data class for robot commands"""

    robot_id: int
    command_type: str  # 'wheel', 'global', 'local'
    move_values: tuple = None  # (vx,vy,w) or (w1,w2,w3,w4) or (forward,left,w)
    kick_values: tuple = None  # (kick_speed_x, kick_speed_z, dribbler_speed)


class ThreadedRobotControlClient:
    """Threaded client for controlling SSL robots in simulation"""

    def __init__(
        self, ip: str = "localhost", team_port: int = 10301, debug_enabled: bool = False
    ):
        """Initialize threaded robot control client

        Args:
            ip: Server IP address
            team_port: Team control port (10301 for blue, 10302 for yellow)
            debug_enabled: Enable debug output for this instance
        """
        self.ip = ip
        self.team_port = team_port
        self.command_queue = Queue()
        self.running = False
        self.thread = None
        self.sent_commands = 0
        self.failed_commands = 0
        self.last_command_time = None

        # Instance-specific debug flag
        self.debug_enabled = debug_enabled or DEBUG_COMMAND_QUEUE or DEBUG_COMMAND_SEND

        if DEBUG_STATS or self.debug_enabled:
            print(f"ThreadedRobotControlClient initialized for port {team_port}")

    def start(self):
        """Start the command processing thread"""
        self.running = True
        self.thread = threading.Thread(target=self._process_commands)
        self.thread.daemon = True
        self.thread.start()
        if DEBUG_STATS or self.debug_enabled:
            print(f"ThreadedRobotControlClient started for port {self.team_port}")

    def stop(self):
        """Stop the command processing thread"""
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None
        if DEBUG_STATS or self.debug_enabled:
            print(f"ThreadedRobotControlClient stopped for port {self.team_port}")

    def _process_commands(self):
        """Process commands from queue and send to simulator"""
        while self.running:
            try:
                if not self.command_queue.empty():
                    cmd = self.command_queue.get_nowait()

                    # Create control message
                    control = RobotControl()
                    robot_command = control.robot_commands.add()
                    robot_command.id = cmd.robot_id

                    # Set movement command
                    if cmd.command_type == "global" and cmd.move_values:
                        vx, vy, angular = cmd.move_values

                        # Debug output for processing (only if significant movement)
                        if DEBUG_COMMAND_PROCESS and (
                            abs(vx) > 0.01 or abs(vy) > 0.01 or abs(angular) > 0.01
                        ):
                            print(
                                f"_process_commands: Processing robot {cmd.robot_id}: vx={vx:.3f}, vy={vy:.3f}, w={angular:.3f}"
                            )

                        global_vel = MoveGlobalVelocity()
                        global_vel.x = vx
                        global_vel.y = vy
                        global_vel.angular = angular
                        robot_command.move_command.global_velocity.CopyFrom(global_vel)

                    elif cmd.command_type == "wheel" and cmd.move_values:
                        w1, w2, w3, w4 = cmd.move_values
                        wheel_vel = MoveWheelVelocity()
                        wheel_vel.front_right = w1
                        wheel_vel.back_right = w2
                        wheel_vel.back_left = w3
                        wheel_vel.front_left = w4
                        robot_command.move_command.wheel_velocity.CopyFrom(wheel_vel)

                    elif cmd.command_type == "local" and cmd.move_values:
                        forward, left, angular = cmd.move_values
                        local_vel = MoveLocalVelocity()
                        local_vel.forward = forward
                        local_vel.left = left
                        local_vel.angular = angular
                        robot_command.move_command.local_velocity.CopyFrom(local_vel)

                    # Set kick command if provided
                    if cmd.kick_values:
                        kick_speed_x, kick_speed_z, dribbler_speed = cmd.kick_values
                        robot_command.kick_speed = kick_speed_x
                        robot_command.kick_angle = kick_speed_z
                        robot_command.dribbler_speed = dribbler_speed

                    # Send command
                    result = self._send_command(control)

                    # Debug output for successful send (much more selective)
                    if DEBUG_COMMAND_SEND and result and cmd.command_type == "global":
                        vx, vy, angular = cmd.move_values or (0, 0, 0)
                        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(angular) > 0.01:
                            print(f"Successfully sent command to robot {cmd.robot_id}")

                time.sleep(1 / 300)  # Process at 300Hz
            except Exception as e:
                if DEBUG_ERRORS:
                    print(f"Error processing command: {e}")
                    import traceback

                    traceback.print_exc()

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
            if DEBUG_ERRORS:
                print(f"Error sending command to port {self.team_port}: {e}")
            self.failed_commands += 1
            return False

    def _get_success_rate(self):
        """Calculate success rate of commands"""
        if self.sent_commands == 0:
            return 100.0
        return ((self.sent_commands - self.failed_commands) / self.sent_commands) * 100

    def get_position(self, robot_id):
        """Get robot position"""
        return {"x": 0.0, "y": 0.0, "theta": 0.0}  # Default position

    def get_stats(self):
        """Get command statistics"""
        return {
            "sent_commands": self.sent_commands,
            "failed_commands": self.failed_commands,
            "success_rate": self._get_success_rate(),
            "last_command_time": self.last_command_time,
        }

    # Public interface methods
    def send_wheel_velocity(
        self, robot_id: int, w1: float, w2: float, w3: float, w4: float
    ):
        """Queue wheel velocity command"""
        self.command_queue.put(
            CommandData(robot_id, "wheel", move_values=(w1, w2, w3, w4))
        )

    def send_global_velocity(self, robot_id: int, vx: float, vy: float, angular: float):
        """Queue global velocity command"""
        # Debug output only for significant movement and when enabled
        if DEBUG_COMMAND_QUEUE and (
            abs(vx) > 0.01 or abs(vy) > 0.01 or abs(angular) > 0.01
        ):
            print(
                f"ThreadedClient port {self.team_port}: Queuing command for robot {robot_id}: vx={vx:.3f}, vy={vy:.3f}, w={angular:.3f}"
            )

        self.command_queue.put(
            CommandData(robot_id, "global", move_values=(vx, vy, angular))
        )

    def send_local_velocity(
        self, robot_id: int, forward: float, left: float, angular: float
    ):
        """Queue local velocity command"""
        self.command_queue.put(
            CommandData(robot_id, "local", move_values=(forward, left, angular))
        )

    def send_command_with_kick(
        self,
        robot_id: int,
        vx: float,
        vy: float,
        angular: float,
        kick_speed_x: float,
        kick_speed_z: float = 0.0,
        dribbler_speed: float = 0.0,
    ):
        """Queue a command that includes global velocity and kick parameters."""
        # Disable dribbler for SSL-EL
        if dribbler_speed != 0.0:
            dribbler_speed = 0.0

        move_vals = (vx, vy, angular)
        kick_vals = (kick_speed_x, kick_speed_z, dribbler_speed)

        # Debug output only when enabled
        if DEBUG_COMMAND_QUEUE and (
            abs(vx) > 0.01 or abs(vy) > 0.01 or abs(angular) > 0.01
        ):
            print(
                f"ThreadedClient port {self.team_port}: Queuing command for robot {robot_id}: vx={vx:.3f}, vy={vy:.3f}, w={angular:.3f}"
            )

        self.command_queue.put(
            CommandData(
                robot_id, "global", move_values=move_vals, kick_values=kick_vals
            )
        )

    def kick_flat(self, robot_id: int, kick_speed: float):
        """Send a flat kick command"""
        if DEBUG_COMMAND_QUEUE:
            print(
                f"ThreadedClient port {self.team_port}: Kick command for robot {robot_id}, speed={kick_speed}"
            )
        self.command_queue.put(
            CommandData(
                robot_id,
                "global",
                move_values=(0, 0, 0),
                kick_values=(kick_speed, 0.0, 0.0),
            )
        )

    def kick_chip(self, robot_id: int, kick_speed: float):
        """Send a chip kick command"""
        if DEBUG_COMMAND_QUEUE:
            print(
                f"ThreadedClient port {self.team_port}: Chip kick command for robot {robot_id}, speed={kick_speed}"
            )
        self.command_queue.put(
            CommandData(
                robot_id,
                "global",
                move_values=(0, 0, 0),
                kick_values=(kick_speed, kick_speed, 0.0),
            )
        )
