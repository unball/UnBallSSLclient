import socket
import time
from datetime import datetime
from protocols.grsim.ssl_simulation_robot_control_pb2 import (
    RobotControl,
    RobotCommand,
    RobotMoveCommand,
    MoveWheelVelocity,
    MoveGlobalVelocity,
    MoveLocalVelocity,
)


class RobotControlClient:
    def __init__(
        self, ip: str = "localhost", team_port: int = 10301, debugger: bool = False
    ):
        """Initialize robot control client

        Args:
            ip: Server IP address
            team_port: Team control port (10301 for blue, 10302 for yellow)
            debugger: Enable debug information output
        """
        self.ip = ip
        self.team_port = team_port
        self.debugger = debugger
        self.sent_commands = 0
        self.failed_commands = 0
        self.last_command_time = None

        if self.debugger:
            print(
                f"[DEBUG] Initialized RobotControl client - IP: {ip}, Port: {team_port}"
            )
            print(f"[DEBUG] Team: {'Yellow' if team_port == 10302 else 'Blue'}")

    def _print_debug(
        self, command_type: str, robot_id: int, success: bool, params: tuple
    ):
        """Print debug information with timestamp"""
        current_time = datetime.now()
        timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]

        # Calculate time since last command if available
        time_diff = ""
        if self.last_command_time:
            diff = (current_time - self.last_command_time).total_seconds() * 1000
            time_diff = f", +{diff:.1f}ms"
        self.last_command_time = current_time

        team = "Yellow" if self.team_port == 10302 else "Blue"
        status = "SUCCESS" if success else "FAILED"

        print(f"[{timestamp}{time_diff}] {team} Robot {robot_id}")
        print(f"    Command: {command_type}")
        print(f"    Status: {status}")
        print(f"    Parameters: {params}")
        print(f"    Total sent: {self.sent_commands}, Failed: {self.failed_commands}")
        print(f"    Success rate: {self._get_success_rate():.1f}%")

    def _get_success_rate(self):
        """Calculate success rate of commands"""
        if self.sent_commands == 0:
            return 100.0
        return ((self.sent_commands - self.failed_commands) / self.sent_commands) * 100

    def send_wheel_velocity(
        self, robot_id: int, w1: float, w2: float, w3: float, w4: float
    ) -> bool:
        """Send wheel velocities to robot"""
        control = RobotControl()
        robot_command = control.robot_commands.add()
        robot_command.id = robot_id

        wheel_vel = MoveWheelVelocity()
        wheel_vel.front_right = w1
        wheel_vel.back_right = w2
        wheel_vel.back_left = w3
        wheel_vel.front_left = w4

        robot_command.move_command.wheel_velocity.CopyFrom(wheel_vel)

        success = False
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                bytes_sent = sock.sendto(
                    control.SerializeToString(), (self.ip, self.team_port)
                )
                self.sent_commands += 1
                success = True
                if self.debugger:
                    print(
                        f"[DEBUG] Sent {bytes_sent} bytes to {self.ip}:{self.team_port}"
                    )
        except Exception as e:
            print(f"[ERROR] Failed to send wheel velocity command: {e}")
            self.failed_commands += 1
            success = False

        if self.debugger:
            self._print_debug(
                "WHEEL_VELOCITY",
                robot_id,
                success,
                (f"FR:{w1:.2f}", f"BR:{w2:.2f}", f"BL:{w3:.2f}", f"FL:{w4:.2f}"),
            )
        return success

    def send_global_velocity(
        self, robot_id: int, vx: float, vy: float, angular: float
    ) -> bool:
        """Send global velocities to robot"""
        control = RobotControl()
        robot_command = control.robot_commands.add()
        robot_command.id = robot_id

        global_vel = MoveGlobalVelocity()
        global_vel.x = vx
        global_vel.y = vy
        global_vel.angular = angular

        robot_command.move_command.global_velocity.CopyFrom(global_vel)

        success = False
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                bytes_sent = sock.sendto(
                    control.SerializeToString(), (self.ip, self.team_port)
                )
                self.sent_commands += 1
                success = True
                if self.debugger:
                    print(
                        f"[DEBUG] Sent {bytes_sent} bytes to {self.ip}:{self.team_port}"
                    )
        except Exception as e:
            print(f"[ERROR] Failed to send global velocity command: {e}")
            self.failed_commands += 1
            success = False

        if self.debugger:
            self._print_debug(
                "GLOBAL_VELOCITY",
                robot_id,
                success,
                (f"vx:{vx:.2f}", f"vy:{vy:.2f}", f"w:{angular:.2f}"),
            )
        return success

    def send_local_velocity(
        self, robot_id: int, forward: float, left: float, angular: float
    ) -> bool:
        """Send local velocities to robot"""
        control = RobotControl()
        robot_command = control.robot_commands.add()
        robot_command.id = robot_id

        local_vel = MoveLocalVelocity()
        local_vel.forward = forward
        local_vel.left = left
        local_vel.angular = angular

        robot_command.move_command.local_velocity.CopyFrom(local_vel)

        success = False
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                bytes_sent = sock.sendto(
                    control.SerializeToString(), (self.ip, self.team_port)
                )
                self.sent_commands += 1
                success = True
                if self.debugger:
                    print(
                        f"[DEBUG] Sent {bytes_sent} bytes to {self.ip}:{self.team_port}"
                    )
        except Exception as e:
            print(f"[ERROR] Failed to send local velocity command: {e}")
            self.failed_commands += 1
            success = False

        if self.debugger:
            self._print_debug(
                "LOCAL_VELOCITY",
                robot_id,
                success,
                (f"fwd:{forward:.2f}", f"left:{left:.2f}", f"w:{angular:.2f}"),
            )
        return success

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
        if dribbler_speed != 0.0:
            dribbler_speed = 0.0

        move_vals = (vx, vy, angular)
        kick_vals = (kick_speed_x, kick_speed_z, dribbler_speed)

        # ADD DEBUG OUTPUT
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(angular) > 0.01:
            print(
                f"ThreadedClient port {self.team_port}: Queuing command for robot {robot_id}: vx={vx:.3f}, vy={vy:.3f}, w={angular:.3f}"
            )

        self.command_queue.put(
            CommandData(
                robot_id, "global", move_values=move_vals, kick_values=kick_vals
            )
        )

    def get_stats(self):
        """Get command statistics"""
        return {
            "sent_commands": self.sent_commands,
            "failed_commands": self.failed_commands,
            "success_rate": self._get_success_rate(),
            "last_command_time": self.last_command_time,
        }


def main():

    debug = True

    try:
        # Test clients
        blue_client = RobotControlClient(team_port=10301, debugger=debug)
        yellow_client = RobotControlClient(team_port=10302, debugger=debug)

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
        print("Blue Team:", blue_client.get_stats())
        print("Yellow Team:", yellow_client.get_stats())
        print("============================")
        print("[DEBUG] Stopping robot control test")


if __name__ == "__main__":
    main()
