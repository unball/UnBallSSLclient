# flake8: noqa

import socket
import sys

try:
    import msvcrt  # For Windows
except ImportError:
    import tty
    import termios  # For Unix-based systems

# Import the necessary proto files
from proto import ssl_simulation_control_pb2 as sim_control
from proto import ssl_gc_common_pb2 as gc_common

class SSLSimulationClient:
    def __init__(self, address='127.0.0.1', port=10300):
        self.address = address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_port_and_address(self, port, address):
        self.port = port
        self.address = address

    def send_command(self, command):
        self.socket.sendto(command.SerializeToString(), (self.address, self.port))

    def close(self):
        self.socket.close()

simulator = SSLSimulationClient()

def list_functions():
    return [
        "1. Move robot",
        "2. Move ball",
        "3. Set simulation speed",
        "4. Set port and address",
        "5. Control ball with WASD",
        "6. Quit"
    ]

def move_robot():
    command = sim_control.SimulatorCommand()
    robot = command.control.teleport_robot.add()
    
    team = input("Enter team (yellow/blue): ").lower()
    robot.id.team = gc_common.YELLOW if team == "yellow" else gc_common.BLUE
    robot.id.id = int(input("Enter robot ID: "))
    robot.x = float(input("Enter x position: "))
    robot.y = float(input("Enter y position: "))
    robot.orientation = float(input("Enter orientation (radians): "))
    
    simulator.send_command(command)
    print(f"Command sent: Move robot {robot.id.id} to ({robot.x}, {robot.y}) with orientation {robot.orientation}")

def move_ball():
    command = sim_control.SimulatorCommand()
    ball = command.control.teleport_ball
    
    ball.x = float(input("Enter x position: "))
    ball.y = float(input("Enter y position: "))
    ball.z = float(input("Enter z position: "))
    ball.vx = float(input("Enter x velocity: "))
    ball.vy = float(input("Enter y velocity: "))
    ball.vz = float(input("Enter z velocity: "))
    
    simulator.send_command(command)
    print(f"Command sent: Move ball to ({ball.x}, {ball.y}, {ball.z}) with velocity ({ball.vx}, {ball.vy}, {ball.vz})")

def set_simulation_speed():
    command = sim_control.SimulatorCommand()
    command.control.simulation_speed = float(input("Enter simulation speed (1.0 is real-time): "))
    
    simulator.send_command(command)
    print(f"Command sent: Set simulation speed to {command.control.simulation_speed}")

def set_port_and_address():
    port = int(input("Enter port number: "))
    address = input("Enter IP address: ")
    simulator.set_port_and_address(port, address)
    print(f"Port and address updated to {port} and {address}")

def get_key():
    if sys.platform.startswith('win'):
        return msvcrt.getch().decode('utf-8').lower()
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def control_ball_wasd():
    print("Controlling ball with WASD. Press 'q' to quit.")
    print("W/S: Move forward/backward")
    print("A/D: Move left/right")
    print("Space: Stop ball")

    ball_x, ball_y, ball_z = 0, 0, 0.1  # Starting position
    speed = 0.5  # meters per second

    while True:
        key = get_key()
        
        if key == 'w':
            ball_y += speed
        elif key == 's':
            ball_y -= speed
        elif key == 'a':
            ball_x -= speed
        elif key == 'd':
            ball_x += speed
        elif key == ' ':
            speed = 0
        elif key == 'q':
            break

        command = sim_control.SimulatorCommand()
        ball = command.control.teleport_ball
        ball.x = ball_x
        ball.y = ball_y
        ball.z = ball_z
        ball.vx = 0
        ball.vy = 0
        ball.vz = 0

        simulator.send_command(command)
        print(f"Ball position: ({ball_x:.2f}, {ball_y:.2f}, {ball_z:.2f})", end='\r')

    print("\nExited WASD control mode.")

def main():
    while True:
        print("\nAvailable functions:")
        for func in list_functions():
            print(func)
        
        choice = input("Enter your choice (1-6): ")
        
        if choice == '1':
            move_robot()
        elif choice == '2':
            move_ball()
        elif choice == '3':
            set_simulation_speed()
        elif choice == '4':
            set_port_and_address()
        elif choice == '5':
            control_ball_wasd()
        elif choice == '6':
            print("Exiting...")
            simulator.close()
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()