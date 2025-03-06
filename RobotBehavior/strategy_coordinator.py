# strategy_coordinator.py
class StrategyCoordinator:
    def __init__(self, game):
        self.game = game
        self.state_machines = {}

    def initialize_robots(self, team_color, robot_roles):
        """Initialize robot state machines based on assigned roles"""
        for robot_id, role in robot_roles.items():
            if role == RobotRole.GOALKEEPER:
                self.state_machines[robot_id] = GoalkeeperStateMachine(
                    robot_id, team_color, self.game
                )
            elif role == RobotRole.DEFENDER:
                self.state_machines[robot_id] = DefenderStateMachine(
                    robot_id, team_color, self.game
                )
            elif role == RobotRole.ATTACKER:
                self.state_machines[robot_id] = AttackerStateMachine(
                    robot_id, team_color, self.game
                )

    def update(self, vision_data, referee_data):
        """Update all robot state machines"""
        for robot_id, state_machine in self.state_machines.items():
            state_machine.update(vision_data)
