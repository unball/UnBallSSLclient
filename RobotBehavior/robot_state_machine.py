from threading import Lock
from typing import Tuple, Dict, Optional
from .robot_states import RobotState, RobotRole


class RobotStateMachine:
    def __init__(self, robot_id: int, team_color: str, game):
        self.robot_id = robot_id
        self.team_color = team_color
        self.game = game
        self.current_state = RobotState.IDLE
        self.role = None
        self.target_position: Optional[Tuple[float, float]] = None
        self.state_lock = Lock()

    def _get_current_pos(self) -> Optional[Tuple[float, float]]:
        """Get current robot position from vision"""
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return None

        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robot_data = vision_data.get(team_key, {}).get(self.robot_id)

        if robot_data and robot_data["x"] is not None:
            return (robot_data["x"], robot_data["y"])
        return None

    def update(self, vision_data: Dict):
        """Update robot state based on vision data"""
        with self.state_lock:
            self._decide_next_action(vision_data)
            self._execute_current_state(vision_data)

    def _decide_next_action(self, vision_data: Dict):
        """To be implemented by specific roles"""
        pass

    def _execute_current_state(self, vision_data: Dict):
        """Execute behavior for current state"""
        pass


class GoalkeeperStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.GOALKEEPER
        # Define default positions based on team color
        self.home_position = (-2.0, 0.0) if team_color == "blue" else (2.0, 0.0)

    def _decide_next_action(self, vision_data: Dict):
        """Goalkeeper decision making"""
        if not vision_data or "ball" not in vision_data:
            self.current_state = RobotState.RETURNING
            return

        ball = vision_data["ball"]
        if ball["x"] is None:
            self.current_state = RobotState.RETURNING
            return

        current_pos = self._get_current_pos()
        if not current_pos:
            return

        # Check if ball is threatening goal
        if self._is_ball_threatening(ball):
            self.current_state = RobotState.BLOCKING
            intercept_pos = self._calculate_intercept_position(ball)
            if intercept_pos:
                self.game.path_planner.request_path(
                    self.robot_id, current_pos, intercept_pos
                )
        else:
            self.current_state = RobotState.RETURNING
            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.home_position
            )

    def _is_ball_threatening(self, ball: Dict) -> bool:
        """Check if ball is threatening our goal"""
        if self.team_color == "blue":
            return ball["x"] < -1.0  # Ball in our half
        return ball["x"] > 1.0  # Ball in our half for yellow team

    def _calculate_intercept_position(
        self, ball: Dict
    ) -> Optional[Tuple[float, float]]:
        """Calculate best position to intercept ball"""
        # Simple implementation - stay in line with ball
        if self.team_color == "blue":
            return (-2.0, ball["y"])
        return (2.0, ball["y"])


class DefenderStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.DEFENDER
        self.home_position = (-1.0, 0.0) if team_color == "blue" else (1.0, 0.0)

    def _decide_next_action(self, vision_data: Dict):
        """Defender decision making"""
        if not vision_data or "ball" not in vision_data:
            self.current_state = RobotState.RETURNING
            return

        ball = vision_data["ball"]
        if ball["x"] is None:
            self.current_state = RobotState.RETURNING
            return

        current_pos = self._get_current_pos()
        if not current_pos:
            return

        # Simple defense behavior
        if self._should_defend(ball):
            self.current_state = RobotState.MARKING
            defend_pos = self._calculate_defense_position(ball)
            self.game.path_planner.request_path(self.robot_id, current_pos, defend_pos)
        else:
            self.current_state = RobotState.RETURNING
            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.home_position
            )

    def _should_defend(self, ball: Dict) -> bool:
        """Decide if defender should move to defend"""
        if self.team_color == "blue":
            return ball["x"] < 0  # Ball in our half
        return ball["x"] > 0  # Ball in our half for yellow team

    def _calculate_defense_position(self, ball: Dict) -> Tuple[float, float]:
        """Calculate best defensive position"""
        # Simple implementation - stay between ball and goal
        if self.team_color == "blue":
            return (-1.0, ball["y"])
        return (1.0, ball["y"])
