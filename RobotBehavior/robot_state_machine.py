import math
from threading import Lock
from typing import Tuple, Dict, Optional, List
from .robot_states import RobotState, RobotRole

DEBUG_ROBOT_BEHAVIOR = False


class PathFollower:
    """Manages following a path smoothly"""

    def __init__(self, lookahead_distance=0.2):
        self.lookahead_distance = lookahead_distance
        print(f"PathFollower initialized with lookahead distance: {lookahead_distance}")

    def get_target_point(
        self, current_pos: Tuple[float, float], path: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """Get target point along path based on lookahead distance"""
        if not path:
            print("WARNING: Empty path provided to PathFollower")  # Not guarded
            return current_pos

        # If we're close to the end, return the end point
        end_distance = self._distance(current_pos, path[-1])
        if end_distance < self.lookahead_distance:
            print(
                f"Close to end point ({end_distance:.2f}m), targeting end directly"
            )  # Not guarded
            return path[-1]

        # Find the point along the path that's approximately lookahead_distance away
        for i in range(len(path)):
            if self._distance(current_pos, path[i]) >= self.lookahead_distance:
                if DEBUG_ROBOT_BEHAVIOR:
                    print(f"Found lookahead point at index {i} in path")  # Not guarded
                return path[i]

        # Default to last point
        print("No suitable point found, using last path point")  # Not guarded
        return path[-1]

    def _distance(
        self, point1: Tuple[float, float], point2: Tuple[float, float]
    ) -> float:
        """Calculate Euclidean distance between two points"""
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

    def _distance(
        self, point1: Tuple[float, float], point2: Tuple[float, float]
    ) -> float:
        """Calculate Euclidean distance between two points"""
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5


class RobotStateMachine:
    def __init__(self, robot_id: int, team_color: str, game):
        self.robot_id = robot_id
        self.team_color = team_color
        self.game = game
        self.current_state = RobotState.IDLE
        self.role = None
        self.target_position: Optional[Tuple[float, float]] = None
        self.state_lock = Lock()
        self.path_follower = PathFollower(lookahead_distance=0.3)
        self.last_path = []
        self.path_replan_time = 0  # Track when we last requested a path
        self.replan_threshold = 0.5  # Seconds between replanning

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

    def _get_current_orientation(self) -> Optional[float]:
        """Get current robot orientation from vision"""
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return None

        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robot_data = vision_data.get(team_key, {}).get(self.robot_id)

        if robot_data and robot_data["theta"] is not None:
            return robot_data["theta"]
        return None

    def update(self, vision_data: Dict):
        """Update robot state based on vision data"""
        with self.state_lock:
            self._decide_next_action(vision_data)
            self._execute_current_state(vision_data)

    def _decide_next_action(self, vision_data: Dict):
        """Robot decision making"""
        # Replace direct prints:
        # print(f"Attacker {self.robot_id} decision making...")

        # With conditional prints:
        if DEBUG_ROBOT_BEHAVIOR:
            print(f"Attacker {self.robot_id} decision making...")

        if not vision_data or "ball" not in vision_data:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"No vision data or ball. Returning home to {self.home_position}")
            return

        ball = vision_data["ball"]
        if ball["x"] is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Ball not detected. Returning home to {self.home_position}")
            return

        current_pos = self._get_current_pos()
        if not current_pos:
            if DEBUG_ROBOT_BEHAVIOR:
                print("Robot position not detected. Cannot make decisions.")
            return

        # Calculate distance to ball
        distance_to_ball = (
            (current_pos[0] - ball["x"]) ** 2 + (current_pos[1] - ball["y"]) ** 2
        ) ** 0.5

        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Ball at ({ball['x']:.2f}, {ball['y']:.2f}), distance: {distance_to_ball:.2f}"
            )
            print(f"Current position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")

    def _execute_current_state(self, vision_data: Dict):
        """Execute behavior for current state"""
        if self.current_state == RobotState.MOVING_TO_POSITION:
            current_pos = self._get_current_pos()
            if current_pos and self.target_position:
                # Your existing path planning code already does this!
                # Request path from planner (already queued in _decide_next_action)
                path = self.game.path_planner.get_path(self.robot_id)

                # If path is available, follow it
                if path and len(path) > 0:
                    # Get next waypoint (or use a more sophisticated path follower)
                    next_point = path[0] if len(path) > 1 else path[0]

                    # Calculate direction vector
                    dx = next_point[0] - current_pos[0]
                    dy = next_point[1] - current_pos[1]

                    # Calculate desired orientation
                    target_orientation = math.atan2(dy, dx)

                    # Get appropriate controller from game
                    controller = self.game.controllers[self.game.mode][self.team_color]

                    # Send velocity commands
                    controller.send_global_velocity(
                        self.robot_id, dx, dy, target_orientation
                    )

    def _should_replan(self, current_pos) -> bool:
        """Determine if path needs to be replanned"""
        import time

        # Check if enough time has passed since last planning
        current_time = time.time()
        if current_time - self.path_replan_time < self.replan_threshold:
            return False

        # Always replan if no path exists
        if not self.last_path:
            return True

        # Check if we're close to the goal
        if self.target_position:
            distance_to_goal = (
                (current_pos[0] - self.target_position[0]) ** 2
                + (current_pos[1] - self.target_position[1]) ** 2
            ) ** 0.5
            if distance_to_goal < 0.1:  # We're very close to goal
                return False

        # Get vision data to check for obstacles
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return False

        # Check if any robot has moved too close to our path
        for team in ["robotsBlue", "robotsYellow"]:
            for robot_id, robot in vision_data.get(team, {}).items():
                if int(robot_id) == self.robot_id or robot["x"] is None:
                    continue

                # Calculate distance to our path
                min_dist = float("inf")
                for point in self.last_path:
                    dist = (
                        (robot["x"] - point[0]) ** 2 + (robot["y"] - point[1]) ** 2
                    ) ** 0.5
                    min_dist = min(min_dist, dist)

                # If a robot is too close to our path, we should replan
                if min_dist < 0.3:  # Adjust threshold as needed
                    return True

        return False

    def _follow_path(self):
        """Follow the computed path using the path follower"""
        current_pos = self._get_current_pos()
        if not current_pos:
            return

        # Get current orientation
        current_orientation = self._get_current_orientation()
        if current_orientation is None:
            return

        # Get the latest path from the path planner
        path = self.game.path_planner.get_path(self.robot_id)

        # Check if we need to replan
        if self._should_replan(current_pos) and self.target_position:
            import time

            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.target_position
            )
            self.path_replan_time = time.time()

        # If we have a path, follow it
        if path and len(path) > 0:
            self.last_path = path
            # Get target point from path follower
            target_point = self.path_follower.get_target_point(
                current_pos, self.last_path
            )

            # Follow the path
            self._move_to_point(current_pos, current_orientation, target_point)
        else:
            # Fallback to direct movement if no path is available
            if self.target_position:
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"No path found for robot {self.robot_id}, using direct movement"
                    )
                self._move_to_point(
                    current_pos, current_orientation, self.target_position
                )

    def _move_to_point(self, current_pos, current_orientation, target_point):
        """Move to a target point with obstacle avoidance"""
        # Calculate direction vector
        dx = target_point[0] - current_pos[0]
        dy = target_point[1] - current_pos[1]

        # Calculate desired orientation
        target_orientation = math.atan2(dy, dx)

        # Calculate distance to target
        distance = math.sqrt(dx**2 + dy**2)

        # Scale velocity based on distance (slow down when close)
        max_speed = 1.0  # Maximum speed in m/s
        speed_factor = min(1.0, distance / 0.5)  # Slow down within 0.5m

        # Calculate velocities
        velocity_x = dx * max_speed * speed_factor
        velocity_y = dy * max_speed * speed_factor

        # Calculate angular velocity to align with target orientation
        angle_diff = target_orientation - current_orientation
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        angular_velocity = angle_diff * 2.0  # Proportional control

        # Get appropriate controller from game context
        controller = self.game.active_controller

        # Send velocity commands
        controller.send_global_velocity(
            self.robot_id, velocity_x, velocity_y, angular_velocity
        )


class GoalkeeperStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.GOALKEEPER
        # Define default positions based on team color
        self.home_position = (-2.0, 0.0) if team_color == "blue" else (2.0, 0.0)
        # Set target position initially to home position
        self.target_position = self.home_position

    def _decide_next_action(self, vision_data: Dict):
        """Goalkeeper decision making"""
        if not vision_data or "ball" not in vision_data:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            return

        ball = vision_data["ball"]
        if ball["x"] is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            return

        current_pos = self._get_current_pos()
        if not current_pos:
            return

        # Check if ball is threatening goal
        if self._is_ball_threatening(ball):
            self.current_state = RobotState.BLOCKING
            intercept_pos = self._calculate_intercept_position(ball)
            if intercept_pos:
                self.target_position = intercept_pos
                self.game.path_planner.request_path(
                    self.robot_id, current_pos, intercept_pos
                )
        else:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.home_position
            )

    def _is_ball_threatening(self, ball: Dict) -> bool:
        """Check if ball is threatening our goal"""
        if self.team_color == "blue":
            # Enhanced threat detection - consider ball position and motion
            return ball["x"] < -1.0  # Ball in our half
        return ball["x"] > 1.0  # Ball in our half for yellow team

    def _calculate_intercept_position(
        self, ball: Dict
    ) -> Optional[Tuple[float, float]]:
        """Calculate best position to intercept ball"""
        # Enhanced intercept calculation
        goal_line_x = -2.0 if self.team_color == "blue" else 2.0
        goal_height = 0.4  # Half the goal height

        # Constrain y position to stay within goal bounds
        ball_y = max(-goal_height, min(goal_height, ball["y"]))

        # Position goalkeeper on goal line in line with ball
        return (goal_line_x, ball_y)


class DefenderStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.DEFENDER
        self.home_position = (-1.0, 0.0) if team_color == "blue" else (1.0, 0.0)
        # Set target position initially to home position
        self.target_position = self.home_position

    def _decide_next_action(self, vision_data: Dict):
        """Defender decision making"""
        if not vision_data or "ball" not in vision_data:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            return

        ball = vision_data["ball"]
        if ball["x"] is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            return

        current_pos = self._get_current_pos()
        if not current_pos:
            return

        # Enhanced defense behavior
        if self._should_defend(ball):
            self.current_state = RobotState.MARKING
            defend_pos = self._calculate_defense_position(ball)
            self.target_position = defend_pos
            self.game.path_planner.request_path(self.robot_id, current_pos, defend_pos)
        else:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
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
        # Enhanced defensive positioning - position between ball and goal
        goal_x = -2.0 if self.team_color == "blue" else 2.0

        # Calculate position between ball and goal
        defense_x = (ball["x"] + goal_x) / 2

        # Ensure defender stays in our half
        if self.team_color == "blue":
            defense_x = min(
                -0.5, defense_x
            )  # Don't go beyond midfield and not too close to goal
        else:
            defense_x = max(
                0.5, defense_x
            )  # Don't go beyond midfield and not too close to goal

        return (defense_x, ball["y"])


class AttackerStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.ATTACKER
        # Define home position based on team color
        self.home_position = (0.5, 0.0) if team_color == "blue" else (-0.5, 0.0)
        # Set target position initially to home position
        self.target_position = self.home_position
        # Threshold for considering ball close enough to own
        self.ball_ownership_threshold = 0.2
        # For debug logging
        if DEBUG_ROBOT_BEHAVIOR:
            print(f"Initialized Attacker (ID: {robot_id}, Team: {team_color})")

    def _decide_next_action(self, vision_data: Dict):
        """Attacker decision making logic"""
        # Print debug info for this frame
        if DEBUG_ROBOT_BEHAVIOR:
            print(f"\nAttacker {self.robot_id} decision making...")

        if not vision_data or "ball" not in vision_data:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            print(f"No vision data or ball. Returning home to {self.home_position}")
            return

        ball = vision_data["ball"]
        if ball["x"] is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            print(f"Ball not detected. Returning home to {self.home_position}")
            return

        current_pos = self._get_current_pos()
        if not current_pos:
            print("Robot position not detected. Cannot make decisions.")
            return

        # Calculate distance to ball
        distance_to_ball = (
            (current_pos[0] - ball["x"]) ** 2 + (current_pos[1] - ball["y"]) ** 2
        ) ** 0.5

        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Ball at ({ball['x']:.2f}, {ball['y']:.2f}), distance: {distance_to_ball:.2f}"
            )
            print(f"Current position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")

        # Decide what to do based on situation
        if distance_to_ball < self.ball_ownership_threshold:
            # We have the ball - try to score
            self.current_state = RobotState.ATTACKING
            attack_pos = self._calculate_attack_position(ball)
            self.target_position = attack_pos

            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Ball owned. ATTACKING towards {attack_pos}")

            self.game.path_planner.request_path(self.robot_id, current_pos, attack_pos)

        elif self._should_go_to_ball(ball):
            # Go to ball
            self.current_state = RobotState.MOVING_TO_BALL
            self.target_position = (ball["x"], ball["y"])

            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Going to ball at {self.target_position}")
            self.game.path_planner.request_path(
                self.robot_id, current_pos, (ball["x"], ball["y"])
            )

        else:
            # Take supporting position
            self.current_state = RobotState.SUPPORTING
            support_pos = self._calculate_support_position(ball)
            self.target_position = support_pos

            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Taking support position at {support_pos}")
            self.game.path_planner.request_path(self.robot_id, current_pos, support_pos)

        # After deciding and requesting path, actually follow it
        self._follow_path()

    def _should_go_to_ball(self, ball: Dict) -> bool:
        """Decide if attacker should go to ball"""
        # In our half, always go to ball
        if (self.team_color == "blue" and ball["x"] > 0) or (
            self.team_color == "yellow" and ball["x"] < 0
        ):
            return True

        # Check if we're the closest robot on our team to the ball
        vision_data = self.game.get_vision_data()
        current_pos = self._get_current_pos()
        if not vision_data or not current_pos:
            return False

        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"

        # Calculate our distance to ball
        our_distance = (
            (current_pos[0] - ball["x"]) ** 2 + (current_pos[1] - ball["y"]) ** 2
        ) ** 0.5

        # Check if any teammates are closer
        for robot_id, robot in vision_data.get(team_key, {}).items():
            if int(robot_id) == self.robot_id or robot["x"] is None:
                continue

            teammate_distance = (
                (robot["x"] - ball["x"]) ** 2 + (robot["y"] - ball["y"]) ** 2
            ) ** 0.5

            if teammate_distance < our_distance:
                return False  # Someone else is closer

        return True  # We're the closest

    def _calculate_attack_position(self, ball: Dict) -> Tuple[float, float]:
        """Calculate position for attacking with ball"""
        # Simple version - head toward opponent goal
        goal_x = 2.0 if self.team_color == "blue" else -2.0
        goal_y = 0.0

        # Vector from ball to goal
        dx = goal_x - ball["x"]
        dy = goal_y - ball["y"]

        # Normalize vector
        dist = (dx**2 + dy**2) ** 0.5
        if dist > 0:
            dx /= dist
            dy /= dist

        # Position slightly behind ball on path to goal
        # This will make the robot approach the ball from the right angle
        behind_x = ball["x"] - dx * 0.2
        behind_y = ball["y"] - dy * 0.2

        return (behind_x, behind_y)

    def _calculate_support_position(self, ball: Dict) -> Tuple[float, float]:
        """Calculate support position when not directly attacking"""
        # Take up position on attacking half, offset from ball
        if self.team_color == "blue":
            support_x = min(1.0, ball["x"] + 0.5)  # Ahead of ball but not too far

            # If ball is on one side, position on the other side
            if ball["y"] > 0:
                support_y = ball["y"] - 0.5
            else:
                support_y = ball["y"] + 0.5
        else:
            support_x = max(-1.0, ball["x"] - 0.5)  # Ahead of ball but not too far

            # If ball is on one side, position on the other side
            if ball["y"] > 0:
                support_y = ball["y"] - 0.5
            else:
                support_y = ball["y"] + 0.5

        return (support_x, support_y)
