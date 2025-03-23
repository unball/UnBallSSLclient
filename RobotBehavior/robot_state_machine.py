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
        """Update robot state based on vision data and game state"""
        with self.state_lock:
            # Check game state first
            game_state = None
            team_for_action = None

            # Get current game state from referee data
            referee_data = self.game.get_referee_data()
            if referee_data:
                game_state = referee_data.get("command")
                team_for_action = referee_data.get("team")
                can_play = referee_data.get("can_play", False)

                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"Robot {self.robot_id} update - Game state: {game_state}, Team: {team_for_action}, Can play: {can_play}"
                    )

                # Check if we're in a pre-kickoff state
                pre_kickoff_states = [
                    "HALT",
                    "STOP",
                    "PREPARE_KICKOFF_BLUE",
                    "PREPARE_KICKOFF_YELLOW",
                ]
                if game_state in pre_kickoff_states:
                    # Get robot position
                    current_pos = self._get_current_pos()
                    if current_pos:
                        # Define halfway line
                        halfway_line = 0

                        # Enforce robots to stay in their own half
                        if self.team_color == "blue" and current_pos[0] > halfway_line:
                            self.current_state = RobotState.RETURNING
                            self.target_position = (-0.5, current_pos[1])
                            self.game.path_planner.request_path(
                                self.robot_id, current_pos, self.target_position
                            )
                            self._follow_path()
                            return
                        elif (
                            self.team_color == "yellow"
                            and current_pos[0] < halfway_line
                        ):
                            self.current_state = RobotState.RETURNING
                            self.target_position = (0.5, current_pos[1])
                            self.game.path_planner.request_path(
                                self.robot_id, current_pos, self.target_position
                            )
                            self._follow_path()
                            return

                # Handle specific game states
                if game_state == "HALT":
                    # Immediately stop robot
                    if DEBUG_ROBOT_BEHAVIOR:
                        print(f"Robot {self.robot_id}: HALT - Stopping immediately")
                    if hasattr(self.game, "active_controller"):
                        self.game.active_controller.send_global_velocity(
                            self.robot_id, 0, 0, 0
                        )
                    self.current_state = RobotState.IDLE
                    return

                elif game_state == "STOP":
                    # Can move but stay away from ball
                    if DEBUG_ROBOT_BEHAVIOR:
                        print(f"Robot {self.robot_id}: STOP - Moving away from ball")
                    self.current_state = RobotState.RETURNING
                    self.target_position = self.home_position

                    # Get ball position to ensure we stay away
                    if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                        ball_x = vision_data["ball"]["x"]
                        ball_y = vision_data["ball"]["y"]
                        current_pos = self._get_current_pos()

                        if current_pos:
                            # Check if we're too close to ball (within 0.5m)
                            distance_to_ball = math.sqrt(
                                (current_pos[0] - ball_x) ** 2
                                + (current_pos[1] - ball_y) ** 2
                            )

                            if distance_to_ball < 0.5:
                                # Move away from ball
                                away_vector_x = current_pos[0] - ball_x
                                away_vector_y = current_pos[1] - ball_y

                                # Normalize vector
                                vector_length = math.sqrt(
                                    away_vector_x**2 + away_vector_y**2
                                )
                                if vector_length > 0:
                                    away_vector_x /= vector_length
                                    away_vector_y /= vector_length

                                # Set target position away from ball
                                self.target_position = (
                                    current_pos[0] + away_vector_x * 0.6,
                                    current_pos[1] + away_vector_y * 0.6,
                                )
                                if DEBUG_ROBOT_BEHAVIOR:
                                    print(
                                        f"Robot {self.robot_id}: Moving away from ball to {self.target_position}"
                                    )

                    # Update path with new target
                    if current_pos and self.target_position:
                        self.game.path_planner.request_path(
                            self.robot_id, current_pos, self.target_position
                        )

                    # Follow path to safe position
                    self._follow_path()
                    return

                elif game_state in [
                    "KICK_OFF",
                    "FREE_KICK",
                    "PENALTY",
                    "GOAL_KICK",
                    "CORNER_KICK",
                    "BALL_PLACEMENT",
                ]:
                    # Check if this command is for our team
                    is_our_team_turn = team_for_action == self.team_color

                    if is_our_team_turn:
                        if DEBUG_ROBOT_BEHAVIOR:
                            print(f"Robot {self.robot_id}: {game_state} for our team")
                        # Handle based on robot role (will be implemented in subclasses)
                        # Let normal decision logic run
                    else:
                        if DEBUG_ROBOT_BEHAVIOR:
                            print(
                                f"Robot {self.robot_id}: {game_state} for opponent team - keeping distance"
                            )
                        # We need to keep distance (0.5m) from ball during opponent's turn
                        self.current_state = RobotState.SUPPORTING

                        # Make sure we're not too close to the ball
                        if (
                            "ball" in vision_data
                            and vision_data["ball"]["x"] is not None
                        ):
                            ball_x = vision_data["ball"]["x"]
                            ball_y = vision_data["ball"]["y"]
                            current_pos = self._get_current_pos()

                            if current_pos:
                                # Check if we're too close to ball
                                distance_to_ball = math.sqrt(
                                    (current_pos[0] - ball_x) ** 2
                                    + (current_pos[1] - ball_y) ** 2
                                )

                                if distance_to_ball < 0.5:
                                    # Move away from ball
                                    away_vector_x = current_pos[0] - ball_x
                                    away_vector_y = current_pos[1] - ball_y

                                    # Normalize vector
                                    vector_length = math.sqrt(
                                        away_vector_x**2 + away_vector_y**2
                                    )
                                    if vector_length > 0:
                                        away_vector_x /= vector_length
                                        away_vector_y /= vector_length

                                    # Set target position away from ball
                                    self.target_position = (
                                        current_pos[0] + away_vector_x * 0.6,
                                        current_pos[1] + away_vector_y * 0.6,
                                    )

                                    # Update path planning
                                    self.game.path_planner.request_path(
                                        self.robot_id, current_pos, self.target_position
                                    )

                                    # Follow path to safe position
                                    self._follow_path()
                                    return

            # If no game state interruption, proceed with normal decision making
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

    def _enforce_field_boundaries(self, target_position):
        """Ensure target position is within field boundaries with safety margin"""
        if not hasattr(self.game, "field_bounds") or not self.game.field_bounds:
            return target_position  # No boundaries defined, return original target

        # Get field boundaries
        field_bounds = self.game.field_bounds

        # Safety margin from field edges (in meters)
        margin = 0.2

        # Calculate constrained target position
        x = max(
            field_bounds.get("x_min", -2.25) + margin,
            min(field_bounds.get("x_max", 2.25) - margin, target_position[0]),
        )
        y = max(
            field_bounds.get("y_min", -1.5) + margin,
            min(field_bounds.get("y_max", 1.5) - margin, target_position[1]),
        )

        # If position was changed, log it
        if (x, y) != target_position and DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Boundary enforcement: Robot {self.robot_id} target adjusted from {target_position} to ({x:.2f}, {y:.2f})"
            )

        return (x, y)

    def _move_to_point(self, current_pos, current_orientation, target_point):
        """Move to a target point with velocity scaling based on curvature"""
        # Apply boundary enforcement to target point
        safe_target = self._enforce_field_boundaries(target_point)

        # Calculate direction vector
        dx = safe_target[0] - current_pos[0]
        dy = safe_target[1] - current_pos[1]

        # Calculate desired orientation
        target_orientation = math.atan2(dy, dx)

        # Calculate distance to target
        distance = math.sqrt(dx**2 + dy**2)

        # Scale velocity based on distance (slow down when close)
        max_speed = 1.0  # Maximum speed in m/s
        speed_factor = min(1.0, distance / 0.5)  # Slow down within 0.5m

        # Add curvature-based speed scaling
        # Calculate angle change required
        angle_diff = abs(
            self._normalize_angle(target_orientation - current_orientation)
        )

        # Scale speed based on angle change (more turning = slower speed)
        curvature_factor = max(0.3, 1.0 - angle_diff / math.pi)

        # Apply both scaling factors
        final_speed_factor = min(speed_factor, curvature_factor)

        # Calculate velocities
        velocity_x = dx * max_speed * final_speed_factor
        velocity_y = dy * max_speed * final_speed_factor

        # Safety check: If robot is already at or beyond field boundary, prevent further movement in that direction
        if hasattr(self.game, "field_bounds"):
            field_bounds = self.game.field_bounds
            margin = 0.05  # Smaller margin for immediate reaction

            # Check if robot is near or beyond field boundaries
            if (
                current_pos[0] <= field_bounds.get("x_min", -2.25) + margin
                and velocity_x < 0
            ):
                velocity_x = 0  # Don't move further in negative x
            if (
                current_pos[0] >= field_bounds.get("x_max", 2.25) - margin
                and velocity_x > 0
            ):
                velocity_x = 0  # Don't move further in positive x
            if (
                current_pos[1] <= field_bounds.get("y_min", -1.5) + margin
                and velocity_y < 0
            ):
                velocity_y = 0  # Don't move further in negative y
            if (
                current_pos[1] >= field_bounds.get("y_max", 1.5) - margin
                and velocity_y > 0
            ):
                velocity_y = 0  # Don't move further in positive y

        # Calculate angular velocity to align with target orientation
        angle_diff = self._normalize_angle(target_orientation - current_orientation)
        angular_velocity = angle_diff * 2.0  # Proportional control

        # Get appropriate controller from game context
        controller = self.game.active_controller

        # Send velocity commands
        controller.send_global_velocity(
            self.robot_id, velocity_x, velocity_y, angular_velocity
        )

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class GoalkeeperStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.GOALKEEPER
        # Define default positions based on team color
        self.home_position = (-2.0, 0.0) if team_color == "blue" else (2.0, 0.0)
        # Set target position initially to home position
        self.target_position = self.home_position

    def _decide_next_action(self, vision_data: Dict):
        """Goalkeeper decision making with game state awareness"""
        # Add debug logging
        if DEBUG_ROBOT_BEHAVIOR:
            self.debug_robot_state(vision_data)

        # Get referee data to check game state
        referee_data = self.game.get_referee_data()
        game_state = None
        team_for_action = None

        if referee_data:
            game_state = referee_data.get("command")
            team_for_action = referee_data.get("team")
            can_play = referee_data.get("can_play", False)

        # For most game states, goalkeeper stays in goal
        # But we need special handling for penalty against us
        if game_state == "PENALTY" and team_for_action != self.team_color:
            # Opposing team's penalty - goalkeeper must stay on goal line
            print(f"Goalkeeper {self.robot_id}: Positioning for PENALTY defense")
            self.current_state = RobotState.BLOCKING

            # Position goalkeeper on goal line
            goal_line_x = -2.0 if self.team_color == "blue" else 2.0

            # Get ball position to align with
            if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                ball_y = vision_data["ball"]["y"]
                # Constrain to goal width
                goal_width = 0.8  # From SSL-EL rules
                ball_y = max(-goal_width / 2, min(goal_width / 2, ball_y))

                self.target_position = (goal_line_x, ball_y)

                current_pos = self._get_current_pos()
                if current_pos:
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
            return

        # For goal kick by our team
        if game_state == "GOAL_KICK" and team_for_action == self.team_color:
            # Our goal kick - goalkeeper should position at the ball
            print(f"Goalkeeper {self.robot_id}: Positioning for GOAL_KICK")
            self.current_state = RobotState.MOVING_TO_POSITION

            # Get ball position
            if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                ball_x = vision_data["ball"]["x"]
                ball_y = vision_data["ball"]["y"]

                # Position slightly behind ball
                offset_x = -0.2 if self.team_color == "blue" else 0.2
                self.target_position = (ball_x + offset_x, ball_y)

                current_pos = self._get_current_pos()
                if current_pos:
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
            return

        # Regular goalkeeper behavior for normal gameplay
        if not vision_data or "ball" not in vision_data:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            # Force path planning to home position
            current_pos = self._get_current_pos()
            if current_pos:
                self.game.path_planner.request_path(
                    self.robot_id, current_pos, self.home_position
                )
            return

        ball = vision_data["ball"]
        if ball["x"] is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            # Force path planning to home position
            current_pos = self._get_current_pos()
            if current_pos:
                self.game.path_planner.request_path(
                    self.robot_id, current_pos, self.home_position
                )
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
                # Force path planning to intercept position
                self.game.path_planner.request_path(
                    self.robot_id, current_pos, intercept_pos
                )
                print(f"Goalkeeper {self.robot_id}: Moving to block at {intercept_pos}")
        else:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            # Force path planning to home position
            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.home_position
            )
            print(
                f"Goalkeeper {self.robot_id}: Returning to home at {self.home_position}"
            )

        # Force path following
        self._follow_path()

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

    def debug_robot_state(self, vision_data):
        """Debug function to log robot state and decisions"""
        current_pos = self._get_current_pos()

        print(f"\n==== {self.role.value} {self.robot_id} DEBUG ====")
        print(f"Team Color: {self.team_color}")

        if current_pos:
            print(f"Current Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
        else:
            print("Current Position: None - Robot not detected")

        print(f"Current State: {self.current_state.value}")

        if self.target_position:
            print(
                f"Target Position: ({self.target_position[0]:.2f}, "
                f"{self.target_position[1]:.2f})"
            )
        else:
            print("Target Position: None")

        # Check if a path is being planned/followed
        path = self.game.path_planner.get_path(self.robot_id)
        if path:
            print(f"Path Length: {len(path)} points")
            if len(path) > 0:
                print(f"First Path Point: ({path[0][0]:.2f}, {path[0][1]:.2f})")
            if len(path) > 1:
                print(f"Last Path Point: ({path[-1][0]:.2f}, {path[-1][1]:.2f})")
        else:
            print("Path: None")

        # Check ball data
        if "ball" in vision_data and vision_data["ball"]["x"] is not None:
            ball = vision_data["ball"]
            print(f"Ball Position: ({ball['x']:.2f}, {ball['y']:.2f})")
        else:
            print("Ball: Not detected")

        # Check referee state
        referee_data = self.game.get_referee_data()
        if referee_data:
            print(f"Referee Command: {referee_data.get('command', 'None')}")
            print(f"Can Play: {referee_data.get('can_play', False)}")
        else:
            print("No referee data available")

        print("==============================")


class DefenderStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.DEFENDER
        self.home_position = (-1.0, 0.0) if team_color == "blue" else (1.0, 0.0)
        # Set target position initially to home position
        self.target_position = self.home_position

    def _decide_next_action(self, vision_data: Dict):
        """Defender decision making with game state awareness"""
        # Get referee data to check game state
        referee_data = self.game.get_referee_data()
        game_state = None
        team_for_action = None

        if referee_data:
            game_state = referee_data.get("command")
            team_for_action = referee_data.get("team")
            can_play = referee_data.get("can_play", False)

        # For defensive plays when our team has the action
        if game_state and team_for_action == self.team_color:
            if game_state == "KICK_OFF":
                # Defender should position in defensive half
                print(f"Defender {self.robot_id}: Positioning for KICK-OFF")
                self.current_state = RobotState.MOVING_TO_POSITION

                # Position defender in defensive half
                x_pos = -1.0 if self.team_color == "blue" else 1.0
                self.target_position = (x_pos, 0.5)  # Offset from center

                current_pos = self._get_current_pos()
                if current_pos:
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
                return

            elif game_state == "FREE_KICK":
                # Defender should position to support attack
                print(f"Defender {self.robot_id}: Supporting FREE-KICK")
                self.current_state = RobotState.SUPPORTING

                # Get ball position
                if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                    ball_x = vision_data["ball"]["x"]
                    ball_y = vision_data["ball"]["y"]

                    # Position behind ball for support
                    offset_x = -0.5 if self.team_color == "blue" else 0.5
                    self.target_position = (ball_x + offset_x, ball_y)

                    current_pos = self._get_current_pos()
                    if current_pos:
                        self.game.path_planner.request_path(
                            self.robot_id, current_pos, self.target_position
                        )
                        self._follow_path()
                return

            elif game_state == "PENALTY":
                # Defender should clear the way for penalty taker
                print(f"Defender {self.robot_id}: Clearing for PENALTY")
                self.current_state = RobotState.SUPPORTING

                # Position away from penalty area
                x_pos = -1.0 if self.team_color == "blue" else 1.0
                self.target_position = (x_pos, -1.0)  # Away from the action

                current_pos = self._get_current_pos()
                if current_pos:
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
                return

            elif game_state in ["CORNER_KICK", "GOAL_KICK"]:
                # Defender should position for these kicks
                print(f"Defender {self.robot_id}: Positioning for {game_state}")
                self.current_state = RobotState.SUPPORTING

                if game_state == "CORNER_KICK":
                    # For corner kick, position in the box
                    x_pos = 1.5 if self.team_color == "blue" else -1.5
                    self.target_position = (x_pos, 0)  # In front of goal
                else:  # GOAL_KICK
                    # For goal kick, position to receive from goalkeeper
                    x_pos = -1.5 if self.team_color == "blue" else 1.5
                    self.target_position = (x_pos, 0.5)

                current_pos = self._get_current_pos()
                if current_pos:
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
                return

        # For opposing team's actions, position defensively
        elif game_state and team_for_action != self.team_color:
            if game_state in ["KICK_OFF", "FREE_KICK", "CORNER_KICK"]:
                # Defender should take defensive position
                print(
                    f"Defender {self.robot_id}: Defensive position for opponent {game_state}"
                )
                self.current_state = RobotState.DEFENDING

                # Position defensively between ball and our goal
                if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                    ball_x = vision_data["ball"]["x"]
                    ball_y = vision_data["ball"]["y"]

                    # Our goal position
                    goal_x = -2.0 if self.team_color == "blue" else 2.0
                    goal_y = 0.0

                    # Position between ball and goal
                    self.target_position = (
                        (ball_x + goal_x) / 2,
                        (ball_y + goal_y) / 2,
                    )

                    current_pos = self._get_current_pos()
                    if current_pos:
                        self.game.path_planner.request_path(
                            self.robot_id, current_pos, self.target_position
                        )
                        self._follow_path()
                return

        # Regular defender behavior for normal gameplay
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

            # Force path planning to defense position
            self.game.path_planner.request_path(self.robot_id, current_pos, defend_pos)
            print(f"Defender {self.robot_id}: Moving to defend at {defend_pos}")
        else:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position

            # Force path planning to home position
            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.home_position
            )
            print(
                f"Defender {self.robot_id}: Returning to home at {self.home_position}"
            )

        # Follow the path
        self._follow_path()

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

    def debug_robot_state(self, vision_data):
        """Debug function to log robot state and decisions"""
        current_pos = self._get_current_pos()

        print(f"\n==== {self.role.value} {self.robot_id} DEBUG ====")
        print(f"Team Color: {self.team_color}")

        if current_pos:
            print(f"Current Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
        else:
            print("Current Position: None - Robot not detected")

        print(f"Current State: {self.current_state.value}")

        if self.target_position:
            print(
                f"Target Position: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f})"
            )
        else:
            print("Target Position: None")

        # Check if a path is being planned/followed
        path = self.game.path_planner.get_path(self.robot_id)
        if path:
            print(f"Path Length: {len(path)} points")
            if len(path) > 0:
                print(f"First Path Point: ({path[0][0]:.2f}, {path[0][1]:.2f})")
            if len(path) > 1:
                print(f"Last Path Point: ({path[-1][0]:.2f}, {path[-1][1]:.2f})")
        else:
            print("Path: None")

        # Check ball data
        if "ball" in vision_data and vision_data["ball"]["x"] is not None:
            ball = vision_data["ball"]
            print(f"Ball Position: ({ball['x']:.2f}, {ball['y']:.2f})")
        else:
            print("Ball: Not detected")

        # Check referee state
        referee_data = self.game.get_referee_data()
        if referee_data:
            print(f"Referee Command: {referee_data.get('command', 'None')}")
            print(f"Can Play: {referee_data.get('can_play', False)}")
        else:
            print("No referee data available")

        print("==============================")


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
        """Attacker decision making with game state awareness"""
        # Get referee data to check game state
        referee_data = self.game.get_referee_data()
        game_state = None
        team_for_action = None

        if referee_data:
            game_state = referee_data.get("command")
            team_for_action = referee_data.get("team")
            can_play = referee_data.get("can_play", False)

        # Special handling for offensive actions that our team executes
        if game_state and team_for_action == self.team_color:
            if game_state == "KICK_OFF":
                # Attacker should position for kickoff
                print(f"Attacker {self.robot_id}: Positioning for KICK-OFF")
                self.current_state = RobotState.MOVING_TO_POSITION

                # Position attacker near center circle to take kickoff
                self.target_position = (0, 0)

                current_pos = self._get_current_pos()
                if current_pos:
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
                return

            elif game_state == "FREE_KICK":
                # Attacker should take free kick if ball is in forward half
                print(f"Attacker {self.robot_id}: Positioning for FREE-KICK")
                self.current_state = RobotState.MOVING_TO_BALL

                # Set target to ball position
                if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                    ball_x = vision_data["ball"]["x"]
                    ball_y = vision_data["ball"]["y"]

                    # Position attacker near ball to take free kick
                    # Position slightly behind ball in direction of goal
                    goal_x = 2.0 if self.team_color == "blue" else -2.0
                    goal_y = 0.0

                    # Vector from ball to goal
                    dx = goal_x - ball_x
                    dy = goal_y - ball_y

                    # Normalize vector
                    dist = (dx**2 + dy**2) ** 0.5
                    if dist > 0:
                        dx /= dist
                        dy /= dist

                    # Position behind ball on path to goal
                    self.target_position = (ball_x - dx * 0.2, ball_y - dy * 0.2)

                    current_pos = self._get_current_pos()
                    if current_pos:
                        self.game.path_planner.request_path(
                            self.robot_id, current_pos, self.target_position
                        )
                        self._follow_path()
                return

            elif game_state == "PENALTY":
                # Attacker should take penalty if assigned
                print(f"Attacker {self.robot_id}: Positioning for PENALTY")
                self.current_state = RobotState.MOVING_TO_POSITION

                # Get penalty mark position
                if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                    ball_x = vision_data["ball"]["x"]
                    ball_y = vision_data["ball"]["y"]

                    # Position attacker behind penalty mark
                    # The ball should be on the penalty mark
                    goal_x = 2.0 if self.team_color == "blue" else -2.0

                    # Position slightly behind ball
                    offset_x = -0.2 if self.team_color == "blue" else 0.2
                    self.target_position = (ball_x + offset_x, ball_y)

                    current_pos = self._get_current_pos()
                    if current_pos:
                        self.game.path_planner.request_path(
                            self.robot_id, current_pos, self.target_position
                        )
                        self._follow_path()
                return

            elif game_state in ["CORNER_KICK", "GOAL_KICK"]:
                # Attacker should position strategically for these kicks
                print(f"Attacker {self.robot_id}: Positioning for {game_state}")
                self.current_state = RobotState.MOVING_TO_POSITION

                # Get ball position
                if "ball" in vision_data and vision_data["ball"]["x"] is not None:
                    ball_x = vision_data["ball"]["x"]
                    ball_y = vision_data["ball"]["y"]

                    # For corner kick, position near ball
                    if game_state == "CORNER_KICK":
                        # Position slightly offset from ball
                        offset_x = -0.2 if ball_x > 0 else 0.2
                        offset_y = -0.2 if ball_y > 0 else 0.2
                        self.target_position = (ball_x + offset_x, ball_y + offset_y)
                    else:  # GOAL_KICK
                        # Position in forward position to receive the kick
                        self.target_position = (0, 0)  # Center field

                    current_pos = self._get_current_pos()
                    if current_pos:
                        self.game.path_planner.request_path(
                            self.robot_id, current_pos, self.target_position
                        )
                        self._follow_path()
                return

        # Check if game has started based on referee data
        if referee_data:
            pre_start_commands = [
                "HALT",
                "STOP",
                "PREPARE_KICKOFF_BLUE",
                "PREPARE_KICKOFF_YELLOW",
            ]
            command = referee_data.get("command", "")

            # Consider the game started if:
            # 1. Command is not a pre-start command AND
            # 2. Either can_play is True OR command is a specific start command
            game_started = command not in pre_start_commands and (
                referee_data.get("can_play", False)
                or command == "NORMAL_START"
                or command == "FORCE_START"
            )
        else:
            game_started = True  # Default to game started if no referee data

        # Enforce halfway line restriction before game starts
        if not game_started:
            # Determine our half based on team color
            halfway_line = 0
            if self.team_color == "blue":
                # Blue team stays in negative x half (left side)
                current_pos = self._get_current_pos()
                if current_pos and current_pos[0] > halfway_line:
                    # In wrong half, return to our half
                    print(
                        f"ENFORCING HALFWAY LINE: Attacker {self.robot_id} returning to correct half"
                    )
                    self.current_state = RobotState.RETURNING
                    self.target_position = (-0.5, current_pos[1])
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
                    return
            else:
                # Yellow team stays in positive x half (right side)
                current_pos = self._get_current_pos()
                if current_pos and current_pos[0] < halfway_line:
                    # In wrong half, return to our half
                    print(
                        f"ENFORCING HALFWAY LINE: Attacker {self.robot_id} returning to correct half"
                    )
                    self.current_state = RobotState.RETURNING
                    self.target_position = (0.5, current_pos[1])
                    self.game.path_planner.request_path(
                        self.robot_id, current_pos, self.target_position
                    )
                    self._follow_path()
                    return

        # Regular attacker behavior for normal gameplay
        if "ball" not in vision_data or vision_data["ball"]["x"] is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            return

        ball = vision_data["ball"]
        current_pos = self._get_current_pos()
        if not current_pos:
            return

        # Calculate distance to ball
        distance_to_ball = (
            (current_pos[0] - ball["x"]) ** 2 + (current_pos[1] - ball["y"]) ** 2
        ) ** 0.5

        if distance_to_ball < self.ball_ownership_threshold:
            # We have the ball - try to score
            self.current_state = RobotState.ATTACKING
            attack_pos = self._calculate_attack_position(ball)
            self.target_position = attack_pos

            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.target_position
            )
        elif self._should_go_to_ball(ball):
            # Go to ball
            self.current_state = RobotState.MOVING_TO_BALL
            self.target_position = (ball["x"], ball["y"])

            self.game.path_planner.request_path(
                self.robot_id, current_pos, self.target_position
            )
        else:
            # Take supporting position
            support_pos = self._calculate_support_position(ball)
            self.current_state = RobotState.SUPPORTING
            self.target_position = support_pos

            self.game.path_planner.request_path(self.robot_id, current_pos, support_pos)

        # Follow the path after deciding on target
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
