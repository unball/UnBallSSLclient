# RobotBehavior/robot_state_machine.py
import math
import time
from threading import Lock
from typing import Tuple, Dict, Optional, List, Any  # Added Any for game object
from .robot_states import RobotState, RobotRole

# --- Constants for Behavior Tuning ---
DEBUG_ROBOT_BEHAVIOR = True  # Master debug flag for this module

# Pathing
PATH_LOOKAHEAD_DISTANCE = 0.2  # Meters for PathFollower
PATH_REPLAN_INTERVAL = (
    0.3  # Seconds between forced replans if target hasn't changed much
)
PATH_TARGET_CHANGE_THRESHOLD = (
    0.1  # Meters, how much target must change to force replan
)

# Movement
ROBOT_MAX_LINEAR_SPEED = 1.5  # m/s
ROBOT_MAX_ANGULAR_SPEED = math.pi * 1.5  # rad/s
APPROACH_SLOWDOWN_DISTANCE = 0.5  # m, distance to start slowing down
FINE_TUNE_DISTANCE = 0.05  # m, very close distance for minimal speed
TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_HIGH = math.pi / 2  # rad (90 deg)
TURN_SPEED_REDUCTION_FACTOR_HIGH = 0.3  # Reduce speed to 30% for large turns
TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_LOW = math.pi / 4  # rad (45 deg)
TURN_SPEED_REDUCTION_FACTOR_LOW = 0.6  # Reduce speed to 60% for medium turns
FIELD_BOUNDARY_MARGIN = 0.1  # m, safety margin from field edges for targets
IMMEDIATE_BOUNDARY_STOP_MARGIN = 0.05  # m, margin to stop if already over edge

# Game Rules
BALL_AVOIDANCE_DISTANCE_STOP = 0.5  # m, distance to keep from ball during STOP
BALL_AVOIDANCE_DISTANCE_OPPONENT_SET_PIECE = 0.5  # m, distance for opponent set pieces
OWN_HALF_OFFSET = 0.05  # m, small offset to ensure robot is clearly in its half

# Role Specific
ATTACKER_BALL_OWNERSHIP_THRESHOLD = 0.15  # m, distance to consider ball "owned"
GOALKEEPER_GOAL_LINE_OFFSET = 0.05  # m, how far in front of goal line to stand


class PathFollower:
    """Manages following a path smoothly using a lookahead point."""

    def __init__(self, lookahead_distance: float = PATH_LOOKAHEAD_DISTANCE):
        self.lookahead_distance = lookahead_distance
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"PathFollower initialized with lookahead: {self.lookahead_distance}m"
            )

    def get_target_point(
        self, current_pos: Tuple[float, float], path: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        if not path:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"PathFollower WARNING: Empty path for current_pos {current_pos}")
            return current_pos

        # If current_pos is already very close to the final goal of the path, target it directly.
        final_goal = path[-1]
        if self._distance(current_pos, final_goal) < self.lookahead_distance:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"PathFollower: Close to final goal {final_goal}. Targeting directly."
                )
            return final_goal

        # Iterate through path segments to find the lookahead point
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            # Project current_pos onto the segment p1-p2
            # This is a more robust way to find the "closest point on path"
            # and then find the lookahead from there.
            # For simplicity, we'll stick to iterating points for now,
            # but Pure Pursuit algorithm would be more advanced here.

            # If a point on the path is beyond lookahead_distance, it's a candidate
            if self._distance(current_pos, p2) >= self.lookahead_distance:
                # Basic interpolation: if p2 is too far, find a point on segment p1-p2
                # For now, just picking p2 is often good enough if path resolution is decent.
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"PathFollower: Lookahead point {p2} from path. Current: {current_pos}"
                    )
                return p2

        # If no point is far enough, or we reached the end of path points, target final goal.
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"PathFollower: Reached end of path or no suitable lookahead. Targeting final goal {final_goal}. Current: {current_pos}"
            )
        return final_goal

    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


class RobotStateMachine:
    def __init__(
        self, robot_id: int, team_color: str, game: Any
    ):  # Use Any for game type hint
        self.robot_id = robot_id
        self.team_color = team_color.lower()  # Ensure consistent casing
        self.game = game
        self.current_state: RobotState = RobotState.IDLE
        self.role: Optional[RobotRole] = None
        self.target_position: Optional[Tuple[float, float]] = None

        self.state_lock = Lock()
        self.path_follower = PathFollower()

        self.current_path: List[Tuple[float, float]] = []
        self.last_path_request_time: float = 0.0
        self.last_target_for_path: Optional[Tuple[float, float]] = None

        # Home position should be defined by subclasses based on role and field bounds
        self.home_position: Tuple[float, float] = (0.0, 0.0)
        if DEBUG_ROBOT_BEHAVIOR:
            print(f"RobotStateMachine {self.robot_id} ({self.team_color}) initialized.")

    def _get_current_pos(self):
        vision_data = self.game.get_vision_data()
        if not vision_data:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: _get_current_pos - No vision_data object."
                )
            return None
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"

        # Ensure self.robot_id is an int for lookup
        robot_data = vision_data.get(team_key, {}).get(
            self.robot_id
        )  # Direct int lookup

        if (
            robot_data
            and robot_data.get("x") is not None
            and robot_data.get("y") is not None
        ):
            return (robot_data["x"], robot_data["y"])
        else:
            if DEBUG_ROBOT_BEHAVIOR:
                # print(f"Robot {self.robot_id} ({team_key}): Position data not found or incomplete in vision. Data: {vision_data.get(team_key, {}).keys()}")
                pass  # This can be very noisy, enable if needed
            return None

    def _get_current_orientation(self):
        vision_data = self.game.get_vision_data()
        if not vision_data:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: _get_current_orientation - No vision_data object."
                )
            return None
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robot_data = vision_data.get(team_key, {}).get(
            self.robot_id
        )  # Direct int lookup

        if robot_data and robot_data.get("theta") is not None:
            return robot_data["theta"]
        else:
            # if DEBUG_ROBOT_BEHAVIOR: print(f"Robot {self.robot_id} ({team_key}): Orientation data not found or incomplete.")
            pass
            return None

    def update(self, vision_data: Dict):
        with self.state_lock:
            current_pos = self._get_current_pos()
            if not current_pos:
                if DEBUG_ROBOT_BEHAVIOR:
                    print(f"Robot {self.robot_id}: No position, cannot update.")
                # Consider stopping the robot if it was moving
                # self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
                return

            referee_data = self.game.get_referee_data()

            action_taken_by_referee_logic = self._handle_referee_commands(
                vision_data, referee_data, current_pos
            )

            if not action_taken_by_referee_logic:
                # If referee didn't dictate a specific action, proceed with role-based decision
                self._decide_next_action(vision_data, referee_data, current_pos)

    def _handle_referee_commands(
        self,
        vision_data: Dict,
        referee_data: Optional[Dict],
        current_pos: Tuple[float, float],
    ) -> bool:
        """
        Handles game state commands from the referee.
        Returns True if a referee command dictated the robot's action, False otherwise.
        """
        if not referee_data:
            return False

        game_command = referee_data.get("command", "")
        # In SSL-Game-Controller, team for action is often part of the command string itself (e.g., KICK_OFF_BLUE)
        # or in a separate `designated_team` or similar field. Adapt as per your GC data structure.
        # For now, let's assume team_for_action is derived correctly.
        team_for_action = referee_data.get(
            "team_color_for_command", self.game.get_team_for_command(game_command)
        )  # Example helper

        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Robot {self.robot_id} ({self.role.value if self.role else 'Unknown'}): "
                f"Ref CMD: {game_command}, TeamForAction: {team_for_action}, OurTeam: {self.team_color}"
            )

        if game_command == "HALT":
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: HALT command - Stopping.")
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            self.current_state = RobotState.IDLE
            self.target_position = None
            self.current_path = []
            return True

        if game_command == "STOP":
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: STOP command - Keeping {BALL_AVOIDANCE_DISTANCE_STOP}m from ball."
                )
            self.current_state = RobotState.AVOIDING_BALL  # Using a dedicated state
            ball_pos_data = vision_data.get("ball")
            if ball_pos_data and ball_pos_data.get("x") is not None:
                ball_coords = (ball_pos_data["x"], ball_pos_data["y"])
                dist_to_ball = self._distance(current_pos, ball_coords)
                if dist_to_ball < BALL_AVOIDANCE_DISTANCE_STOP:
                    # Move away from ball
                    vec_x = current_pos[0] - ball_coords[0]
                    vec_y = current_pos[1] - ball_coords[1]
                    norm = self._distance((0, 0), (vec_x, vec_y))
                    if norm > 0:
                        target_dist = (
                            BALL_AVOIDANCE_DISTANCE_STOP + 0.1
                        )  # Move slightly further
                        self.target_position = (
                            ball_coords[0] + vec_x / norm * target_dist,
                            ball_coords[1] + vec_y / norm * target_dist,
                        )
                    else:  # On top of ball
                        self.target_position = (
                            current_pos[0] + target_dist,
                            current_pos[1],
                        )
                else:  # Already far enough, stay or move to home
                    self.target_position = (
                        current_pos  # Stay put, or self.home_position
                    )
            else:  # No ball, go home
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return True

        # Handle "PREPARE" states (Kickoff, Penalty)
        if "PREPARE_KICKOFF" in game_command or "PREPARE_PENALTY" in game_command:
            # Robots must be in their own half. Specific positioning rules apply.
            # This is a simplified version. More detailed logic needed for specific PREPARE states.
            halfway_line_x = 0.0
            in_own_half = (
                self.team_color == "blue"
                and current_pos[0] < halfway_line_x - OWN_HALF_OFFSET
            ) or (
                self.team_color == "yellow"
                and current_pos[0] > halfway_line_x + OWN_HALF_OFFSET
            )

            if not in_own_half:
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"Robot {self.robot_id}: {game_command} - Not in own half. Returning."
                    )
                self.current_state = RobotState.RETURNING
                # A generic target in own half. Roles might refine this.
                target_x = (
                    -self.game.field_bounds["x_max"] * 0.25
                    if self.team_color == "blue"
                    else self.game.field_bounds["x_max"] * 0.25
                )
                self.target_position = (target_x, current_pos[1])
                self._process_movement(current_pos)
                return True
            else:
                # If in own half, role-specific logic for PREPARE states might apply
                # or simply hold position. For now, we let _decide_next_action handle it.
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"Robot {self.robot_id}: {game_command} - In own half. Role will decide."
                    )
                return False  # Let role logic handle specific positioning

        # Set Pieces (Kickoff, Free Kick, Penalty, etc.)
        # These commands determine which team can act with the ball.
        relevant_set_piece = False
        for piece in [
            "KICK_OFF",
            "FREE_KICK",
            "PENALTY",
            "CORNER_KICK",
            "GOAL_KICK",
            "BALL_PLACEMENT",
            "THROW_IN",
        ]:
            if piece in game_command:
                relevant_set_piece = True
                break

        if relevant_set_piece:
            is_our_turn = team_for_action == self.team_color

            if not is_our_turn:
                # Opponent's turn. We must keep distance from the ball.
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"Robot {self.robot_id}: Opponent's {game_command}. Keeping distance."
                    )
                self.current_state = RobotState.AVOIDING_BALL
                ball_pos_data = vision_data.get("ball")
                if ball_pos_data and ball_pos_data.get("x") is not None:
                    ball_coords = (ball_pos_data["x"], ball_pos_data["y"])
                    dist_to_ball = self._distance(current_pos, ball_coords)
                    # BALL_PLACEMENT allows robots to be close if placing ball. Otherwise, 0.5m rule.
                    required_dist = (
                        0.0
                        if "BALL_PLACEMENT" in game_command
                        and team_for_action == self.team_color
                        else BALL_AVOIDANCE_DISTANCE_OPPONENT_SET_PIECE
                    )

                    if dist_to_ball < required_dist:
                        vec_x = current_pos[0] - ball_coords[0]
                        vec_y = current_pos[1] - ball_coords[1]
                        norm = self._distance((0, 0), (vec_x, vec_y))
                        if norm > 0:
                            target_dist = required_dist + 0.1  # Move slightly further
                            self.target_position = (
                                ball_coords[0] + vec_x / norm * target_dist,
                                ball_coords[1] + vec_y / norm * target_dist,
                            )
                        else:  # On top of ball
                            self.target_position = (
                                current_pos[0] + target_dist,
                                current_pos[1],
                            )
                        self._process_movement(current_pos)
                    # else: already far enough, could hold position or allow role to decide
                return True  # Action dictated by referee rule
            # else (is_our_turn): let role-specific logic in _decide_next_action handle it.

        return False  # No overriding referee command handled here

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Optional[Dict],
        current_pos: Tuple[float, float],
    ):
        """Placeholder: Subclasses MUST implement this to set current_state and target_position, then call _process_movement."""
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"RobotStateMachine {self.robot_id}: Default _decide_next_action. Role: {self.role.value if self.role else 'N/A'}"
            )

        # Default fallback: if idle or no target, go home
        if self.current_state == RobotState.IDLE or not self.target_position:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position

        self._process_movement(current_pos)

    def _process_movement(self, current_pos: Tuple[float, float]):
        """Centralized handler for path requests and initiating path following."""
        if self.target_position:
            self._request_path_if_needed(current_pos, self.target_position)
            self._follow_path(current_pos)  # Pass current_pos to _follow_path
        else:  # No target_position
            if self.current_state != RobotState.IDLE:  # If not intentionally idle, stop
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"Robot {self.robot_id}: No target_position, but not IDLE. Stopping."
                    )
                self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
                self.current_path = []

    def _request_path_if_needed(
        self, current_pos: Tuple[float, float], target_pos: Tuple[float, float]
    ):
        """Requests a path from the planner if conditions are met."""
        now = time.time()

        # Check if target has changed significantly from the last path's target
        target_changed = True
        if self.last_target_for_path:
            if (
                self._distance(target_pos, self.last_target_for_path)
                < PATH_TARGET_CHANGE_THRESHOLD
            ):
                target_changed = False

        if target_changed or (now - self.last_path_request_time > PATH_REPLAN_INTERVAL):
            if DEBUG_ROBOT_BEHAVIOR:
                reason = "target changed" if target_changed else "replan interval"
                print(
                    f"Robot {self.robot_id}: Requesting path from {current_pos} to {target_pos} (Reason: {reason})"
                )
            self.game.path_planner.request_path(self.robot_id, current_pos, target_pos)
            self.last_path_request_time = now
            self.last_target_for_path = (
                target_pos  # Store the target for which this path was requested
            )

    def _follow_path(self, current_pos: Tuple[float, float]):  # Added current_pos
        """Follows the current path using the path follower."""
        current_orientation = self._get_current_orientation()
        if current_orientation is None:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: No orientation data for path following.")
            self.game.active_controller.send_global_velocity(
                self.robot_id, 0, 0, 0
            )  # Stop if no orientation
            return

        # Get latest path from planner
        # This might be slightly delayed from the request, which is okay for now.
        # For critical real-time updates, a callback or future from path_planner would be better.
        path_from_planner = self.game.path_planner.get_path(self.robot_id)

        # Use the new path if available and different from current, or if no current path
        if path_from_planner and path_from_planner != self.current_path:
            self.current_path = path_from_planner
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: Updated to new path with {len(self.current_path)} points."
                )

        if self.current_path and len(self.current_path) > 0:
            lookahead_point = self.path_follower.get_target_point(
                current_pos, self.current_path
            )
            self._move_to_point(current_pos, current_orientation, lookahead_point)
            # Check if close to the final point of the *current_path*
            if (
                self.current_path
                and self._distance(current_pos, self.current_path[-1])
                < FINE_TUNE_DISTANCE
            ):
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"Robot {self.robot_id}: Reached end of current path segment."
                    )
                # If also close to overall target_position, could set state to IDLE or re-evaluate.
                if (
                    self.target_position
                    and self._distance(current_pos, self.target_position)
                    < FINE_TUNE_DISTANCE
                ):
                    self.current_state = (
                        RobotState.IDLE
                    )  # Or a specific "ARRIVED" state
                    # Potentially stop the robot explicitly here if IDLE doesn't guarantee it
                    self.game.active_controller.send_global_velocity(
                        self.robot_id, 0, 0, 0
                    )

        elif self.target_position:  # No path, but have a target: direct move (fallback)
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: No path from planner. Moving directly to {self.target_position}."
                )
            self._move_to_point(current_pos, current_orientation, self.target_position)
        else:  # No path and no target_position
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: No path and no target. Idling.")
            if self.current_state != RobotState.IDLE:  # If not already idle, stop.
                self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)

    def _enforce_field_boundaries(
        self, target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Ensure target position is within field boundaries with a safety margin."""
        bounds = (
            self.game.field_bounds
        )  # Assumes field_bounds is {x_min, x_max, y_min, y_max}

        x = max(
            bounds["x_min"] + FIELD_BOUNDARY_MARGIN,
            min(bounds["x_max"] - FIELD_BOUNDARY_MARGIN, target_pos[0]),
        )
        y = max(
            bounds["y_min"] + FIELD_BOUNDARY_MARGIN,
            min(bounds["y_max"] - FIELD_BOUNDARY_MARGIN, target_pos[1]),
        )

        if (x, y) != target_pos and DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Robot {self.robot_id}: Target {target_pos} adjusted by boundary to ({x:.2f}, {y:.2f})"
            )
        return (x, y)

    def _move_to_point(
        self,
        current_pos: Tuple[float, float],
        current_orientation: float,
        target_point: Tuple[float, float],
    ):
        """Move robot towards a target_point with orientation and speed control."""
        safe_target = self._enforce_field_boundaries(target_point)

        dx = safe_target[0] - current_pos[0]
        dy = safe_target[1] - current_pos[1]
        distance_to_target = self._distance(
            (0, 0), (dx, dy)
        )  # math.sqrt(dx*dx + dy*dy)

        # --- Speed Control ---
        speed_factor = 1.0
        if distance_to_target < FINE_TUNE_DISTANCE:
            speed_factor = 0.0  # Stop if very close (or very low speed for fine tuning)
        elif distance_to_target < APPROACH_SLOWDOWN_DISTANCE:
            # Linear ramp down from 1.0 to 0.1 (or other min_speed_factor)
            min_speed_factor_on_approach = 0.2
            speed_factor = min_speed_factor_on_approach + (
                distance_to_target - FINE_TUNE_DISTANCE
            ) / (APPROACH_SLOWDOWN_DISTANCE - FINE_TUNE_DISTANCE) * (
                1.0 - min_speed_factor_on_approach
            )
            speed_factor = max(
                min_speed_factor_on_approach, min(1.0, speed_factor)
            )  # Clamp

        # --- Orientation Control ---
        # Desired orientation: face the direction of movement towards safe_target
        angle_to_safe_target = math.atan2(dy, dx)
        orientation_error = self._normalize_angle(
            angle_to_safe_target - current_orientation
        )

        # Reduce linear speed if a large turn is needed
        if abs(orientation_error) > TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_HIGH:
            speed_factor *= TURN_SPEED_REDUCTION_FACTOR_HIGH
        elif abs(orientation_error) > TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_LOW:
            speed_factor *= TURN_SPEED_REDUCTION_FACTOR_LOW

        # P-controller for angular velocity
        # Gain can be tuned. Higher gain = faster orientation, but can oscillate.
        angular_p_gain = 2.5
        angular_velocity = orientation_error * angular_p_gain
        angular_velocity = max(
            -ROBOT_MAX_ANGULAR_SPEED, min(ROBOT_MAX_ANGULAR_SPEED, angular_velocity)
        )

        # --- Final Velocity Calculation ---
        # Target linear speed
        target_linear_speed = ROBOT_MAX_LINEAR_SPEED * speed_factor

        # Calculate vx, vy based on current orientation if we want robot-centric control (local_velocity)
        # For global_velocity, vx and vy are world-frame velocities towards target
        if distance_to_target > 0:  # Avoid division by zero
            vx = (dx / distance_to_target) * target_linear_speed
            vy = (dy / distance_to_target) * target_linear_speed
        else:
            vx, vy = 0, 0  # Already at target

        # --- Boundary Safety Stop (Immediate) ---
        # If robot is ALREADY outside (or very near edge) and moving further out, stop that component of velocity.
        bounds = self.game.field_bounds
        if (
            current_pos[0] <= bounds["x_min"] + IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vx < 0
        ) or (
            current_pos[0] >= bounds["x_max"] - IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vx > 0
        ):
            vx = 0
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: X-boundary safety stop.")
        if (
            current_pos[1] <= bounds["y_min"] + IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vy < 0
        ) or (
            current_pos[1] >= bounds["y_max"] - IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vy > 0
        ):
            vy = 0
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: Y-boundary safety stop.")

        if hasattr(self.game, "active_controller") and self.game.active_controller:
            if DEBUG_ROBOT_BEHAVIOR and (
                abs(vx) > ROBOT_MAX_LINEAR_SPEED or abs(vy) > ROBOT_MAX_LINEAR_SPEED
            ):
                print(
                    f"Robot {self.robot_id} WARNING: Velocity ({vx:.2f}, {vy:.2f}) exceeds max speed {ROBOT_MAX_LINEAR_SPEED}"
                )

            self.game.active_controller.send_global_velocity(
                self.robot_id, vx, vy, angular_velocity
            )
            if DEBUG_ROBOT_BEHAVIOR:
                # This print can be very verbose, enable only when deep debugging movement
                # print(f"Robot {self.robot_id}: CMD Vel: vx={vx:.2f}, vy={vy:.2f}, w={angular_velocity:.2f} "
                #       f"| TargetPt: ({target_point[0]:.2f},{target_point[1]:.2f}) Dist: {distance_to_target:.2f} OrientErr: {orientation_error:.2f}")
                pass
        elif DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Robot {self.robot_id}: No active_controller found in game object for _move_to_point!"
            )

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Helper for Euclidean distance, identical to PathFollower's."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def debug_robot_state(self, vision_data: Dict, referee_data: Optional[Dict]):
        """Generic debug printer for robot state. To be called by subclasses."""
        current_pos = self._get_current_pos()
        current_orient = self._get_current_orientation()

        print(f"\n--- DEBUG: Robot ID {self.robot_id} ({self.team_color}) ---")
        print(f"Role: {self.role.value if self.role else 'N/A'}")
        print(f"Current State: {self.current_state.value}")
        print(
            f"Position: {('({:.2f}, {:.2f})'.format(current_pos[0], current_pos[1])) if current_pos else 'N/A'}"
        )
        print(
            f"Orientation: {('{:.2f} rad'.format(current_orient)) if current_orient is not None else 'N/A'}"
        )
        print(
            f"Target Position: {('({:.2f}, {:.2f})'.format(self.target_position[0], self.target_position[1])) if self.target_position else 'N/A'}"
        )

        path_info = "No Path"
        if self.current_path:
            path_info = (
                f"{len(self.current_path)} points. Start: ({self.current_path[0][0]:.2f}, {self.current_path[0][1]:.2f}), "
                f"End: ({self.current_path[-1][0]:.2f}, {self.current_path[-1][1]:.2f})"
            )
        print(f"Current Path: {path_info}")

        ball_data = vision_data.get("ball")
        if ball_data and ball_data.get("x") is not None:
            print(f"Ball Position: ({ball_data['x']:.2f}, {ball_data['y']:.2f})")
            if current_pos:
                print(
                    f"Distance to Ball: {self._distance(current_pos, (ball_data['x'], ball_data['y'])):.2f}m"
                )
        else:
            print("Ball: Not detected or no position data.")

        if referee_data:
            print(
                f"Referee CMD: {referee_data.get('command', 'N/A')}, "
                f"TeamForAction: {self.game.get_team_for_command(referee_data.get('command', ''))}, "
                f"Stage: {referee_data.get('stage', 'N/A')}, "
                f"CanPlay: {referee_data.get('can_play', False)}"
            )
        else:
            print("Referee Data: N/A")
        print(f"------------------------------------")


class GoalkeeperStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game: Any):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.GOALKEEPER

        # Define goal line and home position based on field dimensions and team color
        # Assuming self.game.field_bounds = {'x_min': -W/2, 'x_max': W/2, 'y_min': -H/2, 'y_max': H/2}
        # Assuming self.game.vision.get_geometry() returns a dict with "goalWidth"
        field_width_half = self.game.field_bounds["x_max"]
        self.goal_line_x = (
            -field_width_half if self.team_color == "blue" else field_width_half
        )
        self.home_position = (
            self.goal_line_x
            + (
                GOALKEEPER_GOAL_LINE_OFFSET
                if self.team_color == "blue"
                else -GOALKEEPER_GOAL_LINE_OFFSET
            ),
            0.0,
        )

        vision_geom = self.game.vision.get_geometry()
        self.goal_half_width = (
            vision_geom.get("goalWidth", 0.8) / 2.0
        )  # Default to 0.8m if not found

        self.target_position = self.home_position
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Goalkeeper {self.robot_id} ({self.team_color}) initialized. Home: {self.home_position}, GoalLineX: {self.goal_line_x}, GoalHalfWidth: {self.goal_half_width}"
            )

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Optional[Dict],
        current_pos: Tuple[float, float],
    ):
        if DEBUG_ROBOT_BEHAVIOR:
            self.debug_robot_state(vision_data, referee_data)

        game_command = referee_data.get("command", "") if referee_data else ""
        team_for_action = (
            self.game.get_team_for_command(game_command) if referee_data else ""
        )

        # --- Referee Overrides for Goalkeeper ---
        # PENALTY for opponent: GK must stay on goal line (can move along it before kick)
        if "PENALTY" in game_command and team_for_action != self.team_color:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"GK {self.robot_id}: PENALTY DEFENSE positioning.")
            self.current_state = RobotState.BLOCKING
            ball_y = vision_data.get("ball", {}).get("y", 0.0)
            target_y = max(
                -self.goal_half_width + 0.05, min(self.goal_half_width - 0.05, ball_y)
            )  # Stay within posts
            self.target_position = (self.goal_line_x, target_y)  # Stay ON the line
            self._process_movement(current_pos)
            return

        # GOAL_KICK for our team (SSL-EL: GK often takes it)
        if "GOAL_KICK" in game_command and team_for_action == self.team_color:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"GK {self.robot_id}: Taking GOAL KICK.")
            self.current_state = RobotState.MOVING_TO_BALL  # Or specific KICKING state
            ball_pos_data = vision_data.get("ball")
            if ball_pos_data and ball_pos_data.get("x") is not None:
                ball_coords = (ball_pos_data["x"], ball_pos_data["y"])
                # Approach ball to kick towards opponent goal center
                opp_goal_center_x = (
                    self.game.field_bounds["x_max"]
                    if self.team_color == "blue"
                    else -self.game.field_bounds["x_max"]
                )
                self.target_position = self._calculate_ball_approach_for_kick(
                    ball_coords, (opp_goal_center_x, 0.0)
                )
            else:  # No ball, go to default ball placement spot for goal kick (edge of penalty area)
                penalty_area_depth = (
                    self.game.vision.get_geometry().get("penaltyAreaDepth", 1.0)
                    / 1000.0
                )  # convert mm to m
                default_kick_x = self.goal_line_x + (
                    penalty_area_depth
                    if self.team_color == "blue"
                    else -penalty_area_depth
                )
                self.target_position = (default_kick_x, 0.0)

            self._process_movement(current_pos)
            return

        # --- Normal Goalkeeping Logic ---
        ball_data = vision_data.get("ball")
        if not ball_data or ball_data.get("x") is None:  # No ball visible
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"GK {self.robot_id}: No ball, returning home.")
        elif self._is_ball_threatening(ball_data, current_pos):
            self.current_state = RobotState.BLOCKING
            self.target_position = self._calculate_blocking_position(ball_data)
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"GK {self.robot_id}: Ball threatening. Blocking at {self.target_position}."
                )
        else:  # Ball not threatening
            self.current_state = (
                RobotState.RETURNING
            )  # Or a more passive 'WATCHING' state
            self.target_position = self.home_position
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"GK {self.robot_id}: Ball not threatening, returning home.")

        self._process_movement(current_pos)

    def _is_ball_threatening(
        self, ball_data: Dict, current_pos: Tuple[float, float]
    ) -> bool:
        # Threatening if ball is in defensive third of the field.
        # More advanced: consider ball velocity vector.
        ball_x = ball_data["x"]
        defensive_zone_x_limit = (
            self.game.field_bounds["x_max"] / 3.0
        )  # One third of field from goal line

        if self.team_color == "blue":
            # Threat if ball_x is less than (goal_line_x + defensive_zone_x_limit)
            return ball_x < (self.goal_line_x + defensive_zone_x_limit)
        else:  # Yellow team
            # Threat if ball_x is greater than (goal_line_x - defensive_zone_x_limit)
            return ball_x > (self.goal_line_x - defensive_zone_x_limit)

    def _calculate_blocking_position(self, ball_data: Dict) -> Tuple[float, float]:
        # Stay on (or slightly in front of) goal line, aligned with ball's Y, within goal posts.
        ball_y = ball_data.get("y", 0.0)

        # Clamp target_y to be within goal posts (with a small margin)
        y_margin_from_post = 0.05  # meters
        clamped_y = max(
            -self.goal_half_width + y_margin_from_post,
            min(self.goal_half_width - y_margin_from_post, ball_y),
        )

        # Position slightly in front of the goal line
        block_x = self.goal_line_x + (
            GOALKEEPER_GOAL_LINE_OFFSET
            if self.team_color == "blue"
            else -GOALKEEPER_GOAL_LINE_OFFSET
        )
        return (block_x, clamped_y)

    def _calculate_ball_approach_for_kick(
        self, ball_pos: Tuple[float, float], kick_target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calculates a point behind the ball relative to the kick target."""
        offset_dist = 0.15  # How far behind the ball to position
        dx = kick_target_pos[0] - ball_pos[0]
        dy = kick_target_pos[1] - ball_pos[1]
        norm = self._distance((0, 0), (dx, dy))
        if norm == 0:  # Kick target is at ball position, rare edge case
            return (
                ball_pos[0]
                - (offset_dist if self.team_color == "blue" else -offset_dist),
                ball_pos[1],
            )

        return (
            ball_pos[0] - (dx / norm * offset_dist),
            ball_pos[1] - (dy / norm * offset_dist),
        )


class DefenderStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game: Any):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.DEFENDER
        # Home position: in own half, spread out if multiple defenders
        field_width_half = self.game.field_bounds["x_max"]
        home_x = (
            -field_width_half * 0.6
            if self.team_color == "blue"
            else field_width_half * 0.6
        )
        home_y = 0.75 if self.robot_id % 2 == 0 else -0.75  # Example for two defenders
        self.home_position = (home_x, home_y)
        self.target_position = self.home_position
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Defender {self.robot_id} ({self.team_color}) initialized. Home: {self.home_position}"
            )

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Optional[Dict],
        current_pos: Tuple[float, float],
    ):
        if DEBUG_ROBOT_BEHAVIOR:
            self.debug_robot_state(vision_data, referee_data)

        game_command = referee_data.get("command", "") if referee_data else ""
        team_for_action = (
            self.game.get_team_for_command(game_command) if referee_data else ""
        )

        # --- Referee Overrides for Defender ---
        # Example: Our KICK_OFF - position defensively but out of the way of attacker
        if "KICK_OFF" in game_command and team_for_action == self.team_color:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"DEF {self.robot_id}: Positioning for our KICK-OFF.")
            self.current_state = RobotState.SUPPORTING  # Or a specific formation state
            # Position in own half, slightly wide. Exact position depends on formation.
            target_x = (
                -self.game.field_bounds["x_max"] * 0.25
                if self.team_color == "blue"
                else self.game.field_bounds["x_max"] * 0.25
            )
            target_y = 0.8 if self.robot_id % 2 == 0 else -0.8  # Spread out
            self.target_position = (target_x, target_y)
            self._process_movement(current_pos)
            return

        # Example: Opponent FREE_KICK - form a wall or mark players
        if "FREE_KICK" in game_command and team_for_action != self.team_color:
            # This needs more complex logic: where is the free kick? Should we form a wall?
            # For now, a generic defensive posture.
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"DEF {self.robot_id}: Opponent FREE_KICK. Defensive posture.")
            self.current_state = RobotState.DEFENDING
            ball_pos_data = vision_data.get("ball")
            if ball_pos_data and ball_pos_data.get("x") is not None:
                self.target_position = self._calculate_defensive_line_position(
                    ball_pos_data, current_pos
                )
            else:
                self.target_position = self.home_position  # Fallback
            self._process_movement(current_pos)
            return

        # --- Normal Defender Logic ---
        ball_data = vision_data.get("ball")
        if not ball_data or ball_data.get("x") is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"DEF {self.robot_id}: No ball, returning home.")
        elif self._is_ball_in_defensive_zone(ball_data):
            self.current_state = RobotState.DEFENDING  # Or INTERCEPTING if very close
            self.target_position = self._calculate_intercept_or_block_position(
                ball_data, vision_data, current_pos
            )
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"DEF {self.robot_id}: Ball in def zone. Target: {self.target_position}."
                )
        else:  # Ball is further upfield
            self.current_state = RobotState.SUPPORTING
            self.target_position = self._calculate_midfield_support_position(
                ball_data, current_pos
            )
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"DEF {self.robot_id}: Ball not in def zone. Supporting at {self.target_position}."
                )

        self._process_movement(current_pos)

    def _is_ball_in_defensive_zone(self, ball_data: Dict) -> bool:
        # Defensive zone: roughly own half of the field.
        ball_x = ball_data["x"]
        midfield_x = 0.0
        if self.team_color == "blue":
            return (
                ball_x < midfield_x + self.game.field_bounds["x_max"] * 0.1
            )  # Ball slightly into opponent half is still concern
        else:  # Yellow team
            return ball_x > midfield_x - self.game.field_bounds["x_max"] * 0.1

    def _calculate_intercept_or_block_position(
        self, ball_data: Dict, vision_data: Dict, current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        # Basic: position on a line between the ball and the center of our goal.
        # Don't get too close to own goal; let GK handle that.
        ball_coords = (ball_data["x"], ball_data["y"])
        our_goal_center_x = (
            -self.game.field_bounds["x_max"]
            if self.team_color == "blue"
            else self.game.field_bounds["x_max"]
        )

        # Target a point partway between ball and goal
        interception_ratio = 0.6  # Closer to ball than goal
        target_x = (
            ball_coords[0] * (1 - interception_ratio)
            + our_goal_center_x * interception_ratio
        )
        target_y = (
            ball_coords[1] * (1 - interception_ratio) + 0.0 * interception_ratio
        )  # Towards Y=0 of goal

        # Keep defender in own half and a certain distance from goal line
        min_dist_from_own_goal = 0.75  # meters
        if self.team_color == "blue":
            target_x = min(target_x, -OWN_HALF_OFFSET)  # Stay in own half
            target_x = max(
                target_x, our_goal_center_x + min_dist_from_own_goal
            )  # Don't crowd GK
        else:
            target_x = max(target_x, OWN_HALF_OFFSET)
            target_x = min(target_x, our_goal_center_x - min_dist_from_own_goal)

        return (target_x, target_y)

    def _calculate_midfield_support_position(
        self, ball_data: Dict, current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        # Position in midfield, ready to receive a pass or press.
        # Example: stay slightly behind the ball's x-coordinate, offset in y.
        ball_x, ball_y = ball_data["x"], ball_data["y"]

        x_offset_behind_ball = 0.5  # meters
        target_x = ball_x - (
            x_offset_behind_ball if self.team_color == "blue" else -x_offset_behind_ball
        )

        # Clamp to own half mostly
        if self.team_color == "blue":
            target_x = min(
                target_x, self.game.field_bounds["x_max"] * 0.2
            )  # Can cross midfield a bit
        else:
            target_x = max(target_x, -self.game.field_bounds["x_max"] * 0.2)

        # Y position: if multiple defenders, they can take different y_offsets.
        y_offset_from_ball = 1.0 if self.robot_id % 2 == 0 else -1.0
        target_y = ball_y + y_offset_from_ball
        # Clamp Y to field
        target_y = max(
            self.game.field_bounds["y_min"] + FIELD_BOUNDARY_MARGIN,
            min(self.game.field_bounds["y_max"] - FIELD_BOUNDARY_MARGIN, target_y),
        )

        return (target_x, target_y)

    def _calculate_defensive_line_position(
        self, ball_data: Dict, current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        # Simplified: form a line parallel to goal line, at ball's X or slightly deeper.
        # This is very basic for wall formation.
        ball_x = ball_data["x"]
        our_goal_line_x = (
            -self.game.field_bounds["x_max"]
            if self.team_color == "blue"
            else self.game.field_bounds["x_max"]
        )

        # Position wall slightly in front of penalty area or based on ball_x
        wall_x_offset = 1.2  # meters from goal line (approx penalty area)
        wall_x = our_goal_line_x + (
            wall_x_offset if self.team_color == "blue" else -wall_x_offset
        )

        # If ball is further out, wall can be further out too, but not beyond ball_x
        if self.team_color == "blue":
            wall_x = min(wall_x, ball_x - 0.5)  # Wall 0.5m behind ball
            wall_x = max(wall_x, our_goal_line_x + 0.5)  # Min dist from goal
        else:
            wall_x = max(wall_x, ball_x + 0.5)
            wall_x = min(wall_x, our_goal_line_x - 0.5)

        # Y position based on robot ID to spread out in a line (very simplified)
        num_defenders_in_wall = 2  # Assume 2 defenders might form a wall
        y_spacing = 0.3  # meters
        base_y = ball_data.get("y", 0.0)  # Center wall around ball's Y

        # This needs robot ID coordination if multiple defenders form a wall.
        # For a single defender, it might try to block the direct shot path.
        target_y = (
            base_y
            + (
                self.robot_id % num_defenders_in_wall
                - (num_defenders_in_wall - 1) / 2.0
            )
            * y_spacing
        )

        goal_half_width = self.game.vision.get_geometry().get("goalWidth", 0.8) / 2.0
        target_y = max(
            -goal_half_width, min(goal_half_width, target_y)
        )  # Stay within goal width projection

        return (wall_x, target_y)


class AttackerStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game: Any):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.ATTACKER
        # Home: slightly in opponent's half, or near center circle.
        field_width_half = self.game.field_bounds["x_max"]
        home_x = (
            field_width_half * 0.1
            if self.team_color == "blue"
            else -field_width_half * 0.1
        )
        self.home_position = (
            home_x,
            0.0,
        )  # Y can be offset based on ID if multiple attackers
        self.target_position = self.home_position
        self.has_ball = False
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"Attacker {self.robot_id} ({self.team_color}) initialized. Home: {self.home_position}"
            )

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Optional[Dict],
        current_pos: Tuple[float, float],
    ):
        if DEBUG_ROBOT_BEHAVIOR:
            self.debug_robot_state(vision_data, referee_data)

        game_command = referee_data.get("command", "") if referee_data else ""
        team_for_action = (
            self.game.get_team_for_command(game_command) if referee_data else ""
        )
        can_play_from_ref = (
            referee_data.get("can_play", True) if referee_data else True
        )  # Default to can_play

        # --- Referee Overrides for Attacker ---
        # KICK_OFF for our team: Attacker usually takes it
        if "KICK_OFF" in game_command and team_for_action == self.team_color:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"ATT {self.robot_id}: Preparing for KICK-OFF.")
            self.current_state = RobotState.MOVING_TO_BALL
            ball_pos_data = vision_data.get("ball")
            if ball_pos_data and ball_pos_data.get("x") is not None:
                ball_coords = (ball_pos_data["x"], ball_pos_data["y"])
                # Approach ball from own side for kickoff (ball is at (0,0))
                self.target_position = (
                    ball_coords[0] - (0.15 if self.team_color == "blue" else -0.15),
                    ball_coords[1],
                )
            else:  # Ball not visible, go to center
                self.target_position = (0.0, 0.0)
            self._process_movement(current_pos)
            return

        # FREE_KICK / PENALTY for our team: Attacker might take it
        if (
            ("FREE_KICK" in game_command or "PENALTY" in game_command)
        ) and team_for_action == self.team_color:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"ATT {self.robot_id}: Preparing for {game_command}.")
            self.current_state = RobotState.MOVING_TO_BALL
            ball_pos_data = vision_data.get("ball")
            if ball_pos_data and ball_pos_data.get("x") is not None:
                ball_coords = (ball_pos_data["x"], ball_pos_data["y"])
                opp_goal_center_x = (
                    self.game.field_bounds["x_max"]
                    if self.team_color == "blue"
                    else -self.game.field_bounds["x_max"]
                )
                self.target_position = self._calculate_ball_approach_for_kick(
                    ball_coords, (opp_goal_center_x, 0.0)
                )
            else:  # No ball data, difficult to position. Go to a generic offensive spot or home.
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return

        # HALFWAY LINE ENFORCEMENT for PREPARE states or if game not truly started
        # Game starts on NORMAL_START, FORCE_START, or when a KICK_OFF/FREE_KICK etc. is active for our team
        game_is_live = (
            can_play_from_ref
            and game_command not in ["HALT", "STOP"]
            and "PREPARE" not in game_command
        )

        if not game_is_live:
            halfway_line_x = 0.0
            # Must be in own half if game not live (simplified rule)
            if (
                self.team_color == "blue"
                and current_pos[0] > halfway_line_x - OWN_HALF_OFFSET
            ) or (
                self.team_color == "yellow"
                and current_pos[0] < halfway_line_x + OWN_HALF_OFFSET
            ):
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"ATT {self.robot_id}: Game not live ({game_command}). Enforcing halfway line."
                    )
                self.current_state = RobotState.RETURNING
                target_x = (
                    -self.game.field_bounds["x_max"] * 0.1
                    if self.team_color == "blue"
                    else self.game.field_bounds["x_max"] * 0.1
                )
                self.target_position = (target_x, current_pos[1])  # Stay on current Y
                self._process_movement(current_pos)
                return

        # --- Normal Attacker Logic ---
        ball_data = vision_data.get("ball")
        if not ball_data or ball_data.get("x") is None:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            self.has_ball = False
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"ATT {self.robot_id}: No ball, returning home.")
        else:
            ball_coords = (ball_data["x"], ball_data["y"])
            distance_to_ball = self._distance(current_pos, ball_coords)

            if distance_to_ball < ATTACKER_BALL_OWNERSHIP_THRESHOLD:
                self.has_ball = True
                self.current_state = (
                    RobotState.ATTACKING
                )  # Could be DRIBBLING then SHOOTING
                # Aim for opponent's goal center
                opp_goal_x = (
                    self.game.field_bounds["x_max"]
                    if self.team_color == "blue"
                    else -self.game.field_bounds["x_max"]
                )
                self.target_position = (opp_goal_x, 0.0)  # Dribble towards goal
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"ATT {self.robot_id}: Has ball! Attacking towards {self.target_position}."
                    )
            else:  # Don't have ball
                self.has_ball = False
                if self._is_best_attacker_to_get_ball(
                    current_pos, ball_coords, vision_data
                ):
                    self.current_state = RobotState.MOVING_TO_BALL
                    self.target_position = self._calculate_ball_approach_for_kick(
                        ball_coords, self._get_opponent_goal_center()
                    )
                    if DEBUG_ROBOT_BEHAVIOR:
                        print(
                            f"ATT {self.robot_id}: Best attacker. Moving to ball at {self.target_position}."
                        )
                else:
                    self.current_state = RobotState.SUPPORTING
                    self.target_position = self._calculate_offensive_support_position(
                        ball_coords, vision_data, current_pos
                    )
                    if DEBUG_ROBOT_BEHAVIOR:
                        print(
                            f"ATT {self.robot_id}: Not best attacker. Supporting at {self.target_position}."
                        )

        self._process_movement(current_pos)

    def _get_opponent_goal_center(self) -> Tuple[float, float]:
        """Returns the coordinates of the center of the opponent's goal."""
        opp_goal_x = (
            self.game.field_bounds["x_max"]
            if self.team_color == "blue"
            else -self.game.field_bounds["x_max"]
        )
        return (opp_goal_x, 0.0)

    def _is_best_attacker_to_get_ball(
        self,
        my_pos: Tuple[float, float],
        ball_pos: Tuple[float, float],
        vision_data: Dict,
    ) -> bool:
        # Simple: closest attacker. More advanced: consider orientation, obstacles.
        my_dist_to_ball = self._distance(my_pos, ball_pos)

        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robots_on_my_team = vision_data.get(team_key, {})

        for r_id_str, robot_data in robots_on_my_team.items():
            r_id = int(r_id_str)
            if r_id == self.robot_id or robot_data.get("x") is None:
                continue

            # Check if this teammate is an attacker (needs role info if game object stores it)
            # For now, assume all teammates are potential ball-getters
            teammate_pos = (robot_data["x"], robot_data["y"])
            teammate_dist_to_ball = self._distance(teammate_pos, ball_pos)

            # If another attacker is significantly closer (e.g., by more than robot radius)
            if teammate_dist_to_ball < my_dist_to_ball - 0.1:
                # This check needs to know roles of other robots from self.game.robot_state_machines
                # Assuming other_robot_role = self.game.robot_state_machines[r_id].role
                # if other_robot_role == RobotRole.ATTACKER:
                #    return False
                pass  # For now, any teammate closer is enough to not be 'best'

        return True  # Default to true if no one else is clearly closer or if logic is simple

    def _calculate_offensive_support_position(
        self,
        ball_pos: Tuple[float, float],
        vision_data: Dict,
        current_pos: Tuple[float, float],
    ) -> Tuple[float, float]:
        # Find an open space in the attacking third, or make a run.
        # Simple: Position ahead of the ball towards opponent goal, offset to a side.
        opp_goal_x, opp_goal_y = self._get_opponent_goal_center()

        # Vector from ball to goal
        dir_to_goal_x = opp_goal_x - ball_pos[0]
        dir_to_goal_y = opp_goal_y - ball_pos[1]
        norm_dir = self._distance((0, 0), (dir_to_goal_x, dir_to_goal_y))

        if norm_dir == 0:
            return self.home_position  # Fallback

        # Point ahead of ball
        dist_ahead = 1.0  # meters
        point_ahead_x = ball_pos[0] + (dir_to_goal_x / norm_dir) * dist_ahead
        point_ahead_y = ball_pos[1] + (dir_to_goal_y / norm_dir) * dist_ahead

        # Offset sideways for a passing option
        # Perpendicular vector: (-dy, dx) or (dy, -dx)
        side_offset_dist = 0.8  # meters
        # Alternate side based on robot ID or ball Y position
        if ball_pos[1] > 0:  # If ball is on top half, support on bottom
            side_vec_x = dir_to_goal_y / norm_dir
            side_vec_y = -dir_to_goal_x / norm_dir
        else:  # Ball on bottom half, support on top
            side_vec_x = -dir_to_goal_y / norm_dir
            side_vec_y = dir_to_goal_x / norm_dir

        final_target_x = point_ahead_x + side_vec_x * side_offset_dist
        final_target_y = point_ahead_y + side_vec_y * side_offset_dist

        # Clamp to field (using the general boundary enforcer later is also fine)
        final_target_x = max(
            self.game.field_bounds["x_min"] + FIELD_BOUNDARY_MARGIN,
            min(
                self.game.field_bounds["x_max"] - FIELD_BOUNDARY_MARGIN, final_target_x
            ),
        )
        final_target_y = max(
            self.game.field_bounds["y_min"] + FIELD_BOUNDARY_MARGIN,
            min(
                self.game.field_bounds["y_max"] - FIELD_BOUNDARY_MARGIN, final_target_y
            ),
        )

        return (final_target_x, final_target_y)

    def _calculate_ball_approach_for_kick(
        self, ball_pos: Tuple[float, float], kick_target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Helper to calculate approach point behind ball, facing kick_target_pos."""
        # Same as Goalkeeper's for now
        offset_dist = 0.15
        dx = kick_target_pos[0] - ball_pos[0]
        dy = kick_target_pos[1] - ball_pos[1]
        norm = self._distance((0, 0), (dx, dy))
        if norm == 0:
            return (
                ball_pos[0]
                - (offset_dist if self.team_color == "blue" else -offset_dist),
                ball_pos[1],
            )

        return (
            ball_pos[0] - (dx / norm * offset_dist),
            ball_pos[1] - (dy / norm * offset_dist),
        )
