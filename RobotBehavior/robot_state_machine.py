# At the top of the file, update the imports:
import math
import time
from threading import Lock
from typing import Tuple, Dict, Optional, List, Any, TYPE_CHECKING

import math
import time
from threading import Lock
from typing import Tuple, Dict, Optional, List, Any
from .robot_states import RobotState, RobotRole

if TYPE_CHECKING:
    # Only import these for type checking to avoid circular imports
    from main import Game

# --- Constants for Behavior Tuning & SSL-EL Rules ---
DEBUG_ROBOT_BEHAVIOR = False

# Pathing
PATH_LOOKAHEAD_DISTANCE = 0.2
PATH_REPLAN_INTERVAL = 0.3
PATH_TARGET_CHANGE_THRESHOLD = 0.1

# Movement
ROBOT_MAX_LINEAR_SPEED = 1.5  # m/s (SSL-EL Rule 8.3.2)
ROBOT_MAX_SPEED_STOP_STATE = 0.75  # m/s (SSL-EL Rule 8.3.3)
ROBOT_MAX_ANGULAR_SPEED = math.pi * 2.5  # Increased for agility
APPROACH_SLOWDOWN_DISTANCE = 0.4  # m
FINE_TUNE_DISTANCE = 0.05  # m, for arrival
TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_HIGH = math.pi / 2  # 90 deg
TURN_SPEED_REDUCTION_FACTOR_HIGH = 0.3
TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_LOW = math.pi / 4  # 45 deg
TURN_SPEED_REDUCTION_FACTOR_LOW = 0.6
FIELD_BOUNDARY_MARGIN = 0.05  # m
IMMEDIATE_BOUNDARY_STOP_MARGIN = 0.02  # m, very close to edge

# SSL-EL Game Rules Constants
BALL_AVOIDANCE_DISTANCE_STOP_STATE = 0.5  # m (Rule 5.1.1)
GRACE_PERIOD_HALT_BRAKE = 2.0  # s (Rule 5.1.2)
KICK_RESTART_BALL_MOVE_DISTANCE = 0.05  # m (Rule 5.4)
KICK_TIMEOUT_SECONDS = 10.0  # s (Rule 5.4, 5.3.5.6)
MAX_KICK_SPEED_GRSIM = 6.0  # m/s (grSim default, actual ball speed is different)

DEFENDER_DISTANCE_FROM_BALL_OPPONENT_KICK = 0.5  # m (Rule 5.3.3 / 8.3.3)
DISTANCE_FROM_OPPONENT_DEFENSE_AREA_STOP_FK = 0.2  # m (Rule 8.3.1)
PENALTY_KICK_OTHER_ROBOTS_DISTANCE_BEHIND_BALL = 1.0  # m (Rule 5.3.5.3)

# Ball Placement (Rule 5.2 & 8.3.3)
BALL_PLACEMENT_OPPONENT_STADIUM_DISTANCE = 0.5
BALL_PLACEMENT_SUCCESS_RADIUS = 0.15
BALL_PLACEMENT_MAX_TIME = 30.0
BALL_PLACEMENT_MIN_DIST_NEXT_CMD_FK = 0.05
BALL_PLACEMENT_MIN_DIST_NEXT_CMD_FORCE_START = 0.5

# Role Specific
ATTACKER_BALL_OWNERSHIP_THRESHOLD = (
    0.12  # m, (robot radius ~0.09 + ball_radius ~0.0215 = ~0.11)
)
GOALKEEPER_GOAL_LINE_X_OFFSET = 0.05
OWN_HALF_X_OFFSET = 0.05

# Field constants (Defaults for SSL-EL, updated by game.field_bounds)
FIELD_LENGTH_DEFAULT = 4.5
FIELD_WIDTH_DEFAULT = 3.0
GOAL_WIDTH_DEFAULT = 0.8
GOAL_DEPTH_DEFAULT = 0.18
DEFENSE_AREA_DEPTH_DEFAULT = 0.5
DEFENSE_AREA_WIDTH_DEFAULT = 1.35
CENTER_CIRCLE_RADIUS_DEFAULT = 0.5
PENALTY_SPOT_ABS_X_DEFAULT = (
    FIELD_LENGTH_DEFAULT / 2.0
) - 1.0  # SSL-EL: 1m from goal line (Rule 2.1.1 for Div B is 3m from goal center, EL field is 4.5m, so 2.25 - 1.0 = 1.25 from center)
# For SSL-EL Field (4.5m): Penalty mark is 3m from opponent's goal center (Rule 2.1.3).
# Goals at +/-2.25m. So, penalty spots are at x = 2.25 - 3.0 = -0.75 (for blue attacking right)
# and x = -2.25 + 3.0 = +0.75 (for yellow attacking left).
# So, PENALTY_SPOT_ABS_X_DEFAULT = 0.75 is correct.


class PathFollower:  # (Keep your existing PathFollower, it's good)
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
        final_goal = path[-1]
        if self._distance(current_pos, final_goal) < self.lookahead_distance:
            return final_goal
        for i in range(len(path) - 1):
            p2 = path[i + 1]
            if self._distance(current_pos, p2) >= self.lookahead_distance:
                return p2
        return final_goal

    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


class RobotStateMachine:
    def __init__(self, robot_id: int, team_color: str, game: "Game"):
        self.robot_id = robot_id
        self.team_color = team_color.lower()
        self.game = game
        self.current_state: RobotState = RobotState.IDLE
        self.role: Optional[RobotRole] = None
        self.target_position: Optional[Tuple[float, float]] = None
        self.state_lock = Lock()
        self.path_follower = PathFollower()
        self.current_path: List[Tuple[float, float]] = []
        self.last_path_request_time: float = 0.0
        self.last_target_for_path: Optional[Tuple[float, float]] = None
        self.last_game_command_from_referee: str = ""
        self.is_designated_kicker: bool = False  # Also used for ball placer
        self.home_position: Tuple[float, float] = (0.0, 0.0)
        self.field_bounds = game.field_bounds.copy()  # Take a copy
        self.goal_half_width = GOAL_WIDTH_DEFAULT / 2.0
        self.own_goal_line_x = 0.0
        self.attack_direction_x_sign = 1
        self._update_geometry_references()  # Initial call

        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"RobotStateMachine {self.robot_id} ({self.team_color}) initialized. Attack sign: {self.attack_direction_x_sign}. OwnGoalX: {self.own_goal_line_x}"
            )

    def _update_geometry_references(self):
        self.field_bounds = self.game.field_bounds.copy()
        self.own_goal_line_x = (
            -self.field_bounds.get("x_max", FIELD_LENGTH_DEFAULT / 2.0)
            if self.team_color == "blue"
            else self.field_bounds.get("x_max", FIELD_LENGTH_DEFAULT / 2.0)
        )
        self.attack_direction_x_sign = 1 if self.team_color == "blue" else -1
        vision_geom = (
            self.game.vision.get_geometry()
            if hasattr(self.game, "vision") and self.game.vision.any_geometry
            else {}
        )
        self.goal_half_width = vision_geom.get("goalWidth", GOAL_WIDTH_DEFAULT) / 2.0

    def get_team_from_command_suffix(self, game_command: str) -> Optional[str]:
        if game_command.endswith("_BLUE"):
            return "blue"
        if game_command.endswith("_YELLOW"):
            return "yellow"
        return None

    def _get_current_pos(self) -> Optional[Tuple[float, float]]:
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return None
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robot_data = vision_data.get(team_key, {}).get(self.robot_id)
        if robot_data and robot_data.get("x") is not None:
            return (robot_data["x"], robot_data["y"])
        return None

    def _get_current_orientation(self) -> Optional[float]:
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return None
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robot_data = vision_data.get(team_key, {}).get(self.robot_id)
        if robot_data and robot_data.get("theta") is not None:
            return robot_data["theta"]
        return None

    def _get_ball_pos(self, vision_data: Dict) -> Optional[Tuple[float, float]]:
        ball_data = vision_data.get("ball")
        if ball_data and ball_data.get("x") is not None:
            return (ball_data["x"], ball_data["y"])
        return None

    def update(self, vision_data: Dict):
        with self.state_lock:
            self._update_geometry_references()
            current_pos = self._get_current_pos()
            if not current_pos:
                if DEBUG_ROBOT_BEHAVIOR and self.robot_id == 0:
                    print(f"Robot {self.robot_id}: No position, cannot update.")
                self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
                return

            referee_data = self.game.get_referee_data()
            game_command_from_ref = referee_data.get("command", "")

            if game_command_from_ref != self.last_game_command_from_referee:
                if DEBUG_ROBOT_BEHAVIOR:
                    print(
                        f"Robot {self.robot_id}: New Ref CMD '{game_command_from_ref}', prev was '{self.last_game_command_from_referee}'"
                    )
                self.last_game_command_from_referee = game_command_from_ref
                self.is_designated_kicker = False  # Reset on any new ref command

            action_taken = self._handle_referee_commands(
                vision_data, referee_data, current_pos, game_command_from_ref
            )
            if not action_taken:
                self._decide_next_action(
                    vision_data, referee_data, current_pos, game_command_from_ref
                )

    def _handle_referee_commands(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ) -> bool:
        command_team_suffix = self.get_team_from_command_suffix(game_command)
        base_command = (
            game_command.replace("_BLUE", "").replace("_YELLOW", "")
            if command_team_suffix
            else game_command
        )

        # Simplified logging
        if (
            base_command not in ["NORMAL_START", "FORCE_START"]
            or self.current_state == RobotState.IDLE
        ):
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"SM {self.robot_id} ({self.role.name if self.role else 'NIL'}): Ref '{base_command}' (team: {command_team_suffix}), OurTeam: '{self.team_color}'"
                )

        if base_command == "HALT":
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            self.current_state = RobotState.IDLE
            self.target_position = None
            self.current_path = []
            return True

        if base_command == "STOP":
            self.current_state = RobotState.AVOIDING_BALL
            ball_pos = self._get_ball_pos(vision_data)
            if ball_pos:
                dist_to_ball = self._distance(current_pos, ball_pos)
                if dist_to_ball < BALL_AVOIDANCE_DISTANCE_STOP_STATE:
                    angle_away = math.atan2(
                        current_pos[1] - ball_pos[1], current_pos[0] - ball_pos[0]
                    )
                    self.target_position = (
                        ball_pos[0]
                        + (BALL_AVOIDANCE_DISTANCE_STOP_STATE + 0.1)
                        * math.cos(angle_away),
                        ball_pos[1]
                        + (BALL_AVOIDANCE_DISTANCE_STOP_STATE + 0.1)
                        * math.sin(angle_away),
                    )
                else:
                    self.target_position = current_pos
            else:
                self.target_position = self.home_position
            self._process_movement(
                current_pos, max_speed_override=ROBOT_MAX_SPEED_STOP_STATE
            )
            return True

        # --- Two-Stage Command Logic (PREPARE_X -> NORMAL_START) ---
        prev_base_cmd_from_ref = self.last_game_command_from_referee.replace(
            "_BLUE", ""
        ).replace("_YELLOW", "")
        prev_cmd_team_suffix = self.get_team_from_command_suffix(
            self.last_game_command_from_referee
        )

        if base_command == "NORMAL_START":
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"SM {self.robot_id}: NORMAL_START. Prev Ref CMD: {prev_base_cmd_from_ref}, Team: {prev_cmd_team_suffix}"
                )
            if (
                prev_base_cmd_from_ref == "PREPARE_KICKOFF"
                and prev_cmd_team_suffix == self.team_color
            ):
                self.current_state = (
                    RobotState.TAKING_SET_PIECE
                    if self.is_designated_kicker
                    else RobotState.SUPPORTING_OFFENSE
                )
                return False  # Let _decide_next_action position for kick/support
            elif (
                prev_base_cmd_from_ref == "PREPARE_PENALTY"
                and prev_cmd_team_suffix == self.team_color
            ):
                self.current_state = (
                    RobotState.TAKING_SET_PIECE
                    if self.is_designated_kicker
                    else RobotState.IDLE
                )  # Others stay put
                if not self.is_designated_kicker:
                    self.target_position = current_pos
                    self._process_movement(current_pos)
                    return True
                return False
            # Add similar logic for PREPARE_FREEKICK, BALL_PLACEMENT if GC sends NORMAL_START after them
            # For BALL_PLACEMENT, GC usually sends the next command (e.g. FREE_KICK_OURTEAM or FORCE_START)

            # Default for NORMAL_START if not a recognized two-stage follow-up: resume play
            if self.current_state in [
                RobotState.IDLE,
                RobotState.PREPARING_SET_PIECE,
                RobotState.AVOIDING_BALL,
            ]:
                self.current_state = RobotState.RETURNING
            return False

        if base_command == "FORCE_START":
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"SM {self.robot_id}: FORCE_START.")
            if self.current_state in [
                RobotState.IDLE,
                RobotState.PREPARING_SET_PIECE,
                RobotState.AVOIDING_BALL,
            ]:
                self.current_state = RobotState.RETURNING
            return False

        # --- PREPARE States ---
        if base_command.startswith("PREPARE_"):
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"SM {self.robot_id}: Preparing for {base_command} (for {command_team_suffix})."
                )
            self.current_state = RobotState.PREPARING_SET_PIECE
            # Kicker designation logic (simplified for EL, TODO: make more robust)
            if command_team_suffix == self.team_color:
                if (
                    base_command == "PREPARE_KICKOFF"
                    and self.role == RobotRole.ATTACKER
                    and self.robot_id == 2
                ):  # EL: Attacker 2 takes kickoff
                    self.is_designated_kicker = True
                elif (
                    base_command == "PREPARE_PENALTY"
                    and self.role == RobotRole.ATTACKER
                    and self.robot_id == 2
                ):  # EL: Attacker 2 takes penalty
                    self.is_designated_kicker = True
                # Add more for Free Kick, Corner, Goal Kick if specific robots take them
            return False  # Let _decide_next_action handle actual positioning

        # --- Direct Set Piece Commands (e.g. FREE_KICK_BLUE) ---
        is_our_set_piece = command_team_suffix == self.team_color
        if base_command in [
            "KICK_OFF",
            "FREE_KICK",
            "PENALTY",
            "CORNER_KICK",
            "GOAL_KICK",
        ]:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"SM {self.robot_id}: Direct set piece CMD {base_command} for {command_team_suffix}."
                )
            self.current_state = RobotState.PREPARING_SET_PIECE
            if is_our_set_piece:
                # Designate kicker (simplified)
                if (
                    base_command == "KICK_OFF"
                    and self.role == RobotRole.ATTACKER
                    and self.robot_id == 2
                ):
                    self.is_designated_kicker = True
                elif (
                    base_command == "PENALTY"
                    and self.role == RobotRole.ATTACKER
                    and self.robot_id == 2
                ):
                    self.is_designated_kicker = True
                elif base_command == "GOAL_KICK" and self.role == RobotRole.GOALKEEPER:
                    self.is_designated_kicker = True
                # TODO: Add for FREE_KICK, CORNER_KICK (usually attacker)
            return False

        # --- BALL_PLACEMENT (Rule 5.2) ---
        if base_command == "BALL_PLACEMENT":
            designated_target_from_ref = referee_data.get(
                "designated_position"
            )  # (x,y) from GC
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"SM {self.robot_id}: BALL_PLACEMENT for {command_team_suffix} to {designated_target_from_ref}."
                )
            if is_our_set_piece:
                self.current_state = RobotState.BALL_PLACEMENT_ACTIVE
                # TODO: Designate one robot as placer (e.g., closest non-GK or specific ID)
                if self.role == RobotRole.ATTACKER and self.robot_id == 2:
                    self.is_designated_kicker = True  # Using kicker flag for placer
            else:  # Opponent is placing
                self.current_state = RobotState.BALL_PLACEMENT_AVOIDING
            return False

        return False  # No overriding command handled

    # _decide_next_action, _process_movement, _request_path_if_needed,
    # _follow_path, _enforce_field_boundaries, _move_to_point,
    # _normalize_angle, _distance, _is_point_in_defense_area, _get_penalty_spot, debug_robot_state
    # These methods can largely remain as you refined them in the previous step.
    # I will copy them here for completeness, with minor adjustments if needed.

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ):
        if self.current_state == RobotState.PREPARING_SET_PIECE:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: In PREPARING_SET_PIECE, (base FSM) defaulting to home."
                )
            self.target_position = self.home_position
        elif self.current_state == RobotState.IDLE or (
            not self.target_position and self.current_state != RobotState.RETURNING
        ):
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: Idle/no target, RETURNING home.")
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
        if (
            self.current_state == RobotState.RETURNING
            and self.target_position == self.home_position
            and self._distance(current_pos, self.home_position) < FINE_TUNE_DISTANCE
        ):
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: Arrived home, IDLE.")
            self.current_state = RobotState.IDLE
            self.target_position = None
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            return
        self._process_movement(current_pos)

    def _process_movement(
        self,
        current_pos: Tuple[float, float],
        max_speed_override: Optional[float] = None,
    ):
        if self.target_position:
            self._request_path_if_needed(current_pos, self.target_position)
            self._follow_path(current_pos, max_speed_override)
        elif self.current_state != RobotState.IDLE:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: No target, but not IDLE. Stopping.")
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            self.current_path = []

    def _request_path_if_needed(
        self, current_pos: Tuple[float, float], target_pos: Tuple[float, float]
    ):
        now = time.time()
        target_changed = True
        if (
            self.last_target_for_path
            and self._distance(target_pos, self.last_target_for_path)
            < PATH_TARGET_CHANGE_THRESHOLD
        ):
            target_changed = False
        if target_changed or (now - self.last_path_request_time > PATH_REPLAN_INTERVAL):
            reason = "target changed" if target_changed else "replan interval"
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: Requesting path for {reason} to {target_pos}"
                )
            safe_target = self._enforce_field_boundaries(target_pos)
            self.game.path_planner.request_path(self.robot_id, current_pos, safe_target)
            self.last_path_request_time = now
            self.last_target_for_path = safe_target

    def _follow_path(
        self,
        current_pos: Tuple[float, float],
        max_speed_override: Optional[float] = None,
    ):
        current_orientation = self._get_current_orientation()
        if current_orientation is None:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: No orientation for path following.")
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            return
        path = self.game.path_planner.get_path(self.robot_id)
        if path and path != self.current_path:
            self.current_path = path
        if self.current_path:
            lookahead = self.path_follower.get_target_point(
                current_pos, self.current_path
            )
            self._move_to_point(
                current_pos, current_orientation, lookahead, max_speed_override
            )
            if self._distance(current_pos, self.current_path[-1]) < FINE_TUNE_DISTANCE:
                if DEBUG_ROBOT_BEHAVIOR:
                    print(f"Robot {self.robot_id}: Reached end of path segment.")
                if (
                    self.target_position
                    and self._distance(current_pos, self.target_position)
                    < FINE_TUNE_DISTANCE
                ):
                    if DEBUG_ROBOT_BEHAVIOR:
                        print(
                            f"Robot {self.robot_id}: Arrived at target {self.target_position}. State: {self.current_state.name}"
                        )
                    # Fix the states list - remove non-existent states
                    if self.current_state in [
                        RobotState.MOVING_TO_POSITION,
                        RobotState.RETURNING,
                        RobotState.AVOIDING_BALL,
                        RobotState.SUPPORTING_OFFENSE,
                        RobotState.DEFENDING,
                        RobotState.BALL_PLACEMENT_AVOIDING,  # This now exists
                    ]:
                        self.current_state = RobotState.IDLE
                        self.game.active_controller.send_global_velocity(
                            self.robot_id, 0, 0, 0
                        )
                    self.target_position = None
                    self.current_path = []
        elif self.target_position:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"Robot {self.robot_id}: No path, direct move to {self.target_position}."
                )
            self._move_to_point(
                current_pos,
                current_orientation,
                self.target_position,
                max_speed_override,
            )
        elif self.current_state != RobotState.IDLE:
            if DEBUG_ROBOT_BEHAVIOR:
                print(f"Robot {self.robot_id}: No path, no target, not IDLE. Stopping.")
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)

    def _enforce_field_boundaries(
        self, target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        x = max(
            self.field_bounds["x_min"] + FIELD_BOUNDARY_MARGIN,
            min(self.field_bounds["x_max"] - FIELD_BOUNDARY_MARGIN, target_pos[0]),
        )
        y = max(
            self.field_bounds["y_min"] + FIELD_BOUNDARY_MARGIN,
            min(self.field_bounds["y_max"] - FIELD_BOUNDARY_MARGIN, target_pos[1]),
        )
        if (x, y) != target_pos and DEBUG_ROBOT_BEHAVIOR and self.robot_id == 0:
            print(
                f"Robot {self.robot_id}: Target {target_pos} adj. to ({x:.2f},{y:.2f})"
            )
        return (x, y)

    def _move_to_point(
        self,
        current_pos: Tuple[float, float],
        current_orientation: float,
        target_point: Tuple[float, float],
        max_speed_override: Optional[float] = None,
    ):
        safe_target = self._enforce_field_boundaries(target_point)
        dx, dy = safe_target[0] - current_pos[0], safe_target[1] - current_pos[1]
        dist = self._distance((0, 0), (dx, dy))

        # ADD THIS DEBUG LOG
        if DEBUG_ROBOT_BEHAVIOR and self.robot_id < 3:  # Log all robots
            print(
                f"Robot {self.robot_id} _move_to_point: current={current_pos}, target={safe_target}, dist={dist:.3f}"
            )

        max_speed = (
            max_speed_override
            if max_speed_override is not None
            else ROBOT_MAX_LINEAR_SPEED
        )
        speed_factor = 1.0
        if dist < FINE_TUNE_DISTANCE:
            speed_factor = 0.0
            if DEBUG_ROBOT_BEHAVIOR and self.robot_id < 3:
                print(
                    f"Robot {self.robot_id}: Too close to target (dist={dist:.3f} < {FINE_TUNE_DISTANCE}), speed_factor=0"
                )
        elif dist < APPROACH_SLOWDOWN_DISTANCE:
            min_sf = 0.1
            speed_factor = min_sf + (dist - FINE_TUNE_DISTANCE) / (
                APPROACH_SLOWDOWN_DISTANCE - FINE_TUNE_DISTANCE
            ) * (1.0 - min_sf)
            speed_factor = max(min_sf, min(1.0, speed_factor))
            if DEBUG_ROBOT_BEHAVIOR and self.robot_id < 3:
                print(
                    f"Robot {self.robot_id}: In slowdown zone, speed_factor={speed_factor:.3f}"
                )

        angle_to_target = math.atan2(dy, dx)
        orient_err = self._normalize_angle(angle_to_target - current_orientation)

        # Reduce speed for large turns
        if abs(orient_err) > TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_HIGH:
            speed_factor *= TURN_SPEED_REDUCTION_FACTOR_HIGH
        elif abs(orient_err) > TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_LOW:
            speed_factor *= TURN_SPEED_REDUCTION_FACTOR_LOW

        ang_p_gain = 3.0
        ang_vel = max(
            -ROBOT_MAX_ANGULAR_SPEED,
            min(ROBOT_MAX_ANGULAR_SPEED, orient_err * ang_p_gain),
        )
        final_lin_speed = max_speed * speed_factor
        vx, vy = (
            (dx / dist * final_lin_speed) if dist > 0 else 0,
            (dy / dist * final_lin_speed) if dist > 0 else 0,
        )

        # Boundary safety checks
        ovx, ovy = vx, vy
        if (
            current_pos[0]
            <= self.field_bounds["x_min"] + IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vx < 0
        ) or (
            current_pos[0]
            >= self.field_bounds["x_max"] - IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vx > 0
        ):
            vx = 0
        if (
            current_pos[1]
            <= self.field_bounds["y_min"] + IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vy < 0
        ) or (
            current_pos[1]
            >= self.field_bounds["y_max"] - IMMEDIATE_BOUNDARY_STOP_MARGIN
            and vy > 0
        ):
            vy = 0

        # ADD THIS DEBUG LOG BEFORE SENDING COMMAND
        if DEBUG_ROBOT_BEHAVIOR and self.robot_id < 3:
            print(
                f"Robot {self.robot_id} computed velocities: vx={vx:.3f}, vy={vy:.3f}, w={ang_vel:.3f}"
            )
            print(
                f"Robot {self.robot_id} speed_factor={speed_factor:.3f}, orient_err={orient_err:.3f}"
            )

        if hasattr(self.game, "active_controller") and self.game.active_controller:
            if DEBUG_ROBOT_BEHAVIOR and self.robot_id < 3:
                print(f"Robot {self.robot_id} sending command to controller")
            result = self.game.active_controller.send_command_with_kick(
                self.robot_id, vx, vy, ang_vel, 0, 0, 0
            )
            if DEBUG_ROBOT_BEHAVIOR and self.robot_id < 3:
                print(f"Robot {self.robot_id} command result: {result}")
        else:
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"R{self.robot_id} ERROR: No active_controller for _move_to_point!"
                )

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _is_point_in_defense_area(
        self, point: Tuple[float, float], which_area: str = "own"
    ) -> bool:
        # Using constants for defense area dimensions relative to goal line
        # Defense area depth from goal line into field, width is total width centered on y=0
        depth = DEFENSE_AREA_DEPTH_DEFAULT
        half_width = DEFENSE_AREA_WIDTH_DEFAULT / 2.0

        goal_line_x_ref = (
            self.own_goal_line_x if which_area == "own" else -self.own_goal_line_x
        )
        # Field side for "inside" defense area (towards center of field from goal line)
        inside_sign = (
            self.attack_direction_x_sign
            if which_area == "own"
            else -self.attack_direction_x_sign
        )

        x_min_area = min(goal_line_x_ref, goal_line_x_ref + inside_sign * depth)
        x_max_area = max(goal_line_x_ref, goal_line_x_ref + inside_sign * depth)

        return (
            x_min_area <= point[0] <= x_max_area
            and -half_width <= point[1] <= half_width
        )

    def _get_penalty_spot(self, for_us_to_attack: bool) -> Tuple[float, float]:
        # Penalty spot X is PENALTY_SPOT_ABS_X_DEFAULT from *center*, towards the goal being attacked/defended
        spot_x_sign = (
            -self.attack_direction_x_sign
            if for_us_to_attack
            else self.attack_direction_x_sign
        )
        return (PENALTY_SPOT_ABS_X_DEFAULT * spot_x_sign, 0.0)

    def debug_robot_state(
        self, vision_data: Dict, referee_data: Optional[Dict]
    ):  # Kept as is
        current_pos = self._get_current_pos()
        current_orient = self._get_current_orientation()
        ball_pos = self._get_ball_pos(vision_data)
        print(
            f"\n--- DEBUG: R{self.robot_id} ({self.team_color} - {self.role.name if self.role else 'NIL'}) ---"
        )
        print(f"State: {self.current_state.name}, Kicker: {self.is_designated_kicker}")
        print(
            f"Pos: {current_pos}, Orient: {current_orient:.2f}r"
            if current_pos and current_orient is not None
            else "Pos/Orient: N/A"
        )
        print(f"Target: {self.target_position}")
        path_len = len(self.current_path)
        print(
            f"Path: {path_len}pts {'('+str(self.current_path[0])+'...'+str(self.current_path[-1])+')' if path_len > 1 else str(self.current_path)}"
        )
        print(f"Ball: {ball_pos}" if ball_pos else "Ball: N/A")
        if referee_data:
            print(
                f"RefCMD: {referee_data.get('command', 'N/A')}, Stage: {referee_data.get('stage', 'N/A')}, ForTeam: {referee_data.get('team_color_for_command', 'N/A')}"
            )
        print(f"LastRefCMDseen: {self.last_game_command_from_referee}")
        print(f"------------------------------------")


# --- Goalkeeper, Defender, Attacker StateMachines ---
# (These will inherit the updated RobotStateMachine and override _decide_next_action and _update_home_position)
# (The implementations from the previous step are largely compatible, with minor adjustments for new constants and kick integration)


class GoalkeeperStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game: Any):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.GOALKEEPER
        self._update_home_position()
        self.target_position = self.home_position
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"GK {self.robot_id} ({self.team_color}) Initialized. Home: {self.home_position}"
            )

    def _update_home_position(self):
        super()._update_geometry_references()
        home_x = self.own_goal_line_x + (
            GOALKEEPER_GOAL_LINE_X_OFFSET * self.attack_direction_x_sign
        )
        self.home_position = (home_x, 0.0)

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ):
        if DEBUG_ROBOT_BEHAVIOR and self.robot_id == 0:
            self.debug_robot_state(vision_data, referee_data)
        self._update_home_position()
        ball_pos = self._get_ball_pos(vision_data)
        base_ref_cmd = self.last_game_command_from_referee.replace("_BLUE", "").replace(
            "_YELLOW", ""
        )
        ref_cmd_team = self.get_team_from_command_suffix(
            self.last_game_command_from_referee
        )

        if self.current_state == RobotState.PREPARING_SET_PIECE:
            if base_ref_cmd == "PREPARE_KICKOFF":
                self.target_position = self.home_position
            elif base_ref_cmd == "PREPARE_PENALTY":
                if ref_cmd_team != self.team_color:  # Opponent penalty
                    penalty_spot_y = self._get_penalty_spot(for_us_to_attack=False)[1]
                    target_y = ball_pos[1] if ball_pos else penalty_spot_y
                    target_y = max(
                        -self.goal_half_width + 0.02,
                        min(self.goal_half_width - 0.02, target_y),
                    )  # Stay slightly inside posts
                    self.target_position = (self.own_goal_line_x, target_y)
                else:
                    self.target_position = self.home_position  # Our penalty
            elif (
                base_ref_cmd == "GOAL_KICK"
                and ref_cmd_team == self.team_color
                and self.is_designated_kicker
            ):
                if ball_pos:
                    self.target_position = self._calculate_ball_approach_for_kick(
                        ball_pos, (-self.own_goal_line_x, 0.0)
                    )
                else:
                    self.target_position = self.home_position  # Wait for ball placement
            else:
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return

        if (
            self.current_state == RobotState.TAKING_SET_PIECE
            and base_ref_cmd == "GOAL_KICK"
            and ref_cmd_team == self.team_color
        ):
            if (
                ball_pos
                and self._distance(current_pos, ball_pos)
                < ATTACKER_BALL_OWNERSHIP_THRESHOLD + 0.03
            ):
                if DEBUG_ROBOT_BEHAVIOR:
                    print(f"GK {self.robot_id}: KICKING for Goal Kick.")
                self.game.active_controller.kick_flat(
                    self.robot_id, MAX_KICK_SPEED_GRSIM * 0.8
                )
                self.current_state = RobotState.RETURNING  # After kick
            elif ball_pos:
                self.target_position = self._calculate_ball_approach_for_kick(
                    ball_pos, (-self.own_goal_line_x, 0.0)
                )
            else:
                self.current_state = RobotState.RETURNING
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return

        if not ball_pos:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
        elif self._is_ball_threatening(ball_pos):
            self.current_state = RobotState.BLOCKING
            self.target_position = self._calculate_blocking_position(ball_pos)
        else:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
        self._process_movement(current_pos)

    def _is_ball_threatening(self, ball_pos: Tuple[float, float]) -> bool:
        defensive_third_x = self.own_goal_line_x + (
            (self.field_bounds["x_max"] - self.field_bounds["x_min"])
            / 3.0
            * self.attack_direction_x_sign
        )
        if self.attack_direction_x_sign == 1:
            return ball_pos[0] < defensive_third_x  # Blue
        return ball_pos[0] > defensive_third_x  # Yellow

    def _calculate_blocking_position(
        self, ball_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        target_y = max(
            -self.goal_half_width + 0.05, min(self.goal_half_width - 0.05, ball_pos[1])
        )
        target_x = self.own_goal_line_x + (
            GOALKEEPER_GOAL_LINE_X_OFFSET * self.attack_direction_x_sign
        )
        return (target_x, target_y)

    def _calculate_ball_approach_for_kick(
        self, ball_pos: Tuple[float, float], kick_target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        offset_dist = 0.15
        opponent_goal_center_x = -self.own_goal_line_x
        dx, dy = opponent_goal_center_x - ball_pos[0], 0.0 - ball_pos[1]
        norm = self._distance((0, 0), (dx, dy))
        if norm == 0:
            return (
                ball_pos[0] - offset_dist * self.attack_direction_x_sign,
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
        self._update_home_position()
        self.target_position = self.home_position
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"DEF {self.robot_id} ({self.team_color}) Initialized. Home: {self.home_position}"
            )

    def _update_home_position(self):
        super()._update_geometry_references()
        home_x = self.own_goal_line_x + (
            (self.field_bounds["x_max"] - self.field_bounds["x_min"])
            * 0.25
            * self.attack_direction_x_sign
        )
        home_y_offset = DEFENSE_AREA_WIDTH_DEFAULT / 2.0 + 0.2
        home_y = (
            home_y_offset if self.robot_id % 2 != 0 else -home_y_offset
        )  # SSL-EL has 1 defender usually, adjust if 2
        self.home_position = (home_x, home_y)

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ):
        if DEBUG_ROBOT_BEHAVIOR and self.robot_id == 1:
            self.debug_robot_state(vision_data, referee_data)
        self._update_home_position()
        ball_pos = self._get_ball_pos(vision_data)
        base_ref_cmd = self.last_game_command_from_referee.replace("_BLUE", "").replace(
            "_YELLOW", ""
        )
        ref_cmd_team = self.get_team_from_command_suffix(
            self.last_game_command_from_referee
        )

        if self.current_state == RobotState.PREPARING_SET_PIECE:
            if base_ref_cmd == "PREPARE_KICKOFF":
                target_x = self.own_goal_line_x + (
                    (self.field_bounds["x_max"] - self.field_bounds["x_min"])
                    * 0.3
                    * self.attack_direction_x_sign
                )
                target_y = (
                    self.field_bounds["y_max"] - CENTER_CIRCLE_RADIUS_DEFAULT - 0.1
                ) * (0.7 if self.robot_id % 2 != 0 else -0.7)
                self.target_position = (target_x, target_y)
            elif base_ref_cmd == "PREPARE_PENALTY":
                penalty_spot_x, _ = self._get_penalty_spot(
                    for_us_to_attack=(ref_cmd_team == self.team_color)
                )
                target_x = penalty_spot_x - (
                    PENALTY_KICK_OTHER_ROBOTS_DISTANCE_BEHIND_BALL
                    * -self.attack_direction_x_sign
                    if ref_cmd_team == self.team_color
                    else PENALTY_KICK_OTHER_ROBOTS_DISTANCE_BEHIND_BALL
                    * self.attack_direction_x_sign
                )
                target_y = (self.field_bounds["y_max"] * 0.3) * (
                    1 if self.robot_id % 2 != 0 else -1
                )  # Spread out
                self.target_position = self._enforce_field_boundaries(
                    (target_x, target_y)
                )
            elif base_ref_cmd in ["PREPARE_FREEKICK", "CORNER_KICK", "GOAL_KICK"]:
                if ref_cmd_team != self.team_color and ball_pos:
                    self.target_position = self._calculate_defensive_wall_position(
                        ball_pos, current_pos
                    )
                else:
                    self.target_position = (
                        self._calculate_support_position_our_set_piece(
                            ball_pos, current_pos
                        )
                    )
            else:
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return

        if not ball_pos:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
        elif self._is_ball_in_our_half(ball_pos):
            self.current_state = RobotState.DEFENDING
            self.target_position = self._calculate_intercept_or_block_position(
                ball_pos, current_pos
            )
        else:
            self.current_state = RobotState.SUPPORTING_OFFENSE
            self.target_position = self._calculate_midfield_holding_position(
                ball_pos, current_pos
            )

        # Avoid own defense area (Rule 8.3.1) unless actively clearing/blocking deep
        if (
            self.target_position
            and self._is_point_in_defense_area(self.target_position, "own")
            and self.current_state != RobotState.CLEARING_BALL
        ):
            if DEBUG_ROBOT_BEHAVIOR:
                print(
                    f"DEF {self.robot_id}: Target {self.target_position} in own def area. Adjusting."
                )
            defense_area_edge_x = (
                self.own_goal_line_x
                + (DEFENSE_AREA_DEPTH_DEFAULT + 0.1) * self.attack_direction_x_sign
            )  # 0.1 margin
            self.target_position = (defense_area_edge_x, self.target_position[1])

        self._process_movement(current_pos)

    def _is_ball_in_our_half(self, ball_pos: Tuple[float, float]) -> bool:
        return (ball_pos[0] * self.attack_direction_x_sign) < (0.0 + OWN_HALF_X_OFFSET)

    def _calculate_intercept_or_block_position(
        self, ball_pos: Tuple[float, float], current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        target_x = (
            ball_pos[0] * 0.4 + self.own_goal_line_x * 0.6
        )  # Closer to goal than ball
        target_y = ball_pos[1] * 0.6  # Try to stay on ball's y line
        min_x_from_goal = (
            self.own_goal_line_x
            + (DEFENSE_AREA_DEPTH_DEFAULT + 0.15) * self.attack_direction_x_sign
        )
        if self.attack_direction_x_sign == 1:
            target_x = max(target_x, min_x_from_goal)
        else:
            target_x = min(target_x, min_x_from_goal)
        return self._enforce_field_boundaries((target_x, target_y))

    def _calculate_midfield_holding_position(
        self, ball_pos: Tuple[float, float], current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        target_x = (OWN_HALF_X_OFFSET * 2.0) * self.attack_direction_x_sign
        target_y = ball_pos[1] * 0.3
        return self._enforce_field_boundaries((target_x, target_y))

    def _calculate_defensive_wall_position(
        self, ball_pos: Tuple[float, float], current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        dist_from_ball = DEFENDER_DISTANCE_FROM_BALL_OPPONENT_KICK + 0.05
        # Angle from ball towards a point slightly offset from our goal center (to avoid perfect line blocking vision)
        angle_ball_to_our_goal = math.atan2(
            0.1 - ball_pos[1], self.own_goal_line_x - ball_pos[0]
        )
        target_x = ball_pos[0] + dist_from_ball * math.cos(angle_ball_to_our_goal)
        target_y = ball_pos[1] + dist_from_ball * math.sin(angle_ball_to_our_goal)
        return self._enforce_field_boundaries((target_x, target_y))

    def _calculate_support_position_our_set_piece(
        self, ball_pos: Optional[Tuple[float, float]], current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        if ball_pos:
            target_x = ball_pos[0] - 1.2 * self.attack_direction_x_sign
            target_y = ball_pos[1] + (
                0.8 if self.robot_id % 2 != 0 else -0.8
            )  # Spread out
            return self._enforce_field_boundaries((target_x, target_y))
        return self.home_position


class AttackerStateMachine(RobotStateMachine):
    def __init__(self, robot_id: int, team_color: str, game: Any):
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.ATTACKER
        self._update_home_position()
        self.target_position = self.home_position
        self.has_ball = False  # Local state for attacker
        if DEBUG_ROBOT_BEHAVIOR:
            print(
                f"ATT {self.robot_id} ({self.team_color}) Initialized. Home: {self.home_position}"
            )

    def _update_home_position(self):
        super()._update_geometry_references()
        home_x = (
            (self.field_bounds["x_max"] - self.field_bounds["x_min"])
            * 0.05
            * self.attack_direction_x_sign
        )  # Slightly in opponent half
        self.home_position = (
            home_x,
            0.0 + (self.robot_id * 0.1),
        )  # Spread out slightly if multiple attackers

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ):
        if DEBUG_ROBOT_BEHAVIOR and self.robot_id == 2:
            self.debug_robot_state(vision_data, referee_data)
        self._update_home_position()
        ball_pos = self._get_ball_pos(vision_data)
        base_ref_cmd = self.last_game_command_from_referee.replace("_BLUE", "").replace(
            "_YELLOW", ""
        )
        ref_cmd_team = self.get_team_from_command_suffix(
            self.last_game_command_from_referee
        )
        opp_goal_center = (-self.own_goal_line_x, 0.0)

        if self.current_state == RobotState.PREPARING_SET_PIECE:
            if base_ref_cmd == "PREPARE_KICKOFF" and self.is_designated_kicker:
                self.target_position = self._calculate_ball_approach_for_kick(
                    (0.0, 0.0), opp_goal_center
                )  # Approach center spot
            elif base_ref_cmd == "PREPARE_PENALTY" and self.is_designated_kicker:
                penalty_spot = self._get_penalty_spot(for_us_to_attack=True)
                self.target_position = self._calculate_ball_approach_for_kick(
                    penalty_spot, opp_goal_center
                )
            elif (
                base_ref_cmd in ["PREPARE_FREEKICK", "CORNER_KICK"]
                and self.is_designated_kicker
                and ball_pos
            ):
                self.target_position = self._calculate_ball_approach_for_kick(
                    ball_pos, opp_goal_center
                )
            elif (
                self.current_state == RobotState.BALL_PLACEMENT_ACTIVE
                and base_ref_cmd == "BALL_PLACEMENT"
            ):  # (is_designated_kicker used for placer)
                designated_target = referee_data.get("designated_position")
                if ball_pos and designated_target:
                    dist_to_ball = self._distance(current_pos, ball_pos)
                    if (
                        self.has_ball
                        or dist_to_ball < ATTACKER_BALL_OWNERSHIP_THRESHOLD
                    ):
                        self.has_ball = True
                        self.target_position = designated_target
                        if DEBUG_ROBOT_BEHAVIOR:
                            print(
                                f"ATT {self.robot_id} (Placer): Has ball, moving to {designated_target}"
                            )
                        # Check if ball is at designated target
                        if (
                            self._distance(ball_pos, designated_target)
                            < BALL_PLACEMENT_SUCCESS_RADIUS
                        ):
                            if DEBUG_ROBOT_BEHAVIOR:
                                print(
                                    f"ATT {self.robot_id} (Placer): Ball PLACED at {designated_target}!"
                                )
                            self.current_state = (
                                RobotState.IDLE
                            )  # Placement done by this robot
                            # GC will continue game
                    else:
                        self.has_ball = False
                        self.target_position = ball_pos  # Go to ball first
                else:
                    self.target_position = ball_pos if ball_pos else self.home_position
            else:  # Not kicker or other set piece
                self.target_position = (
                    self.home_position
                )  # Default for non-kickers/other situations
            self._process_movement(current_pos)
            return

        if (
            self.current_state == RobotState.TAKING_SET_PIECE
        ):  # After NORMAL_START from PREPARE_X
            if (
                base_ref_cmd
                in [
                    "PREPARE_KICKOFF",
                    "PREPARE_PENALTY",
                    "PREPARE_FREEKICK",
                    "CORNER_KICK",
                ]
                and self.is_designated_kicker
            ):
                # Determine ball position for this set piece
                actual_ball_pos_for_kick = (0, 0)  # Default for kickoff
                if base_ref_cmd == "PREPARE_PENALTY":
                    actual_ball_pos_for_kick = self._get_penalty_spot(
                        for_us_to_attack=True
                    )
                elif ball_pos:
                    actual_ball_pos_for_kick = ball_pos  # For FK, Corner

                if (
                    self._distance(current_pos, actual_ball_pos_for_kick)
                    < ATTACKER_BALL_OWNERSHIP_THRESHOLD + 0.02
                ):  # Close enough to kick
                    if DEBUG_ROBOT_BEHAVIOR:
                        print(f"ATT {self.robot_id}: KICKING for {base_ref_cmd}!")
                    self.game.active_controller.kick_flat(
                        self.robot_id, MAX_KICK_SPEED_GRSIM * 0.7
                    )
                    self.current_state = RobotState.SUPPORTING_OFFENSE  # Or IDLE
                    self.is_designated_kicker = False  # Kick taken
                else:  # Not at ball yet
                    self.target_position = self._calculate_ball_approach_for_kick(
                        actual_ball_pos_for_kick, opp_goal_center
                    )
            else:  # Should not be in TAKING_SET_PIECE if not kicker or unknown set piece
                self.current_state = RobotState.RETURNING
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return

        # Normal Play (FORCE_START, or after set piece kick)
        if not ball_pos:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            self.has_ball = False
        else:
            dist_to_ball = self._distance(current_pos, ball_pos)
            if dist_to_ball < ATTACKER_BALL_OWNERSHIP_THRESHOLD:
                self.has_ball = True
            else:
                self.has_ball = False

            if self.has_ball:
                self.current_state = RobotState.ATTACKING
                # Rule 8.3.2: Attacker Touched Ball In Opponent Defense Area
                if self._is_point_in_defense_area(
                    current_pos, "opponent"
                ):  # Check robot's position with ball
                    if DEBUG_ROBOT_BEHAVIOR:
                        print(
                            f"ATT {self.robot_id}: Has ball IN OPPONENT DEFENSE AREA."
                        )
                    # Kick if aimed, otherwise try to move out slightly
                    current_orientation = self._get_current_orientation()
                    if current_orientation and abs(
                        self._normalize_angle(
                            math.atan2(
                                opp_goal_center[1] - current_pos[1],
                                opp_goal_center[0] - current_pos[0],
                            )
                            - current_orientation
                        )
                    ) < math.radians(20):
                        self.game.active_controller.kick_flat(
                            self.robot_id, MAX_KICK_SPEED_GRSIM * 0.9
                        )
                        self.current_state = RobotState.IDLE
                        self.target_position = current_pos
                    else:  # Move towards edge of defense area
                        target_x_out = (
                            -self.own_goal_line_x
                            - (DEFENSE_AREA_DEPTH_DEFAULT + 0.1)
                            * self.attack_direction_x_sign
                        )  # Just outside
                        self.target_position = (target_x_out, current_pos[1])
                else:  # Has ball, not in opponent D-Area: Can we kick?
                    current_orientation = self._get_current_orientation()
                    dist_to_opp_goal = self._distance(current_pos, opp_goal_center)
                    if current_orientation and dist_to_opp_goal < (
                        (self.field_bounds["x_max"] - self.field_bounds["x_min"]) / 2.0
                        + 0.5
                    ):  # Within ~half field + bit more
                        angle_to_goal = math.atan2(
                            opp_goal_center[1] - current_pos[1],
                            opp_goal_center[0] - current_pos[0],
                        )
                        if abs(
                            self._normalize_angle(angle_to_goal - current_orientation)
                        ) < math.radians(
                            15
                        ):  # Aimed
                            if DEBUG_ROBOT_BEHAVIOR:
                                print(f"ATT {self.robot_id}: KICKING at goal!")
                            self.game.active_controller.kick_flat(
                                self.robot_id, MAX_KICK_SPEED_GRSIM
                            )
                            self.current_state = RobotState.IDLE
                            self.target_position = current_pos
                        else:  # Dribble/position
                            self.target_position = opp_goal_center
                    else:  # Too far or no orientation, dribble
                        self.target_position = opp_goal_center
            else:  # Don't have ball
                # Simplified: SSL-EL attacker (Robot 2) always goes for ball if free
                is_main_attacker = (
                    self.role == RobotRole.ATTACKER and self.robot_id == 2
                )  # TODO: make this dynamic
                if is_main_attacker or self._is_closest_teammate_to_ball(
                    current_pos, ball_pos, vision_data
                ):
                    self.current_state = RobotState.MOVING_TO_BALL
                    self.target_position = self._calculate_ball_approach_for_kick(
                        ball_pos, opp_goal_center
                    )
                else:  # Other attackers support
                    self.current_state = RobotState.SUPPORTING_OFFENSE
                    self.target_position = self._calculate_offensive_support_position(
                        ball_pos, vision_data, current_pos
                    )
        self._process_movement(current_pos)

    def _is_closest_teammate_to_ball(
        self,
        my_pos: Tuple[float, float],
        ball_pos: Tuple[float, float],
        vision_data: Dict,
    ) -> bool:
        my_dist = self._distance(my_pos, ball_pos)
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        for r_id_str, r_data in vision_data.get(team_key, {}).items():
            r_id = int(r_id_str)
            if r_id == self.robot_id or r_data.get("x") is None:
                continue
            if self._distance((r_data["x"], r_data["y"]), ball_pos) < my_dist - 0.1:
                return False  # Teammate clearly closer
        return True

    def _calculate_offensive_support_position(
        self,
        ball_pos: Tuple[float, float],
        vision_data: Dict,
        current_pos: Tuple[float, float],
    ) -> Tuple[float, float]:
        opp_goal_x, opp_goal_y = (-self.own_goal_line_x, 0.0)
        dir_ball_goal_x, dir_ball_goal_y = (
            opp_goal_x - ball_pos[0],
            opp_goal_y - ball_pos[1],
        )
        norm_dir = self._distance((0, 0), (dir_ball_goal_x, dir_ball_goal_y))
        if norm_dir == 0:
            return self.home_position

        dist_ahead = 0.8 + (self.robot_id * 0.15)  # Varied support distance
        point_ahead_x = ball_pos[0] + (dir_ball_goal_x / norm_dir) * dist_ahead
        point_ahead_y = ball_pos[1] + (dir_ball_goal_y / norm_dir) * dist_ahead

        side_offset = 0.6
        side_sign = (
            1 if (self.robot_id % 2 == 0 and self.robot_id != 2) else -1
        )  # Main attacker (2) might take center
        if self.robot_id == 2:
            side_sign = (
                0  # Main attacker stays more central if not primary ball pursuer
            )

        perp_x, perp_y = -dir_ball_goal_y / norm_dir, dir_ball_goal_x / norm_dir
        final_x = point_ahead_x + perp_x * side_offset * side_sign
        final_y = point_ahead_y + perp_y * side_offset * side_sign
        return self._enforce_field_boundaries((final_x, final_y))

    def _calculate_ball_approach_for_kick(
        self, ball_pos: Tuple[float, float], kick_target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        offset_dist = (
            ATTACKER_BALL_OWNERSHIP_THRESHOLD * 0.8
        )  # Approach slightly inside ownership radius
        dx, dy = kick_target_pos[0] - ball_pos[0], kick_target_pos[1] - ball_pos[1]
        norm = self._distance((0, 0), (dx, dy))
        if (
            norm == 0
        ):  # Target is at ball pos, approach from behind relative to current orientation or own goal
            current_orientation = self._get_current_orientation()
            if current_orientation is not None:
                approach_x = ball_pos[0] - offset_dist * math.cos(
                    current_orientation + math.pi
                )  # Approach from behind robot
                approach_y = ball_pos[1] - offset_dist * math.sin(
                    current_orientation + math.pi
                )
                return (approach_x, approach_y)
            else:  # Fallback: approach from own goal side
                return (
                    ball_pos[0] - offset_dist * self.attack_direction_x_sign,
                    ball_pos[1],
                )

        return (
            ball_pos[0] - (dx / norm * offset_dist),
            ball_pos[1] - (dy / norm * offset_dist),
        )
