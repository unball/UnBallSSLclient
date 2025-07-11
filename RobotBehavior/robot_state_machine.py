import math
import time
from threading import Lock
from typing import Tuple, Dict, Optional, List, Any
from .robot_states import RobotState, RobotRole
from utils.logger import get_logger

try:
    from GoalkeeperOptimization.goalkeeper_optimizer_simple import (
        SimpleGoalkeeperOptimizer,
    )

    SIMPLE_OPTIMIZER_AVAILABLE = True
    print("SimpleGoalkeeperOptimizer imported successfully")
except ImportError as e:
    SIMPLE_OPTIMIZER_AVAILABLE = False
    print(f"SimpleGoalkeeperOptimizer not available: {e}")
    exit()

# --- Constantes para Ajuste de Comportamento e Regras da SSL-EL ---
DEBUG_ROBOT_BEHAVIOR = False

# --- Constantes do Planejador de Trajetória (Path Planning) ---
PATH_LOOKAHEAD_DISTANCE = 0.2  # Distância (m) à frente no caminho que o robô mira.
PATH_REPLAN_INTERVAL = 0.3  # Intervalo (s) para solicitar um novo caminho.
PATH_TARGET_CHANGE_THRESHOLD = (
    0.1  # Distância (m) que o alvo precisa mudar para forçar um novo planejamento.
)

# --- Constantes de Movimento ---
ROBOT_MAX_LINEAR_SPEED = 1.5  # m/s (Regra 8.3.2 da SSL-EL)
ROBOT_MAX_SPEED_STOP_STATE = 0.75  # m/s (Regra 8.3.3 da SSL-EL para o estado STOP)
ROBOT_MAX_ANGULAR_SPEED = math.pi * 2.5  # rad/s, aumentado para maior agilidade.
APPROACH_SLOWDOWN_DISTANCE = (
    0.4  # m, distância para começar a desacelerar ao se aproximar do alvo.
)
FINE_TUNE_DISTANCE = 0.01  # m, distância para considerar que o robô chegou ao alvo.
TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_HIGH = (
    math.pi / 2
)  # 90 graus, limiar para grande redução de velocidade em curvas.
TURN_SPEED_REDUCTION_FACTOR_HIGH = 0.3  # Fator de redução para curvas acentuadas.
TURN_SPEED_REDUCTION_ANGLE_THRESHOLD_LOW = (
    math.pi / 4
)  # 45 graus, limiar para pequena redução de velocidade em curvas.
TURN_SPEED_REDUCTION_FACTOR_LOW = 0.6  # Fator de redução para curvas suaves.
FIELD_BOUNDARY_MARGIN = 0.05  # m, margem para evitar que o robô saia do campo.
IMMEDIATE_BOUNDARY_STOP_MARGIN = (
    0.02  # m, margem muito próxima da borda para parada imediata.
)

# --- Constantes das Regras de Jogo (SSL-EL) ---
BALL_AVOIDANCE_DISTANCE_STOP_STATE = 0.5  # m (Regra 5.1.1)
GRACE_PERIOD_HALT_BRAKE = 2.0  # s (Regra 5.1.2)
KICK_RESTART_BALL_MOVE_DISTANCE = 0.05  # m (Regra 5.4)
KICK_TIMEOUT_SECONDS = 10.0  # s (Regra 5.4, 5.3.5.6)
MAX_KICK_SPEED_GRSIM = (
    6.0  # m/s (padrão do grSim, a velocidade real da bola é diferente)
)

DEFENDER_DISTANCE_FROM_BALL_OPPONENT_KICK = 0.5  # m (Regra 5.3.3 / 8.3.3)
DISTANCE_FROM_OPPONENT_DEFENSE_AREA_STOP_FK = 0.2  # m (Regra 8.3.1)
PENALTY_KICK_OTHER_ROBOTS_DISTANCE_BEHIND_BALL = 1.0  # m (Regra 5.3.5.3)

# --- Constantes de Posicionamento de Bola (Ball Placement) (Regra 5.2 & 8.3.3) ---
BALL_PLACEMENT_OPPONENT_STADIUM_DISTANCE = 0.5
BALL_PLACEMENT_SUCCESS_RADIUS = 0.15
BALL_PLACEMENT_MAX_TIME = 30.0
BALL_PLACEMENT_MIN_DIST_NEXT_CMD_FK = 0.05
BALL_PLACEMENT_MIN_DIST_NEXT_CMD_FORCE_START = 0.5

# --- Constantes Específicas de Papéis (Roles) ---
ATTACKER_BALL_OWNERSHIP_THRESHOLD = (
    0.12  # m, (raio do robô ~0.09 + raio da bola ~0.0215 = ~0.11)
)
GOALKEEPER_GOAL_LINE_X_OFFSET = 0.05
OWN_HALF_X_OFFSET = 0.05

# --- Constantes do Campo (Padrões para SSL-EL, atualizadas por game.field_bounds) ---
FIELD_LENGTH_DEFAULT = 4.5
FIELD_WIDTH_DEFAULT = 3.0
GOAL_WIDTH_DEFAULT = 0.8
GOAL_DEPTH_DEFAULT = 0.18
DEFENSE_AREA_DEPTH_DEFAULT = 0.5
DEFENSE_AREA_WIDTH_DEFAULT = 1.35
CENTER_CIRCLE_RADIUS_DEFAULT = 0.5
PENALTY_SPOT_ABS_X_DEFAULT = (FIELD_LENGTH_DEFAULT / 2.0) - 1.0


class PathFollower:
    """
    Implementa um seguidor de caminho simples usando o algoritmo Pure Pursuit.
    O robô mira em um ponto "lookahead" à frente em sua trajetória planejada.
    """

    def __init__(self, lookahead_distance: float = PATH_LOOKAHEAD_DISTANCE):
        """
        Inicializa o seguidor de caminho.

        Args:
            lookahead_distance: A distância à frente no caminho para mirar.
        """
        self.lookahead_distance = lookahead_distance
        self.logger = get_logger("path_follower")
        self.logger.debug(
            f"PathFollower initialized with lookahead: {self.lookahead_distance}m"
        )

    def get_target_point(
        self, current_pos: Tuple[float, float], path: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """
        Encontra o ponto alvo no caminho para o robô seguir.

        Args:
            current_pos: A posição atual do robô (x, y).
            path: A lista de pontos (x, y) que formam o caminho.

        Returns:
            O ponto alvo (lookahead) no caminho.
        """
        if not path:
            self.logger.warning(f"Empty path for current_pos {current_pos}")
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
        """Calcula a distância euclidiana entre dois pontos."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


class RobotStateMachine:
    """
    Máquina de estados base para um robô.
    Gerencia o estado atual, a lógica de movimento, o processamento de comandos do juiz
    e a interação com outros componentes como o planejador de caminho.
    """

    def __init__(self, robot_id: int, team_color: str, game: Any):
        """
        Inicializa a máquina de estados do robô.

        Args:
            robot_id: O ID do robô (0 a N-1).
            team_color: A cor da equipe ('blue' ou 'yellow').
            game: A instância principal do jogo, que contém dados de visão, juiz, etc.
        """
        self.robot_id = robot_id
        self.team_color = team_color.lower()
        self.game = game
        self.logger = get_logger(f"robot_behavior.sm.{self.robot_id}")

        self.current_state: RobotState = RobotState.IDLE
        self.role: Optional[RobotRole] = None
        self.target_position: Optional[Tuple[float, float]] = None

        self.state_lock = Lock()
        self.path_follower = PathFollower()
        self.current_path: List[Tuple[float, float]] = []
        self.last_path_request_time: float = 0.0
        self.last_target_for_path: Optional[Tuple[float, float]] = None

        self.last_game_command_from_referee: str = ""
        self.is_designated_kicker: bool = (
            False  # Usado para cobrador de faltas e posicionador de bola.
        )

        self.home_position: Tuple[float, float] = (0.0, 0.0)
        self.field_bounds = game.field_bounds.copy()
        self.goal_half_width = GOAL_WIDTH_DEFAULT / 2.0
        self.own_goal_line_x = 0.0
        self.attack_direction_x_sign = 1

        self._update_geometry_references()

        self.logger.info(
            f"RobotStateMachine {self.robot_id} ({self.team_color}) initialized. Attack sign: {self.attack_direction_x_sign}. OwnGoalX: {self.own_goal_line_x}"
        )

    def _update_geometry_references(self):
        """Atualiza as referências de geometria do campo com base nos dados mais recentes da visão."""
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
        """Extrai a cor da equipe ('blue' ou 'yellow') de um comando do juiz."""
        if game_command.endswith("_BLUE"):
            return "blue"
        if game_command.endswith("_YELLOW"):
            return "yellow"
        return None

    def _get_current_pos(self) -> Optional[Tuple[float, float]]:
        """Obtém a posição atual do robô a partir dos dados da visão."""
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return None
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robot_data = vision_data.get(team_key, {}).get(self.robot_id)
        if robot_data and robot_data.get("x") is not None:
            return (robot_data["x"], robot_data["y"])
        return None

    def _get_current_orientation(self) -> Optional[float]:
        """Obtém a orientação atual do robô (em radianos) a partir dos dados da visão."""
        vision_data = self.game.get_vision_data()
        if not vision_data:
            return None
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        robot_data = vision_data.get(team_key, {}).get(self.robot_id)
        if robot_data and robot_data.get("theta") is not None:
            return robot_data["theta"]
        return None

    def _get_ball_pos(self, vision_data: Dict) -> Optional[Tuple[float, float]]:
        """Obtém a posição da bola a partir dos dados da visão."""
        ball_data = vision_data.get("ball")
        if ball_data and ball_data.get("x") is not None:
            return (ball_data["x"], ball_data["y"])
        return None

    def update(self, vision_data: Dict):
        """
        O principal método de atualização da máquina de estados, chamado a cada ciclo do jogo.

        Args:
            vision_data: O dicionário mais recente com dados da visão.
        """
        with self.state_lock:
            self._update_geometry_references()
            current_pos = self._get_current_pos()
            if not current_pos:
                self.logger.debug(f"Robot {self.robot_id}: No position, cannot update.")
                self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
                return

            referee_data = self.game.get_referee_data()
            game_command_from_ref = referee_data.get("command", "")

            if game_command_from_ref != self.last_game_command_from_referee:
                self.logger.debug(
                    f"Robot {self.robot_id}: New Ref CMD '{game_command_from_ref}', prev was '{self.last_game_command_from_referee}'"
                )
                self.last_game_command_from_referee = game_command_from_ref
                self.is_designated_kicker = False

            # Primeiro, lida com comandos que têm prioridade (HALT, STOP)
            action_taken = self._handle_referee_commands(
                vision_data, referee_data, current_pos, game_command_from_ref
            )
            # Se nenhum comando prioritário foi executado, decide a próxima ação com base no estado e papel.
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
        """
        Processa os comandos do juiz que exigem uma ação imediata e que sobrepõem a lógica normal.

        Args:
            vision_data: Dados da visão.
            referee_data: Dados do juiz.
            current_pos: Posição atual do robô.
            game_command: O comando atual do juiz.

        Returns:
            True se um comando foi tratado e a lógica normal deve ser ignorada, False caso contrário.
        """
        command_team_suffix = self.get_team_from_command_suffix(game_command)
        base_command = (
            game_command.replace("_BLUE", "").replace("_YELLOW", "")
            if command_team_suffix
            else game_command
        )

        if (
            base_command not in ["NORMAL_START", "FORCE_START"]
            or self.current_state == RobotState.IDLE
        ):
            self.logger.debug(
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

        prev_base_cmd_from_ref = self.last_game_command_from_referee.replace(
            "_BLUE", ""
        ).replace("_YELLOW", "")
        prev_cmd_team_suffix = self.get_team_from_command_suffix(
            self.last_game_command_from_referee
        )

        if base_command == "NORMAL_START":
            self.logger.debug(
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
                return False
            elif (
                prev_base_cmd_from_ref == "PREPARE_PENALTY"
                and prev_cmd_team_suffix == self.team_color
            ):
                self.current_state = (
                    RobotState.TAKING_SET_PIECE
                    if self.is_designated_kicker
                    else RobotState.IDLE
                )
                if not self.is_designated_kicker:
                    self.target_position = current_pos
                    self._process_movement(current_pos)
                    return True
                return False

            if self.current_state in [
                RobotState.IDLE,
                RobotState.PREPARING_SET_PIECE,
                RobotState.AVOIDING_BALL,
            ]:
                self.current_state = RobotState.RETURNING
            return False

        if base_command == "FORCE_START":
            self.logger.debug(f"SM {self.robot_id}: FORCE_START.")
            if self.current_state in [
                RobotState.IDLE,
                RobotState.PREPARING_SET_PIECE,
                RobotState.AVOIDING_BALL,
            ]:
                self.current_state = RobotState.RETURNING
            return False

        if base_command.startswith("PREPARE_"):
            self.logger.debug(
                f"SM {self.robot_id}: Preparing for {base_command} (for {command_team_suffix})."
            )
            self.current_state = RobotState.PREPARING_SET_PIECE
            if command_team_suffix == self.team_color:
                if (
                    base_command == "PREPARE_KICKOFF"
                    and self.role == RobotRole.ATTACKER
                    and self.robot_id == 2
                ):
                    self.is_designated_kicker = True
                elif (
                    base_command == "PREPARE_PENALTY"
                    and self.role == RobotRole.ATTACKER
                    and self.robot_id == 2
                ):
                    self.is_designated_kicker = True
            return False

        is_our_set_piece = command_team_suffix == self.team_color
        if base_command in [
            "KICK_OFF",
            "FREE_KICK",
            "PENALTY",
            "CORNER_KICK",
            "GOAL_KICK",
        ]:
            self.logger.debug(
                f"SM {self.robot_id}: Direct set piece CMD {base_command} for {command_team_suffix}."
            )
            self.current_state = RobotState.PREPARING_SET_PIECE
            if is_our_set_piece:
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
            return False

        if base_command == "BALL_PLACEMENT":
            designated_target_from_ref = referee_data.get("designated_position")
            self.logger.debug(
                f"SM {self.robot_id}: BALL_PLACEMENT for {command_team_suffix} to {designated_target_from_ref}."
            )
            if is_our_set_piece:
                self.current_state = RobotState.BALL_PLACEMENT_ACTIVE
                if self.role == RobotRole.ATTACKER and self.robot_id == 2:
                    self.is_designated_kicker = True
            else:
                self.current_state = RobotState.BALL_PLACEMENT_AVOIDING
            return False

        return False

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ):
        """
        Decide a próxima ação para o robô com base em seu estado e papel.
        Este método é uma implementação base e deve ser sobrescrito pelas classes filhas.
        """
        if self.current_state == RobotState.PREPARING_SET_PIECE:
            self.logger.debug(
                f"Robot {self.robot_id}: In PREPARING_SET_PIECE, (base FSM) defaulting to home."
            )
            self.target_position = self.home_position
        elif self.current_state == RobotState.IDLE or (
            not self.target_position and self.current_state != RobotState.RETURNING
        ):
            self.logger.debug(f"Robot {self.robot_id}: Idle/no target, RETURNING home.")
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
        if (
            self.current_state == RobotState.RETURNING
            and self.target_position == self.home_position
            and self._distance(current_pos, self.home_position) < FINE_TUNE_DISTANCE
        ):
            self.logger.debug(f"Robot {self.robot_id}: Arrived home, IDLE.")
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
        """
        Gerencia o processo de movimento, incluindo a solicitação de caminho e o seguimento.

        Args:
            current_pos: A posição atual do robô.
            max_speed_override: Uma velocidade máxima para sobrepor a padrão.
        """
        if self.target_position:
            self._request_path_if_needed(current_pos, self.target_position)
            self._follow_path(current_pos, max_speed_override)
        elif self.current_state != RobotState.IDLE:
            self.logger.warning(
                f"Robot {self.robot_id}: No target, but not IDLE. Stopping."
            )
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            self.current_path = []

    def _request_path_if_needed(
        self, current_pos: Tuple[float, float], target_pos: Tuple[float, float]
    ):
        """
        Solicita um novo caminho ao planejador se o alvo mudou ou o tempo de replanejamento expirou.

        Args:
            current_pos: A posição inicial do caminho.
            target_pos: A posição final do caminho.
        """
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
            self.logger.debug(
                f"Robot {self.robot_id}: Requesting path for {reason} to {target_pos}"
            )
            # Garante que a posição alvo e a inicial estão dentro dos limites para o planejador
            safe_target = self._enforce_field_boundaries(target_pos)
            safe_start = self._enforce_field_boundaries(current_pos)

            self.game.path_planner.request_path(self.robot_id, safe_start, safe_target)
            self.last_path_request_time = now
            self.last_target_for_path = safe_target

    def _follow_path(
        self,
        current_pos: Tuple[float, float],
        max_speed_override: Optional[float] = None,
    ):
        """Segue o caminho atual ou move-se diretamente para o ponto se não houver caminho."""
        current_orientation = self._get_current_orientation()
        if current_orientation is None:
            self.logger.warning(
                f"Robot {self.robot_id}: No orientation for path following."
            )
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
                self.logger.debug(
                    f"Robot {self.robot_id}: Reached end of path segment."
                )
                if (
                    self.target_position
                    and self._distance(current_pos, self.target_position)
                    < FINE_TUNE_DISTANCE
                ):
                    self.logger.debug(
                        f"Robot {self.robot_id}: Arrived at target {self.target_position}. State: {self.current_state.name}"
                    )
                    if self.current_state in [
                        RobotState.MOVING_TO_POSITION,
                        RobotState.RETURNING,
                        RobotState.AVOIDING_BALL,
                        RobotState.SUPPORTING_OFFENSE,
                        RobotState.DEFENDING,
                        RobotState.BALL_PLACEMENT_AVOIDING,
                    ]:
                        self.current_state = RobotState.IDLE
                        self.game.active_controller.send_global_velocity(
                            self.robot_id, 0, 0, 0
                        )
                    self.target_position = None
                    self.current_path = []
        elif self.target_position:
            self.logger.debug(
                f"Robot {self.robot_id}: No path, direct move to {self.target_position}."
            )
            self._move_to_point(
                current_pos,
                current_orientation,
                self.target_position,
                max_speed_override,
            )
        elif self.current_state != RobotState.IDLE:
            self.logger.warning(
                f"Robot {self.robot_id}: No path, no target, not IDLE. Stopping."
            )
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)

    def _enforce_field_boundaries(
        self, target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Garante que uma posição esteja dentro dos limites jogáveis do campo."""
        x = max(
            self.field_bounds["x_min"] + FIELD_BOUNDARY_MARGIN,
            min(self.field_bounds["x_max"] - FIELD_BOUNDARY_MARGIN, target_pos[0]),
        )
        y = max(
            self.field_bounds["y_min"] + FIELD_BOUNDARY_MARGIN,
            min(self.field_bounds["y_max"] - FIELD_BOUNDARY_MARGIN, target_pos[1]),
        )
        if (x, y) != target_pos:
            self.logger.debug(
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
        """
        Calcula e envia as velocidades (vx, vy, w) para mover o robô em direção a um ponto.

        Args:
            current_pos: Posição atual do robô.
            current_orientation: Orientação atual do robô.
            target_point: O ponto de destino.
            max_speed_override: Velocidade máxima opcional.
        """
        safe_target = self._enforce_field_boundaries(target_point)
        dx, dy = safe_target[0] - current_pos[0], safe_target[1] - current_pos[1]
        dist = self._distance((0, 0), (dx, dy))

        max_speed = (
            max_speed_override
            if max_speed_override is not None
            else ROBOT_MAX_LINEAR_SPEED
        )
        speed_factor = 1.0
        if dist < FINE_TUNE_DISTANCE:
            speed_factor = 0.0
        elif dist < APPROACH_SLOWDOWN_DISTANCE:
            min_sf = 0.1
            speed_factor = min_sf + (dist - FINE_TUNE_DISTANCE) / (
                APPROACH_SLOWDOWN_DISTANCE - FINE_TUNE_DISTANCE
            ) * (1.0 - min_sf)
            speed_factor = max(min_sf, min(1.0, speed_factor))

        angle_to_target = math.atan2(dy, dx)
        orient_err = self._normalize_angle(angle_to_target - current_orientation)

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

        if hasattr(self.game, "active_controller") and self.game.active_controller:
            self.game.active_controller.send_command_with_kick(
                self.robot_id, vx, vy, ang_vel, 0, 0, 0
            )
        else:
            self.logger.error(
                f"R{self.robot_id} ERROR: No active_controller for _move_to_point!"
            )

    def _normalize_angle(self, angle: float) -> float:
        """Normaliza um ângulo para o intervalo [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calcula a distância euclidiana entre dois pontos."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _is_point_in_defense_area(
        self, point: Tuple[float, float], which_area: str = "own"
    ) -> bool:
        """Verifica se um ponto está dentro da área de defesa (própria ou adversária)."""
        depth = DEFENSE_AREA_DEPTH_DEFAULT
        half_width = DEFENSE_AREA_WIDTH_DEFAULT / 2.0

        goal_line_x_ref = (
            self.own_goal_line_x if which_area == "own" else -self.own_goal_line_x
        )
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
        """Calcula a coordenada da marca do pênalti."""
        spot_x_sign = (
            -self.attack_direction_x_sign
            if for_us_to_attack
            else self.attack_direction_x_sign
        )
        return (PENALTY_SPOT_ABS_X_DEFAULT * spot_x_sign, 0.0)

    def debug_robot_state(self, vision_data: Dict, referee_data: Optional[Dict]):
        """Imprime o estado completo do robô para depuração."""
        current_pos = self._get_current_pos()
        current_orient = self._get_current_orientation()
        ball_pos = self._get_ball_pos(vision_data)
        self.logger.debug(
            f"\n--- DEBUG: R{self.robot_id} ({self.team_color} - {self.role.name if self.role else 'NIL'}) ---"
        )
        self.logger.debug(
            f"State: {self.current_state.name}, Kicker: {self.is_designated_kicker}"
        )
        self.logger.debug(
            f"Pos: {current_pos}, Orient: {current_orient:.2f}r"
            if current_pos and current_orient is not None
            else "Pos/Orient: N/A"
        )
        self.logger.debug(f"Target: {self.target_position}")
        path_len = len(self.current_path)
        self.logger.debug(
            f"Path: {path_len}pts {'('+str(self.current_path[0])+'...'+str(self.current_path[-1])+')' if path_len > 1 else str(self.current_path)}"
        )
        self.logger.debug(f"Ball: {ball_pos}" if ball_pos else "Ball: N/A")
        if referee_data:
            self.logger.debug(
                f"RefCMD: {referee_data.get('command', 'N/A')}, Stage: {referee_data.get('stage', 'N/A')}, ForTeam: {referee_data.get('team_color_for_command', 'N/A')}"
            )
        self.logger.debug(f"LastRefCMDseen: {self.last_game_command_from_referee}")
        self.logger.debug(f"------------------------------------")

        if hasattr(self, "optimization_stats") and self.optimization_stats["calls"] > 0:
            stats = self.optimization_stats
            success_rate = (
                stats["successes"] / stats["calls"] * 100 if stats["calls"] > 0 else 0
            )
            self.logger.debug(
                f"Optimization: {stats['calls']} calls, {success_rate:.1f}% success"
            )


class GoalkeeperStateMachine(RobotStateMachine):
    """
    Máquina de estados especializada para o goleiro.
    Sua principal responsabilidade é defender o gol, bloqueando a bola
    e se posicionando corretamente na linha do gol.
    """

    def __init__(self, robot_id: int, team_color: str, game: Any):
        """Inicializa a FSM do goleiro com otimização opcional."""
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.GOALKEEPER
        self.logger = get_logger(f"robot_behavior.goalkeeper.{self.robot_id}")
        self._update_home_position()
        self.target_position = self.home_position

        self.use_optimization = True  # Flag para ativar/desativar
        self.optimizer = None
        self.optimization_stats = {
            "calls": 0,
            "successes": 0,
            "failures": 0,
            "total_time_ms": 0.0,
        }

        # Inicializar otimizador se disponível
        if SIMPLE_OPTIMIZER_AVAILABLE and self.use_optimization:
            try:
                self.optimizer = SimpleGoalkeeperOptimizer()
                self.logger.info(f"🎯 GK {self.robot_id}: OPTIMIZATION ENABLED")
            except Exception as e:
                self.logger.warning(f"GK {self.robot_id}: Optimizer init failed: {e}")
                self.optimizer = None
        else:
            self.logger.info(f"📋 GK {self.robot_id}: Using ORIGINAL method")

        self.logger.info(
            f"GK {self.robot_id} ({self.team_color}) Initialized. Home: {self.home_position}"
        )

    def _update_home_position(self):
        """Calcula a posição inicial (home) do goleiro, ligeiramente à frente da linha do gol."""
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
        """Lógica de decisão principal para o goleiro."""
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
                    )
                    self.target_position = (self.own_goal_line_x, target_y)
                else:
                    self.target_position = self.home_position
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
                    self.target_position = self.home_position
            else:
                self.target_position = self.home_position
            self._process_movement_with_orientation(current_pos)
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
                self.logger.info(f"GK {self.robot_id}: KICKING for Goal Kick.")
                self.game.active_controller.kick_flat(
                    self.robot_id, MAX_KICK_SPEED_GRSIM * 0.8
                )
                self.current_state = RobotState.RETURNING
            elif ball_pos:
                self.target_position = self._calculate_ball_approach_for_kick(
                    ball_pos, (-self.own_goal_line_x, 0.0)
                )
            else:
                self.current_state = RobotState.RETURNING
                self.target_position = self.home_position
            self._process_movement_with_orientation(current_pos)
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
        self._process_movement_with_orientation(current_pos)

    def _is_ball_threatening(self, ball_pos: Tuple[float, float]) -> bool:
        """Verifica se a bola está no terço defensivo do campo, representando uma ameaça."""
        defensive_third_x = self.own_goal_line_x + (
            (self.field_bounds["x_max"] - self.field_bounds["x_min"])
            / 3.0
            * self.attack_direction_x_sign
        )
        if self.attack_direction_x_sign == 1:  # Ataca para a direita (gol à esquerda)
            return ball_pos[0] < defensive_third_x
        return ball_pos[0] > defensive_third_x  # Ataca para a esquerda (gol à direita)

    def _calculate_blocking_position(
        self, ball_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Calcula a melhor posição na linha do gol para bloquear um chute.
        VERSÃO OTIMIZADA: Usa SimpleGoalkeeperOptimizer quando disponível.
        """
        import time

        start_time = time.time()
        self.optimization_stats["calls"] += 1

        # Tentar usar otimização
        if self.optimizer and self.use_optimization:
            try:
                optimal_pos = self.optimizer.calculate_optimal_position(ball_pos)
                self.optimization_stats["successes"] += 1

                # Calcular tempo
                solve_time = (time.time() - start_time) * 1000
                self.optimization_stats["total_time_ms"] += solve_time

                # Log de comparação detalhado se debug ativo
                if DEBUG_ROBOT_BEHAVIOR:
                    original_pos = self._calculate_blocking_position_original(ball_pos)
                    distance = math.sqrt(
                        (optimal_pos[0] - original_pos[0]) ** 2
                        + (optimal_pos[1] - original_pos[1]) ** 2
                    )
                    self.logger.debug(
                        f"GK {self.robot_id}: Ball({ball_pos[0]:.2f},{ball_pos[1]:.2f}) -> "
                        f"OPTIMIZED({optimal_pos[0]:.2f},{optimal_pos[1]:.2f}) vs "
                        f"ORIGINAL({original_pos[0]:.2f},{original_pos[1]:.2f}) "
                        f"[diff: {distance:.3f}m, time: {solve_time:.1f}ms]"
                    )

                return optimal_pos

            except Exception as e:
                self.optimization_stats["failures"] += 1
                self.logger.warning(f"GK {self.robot_id}: Optimization failed: {e}")

        # Fallback para método original
        return self._calculate_blocking_position_original(ball_pos)

    def _process_movement_with_orientation(
        self,
        current_pos: Tuple[float, float],
        max_speed_override: Optional[float] = None,
    ):
        """
        Versão melhorada do processamento de movimento que inclui orientação.

        ADICIONAR este método na classe GoalkeeperStateMachine.
        MODIFICAR _decide_next_action para chamar este método em vez de _process_movement.
        """
        if self.target_position:
            # Processar movimento normal
            self._request_path_if_needed(current_pos, self.target_position)

            # NOVO: Calcular orientação ideal se há bola visível
            vision_data = self.game.get_vision_data()
            ball_pos = self._get_ball_pos(vision_data) if vision_data else None

            if ball_pos:
                # Calcular orientação ideal
                target_orientation = self._calculate_optimal_orientation(
                    current_pos, ball_pos
                )

                # Seguir caminho com orientação específica
                self._follow_path_with_orientation(
                    current_pos, target_orientation, max_speed_override
                )
            else:
                # Sem bola visível, movimento normal
                self._follow_path(current_pos, max_speed_override)

        elif self.current_state != RobotState.IDLE:
            self.logger.warning(
                f"Robot {self.robot_id}: No target, but not IDLE. Stopping."
            )
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            self.current_path = []

    def _follow_path_with_orientation(
        self,
        current_pos: Tuple[float, float],
        target_orientation: float,
        max_speed_override: Optional[float] = None,
    ):
        """
        Segue o caminho mantendo orientação específica para a bola.

        ADICIONAR este método na classe GoalkeeperStateMachine.
        """
        current_orientation = self._get_current_orientation()
        if current_orientation is None:
            self.logger.warning(
                f"Robot {self.robot_id}: No orientation for path following."
            )
            self.game.active_controller.send_global_velocity(self.robot_id, 0, 0, 0)
            return

        path = self.game.path_planner.get_path(self.robot_id)
        if path and path != self.current_path:
            self.current_path = path

        if self.current_path:
            lookahead = self.path_follower.get_target_point(
                current_pos, self.current_path
            )
            self._move_to_point_with_orientation(
                current_pos,
                current_orientation,
                lookahead,
                target_orientation,
                max_speed_override,
            )

            # Verificar se chegou ao destino
            if (
                self._distance(current_pos, self.current_path[-1]) < 0.01
            ):  # FINE_TUNE_DISTANCE
                self.logger.debug(
                    f"Robot {self.robot_id}: Reached end of path segment."
                )
                if (
                    self.target_position
                    and self._distance(current_pos, self.target_position) < 0.01
                ):

                    self.logger.debug(
                        f"Robot {self.robot_id}: Arrived at target {self.target_position}."
                    )
                    if self.current_state in [
                        RobotState.MOVING_TO_POSITION,
                        RobotState.RETURNING,
                        RobotState.BLOCKING,
                        RobotState.DEFENDING,
                    ]:
                        self.current_state = RobotState.IDLE
                        # Manter orientação para a bola mesmo parado
                        angular_vel = self._calculate_angular_velocity(
                            current_orientation, target_orientation
                        )
                        self.game.active_controller.send_global_velocity(
                            self.robot_id, 0, 0, angular_vel
                        )

                    self.target_position = None
                    self.current_path = []

        elif self.target_position:
            self.logger.debug(
                f"Robot {self.robot_id}: No path, direct move to {self.target_position}."
            )
            self._move_to_point_with_orientation(
                current_pos,
                current_orientation,
                self.target_position,
                target_orientation,
                max_speed_override,
            )

        elif self.current_state != RobotState.IDLE:
            self.logger.warning(
                f"Robot {self.robot_id}: No path, no target, not IDLE. Stopping with orientation."
            )
            # Manter orientação mesmo parado
            angular_vel = self._calculate_angular_velocity(
                current_orientation, target_orientation
            )
            self.game.active_controller.send_global_velocity(
                self.robot_id, 0, 0, angular_vel
            )

    def _move_to_point_with_orientation(
        self,
        current_pos: Tuple[float, float],
        current_orientation: float,
        target_point: Tuple[float, float],
        target_orientation: float,
        max_speed_override: Optional[float] = None,
    ):
        """
        Move para um ponto mantendo orientação específica.
        """
        safe_target = self._enforce_field_boundaries(target_point)
        dx, dy = safe_target[0] - current_pos[0], safe_target[1] - current_pos[1]
        dist = self._distance((0, 0), (dx, dy))

        # Calcular velocidade linear (igual ao método original)
        max_speed = (
            max_speed_override if max_speed_override is not None else 1.5
        )  # ROBOT_MAX_LINEAR_SPEED
        speed_factor = 1.0

        if dist < 0.01:  # FINE_TUNE_DISTANCE
            speed_factor = 0.0
        elif dist < 0.4:  # APPROACH_SLOWDOWN_DISTANCE
            min_sf = 0.1
            speed_factor = min_sf + (dist - 0.01) / (0.4 - 0.01) * (1.0 - min_sf)
            speed_factor = max(min_sf, min(1.0, speed_factor))

        final_lin_speed = max_speed * speed_factor
        vx, vy = (
            (dx / dist * final_lin_speed) if dist > 0 else 0,
            (dy / dist * final_lin_speed) if dist > 0 else 0,
        )

        # NOVO: Calcular velocidade angular para orientação
        angular_vel = self._calculate_angular_velocity(
            current_orientation, target_orientation
        )

        # Verificar limites do campo (igual ao método original)
        field_bounds = self.field_bounds
        if (current_pos[0] <= field_bounds["x_min"] + 0.02 and vx < 0) or (
            current_pos[0] >= field_bounds["x_max"] - 0.02 and vx > 0
        ):
            vx = 0
        if (current_pos[1] <= field_bounds["y_min"] + 0.02 and vy < 0) or (
            current_pos[1] >= field_bounds["y_max"] - 0.02 and vy > 0
        ):
            vy = 0

        # Enviar comando com orientação
        if hasattr(self.game, "active_controller") and self.game.active_controller:
            self.game.active_controller.send_command_with_kick(
                self.robot_id, vx, vy, angular_vel, 0, 0, 0
            )
        else:
            self.logger.error(f"R{self.robot_id} ERROR: No active_controller!")

    def _calculate_angular_velocity(
        self, current_orientation: float, target_orientation: float
    ) -> float:
        """
        Calcula velocidade angular para atingir orientação desejada.

        ADICIONAR este método na classe GoalkeeperStateMachine.
        """
        # Calcular erro angular
        orient_err = self._normalize_angle(target_orientation - current_orientation)

        # Controle proporcional para orientação
        ang_p_gain = 4.0  # Ganho mais alto para resposta mais rápida
        max_angular_speed = math.pi * 2.0  # Velocidade angular máxima

        angular_vel = max(
            -max_angular_speed, min(max_angular_speed, orient_err * ang_p_gain)
        )

        return angular_vel

    def _calculate_optimal_orientation(
        self, current_pos: Tuple[float, float], ball_pos: Tuple[float, float]
    ) -> float:
        """
        Calcula a orientação ideal do goleiro para ficar sempre de frente para a bola.

        Args:
            current_pos: Posição atual do goleiro
            ball_pos: Posição da bola

        Returns:
            Ângulo em radianos para o goleiro ficar de frente para a bola
        """
        # Calcular ângulo da posição atual para a bola
        dx = ball_pos[0] - current_pos[0]
        dy = ball_pos[1] - current_pos[1]

        # Ângulo para a bola
        angle_to_ball = math.atan2(dy, dx)

        # Log para debug
        if hasattr(self, "logger"):
            self.logger.debug(
                f"GK {self.robot_id}: Angle to ball: {math.degrees(angle_to_ball):.1f}°"
            )

        return angle_to_ball

    def _calculate_blocking_position_original(
        self, ball_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Método original de cálculo (mantido como fallback)."""
        target_y = max(
            -self.goal_half_width + 0.05, min(self.goal_half_width - 0.05, ball_pos[1])
        )
        target_x = self.own_goal_line_x + (
            GOALKEEPER_GOAL_LINE_X_OFFSET * self.attack_direction_x_sign
        )
        return (target_x, target_y)

    def debug_optimization_stats(self):
        """Mostra estatísticas de uso da otimização."""
        stats = self.optimization_stats
        if stats["calls"] > 0:
            success_rate = stats["successes"] / stats["calls"] * 100
            avg_time = (
                stats["total_time_ms"] / stats["calls"] if stats["calls"] > 0 else 0
            )

            self.logger.info(
                f"📊 GK {self.robot_id} Optimization Stats: "
                f"{stats['calls']} calls, {success_rate:.1f}% success, "
                f"{avg_time:.1f}ms avg, {stats['failures']} failures"
            )
        else:
            self.logger.info(f"📊 GK {self.robot_id}: No optimization calls yet")

    def _calculate_ball_approach_for_kick(
        self, ball_pos: Tuple[float, float], kick_target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calcula a posição de aproximação da bola para um chute."""
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
    """
    Máquina de estados especializada para o zagueiro.
    Atua principalmente no campo de defesa, interceptando a bola e apoiando o meio-campo.
    """

    def __init__(self, robot_id: int, team_color: str, game: Any):
        """Inicializa a FSM do zagueiro."""
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.DEFENDER
        self.logger = get_logger(f"robot_behavior.defender.{self.robot_id}")
        self._update_home_position()
        self.target_position = self.home_position
        self.logger.info(
            f"DEF {self.robot_id} ({self.team_color}) Initialized. Home: {self.home_position}"
        )

    def _update_home_position(self):
        """Calcula a posição inicial (home) do zagueiro no primeiro quarto do campo."""
        super()._update_geometry_references()
        home_x = self.own_goal_line_x + (
            (self.field_bounds["x_max"] - self.field_bounds["x_min"])
            * 0.25
            * self.attack_direction_x_sign
        )
        home_y_offset = DEFENSE_AREA_WIDTH_DEFAULT / 2.0 + 0.2
        home_y = home_y_offset if self.robot_id % 2 != 0 else -home_y_offset
        self.home_position = (home_x, home_y)

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ):
        """Lógica de decisão principal para o zagueiro."""
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
                )
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

        if (
            self.target_position
            and self._is_point_in_defense_area(self.target_position, "own")
            and self.current_state != RobotState.CLEARING_BALL
        ):
            self.logger.debug(
                f"DEF {self.robot_id}: Target {self.target_position} in own def area. Adjusting."
            )
            defense_area_edge_x = (
                self.own_goal_line_x
                + (DEFENSE_AREA_DEPTH_DEFAULT + 0.1) * self.attack_direction_x_sign
            )
            self.target_position = (defense_area_edge_x, self.target_position[1])

        self._process_movement(current_pos)

    def _is_ball_in_our_half(self, ball_pos: Tuple[float, float]) -> bool:
        """Verifica se a bola está na metade do campo da nossa equipe."""
        return (ball_pos[0] * self.attack_direction_x_sign) < (0.0 + OWN_HALF_X_OFFSET)

    def _calculate_intercept_or_block_position(
        self, ball_pos: Tuple[float, float], current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calcula uma posição defensiva entre a bola e o gol."""
        target_x = ball_pos[0] * 0.4 + self.own_goal_line_x * 0.6
        target_y = ball_pos[1] * 0.6
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
        """Calcula uma posição de espera no meio-campo para apoiar o ataque."""
        target_x = (OWN_HALF_X_OFFSET * 2.0) * self.attack_direction_x_sign
        target_y = ball_pos[1] * 0.3
        return self._enforce_field_boundaries((target_x, target_y))

    def _calculate_defensive_wall_position(
        self, ball_pos: Tuple[float, float], current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calcula a posição para formar uma barreira em cobranças de falta adversárias."""
        dist_from_ball = DEFENDER_DISTANCE_FROM_BALL_OPPONENT_KICK + 0.05
        angle_ball_to_our_goal = math.atan2(
            0.1 - ball_pos[1], self.own_goal_line_x - ball_pos[0]
        )
        target_x = ball_pos[0] + dist_from_ball * math.cos(angle_ball_to_our_goal)
        target_y = ball_pos[1] + dist_from_ball * math.sin(angle_ball_to_our_goal)
        return self._enforce_field_boundaries((target_x, target_y))

    def _calculate_support_position_our_set_piece(
        self, ball_pos: Optional[Tuple[float, float]], current_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calcula uma posição de apoio durante uma cobrança de falta da nossa equipe."""
        if ball_pos:
            target_x = ball_pos[0] - 1.2 * self.attack_direction_x_sign
            target_y = ball_pos[1] + (0.8 if self.robot_id % 2 != 0 else -0.8)
            return self._enforce_field_boundaries((target_x, target_y))
        return self.home_position


class AttackerStateMachine(RobotStateMachine):
    """
    Máquina de estados especializada para o atacante.
    Focada em buscar a bola, se posicionar para chutar e marcar gols.
    """

    def __init__(self, robot_id: int, team_color: str, game: Any):
        """Inicializa a FSM do atacante."""
        super().__init__(robot_id, team_color, game)
        self.role = RobotRole.ATTACKER
        self.logger = get_logger(f"robot_behavior.attacker.{self.robot_id}")
        self._update_home_position()
        self.target_position = self.home_position
        self.has_ball = False
        self.logger.info(
            f"ATT {self.robot_id} ({self.team_color}) Initialized. Home: {self.home_position}"
        )

    def _update_home_position(self):
        """Calcula a posição inicial (home) do atacante, ligeiramente no campo adversário."""
        super()._update_geometry_references()
        home_x = (
            (self.field_bounds["x_max"] - self.field_bounds["x_min"])
            * 0.05
            * self.attack_direction_x_sign
        )
        self.home_position = (
            home_x,
            0.0 + (self.robot_id * 0.1),
        )

    def _decide_next_action(
        self,
        vision_data: Dict,
        referee_data: Dict,
        current_pos: Tuple[float, float],
        game_command: str,
    ):
        """Lógica de decisão principal para o atacante."""
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
                )
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
            ):
                designated_target = referee_data.get("designated_position")
                if ball_pos and designated_target:
                    dist_to_ball = self._distance(current_pos, ball_pos)
                    if (
                        self.has_ball
                        or dist_to_ball < ATTACKER_BALL_OWNERSHIP_THRESHOLD
                    ):
                        self.has_ball = True
                        self.target_position = designated_target
                        self.logger.debug(
                            f"ATT {self.robot_id} (Placer): Has ball, moving to {designated_target}"
                        )
                        if (
                            self._distance(ball_pos, designated_target)
                            < BALL_PLACEMENT_SUCCESS_RADIUS
                        ):
                            self.logger.info(
                                f"ATT {self.robot_id} (Placer): Ball PLACED at {designated_target}!"
                            )
                            self.current_state = RobotState.IDLE
                    else:
                        self.has_ball = False
                        self.target_position = ball_pos
                else:
                    self.target_position = ball_pos if ball_pos else self.home_position
            else:
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return

        if self.current_state == RobotState.TAKING_SET_PIECE:
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
                actual_ball_pos_for_kick = (0, 0)
                if base_ref_cmd == "PREPARE_PENALTY":
                    actual_ball_pos_for_kick = self._get_penalty_spot(
                        for_us_to_attack=True
                    )
                elif ball_pos:
                    actual_ball_pos_for_kick = ball_pos

                if (
                    self._distance(current_pos, actual_ball_pos_for_kick)
                    < ATTACKER_BALL_OWNERSHIP_THRESHOLD + 0.02
                ):
                    self.logger.info(
                        f"ATT {self.robot_id}: KICKING for {base_ref_cmd}!"
                    )
                    self.game.active_controller.kick_flat(
                        self.robot_id, MAX_KICK_SPEED_GRSIM * 0.7
                    )
                    self.current_state = RobotState.SUPPORTING_OFFENSE
                    self.is_designated_kicker = False
                else:
                    self.target_position = self._calculate_ball_approach_for_kick(
                        actual_ball_pos_for_kick, opp_goal_center
                    )
            else:
                self.current_state = RobotState.RETURNING
                self.target_position = self.home_position
            self._process_movement(current_pos)
            return

        if not ball_pos:
            self.current_state = RobotState.RETURNING
            self.target_position = self.home_position
            self.has_ball = False
        else:
            dist_to_ball = self._distance(current_pos, ball_pos)
            self.has_ball = dist_to_ball < ATTACKER_BALL_OWNERSHIP_THRESHOLD

            if self.has_ball:
                self.current_state = RobotState.ATTACKING
                if self._is_point_in_defense_area(current_pos, "opponent"):
                    self.logger.debug(
                        f"ATT {self.robot_id}: Has ball IN OPPONENT DEFENSE AREA."
                    )
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
                    else:
                        target_x_out = (
                            -self.own_goal_line_x
                            - (DEFENSE_AREA_DEPTH_DEFAULT + 0.1)
                            * self.attack_direction_x_sign
                        )
                        self.target_position = (target_x_out, current_pos[1])
                else:
                    current_orientation = self._get_current_orientation()
                    dist_to_opp_goal = self._distance(current_pos, opp_goal_center)
                    if current_orientation and dist_to_opp_goal < (
                        (self.field_bounds["x_max"] - self.field_bounds["x_min"]) / 2.0
                        + 0.5
                    ):
                        angle_to_goal = math.atan2(
                            opp_goal_center[1] - current_pos[1],
                            opp_goal_center[0] - current_pos[0],
                        )
                        if abs(
                            self._normalize_angle(angle_to_goal - current_orientation)
                        ) < math.radians(15):
                            self.logger.info(f"ATT {self.robot_id}: KICKING at goal!")
                            self.game.active_controller.kick_flat(
                                self.robot_id, MAX_KICK_SPEED_GRSIM
                            )
                            self.current_state = RobotState.IDLE
                            self.target_position = current_pos
                        else:
                            self.target_position = opp_goal_center
                    else:
                        self.target_position = opp_goal_center
            else:
                is_main_attacker = (
                    self.role == RobotRole.ATTACKER and self.robot_id == 2
                )
                if is_main_attacker or self._is_closest_teammate_to_ball(
                    current_pos, ball_pos, vision_data
                ):
                    self.current_state = RobotState.MOVING_TO_BALL
                    self.target_position = self._calculate_ball_approach_for_kick(
                        ball_pos, opp_goal_center
                    )
                else:
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
        """Verifica se este robô é o companheiro de equipe mais próximo da bola."""
        my_dist = self._distance(my_pos, ball_pos)
        team_key = "robotsBlue" if self.team_color == "blue" else "robotsYellow"
        for r_id_str, r_data in vision_data.get(team_key, {}).items():
            r_id = int(r_id_str)
            if r_id == self.robot_id or r_data.get("x") is None:
                continue
            if self._distance((r_data["x"], r_data["y"]), ball_pos) < my_dist - 0.1:
                return False
        return True

    def _calculate_offensive_support_position(
        self,
        ball_pos: Tuple[float, float],
        vision_data: Dict,
        current_pos: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Calcula uma posição de apoio ofensivo, se posicionando para receber um passe ou um rebote."""
        opp_goal_x, opp_goal_y = (-self.own_goal_line_x, 0.0)
        dir_ball_goal_x, dir_ball_goal_y = (
            opp_goal_x - ball_pos[0],
            opp_goal_y - ball_pos[1],
        )
        norm_dir = self._distance((0, 0), (dir_ball_goal_x, dir_ball_goal_y))
        if norm_dir == 0:
            return self.home_position

        dist_ahead = 0.8 + (self.robot_id * 0.15)
        point_ahead_x = ball_pos[0] + (dir_ball_goal_x / norm_dir) * dist_ahead
        point_ahead_y = ball_pos[1] + (dir_ball_goal_y / norm_dir) * dist_ahead

        side_offset = 0.6
        side_sign = 1 if (self.robot_id % 2 == 0 and self.robot_id != 2) else -1
        if self.robot_id == 2:
            side_sign = 0

        perp_x, perp_y = -dir_ball_goal_y / norm_dir, dir_ball_goal_x / norm_dir
        final_x = point_ahead_x + perp_x * side_offset * side_sign
        final_y = point_ahead_y + perp_y * side_offset * side_sign
        return self._enforce_field_boundaries((final_x, final_y))

    def _calculate_ball_approach_for_kick(
        self, ball_pos: Tuple[float, float], kick_target_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calcula a posição ideal para se aproximar da bola, alinhado com o alvo do chute."""
        offset_dist = ATTACKER_BALL_OWNERSHIP_THRESHOLD * 0.8
        dx, dy = kick_target_pos[0] - ball_pos[0], kick_target_pos[1] - ball_pos[1]
        norm = self._distance((0, 0), (dx, dy))
        if norm == 0:
            current_orientation = self._get_current_orientation()
            if current_orientation is not None:
                approach_x = ball_pos[0] - offset_dist * math.cos(
                    current_orientation + math.pi
                )
                approach_y = ball_pos[1] - offset_dist * math.sin(
                    current_orientation + math.pi
                )
                return (approach_x, approach_y)
            else:
                return (
                    ball_pos[0] - offset_dist * self.attack_direction_x_sign,
                    ball_pos[1],
                )

        return (
            ball_pos[0] - (dx / norm * offset_dist),
            ball_pos[1] - (dy / norm * offset_dist),
        )
