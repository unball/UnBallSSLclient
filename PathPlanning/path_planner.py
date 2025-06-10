import threading
import queue
import time
import numpy as np
from typing import Dict, List, Tuple, Set, Optional
import math

from utils.logger import get_logger
from .astar import AStar


class PathPlanner(threading.Thread):
    """
    Planejador de trajetórias baseado em A* com execução em thread.

    Responsável por calcular caminhos para os robôs evitando obstáculos
    como outros robôs e limites do campo. Executa em uma thread separada
    para não bloquear o loop principal do jogo.

    Attributes:
        game: Instância do jogo com acesso aos dados de visão
        running: Flag para controle da thread
        planning_queue: Fila de requisições de planejamento
        paths: Dicionário com caminhos calculados por robô
        resolution: Resolução da grade em metros
        inflation_radius: Raio de inflação dos obstáculos
        logger: Logger para registrar eventos do planejador
        debug: Flag que indica se logs detalhados estão ativos
    """

    def __init__(self, game):
        """
        Inicializa o planejador de trajetórias.

        Args:
            game: Instância do jogo com acesso aos dados de visão
        """
        super().__init__()
        self.daemon = True  # Thread encerra quando o programa principal encerra
        self.game = game

        # Thread control
        self.running = False
        self.planning_lock = threading.Lock()
        self.paths_lock = threading.Lock()

        # Planning queue and results storage
        self.planning_queue = queue.Queue()
        self.paths = {}  # robot_id -> path

        # Path planning parameters
        self.resolution = 0.05  # Grid resolution in meters
        self.inflation_radius = 0.15  # Robot radius + safety margin

        # Configurar logger
        self.logger = get_logger("path_planning")
        self.debug = game.debug.get("path_planning", False)

        self.logger.info(
            "PathPlanner inicializado com resolução %.3fm e raio de inflação %.3fm",
            self.resolution,
            self.inflation_radius,
        )

    def run(self):
        """
        Thread principal que processa requisições de planejamento de trajetória.

        Monitora continuamente a fila de requisições e executa o algoritmo A*
        para calcular caminhos. Os resultados são armazenados no dicionário paths.
        """
        self.running = True
        self.logger.info("Thread do PathPlanner iniciada")

        while self.running:
            try:
                # Get next planning request from queue (non-blocking)
                try:
                    robot_id, start, goal = self.planning_queue.get(block=False)

                    # Plan path for this robot
                    if self.debug:
                        self.logger.debug(
                            f"Planejando trajetória para robô {robot_id} de {start} para {goal}"
                        )

                    path = self._plan_path(robot_id, start, goal)

                    # Store result
                    with self.paths_lock:
                        self.paths[robot_id] = path

                    # Mark task as done
                    self.planning_queue.task_done()

                except queue.Empty:
                    # No planning requests, sleep briefly
                    time.sleep(0.005)  # 5ms

            except Exception as e:
                self.logger.error(f"Erro no thread de planejamento: {e}", exc_info=True)
                time.sleep(0.1)  # Recover from errors

        self.logger.info("Thread do PathPlanner encerrada")

    def stop(self):
        """
        Para a thread do planejador de forma segura.

        Limpa a fila de requisições e aguarda a thread terminar.
        """
        self.logger.info("Parando o PathPlanner...")
        self.running = False

        # Clear queue
        while not self.planning_queue.empty():
            try:
                self.planning_queue.get_nowait()
                self.planning_queue.task_done()
            except queue.Empty:
                break

        # Wait for thread to finish
        if self.is_alive():
            self.join(timeout=1.0)

        self.logger.info("PathPlanner parado")

    def request_path(
        self, robot_id: int, start: Tuple[float, float], goal: Tuple[float, float]
    ):
        """
        Solicita o planejamento de um caminho para um robô.

        Args:
            robot_id: ID do robô
            start: Posição inicial (x, y)
            goal: Posição objetivo (x, y)
        """
        # Validate input
        if not self._is_valid_position(start) or not self._is_valid_position(goal):
            if self.debug:
                self.logger.warning(
                    f"Posição inicial ou objetivo inválida: {start} -> {goal}"
                )
            return

        # Add to planning queue
        self.planning_queue.put((robot_id, start, goal))
        if self.debug:
            self.logger.debug(
                f"Requisição de trajetória para robô {robot_id} adicionada à fila"
            )

    def get_path(self, robot_id: int) -> List[Tuple[float, float]]:
        """
        Obtém o caminho atualmente planejado para um robô.

        Args:
            robot_id: ID do robô

        Returns:
            Lista de pontos (x, y) formando o caminho ou lista vazia se não houver caminho
        """
        with self.paths_lock:
            return self.paths.get(robot_id, [])

    def _is_valid_position(self, pos: Tuple[float, float]) -> bool:
        """
        Verifica se uma posição é válida (dentro dos limites do campo e não None).

        Args:
            pos: Posição a verificar (x, y)

        Returns:
            True se a posição for válida
        """
        if pos is None or len(pos) != 2:
            return False

        x, y = pos
        if x is None or y is None or math.isnan(x) or math.isnan(y):
            return False

        # Check if within field bounds with some margin
        bounds = self.game.field_bounds
        margin = 0.1  # 10cm margin

        return (
            bounds["x_min"] - margin <= x <= bounds["x_max"] + margin
            and bounds["y_min"] - margin <= y <= bounds["y_max"] + margin
        )

    def _plan_path(
        self, robot_id: int, start: Tuple[float, float], goal: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """
        Planeja um caminho usando o algoritmo A*.

        Args:
            robot_id: ID do robô
            start: Posição inicial (x, y)
            goal: Posição objetivo (x, y)

        Returns:
            Lista de pontos (x, y) formando o caminho
        """
        # Start timing
        start_time = time.time()

        # Get vision data to extract obstacles
        vision_data = self.game.get_vision_data()
        if not vision_data:
            # No vision data, return direct path
            self.logger.warning("Sem dados de visão, retornando caminho direto")
            return [start, goal]

        # Extract obstacles from other robots
        obstacles = self._get_obstacles(vision_data, robot_id)

        # If no obstacles or close enough, return direct path
        if not obstacles or self._euclidean_distance(start, goal) < 0.3:
            if self.debug:
                self.logger.debug(
                    f"Caminho direto para robô {robot_id}: {len(obstacles)} obstáculos, "
                    f"distância={self._euclidean_distance(start, goal):.2f}m"
                )
            return [start, goal]

        # Run A* algorithm
        astar = AStar(resolution=self.resolution)
        astar.debug = self.debug  # Compartilhar a configuração de debug

        path = astar.find_path(start, goal, self.game.field_bounds, obstacles)

        # If path couldn't be found, return direct path
        if not path:
            self.logger.warning(
                f"Nenhum caminho encontrado para robô {robot_id}, usando caminho direto"
            )
            return [start, goal]

        # Post-process path to smooth it
        smoothed_path = self._smooth_path(path)

        # Calculate and log planning time
        planning_time = time.time() - start_time
        if self.debug:
            self.logger.debug(
                f"Planejamento para robô {robot_id} levou {planning_time*1000:.2f}ms: "
                f"{len(path)} pontos → {len(smoothed_path)} após suavização"
            )

        return smoothed_path

    def _get_obstacles(
        self, vision_data: Dict, robot_id: int
    ) -> Set[Tuple[float, float]]:
        """
        Extrai obstáculos dos dados de visão.

        Considera todos os robôs, exceto o que está sendo planejado,
        como obstáculos para o planejamento.

        Args:
            vision_data: Dados de visão do SSL-Vision
            robot_id: ID do robô para o qual estamos planejando

        Returns:
            Conjunto de posições (x, y) dos obstáculos
        """
        obstacles = set()

        # Extract all robots except the one we're planning for
        for team in ["robotsBlue", "robotsYellow"]:
            for rid, robot in vision_data.get(team, {}).items():
                # Skip if robot is not detected or is the planning robot
                if robot["x"] is None or int(rid) == robot_id:
                    continue

                # Add robot position to obstacles
                obstacles.add((robot["x"], robot["y"]))

        # Inflate obstacles
        inflated_obstacles = self._inflate_obstacles(obstacles)

        if self.debug:
            self.logger.debug(
                f"Extraídos {len(obstacles)} obstáculos, inflados para {len(inflated_obstacles)}"
            )

        return inflated_obstacles

    def _inflate_obstacles(
        self, obstacles: Set[Tuple[float, float]]
    ) -> Set[Tuple[float, float]]:
        """
        Infla obstáculos pelo raio do robô mais margem de segurança.

        Cada obstáculo é expandido para um conjunto de pontos que cobrem
        a área de colisão potencial.

        Args:
            obstacles: Posições originais dos obstáculos

        Returns:
            Conjunto de posições de obstáculos inflados
        """
        if not obstacles:
            return set()

        # Convert to numpy array for efficient operations
        obstacle_array = np.array(list(obstacles))

        # Create a grid of points around each obstacle
        steps = int(self.inflation_radius / self.resolution)
        inflated = set()

        for x, y in obstacle_array:
            for dx in range(-steps, steps + 1):
                for dy in range(-steps, steps + 1):
                    # Check if point is within inflation radius
                    px = x + dx * self.resolution
                    py = y + dy * self.resolution
                    distance = math.sqrt(dx**2 + dy**2) * self.resolution

                    if distance <= self.inflation_radius:
                        # Add to inflated obstacles set
                        inflated.add((round(px, 3), round(py, 3)))

        return inflated

    def _smooth_path(
        self, path: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """
        Suaviza o caminho removendo pontos desnecessários.

        Simplifica o caminho para torná-lo mais natural e eficiente,
        removendo pontos que não contribuem significativamente para a forma.

        Args:
            path: Caminho original

        Returns:
            Caminho suavizado
        """
        if len(path) <= 2:
            return path

        # Simple smoothing: keep only essential waypoints
        smoothed = [path[0]]

        for i in range(1, len(path) - 1):
            prev_vec = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            next_vec = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])

            # Calculate angle between vectors
            angle = self._angle_between(prev_vec, next_vec)

            # Keep waypoint if angle is significant
            if abs(angle) > 0.3:  # ~17 degrees
                smoothed.append(path[i])

        smoothed.append(path[-1])

        if self.debug and len(smoothed) < len(path):
            self.logger.debug(
                f"Caminho suavizado: reduzido de {len(path)} para {len(smoothed)} pontos"
            )

        return smoothed

    def _euclidean_distance(
        self, p1: Tuple[float, float], p2: Tuple[float, float]
    ) -> float:
        """
        Calcula a distância euclidiana entre dois pontos.

        Args:
            p1: Primeiro ponto
            p2: Segundo ponto

        Returns:
            Distância euclidiana
        """
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _angle_between(self, v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
        """
        Calcula o ângulo entre dois vetores.

        Args:
            v1: Primeiro vetor
            v2: Segundo vetor

        Returns:
            Ângulo em radianos
        """
        # Calculate dot product
        dot = v1[0] * v2[0] + v1[1] * v2[1]

        # Calculate magnitudes
        mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

        # Calculate angle
        if mag1 * mag2 == 0:  # Avoid division by zero
            return 0

        cos_angle = dot / (mag1 * mag2)

        # Clamp value to valid range
        cos_angle = max(-1.0, min(1.0, cos_angle))

        return math.acos(cos_angle)
