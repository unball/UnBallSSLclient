# main.py
"""
UnBall SSL Client - Sistema Principal de Controle de Robôs
=========================================================

Este é o arquivo principal do sistema UnBall para RoboCup Small Size League (SSL).
O sistema controla múltiplos robôs autônomos em tempo real, integrando:
- Visão computacional (SSL-Vision)
- Controle de robôs (grSim/IRL)
- Planejamento de trajetórias (A*)
- Máquinas de estado para comportamentos dos robôs
- Interface gráfica (PyQt5)
"""

import sys
import warnings
import os
import json
import threading
import time
import math
import signal
import traceback
from typing import Optional, Dict, Any
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt, QTimer

# Importar sistema de logging personalizado
from utils import logger as logging_utils

warnings.filterwarnings("ignore", category=DeprecationWarning)
os.environ["GTK_MODULES"] = ""

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Importações dos módulos do projeto
from PyQt.ssl_client import SSLClientWindow  # Interface gráfica principal
from GameController.GameController import GameController  # Comunicação com árbitro SSL
from VisionClient.Vision import Vision  # Cliente de visão SSL-Vision
from SimulationGrSim.RobotControlClient_threaded import (
    ThreadedRobotControlClient,
)  # Cliente de controle threaded
from RobotBehavior.grsim_client import (
    GrSimController,
)  # Controlador para simulador grSim
from RobotBehavior.irl_client import IRLController  # Controlador para robôs reais
from PathPlanning.path_planner import PathPlanner  # Planejador de trajetórias A*
from RobotBehavior.robot_state_machine import (  # Máquinas de estado dos robôs
    GoalkeeperStateMachine,
    DefenderStateMachine,
    AttackerStateMachine,
)
from RobotBehavior.robot_states import RobotRole  # Enumeração dos papéis dos robôs

# Configurar logger principal do sistema
logger = logging_utils.get_logger("main")

# Flag global de debug
GAME_DEBUG = False

# Forçar debug do sistema de comportamento dos robôs
import RobotBehavior.robot_state_machine as rsm

rsm.DEBUG_ROBOT_BEHAVIOR = True  # Ativar para desenvolvimento

# Dimensões de campo por divisão (local centralizado para essas informações)
FIELD_DIMENSIONS_BY_DIVISION = {
    "Division A": {
        "width": 12.0,  # Largura do campo em metros
        "height": 9.0,  # Altura do campo em metros
        "max_robots": 11,  # Número máximo de robôs por time
        "roles": {  # Papéis padrão dos robôs por ID
            0: RobotRole.GOALKEEPER,
            1: RobotRole.DEFENDER,
            2: RobotRole.DEFENDER,
            3: RobotRole.ATTACKER,
            4: RobotRole.ATTACKER,
        },
    },
    "Division B": {
        "width": 9.0,
        "height": 6.0,
        "max_robots": 6,
        "roles": {
            0: RobotRole.GOALKEEPER,
            1: RobotRole.DEFENDER,
            2: RobotRole.ATTACKER,
            3: RobotRole.ATTACKER,
        },
    },
    "Entry Level": {
        "width": 4.5,
        "height": 3.0,
        "max_robots": 3,
        "roles": {
            0: RobotRole.GOALKEEPER,
            1: RobotRole.DEFENDER,
            2: RobotRole.ATTACKER,
        },
    },
}


class Game:
    """
    Classe principal do jogo que coordena todos os subsistemas.

    Esta classe é o núcleo do sistema UnBall, responsável por:
    - Gerenciar a configuração do sistema
    - Coordenar threads de visão, árbitro e controle
    - Manter o loop principal do jogo (60 FPS)
    - Gerenciar máquinas de estado dos robôs
    - Controlar comunicação com simulador/robôs reais
    """

    def __init__(self) -> None:
        """
        Inicializa o sistema de jogo completo.

        Carrega configurações, inicializa subsistemas e prepara o estado do jogo.
        Todo o sistema é configurado mas não iniciado até chamar start().
        """
        # Carregar configuração do arquivo config.json
        self.config = get_config()
        if GAME_DEBUG:
            logger.info(f"Game: Config loaded: {json.dumps(self.config, indent=2)}")

        # Configurar loggers baseado nas configurações carregadas
        if self.config.get("debug_flags", {}).get("robot_behavior", True):
            logging_utils.setup_logger("robot_behavior", level="DEBUG")
        if self.config.get("debug_flags", {}).get("path_planning", True):
            logging_utils.setup_logger("path_planning", level="DEBUG")
        if self.config.get("debug_flags", {}).get("all", False):
            logging_utils.setup_logger("main", level="DEBUG")

        # Flags de debug extraídas da configuração
        self.debug = self.config.get(
            "debug_flags",
            {
                "vision": False,
                "referee": False,
                "threads": False,
                "timing": False,
                "path_planning": True,
                "all": False,
            },
        )
        rsm.DEBUG_ROBOT_BEHAVIOR = self.debug.get("robot_behavior", True)

        # Estado do jogo e controle de comandos
        self.current_command: Optional[str] = "HALT"  # Estado inicial do jogo
        self.team_color_for_current_command: Optional[str] = (
            None  # Time associado ao comando
        )
        self.command_thread = None  # Thread para processamento de comandos

        if GAME_DEBUG:
            logger.info("Game: Initializing...")

        # Inicializar clientes de controle de robôs usando portas da configuração
        self.blue_robots = ThreadedRobotControlClient(
            team_port=self.config["network"].get("blue_control_port", 10301)
        )
        self.yellow_robots = ThreadedRobotControlClient(
            team_port=self.config["network"].get("yellow_control_port", 10302)
        )
        if GAME_DEBUG:
            logger.info("Game: Robot control clients initialized")

        # Camada de entrada - Inicializar ANTES de usar
        self.vision = Vision(self)  # Sistema de visão SSL-Vision
        self.referee = GameController(self)  # Comunicação com árbitro SSL
        if GAME_DEBUG:
            logger.info("Game: Vision and referee systems initialized")

        # Limites do campo - Inicializar APÓS visão ser criada, com valores padrão primeiro
        self.field_bounds = {"x_min": -2.25, "x_max": 2.25, "y_min": -1.5, "y_max": 1.5}
        # Definir limites adequados baseados na divisão
        self.update_field_bounds_for_division(
            self.config.get("match", {}).get("division", "Entry Level")
        )

        # Inicializar controladores com portas da configuração
        self.controllers = {
            "grSim": {  # Controladores para simulador grSim
                "blue": GrSimController(
                    team_port=self.config["network"].get("blue_control_port", 10301)
                ),
                "yellow": GrSimController(
                    team_port=self.config["network"].get("yellow_control_port", 10302)
                ),
            },
            "IRL": {  # Controladores para robôs reais
                "blue": IRLController(),
                "yellow": IRLController(),
            },
        }

        # Definir controlador ativo baseado na configuração
        initial_team_color = self.config.get("match", {}).get("team_color", "blue")
        initial_control_mode = self.config.get("match", {}).get("control_mode", "grSim")
        self.active_controller = self.controllers[initial_control_mode][
            initial_team_color
        ]

        if GAME_DEBUG:
            logger.info(
                f"Game: Active controller set to {initial_control_mode} for {initial_team_color}"
            )

        # Planejador de trajetórias A*
        self.path_planner = PathPlanner(self)

        # Rastreamento de estado do sistema
        self.running = False  # Flag de execução do sistema
        self._update_thread: Optional[threading.Thread] = (
            None  # Thread do loop principal
        )
        self._fps = self.config.get("match", {}).get("fps", 60)  # Taxa de atualização
        self.last_vision_data: Optional[Dict] = None  # Últimos dados de visão recebidos
        self.last_referee_data: Optional[Dict] = (
            None  # Últimos dados do árbitro recebidos
        )

        # Máquinas de estado dos robôs (inicializadas posteriormente)
        self.robot_state_machines: Dict[int, Any] = {}

        # Rastreamento da interface gráfica
        self.window: Optional[SSLClientWindow] = None  # Janela principal da UI
        self.visualization_timer = None  # Timer para atualização visual

    def start(self):
        """
        Inicia todos os componentes do sistema.

        Ordem de inicialização:
        1. Clientes de controle de robôs
        2. Sistemas de visão e árbitro
        3. Aguarda geometria do campo da visão
        4. Planejador de trajetórias
        5. Controlador ativo
        6. Máquinas de estado dos robôs
        7. Loop principal do jogo
        """
        if self.running:
            if GAME_DEBUG:
                logger.info("Game: Start called, but already running.")
            return

        if GAME_DEBUG:
            logger.info("Game: Starting all systems...")

        # Iniciar controle de robôs (threads de comunicação)
        self.blue_robots.start()
        self.yellow_robots.start()
        if GAME_DEBUG:
            logger.info("Game: Robot control clients started")

        # Iniciar visão e árbitro (threads de recepção de dados)
        self.vision.start()
        self.referee.start()
        if GAME_DEBUG:
            logger.info("Game: Vision and referee systems started")

        # Aguardar geometria da visão para definir limites corretos do campo
        max_wait_geom = 5.0  # segundos
        start_wait_geom = time.time()
        while not self.vision.any_geometry and (
            time.time() - start_wait_geom < max_wait_geom
        ):
            time.sleep(0.1)
        if self.vision.any_geometry:
            self.update_field_bounds_for_division(
                self.config.get("match", {}).get("division", "Entry Level")
            )
            if GAME_DEBUG:
                logger.info(f"Game: Field bounds set from vision: {self.field_bounds}")

        # Iniciar planejador de trajetórias
        self.path_planner.start()
        if GAME_DEBUG:
            logger.info("Game: Path planner started")

        # Iniciar controlador ativo
        self.active_controller.start()
        if GAME_DEBUG:
            logger.info("Game: Active controller started")

        # Inicializar máquinas de estado dos robôs
        self._initialize_robot_state_machines()

        # Iniciar loop principal do jogo
        self.running = True
        self._update_thread = threading.Thread(
            target=self.update_loop, name="GameUpdateLoop"
        )
        self._update_thread.daemon = True
        self._update_thread.start()
        if GAME_DEBUG:
            logger.info("Game: Main update loop started")

    def test_simple_movement(self):
        """
        Testa se os robôs conseguem receber comandos básicos de movimento.

        Método útil para debug e verificação de conectividade.
        Move o robô 0 para frente por 2 segundos e depois para.
        """
        logger.info("Testing simple robot movement...")
        if self.active_controller:
            logger.info(f"Active controller: {self.active_controller}")
            logger.info(f"Controller mode: {self.active_controller.mode}")

            # Testar robô 0
            logger.info("Sending test command to robot 0: move forward")
            result = self.active_controller.send_global_velocity(0, 0.5, 0.0, 0.0)
            logger.info(f"Command result: {result}")

            # Aguardar e parar
            time.sleep(2)
            logger.info("Stopping robot 0")
            self.active_controller.send_global_velocity(0, 0.0, 0.0, 0.0)
        else:
            logger.error("ERROR: No active controller!")

    def update_field_bounds_for_division(self, division_name: str):
        """
        Atualiza os limites do campo baseado na divisão e dados de visão.

        Prioriza dados de geometria da visão SSL-Vision quando disponíveis,
        caso contrário usa valores padrão das configurações de divisão.

        Args:
            division_name: Nome da divisão ("Entry Level", "Division B", "Division A")
        """
        # Tentar obter da visão primeiro (apenas se visão existe e tem geometria)
        if (
            hasattr(self, "vision")
            and self.vision
            and hasattr(self.vision, "any_geometry")
            and self.vision.any_geometry
            and hasattr(self.vision, "raw_geometry")
            and self.vision.raw_geometry.get("fieldLength", 0) > 0
        ):
            field_length = self.vision.raw_geometry["fieldLength"]  # Já em metros
            field_width = self.vision.raw_geometry["fieldWidth"]  # Já em metros
            if GAME_DEBUG:
                logger.info(
                    f"Game: Using vision geometry: L={field_length}, W={field_width}"
                )
        else:
            # Fallback para FIELD_DIMENSIONS_BY_DIVISION
            dims = FIELD_DIMENSIONS_BY_DIVISION.get(
                division_name, FIELD_DIMENSIONS_BY_DIVISION["Entry Level"]
            )
            field_length = dims["width"]
            field_width = dims["height"]
            if GAME_DEBUG:
                logger.info(
                    f"Game: Using default dimensions for {division_name}: L={field_length}, W={field_width}"
                )

        # Definir limites do campo (campo centralizado em 0,0)
        self.field_bounds = {
            "x_min": -field_length / 2.0,
            "x_max": field_length / 2.0,
            "y_min": -field_width / 2.0,
            "y_max": field_width / 2.0,
        }
        if GAME_DEBUG:
            logger.info(f"Game: Field bounds updated to: {self.field_bounds}")

    def update_field_bounds(self):
        """
        Atualiza limites do campo a partir da geometria de visão (método legado).

        Método mantido para compatibilidade com código existente.
        Use update_field_bounds_for_division() para novas implementações.
        """
        if self.vision and self.vision.raw_geometry:
            field_length = self.vision.raw_geometry["fieldLength"]
            field_width = self.vision.raw_geometry["fieldWidth"]

            # Dimensões do campo são centralizadas em 0,0
            self.field_bounds = {
                "x_min": -field_length / 2,
                "x_max": field_length / 2,
                "y_min": -field_width / 2,
                "y_max": field_width / 2,
            }

    def check_bounds(self, x, y):
        """
        Verifica se uma posição está dentro dos limites do campo.

        Args:
            x: Coordenada X da posição
            y: Coordenada Y da posição

        Returns:
            bool: True se a posição está dentro dos limites
        """
        return (
            self.field_bounds["x_min"] <= x <= self.field_bounds["x_max"]
            and self.field_bounds["y_min"] <= y <= self.field_bounds["y_max"]
        )

    def get_team_for_command(self, command_str: Optional[str]) -> str:
        """
        Extrai a cor do time de uma string de comando do árbitro.

        Comandos específicos de time terminam com "_BLUE" ou "_YELLOW".

        Args:
            command_str: String do comando do árbitro

        Returns:
            str: "blue", "yellow" ou string vazia se não específico
        """
        if not command_str:
            return ""
        if command_str.endswith("_BLUE"):
            return "blue"
        if command_str.endswith("_YELLOW"):
            return "yellow"
        return ""

    def set_game_state(self, state):
        """
        Processa comandos de estado do jogo e atualiza comportamento dos robôs.

        Principais comandos:
        - HALT: Para todos os robôs imediatamente
        - STOP: Robôs devem ficar longe da bola
        - FORCE_START/NORMAL_START: Jogo ativo, robôs podem se mover livremente
        - Comandos específicos de time: FREE_KICK_BLUE, PENALTY_YELLOW, etc.

        Args:
            state: String do comando de estado (ex: "HALT", "KICK_OFF_BLUE")
        """
        if GAME_DEBUG:
            logger.info(f"Game: set_game_state received: '{state}'")

        self.current_command = state
        self.team_color_for_current_command = self.get_team_for_command(state)

        # Analisar comandos específicos de time
        team_commands = [
            "FREE_KICK",
            "KICK_OFF",
            "PENALTY",
            "GOAL_KICK",
            "CORNER_KICK",
            "BALL_PLACEMENT",
        ]

        team_color = None
        base_command = state

        # Verificar se este é um comando específico de time
        for cmd in team_commands:
            if state.startswith(cmd + "_"):
                parts = state.split("_", 1)
                if len(parts) > 1:
                    base_command = parts[0]
                    team_color = parts[1].lower()
                    if GAME_DEBUG:
                        logger.info(
                            f"Game: Extracted base command: {base_command}, team: {team_color}"
                        )
                    break

        # Atualizar dados do árbitro com este comando
        if hasattr(self, "last_referee_data") and self.last_referee_data:
            self.last_referee_data["command"] = base_command
            self.last_referee_data["team_color_for_command"] = team_color
            self.last_referee_data["can_play"] = base_command in [
                "FORCE_START",
                "NORMAL_START",
            ]

        # Tomar ação imediata baseada no comando
        if state == "HALT":
            if GAME_DEBUG:
                logger.info("Game: HALT command - stopping all robots")
            num_robots = FIELD_DIMENSIONS_BY_DIVISION[self.config["match"]["division"]][
                "max_robots"
            ]
            for i in range(num_robots):
                self.blue_robots.send_global_velocity(i, 0, 0, 0)
                self.yellow_robots.send_global_velocity(i, 0, 0, 0)

        elif state == "STOP":
            if GAME_DEBUG:
                logger.info("Game: STOP command - robots must stay away from ball")

        elif state.startswith("FORCE_START") or state.startswith("NORMAL_START"):
            if GAME_DEBUG:
                logger.info(f"Game: {state} - game starting, robots can move freely")

        # Forçar atualização imediata de todas as máquinas de estado dos robôs
        vision_data = self.get_vision_data()
        if vision_data and hasattr(self, "robot_state_machines"):
            if GAME_DEBUG:
                logger.debug("Game: Updating all robot state machines with new command")
            for robot_id, state_machine in self.robot_state_machines.items():
                if hasattr(state_machine, "update"):
                    state_machine.update(vision_data)

    def update_loop(self):
        """
        Loop principal da lógica do jogo.

        Executa a 60 FPS e processa:
        1. Dados de visão (posições de robôs e bola)
        2. Dados do árbitro (comandos e estado do jogo)
        3. Atualização das máquinas de estado dos robôs

        O loop continua até self.running ser False.
        """
        if GAME_DEBUG:
            logger.info("Game: Update loop starting")
        loop_counter = 0

        while self.running:
            try:
                loop_start_time = time.perf_counter()

                # 1. Obter dados de visão
                if self.vision.new_data or loop_counter == 0:
                    self.last_vision_data = self.vision.get_last_frame()
                    self.vision.new_data = False
                    if self.vision.any_geometry:
                        self.update_field_bounds_for_division(
                            self.config["match"]["division"]
                        )

                # 2. Obter dados do árbitro
                current_ref_command = self.referee.raw_referee.get("command")
                current_ref_stage = self.referee.raw_referee.get("stage")
                team_for_gc_command = self.get_team_for_command(current_ref_command)

                self.last_referee_data = {
                    "command": current_ref_command,
                    "stage": current_ref_stage,
                    "team_color_for_command": team_for_gc_command,
                    "can_play": not (current_ref_command in ["HALT", "STOP"]),
                    "designated_position": (
                        self.referee.raw_referee.get("position")
                        if self.referee.raw_referee.get("meta", {}).get(
                            "has_designated_position"
                        )
                        else None
                    ),
                    "blue_team_details": self.referee.raw_referee.get("blue"),
                    "yellow_team_details": self.referee.raw_referee.get("yellow"),
                }

                # Sobrescrever com comando UI/Teste se ativo
                if self.current_command is not None:
                    self.last_referee_data["command"] = self.current_command
                    self.last_referee_data["team_color_for_command"] = (
                        self.team_color_for_current_command
                    )
                    self.last_referee_data["can_play"] = not (
                        self.current_command in ["HALT", "STOP"]
                    )

                # 3. Atualizar máquinas de estado dos robôs
                if self.last_vision_data and self.robot_state_machines:
                    # Apenas atualizar se o jogo estiver ativo
                    if self.last_referee_data.get("can_play", False):
                        for (
                            robot_id,
                            state_machine,
                        ) in self.robot_state_machines.items():
                            state_machine.update(self.last_vision_data)

                # Controle de timing para manter FPS
                loop_processing_time = time.perf_counter() - loop_start_time
                sleep_time = (1.0 / self._fps) - loop_processing_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

                loop_counter += 1

            except Exception as e:
                logger.error(f"Game ERROR in update_loop: {e}")
                traceback.print_exc()
                time.sleep(0.1)

        if GAME_DEBUG:
            logger.info("Game: Update loop finished")

    def _initialize_robot_state_machines(self):
        """
        Inicializa máquinas de estado para diferentes papéis de robôs baseado na divisão.

        Cada divisão tem configurações específicas de:
        - Número de robôs por time
        - Atribuição de papéis (goleiro, defensor, atacante)

        As máquinas de estado controlam o comportamento autônomo de cada robô.
        """
        self.robot_state_machines.clear()
        team_color = self.config.get("match", {}).get("team_color", "blue")
        current_division_name = self.config.get("match", {}).get(
            "division", "Entry Level"
        )

        division_info = FIELD_DIMENSIONS_BY_DIVISION.get(
            current_division_name, FIELD_DIMENSIONS_BY_DIVISION["Entry Level"]
        )
        num_robots = division_info["max_robots"]
        role_assignments = division_info["roles"]

        if GAME_DEBUG:
            logger.info(
                f"Game: Initializing {num_robots} SMs for '{current_division_name}', team '{team_color}'"
            )
            logger.info(f"Game: Role assignments: {role_assignments}")

        # Criar máquina de estado específica para cada robô
        for robot_id_int in range(num_robots):
            role_to_assign = role_assignments.get(robot_id_int, RobotRole.ATTACKER)

            sm_instance = None
            if role_to_assign == RobotRole.GOALKEEPER:
                sm_instance = GoalkeeperStateMachine(robot_id_int, team_color, self)
            elif role_to_assign == RobotRole.DEFENDER:
                sm_instance = DefenderStateMachine(robot_id_int, team_color, self)
            elif role_to_assign == RobotRole.ATTACKER:
                sm_instance = AttackerStateMachine(robot_id_int, team_color, self)

            if sm_instance:
                self.robot_state_machines[robot_id_int] = sm_instance
                if GAME_DEBUG:
                    logger.info(
                        f"Game: Initialized Robot {robot_id_int} as {sm_instance.role.value}"
                    )

    def switch_team_color(self, new_team_color: str):
        """
        Alterna a cor do time e reinicializa os robôs.

        Processo:
        1. Para todos os robôs
        2. Atualiza configuração
        3. Alterna controlador ativo
        4. Reinicializa máquinas de estado

        Args:
            new_team_color: Nova cor do time ("blue" ou "yellow")
        """
        if new_team_color not in ["blue", "yellow"]:
            if GAME_DEBUG:
                logger.warning(f"Game: Invalid team color '{new_team_color}'")
            return

        old_color = self.config["match"]["team_color"]
        if old_color == new_team_color:
            if GAME_DEBUG:
                logger.info(f"Game: Team color already {new_team_color}")
            return

        if GAME_DEBUG:
            logger.info(
                f"Game: Switching team color from {old_color} to {new_team_color}"
            )

        # Parar robôs antes de alternar
        if self.active_controller:
            num_robots = FIELD_DIMENSIONS_BY_DIVISION[self.config["match"]["division"]][
                "max_robots"
            ]
            for i in range(num_robots):
                self.active_controller.send_global_velocity(i, 0, 0, 0)
            time.sleep(0.1)

        # Atualizar configuração
        self.config["match"]["team_color"] = new_team_color

        # Alternar controlador
        current_mode = self.config.get("match", {}).get("control_mode", "grSim")
        if self.active_controller:
            self.active_controller.stop()
        self.active_controller = self.controllers[current_mode][new_team_color]
        self.active_controller.start()

        # Reinicializar máquinas de estado dos robôs
        self._initialize_robot_state_machines()

        if GAME_DEBUG:
            logger.info(f"Game: Team switch complete to {new_team_color}")

    def set_control_mode(self, mode: str, team_color: str = None):
        """
        Alterna entre controle grSim e IRL (robôs reais).

        Args:
            mode: Modo de controle ("grSim" ou "IRL")
            team_color: Cor do time (opcional, usa cor atual se não especificado)
        """
        if mode not in self.controllers:
            if GAME_DEBUG:
                logger.warning(f"Game: Invalid control mode '{mode}'")
            return

        # Usar cor do time fornecida ou cor atual
        current_team_color = team_color or self.config["match"]["team_color"]

        if self.active_controller == self.controllers[mode][current_team_color]:
            if GAME_DEBUG:
                logger.info(f"Game: Already in {mode} for team {current_team_color}")
            return

        if GAME_DEBUG:
            logger.info(
                f"Game: Switching control mode to {mode} for team {current_team_color}"
            )

        # Parar controlador atual
        if self.active_controller:
            self.active_controller.stop()

        # Atualizar config e definir novo controlador
        self.config["match"]["control_mode"] = mode
        self.active_controller = self.controllers[mode][current_team_color]
        self.active_controller.start()

    def get_vision_data(self):
        """Retorna os últimos dados de visão recebidos."""
        return self.last_vision_data

    def get_referee_data(self):
        """Retorna os últimos dados do árbitro recebidos."""
        return self.last_referee_data

    def send_command(self, command):
        """Método legado para compatibilidade. Use `set_game_state`."""
        self.set_game_state(command)

    def stop(self):
        """
        Para todos os componentes do sistema de forma segura.

        Executa a sequência de desligamento para evitar threads órfãs ou
        estados inconsistentes. Para os robôs, threads de processamento
        e clientes de rede.
        """
        if not self.running:
            return

        if GAME_DEBUG:
            logger.info("Game: Stopping all systems...")
        self.running = False
        self.current_command = None

        # Parar planejador de trajetória
        if hasattr(self, "path_planner"):
            self.path_planner.stop()

        # Parar robôs
        if self.active_controller:
            num_robots = FIELD_DIMENSIONS_BY_DIVISION[self.config["match"]["division"]][
                "max_robots"
            ]
            for i in range(num_robots):
                self.active_controller.send_global_velocity(i, 0, 0, 0)

        # Parar thread de update
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=1.0)

        # Parar componentes (visão, árbitro)
        components = ["vision", "referee"]
        for comp in components:
            if hasattr(self, comp):
                try:
                    getattr(self, comp).stop()
                except Exception as e:
                    if GAME_DEBUG:
                        logger.error(f"Error stopping {comp}: {e}")

        # Parar todos os controladores (grSim, IRL)
        for mode_key in self.controllers:
            for color_key in self.controllers[mode_key]:
                try:
                    self.controllers[mode_key][color_key].stop()
                except Exception as e:
                    if GAME_DEBUG:
                        logger.error(
                            f"Error stopping controller {mode_key}/{color_key}: {e}"
                        )

        # Parar clientes de controle de robôs
        if hasattr(self, "blue_robots"):
            self.blue_robots.stop()
        if hasattr(self, "yellow_robots"):
            self.yellow_robots.stop()

        if GAME_DEBUG:
            logger.info("Game: All systems stopped")

    def update_network_settings(self, network_config: dict):
        """
        Atualiza as configurações de rede para visão e árbitro em tempo de execução.

        Para e reinicia os serviços de visão e árbitro com os novos parâmetros.

        Args:
            network_config: Dicionário com as novas configurações de rede.

        Returns:
            bool: True se a atualização foi bem-sucedida, False caso contrário.
        """
        try:
            if GAME_DEBUG:
                logger.info(f"Game: Updating network settings: {network_config}")

            # Parar threads existentes
            if self.vision:
                self.vision.stop()
            if self.referee:
                self.referee.stop()

            # Atualizar configuração
            self.config["network"].update(network_config)

            # Reinicializar com novas configurações
            self.vision = Vision(self)
            self.referee = GameController(self)

            # Reiniciar
            self.vision.start()
            self.referee.start()

            if GAME_DEBUG:
                logger.info("Game: Network settings updated successfully")
            return True

        except Exception as e:
            if GAME_DEBUG:
                logger.error(f"Game ERROR updating network settings: {e}")
            traceback.print_exc()
            return False

    def get_unball_data_vision(self) -> Optional[Dict]:
        """
        Retorna dados de visão formatados para a UI (método legado).

        Filtra os dados de visão brutos para o time atualmente controlado e
        os formata em uma estrutura esperada pela interface gráfica.

        Returns:
            Dicionário com dados de visão do time ou None se não houver dados.
        """
        if not self.last_vision_data:
            return None

        is_blue = self.config["match"]["team_color"] == "blue"
        robot_data = (
            self.last_vision_data["robotsBlue"]
            if is_blue
            else self.last_vision_data["robotsYellow"]
        )

        active_robots = {
            robot_id: {
                "position": {"x": data["x"], "y": data["y"]},
                "orientation": data["theta"],
                "timestamp": data["tCapture"],
                "camera_id": data["cCapture"],
            }
            for robot_id, data in robot_data.items()
            if data["x"] is not None
        }

        return {
            "team_color": "blue" if is_blue else "yellow",
            "robots": active_robots,
            "total_robots": len(active_robots),
            "ball_position": (
                {
                    "x": self.last_vision_data["ball"]["x"],
                    "y": self.last_vision_data["ball"]["y"],
                }
                if self.last_vision_data["ball"]
                else None
            ),
        }

    def get_unball_data_referee(self) -> Optional[Dict]:
        """
        Retorna dados do árbitro formatados para a UI (método legado).

        Filtra os dados do árbitro brutos para o time e os formata em uma
        estrutura esperada pela interface gráfica.

        Returns:
            Dicionário com dados do árbitro do time ou None se não houver dados.
        """
        if not self.last_referee_data:
            return None

        team_color = self.config["match"]["team_color"]
        team_data = self.last_referee_data.get(f"{team_color}_team_details", {})

        return {
            "side": team_color,
            "score": team_data.get("score", 0),
            "red_cards": team_data.get("red_cards", 0),
            "yellow_cards": team_data.get("yellow_cards", 0),
            "timeouts": team_data.get("timeouts", 0),
            "timeout_time": team_data.get("timeout_time", 0),
            "goalkeeper": team_data.get("goalkeeper", 0),
        }


def get_config(config_file: Optional[str] = None) -> dict:
    """
    Carrega o arquivo de configuração `config.json`.

    Se o arquivo não existir ou for inválido, um novo arquivo com as
    configurações padrão será criado. As configurações carregadas são
    mescladas com as padrões para garantir que todas as chaves existam.

    Args:
        config_file: Caminho opcional para o arquivo de configuração.

    Returns:
        Um dicionário com as configurações do sistema.
    """
    config_path = config_file or os.path.join(SCRIPT_DIR, "config.json")

    # Estrutura de configuração padrão
    default_cfg_data = {
        "network": {
            "multicast_ip": "224.5.23.2",
            "vision_port": 10020,
            "referee_ip": "224.5.23.1",
            "referee_port": 10003,
            "blue_control_port": 10301,
            "yellow_control_port": 10302,
            "simulation_control_port": 10300,
        },
        "match": {
            "team_1_name": "UnBall",
            "team_2_name": "Opponent",
            "event_name": "RoboCup SSL",
            "team_side": "left",
            "team_color": "blue",
            "division": "Entry Level",
            "control_mode": "grSim",
            "fps": 60,
            "time_logging": False,
        },
        "debug_flags": {
            "vision": False,
            "referee": False,
            "threads": False,
            "timing": False,
            "path_planning": True,
            "robot_behavior": True,
            "all": False,
        },
    }

    if os.path.exists(config_path):
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            # Mesclar com padrões para garantir que todas as chaves existam
            for section, section_defaults in default_cfg_data.items():
                if section not in config:
                    config[section] = section_defaults
                else:
                    for key, value in section_defaults.items():
                        config[section].setdefault(key, value)
            return config
        except json.JSONDecodeError:
            logger.warning(f"AVISO: JSON inválido em {config_path}. Usando padrões.")
        except Exception as e:
            logger.warning(
                f"AVISO: Erro ao carregar {config_path}: {e}. Usando padrões."
            )

    # Salvar configuração padrão se não existir
    try:
        with open(config_path, "w") as f:
            json.dump(default_cfg_data, f, indent=4)
        logger.info(f"Configuração padrão criada em {config_path}")
    except Exception as e:
        logger.warning(f"AVISO: Não foi possível salvar a configuração: {e}")

    return default_cfg_data


# Instância global do jogo para o manipulador de sinal
_game_instance_for_signal_handler: Optional[Game] = None


def signal_handler(sig, frame):
    """
    Manipula o sinal de interrupção (Ctrl+C) para um desligamento seguro.

    Invoca o método `stop()` da instância global do jogo para encerrar
    todas as threads e processos de forma controlada.
    """
    logger.info("\n! Sinal de interrupção recebido. Desligando sistemas...")
    rsm.DEBUG_ROBOT_BEHAVIOR = False

    if _game_instance_for_signal_handler is not None:
        _game_instance_for_signal_handler.stop()

    QApplication.quit()
    time.sleep(0.5)
    logger.info("Desligamento completo. Saindo.")
    sys.exit(0)


def main():
    """
    Ponto de entrada principal do sistema UnBall SSL Client.

    Configura os manipuladores de sinal, inicializa a aplicação PyQt,
    cria a instância principal do jogo e a janela da UI, inicia todos os
    subsistemas e executa o loop de eventos da aplicação.
    """
    global _game_instance_for_signal_handler

    # Configurar manipuladores de sinal para desligamento seguro
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        app = QApplication(sys.argv)
        if GAME_DEBUG:
            logger.info("Main: QApplication criada")

        game = Game()
        _game_instance_for_signal_handler = game
        if GAME_DEBUG:
            logger.info("Main: Instância Game criada")

        window = SSLClientWindow(game=game)
        game.window = window
        window.show()
        if GAME_DEBUG:
            logger.info("Main: SSLClientWindow exibida")

        game.start()
        if GAME_DEBUG:
            logger.info("Main: Sistemas do jogo iniciados")

        # Timer para manter a thread principal do Python responsiva aos sinais
        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(100)

        sys.exit(app.exec_())

    except Exception as e:
        logger.error(f"ERRO FATAL em main: {e}")
        traceback.print_exc()
        if (
            "_game_instance_for_signal_handler" in globals()
            and _game_instance_for_signal_handler
        ):
            _game_instance_for_signal_handler.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()
