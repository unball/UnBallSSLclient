# PyQt/ssl_client.py
import os
import json
import math
import copy
import sys
from typing import Optional, Dict, List

from PyQt5.QtCore import Qt, QTimer
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import (
    QMainWindow,
    QVBoxLayout,
    QDialog,
    QPushButton,
    QMessageBox,
    QAction,
    QFrame,
    QCheckBox,
    QComboBox,
)

from .field_visualization import FieldVisualization
from .settings_ui import Ui_Dialog
from .debug_window import DebugWindow, DebugStreamRedirector
from RobotBehavior.robot_states import RobotState
from utils.logger import get_logger

logger = get_logger("SSLClientWindow")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(ROOT_DIR, "config.json")


class SSLClientWindow(QMainWindow):
    """
    Janela principal da interface gráfica (UI) para o UnBall SSL Client.
    """

    def __init__(self, game=None):
        super().__init__()
        logger.info("Inicializando a janela principal do SSL Client...")
        self.game = game
        uic.loadUi(os.path.join(SCRIPT_DIR, "main.ui"), self)

        self._setup_logging()
        self._setup_visualization()
        self._setup_connections_and_ui()

        if self.game:
            self.update_timer = QTimer()
            self.update_timer.timeout.connect(self.update_display)
            self.update_timer.start(16)
            logger.debug("Timer de atualização da UI iniciado a 60 FPS.")

        logger.info("SSL Client inicializado com sucesso.")

    def _setup_logging(self):
        """Configura o redirecionamento de logs para a janela de debug."""
        self.original_stdout = sys.stdout
        self.stdout_redirector = DebugStreamRedirector(self.original_stdout)
        sys.stdout = self.stdout_redirector
        self.stdout_redirector.text_written.connect(self.capture_log)

        self.path_planning_logs: List[str] = []
        self.vision_logs: List[str] = []
        self.sim_logs: List[str] = []
        self.debug_window: Optional[DebugWindow] = None
        logger.debug("Sistema de captura de logs configurado.")

    def _setup_visualization(self):
        """Configura o widget de visualização do campo."""
        self.field_widget = FieldVisualization()
        self.field_frame = self.findChild(QFrame, "field_frame")
        layout = QVBoxLayout(self.field_frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.field_widget)
        logger.debug("Widget de visualização do campo configurado.")

    def _setup_connections_and_ui(self):
        """Conecta todos os sinais de widgets da UI e configura o estado inicial."""
        # Menu
        self.findChild(QAction, "actionAbrir").triggered.connect(self.open_settings)
        self.findChild(QAction, "actionPathing_Debug").triggered.connect(
            self.show_debug_window
        )

        # Mapeamento de botões de controle
        self.team_aware_commands = [
            "KICK_OFF",
            "PENALTY",
            "FREE_KICK",
            "GOAL_KICK",
            "CORNER_KICK",
            "BALL_PLACEMENT",
        ]
        self.button_to_command = {
            "HALT": "HALT",
            "STOP": "STOP",
            "FORCE START": "FORCE_START",
            "NORMAL START": "NORMAL_START",
            "KICK-OFF": "KICK_OFF",
            "PENALTY": "PENALTY",
            "FREE-KICK POSITION": "FREE_KICK",  # Adicionado para mapear texto da UI
        }
        button_mapping = {
            "halt_button": "HALT",
            "stop_button": "STOP",
            "force_start_button": self.handle_force_start,
            "normal_start_button": self.handle_normal_start,
            "kickoff_button": "KICK_OFF",
            "free_kick_button": "FREE_KICK",
            "penalty_button": "PENALTY",
            "goal_kick_button": "GOAL_KICK",
            "corner_kick_button": "CORNER_KICK",
            "ball_placement_button": "BALL_PLACEMENT",
        }
        for btn_name, action in button_mapping.items():
            button = self.findChild(QPushButton, btn_name)
            if button:
                if callable(action):
                    button.clicked.connect(action)
                else:
                    button.clicked.connect(
                        lambda ch, a=action: self.handle_control_action(a)
                    )

        # ComboBoxes e CheckBoxes
        self.findChild(QComboBox, "comboBox").currentTextChanged.connect(
            self.handle_team_selection
        )
        self.division_combo = self.findChild(QComboBox, "division_combo")
        self.division_combo.currentTextChanged.connect(self.handle_division_change)

        self.findChild(QCheckBox, "gc_checkbox").toggled.connect(
            self.handle_game_controller_toggle
        )
        self.findChild(QCheckBox, "aestrela_checkbox").toggled.connect(
            self.update_a_star_visualization
        )
        self.findChild(QCheckBox, "show_blue").toggled.connect(
            self.update_team_visibility
        )
        self.findChild(QCheckBox, "show_yellow").toggled.connect(
            self.update_team_visibility
        )

        # Lógica de inicialização de divisão que depende da UI estar pronta
        QTimer.singleShot(100, self.initial_division_setup)
        logger.debug("Todas as conexões de widgets da UI foram estabelecidas.")

    # --- MÉTODOS DE ATUALIZAÇÃO DA UI ---

    def update_display(self):
        """Atualiza a visualização do campo com os dados mais recentes. Chamado a 60 FPS."""
        if not self.game:
            return

        vision_data = self.game.get_vision_data()
        if not vision_data:
            self.field_widget.clear_safely()
            return

        def is_valid(coord):
            return coord is not None and not math.isnan(coord)

        # Desenha bola e robôs
        ball = vision_data.get("ball", {})
        if is_valid(ball.get("x")) and is_valid(ball.get("y")):
            self.field_widget.update_ball(ball["x"], ball["y"])

        for robot_id, robot in vision_data.get("robotsBlue", {}).items():
            if (
                is_valid(robot.get("x"))
                and is_valid(robot.get("y"))
                and is_valid(robot.get("theta"))
            ):
                self.field_widget.update_robot(
                    robot["x"], robot["y"], robot["theta"], Qt.blue, int(robot_id)
                )

        for robot_id, robot in vision_data.get("robotsYellow", {}).items():
            if (
                is_valid(robot.get("x"))
                and is_valid(robot.get("y"))
                and is_valid(robot.get("theta"))
            ):
                self.field_widget.update_robot(
                    robot["x"], robot["y"], robot["theta"], Qt.yellow, int(robot_id)
                )

        # Desenha ou limpa as trajetórias do A*
        if self.findChild(QCheckBox, "aestrela_checkbox").isChecked():
            if hasattr(self.game, "path_planner") and self.game.robot_state_machines:
                for robot_id in self.game.robot_state_machines.keys():
                    path = self.game.path_planner.get_path(robot_id)
                    self.field_widget.update_path(robot_id, path or [])
        else:
            if hasattr(self.game, "robot_state_machines"):
                for robot_id in self.game.robot_state_machines.keys():
                    self.field_widget.update_path(robot_id, [])

        self.update_game_state_ui()

    def update_game_state_ui(self):
        """Atualiza a UI (dropdown) para refletir o estado atual do jogo."""
        if not (self.game and hasattr(self.game, "current_command")):
            return

        current_state = self.game.current_command or "HALT"

        ui_text = current_state
        for text, command in self.button_to_command.items():
            if command == current_state:
                ui_text = text
                break

        dropdown = self.findChild(QComboBox, "states")
        if dropdown:
            dropdown.blockSignals(True)
            index = dropdown.findText(ui_text, Qt.MatchFixedString)
            if index >= 0:
                dropdown.setCurrentIndex(index)
            dropdown.blockSignals(False)

    # --- HANDLERS DE AÇÕES DO USUÁRIO ---

    def start_game(self):
        """Garante que o jogo e as máquinas de estado sejam inicializados corretamente."""
        if not self.game:
            return

        if not self.game.running:
            logger.info("Primeira inicialização do jogo a partir da UI.")
            self.game.start()
        elif not self.game.robot_state_machines:
            logger.info("Jogo já rodando, inicializando máquinas de estado.")
            self.game._initialize_robot_state_machines()
        self.update_display()

    def handle_control_action(self, action: str):
        """Envia um comando de controle para o jogo (ex: HALT, STOP, KICK_OFF)."""
        if not self.game:
            return

        final_action = action
        if action in self.team_aware_commands:
            team_color = self.game.config["match"]["team_color"]
            final_action = f"{action}_{team_color.upper()}"

        logger.info(f"UI: Enviando comando '{final_action}'")
        self.game.set_game_state(final_action)
        self.update_game_state_ui()

    def handle_force_start(self):
        """Manipula o botão FORCE START."""
        logger.info("UI: Comando FORCE START")
        self.start_game()
        self.handle_control_action("FORCE_START")
        if self.game.get_vision_data():
            for sm in self.game.robot_state_machines.values():
                sm.update(self.game.get_vision_data())

    def handle_normal_start(self):
        """Manipula o botão NORMAL START."""
        logger.info("UI: Comando NORMAL START")
        self.start_game()
        self.handle_control_action("NORMAL_START")
        if self.game.get_vision_data():
            for sm in self.game.robot_state_machines.values():
                sm.update(self.game.get_vision_data())

    def handle_division_change(self, division: str):
        """Manipula a mudança de divisão, atualizando a lógica e a visualização."""
        if not self.game:
            return
        logger.info(f"UI: Divisão alterada para '{division}'")
        self.game.config["match"]["division"] = division
        self.game.update_field_bounds_for_division(division)
        self.game._initialize_robot_state_machines()
        self.field_widget.set_division(division)

    def handle_team_selection(self, team_text: str):
        """Manipula a troca de time (Azul/Amarelo)."""
        if not self.game:
            return
        team_color = "yellow" if "Amarelo" in team_text else "blue"
        if self.game.config["match"]["team_color"] != team_color:
            logger.info(f"UI: Time alterado para '{team_color}'")
            self.game.switch_team_color(team_color)

    def handle_game_controller_toggle(self, checked: bool):
        """Manipula o checkbox do Game Controller."""
        logger.info(f"UI: Game Controller {'ativado' if checked else 'desativado'}")
        if hasattr(self.game, "set_gc_mode"):
            self.game.set_gc_mode(checked)

    def update_team_visibility(self):
        """Atualiza quais times são visíveis no campo."""
        if self.field_widget:
            show_blue = self.findChild(QCheckBox, "show_blue").isChecked()
            show_yellow = self.findChild(QCheckBox, "show_yellow").isChecked()
            self.field_widget.set_team_visibility(show_blue, show_yellow)

    def update_a_star_visualization(self, checked: bool):
        """Avisa o widget do campo para mostrar/esconder as trajetórias A*."""
        logger.debug(f"UI: Visualização A* {'ativada' if checked else 'desativada'}")
        if self.field_widget:
            self.field_widget.set_show_paths(checked)

    def show_debug_window(self):
        """Abre ou foca a janela de debug."""
        if not self.debug_window or not self.debug_window.isVisible():
            self.debug_window = DebugWindow(parent=None, game=self.game)
        self.debug_window.show()
        self.debug_window.raise_()
        self.debug_window.activateWindow()
        logger.info("Janela de debug aberta.")

    def initial_division_setup(self):
        """Executa a configuração inicial da divisão após a UI estar pronta."""
        if self.division_combo:
            self.handle_division_change(self.division_combo.currentText())

    def open_settings(self):
        """Abre a janela de diálogo para configurar as definições de rede."""
        try:
            dialog = QDialog(self)
            settings_ui = Ui_Dialog()
            settings_ui.setupUi(dialog)

            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)

            net_config = config.get("network", {})
            settings_ui.multicast_ip_input.setText(net_config.get("multicast_ip", ""))
            settings_ui.vision_port_input.setText(
                str(net_config.get("vision_port", ""))
            )
            settings_ui.referee_ip_input.setText(net_config.get("referee_ip", ""))
            settings_ui.referee_port_input.setText(
                str(net_config.get("referee_port", ""))
            )

            # Conectar o botão de confirmação à função de salvar
            settings_ui.confirm_button.clicked.connect(
                lambda: self.save_settings(settings_ui, dialog, config)
            )
            dialog.exec_()
        except Exception as e:
            logger.error(f"Erro ao abrir configurações: {e}")
            QMessageBox.critical(
                self, "Erro", f"Não foi possível abrir as configurações: {e}"
            )

    def save_settings(self, settings_ui, dialog, config):
        """Salva as configurações de rede modificadas."""
        try:
            new_net_settings = {
                "multicast_ip": settings_ui.multicast_ip_input.text(),
                "vision_port": int(settings_ui.vision_port_input.text()),
                "referee_ip": settings_ui.referee_ip_input.text(),
                "referee_port": int(settings_ui.referee_port_input.text()),
            }

            if self.game.update_network_settings(new_net_settings):
                config["network"].update(new_net_settings)
                with open(CONFIG_PATH, "w") as f:
                    json.dump(config, f, indent=4)
                logger.info("Configurações de rede salvas e aplicadas.")
                QMessageBox.information(
                    dialog, "Sucesso", "Configurações salvas e aplicadas!"
                )
                dialog.accept()
            else:
                QMessageBox.warning(
                    dialog, "Erro", "Falha ao aplicar as configurações de rede no jogo."
                )
        except ValueError as e:
            QMessageBox.warning(
                dialog,
                "Entrada Inválida",
                f"Por favor, verifique os valores inseridos. Portas devem ser números.\n{e}",
            )
        except Exception as e:
            logger.error(f"Erro ao salvar configurações: {e}")
            QMessageBox.critical(
                dialog, "Erro", f"Ocorreu um erro ao salvar as configurações: {e}"
            )

    def capture_log(self, text: str):
        """Captura logs para a janela de debug (a categorização pode ser expandida)."""
        pass

    def closeEvent(self, event):
        """Executa limpeza ao fechar a janela."""
        logger.info("Fechando a janela e restaurando stdout.")
        sys.stdout = self.original_stdout
        if self.debug_window:
            self.debug_window.close()
        event.accept()
