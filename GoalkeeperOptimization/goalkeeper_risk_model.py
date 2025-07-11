"""
GoalkeeperOptimization/goalkeeper_risk_model.py

Modelo de risco para otimização do posicionamento do goleiro.
Este módulo define as funções de risco e probabilidade de defesa baseadas
na posição da bola e posição do goleiro.
"""

import math
import numpy as np
from typing import Tuple, Dict, List, Optional
from dataclasses import dataclass

from utils.logger import get_logger

logger = get_logger("goalkeeper_risk_model")


@dataclass
class FieldDimensions:
    """Dimensões do campo para cálculo de risco."""

    field_length: float = 4.5  # Entry Level default
    field_width: float = 3.0
    goal_width: float = 0.8
    goal_depth: float = 0.18
    defense_area_depth: float = 0.5
    defense_area_width: float = 1.35


@dataclass
class GoalkeeperConstraints:
    """Restrições físicas e operacionais do goleiro."""

    max_speed: float = 2.0  # m/s
    reaction_time: float = 0.1  # segundos
    robot_radius: float = 0.09  # metros
    safe_distance_from_goal_line: float = 0.05  # metros


class GoalkeeperRiskModel:
    """
    Modelo de risco para posicionamento do goleiro.

    Este modelo calcula o risco de tomar gol baseado na:
    - Posição atual da bola
    - Posição atual do goleiro
    - Ângulo de chute possível
    - Cobertura do gol pelo goleiro
    """

    def __init__(self, field_dims: FieldDimensions, constraints: GoalkeeperConstraints):
        self.field_dims = field_dims
        self.constraints = constraints
        self.logger = logger

        # Calcular posições importantes do campo
        self._calculate_field_positions()

        logger.info(
            f"GoalkeeperRiskModel initialized for field: {field_dims.field_length}x{field_dims.field_width}"
        )

    def _calculate_field_positions(self):
        """Calcula posições importantes do campo."""
        # Assumindo que o goleiro defende o gol no lado negativo do campo
        self.goal_line_x = -self.field_dims.field_length / 2
        self.goal_center = (self.goal_line_x, 0.0)
        self.goal_top = (self.goal_line_x, self.field_dims.goal_width / 2)
        self.goal_bottom = (self.goal_line_x, -self.field_dims.goal_width / 2)

        # Linha onde o goleiro pode se posicionar (ligeiramente à frente da linha do gol)
        self.goalkeeper_line_x = (
            self.goal_line_x + self.constraints.safe_distance_from_goal_line
        )

        logger.debug(
            f"Field positions calculated. Goal at x={self.goal_line_x}, GK line at x={self.goalkeeper_line_x}"
        )

    def calculate_optimal_position(
        self, ball_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Calcula a posição ótima do goleiro baseada na posição da bola.

        Estratégia: Posicionar o goleiro na linha de defesa, alinhado com o ângulo
        que minimiza a área de gol descoberta.

        Args:
            ball_pos: Posição da bola (x, y)

        Returns:
            Posição ótima do goleiro (x, y)
        """
        ball_x, ball_y = ball_pos

        # Se a bola está atrás da linha do gol, goleiro vai para o centro
        if ball_x <= self.goal_line_x:
            optimal_y = 0.0
        else:
            # Calcular o ponto na linha do gol que bisecta o ângulo de chute
            optimal_y = self._calculate_angle_bisector_position(ball_pos)

        # Restringir à largura do gol (com margem de segurança)
        goal_margin = self.constraints.robot_radius
        max_y = self.field_dims.goal_width / 2 - goal_margin
        min_y = -self.field_dims.goal_width / 2 + goal_margin

        optimal_y = max(min_y, min(max_y, optimal_y))

        optimal_pos = (self.goalkeeper_line_x, optimal_y)

        logger.debug(f"Ball at {ball_pos} -> Optimal GK position: {optimal_pos}")
        return optimal_pos

    def _calculate_angle_bisector_position(
        self, ball_pos: Tuple[float, float]
    ) -> float:
        """
        Calcula a posição Y que bisecta o ângulo de chute possível.

        Esta é uma aproximação da estratégia ótima de posicionamento do goleiro:
        posicionar-se no ponto que divide igualmente o ângulo de chute.

        Args:
            ball_pos: Posição da bola (x, y)

        Returns:
            Coordenada Y ótima na linha do gol
        """
        ball_x, ball_y = ball_pos

        # Vetores da bola para os cantos do gol
        vec_to_top = (self.goal_top[0] - ball_x, self.goal_top[1] - ball_y)
        vec_to_bottom = (self.goal_bottom[0] - ball_x, self.goal_bottom[1] - ball_y)

        # Normalizar vetores
        mag_top = math.sqrt(vec_to_top[0] ** 2 + vec_to_top[1] ** 2)
        mag_bottom = math.sqrt(vec_to_bottom[0] ** 2 + vec_to_bottom[1] ** 2)

        if mag_top == 0 or mag_bottom == 0:
            return 0.0  # Caso degenerado, retorna centro

        unit_to_top = (vec_to_top[0] / mag_top, vec_to_top[1] / mag_top)
        unit_to_bottom = (vec_to_bottom[0] / mag_bottom, vec_to_bottom[1] / mag_bottom)

        # Bisector é a média normalizada dos vetores unitários
        bisector = (
            (unit_to_top[0] + unit_to_bottom[0]) / 2,
            (unit_to_top[1] + unit_to_bottom[1]) / 2,
        )

        # Encontrar interseção do bisector com a linha do gol
        # Linha do gol tem x = goal_line_x, então resolvemos para y
        if abs(bisector[0]) < 1e-6:  # Bisector é vertical
            return ball_y

        # Equação da linha: (y - ball_y) = (bisector[1]/bisector[0]) * (x - ball_x)
        # Substituindo x = goal_line_x
        optimal_y = ball_y + (bisector[1] / bisector[0]) * (self.goal_line_x - ball_x)

        return optimal_y

    def calculate_defense_probability(
        self, ball_pos: Tuple[float, float], goalkeeper_pos: Tuple[float, float]
    ) -> float:
        """
        Calcula a probabilidade de defesa baseada nas posições.

        Args:
            ball_pos: Posição da bola (x, y)
            goalkeeper_pos: Posição do goleiro (x, y)

        Returns:
            Probabilidade de defesa (0.0 a 1.0)
        """
        # Calcular ângulo de chute efetivo (área do gol coberta pelo goleiro)
        covered_angle = self._calculate_covered_angle(ball_pos, goalkeeper_pos)
        total_angle = self._calculate_total_goal_angle(ball_pos)

        if total_angle == 0:
            return 1.0  # Bola atrás da linha, 100% de defesa

        coverage_ratio = covered_angle / total_angle

        # Modelo simplificado: probabilidade baseada na cobertura angular
        # Adicionar fatores como distância da bola e tempo de reação
        distance_factor = self._calculate_distance_factor(ball_pos)

        defense_prob = coverage_ratio * distance_factor
        return min(1.0, max(0.0, defense_prob))

    def _calculate_covered_angle(
        self, ball_pos: Tuple[float, float], goalkeeper_pos: Tuple[float, float]
    ) -> float:
        """Calcula o ângulo do gol coberto pelo goleiro."""
        ball_x, ball_y = ball_pos
        gk_x, gk_y = goalkeeper_pos

        # Projetar o goleiro na linha do gol
        gk_radius = self.constraints.robot_radius

        # Pontos extremos da cobertura do goleiro
        gk_coverage_top = gk_y + gk_radius
        gk_coverage_bottom = gk_y - gk_radius

        # Limitar à área do gol
        goal_top_y = self.field_dims.goal_width / 2
        goal_bottom_y = -self.field_dims.goal_width / 2

        covered_top = min(goal_top_y, gk_coverage_top)
        covered_bottom = max(goal_bottom_y, gk_coverage_bottom)

        if covered_top <= covered_bottom:
            return 0.0  # Nenhuma cobertura

        # Calcular ângulo subtendido pela área coberta
        vec_to_covered_top = (self.goal_line_x - ball_x, covered_top - ball_y)
        vec_to_covered_bottom = (self.goal_line_x - ball_x, covered_bottom - ball_y)

        angle_top = math.atan2(vec_to_covered_top[1], vec_to_covered_top[0])
        angle_bottom = math.atan2(vec_to_covered_bottom[1], vec_to_covered_bottom[0])

        covered_angle = abs(angle_top - angle_bottom)
        return covered_angle

    def _calculate_total_goal_angle(self, ball_pos: Tuple[float, float]) -> float:
        """Calcula o ângulo total do gol visto da posição da bola."""
        ball_x, ball_y = ball_pos

        vec_to_top = (self.goal_top[0] - ball_x, self.goal_top[1] - ball_y)
        vec_to_bottom = (self.goal_bottom[0] - ball_x, self.goal_bottom[1] - ball_y)

        angle_top = math.atan2(vec_to_top[1], vec_to_top[0])
        angle_bottom = math.atan2(vec_to_bottom[1], vec_to_bottom[0])

        total_angle = abs(angle_top - angle_bottom)
        return total_angle

    def _calculate_distance_factor(self, ball_pos: Tuple[float, float]) -> float:
        """
        Calcula fator baseado na distância da bola.
        Bolas mais próximas são mais difíceis de defender.
        """
        ball_x, ball_y = ball_pos
        distance_to_goal = math.sqrt(
            (ball_x - self.goal_center[0]) ** 2 + (ball_y - self.goal_center[1]) ** 2
        )

        # Normalizar distância (0 a comprimento do campo)
        max_distance = self.field_dims.field_length
        normalized_distance = min(1.0, distance_to_goal / max_distance)

        # Fator varia de 0.5 (muito próximo) a 1.0 (muito longe)
        distance_factor = 0.5 + 0.5 * normalized_distance
        return distance_factor

    def calculate_risk_score(
        self, ball_pos: Tuple[float, float], goalkeeper_pos: Tuple[float, float]
    ) -> float:
        """
        Calcula score de risco (0.0 = sem risco, 1.0 = risco máximo).

        Args:
            ball_pos: Posição da bola
            goalkeeper_pos: Posição do goleiro

        Returns:
            Score de risco (complemento da probabilidade de defesa)
        """
        defense_prob = self.calculate_defense_probability(ball_pos, goalkeeper_pos)
        risk_score = 1.0 - defense_prob

        logger.debug(
            f"Risk score: {risk_score:.3f} for ball at {ball_pos}, GK at {goalkeeper_pos}"
        )
        return risk_score

    def generate_risk_map(
        self, ball_pos: Tuple[float, float], resolution: float = 0.05
    ) -> Dict:
        """
        Gera mapa de risco para diferentes posições do goleiro.

        Args:
            ball_pos: Posição fixa da bola
            resolution: Resolução do grid de posições

        Returns:
            Dict com informações do mapa de risco
        """
        # Grid de posições possíveis do goleiro
        goal_half_width = self.field_dims.goal_width / 2
        margin = self.constraints.robot_radius

        y_positions = np.arange(
            -goal_half_width + margin, goal_half_width - margin + resolution, resolution
        )

        # Posição X fixa na linha de defesa
        gk_x = self.goalkeeper_line_x

        risk_map = {
            "ball_position": ball_pos,
            "goalkeeper_x": gk_x,
            "y_positions": y_positions.tolist(),
            "risk_scores": [],
            "defense_probabilities": [],
            "optimal_position": self.calculate_optimal_position(ball_pos),
        }

        for y in y_positions:
            gk_pos = (gk_x, y)
            risk_score = self.calculate_risk_score(ball_pos, gk_pos)
            defense_prob = self.calculate_defense_probability(ball_pos, gk_pos)

            risk_map["risk_scores"].append(risk_score)
            risk_map["defense_probabilities"].append(defense_prob)

        logger.info(
            f"Generated risk map for ball at {ball_pos} with {len(y_positions)} positions"
        )
        return risk_map


def create_default_risk_model() -> GoalkeeperRiskModel:
    """Cria modelo de risco com configurações padrão para Entry Level."""
    field_dims = FieldDimensions()  # Entry Level default
    constraints = GoalkeeperConstraints()

    return GoalkeeperRiskModel(field_dims, constraints)


# Exemplo de uso e teste
if __name__ == "__main__":
    # Criar modelo
    risk_model = create_default_risk_model()

    # Testar com posição da bola
    ball_position = (1.0, 0.5)
    optimal_gk_pos = risk_model.calculate_optimal_position(ball_position)

    print(f"Ball at: {ball_position}")
    print(f"Optimal goalkeeper position: {optimal_gk_pos}")

    # Gerar mapa de risco
    risk_map = risk_model.generate_risk_map(ball_position)

    print(f"\nRisk map generated:")
    print(f"- Positions evaluated: {len(risk_map['y_positions'])}")
    print(f"- Min risk score: {min(risk_map['risk_scores']):.3f}")
    print(f"- Max risk score: {max(risk_map['risk_scores']):.3f}")
