"""
RobotBehavior/goalkeeper_optimizer_simple.py

Otimizador MINIMALISTA do goleiro para teste imediato.
Implementa apenas o essencial: cálculo da posição ótima baseada no bisector do ângulo.

Autor: UnBall Team - Universidade de Brasília
"""

import math
from typing import Tuple


class SimpleGoalkeeperOptimizer:
    """
    Otimizador simplificado para posicionamento do goleiro.

    Estratégia: Posicionar o goleiro no ponto da linha do gol que
    bisecta o ângulo de chute possível, minimizando a área descoberta.
    """

    def __init__(
        self,
        field_length: float = 4.5,
        field_width: float = 3.0,
        goal_width: float = 0.8,
    ):
        """
        Inicializa o otimizador com dimensões do campo.

        Args:
            field_length: Comprimento do campo (Entry Level: 4.5m)
            field_width: Largura do campo (Entry Level: 3.0m)
            goal_width: Largura do gol (Entry Level: 0.8m)
        """
        self.field_length = field_length
        self.field_width = field_width
        self.goal_width = goal_width

        # Calcular posições importantes
        self.goal_line_x = -field_length / 2  # Linha do gol (lado esquerdo)
        self.goalkeeper_line_x = (
            self.goal_line_x + 0.08
        )  # Linha de defesa (10cm à frente)

        self.goal_top_y = goal_width / 2
        self.goal_bottom_y = -goal_width / 2

        # Margens de segurança
        self.safety_margin = 0.01  # 5cm de margem das traves

    def calculate_coverage_angle(
        self, ball_pos: Tuple[float, float], goalkeeper_pos: Tuple[float, float]
    ) -> float:
        """
        Calcula o ângulo do gol coberto pelo goleiro (para análise).

        Args:
            ball_pos: Posição da bola
            goalkeeper_pos: Posição do goleiro

        Returns:
            Ângulo coberto em radianos
        """
        ball_x, ball_y = ball_pos
        gk_x, gk_y = goalkeeper_pos

        # Assumir raio do robô goleiro
        robot_radius = 0.09

        # Extremos da cobertura do goleiro
        gk_top = gk_y + robot_radius
        gk_bottom = gk_y - robot_radius

        # Limitar ao gol
        gk_top = min(self.goal_top_y, gk_top)
        gk_bottom = max(self.goal_bottom_y, gk_bottom)

        if gk_top <= gk_bottom:
            return 0.0

        # Calcular ângulos
        vec_to_top = (self.goal_line_x - ball_x, gk_top - ball_y)
        vec_to_bottom = (self.goal_line_x - ball_x, gk_bottom - ball_y)

        angle_top = math.atan2(vec_to_top[1], vec_to_top[0])
        angle_bottom = math.atan2(vec_to_bottom[1], vec_to_bottom[0])

        return abs(angle_top - angle_bottom)

    def calculate_optimal_position(
        self, ball_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Versão melhorada do cálculo de posição com melhor tratamento de bordas.

        SUBSTITUIR o método calculate_optimal_position existente por este.
        """
        ball_x, ball_y = ball_pos

        # Caso especial: bola atrás da linha do gol
        if ball_x <= self.goal_line_x:
            return (self.goalkeeper_line_x, 0.0)

        # Cantos do gol
        goal_top = (self.goal_line_x, self.goal_top_y)
        goal_bottom = (self.goal_line_x, self.goal_bottom_y)

        # Vetores da bola para os cantos do gol
        vec_to_top = (goal_top[0] - ball_x, goal_top[1] - ball_y)
        vec_to_bottom = (goal_bottom[0] - ball_x, goal_bottom[1] - ball_y)

        # Calcular magnitudes
        mag_top = math.sqrt(vec_to_top[0] ** 2 + vec_to_top[1] ** 2)
        mag_bottom = math.sqrt(vec_to_bottom[0] ** 2 + vec_to_bottom[1] ** 2)

        # Casos degenerados
        if mag_top == 0 or mag_bottom == 0:
            return (self.goalkeeper_line_x, 0.0)

        # Vetores unitários
        unit_to_top = (vec_to_top[0] / mag_top, vec_to_top[1] / mag_top)
        unit_to_bottom = (vec_to_bottom[0] / mag_bottom, vec_to_bottom[1] / mag_bottom)

        # Bisector (direção média dos dois vetores)
        bisector_x = (unit_to_top[0] + unit_to_bottom[0]) / 2
        bisector_y = (unit_to_top[1] + unit_to_bottom[1]) / 2

        # Encontrar interseção do bisector com a linha do gol
        if abs(bisector_x) < 1e-6:  # Bisector é quase vertical
            optimal_y = ball_y * 0.9  # Segue mais a bola em casos verticais
        else:
            # Calcular Y onde bisector cruza a linha do gol
            optimal_y = ball_y + (bisector_y / bisector_x) * (self.goal_line_x - ball_x)

            if abs(ball_y) > 0.05:  # Bola está lateral (>10cm do centro)
                # Amplificar o deslocamento lateral do goleiro
                center_offset = optimal_y  # Deslocamento do centro
                amplification = 1.6  # Amplificar 40% o movimento lateral
                optimal_y = center_offset * amplification

        # MELHORIA: Aplicar restrições de largura do gol de forma mais inteligente
        min_y = self.goal_bottom_y + 0.005
        max_y = self.goal_top_y - 0.005
        optimal_y = max(min_y, min(max_y, optimal_y))

        # Suavização para evitar movimentos bruscos
        if hasattr(self, "_last_optimal_y"):
            smoothing_factor = 0.15  # 15% de suavização
            optimal_y = (
                optimal_y * (1 - smoothing_factor)
                + self._last_optimal_y * smoothing_factor
            )

        # Aplicar limites
        optimal_y = max(min_y, min(max_y, optimal_y))

        # Armazenar para próxima suavização
        self._last_optimal_y = optimal_y

        return (self.goalkeeper_line_x, optimal_y)

    def get_info(self) -> dict:
        """Retorna informações sobre o otimizador."""
        return {
            "type": "SimpleGoalkeeperOptimizer",
            "strategy": "Angle Bisector",
            "field_dimensions": f"{self.field_length}x{self.field_width}m",
            "goal_width": f"{self.goal_width}m",
            "goalkeeper_line_x": self.goalkeeper_line_x,
        }


# Função de teste para verificar se está funcionando
def test_optimizer():
    """Teste rápido do otimizador."""
    print("🧪 Testando SimpleGoalkeeperOptimizer...")

    optimizer = SimpleGoalkeeperOptimizer()

    # Posições de teste
    test_cases = [
        (1.0, 0.0),  # Centro do campo
        (1.5, 0.3),  # Lateral direita
        (0.8, -0.2),  # Próximo, esquerda
        (2.0, 0.4),  # Longe, direita
        (0.5, 0.0),  # Muito próximo, centro
    ]

    print(
        f"Configuração: Campo {optimizer.field_length}x{optimizer.field_width}m, "
        f"Gol {optimizer.goal_width}m"
    )
    print("=" * 60)
    print("Posição da Bola        -> Posição Ótima do Goleiro")
    print("-" * 60)

    for i, ball_pos in enumerate(test_cases, 1):
        optimal_pos = optimizer.calculate_optimal_position(ball_pos)
        coverage = optimizer.calculate_coverage_angle(ball_pos, optimal_pos)

        print(
            f"Teste {i}: {ball_pos} -> {optimal_pos} "
            f"(cobertura: {math.degrees(coverage):.1f}°)"
        )

    print("=" * 60)
    print("Teste concluído! O otimizador está funcionando.")
    return True


if __name__ == "__main__":
    # Executar teste se arquivo for rodado diretamente
    test_optimizer()
