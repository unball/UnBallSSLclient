"""
Módulo de utilitários para o UnBall SSL Client.

Este módulo contém funções auxiliares utilizadas em várias partes do sistema,
como cálculos geométricos, manipulação de configurações, e utilitários para
controle de posições e orientações.
"""

import math
import json
import os
import numpy as np
from typing import Tuple, List, Dict, Any, Optional, Set

from utils.logger import get_logger

# Configurar logger para este módulo
logger = get_logger("utils")


def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """
    Calcula a distância euclidiana entre dois pontos.

    Args:
        p1: Primeiro ponto (x1, y1)
        p2: Segundo ponto (x2, y2)

    Returns:
        Distância euclidiana entre os pontos
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def normalize_angle(angle: float) -> float:
    """
    Normaliza um ângulo para o intervalo [-π, π].

    Importante para garantir que os ângulos utilizados no controle dos robôs
    estejam sempre em um intervalo consistente.

    Args:
        angle: Ângulo em radianos

    Returns:
        Ângulo normalizado
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def angle_between(v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
    """
    Calcula o ângulo entre dois vetores.

    Útil para determinar a diferença angular entre duas direções,
    como a orientação atual do robô e a direção desejada.

    Args:
        v1: Primeiro vetor (x, y)
        v2: Segundo vetor (x, y)

    Returns:
        Ângulo em radianos
    """
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

    # Evitar divisão por zero
    if mag1 * mag2 == 0:
        return 0.0

    cos_angle = dot / (mag1 * mag2)
    cos_angle = max(
        -1.0, min(1.0, cos_angle)
    )  # Clamp para evitar erro de arredondamento
    return math.acos(cos_angle)


def angle_to_point(
    current_pos: Tuple[float, float], target_pos: Tuple[float, float]
) -> float:
    """
    Calcula o ângulo de um ponto atual para um ponto alvo.

    Args:
        current_pos: Posição atual (x, y)
        target_pos: Posição alvo (x, y)

    Returns:
        Ângulo em radianos
    """
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    return math.atan2(dy, dx)


def load_config(config_path: str, default_config_path: str = None) -> Dict[str, Any]:
    """
    Carrega configuração de um arquivo JSON.

    Tenta carregar do caminho principal, e se falhar, tenta o caminho padrão.
    Se ambos falharem, retorna uma configuração mínima.

    Args:
        config_path: Caminho para o arquivo de configuração
        default_config_path: Caminho opcional para configuração padrão

    Returns:
        Dicionário com configurações
    """
    try:
        with open(config_path, "r") as f:
            config = json.load(f)
            logger.info(f"Configuração carregada de {config_path}")
            return config
    except (FileNotFoundError, json.JSONDecodeError) as e:
        logger.warning(f"Erro ao carregar configuração de {config_path}: {e}")

        # Tentar carregar configuração padrão
        if default_config_path and os.path.exists(default_config_path):
            try:
                with open(default_config_path, "r") as f:
                    config = json.load(f)
                    logger.info(
                        f"Configuração padrão carregada de {default_config_path}"
                    )
                    return config
            except Exception as e:
                logger.error(
                    f"Erro ao carregar configuração padrão de {default_config_path}: {e}"
                )

        # Retornar configuração mínima
        logger.warning("Utilizando configuração mínima padrão")
        return {
            "network": {
                "multicast_ip": "224.5.23.2",
                "vision_port": 10020,
                "referee_ip": "224.5.23.1",
                "referee_port": 10003,
                "yellow_port": 10004,
                "blue_port": 10005,
                "blue_control_port": 10301,
                "yellow_control_port": 10302,
            },
            "match": {
                "team_color": "blue",
                "division": "Entry Level",
                "team_side": "left",
                "control_mode": "grSim",
                "fps": 60,
            },
            "debug_flags": {
                "path_planning": True,
                "robot_behavior": True,
                "all": False,
            },
        }


def is_valid_position(pos: Tuple[float, float], field_bounds: Dict[str, float]) -> bool:
    """
    Verifica se uma posição está dentro dos limites do campo.

    Útil para garantir que alvos e caminhos estejam dentro dos limites jogáveis.

    Args:
        pos: Posição (x, y) para verificar
        field_bounds: Limites do campo {x_min, x_max, y_min, y_max}

    Returns:
        True se a posição for válida, False caso contrário
    """
    if pos is None or len(pos) != 2:
        return False

    x, y = pos
    if x is None or y is None or math.isnan(x) or math.isnan(y):
        return False

    margin = 0.1  # Margem de segurança de 10cm

    return (
        field_bounds["x_min"] - margin <= x <= field_bounds["x_max"] + margin
        and field_bounds["y_min"] - margin <= y <= field_bounds["y_max"] + margin
    )


def calculate_approaching_point(
    ball_pos: Tuple[float, float], target_pos: Tuple[float, float], distance: float
) -> Tuple[float, float]:
    """
    Calcula um ponto a uma certa distância do alvo, na direção oposta ao ponto final.

    Útil para calcular pontos de aproximação para chutes. Por exemplo, para chutar
    a bola em direção ao gol, o robô precisa se posicionar atrás da bola.

    Args:
        ball_pos: Posição da bola (x, y)
        target_pos: Posição alvo (ex: gol adversário) (x, y)
        distance: Distância do ponto de aproximação

    Returns:
        Ponto de aproximação (x, y)
    """
    dx = target_pos[0] - ball_pos[0]
    dy = target_pos[1] - ball_pos[1]

    # Distância entre os pontos
    dist = euclidean_distance(ball_pos, target_pos)

    # Se os pontos forem coincidentes, retornar uma posição padrão
    if dist < 0.001:
        logger.debug(
            f"Pontos coincidentes em calculate_approaching_point, usando offset padrão"
        )
        return (ball_pos[0] - distance, ball_pos[1])

    # Calcular o ponto de aproximação
    approach_x = ball_pos[0] - (dx / dist * distance)
    approach_y = ball_pos[1] - (dy / dist * distance)

    return (approach_x, approach_y)


def enforce_field_boundaries(
    pos: Tuple[float, float], field_bounds: Dict[str, float], margin: float = 0.05
) -> Tuple[float, float]:
    """
    Garante que uma posição esteja dentro dos limites do campo.

    Ajusta a posição para ficar dentro dos limites, se necessário.

    Args:
        pos: Posição (x, y) para ajustar
        field_bounds: Limites do campo {x_min, x_max, y_min, y_max}
        margin: Margem de segurança (metros)

    Returns:
        Posição ajustada dentro dos limites
    """
    x, y = pos

    original_pos = (x, y)
    x = max(field_bounds["x_min"] + margin, min(field_bounds["x_max"] - margin, x))
    y = max(field_bounds["y_min"] + margin, min(field_bounds["y_max"] - margin, y))

    if original_pos != (x, y):
        logger.debug(
            f"Posição {original_pos} ajustada para ({x:.2f}, {y:.2f}) para ficar dentro dos limites"
        )

    return (x, y)


def inflate_obstacles(
    obstacles: Set[Tuple[float, float]],
    inflation_radius: float,
    resolution: float,
    field_bounds: Dict[str, float] = None,
) -> Set[Tuple[float, float]]:
    """
    Infla obstáculos pelo raio do robô mais margem de segurança.

    Usado para planejamento de trajetória, garantindo que os robôs não colidam.

    Args:
        obstacles: Conjunto de posições de obstáculos
        inflation_radius: Raio de inflação (robô + margem)
        resolution: Resolução da grade
        field_bounds: Limites do campo (opcional)

    Returns:
        Conjunto de posições de obstáculos inflados
    """
    if not obstacles:
        return set()

    logger.debug(f"Inflando {len(obstacles)} obstáculos com raio {inflation_radius}m")

    # Convert to numpy array for efficient operations
    obstacle_array = np.array(list(obstacles))

    # Create a grid of points around each obstacle
    steps = int(inflation_radius / resolution)
    inflated = set()

    for x, y in obstacle_array:
        for dx in range(-steps, steps + 1):
            for dy in range(-steps, steps + 1):
                # Check if point is within inflation radius
                px = x + dx * resolution
                py = y + dy * resolution
                distance = math.sqrt(dx**2 + dy**2) * resolution

                if distance <= inflation_radius:
                    # Add to inflated obstacles set
                    inflated.add((round(px, 3), round(py, 3)))

                    # Optional: Constrain to field bounds
                    if field_bounds and not is_valid_position((px, py), field_bounds):
                        inflated.remove((round(px, 3), round(py, 3)))

    logger.debug(f"Obstáculos inflados para {len(inflated)} pontos")
    return inflated


def vector_to_speed(
    vector: Tuple[float, float], max_speed: float = 1.0
) -> Tuple[float, float]:
    """
    Converte um vetor de direção em comandos de velocidade para o robô.

    Normaliza o vetor e escalona para a velocidade máxima permitida.

    Args:
        vector: Vetor direção (dx, dy)
        max_speed: Velocidade máxima permitida

    Returns:
        Velocidades (vx, vy) a serem aplicadas
    """
    magnitude = math.sqrt(vector[0] ** 2 + vector[1] ** 2)

    if magnitude < 0.001:  # Evitar divisão por zero
        return (0.0, 0.0)

    # Normalizar e escalonar
    scale = min(max_speed / magnitude, 1.0)
    vx = vector[0] * scale
    vy = vector[1] * scale

    return (vx, vy)


def get_config() -> Dict[str, Any]:
    """
    Função auxiliar para obter a configuração do sistema.

    Procura o arquivo de configuração nos locais padrão.

    Returns:
        Dicionário com configurações
    """
    script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(script_dir, "config.json")
    default_config_path = os.path.join(script_dir, "config.default.json")

    logger.debug(f"Buscando configuração em: {config_path}")
    return load_config(config_path, default_config_path)
