# RobotBehavior/robot_states.py
from enum import Enum


class RobotState(Enum):
    """
    Estados possíveis do robô durante uma partida de futebol robótico.

    Esta enumeração define todos os estados comportamentais que um robô pode assumir,
    organizados por categoria (comum, ofensivo, defensivo, etc.).
    """

    # --- Este deve ser o ÚNICO bloco definindo membros de RobotState ---

    # Estados Comuns
    IDLE = "IDLE"  # Robô está parado, aguardando novos comandos.
    STOPPED = "STOPPED"  # Robô completamente parado
    MOVING_TO_POSITION = (
        "MOVING_TO_POSITION"  # Genérico: movendo para um ponto alvo calculado.
    )
    FOLLOWING_PATH = "FOLLOWING_PATH"  # Seguindo um caminho específico
    RETURNING = "RETURNING"  # Especificamente movendo para uma posição 'inicial' ou de fallback predefinida.
    AVOIDING_BALL = "AVOIDING_BALL"  # Ativamente se afastando da bola (ex: para comando STOP ou jogadas do adversário).

    # Estados Ofensivos
    MOVING_TO_BALL = "MOVING_TO_BALL"  # Movendo em direção à bola para ganhar posse.
    ATTACKING = "ATTACKING"  # Estado geral de ter/aproximar da bola e tentar marcar ou progredir ofensivamente.
    # Pode ser refinado em DRIBBLING, AIMING, SHOOTING posteriormente.
    SUPPORTING_OFFENSE = "SUPPORTING_OFFENSE"  # Posicionando para criar opções de passe ou apoiar o portador da bola.
    CHARGING_KICK = "CHARGING_KICK"  # Preparando para chutar
    KICKING = "KICKING"  # Executando chute

    # Estados Defensivos
    DEFENDING = "DEFENDING"  # Ação/postura defensiva geral, cobrindo espaço ou caminho para o gol.
    BLOCKING = "BLOCKING"  # Especificamente para goleiros ou defensores tentando bloquear um chute ou caminho direto ao gol.
    INTERCEPTING = (
        "INTERCEPTING"  # Ativamente tentando interceptar um passe ou uma bola lenta.
    )
    MARKING_PLAYER = (
        "MARKING_PLAYER"  # Seguindo e cobrindo um jogador adversário específico.
    )
    MARKING_SPACE = "MARKING_SPACE"  # Cobrindo uma zona defensiva estratégica.
    CLEARING_BALL = "CLEARING_BALL"  # Ativamente tentando chutar a bola para fora de uma área defensiva perigosa.

    # Estados de Jogadas Especiais (podem ser usados para denotar participação ativa em uma jogada especial)
    PREPARING_SET_PIECE = "PREPARING_SET_PIECE"  # Robô está se posicionando conforme regras para uma jogada especial (pontapé inicial, falta).
    TAKING_SET_PIECE = "TAKING_SET_PIECE"  # Robô é o designado para executar a jogada especial (ex: cobrando pênalti ou falta).
    PREPARE_KICKOFF = "PREPARE_KICKOFF"  # Preparando para pontapé inicial

    # Estados de Posicionamento da Bola (ADICIONAR ESTES ESTADOS FALTANTES)
    BALL_PLACEMENT_ACTIVE = (
        "BALL_PLACEMENT_ACTIVE"  # Robô está ativamente posicionando a bola
    )
    BALL_PLACEMENT_AVOIDING = "BALL_PLACEMENT_AVOIDING"  # Robô está se mantendo longe do posicionamento da bola
    # -----------------------------------------------------------------


class RobotRole(Enum):
    """
    Papéis/funções possíveis do robô em campo.

    Define as diferentes posições que um robô pode assumir durante uma partida,
    determinando suas responsabilidades gerais.
    """

    GOALKEEPER = "GOALKEEPER"  # Goleiro - responsável por defender o gol
    DEFENDER = "DEFENDER"  # Defensor - foca em ações defensivas
    ATTACKER = "ATTACKER"  # Atacante - foca em ações ofensivas
    # Considere adicionar se necessário:
    # SUPPORT = "SUPPORT"       # Apoio - papel de suporte geral
    # MIDFIELDER = "MIDFIELDER" # Meio-campo - transição entre defesa e ataque
