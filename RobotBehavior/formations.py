# formations.py
class FormationManager:
    """Manages team formations for different scenarios"""

    def __init__(self, field_bounds):
        self.field_bounds = field_bounds

    def get_kickoff_formation(self, team_color):
        """Get positions for kickoff formation"""
        if team_color == "blue":
            return {
                0: (-2.0, 0.0),  # Goalkeeper
                1: (-1.0, 0.5),  # Defender
                2: (-0.5, 0.0),  # Attacker
            }
        else:
            # Yellow team positions (mirrored)
            return {
                0: (2.0, 0.0),  # Goalkeeper
                1: (1.0, 0.5),  # Defender
                2: (0.5, 0.0),  # Attacker
            }

    def get_penalty_attacking_formation(self):
        """Get positions when our team is attacking (taking penalty)"""
        return {
            # Robô 0 (Goleiro) - Fica no nosso gol
            0: (-4.3, 0.0, 0.0),
            # Robô 2 (Atacante/Cobrador) - Posicionado para cobrar o pênalti
            # Um pouco atrás da marca do pênalti para pegar impulso
            2: (3.2, 0.0, 0.0),
            # Robôs 1 e 3 (Apoio) - Atrás da linha do meio-campo, prontos para o rebote
            1: (-1.0, -1.5, 0.0),
            3: (-1.0, 1.5, 0.0),
        }

    def get_penalty_defending_formation(self):
        """Get positions when our team is defending penalty"""
        return {
            # Robô 0 (Goleiro) - Na linha do gol, pronto para defender
            0: (-4.4, 0.0, 0.0),
            # Robôs 1, 2 e 3 (Apoio) - Atrás da linha de meio-campo
            1: (-1.0, -2.0, 0.0),
            2: (-1.0, 0.0, 0.0),
            3: (-1.0, 2.0, 0.0),
        }
