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
