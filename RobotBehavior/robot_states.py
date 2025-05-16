# RobotBehavior/robot_states.py
from enum import Enum


class RobotState(Enum):
    # --- This should be the ONLY block defining RobotState members ---

    # Common States
    IDLE = "IDLE"  # Robot is stationary, awaiting new commands.
    MOVING_TO_POSITION = (
        "MOVING_TO_POSITION"  # Generic: moving to a calculated target point.
    )
    RETURNING = (
        "RETURNING"  # Specifically moving to a 'home' or predefined fallback position.
    )
    AVOIDING_BALL = "AVOIDING_BALL"  # Actively moving away from the ball (e.g., for STOP command or opponent set pieces).

    # Offensive States
    MOVING_TO_BALL = "MOVING_TO_BALL"  # Moving towards the ball to gain possession.
    ATTACKING = "ATTACKING"  # General state of having/nearing ball and trying to score or progress offensively.
    # This can be refined into DRIBBLING, AIMING, SHOOTING later.
    SUPPORTING_OFFENSE = "SUPPORTING_OFFENSE"  # Positioning to create passing options or support the ball carrier.

    # Defensive States
    DEFENDING = (
        "DEFENDING"  # General defensive action/posture, covering space or goal path.
    )
    BLOCKING = "BLOCKING"  # Specifically for goalkeepers or defenders trying to block a shot or direct path to goal.
    INTERCEPTING = (
        "INTERCEPTING"  # Actively trying to intercept a pass or a slowly moving ball.
    )
    MARKING_PLAYER = (
        "MARKING_PLAYER"  # Following and covering a specific opponent player.
    )
    MARKING_SPACE = "MARKING_SPACE"  # Covering a strategic defensive zone.
    CLEARING_BALL = "CLEARING_BALL"  # Actively trying to kick the ball out of a dangerous defensive area.

    # Set Piece States (can be used to denote active participation in a set piece)
    PREPARING_SET_PIECE = "PREPARING_SET_PIECE"  # Robot is positioning itself according to rules for a set piece (kick-off, free-kick).
    TAKING_SET_PIECE = "TAKING_SET_PIECE"  # Robot is the one designated to execute the set piece (e.g., taking a penalty or free kick).

    # Ball Placement States (ADD THESE MISSING STATES)
    BALL_PLACEMENT_ACTIVE = (
        "BALL_PLACEMENT_ACTIVE"  # Robot is actively placing the ball
    )
    BALL_PLACEMENT_AVOIDING = (
        "BALL_PLACEMENT_AVOIDING"  # Robot is staying away from ball placement
    )
    # -----------------------------------------------------------------


class RobotRole(Enum):
    GOALKEEPER = "GOALKEEPER"
    DEFENDER = "DEFENDER"
    ATTACKER = "ATTACKER"
    # Consider adding if needed:
    # SUPPORT = "SUPPORT"
    # MIDFIELDER = "MIDFIELDER"
