"""
Behavior factory - create wandering behaviors by name.
"""

import logging

logger = logging.getLogger(__name__)


def create_behavior(behavior_name: str, forward_speed: float, turn_speed: float):
    """Create behavior instance by name."""
    from pathfinder.behaviors import (
        NaturalWanderBehavior,
        MaxClearanceBehavior,
        WallFollowerBehavior,
        RandomWanderBehavior,
        AvoidAndGoBehavior,
        SafetyWrapper,
        create_safe_wanderer,
        WallSide,
    )

    if behavior_name == "safe_wanderer":
        return create_safe_wanderer(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
    elif behavior_name == "natural_wander":
        inner = NaturalWanderBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "max_clearance":
        inner = MaxClearanceBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "wall_follower_right":
        inner = WallFollowerBehavior(
            wall_side=WallSide.RIGHT,
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "wall_follower_left":
        inner = WallFollowerBehavior(
            wall_side=WallSide.LEFT,
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "random_wander":
        inner = RandomWanderBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "avoid_and_go":
        inner = AvoidAndGoBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    else:
        raise ValueError(f"Unknown behavior: {behavior_name}")
