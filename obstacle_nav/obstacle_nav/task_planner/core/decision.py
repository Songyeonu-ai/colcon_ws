import math


def is_front_obstacle(obstacle, params) -> bool:
    if not obstacle.detected:
        return False

    if obstacle.score < params.detect.min_score:
        return False

    if obstacle.distance > params.detect.avoid_trigger_dist:
        return False

    theta_limit_rad = math.radians(params.detect.frontal_theta_limit_deg)
    if abs(obstacle.theta) > theta_limit_rad:
        return False

    return True


def determine_target_lane(current_lane: int, obstacle, params) -> int:
    left_blocked = obstacle.lane_hint.left_blocked
    right_blocked = obstacle.lane_hint.right_blocked

    if current_lane == 0:
        return 1

    if current_lane == 2:
        return 1

    if left_blocked and not right_blocked:
        return 2

    if right_blocked and not left_blocked:
        return 0

    if params.decision.prefer_theta_based_avoid:
        if obstacle.theta >= 0.0:
            return 0
        return 2

    if params.decision.default_avoid_side == "right":
        return 2
    return 0