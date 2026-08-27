"""Shared active-map eligibility policy for the PC demo and map utilities."""

ACTIVE_MAX_BOXES = 10
ACTIVE_MAX_BOMBS = 5
BOMB_IDENTIFICATION_MIN_BOXES = 2
BOMB_IDENTIFICATION_MAX_BOXES = 10
BOMB_IDENTIFICATION_MIN_BOMBS = 1
BOMB_IDENTIFICATION_MAX_BOMBS = 5
BOX_CHARS = frozenset("$0123456789*")


def count_map_entities(rows):
    """Return physical box and bomb counts for normalized or ID-bearing rows."""
    box_count = 0
    bomb_count = 0
    for row in rows or ():
        for char in row:
            if char in BOX_CHARS:
                box_count += 1
            elif char == "B":
                bomb_count += 1
    return box_count, bomb_count


def map_scope(rows):
    """Describe whether a map is in the active UI/performance scope.

    Maps outside this range remain loadable when selected explicitly.  This
    policy only controls the demo's default automatic workflows.
    """
    box_count, bomb_count = count_map_entities(rows)
    reasons = []
    if box_count > ACTIVE_MAX_BOXES:
        reasons.append(f"box_count={box_count} exceeds {ACTIVE_MAX_BOXES}")
    if bomb_count > ACTIVE_MAX_BOMBS:
        reasons.append(f"bomb_count={bomb_count} exceeds {ACTIVE_MAX_BOMBS}")
    return {
        "eligible": not reasons,
        "box_count": box_count,
        "bomb_count": bomb_count,
        "reason": "; ".join(reasons),
    }


def split_active_cases(cases):
    """Return (eligible_cases, excluded_case_records) without mutating inputs."""
    eligible = []
    excluded = []
    for case in cases:
        scope = map_scope(case.get("rows"))
        if scope["eligible"]:
            eligible.append(case)
            continue
        excluded.append({
            "name": case.get("name", ""),
            "box_count": scope["box_count"],
            "bomb_count": scope["bomb_count"],
            "reason": scope["reason"],
        })
    return eligible, excluded


def bomb_identification_scope(rows):
    """Describe eligibility for the stricter bomb-identification CPU gate."""
    box_count, bomb_count = count_map_entities(rows)
    reasons = []
    if box_count < BOMB_IDENTIFICATION_MIN_BOXES:
        reasons.append(
            f"box_count={box_count} below {BOMB_IDENTIFICATION_MIN_BOXES}"
        )
    elif box_count > BOMB_IDENTIFICATION_MAX_BOXES:
        reasons.append(
            f"box_count={box_count} exceeds {BOMB_IDENTIFICATION_MAX_BOXES}"
        )
    if bomb_count < BOMB_IDENTIFICATION_MIN_BOMBS:
        reasons.append(
            f"bomb_count={bomb_count} below {BOMB_IDENTIFICATION_MIN_BOMBS}"
        )
    elif bomb_count > BOMB_IDENTIFICATION_MAX_BOMBS:
        reasons.append(
            f"bomb_count={bomb_count} exceeds {BOMB_IDENTIFICATION_MAX_BOMBS}"
        )
    return {
        "eligible": not reasons,
        "box_count": box_count,
        "bomb_count": bomb_count,
        "reason": "; ".join(reasons),
    }


def split_bomb_identification_cases(cases):
    """Return the exact 2..10-box, 1..5-bomb recognition-performance set."""
    eligible = []
    excluded = []
    for case in cases:
        scope = bomb_identification_scope(case.get("rows"))
        if scope["eligible"]:
            eligible.append(case)
            continue
        excluded.append({
            "name": case.get("name", ""),
            "box_count": scope["box_count"],
            "bomb_count": scope["bomb_count"],
            "reason": scope["reason"],
        })
    return eligible, excluded
