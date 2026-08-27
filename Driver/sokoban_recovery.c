#include "sokoban_recovery.h"
#include "sokoban_scan.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifndef ALLOC_IN_SDRAM_CACHE
#define ALLOC_IN_SDRAM_CACHE SOKOBAN_BSS_SECTION("SDRAM_CACHE")
#endif

#ifndef ALLOC_IN_SDRAM
#define ALLOC_IN_SDRAM ALLOC_IN_SDRAM_CACHE
#endif

#define RECOVERY_SEARCH_CAPACITY 512u
#define RECOVERY_CANDIDATE_CAPACITY 64u
#define RECOVERY_TILE_CAPACITY (MAP_ROWS * MAP_COLS)
#define RECOVERY_INVALID_NODE 0xFFFFu
#define RECOVERY_INVALID_SLOT 0xFFu
#define RECOVERY_SCAN_STEP_SCORE 2u
#define RECOVERY_SCAN_TURN_SCORE 7u
#define RECOVERY_SCAN_BEND_SCORE 7u
#define RECOVERY_SCAN_VIEW_CORNER_PENALTY 24u
#define RECOVERY_SCAN_VIEW_BLOCKED_LINE_PENALTY 16u
#define RECOVERY_SAME_OBSERVATION_LIMIT 3u

typedef enum {
    RECOVERY_PHASE_IDLE = 0,
    RECOVERY_PHASE_WAIT_FIRST_OBSERVATION,
    RECOVERY_PHASE_WAIT_SECOND_OBSERVATION,
    RECOVERY_PHASE_PREPARE_ID,
    RECOVERY_PHASE_WAIT_ID,
    RECOVERY_PHASE_FINISHED
} SokobanRecoveryPhase;

typedef enum {
    RECOVERY_ENTITY_BOX = 0,
    RECOVERY_ENTITY_BOMB = 1
} RecoveryMoveEntity;

typedef enum {
    RECOVERY_IDENTIFY_TARGET = 0,
    RECOVERY_IDENTIFY_BOX = 1
} RecoveryIdentifyEntity;

typedef struct {
    uint16_t walls[MAP_ROWS];
    uint16_t targets[MAP_ROWS];
    Position player;
    Position boxes[MAX_BOXES];
    Entity bombs[MAX_BOMBS];
    uint16_t parent;
    uint16_t cost;
    uint16_t goal_distance;
    uint8_t bomb_count;
    uint8_t deliveries;
    uint8_t move_entity;
    uint8_t move_slot;
    uint8_t move_direction;
    uint8_t expanded;
} RecoveryNode;

typedef struct {
    uint8_t entity;
    uint8_t slot;
    uint8_t direction;
    uint8_t deliveries;
    uint16_t cost;
} RecoveryCandidate;

/* The camera-visible layers are contiguous in BitboardMap; keep only those
   four layers instead of repeating them in the session state. */
typedef struct {
    uint16_t walls[MAP_ROWS];
    uint16_t targets[MAP_ROWS];
    uint16_t bombs[MAP_ROWS];
    uint16_t boxes[MAP_ROWS];
} RecoveryPhysicalLayers;

typedef enum {
    RECOVERY_PARKING_NOT_NEEDED = 0,
    RECOVERY_PARKING_APPENDED,
    RECOVERY_PARKING_NO_SPACE,
    RECOVERY_PARKING_ERROR
} RecoveryParkingResult;

typedef struct {
    uint8_t kind;
    uint8_t slot;
    uint8_t view_direction;
    uint8_t end_heading;
    Position observation_pos;
    uint16_t path_len;
    uint16_t score;
} RecoveryScanRoute;

typedef struct {
    uint16_t virtual_boxes[MAP_ROWS];
    Position player;
    uint16_t parent;
    uint16_t cost;
    uint8_t next_target;
    uint8_t pushes;
    uint8_t direction;
    uint8_t expanded;
} RecheckNode;

typedef union {
    RecoveryNode recovery[RECOVERY_SEARCH_CAPACITY];
    RecheckNode recheck[RECOVERY_SEARCH_CAPACITY];
} RecoverySearchPool;

typedef struct {
    bool identified;
    bool fixed_pairs;
    bool fixed_group;
    uint16_t movable_boxes;
    uint16_t allowed_targets;
    uint8_t fixed_target_slots[MAX_BOXES];
    int8_t box_ids[MAX_BOXES];
    int8_t target_ids[MAX_TARGETS];
    Position target_positions[MAX_TARGETS];
} RecoveryRules;

static bool recovery_rules_can_move_box(const RecoveryRules* rules, uint8_t slot);
static bool recovery_rules_can_absorb(
    const RecoveryRules* rules,
    const RecoveryNode* node,
    uint8_t box_slot,
    Position target_pos
);
static RecoveryParkingResult recovery_append_target_parking_path(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    PathReplayState* state,
    const uint16_t historical_targets[MAP_ROWS],
    bool box_pushed
);

_Static_assert(sizeof(RecoveryNode) <= 256u, "Recovery node exceeds the fixed 256-byte budget.");
_Static_assert(sizeof(RecoveryCandidate) <= 32u, "Recovery candidate exceeds the fixed 32-byte budget.");
_Static_assert(sizeof(RecheckNode) <= 256u, "Recheck node exceeds the fixed 256-byte budget.");
_Static_assert(
    offsetof(BitboardMap, deadlocks) == sizeof(RecoveryPhysicalLayers),
    "Camera layers must remain contiguous in BitboardMap."
);
_Static_assert(RECOVERY_TILE_CAPACITY < RECOVERY_INVALID_SLOT, "Recovery tile index exceeds the byte queue.");

struct SokobanRecovery {
    Position return_point;
    Position recognition_player;
    SokobanRecoveryMode mode;
    SokobanRecoveryPhase phase;
    bool active;
    bool progress_observation_pending;
    bool last_observation_valid;
    uint8_t same_observation_count;
    RecoveryPhysicalLayers last_observation;
    /* A fully bound IDENTIFIED round may prove that every currently usable
       delivery was exhausted.  The next valid photo must match this complete
       post-segment prediction before the session may return without rescanning. */
    bool confirmed_exhaustion_observation_valid;
    RecoveryPhysicalLayers confirmed_exhaustion_observation;
    bool use_recheck_scan_route;
    bool use_dynamic_rescan_route;
    /* A two-target recheck records where each possible hidden box was pushed.
       A balanced second photo can then recover the fixed crossed pairing. */
    bool infer_recheck_cross_pair;
    uint8_t recheck_initial_box_count;
    uint8_t recheck_target_count;
    Position recheck_initial_boxes[MAX_BOXES];
    Position recheck_target_positions[MAX_TARGETS];
    Position recheck_landing_positions[MAX_TARGETS];
    uint16_t unresolved_suspect_targets[MAP_ROWS];

    uint16_t pool_count;
    bool pool_exhausted;
    uint8_t recognition_count;
    uint8_t recognition_index;
    uint8_t recognition_kind[MAX_BOXES + MAX_TARGETS];
    uint8_t recognition_slot[MAX_BOXES + MAX_TARGETS];
    uint8_t pending_kind;
    uint8_t pending_slot;
    uint8_t pending_directions;
    bool pending_id;
    uint8_t recognition_heading;

    uint8_t rescan_box_done[MAX_BOXES];
    uint8_t rescan_target_done[MAX_TARGETS];

    /* Snapshot the mixed scan order before residual-map loading clears it. */
    bool scan_route_valid;
    uint8_t scan_route_count;
    uint8_t scan_route_index;
    uint8_t scan_route_kind[MAX_BOXES + MAX_TARGETS];
    uint8_t scan_route_source_slot[MAX_BOXES + MAX_TARGETS];
    uint8_t scan_route_bound_slot[MAX_BOXES + MAX_TARGETS];
    Position scan_route_source_positions[MAX_BOXES + MAX_TARGETS];
    Entity scan_route_waypoints[MAX_BOXES + MAX_TARGETS];
    Position scan_route_pauses[MAX_BOXES + MAX_TARGETS];
    Direction scan_route_path[MAX_PATH_LENGTH];
    uint16_t scan_route_path_len;

    uint8_t box_recognized[MAX_BOXES];
    uint8_t target_recognized[MAX_TARGETS];
    int8_t box_ids[MAX_BOXES];
    int8_t target_ids[MAX_TARGETS];
    uint8_t fixed_target_slots[MAX_BOXES];
    bool fixed_group;

    Direction path[MAX_PATH_LENGTH];
    uint16_t path_len;
    SokobanRecoveryResult last_result;
};

static SokobanRecovery g_recovery ALLOC_IN_SDRAM;
static RecoverySearchPool g_recovery_pool ALLOC_IN_SDRAM;
static RecoveryCandidate g_recovery_candidates[RECOVERY_CANDIDATE_CAPACITY] ALLOC_IN_SDRAM;
static PathReplayState g_recovery_replay_state ALLOC_IN_SDRAM;
static PathReplayState g_recovery_recheck_state ALLOC_IN_SDRAM;
static PathReplayState g_recovery_recheck_candidate ALLOC_IN_SDRAM;
static BitboardMap g_recovery_feasibility_map ALLOC_IN_SDRAM;
static Direction g_recovery_nav_path[MAX_SINGLE_PATH] ALLOC_IN_SDRAM;
static Direction g_recovery_feasibility_path[MAX_SINGLE_PATH] ALLOC_IN_SDRAM;
static Direction g_recovery_return_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static uint8_t g_recovery_return_queue[RECOVERY_TILE_CAPACITY] ALLOC_IN_SDRAM;
static uint8_t g_recovery_return_parent[RECOVERY_TILE_CAPACITY] ALLOC_IN_SDRAM;
static uint8_t g_recovery_return_direction[RECOVERY_TILE_CAPACITY] ALLOC_IN_SDRAM;
static uint8_t g_recovery_scan_distance[RECOVERY_TILE_CAPACITY] ALLOC_IN_SDRAM;
static uint16_t g_recovery_scan_turn_cost[4][RECOVERY_TILE_CAPACITY] ALLOC_IN_SDRAM;
static uint8_t g_recovery_scan_turn_parent[4][RECOVERY_TILE_CAPACITY] ALLOC_IN_SDRAM;
static uint16_t g_recovery_chain[RECOVERY_SEARCH_CAPACITY] ALLOC_IN_SDRAM;
static Direction g_recovery_scan_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static Direction g_recovery_scan_candidate_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static Direction g_recovery_scan_best_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static Direction g_recovery_scan_generation_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;

uint8_t g_sokoban_recovery_need_return_path = 0u;

#define g_recovery_nodes (g_recovery_pool.recovery)
#define g_recovery_recheck_nodes (g_recovery_pool.recheck)

_Static_assert(
    sizeof(g_recovery) + sizeof(g_recovery_pool) + sizeof(g_recovery_candidates) +
    sizeof(g_recovery_replay_state) + sizeof(g_recovery_recheck_state) +
    sizeof(g_recovery_recheck_candidate) + sizeof(g_recovery_feasibility_map) + sizeof(g_recovery_nav_path) +
    sizeof(g_recovery_feasibility_path) + sizeof(g_recovery_return_path) + sizeof(g_recovery_return_queue) +
    sizeof(g_recovery_return_parent) + sizeof(g_recovery_return_direction) + sizeof(g_recovery_scan_distance) +
    sizeof(g_recovery_scan_turn_cost) + sizeof(g_recovery_scan_turn_parent) + sizeof(g_recovery_chain) +
    sizeof(g_recovery_scan_path) + sizeof(g_recovery_scan_candidate_path) + sizeof(g_recovery_scan_best_path) +
    sizeof(g_recovery_scan_generation_path) <= 72u * 1024u,
    "Recovery static storage exceeds the SDRAM budget."
);

static Position recovery_invalid_position(void) {
    return (Position){0xFFu, 0xFFu};
}

static bool recovery_position_valid(Position pos) {
    return pos.x < MAP_COLS && pos.y < MAP_ROWS;
}

static void recovery_clear_fixed_pairs(SokobanRecovery* recovery) {
    if (!recovery) return;
    recovery->fixed_group = false;
    for (uint8_t box = 0u; box < MAX_BOXES; box++) {
        recovery->fixed_target_slots[box] = RECOVERY_INVALID_SLOT;
    }
}

static void recovery_clear_recheck_snapshot(SokobanRecovery* recovery) {
    Position invalid = recovery_invalid_position();

    if (!recovery) return;
    recovery->recheck_initial_box_count = 0u;
    recovery->recheck_target_count = 0u;
    for (uint8_t box = 0u; box < MAX_BOXES; box++) {
        recovery->recheck_initial_boxes[box] = invalid;
    }
    for (uint8_t target = 0u; target < MAX_TARGETS; target++) {
        recovery->recheck_target_positions[target] = invalid;
        recovery->recheck_landing_positions[target] = invalid;
    }
}

static void recovery_clear_progress_tracking(SokobanRecovery* recovery) {
    if (!recovery) return;
    recovery->progress_observation_pending = false;
    recovery->last_observation_valid = false;
    recovery->same_observation_count = 0u;
    memset(&recovery->last_observation, 0, sizeof(recovery->last_observation));
    recovery->confirmed_exhaustion_observation_valid = false;
    memset(&recovery->confirmed_exhaustion_observation, 0,
           sizeof(recovery->confirmed_exhaustion_observation));
}

static void recovery_mark_progress_observation_pending(
    SokobanRecovery* recovery,
    const PathReplayState* state,
    bool can_confirm_exhaustion
) {
    if (!recovery) return;
    recovery->progress_observation_pending = true;
    recovery->confirmed_exhaustion_observation_valid =
        can_confirm_exhaustion && state != NULL;
    if (recovery->confirmed_exhaustion_observation_valid) {
        memcpy(&recovery->confirmed_exhaustion_observation, &state->map,
               sizeof(recovery->confirmed_exhaustion_observation));
    }
}

static bool recovery_observation_matches_last(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    if (!recovery || !solver || !recovery->last_observation_valid) return false;
    return memcmp(&recovery->last_observation, &solver->bmap,
                  sizeof(recovery->last_observation)) == 0;
}

static bool recovery_observation_confirms_exhaustion(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    if (!recovery || !solver ||
        !recovery->confirmed_exhaustion_observation_valid) {
        return false;
    }
    return memcmp(&recovery->confirmed_exhaustion_observation, &solver->bmap,
                  sizeof(recovery->confirmed_exhaustion_observation)) == 0;
}

static bool recovery_note_valid_observation(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    bool count_observation
) {
    bool unchanged;

    if (!recovery || !solver) return false;
    unchanged = recovery_observation_matches_last(recovery, solver);
    if (unchanged && count_observation) {
        if (recovery->same_observation_count < 0xFFu) {
            recovery->same_observation_count++;
        }
    } else if (!unchanged) {
        memcpy(&recovery->last_observation, &solver->bmap,
               sizeof(recovery->last_observation));
        recovery->last_observation_valid = true;
        /* The first valid map is the pre-action baseline and does not consume
           the three fresh-observation allowance.  A later changed map is
           itself the first fresh observation in the new physical streak. */
        recovery->same_observation_count = count_observation ? 1u : 0u;
    }
    return recovery->same_observation_count >= RECOVERY_SAME_OBSERVATION_LIMIT;
}

static uint8_t recovery_unresolved_suspect_count(
    const SokobanRecovery* recovery
) {
    uint8_t count = 0u;

    if (!recovery) return 0u;
    for (uint8_t row = 0u; row < MAP_ROWS; row++) {
        uint16_t bits = recovery->unresolved_suspect_targets[row];
        for (uint8_t col = 0u; col < MAP_COLS; col++) {
            if ((bits & (uint16_t)(1u << col)) != 0u) count++;
        }
    }
    return count;
}

/* Clear state that belongs to one observed recovery round, while retaining the
   original scan-route snapshot captured at begin(). */
static void recovery_reset_round_state(
    SokobanRecovery* recovery,
    bool clear_suspect_targets
) {
    if (!recovery) return;
    recovery_clear_recheck_snapshot(recovery);
    if (clear_suspect_targets) {
        memset(recovery->unresolved_suspect_targets, 0,
               sizeof(recovery->unresolved_suspect_targets));
    }
    recovery->infer_recheck_cross_pair = false;
    recovery->pending_id = false;
    recovery->pending_directions = 0u;
    recovery->recognition_index = 0u;
    recovery->recognition_count = 0u;
    recovery->recognition_heading = DIRECTION_INDEX_NONE;
    memset(recovery->box_recognized, 0, sizeof(recovery->box_recognized));
    memset(recovery->target_recognized, 0, sizeof(recovery->target_recognized));
    memset(recovery->rescan_box_done, 0, sizeof(recovery->rescan_box_done));
    memset(recovery->rescan_target_done, 0, sizeof(recovery->rescan_target_done));
    recovery->scan_route_index = 0u;
    for (uint8_t route = 0u; route < MAX_BOXES + MAX_TARGETS; route++) {
        recovery->scan_route_bound_slot[route] = RECOVERY_INVALID_SLOT;
    }
    for (uint8_t box = 0u; box < MAX_BOXES; box++) recovery->box_ids[box] = -1;
    for (uint8_t target = 0u; target < MAX_TARGETS; target++) recovery->target_ids[target] = -1;
    recovery_clear_fixed_pairs(recovery);
}

static bool recovery_offset(Position pos, Direction direction, int multiplier, Position* out) {
    int x;
    int y;

    if (!out) return false;
    x = (int)pos.x + direction.dx * multiplier;
    y = (int)pos.y + direction.dy * multiplier;
    if (!is_in_bounds(x, y)) return false;
    *out = (Position){(uint8_t)x, (uint8_t)y};
    return true;
}

static SokobanRecoveryResult recovery_result(
    SokobanRecoveryStatus status,
    SokobanRecoveryStatus next_status
) {
    SokobanRecoveryResult result;
    result.status = status;
    result.next_status = next_status;
    result.path = NULL;
    result.path_len = 0;
    result.observation_pos = recovery_invalid_position();
    result.observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE;
    result.entity_pos = recovery_invalid_position();
    result.view_direction = (Direction){0, 0};
    return result;
}

static uint8_t recovery_public_entity_kind(uint8_t kind) {
    if (kind == RECOVERY_IDENTIFY_TARGET) return SOKOBAN_RECOVERY_ENTITY_TARGET;
    if (kind == RECOVERY_IDENTIFY_BOX) return SOKOBAN_RECOVERY_ENTITY_BOX;
    return SOKOBAN_RECOVERY_ENTITY_NONE;
}

static bool recovery_pending_identification_metadata(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    Position observation_pos,
    Position* entity_pos,
    Direction* view_direction,
    uint8_t* observation_kind
) {
    const Entity* entity;
    int dx;
    int dy;
    Direction direction;
    Position resolved_entity;
    uint8_t public_kind;

    if (!recovery || !solver || !entity_pos || !view_direction ||
        !observation_kind || !recovery->pending_id ||
        !recovery_position_valid(observation_pos)) {
        return false;
    }
    public_kind = recovery_public_entity_kind(recovery->pending_kind);
    if (recovery->pending_kind == RECOVERY_IDENTIFY_TARGET) {
        if (recovery->pending_slot >= solver->num_targets) return false;
        entity = &solver->targets[recovery->pending_slot];
    } else if (recovery->pending_kind == RECOVERY_IDENTIFY_BOX) {
        if (recovery->pending_slot >= solver->num_boxes) return false;
        entity = &solver->boxes[recovery->pending_slot];
    } else {
        return false;
    }
    if (public_kind == SOKOBAN_RECOVERY_ENTITY_NONE || !entity->is_active ||
        !recovery_position_valid(entity->pos)) {
        return false;
    }

    dx = (int)entity->pos.x - (int)observation_pos.x;
    dy = (int)entity->pos.y - (int)observation_pos.y;
    if (dx < -1 || dx > 1 || dy < -1 || dy > 1) return false;
    direction = (Direction){(int8_t)dx, (int8_t)dy};
    if (!direction_is_cardinal(direction) ||
        !recovery_offset(observation_pos, direction, 1, &resolved_entity) ||
        !pos_equal(resolved_entity, entity->pos)) {
        return false;
    }

    *entity_pos = entity->pos;
    *view_direction = direction;
    *observation_kind = public_kind;
    return true;
}

static bool recovery_fill_identification_metadata(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    Position observation_pos,
    SokobanRecoveryResult* result
) {
    if (!result ||
        !recovery_pending_identification_metadata(
            recovery,
            solver,
            observation_pos,
            &result->entity_pos,
            &result->view_direction,
            &result->observation_kind)) {
        return false;
    }
    result->observation_pos = observation_pos;
    return true;
}

static bool recovery_identification_metadata_matches(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    const SokobanRecoveryResult* result
) {
    Position entity_pos;
    Direction view_direction;
    uint8_t observation_kind;

    if (!result ||
        !recovery_pending_identification_metadata(
            recovery,
            solver,
            result->observation_pos,
            &entity_pos,
            &view_direction,
            &observation_kind)) {
        return false;
    }
    return pos_equal(result->entity_pos, entity_pos) &&
           direction_equal(result->view_direction, view_direction) &&
           result->observation_kind == observation_kind;
}

static SokobanRecoveryResult recovery_path_result(
    const SokobanRecovery* recovery,
    SokobanRecoveryStatus next_status
) {
    SokobanRecoveryResult result = recovery_result(SOKOBAN_RECOVERY_PATH_READY, next_status);
    result.path = recovery->path;
    result.path_len = recovery->path_len;
    return result;
}

static SokobanRecoveryResult recovery_finish_with_error(SokobanRecovery* recovery) {
    g_sokoban_recovery_need_return_path = 0u;
    if (recovery) {
        recovery->path_len = 0;
        recovery->phase = RECOVERY_PHASE_FINISHED;
        recovery->active = false;
        recovery_clear_progress_tracking(recovery);
    }
    return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
}

static SokobanRecoveryResult recovery_retry_observation(SokobanRecovery* recovery) {
    SokobanRecoveryResult result;

    if (!recovery) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    result = recovery_result(
        SOKOBAN_RECOVERY_RETRY_OBSERVATION,
        SOKOBAN_RECOVERY_RETRY_OBSERVATION
    );
    recovery->last_result = result;
    return result;
}

static bool recovery_wall_at(const BitboardMap* map, int x, int y) {
    if (!is_in_bounds(x, y)) return true;
    return get_bit(map->walls, x, y);
}

static bool recovery_is_static_deadlock_at(
    const BitboardMap* map,
    Position pos,
    bool target_absorbs_box
) {
    bool wall_up;
    bool wall_down;
    bool wall_left;
    bool wall_right;

    if (!map || !recovery_position_valid(pos)) return true;
    if (target_absorbs_box && get_bit(map->targets, pos.x, pos.y)) return false;

    wall_up = recovery_wall_at(map, (int)pos.x, (int)pos.y - 1);
    wall_down = recovery_wall_at(map, (int)pos.x, (int)pos.y + 1);
    wall_left = recovery_wall_at(map, (int)pos.x - 1, (int)pos.y);
    wall_right = recovery_wall_at(map, (int)pos.x + 1, (int)pos.y);
    return (wall_up || wall_down) && (wall_left || wall_right);
}

static bool recovery_is_static_deadlock(const BitboardMap* map, Position pos) {
    return recovery_is_static_deadlock_at(map, pos, true);
}

static void recovery_refresh_layers(BitboardMap* map) {
    const uint16_t valid_cols_mask = (uint16_t)((1u << MAP_COLS) - 1u);
    const uint16_t edge_cols_mask = (uint16_t)(bit_mask_at(0) | bit_mask_at(MAP_COLS - 1));
    const uint16_t interior_cols_mask = (uint16_t)(valid_cols_mask & (uint16_t)(~edge_cols_mask));

    if (!map) return;
    memset(map->deadlocks, 0, sizeof(map->deadlocks));
    memset(map->h_tunnels, 0, sizeof(map->h_tunnels));
    memset(map->v_tunnels, 0, sizeof(map->v_tunnels));

    for (int y = 0; y < MAP_ROWS; y++) {
        for (int x = 0; x < MAP_COLS; x++) {
            Position pos = {(uint8_t)x, (uint8_t)y};
            if (get_bit(map->walls, x, y) || get_bit(map->targets, x, y)) continue;
            if (recovery_is_static_deadlock(map, pos)) set_bit(map->deadlocks, x, y);
        }
    }

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        uint16_t self_empty = (uint16_t)(~(map->walls[y] | map->targets[y])) & interior_cols_mask;
        uint16_t wall_up = map->walls[y - 1] & valid_cols_mask;
        uint16_t wall_down = map->walls[y + 1] & valid_cols_mask;
        uint16_t wall_left = (uint16_t)(map->walls[y] << 1) & valid_cols_mask;
        uint16_t wall_right = (uint16_t)(map->walls[y] >> 1);

        map->h_tunnels[y] = (uint16_t)(wall_up & wall_down & self_empty);
        map->v_tunnels[y] = (uint16_t)(wall_left & wall_right & self_empty);
    }
}

static void recovery_node_init_from_solver(const SokobanSolver* solver, RecoveryNode* node) {
    if (!solver || !node) return;

    memset(node, 0, sizeof(*node));
    memcpy(node->walls, solver->bmap.walls, sizeof(node->walls));
    memcpy(node->targets, solver->bmap.targets, sizeof(node->targets));
    node->player = solver->start_player;
    node->parent = RECOVERY_INVALID_NODE;
    node->move_slot = RECOVERY_INVALID_SLOT;
    for (int i = 0; i < MAX_BOXES; i++) {
        node->boxes[i] = solver->boxes[i].is_active ? solver->boxes[i].pos : recovery_invalid_position();
    }
    node->bomb_count = solver->num_bombs;
    for (int i = 0; i < node->bomb_count && i < MAX_BOMBS; i++) node->bombs[i] = solver->bombs[i];
}

static void recovery_node_from_replay_state(
    const PathReplayState* state,
    const RecoveryNode* parent,
    RecoveryNode* node
) {
    if (!state || !parent || !node) return;

    memset(node, 0, sizeof(*node));
    memcpy(node->walls, state->map.walls, sizeof(node->walls));
    memcpy(node->targets, state->map.targets, sizeof(node->targets));
    node->player = state->player;
    memcpy(node->boxes, state->boxes, sizeof(node->boxes));
    node->bomb_count = (uint8_t)state->bomb_count;
    for (int i = 0; i < node->bomb_count && i < MAX_BOMBS; i++) node->bombs[i] = state->bombs[i];
    node->deliveries = parent->deliveries;
}

static bool recovery_node_to_replay_state(
    const SokobanRecovery* recovery,
    const RecoveryNode* node,
    PathReplayState* state
) {
    if (!node || !state || !recovery_position_valid(node->player)) return false;

    path_replay_init_state(state);
    memcpy(state->map.walls, node->walls, sizeof(node->walls));
    memcpy(state->map.targets, node->targets, sizeof(node->targets));
    state->player = node->player;
    memcpy(state->boxes, node->boxes, sizeof(node->boxes));

    for (int i = 0; i < MAX_BOXES; i++) {
        if (recovery_position_valid(state->boxes[i])) {
            set_bit(state->map.boxes, state->boxes[i].x, state->boxes[i].y);
        }
    }
    if (recovery) {
        for (int y = 0; y < MAP_ROWS; y++) {
            state->map.boxes[y] |= recovery->unresolved_suspect_targets[y];
        }
    }

    state->bomb_count = node->bomb_count;
    if (state->bomb_count > MAX_BOMBS) return false;
    for (int i = 0; i < state->bomb_count; i++) {
        state->bombs[i] = node->bombs[i];
        if (!recovery_position_valid(state->bombs[i].pos)) return false;
        set_bit(state->map.bombs, state->bombs[i].pos.x, state->bombs[i].pos.y);
    }
    recovery_refresh_layers(&state->map);
    return true;
}

static bool recovery_recheck_node_to_replay_state(
    const SokobanRecovery* recovery,
    const RecoveryNode* root,
    const RecheckNode* node,
    PathReplayState* state
) {
    if (!root || !node || !state) return false;
    if (!recovery_node_to_replay_state(recovery, root, state)) return false;
    for (int y = 0; y < MAP_ROWS; y++) {
        state->map.boxes[y] |= node->virtual_boxes[y];
    }
    state->player = node->player;
    recovery_refresh_layers(&state->map);
    return true;
}

static bool recovery_node_is_complete(const SokobanRecovery* recovery, const RecoveryNode* node) {
    if (!node) return false;
    for (int y = 0; y < MAP_ROWS; y++) {
        if (node->targets[y] != 0u) return false;
        if (recovery && recovery->unresolved_suspect_targets[y] != 0u) return false;
    }
    for (int i = 0; i < MAX_BOXES; i++) {
        if (recovery_position_valid(node->boxes[i])) return false;
    }
    return true;
}

static bool recovery_node_same_layout(const RecoveryNode* lhs, const RecoveryNode* rhs) {
    if (!lhs || !rhs) return false;
    if (memcmp(lhs->walls, rhs->walls, sizeof(lhs->walls)) != 0) return false;
    if (memcmp(lhs->targets, rhs->targets, sizeof(lhs->targets)) != 0) return false;
    if (memcmp(lhs->boxes, rhs->boxes, sizeof(lhs->boxes)) != 0) return false;
    if (lhs->bomb_count != rhs->bomb_count) return false;
    for (int i = 0; i < lhs->bomb_count; i++) {
        if (!pos_equal(lhs->bombs[i].pos, rhs->bombs[i].pos)) return false;
    }
    return true;
}

static bool recovery_node_has_monotonic_progress(
    const RecoveryNode* root,
    const RecoveryNode* node
) {
    if (!root || !node) return false;
    if (node->deliveries > root->deliveries || node->bomb_count < root->bomb_count) {
        return true;
    }
    for (uint8_t row = 0u; row < MAP_ROWS; row++) {
        uint16_t removed_walls = (uint16_t)(
            root->walls[row] & (uint16_t)~node->walls[row]
        );

        if (removed_walls != 0u) return true;
    }
    return false;
}

static uint16_t recovery_axis_distance(uint8_t lhs, uint8_t rhs) {
    return lhs >= rhs ? (uint16_t)(lhs - rhs) : (uint16_t)(rhs - lhs);
}

static uint16_t recovery_node_goal_distance(
    const RecoveryNode* node,
    const RecoveryRules* rules
) {
    uint16_t best = 0xFFFFu;

    if (!node) return best;
    for (uint8_t box = 0u; box < MAX_BOXES; box++) {
        Position box_pos = node->boxes[box];

        if (!recovery_position_valid(box_pos) ||
            !recovery_rules_can_move_box(rules, box)) {
            continue;
        }
        for (uint8_t row = 0u; row < MAP_ROWS; row++) {
            for (uint8_t col = 0u; col < MAP_COLS; col++) {
                Position target_pos = {col, row};
                uint16_t distance;

                if (!get_bit(node->targets, col, row)) continue;
                if (rules && rules->identified &&
                    !recovery_rules_can_absorb(rules, node, box, target_pos)) {
                    continue;
                }
                distance = (uint16_t)(
                    recovery_axis_distance(box_pos.x, col) +
                    recovery_axis_distance(box_pos.y, row)
                );
                if (distance < best) best = distance;
            }
        }
    }

    /* A blast is also irreversible progress.  Including bomb-to-wall distance
       lets the bounded retry reach a required topology change instead of
       spending the entire pool on reversible bomb displacement. */
    for (uint8_t bomb = 0u; bomb < node->bomb_count; bomb++) {
        Position bomb_pos = node->bombs[bomb].pos;

        if (!recovery_position_valid(bomb_pos)) continue;
        for (uint8_t row = 0u; row < MAP_ROWS; row++) {
            for (uint8_t col = 0u; col < MAP_COLS; col++) {
                uint16_t distance;

                if (!get_bit(node->walls, col, row)) continue;
                distance = (uint16_t)(
                    recovery_axis_distance(bomb_pos.x, col) +
                    recovery_axis_distance(bomb_pos.y, row)
                );
                if (distance < best) best = distance;
            }
        }
    }
    return best;
}

static void recovery_pool_reset(SokobanRecovery* recovery) {
    if (!recovery) return;
    memset(g_recovery_nodes, 0, sizeof(g_recovery_nodes));
    recovery->pool_count = 0;
    recovery->pool_exhausted = false;
}

static bool recovery_pool_add(
    SokobanRecovery* recovery,
    const RecoveryNode* candidate,
    uint16_t* out_index
) {
    if (!recovery || !candidate) return false;

    for (uint16_t i = 0; i < recovery->pool_count; i++) {
        RecoveryNode* existing = &g_recovery_nodes[i];

        if (!recovery_node_same_layout(existing, candidate)) continue;
        if (!pos_equal(existing->player, candidate->player)) continue;
        if (existing->cost <= candidate->cost || existing->expanded) return false;
        *existing = *candidate;
        if (out_index) *out_index = i;
        return true;
    }

    if (recovery->pool_count >= RECOVERY_SEARCH_CAPACITY) {
        recovery->pool_exhausted = true;
        return false;
    }
    g_recovery_nodes[recovery->pool_count] = *candidate;
    if (out_index) *out_index = recovery->pool_count;
    recovery->pool_count++;
    if (recovery->pool_count >= RECOVERY_SEARCH_CAPACITY) recovery->pool_exhausted = true;
    return true;
}

static uint16_t recovery_pool_select_next(
    SokobanRecovery* recovery,
    const RecoveryNode* root,
    bool prioritize_goal_distance
) {
    uint16_t best = RECOVERY_INVALID_NODE;

    if (!recovery) return RECOVERY_INVALID_NODE;
    for (uint16_t i = 0; i < recovery->pool_count; i++) {
        RecoveryNode* node = &g_recovery_nodes[i];
        RecoveryNode* best_node;
        bool node_progress;
        bool best_progress;

        if (node->expanded) continue;
        if (best == RECOVERY_INVALID_NODE) {
            best = i;
            continue;
        }
        best_node = &g_recovery_nodes[best];
        if (node->deliveries != best_node->deliveries) {
            if (node->deliveries > best_node->deliveries) best = i;
            continue;
        }
        if (prioritize_goal_distance) {
            node_progress = recovery_node_has_monotonic_progress(root, node);
            best_progress = recovery_node_has_monotonic_progress(root, best_node);
            if (node_progress != best_progress) {
                if (node_progress) best = i;
                continue;
            }
            if (node->goal_distance != best_node->goal_distance) {
                if (node->goal_distance < best_node->goal_distance) best = i;
                continue;
            }
        }
        if (node->cost < best_node->cost) {
            best = i;
        }
    }
    if (best != RECOVERY_INVALID_NODE) g_recovery_nodes[best].expanded = 1u;
    return best;
}

static void recovery_recheck_pool_reset(SokobanRecovery* recovery) {
    if (!recovery) return;
    memset(g_recovery_recheck_nodes, 0, sizeof(g_recovery_recheck_nodes));
    recovery->pool_count = 0;
    recovery->pool_exhausted = false;
}

static bool recovery_recheck_node_same_state(const RecheckNode* lhs, const RecheckNode* rhs) {
    if (!lhs || !rhs) return false;
    return lhs->next_target == rhs->next_target &&
           pos_equal(lhs->player, rhs->player) &&
           memcmp(lhs->virtual_boxes, rhs->virtual_boxes, sizeof(lhs->virtual_boxes)) == 0;
}

static bool recovery_recheck_node_is_better(const RecheckNode* candidate, const RecheckNode* existing) {
    if (!candidate || !existing) return false;
    if (candidate->pushes != existing->pushes) return candidate->pushes > existing->pushes;
    return candidate->cost < existing->cost;
}

static bool recovery_recheck_pool_add(
    SokobanRecovery* recovery,
    const RecheckNode* candidate,
    uint16_t* out_index
) {
    if (!recovery || !candidate) return false;

    for (uint16_t i = 0; i < recovery->pool_count; i++) {
        RecheckNode* existing = &g_recovery_recheck_nodes[i];
        if (!recovery_recheck_node_same_state(existing, candidate)) continue;
        if (existing->expanded || !recovery_recheck_node_is_better(candidate, existing)) return false;
        *existing = *candidate;
        if (out_index) *out_index = i;
        return true;
    }

    if (recovery->pool_count >= RECOVERY_SEARCH_CAPACITY) {
        recovery->pool_exhausted = true;
        return false;
    }
    g_recovery_recheck_nodes[recovery->pool_count] = *candidate;
    if (out_index) *out_index = recovery->pool_count;
    recovery->pool_count++;
    if (recovery->pool_count >= RECOVERY_SEARCH_CAPACITY) recovery->pool_exhausted = true;
    return true;
}

static uint16_t recovery_recheck_pool_select_next(
    SokobanRecovery* recovery,
    uint8_t target_count
) {
    uint16_t best = RECOVERY_INVALID_NODE;

    if (!recovery) return RECOVERY_INVALID_NODE;
    for (uint16_t i = 0; i < recovery->pool_count; i++) {
        RecheckNode* node = &g_recovery_recheck_nodes[i];
        uint8_t node_remaining;
        uint8_t node_upper_bound;
        uint8_t best_remaining;
        uint8_t best_upper_bound;

        if (node->expanded) continue;
        node_remaining = node->next_target < target_count ?
            (uint8_t)(target_count - node->next_target) : 0u;
        node_upper_bound = (uint8_t)(node->pushes + node_remaining);
        if (best == RECOVERY_INVALID_NODE) {
            best = i;
            continue;
        }
        best_remaining = g_recovery_recheck_nodes[best].next_target < target_count ?
            (uint8_t)(target_count - g_recovery_recheck_nodes[best].next_target) : 0u;
        best_upper_bound = (uint8_t)(g_recovery_recheck_nodes[best].pushes + best_remaining);
        if (node_upper_bound > best_upper_bound ||
            (node_upper_bound == best_upper_bound &&
             (node->pushes > g_recovery_recheck_nodes[best].pushes ||
              (node->pushes == g_recovery_recheck_nodes[best].pushes &&
               node->cost < g_recovery_recheck_nodes[best].cost)))) {
            best = i;
        }
    }
    if (best != RECOVERY_INVALID_NODE) g_recovery_recheck_nodes[best].expanded = 1u;
    return best;
}

static int recovery_recheck_best_node(const SokobanRecovery* recovery, uint8_t target_count) {
    int best_complete = -1;
    int best_prefix = -1;

    if (!recovery) return -1;
    for (uint16_t i = 0; i < recovery->pool_count; i++) {
        const RecheckNode* node = &g_recovery_recheck_nodes[i];
        int* best = node->next_target >= target_count ? &best_complete : &best_prefix;

        if (*best < 0 || recovery_recheck_node_is_better(node, &g_recovery_recheck_nodes[*best])) {
            *best = (int)i;
        }
    }
    if (best_complete < 0) return best_prefix;
    if (best_prefix < 0 ||
        !recovery_recheck_node_is_better(
            &g_recovery_recheck_nodes[best_prefix],
            &g_recovery_recheck_nodes[best_complete])) {
        return best_complete;
    }
    return best_prefix;
}

static bool recovery_rules_can_move_box(const RecoveryRules* rules, uint8_t slot) {
    if (!rules || !rules->identified) return true;
    if (slot >= MAX_BOXES) return false;
    return (rules->movable_boxes & (uint16_t)(1u << slot)) != 0u;
}

static bool recovery_rules_can_absorb(
    const RecoveryRules* rules,
    const RecoveryNode* node,
    uint8_t box_slot,
    Position target_pos
) {
    if (!rules || !rules->identified || !node || box_slot >= MAX_BOXES) return false;
    if ((rules->movable_boxes & (uint16_t)(1u << box_slot)) == 0u) return false;
    if (!get_bit(node->targets, target_pos.x, target_pos.y)) return false;

    if (rules->fixed_pairs) {
        uint8_t target_slot = rules->fixed_target_slots[box_slot];

        if (rules->fixed_group) return true;
        return target_slot < MAX_TARGETS &&
            pos_equal(rules->target_positions[target_slot], target_pos);
    }

    for (int target = 0; target < MAX_TARGETS; target++) {
        if ((rules->allowed_targets & (uint16_t)(1u << target)) == 0u) continue;
        if (!pos_equal(rules->target_positions[target], target_pos)) continue;
        return rules->box_ids[box_slot] == rules->target_ids[target];
    }
    return false;
}

static bool recovery_replay_path(
    const SokobanSolver* solver,
    PathReplayState* state,
    const Direction* path,
    uint16_t path_len,
    const PathReplayOptions* options,
    bool* out_box_pushed
) {
    if (out_box_pushed) *out_box_pushed = false;
    if (!solver || !state || (!path && path_len > 0u)) return false;
    for (uint16_t i = 0; i < path_len; i++) {
        PathReplayStepResult step = path_replay_step(solver, state, path[i], options);
        if (step.kind == PATH_REPLAY_STEP_ERROR || step.kind == PATH_REPLAY_STEP_STOPPED) return false;
        if (out_box_pushed && step.kind == PATH_REPLAY_STEP_PUSHED_BOX) {
            *out_box_pushed = true;
        }
    }
    return true;
}

static bool recovery_main_solver_configure_ids(
    const RecoveryRules* rules,
    SokobanSolver* solver
) {
    uint16_t all_boxes;
    uint16_t all_targets;

    /* Both recovery modes use the main solver at its normal MAX_BOXES/
       MAX_TARGETS capacity.  The 3+ count-equality eligibility gate is
       enforced by sokoban_recovery_submit_observation(), not here. */
    if (!rules || !solver || solver->num_boxes == 0u ||
        solver->num_boxes != solver->num_targets) {
        return false;
    }

    if (!rules->identified) {
        for (uint8_t box = 0u; box < solver->num_boxes; box++) solver->boxes[box].id = -1;
        for (uint8_t target = 0u; target < solver->num_targets; target++) solver->targets[target].id = -1;
        solver->strict_target_mode = false;
        return true;
    }

    solver->strict_target_mode = true;
    if (rules->fixed_pairs) {
        uint16_t claimed_targets = 0u;

        for (uint8_t target = 0u; target < solver->num_targets; target++) {
            solver->targets[target].id = rules->fixed_group ? 0 : (int8_t)target;
        }
        for (uint8_t box = 0u; box < solver->num_boxes; box++) {
            uint8_t target = rules->fixed_target_slots[box];

            if (target >= solver->num_targets ||
                (!rules->fixed_group &&
                 (claimed_targets & (uint16_t)(1u << target)) != 0u)) {
                return false;
            }
            claimed_targets |= (uint16_t)(1u << target);
            solver->boxes[box].id = rules->fixed_group ? 0 : (int8_t)target;
        }
        return true;
    }

    all_boxes = (uint16_t)((1u << solver->num_boxes) - 1u);
    all_targets = (uint16_t)((1u << solver->num_targets) - 1u);
    if ((rules->movable_boxes & all_boxes) != all_boxes ||
        (rules->allowed_targets & all_targets) != all_targets) {
        return false;
    }
    for (uint8_t box = 0u; box < solver->num_boxes; box++) {
        if (rules->box_ids[box] < 0 || rules->box_ids[box] > 9) return false;
        solver->boxes[box].id = rules->box_ids[box];
    }
    for (uint8_t target = 0u; target < solver->num_targets; target++) {
        if (rules->target_ids[target] < 0 || rules->target_ids[target] > 9) return false;
        solver->targets[target].id = rules->target_ids[target];
    }
    return true;
}

static bool recovery_solve_complete_with_main(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryNode* root,
    const RecoveryRules* rules,
    bool* out_box_pushed
) {
    SokobanSolver main_solver;
    PathReplayOptions replay_options;

    if (out_box_pushed) *out_box_pushed = false;
    if (!recovery || !solver || !root || !rules ||
        !recovery_position_valid(root->player)) {
        return false;
    }
    for (uint8_t row = 0u; row < MAP_ROWS; row++) {
        if (recovery->unresolved_suspect_targets[row] != 0u) return false;
    }
    if (!recovery_node_to_replay_state(recovery, root, &g_recovery_replay_state)) {
        return false;
    }

    /* Preserve the live solver's map layers and shared work buffers. */
    main_solver = *solver;
    main_solver.start_player = root->player;
    main_solver.is_scanning = false;
    main_solver.scan_waypoint_count = 0;
    main_solver.scan_current_index = 0;
    main_solver.best_path = recovery->path;
    main_solver.best_path_len = 0u;
    main_solver.best_steps = 0xFFFFu;
    for (uint8_t box = 0u; box < main_solver.num_boxes; box++) {
        if (!recovery_position_valid(root->boxes[box])) return false;
        main_solver.boxes[box].pos = root->boxes[box];
        main_solver.boxes[box].is_active = true;
    }
    main_solver.num_bombs = root->bomb_count;
    for (uint8_t bomb = 0u; bomb < main_solver.num_bombs; bomb++) {
        main_solver.bombs[bomb] = root->bombs[bomb];
        main_solver.bombs[bomb].is_active = true;
    }
    if (!recovery_main_solver_configure_ids(rules, &main_solver) ||
        !solver_solve_robust(&main_solver) ||
        main_solver.best_path_len >= MAX_PATH_LENGTH) {
        return false;
    }

    g_recovery_recheck_candidate = g_recovery_replay_state;
    replay_options = path_replay_default_options();
    replay_options.mode = PATH_REPLAY_STRICT_VALIDATE;
    if (!recovery_replay_path(
            &main_solver,
            &g_recovery_recheck_candidate,
            recovery->path,
            main_solver.best_path_len,
            &replay_options,
            out_box_pushed)) {
        return false;
    }
    for (uint8_t row = 0u; row < MAP_ROWS; row++) {
        if (g_recovery_recheck_candidate.map.boxes[row] != 0u ||
            g_recovery_recheck_candidate.map.targets[row] != 0u) {
            return false;
        }
    }
    for (uint8_t box = 0u; box < main_solver.num_boxes; box++) {
        if (recovery_position_valid(g_recovery_recheck_candidate.boxes[box])) return false;
    }

    recovery->path_len = main_solver.best_path_len;
    return true;
}

static bool recovery_navigate(
    SokobanSolver* solver,
    const BitboardMap* map,
    Position start,
    Position target,
    Direction* out_path,
    uint16_t* out_len
) {
    if (!solver || !map || !out_path || !out_len) return false;
    hash_table_clear();
    return astar_navigate_mask(
        solver->heap,
        solver->closed_list,
        map,
        start,
        target,
        MASK_WALL | MASK_BOX | MASK_BOMB,
        out_path,
        out_len
    );
}

static bool recovery_return_tile_blocked(const BitboardMap* map, Position pos) {
    if (!map || !recovery_position_valid(pos)) return true;
    return get_bit(map->walls, pos.x, pos.y) ||
           get_bit(map->boxes, pos.x, pos.y) ||
           get_bit(map->bombs, pos.x, pos.y);
}

/* The return path may span more than MAX_SINGLE_PATH, so use the map-sized fixed BFS. */
static bool recovery_find_return_path(
    const BitboardMap* map,
    Position start,
    Position target,
    Direction* out_path,
    uint16_t* out_len
) {
    uint16_t head = 0;
    uint16_t tail = 0;
    uint8_t start_index;
    uint8_t target_index;
    uint8_t current_index;
    uint16_t path_len = 0;

    if (!map || !out_path || !out_len ||
        !recovery_position_valid(start) || !recovery_position_valid(target)) {
        return false;
    }
    *out_len = 0;
    if (pos_equal(start, target)) return true;
    if (recovery_return_tile_blocked(map, start) || recovery_return_tile_blocked(map, target)) return false;

    start_index = (uint8_t)((uint16_t)start.y * MAP_COLS + start.x);
    target_index = (uint8_t)((uint16_t)target.y * MAP_COLS + target.x);
    memset(g_recovery_return_parent, RECOVERY_INVALID_SLOT, sizeof(g_recovery_return_parent));
    g_recovery_return_parent[start_index] = start_index;
    g_recovery_return_queue[tail++] = start_index;

    while (head < tail && g_recovery_return_parent[target_index] == RECOVERY_INVALID_SLOT) {
        Position current_pos;

        current_index = g_recovery_return_queue[head++];
        current_pos = (Position){
            (uint8_t)(current_index % MAP_COLS),
            (uint8_t)(current_index / MAP_COLS)
        };
        for (uint8_t direction = 0; direction < 4u; direction++) {
            Position next_pos;
            uint8_t next_index;

            if (!recovery_offset(current_pos, direction_from_index(direction), 1, &next_pos) ||
                recovery_return_tile_blocked(map, next_pos)) {
                continue;
            }
            next_index = (uint8_t)((uint16_t)next_pos.y * MAP_COLS + next_pos.x);
            if (g_recovery_return_parent[next_index] != RECOVERY_INVALID_SLOT) continue;
            g_recovery_return_parent[next_index] = current_index;
            g_recovery_return_direction[next_index] = direction;
            g_recovery_return_queue[tail++] = next_index;
        }
    }
    if (g_recovery_return_parent[target_index] == RECOVERY_INVALID_SLOT) return false;

    current_index = target_index;
    while (current_index != start_index) {
        Direction direction;

        if (path_len >= MAX_PATH_LENGTH) return false;
        direction = direction_from_index(g_recovery_return_direction[current_index]);
        out_path[path_len++] = direction;
        current_index = g_recovery_return_parent[current_index];
    }
    for (uint16_t left = 0, right = (uint16_t)(path_len - 1u); left < right; left++, right--) {
        Direction tmp = out_path[left];
        out_path[left] = out_path[right];
        out_path[right] = tmp;
    }
    *out_len = path_len;
    return true;
}

static bool recovery_apply_macro(
    const SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryNode* parent,
    const RecoveryRules* rules,
    uint8_t entity_kind,
    uint8_t entity_slot,
    uint8_t direction_index_value,
    Direction* out_path,
    uint16_t out_capacity,
    uint16_t* out_len,
    RecoveryNode* out_node
) {
    PathReplayState* state = &g_recovery_replay_state;
    Position entity_pos;
    Position stance;
    Position destination;
    Direction direction;
    PathReplayOptions options;
    PathReplayStepResult step;
    uint16_t nav_len = 0;
    uint16_t emitted_len = 0;

    if (!solver || !parent || !out_node || direction_index_value >= 4u) return false;
    if (!recovery_node_to_replay_state(recovery, parent, state)) return false;

    if (entity_kind == RECOVERY_ENTITY_BOX) {
        if (entity_slot >= MAX_BOXES || !recovery_rules_can_move_box(rules, entity_slot)) return false;
        entity_pos = state->boxes[entity_slot];
    } else if (entity_kind == RECOVERY_ENTITY_BOMB) {
        if (entity_slot >= (uint8_t)state->bomb_count) return false;
        entity_pos = state->bombs[entity_slot].pos;
    } else {
        return false;
    }
    if (!recovery_position_valid(entity_pos)) return false;

    direction = direction_from_index(direction_index_value);
    if (!recovery_offset(entity_pos, direction, -1, &stance) ||
        !recovery_offset(entity_pos, direction, 1, &destination)) {
        return false;
    }
    if (entity_kind == RECOVERY_ENTITY_BOX && rules && rules->fixed_pairs &&
        get_bit(parent->targets, destination.x, destination.y) &&
        !recovery_rules_can_absorb(rules, parent, entity_slot, destination)) {
        return false;
    }

    if (!recovery_navigate(
            solver,
            &state->map,
            state->player,
            stance,
            g_recovery_nav_path,
            &nav_len)) {
        return false;
    }
    if ((uint32_t)parent->cost + nav_len + 1u > MAX_PATH_LENGTH) return false;
    if (out_path && (uint32_t)nav_len + 1u > out_capacity) return false;

    options = path_replay_default_options();
    options.mode = PATH_REPLAY_STRICT_VALIDATE;
    if (rules && rules->identified) {
        options.box_target_mode = PATH_REPLAY_BOX_TARGET_SET_BOX_ON_TARGET;
        if (entity_kind == RECOVERY_ENTITY_BOX &&
            recovery_rules_can_absorb(rules, parent, entity_slot, destination)) {
            options.box_target_mode = PATH_REPLAY_BOX_TARGET_MARKED_CLEAR_TARGET_HIDE_BOX;
            options.marked_box_idx = entity_slot;
            options.marked_target_pos = destination;
        }
    }

    if (!recovery_replay_path(solver, state, g_recovery_nav_path, nav_len, &options, NULL)) return false;
    step = path_replay_step(solver, state, direction, &options);
    if (step.kind == PATH_REPLAY_STEP_ERROR || step.kind == PATH_REPLAY_STEP_STOPPED) return false;
    if (entity_kind == RECOVERY_ENTITY_BOX && step.kind != PATH_REPLAY_STEP_PUSHED_BOX) return false;
    if (entity_kind == RECOVERY_ENTITY_BOMB &&
        step.kind != PATH_REPLAY_STEP_PUSHED_BOMB &&
        step.kind != PATH_REPLAY_STEP_BLASTED_WALL) {
        return false;
    }
    if (entity_kind == RECOVERY_ENTITY_BOX && !step.box_absorbed &&
        recovery_is_static_deadlock_at(&state->map, destination, false)) {
        return false;
    }

    recovery_node_from_replay_state(state, parent, out_node);
    out_node->parent = RECOVERY_INVALID_NODE;
    out_node->cost = (uint16_t)(parent->cost + nav_len + 1u);
    out_node->deliveries = (uint8_t)(parent->deliveries + (step.box_absorbed ? 1u : 0u));
    out_node->move_entity = entity_kind;
    out_node->move_slot = entity_slot;
    out_node->move_direction = direction_index_value;

    if (out_path) {
        if (nav_len > 0u) memcpy(out_path, g_recovery_nav_path, nav_len * sizeof(Direction));
        out_path[nav_len] = direction;
        emitted_len = (uint16_t)(nav_len + 1u);
    }
    if (out_len) *out_len = emitted_len;
    return true;
}

static uint8_t recovery_collect_candidates(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryNode* node,
    const RecoveryRules* rules
) {
    uint8_t count = 0;
    RecoveryNode probe;

    if (!recovery || !solver || !node) return 0;
    for (uint8_t slot = 0; slot < MAX_BOXES; slot++) {
        if (!recovery_position_valid(node->boxes[slot])) continue;
        if (!recovery_rules_can_move_box(rules, slot)) continue;
        for (uint8_t direction = 0; direction < 4u; direction++) {
            if (count >= RECOVERY_CANDIDATE_CAPACITY) return count;
            if (!recovery_apply_macro(
                    recovery, solver, node, rules, RECOVERY_ENTITY_BOX, slot, direction,
                    NULL, 0, NULL, &probe)) {
                continue;
            }
            g_recovery_candidates[count++] = (RecoveryCandidate){
                RECOVERY_ENTITY_BOX, slot, direction, probe.deliveries, probe.cost
            };
        }
    }

    for (uint8_t slot = 0; slot < node->bomb_count; slot++) {
        for (uint8_t direction = 0; direction < 4u; direction++) {
            if (count >= RECOVERY_CANDIDATE_CAPACITY) return count;
            if (!recovery_apply_macro(
                    recovery, solver, node, rules, RECOVERY_ENTITY_BOMB, slot, direction,
                    NULL, 0, NULL, &probe)) {
                continue;
            }
            g_recovery_candidates[count++] = (RecoveryCandidate){
                RECOVERY_ENTITY_BOMB, slot, direction, probe.deliveries, probe.cost
            };
        }
    }
    return count;
}

static void recovery_expand_partial_pool(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryNode* root,
    const RecoveryRules* rules,
    bool prioritize_goal_distance
) {
    RecoveryNode initial;

    if (!recovery || !solver || !root) return;
    recovery_pool_reset(recovery);
    initial = *root;
    initial.goal_distance = prioritize_goal_distance ?
        recovery_node_goal_distance(&initial, rules) : 0u;
    if (!recovery_pool_add(recovery, &initial, NULL)) return;

    for (;;) {
        uint16_t node_index = recovery_pool_select_next(
            recovery, &initial, prioritize_goal_distance
        );
        RecoveryNode* node;
        uint8_t candidate_count;

        if (node_index == RECOVERY_INVALID_NODE || recovery->pool_exhausted) break;
        node = &g_recovery_nodes[node_index];
        candidate_count = recovery_collect_candidates(recovery, solver, node, rules);
        for (uint8_t i = 1; i < candidate_count; i++) {
            RecoveryCandidate candidate = g_recovery_candidates[i];
            uint8_t insert_at = i;

            while (insert_at > 0u &&
                   (candidate.deliveries > g_recovery_candidates[insert_at - 1u].deliveries ||
                    (candidate.deliveries == g_recovery_candidates[insert_at - 1u].deliveries &&
                     candidate.cost < g_recovery_candidates[insert_at - 1u].cost))) {
                g_recovery_candidates[insert_at] = g_recovery_candidates[insert_at - 1u];
                insert_at--;
            }
            g_recovery_candidates[insert_at] = candidate;
        }
        for (uint8_t i = 0; i < candidate_count; i++) {
            const RecoveryCandidate* candidate = &g_recovery_candidates[i];
            RecoveryNode child;

            if (!recovery_apply_macro(
                    recovery,
                    solver,
                    node,
                    rules,
                    candidate->entity,
                    candidate->slot,
                    candidate->direction,
                    NULL,
                    0,
                    NULL,
                    &child)) {
                continue;
            }
            child.parent = node_index;
            child.goal_distance = prioritize_goal_distance ?
                recovery_node_goal_distance(&child, rules) : 0u;
            (void)recovery_pool_add(recovery, &child, NULL);
            if (recovery->pool_exhausted) break;
        }
    }
}

static int recovery_find_best_partial_node(
    const SokobanRecovery* recovery,
    const RecoveryNode* root
) {
    int best = -1;
    int best_complete = -1;
    uint8_t best_deliveries = 0;
    uint32_t best_score = 0xFFFFFFFFu;

    if (!recovery || !root) return -1;

    for (uint16_t i = 0; i < recovery->pool_count; i++) {
        const RecoveryNode* node = &g_recovery_nodes[i];
        uint32_t score;

        if (recovery_node_is_complete(recovery, node)) {
            if (best_complete < 0 || node->cost < g_recovery_nodes[best_complete].cost) {
                best_complete = (int)i;
            }
            continue;
        }
        /* The return route is deliberately deferred until a later observation
           proves that no further progress is possible.  A published segment
           must still make monotonic progress: at least one delivery, consumed
           bomb, or destroyed wall.  Mere box/bomb displacement can alternate
           between layouts forever while evading the consecutive-same-layout
           observation guard. */
        if (i == 0u || recovery_node_same_layout(root, node)) continue;
        if (!recovery_node_has_monotonic_progress(root, node)) continue;
        score = node->cost;
        if (score > MAX_PATH_LENGTH) continue;
        if (best < 0 || node->deliveries > best_deliveries ||
            (node->deliveries == best_deliveries && score < best_score)) {
            best = (int)i;
            best_deliveries = node->deliveries;
            best_score = score;
        }
    }
    if (best_complete >= 0) return best_complete;
    return best;
}

static int recovery_search_partial(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryNode* root,
    const RecoveryRules* rules
) {
    int best;

    if (!recovery || !solver || !root) return -1;
    recovery_expand_partial_pool(recovery, solver, root, rules, false);
    best = recovery_find_best_partial_node(recovery, root);
    if (best >= 0) return best;

    /* Cost-first expansion can fill the fixed pool before reaching a distant
       delivery.  Retry the same bounded storage with a goal-directed frontier;
       the published result is still subject to the irreversible-progress gate. */
    recovery_expand_partial_pool(recovery, solver, root, rules, true);
    return recovery_find_best_partial_node(recovery, root);
}

static bool recovery_reconstruct_path_with_prefix(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryRules* rules,
    uint16_t final_index,
    uint16_t path_prefix_len,
    RecoveryNode* out_final_node,
    bool* out_box_pushed
) {
    uint16_t chain_count = 0;
    uint16_t current = final_index;
    RecoveryNode replay_node;

    if (!recovery || !solver || final_index >= recovery->pool_count ||
        path_prefix_len > MAX_PATH_LENGTH) return false;
    if (out_box_pushed) *out_box_pushed = false;
    while (current != RECOVERY_INVALID_NODE) {
        if (chain_count >= RECOVERY_SEARCH_CAPACITY) return false;
        g_recovery_chain[chain_count++] = current;
        current = g_recovery_nodes[current].parent;
    }
    if (chain_count == 0u || g_recovery_chain[chain_count - 1u] != 0u) return false;

    recovery->path_len = path_prefix_len;
    replay_node = g_recovery_nodes[0];
    for (int i = (int)chain_count - 2; i >= 0; i--) {
        const RecoveryNode* selected = &g_recovery_nodes[g_recovery_chain[i]];
        RecoveryNode next;
        uint16_t macro_len = 0;

        if (!recovery_apply_macro(
                recovery,
                solver,
                &replay_node,
                rules,
                selected->move_entity,
                selected->move_slot,
                selected->move_direction,
                &recovery->path[recovery->path_len],
                (uint16_t)(MAX_PATH_LENGTH - recovery->path_len),
                &macro_len,
                &next)) {
            return false;
        }
        recovery->path_len = (uint16_t)(recovery->path_len + macro_len);
        if (out_box_pushed && selected->move_entity == RECOVERY_ENTITY_BOX) {
            *out_box_pushed = true;
        }
        replay_node = next;
    }
    if (out_final_node) *out_final_node = replay_node;
    return true;
}

static SokobanRecoveryResult recovery_finish_with_return_from_state(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    PathReplayState* state
) {
    PathReplayOptions options;
    uint16_t return_len = 0u;

    if (!recovery || !solver || !state) {
        return recovery_finish_with_error(recovery);
    }
    g_sokoban_recovery_need_return_path = 0u;
    if (!recovery_find_return_path(
            &state->map,
            state->player,
            recovery->return_point,
            g_recovery_return_path,
            &return_len
        ) ||
        (uint32_t)recovery->path_len + return_len > MAX_PATH_LENGTH) {
        return recovery_finish_with_error(recovery);
    }
    options = path_replay_default_options();
    options.mode = PATH_REPLAY_STRICT_VALIDATE;
    if (!recovery_replay_path(
            solver, state, g_recovery_return_path, return_len, &options, NULL
        )) {
        return recovery_finish_with_error(recovery);
    }
    if (return_len > 0u) {
        memcpy(
            &recovery->path[recovery->path_len],
            g_recovery_return_path,
            return_len * sizeof(Direction)
        );
        recovery->path_len = (uint16_t)(recovery->path_len + return_len);
    }
    recovery_clear_progress_tracking(recovery);
    g_sokoban_recovery_need_return_path = 1u;
    recovery->phase = RECOVERY_PHASE_FINISHED;
    recovery->active = false;
    return recovery->path_len == 0u ?
        recovery_result(SOKOBAN_RECOVERY_PARTIAL_RETURNED, SOKOBAN_RECOVERY_PARTIAL_RETURNED) :
        recovery_path_result(recovery, SOKOBAN_RECOVERY_PARTIAL_RETURNED);
}

/* Publish every non-terminal recovery segment through one internal seam.  The
   caller supplies the physically replayed final state; this keeps parking and
   the no-space return fallback identical for complete delivery, bounded
   partial delivery, and conservative recheck paths. */
static SokobanRecoveryResult recovery_finish_progress_segment(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    PathReplayState* state,
    const uint16_t historical_targets[MAP_ROWS],
    bool box_pushed,
    bool can_confirm_exhaustion,
    SokobanRecoveryPhase next_phase
) {
    RecoveryParkingResult parking_result;

    if (!recovery || !solver || !state || !historical_targets) {
        return recovery_finish_with_error(recovery);
    }
    g_sokoban_recovery_need_return_path = 0u;
    parking_result = recovery_append_target_parking_path(
        recovery, solver, state, historical_targets, box_pushed
    );
    if (parking_result == RECOVERY_PARKING_ERROR) {
        return recovery_finish_with_error(recovery);
    }
    if (parking_result == RECOVERY_PARKING_NO_SPACE) {
        return recovery_finish_with_return_from_state(recovery, solver, state);
    }

    recovery_mark_progress_observation_pending(
        recovery, state, can_confirm_exhaustion
    );
    recovery->phase = next_phase;
    recovery->active = true;
    return recovery->path_len == 0u ?
        recovery_result(SOKOBAN_RECOVERY_NEED_OBSERVATION, SOKOBAN_RECOVERY_NEED_OBSERVATION) :
        recovery_path_result(recovery, SOKOBAN_RECOVERY_NEED_OBSERVATION);
}

static SokobanRecoveryResult recovery_plan_return_only(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    Position player
) {
    RecoveryNode root;

    if (!recovery || !solver || !recovery_position_valid(player)) {
        return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    }
    g_sokoban_recovery_need_return_path = 0u;
    recovery_clear_progress_tracking(recovery);
    recovery_node_init_from_solver(solver, &root);
    root.player = player;
    recovery->path_len = 0u;
    if (!recovery_node_to_replay_state(
            recovery, &root, &g_recovery_replay_state
        )) {
        return recovery_finish_with_error(recovery);
    }
    return recovery_finish_with_return_from_state(
        recovery, solver, &g_recovery_replay_state
    );
}

static bool recovery_round_has_complete_bindings(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    const RecoveryRules* rules
) {
    uint16_t claimed_targets = 0u;
    uint16_t all_targets;

    if (!recovery || !solver || !rules ||
        recovery->mode != SOKOBAN_RECOVERY_IDENTIFIED ||
        !rules->identified || solver->num_boxes == 0u ||
        solver->num_boxes != solver->num_targets) {
        return false;
    }

    if (rules->fixed_pairs) {
        all_targets = (uint16_t)((1u << solver->num_targets) - 1u);
        for (uint8_t box = 0u; box < solver->num_boxes; box++) {
            uint8_t target = rules->fixed_target_slots[box];

            if (target >= solver->num_targets ||
                (claimed_targets & (uint16_t)(1u << target)) != 0u) {
                return false;
            }
            claimed_targets |= (uint16_t)(1u << target);
        }
        return claimed_targets == all_targets;
    }

    for (uint8_t box = 0u; box < solver->num_boxes; box++) {
        if (!recovery->box_recognized[box] ||
            recovery->box_ids[box] < 0 || recovery->box_ids[box] > 9) {
            return false;
        }
    }
    for (uint8_t target = 0u; target < solver->num_targets; target++) {
        if (!recovery->target_recognized[target] ||
            recovery->target_ids[target] < 0 || recovery->target_ids[target] > 9) {
            return false;
        }
    }
    return true;
}

static SokobanRecoveryResult recovery_plan_delivery(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    RecoveryNode* root,
    const RecoveryRules* rules,
    bool allow_incomplete_progress
) {
    int partial_index;
    RecoveryNode search_root;
    uint16_t path_prefix_len = 0u;
    bool delivered_in_round = false;
    bool box_pushed_in_path = false;
    bool complete_box_pushed = false;
    bool layout_same;
    bool complete_bindings;

    if (!recovery || !solver || !root || !rules) {
        return recovery_finish_with_error(recovery);
    }

    g_sokoban_recovery_need_return_path = 0u;
    complete_bindings = recovery_round_has_complete_bindings(
        recovery, solver, rules
    );
    /* Partial recovery is a fallback, not a second complete-solve candidate. */
    if (recovery_solve_complete_with_main(
            recovery, solver, root, rules, &complete_box_pushed
        )) {
        if (recovery->path_len == 0u) {
            return recovery_plan_return_only(recovery, solver, root->player);
        }
        return recovery_finish_progress_segment(
            recovery,
            solver,
            &g_recovery_recheck_candidate,
            root->targets,
            complete_box_pushed,
            false,
            RECOVERY_PHASE_WAIT_FIRST_OBSERVATION
        );
    }

    /* The bounded fallback can find one safe delivery before its fixed pool
       fills.  Reuse the same identified bindings from that resulting state
       and search again, appending only when a later search finds another
       delivery.  This keeps several already-observed pairs in one physical
       action segment instead of forcing an unnecessary new recognition round. */
    search_root = *root;
    recovery->path_len = 0u;
    for (uint8_t delivery_round = 0u; delivery_round < MAX_BOXES; delivery_round++) {
        uint16_t previous_path_len = path_prefix_len;
        uint8_t previous_deliveries = search_root.deliveries;
        bool previous_box_pushed = box_pushed_in_path;
        bool box_pushed_in_round = false;

        partial_index = recovery_search_partial(recovery, solver, &search_root, rules);
        if (partial_index < 0) {
            if (!delivered_in_round) recovery->path_len = 0u;
            else recovery->path_len = previous_path_len;
            box_pushed_in_path = previous_box_pushed;
            break;
        }

        if (g_recovery_nodes[partial_index].deliveries <= previous_deliveries &&
            delivered_in_round) {
            recovery->path_len = previous_path_len;
            break;
        }

        if (!recovery_reconstruct_path_with_prefix(
                recovery, solver, rules, (uint16_t)partial_index,
                path_prefix_len, &search_root, &box_pushed_in_round
            )) {
            if (!delivered_in_round) recovery->path_len = 0u;
            else recovery->path_len = previous_path_len;
            box_pushed_in_path = previous_box_pushed;
            break;
        }
        box_pushed_in_path = previous_box_pushed || box_pushed_in_round;

        if (search_root.deliveries <= previous_deliveries) {
            /* Preserve the old behavior for a first, non-delivery monotonic
               topology change.  Keep this displacement-only path for the legacy
               allow_incomplete_progress branch below. */
            break;
        }

        delivered_in_round = true;
        path_prefix_len = recovery->path_len;
        if (recovery_node_is_complete(recovery, &search_root)) break;
    }

    layout_same = recovery_node_same_layout(root, &search_root);
    if (recovery->path_len == 0u && layout_same) {
        return recovery_plan_return_only(recovery, solver, root->player);
    }

    if (recovery_node_is_complete(recovery, &search_root)) {
        if (recovery->path_len == 0u) {
            return recovery_plan_return_only(recovery, solver, root->player);
        }
    } else {
        /* With one box and one target the pairing is already certain.  If the
           exhaustive fallback still cannot reach that target, a zero-delivery
           push only creates another photo/scan round and can oscillate forever. */
        if (!allow_incomplete_progress || layout_same) {
            return recovery_plan_return_only(recovery, solver, root->player);
        }
    }

    if (!recovery_node_to_replay_state(
            recovery, &search_root, &g_recovery_recheck_state
        )) {
        return recovery_finish_with_error(recovery);
    }
    return recovery_finish_progress_segment(
        recovery,
        solver,
        &g_recovery_recheck_state,
        root->targets,
        box_pushed_in_path,
        complete_bindings && delivered_in_round,
        RECOVERY_PHASE_WAIT_FIRST_OBSERVATION
    );
}

static bool recovery_recheck_box_can_reach_target(
    SokobanSolver* solver,
    const PathReplayState* state,
    Position box_pos
) {
    Position targets[MAX_TARGETS];
    uint16_t path_len = 0;
    int target_count = 0;
    int reached_target = -1;

    if (!solver || !state || !recovery_position_valid(box_pos)) return false;
    for (int y = 0; y < MAP_ROWS; y++) {
        for (int x = 0; x < MAP_COLS; x++) {
            if (!get_bit(state->map.targets, x, y)) continue;
            if (target_count >= MAX_TARGETS) return false;
            targets[target_count++] = (Position){(uint8_t)x, (uint8_t)y};
        }
    }
    if (target_count == 0) return false;
    g_recovery_feasibility_map = state->map;
    clear_bit(g_recovery_feasibility_map.boxes, box_pos.x, box_pos.y);
    recovery_refresh_layers(&g_recovery_feasibility_map);
    hash_table_clear();
    return astar_solve_multi_target_mask(
        solver->heap,
        solver->closed_list,
        &g_recovery_feasibility_map,
        state->player,
        box_pos,
        targets,
        target_count,
        &reached_target,
        MASK_WALL | MASK_BOX | MASK_BOMB,
        g_recovery_feasibility_path,
        &path_len,
        ASTAR_NO_MACRO_DEPTH
    );
}

static bool recovery_probe_recheck_push(
    SokobanSolver* solver,
    const PathReplayState* state,
    Position virtual_box,
    uint8_t direction_index_value,
    uint16_t path_prefix_len,
    PathReplayState* out_state,
    Position* out_virtual_box,
    uint16_t* out_nav_len
) {
    PathReplayState* candidate = out_state;
    Position stance;
    Position destination;
    Direction direction;
    PathReplayOptions options;
    PathReplayStepResult step;
    uint16_t nav_len = 0;

    if (!solver || !state || !candidate || !out_virtual_box || direction_index_value >= 4u) return false;
    direction = direction_from_index(direction_index_value);
    if (!recovery_offset(virtual_box, direction, -1, &stance) ||
        !recovery_offset(virtual_box, direction, 1, &destination)) {
        return false;
    }
    *candidate = *state;
    if (!recovery_navigate(
            solver,
            &candidate->map,
            candidate->player,
            stance,
            g_recovery_nav_path,
            &nav_len)) {
        return false;
    }
    if ((uint32_t)path_prefix_len + nav_len + 1u > MAX_PATH_LENGTH) return false;

    options = path_replay_default_options();
    options.mode = PATH_REPLAY_STRICT_VALIDATE;
    options.box_target_mode = PATH_REPLAY_BOX_TARGET_SET_BOX_ON_TARGET;
    if (!recovery_replay_path(solver, candidate, g_recovery_nav_path, nav_len, &options, NULL)) return false;
    step = path_replay_step(solver, candidate, direction, &options);
    if (step.kind != PATH_REPLAY_STEP_PUSHED_BOX ||
        recovery_is_static_deadlock_at(&candidate->map, destination, false) ||
        !recovery_recheck_box_can_reach_target(solver, candidate, destination)) {
        return false;
    }

    *out_virtual_box = destination;
    if (out_nav_len) *out_nav_len = nav_len;
    return true;
}

static bool recovery_append_recheck_push(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    PathReplayState* state,
    Position virtual_box,
    uint8_t direction_index_value
) {
    Position moved_box;
    uint16_t nav_len = 0;

    if (!recovery || !state) return false;
    if (!recovery_probe_recheck_push(
            solver,
            state,
            virtual_box,
            direction_index_value,
            recovery->path_len,
            &g_recovery_recheck_candidate,
            &moved_box,
            &nav_len)) {
        return false;
    }
    if (nav_len > 0u) {
        memcpy(&recovery->path[recovery->path_len], g_recovery_nav_path, nav_len * sizeof(Direction));
        recovery->path_len = (uint16_t)(recovery->path_len + nav_len);
    }
    recovery->path[recovery->path_len++] = direction_from_index(direction_index_value);
    *state = g_recovery_recheck_candidate;
    return true;
}

static bool recovery_position_was_historical_target(
    const SokobanRecovery* recovery,
    const uint16_t historical_targets[MAP_ROWS],
    Position pos
) {
    if (!recovery_position_valid(pos)) return false;
    if (historical_targets && get_bit(historical_targets, pos.x, pos.y)) return true;
    return recovery && get_bit(recovery->unresolved_suspect_targets, pos.x, pos.y);
}

static RecoveryParkingResult recovery_append_target_parking_path(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    PathReplayState* state,
    const uint16_t historical_targets[MAP_ROWS],
    bool box_pushed
) {
    Position best_parking = recovery_invalid_position();
    uint16_t best_len = (uint16_t)(MAX_PATH_LENGTH + 1u);
    uint16_t parking_len = 0;
    PathReplayOptions options;

    if (!recovery || !solver || !state || !recovery_position_valid(state->player)) {
        return RECOVERY_PARKING_ERROR;
    }
    if (!box_pushed || !recovery_position_was_historical_target(
            recovery, historical_targets, state->player)) {
        return RECOVERY_PARKING_NOT_NEEDED;
    }

    for (uint8_t y = 0; y < MAP_ROWS; y++) {
        for (uint8_t x = 0; x < MAP_COLS; x++) {
            Position candidate = {x, y};
            uint16_t candidate_len = 0;

            if (pos_equal(candidate, state->player) ||
                get_bit(state->map.targets, x, y) ||
                recovery_return_tile_blocked(&state->map, candidate)) {
                continue;
            }
            if (!recovery_find_return_path(
                    &state->map,
                    state->player,
                    candidate,
                    g_recovery_return_path,
                    &candidate_len)) {
                continue;
            }
            if (candidate_len > 0u &&
                (uint32_t)recovery->path_len + candidate_len <= MAX_PATH_LENGTH &&
                candidate_len < best_len) {
                best_parking = candidate;
                best_len = candidate_len;
            }
        }
    }

    if (!recovery_position_valid(best_parking) || best_len == 0u ||
        !recovery_find_return_path(
            &state->map,
            state->player,
            best_parking,
            g_recovery_return_path,
            &parking_len) ||
        parking_len != best_len) {
        return RECOVERY_PARKING_NO_SPACE;
    }

    options = path_replay_default_options();
    options.mode = PATH_REPLAY_STRICT_VALIDATE;
    for (uint16_t i = 0; i < parking_len; i++) {
        PathReplayStepResult step = path_replay_step(solver, state, g_recovery_return_path[i], &options);

        if (step.kind != PATH_REPLAY_STEP_MOVED) return RECOVERY_PARKING_ERROR;
    }
    if (get_bit(state->map.targets, state->player.x, state->player.y) ||
        pos_equal(state->player, best_parking) == false) {
        return RECOVERY_PARKING_ERROR;
    }

    memcpy(&recovery->path[recovery->path_len], g_recovery_return_path, parking_len * sizeof(Direction));
    recovery->path_len = (uint16_t)(recovery->path_len + parking_len);
    return RECOVERY_PARKING_APPENDED;
}

static int recovery_search_recheck(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryNode* root
) {
    RecheckNode initial;
    uint8_t target_count;

    if (!recovery || !solver || !root || solver->num_targets > MAX_TARGETS) return -1;
    target_count = (uint8_t)solver->num_targets;
    recovery_recheck_pool_reset(recovery);
    memset(&initial, 0, sizeof(initial));
    initial.player = root->player;
    initial.parent = RECOVERY_INVALID_NODE;
    initial.direction = RECOVERY_INVALID_SLOT;
    for (uint8_t target = 0; target < target_count; target++) {
        Position target_pos = solver->targets[target].pos;
        set_bit(initial.virtual_boxes, target_pos.x, target_pos.y);
    }
    if (!recovery_recheck_pool_add(recovery, &initial, NULL)) return -1;

    for (;;) {
        uint16_t node_index = recovery_recheck_pool_select_next(recovery, target_count);
        RecheckNode* node;
        PathReplayState* state = &g_recovery_recheck_state;
        Position virtual_box;

        if (node_index == RECOVERY_INVALID_NODE || recovery->pool_exhausted) break;
        node = &g_recovery_recheck_nodes[node_index];
        if (node->next_target >= target_count) continue;
        if (!recovery_recheck_node_to_replay_state(recovery, root, node, state)) continue;

        virtual_box = solver->targets[node->next_target].pos;
        for (uint8_t direction = 0; direction < 4u; direction++) {
            PathReplayState* candidate_state = &g_recovery_recheck_candidate;
            Position moved_box;
            uint16_t nav_len = 0;
            RecheckNode child;

            if (!recovery_probe_recheck_push(
                    solver,
                    state,
                    virtual_box,
                    direction,
                    node->cost,
                    candidate_state,
                    &moved_box,
                    &nav_len)) {
                continue;
            }

            child = *node;
            child.parent = node_index;
            child.player = candidate_state->player;
            child.cost = (uint16_t)(node->cost + nav_len + 1u);
            child.next_target = (uint8_t)(node->next_target + 1u);
            child.pushes = (uint8_t)(node->pushes + 1u);
            child.direction = direction;
            child.expanded = 0u;
            clear_bit(child.virtual_boxes, virtual_box.x, virtual_box.y);
            set_bit(child.virtual_boxes, moved_box.x, moved_box.y);
            (void)recovery_recheck_pool_add(recovery, &child, NULL);
            if (recovery->pool_exhausted) break;
        }
        if (recovery->pool_exhausted) break;

        {
            RecheckNode child = *node;
            child.parent = node_index;
            child.next_target = (uint8_t)(node->next_target + 1u);
            child.direction = RECOVERY_INVALID_SLOT;
            child.expanded = 0u;
            (void)recovery_recheck_pool_add(recovery, &child, NULL);
        }
    }

    return recovery_recheck_best_node(recovery, target_count);
}

static bool recovery_capture_recheck_snapshot(
    SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    if (!recovery || !solver || solver->num_boxes > MAX_BOXES ||
        solver->num_targets > MAX_TARGETS) {
        return false;
    }

    recovery_clear_recheck_snapshot(recovery);
    recovery->recheck_initial_box_count = (uint8_t)solver->num_boxes;
    recovery->recheck_target_count = (uint8_t)solver->num_targets;
    for (uint8_t box = 0u; box < recovery->recheck_initial_box_count; box++) {
        if (!solver->boxes[box].is_active || !recovery_position_valid(solver->boxes[box].pos)) {
            return false;
        }
        recovery->recheck_initial_boxes[box] = solver->boxes[box].pos;
    }
    for (uint8_t target = 0u; target < recovery->recheck_target_count; target++) {
        if (!solver->targets[target].is_active || !recovery_position_valid(solver->targets[target].pos)) {
            return false;
        }
        recovery->recheck_target_positions[target] = solver->targets[target].pos;
    }
    return true;
}

/* A two-target recheck starts with one visible box so the other box may be
   hidden under a target.  If the follow-up photo still contains exactly one
   box at the same physical position, no hidden box was exposed by the
   recheck.  The unresolved target therefore represents a box already stuck
   in a dead corner; do not spend another round scanning IDs. */
static bool recovery_recheck_missing_box_is_still_hidden(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    if (!recovery || !solver || recovery->recheck_initial_box_count != 1u ||
        recovery->recheck_target_count != 2u || solver->num_boxes != 1u ||
        solver->num_targets != 2u || !solver->boxes[0].is_active) {
        return false;
    }
    return pos_equal(solver->boxes[0].pos, recovery->recheck_initial_boxes[0]);
}

static bool recovery_capture_unresolved_recheck_targets(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    uint16_t final_index
) {
    const RecheckNode* final_node;
    uint16_t current = final_index;
    uint16_t chain_count = 0u;
    uint8_t target_count;

    if (!recovery || !solver || final_index >= recovery->pool_count) return false;
    target_count = (uint8_t)solver->num_targets;
    for (uint8_t target = 0u; target < MAX_TARGETS; target++) {
        recovery->recheck_landing_positions[target] = recovery_invalid_position();
    }
    final_node = &g_recovery_recheck_nodes[final_index];

    while (current != 0u) {
        const RecheckNode* child;
        const RecheckNode* parent;
        Position target_pos;

        if (current == RECOVERY_INVALID_NODE || current >= recovery->pool_count ||
            chain_count++ >= RECOVERY_SEARCH_CAPACITY) {
            return false;
        }
        child = &g_recovery_recheck_nodes[current];
        if (child->parent == RECOVERY_INVALID_NODE || child->parent >= recovery->pool_count) return false;
        parent = &g_recovery_recheck_nodes[child->parent];
        if (child->direction == RECOVERY_INVALID_SLOT) {
            if (parent->next_target >= target_count) return false;
            target_pos = solver->targets[parent->next_target].pos;
            if (!recovery_position_valid(target_pos)) return false;
            set_bit(recovery->unresolved_suspect_targets, target_pos.x, target_pos.y);
        } else {
            Position landing;

            if (parent->next_target >= target_count ||
                !recovery_offset(
                    solver->targets[parent->next_target].pos,
                    direction_from_index(child->direction),
                    1,
                    &landing)) {
                return false;
            }
            recovery->recheck_landing_positions[parent->next_target] = landing;
        }
        current = child->parent;
    }

    for (uint8_t target = final_node->next_target; target < target_count; target++) {
        Position target_pos = solver->targets[target].pos;

        if (!recovery_position_valid(target_pos)) return false;
        set_bit(recovery->unresolved_suspect_targets, target_pos.x, target_pos.y);
    }
    return true;
}

static bool recovery_reconstruct_recheck_path(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryNode* root,
    uint16_t final_index
) {
    uint16_t chain_count = 0;
    uint16_t current = final_index;
    PathReplayState* state = &g_recovery_recheck_state;

    if (!recovery || !solver || !root || final_index >= recovery->pool_count) {
        return false;
    }
    while (current != RECOVERY_INVALID_NODE) {
        if (chain_count >= RECOVERY_SEARCH_CAPACITY) return false;
        g_recovery_chain[chain_count++] = current;
        current = g_recovery_recheck_nodes[current].parent;
    }
    if (chain_count == 0u || g_recovery_chain[chain_count - 1u] != 0u) {
        return false;
    }
    if (!recovery_recheck_node_to_replay_state(
            recovery, root, &g_recovery_recheck_nodes[0], state)) {
        return false;
    }

    recovery->path_len = 0;
    for (int i = (int)chain_count - 2; i >= 0; i--) {
        const RecheckNode* child = &g_recovery_recheck_nodes[g_recovery_chain[i]];
        const RecheckNode* parent = &g_recovery_recheck_nodes[child->parent];

        if (child->direction != RECOVERY_INVALID_SLOT) {
            Position virtual_box;

            if (parent->next_target >= (uint8_t)solver->num_targets) {
                return false;
            }
            virtual_box = solver->targets[parent->next_target].pos;
            if (!recovery_append_recheck_push(
                    recovery,
                    solver,
                    state,
                    virtual_box,
                    child->direction)) {
                return false;
            }
        }
        if (!pos_equal(state->player, child->player) || recovery->path_len != child->cost) {
            return false;
        }
    }
    return true;
}

static SokobanRecoveryResult recovery_plan_recheck(SokobanRecovery* recovery, SokobanSolver* solver) {
    RecoveryNode root;
    int final_index;

    if (!recovery || !solver) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    recovery_node_init_from_solver(solver, &root);
    if (!recovery_capture_recheck_snapshot(recovery, solver)) {
        return recovery_finish_with_error(recovery);
    }
    final_index = recovery_search_recheck(recovery, solver, &root);
    if (final_index < 0) {
        return recovery_finish_with_error(recovery);
    }

    if (!recovery_reconstruct_recheck_path(
            recovery, solver, &root, (uint16_t)final_index
        ) ||
        !recovery_capture_unresolved_recheck_targets(recovery, solver, (uint16_t)final_index)) {
        return recovery_finish_with_error(recovery);
    }
    return recovery_finish_progress_segment(
        recovery,
        solver,
        &g_recovery_recheck_state,
        root.targets,
        g_recovery_recheck_nodes[final_index].pushes > 0u,
        false,
        RECOVERY_PHASE_WAIT_SECOND_OBSERVATION
    );
}

static void recovery_block_unresolved_suspect_targets(
    SokobanRecovery* recovery,
    SokobanSolver* solver
) {
    uint16_t blocked[MAP_ROWS];
    uint8_t write_index = 0u;

    if (!recovery || !solver) return;
    for (int y = 0; y < MAP_ROWS; y++) {
        blocked[y] = (uint16_t)(
            recovery->unresolved_suspect_targets[y] & solver->bmap.targets[y]
        );
        recovery->unresolved_suspect_targets[y] = blocked[y];
        solver->bmap.targets[y] &= (uint16_t)~blocked[y];
        solver->bmap.boxes[y] |= blocked[y];
    }

    for (uint8_t target = 0u; target < solver->num_targets; target++) {
        Entity entity = solver->targets[target];

        if (!entity.is_active || !recovery_position_valid(entity.pos)) continue;
        if (get_bit(blocked, entity.pos.x, entity.pos.y)) continue;
        solver->targets[write_index++] = entity;
    }
    for (uint8_t target = write_index; target < MAX_TARGETS; target++) {
        solver->targets[target] = (Entity){recovery_invalid_position(), -1, false};
    }
    solver->num_targets = write_index;
    recovery_refresh_layers(&solver->bmap);
}

static int recovery_position_compare(Position lhs, Position rhs) {
    if (lhs.y != rhs.y) return lhs.y < rhs.y ? -1 : 1;
    if (lhs.x != rhs.x) return lhs.x < rhs.x ? -1 : 1;
    return 0;
}

static void recovery_build_identification_order(SokobanRecovery* recovery, const SokobanSolver* solver) {
    bool chosen_targets[MAX_TARGETS] = {false};
    bool chosen_boxes[MAX_BOXES] = {false};

    if (!recovery || !solver) return;
    recovery->recognition_count = 0;
    for (int group = 0; group < 2; group++) {
        int limit = group == 0 ? solver->num_targets : solver->num_boxes;
        for (int count = 0; count < limit; count++) {
            int selected = -1;
            for (int slot = 0; slot < limit; slot++) {
                bool used = group == 0 ? chosen_targets[slot] : chosen_boxes[slot];
                Position pos = group == 0 ? solver->targets[slot].pos : solver->boxes[slot].pos;
                Position selected_pos;

                if (used) continue;
                if (selected < 0) {
                    selected = slot;
                    continue;
                }
                selected_pos = group == 0 ? solver->targets[selected].pos : solver->boxes[selected].pos;
                if (recovery_position_compare(pos, selected_pos) < 0) selected = slot;
            }
            if (selected < 0) break;
            recovery->recognition_kind[recovery->recognition_count] =
                group == 0 ? RECOVERY_IDENTIFY_TARGET : RECOVERY_IDENTIFY_BOX;
            recovery->recognition_slot[recovery->recognition_count] = (uint8_t)selected;
            recovery->recognition_count++;
            if (group == 0) chosen_targets[selected] = true;
            else chosen_boxes[selected] = true;
        }
    }
}

static bool recovery_decode_scan_waypoint_tag(
    int8_t tag,
    uint8_t* out_kind,
    uint8_t* out_slot
) {
    int value = (int)tag;
    int slot;

    if (!out_kind || !out_slot) return false;
    if (value <= SCAN_WAYPOINT_BOX_TAG_BASE &&
        value > SCAN_WAYPOINT_BOX_TAG_BASE - MAX_BOXES) {
        slot = SCAN_WAYPOINT_BOX_TAG_BASE - value;
        if (slot >= 0 && slot < MAX_BOXES) {
            *out_kind = RECOVERY_IDENTIFY_BOX;
            *out_slot = (uint8_t)slot;
            return true;
        }
    }
    if (value <= SCAN_WAYPOINT_TARGET_TAG_BASE &&
        value > SCAN_WAYPOINT_TARGET_TAG_BASE - MAX_TARGETS) {
        slot = SCAN_WAYPOINT_TARGET_TAG_BASE - value;
        if (slot >= 0 && slot < MAX_TARGETS) {
            *out_kind = RECOVERY_IDENTIFY_TARGET;
            *out_slot = (uint8_t)slot;
            return true;
        }
    }
    return false;
}

static bool recovery_copy_scan_route(
    SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    int waypoint_count;
    bool has_target_waypoint = false;
    bool has_target_anchor = false;

    if (!recovery || !solver) return false;
    waypoint_count = solver->scan_waypoint_count;
    if (waypoint_count <= 0 || waypoint_count > MAX_BOXES + MAX_TARGETS) return false;

    /* Reject a retained route from a previous map.  A valid route normally
       keeps at least one target waypoint on its target coordinate; detour
       waypoints are still accepted when another target anchors the snapshot. */
    for (int i = 0; i < waypoint_count; i++) {
        uint8_t kind;
        uint8_t slot;
        if (!recovery_decode_scan_waypoint_tag(solver->scan_waypoints[i].id, &kind, &slot) ||
            kind != RECOVERY_IDENTIFY_TARGET) {
            continue;
        }
        has_target_waypoint = true;
        if (slot < solver->num_targets &&
            pos_equal(solver->scan_waypoints[i].pos, solver->targets[slot].pos)) {
            has_target_anchor = true;
        }
    }
    if (has_target_waypoint && !has_target_anchor) return false;

    recovery->scan_route_valid = false;
    recovery->scan_route_count = (uint8_t)waypoint_count;
    recovery->scan_route_index = 0u;
    recovery->scan_route_path_len = 0u;
    for (int i = 0; i < MAX_BOXES + MAX_TARGETS; i++) {
        recovery->scan_route_kind[i] = RECOVERY_INVALID_SLOT;
        recovery->scan_route_source_slot[i] = RECOVERY_INVALID_SLOT;
        recovery->scan_route_bound_slot[i] = RECOVERY_INVALID_SLOT;
        recovery->scan_route_source_positions[i] = recovery_invalid_position();
    }
    memcpy(
        recovery->scan_route_waypoints,
        solver->scan_waypoints,
        (size_t)waypoint_count * sizeof(Entity)
    );
    memcpy(
        recovery->scan_route_pauses,
        solver->scan_player_pause_positions,
        (size_t)waypoint_count * sizeof(Position)
    );
    if (solver->best_path && solver->best_path_len <= MAX_PATH_LENGTH) {
        recovery->scan_route_path_len = solver->best_path_len;
        if (solver->best_path_len > 0u) {
            memcpy(
                recovery->scan_route_path,
                solver->best_path,
                (size_t)solver->best_path_len * sizeof(Direction)
            );
        }
    }

    for (int i = 0; i < waypoint_count; i++) {
        uint8_t kind;
        uint8_t slot;

        if (recovery_decode_scan_waypoint_tag(
                recovery->scan_route_waypoints[i].id, &kind, &slot)) {
            recovery->scan_route_kind[i] = kind;
            recovery->scan_route_source_slot[i] = slot;
            if (kind == RECOVERY_IDENTIFY_TARGET && slot < solver->num_targets) {
                recovery->scan_route_source_positions[i] = solver->targets[slot].pos;
            } else if (kind == RECOVERY_IDENTIFY_BOX && slot < solver->num_boxes) {
                recovery->scan_route_source_positions[i] = solver->boxes[slot].pos;
            }
            continue;
        }

        for (int target = 0; target < solver->num_targets && target < MAX_TARGETS; target++) {
            if (pos_equal(recovery->scan_route_waypoints[i].pos, solver->targets[target].pos)) {
                recovery->scan_route_kind[i] = RECOVERY_IDENTIFY_TARGET;
                recovery->scan_route_source_slot[i] = (uint8_t)target;
                recovery->scan_route_source_positions[i] = solver->targets[target].pos;
                break;
            }
        }
        if (recovery->scan_route_kind[i] != RECOVERY_INVALID_SLOT) continue;
        for (int box = 0; box < solver->num_boxes && box < MAX_BOXES; box++) {
            if (pos_equal(recovery->scan_route_waypoints[i].pos, solver->boxes[box].pos)) {
                recovery->scan_route_kind[i] = RECOVERY_IDENTIFY_BOX;
                recovery->scan_route_source_slot[i] = (uint8_t)box;
                recovery->scan_route_source_positions[i] = solver->boxes[box].pos;
                break;
            }
        }
    }

    recovery->scan_route_valid = false;
    for (int i = 0; i < waypoint_count; i++) {
        if (recovery->scan_route_kind[i] != RECOVERY_INVALID_SLOT) {
            recovery->scan_route_valid = true;
            break;
        }
    }
    return recovery->scan_route_valid;
}

static void recovery_restore_scan_source_positions(
    SokobanRecovery* recovery,
    const SokobanSolver* source_solver
) {
    if (!recovery || !source_solver) return;
    for (int i = 0; i < recovery->scan_route_count; i++) {
        uint8_t kind = recovery->scan_route_kind[i];
        uint8_t slot = recovery->scan_route_source_slot[i];

        if (kind == RECOVERY_IDENTIFY_TARGET && slot < source_solver->num_targets) {
            recovery->scan_route_source_positions[i] = source_solver->targets[slot].pos;
        } else if (kind == RECOVERY_IDENTIFY_BOX && slot < source_solver->num_boxes) {
            recovery->scan_route_source_positions[i] = source_solver->boxes[slot].pos;
        }
    }
}

static bool recovery_capture_scan_route(
    SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    SokobanSolver scan_solver;

    if (!recovery || !solver) return false;

    /* A live solver may still retain the original waypoint list after playback.
       A zero-length path normally means load_map() was reused after a residual
       session, so do not mistake its stale waypoint array for this map. */
    if (solver->best_path_len > 0u && recovery_copy_scan_route(recovery, solver)) return true;

    /* The PC demo creates a fresh solver for recovery, so rebuild the route in a
       private solver copy.  This keeps the caller's root state untouched. */
    scan_solver = *solver;
    scan_solver.best_path = g_recovery_scan_generation_path;
    scan_solver.best_path_len = 0u;
    scan_solver.best_steps = 0xFFFFu;
    scan_solver.is_scanning = false;
    scan_solver.scan_waypoint_count = 0;
    scan_solver.scan_current_index = 0;
    if (!solver_generate_scan_path(&scan_solver)) return false;
    if (!recovery_copy_scan_route(recovery, &scan_solver)) return false;
    recovery_restore_scan_source_positions(recovery, solver);
    return true;
}

static void recovery_prepare_scan_route_bindings(
    SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    bool used_boxes[MAX_BOXES] = {false};
    bool used_targets[MAX_TARGETS] = {false};
    uint8_t exact_target_matches = 0u;

    if (!recovery || !solver || !recovery->scan_route_valid) return;
    for (int i = 0; i < recovery->scan_route_count; i++) {
        uint8_t kind = recovery->scan_route_kind[i];
        Position waypoint = recovery->scan_route_source_positions[i];
        if (!recovery_position_valid(waypoint)) waypoint = recovery->scan_route_waypoints[i].pos;

        if (kind == RECOVERY_IDENTIFY_TARGET) {
            for (int slot = 0; slot < solver->num_targets && slot < MAX_TARGETS; slot++) {
                if (used_targets[slot] || !pos_equal(solver->targets[slot].pos, waypoint)) continue;
                recovery->scan_route_bound_slot[i] = (uint8_t)slot;
                used_targets[slot] = true;
                exact_target_matches++;
                break;
            }
        } else if (kind == RECOVERY_IDENTIFY_BOX) {
            for (int slot = 0; slot < solver->num_boxes && slot < MAX_BOXES; slot++) {
                if (used_boxes[slot] || !pos_equal(solver->boxes[slot].pos, waypoint)) continue;
                recovery->scan_route_bound_slot[i] = (uint8_t)slot;
                used_boxes[slot] = true;
                break;
            }
        }
    }

    /* Without a surviving target anchor this snapshot belongs to another map
       (or only contains stale waypoints), so use the conservative fallback. */
    if (exact_target_matches == 0u) {
        recovery->scan_route_valid = false;
        return;
    }

    /* A box can be physically nudged after the original scan.  Bind each
       unmatched scan box to the nearest still-unbound residual box. */
    for (int i = 0; i < recovery->scan_route_count; i++) {
        int best_slot = -1;
        int best_distance = 0x7FFF;
        Position waypoint = recovery->scan_route_source_positions[i];

        if (!recovery_position_valid(waypoint)) waypoint = recovery->scan_route_waypoints[i].pos;

        if (recovery->scan_route_kind[i] != RECOVERY_IDENTIFY_BOX ||
            recovery->scan_route_bound_slot[i] != RECOVERY_INVALID_SLOT) {
            continue;
        }
        for (int slot = 0; slot < solver->num_boxes && slot < MAX_BOXES; slot++) {
            int distance;
            if (used_boxes[slot]) continue;
            distance = abs((int)solver->boxes[slot].pos.x - (int)waypoint.x) +
                abs((int)solver->boxes[slot].pos.y - (int)waypoint.y);
            if (best_slot < 0 || distance < best_distance) {
                best_slot = slot;
                best_distance = distance;
            }
        }
        if (best_slot >= 0) {
            recovery->scan_route_bound_slot[i] = (uint8_t)best_slot;
            used_boxes[best_slot] = true;
        }
    }
}

/* Recovery keeps its own read-only scan route so the normal scan state remains untouched. */
static uint16_t recovery_scan_turn_score(uint8_t from_heading, uint8_t to_heading) {
    return (uint16_t)(direction_quarter_turns(from_heading, to_heading) * RECOVERY_SCAN_TURN_SCORE);
}

static bool recovery_scan_route_is_better(
    uint16_t candidate_len,
    uint16_t candidate_score,
    uint16_t best_len,
    uint16_t best_score
) {
    if (best_score == 0xFFFFu) return true;
    if (candidate_score < best_score) return true;
    if (candidate_score > best_score) return false;
    return candidate_len < best_len;
}

static uint16_t recovery_scan_path_bend_score(const Direction* path, uint16_t path_len) {
    return (uint16_t)(path_direction_bend_count(path, path_len) * RECOVERY_SCAN_BEND_SCORE);
}

static uint16_t recovery_scan_path_score(
    const Direction* path,
    uint16_t path_len,
    uint8_t preferred_heading
) {
    uint32_t score = (uint32_t)path_len * RECOVERY_SCAN_STEP_SCORE;

    if (path_len > 0u) {
        score += recovery_scan_turn_score(
            preferred_heading,
            path_first_direction_index(path, path_len, preferred_heading)
        );
        score += recovery_scan_path_bend_score(path, path_len);
    }
    return score > 0xFFFEu ? 0xFFFEu : (uint16_t)score;
}

static bool recovery_scan_build_field(const BitboardMap* map, Position start) {
    uint16_t head = 0u;
    uint16_t tail = 0u;
    uint8_t start_index;

    if (!map || !recovery_position_valid(start) || recovery_return_tile_blocked(map, start)) return false;
    start_index = (uint8_t)((uint16_t)start.y * MAP_COLS + start.x);
    memset(g_recovery_return_parent, RECOVERY_INVALID_SLOT, sizeof(g_recovery_return_parent));
    memset(g_recovery_scan_distance, 0xFF, sizeof(g_recovery_scan_distance));
    g_recovery_return_parent[start_index] = start_index;
    g_recovery_scan_distance[start_index] = 0u;
    g_recovery_return_queue[tail++] = start_index;

    while (head < tail) {
        uint8_t current_index = g_recovery_return_queue[head++];
        Position current_pos = {
            (uint8_t)(current_index % MAP_COLS),
            (uint8_t)(current_index / MAP_COLS)
        };

        for (uint8_t direction = 0; direction < 4u; direction++) {
            Position next_pos;
            uint8_t next_index;

            if (!recovery_offset(current_pos, direction_from_index(direction), 1, &next_pos) ||
                recovery_return_tile_blocked(map, next_pos)) {
                continue;
            }
            next_index = (uint8_t)((uint16_t)next_pos.y * MAP_COLS + next_pos.x);
            if (g_recovery_scan_distance[next_index] != 0xFFu) continue;
            g_recovery_return_parent[next_index] = current_index;
            g_recovery_return_direction[next_index] = direction;
            g_recovery_scan_distance[next_index] = (uint8_t)(g_recovery_scan_distance[current_index] + 1u);
            g_recovery_return_queue[tail++] = next_index;
        }
    }
    return true;
}

static bool recovery_scan_reconstruct_basic_path(
    Position start,
    Position target,
    Direction* out_path,
    uint16_t out_capacity,
    uint16_t* out_len
) {
    uint8_t start_index;
    uint8_t current_index;
    uint16_t path_len;

    if (!out_path || !out_len || !recovery_position_valid(start) ||
        !recovery_position_valid(target)) {
        return false;
    }
    start_index = (uint8_t)((uint16_t)start.y * MAP_COLS + start.x);
    current_index = (uint8_t)((uint16_t)target.y * MAP_COLS + target.x);
    if (g_recovery_scan_distance[current_index] == 0xFFu) return false;
    path_len = g_recovery_scan_distance[current_index];
    if (path_len >= out_capacity) return false;

    for (int index = (int)path_len - 1; index >= 0; index--) {
        uint8_t direction = g_recovery_return_direction[current_index];
        uint8_t parent = g_recovery_return_parent[current_index];

        if (direction >= 4u || parent == RECOVERY_INVALID_SLOT) return false;
        out_path[index] = direction_from_index(direction);
        current_index = parent;
    }
    if (current_index != start_index) return false;
    *out_len = path_len;
    return true;
}

static bool recovery_scan_reconstruct_field_path(
    Position start,
    Position target,
    uint8_t preferred_heading,
    uint8_t scan_heading,
    bool lock_scan_heading,
    Direction* out_path,
    uint16_t out_capacity,
    uint16_t* out_len
) {
    uint8_t start_index;
    uint8_t current_index;
    uint16_t path_len;
    int best_heading = -1;
    uint16_t best_score = 0xFFFFu;

    if (!out_path || !out_len || !recovery_position_valid(start) || !recovery_position_valid(target)) return false;
    start_index = (uint8_t)((uint16_t)start.y * MAP_COLS + start.x);
    current_index = (uint8_t)((uint16_t)target.y * MAP_COLS + target.x);
    if (g_recovery_scan_distance[current_index] == 0xFFu) return false;
    path_len = g_recovery_scan_distance[current_index];
    if (path_len >= out_capacity) return false;
    if (path_len == 0u) {
        *out_len = 0u;
        return true;
    }

    memset(g_recovery_scan_turn_cost, 0xFF, sizeof(g_recovery_scan_turn_cost));
    memset(g_recovery_scan_turn_parent, 0xFF, sizeof(g_recovery_scan_turn_parent));

    for (uint16_t step = 1u; step <= path_len; step++) {
        for (uint16_t index = 0u; index < RECOVERY_TILE_CAPACITY; index++) {
            uint8_t x;
            uint8_t y;

            if (g_recovery_scan_distance[index] != step) continue;
            x = (uint8_t)(index % MAP_COLS);
            y = (uint8_t)(index / MAP_COLS);
            for (uint8_t direction = 0u; direction < 4u; direction++) {
                int previous_x = (int)x - direction_from_index(direction).dx;
                int previous_y = (int)y - direction_from_index(direction).dy;
                uint8_t previous_index;

                if (!is_in_bounds(previous_x, previous_y)) continue;
                previous_index = (uint8_t)(previous_y * MAP_COLS + previous_x);
                if (g_recovery_scan_distance[previous_index] != (uint8_t)(step - 1u)) continue;

                if (step == 1u) {
                    g_recovery_scan_turn_cost[direction][index] =
                        recovery_scan_turn_score(preferred_heading, direction);
                    g_recovery_scan_turn_parent[direction][index] = 4u;
                    continue;
                }

                for (uint8_t previous_heading = 0u; previous_heading < 4u; previous_heading++) {
                    uint16_t previous_cost = g_recovery_scan_turn_cost[previous_heading][previous_index];
                    uint32_t candidate_cost;

                    if (previous_cost == 0xFFFFu) continue;
                    candidate_cost = (uint32_t)previous_cost;
                    if (previous_heading != direction) candidate_cost += RECOVERY_SCAN_BEND_SCORE;
                    if (candidate_cost >= g_recovery_scan_turn_cost[direction][index]) continue;
                    g_recovery_scan_turn_cost[direction][index] =
                        candidate_cost > 0xFFFEu ? 0xFFFEu : (uint16_t)candidate_cost;
                    g_recovery_scan_turn_parent[direction][index] = previous_heading;
                }
            }
        }
    }

    for (uint8_t direction = 0u; direction < 4u; direction++) {
        uint16_t route_score = g_recovery_scan_turn_cost[direction][current_index];
        uint32_t score;

        if (lock_scan_heading && direction != scan_heading) continue;
        if (route_score == 0xFFFFu) continue;
        score = (uint32_t)route_score;
        if (!lock_scan_heading) score += recovery_scan_turn_score(direction, scan_heading);
        if (score >= best_score) continue;
        best_score = score > 0xFFFEu ? 0xFFFEu : (uint16_t)score;
        best_heading = direction;
    }
    if (best_heading < 0) return false;

    for (int index = (int)path_len - 1; index >= 0; index--) {
        uint8_t parent_heading;
        int previous_x;
        int previous_y;

        if (best_heading >= 4) return false;
        out_path[index] = direction_from_index((uint8_t)best_heading);
        previous_x = (int)(current_index % MAP_COLS) - out_path[index].dx;
        previous_y = (int)(current_index / MAP_COLS) - out_path[index].dy;
        if (!is_in_bounds(previous_x, previous_y)) return false;
        parent_heading = g_recovery_scan_turn_parent[best_heading][current_index];
        current_index = (uint8_t)(previous_y * MAP_COLS + previous_x);
        if (parent_heading == 4u) {
            if (index != 0 || current_index != start_index) return false;
            best_heading = -1;
        } else {
            if (parent_heading >= 4u) return false;
            best_heading = parent_heading;
        }
    }
    *out_len = path_len;
    return true;
}

static bool recovery_scan_plan_entity(
    const BitboardMap* map,
    Position start,
    uint8_t preferred_heading,
    Position entity_pos,
    uint8_t allowed_directions,
    Direction* out_path,
    uint16_t out_path_capacity,
    RecoveryScanRoute* out_route
) {
    RecoveryScanRoute best_route;
    bool found = false;

    if (!map || !out_path || !out_route || !recovery_scan_build_field(map, start)) return false;
    memset(&best_route, 0, sizeof(best_route));
    best_route.score = 0xFFFFu;
    for (uint8_t direction = 0; direction < 4u; direction++) {
        Position observation_pos;
        uint16_t path_len;
        uint16_t score;
        uint8_t end_heading;
        RecoveryScanRoute candidate;

        if ((allowed_directions & (uint8_t)(1u << direction)) == 0u ||
            !recovery_offset(entity_pos, direction_from_index(direction), -1, &observation_pos) ||
            recovery_return_tile_blocked(map, observation_pos) ||
            !recovery_scan_reconstruct_basic_path(
                start,
                observation_pos,
                g_recovery_scan_path,
                MAX_PATH_LENGTH,
                &path_len)) {
            continue;
        }

        end_heading = path_end_direction_index(g_recovery_scan_path, path_len, preferred_heading);
        score = recovery_scan_path_score(g_recovery_scan_path, path_len, preferred_heading);

        candidate.view_direction = direction;
        candidate.end_heading = end_heading;
        candidate.observation_pos = observation_pos;
        candidate.path_len = path_len;
        candidate.score = score;
        if (!recovery_scan_route_is_better(
                candidate.path_len,
                candidate.score,
                best_route.path_len,
                best_route.score)) {
            continue;
        }
        if (path_len > out_path_capacity) continue;
        if (path_len > 0u) memcpy(g_recovery_scan_best_path, g_recovery_scan_path, path_len * sizeof(Direction));
        best_route = candidate;
        found = true;
    }
    if (!found) return false;

    if (best_route.path_len > 0u) {
        uint16_t turn_len = 0u;
        uint16_t old_bends = recovery_scan_path_bend_score(
            g_recovery_scan_best_path, best_route.path_len
        );

        if (recovery_scan_reconstruct_field_path(
                start,
                best_route.observation_pos,
                preferred_heading,
                best_route.end_heading,
                true,
                g_recovery_scan_path,
                MAX_PATH_LENGTH,
                &turn_len
            ) && turn_len == best_route.path_len &&
            recovery_scan_path_bend_score(g_recovery_scan_path, turn_len) < old_bends) {
            memcpy(
                g_recovery_scan_best_path,
                g_recovery_scan_path,
                (size_t)turn_len * sizeof(Direction)
            );
        }
    }
    if (best_route.path_len > 0u) {
        memcpy(out_path, g_recovery_scan_best_path, best_route.path_len * sizeof(Direction));
    }
    *out_route = best_route;
    return true;
}

static bool recovery_recheck_scan_has_pending(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    if (!recovery || !solver) return false;
    for (uint8_t slot = 0; slot < solver->num_targets && slot < MAX_TARGETS; slot++) {
        if (!recovery->rescan_target_done[slot]) return true;
    }
    for (uint8_t slot = 0; slot < solver->num_boxes && slot < MAX_BOXES; slot++) {
        if (!recovery->rescan_box_done[slot]) return true;
    }
    return false;
}

static bool recovery_recheck_scan_select_first_pending(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    uint8_t* out_kind,
    uint8_t* out_slot
) {
    if (!recovery || !solver || !out_kind || !out_slot) return false;
    for (uint8_t slot = 0; slot < solver->num_targets && slot < MAX_TARGETS; slot++) {
        if (recovery->rescan_target_done[slot]) continue;
        *out_kind = RECOVERY_IDENTIFY_TARGET;
        *out_slot = slot;
        return true;
    }
    for (uint8_t slot = 0; slot < solver->num_boxes && slot < MAX_BOXES; slot++) {
        if (recovery->rescan_box_done[slot]) continue;
        *out_kind = RECOVERY_IDENTIFY_BOX;
        *out_slot = slot;
        return true;
    }
    return false;
}

static bool recovery_scan_line_clear_between(
    const BitboardMap* map,
    Position from,
    Position to
) {
    if (!map) return false;
    if (from.x == to.x) {
        int step = to.y > from.y ? 1 : -1;
        for (int y = (int)from.y + step; y != (int)to.y; y += step) {
            if (recovery_return_tile_blocked(map, (Position){from.x, (uint8_t)y})) return false;
        }
        return true;
    }
    if (from.y == to.y) {
        int step = to.x > from.x ? 1 : -1;
        for (int x = (int)from.x + step; x != (int)to.x; x += step) {
            if (recovery_return_tile_blocked(map, (Position){(uint8_t)x, from.y})) return false;
        }
        return true;
    }
    return false;
}

static uint16_t recovery_scan_followup_entity_score(
    const BitboardMap* map,
    Position view_pos,
    uint8_t heading_after_scan,
    Position entity_pos
) {
    uint16_t best = 0xFFFFu;

    for (uint8_t direction = 0u; direction < 4u; direction++) {
        Position next_view;
        uint8_t next_heading;
        uint32_t score;
        uint8_t travel_heading;

        if (!recovery_offset(entity_pos, direction_from_index(direction), -1, &next_view) ||
            recovery_return_tile_blocked(map, next_view)) {
            continue;
        }
        next_heading = direction_index_between(next_view, entity_pos);
        if (pos_equal(view_pos, next_view)) {
            score = recovery_scan_turn_score(heading_after_scan, next_heading);
        } else {
            score = (uint32_t)manhattan_distance(view_pos, next_view) * RECOVERY_SCAN_STEP_SCORE;
            travel_heading = direction_axis_index(view_pos, next_view);
            if (travel_heading < 4u) {
                if (!recovery_scan_line_clear_between(map, view_pos, next_view)) {
                    score += RECOVERY_SCAN_VIEW_BLOCKED_LINE_PENALTY;
                }
                score += recovery_scan_turn_score(heading_after_scan, travel_heading);
                score += recovery_scan_turn_score(travel_heading, next_heading);
            } else {
                score += RECOVERY_SCAN_VIEW_CORNER_PENALTY;
            }
        }
        if (score < best) best = score > 0xFFFEu ? 0xFFFEu : (uint16_t)score;
    }
    return best;
}

static uint16_t recovery_recheck_scan_followup_score(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    const RecoveryScanRoute* current
) {
    uint16_t best_score = 0xFFFFu;

    if (!recovery || !solver || !current) return best_score;
    for (uint8_t slot = 0u; slot < solver->num_boxes && slot < MAX_BOXES; slot++) {
        uint16_t score;

        if (recovery->rescan_box_done[slot] ||
            (current->kind == RECOVERY_IDENTIFY_BOX && current->slot == slot)) {
            continue;
        }
        score = recovery_scan_followup_entity_score(
            &solver->bmap,
            current->observation_pos,
            current->end_heading,
            solver->boxes[slot].pos
        );
        if (score < best_score) best_score = score;
    }
    for (uint8_t slot = 0u; slot < solver->num_targets && slot < MAX_TARGETS; slot++) {
        uint16_t score;

        if (recovery->rescan_target_done[slot] ||
            (current->kind == RECOVERY_IDENTIFY_TARGET && current->slot == slot)) {
            continue;
        }
        score = recovery_scan_followup_entity_score(
            &solver->bmap,
            current->observation_pos,
            current->end_heading,
            solver->targets[slot].pos
        );
        if (score < best_score) best_score = score;
    }
    return best_score;
}

static bool recovery_recheck_scan_plan_snapshot_entity(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    uint8_t route_index,
    Position entity_pos,
    Direction* out_path,
    RecoveryScanRoute* out_route
) {
    Position pause;
    Direction view_direction;
    uint8_t direction_index_value;

    if (!recovery || !solver || !out_path || !out_route ||
        route_index >= recovery->scan_route_count) {
        return false;
    }

    /* Preserve the original observation side whenever the entity stayed put. */
    pause = recovery->scan_route_pauses[route_index];
    if (recovery_position_valid(pause) &&
        abs((int)entity_pos.x - (int)pause.x) + abs((int)entity_pos.y - (int)pause.y) == 1 &&
        !recovery_return_tile_blocked(&solver->bmap, pause)) {
        view_direction = (Direction){
            (int8_t)((int)entity_pos.x - (int)pause.x),
            (int8_t)((int)entity_pos.y - (int)pause.y)
        };
        direction_index_value = direction_index(view_direction);
        if (direction_index_value < 4u &&
            recovery_scan_plan_entity(
                &solver->bmap,
                recovery->recognition_player,
                recovery->recognition_heading,
                entity_pos,
                (uint8_t)(1u << direction_index_value),
                out_path,
                MAX_PATH_LENGTH,
                out_route
            )) {
            return true;
        }
    }

    return recovery_scan_plan_entity(
        &solver->bmap,
        recovery->recognition_player,
        recovery->recognition_heading,
        entity_pos,
        0x0Fu,
        out_path,
        MAX_PATH_LENGTH,
        out_route
    );
}

static int recovery_recheck_scan_box_route_owner(
    const SokobanRecovery* recovery,
    uint8_t route_index,
    uint8_t box_slot
) {
    if (!recovery) return -1;
    for (uint8_t index = 0u; index < recovery->scan_route_count; index++) {
        if (index == route_index || recovery->scan_route_kind[index] != RECOVERY_IDENTIFY_BOX) continue;
        if (recovery->scan_route_bound_slot[index] == box_slot) return (int)index;
    }
    return -1;
}

/* Replan each residual leg through the normal mixed scan route builder. */
static bool recovery_recheck_scan_plan_dynamic(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    RecoveryScanRoute* out_route
) {
    bool visited_boxes[MAX_BOXES] = {false};
    bool visited_targets[MAX_TARGETS] = {false};
    Entity waypoints[MAX_BOXES + MAX_TARGETS];
    Position pauses[MAX_BOXES + MAX_TARGETS];
    Direction route_path[MAX_PATH_LENGTH];
    uint16_t route_len = 0u;
    int waypoint_count = 0;
    Position final_pos;
    uint8_t final_heading;
    uint16_t first_segment_len = 0u;
    uint8_t kind;
    uint8_t slot;
    Position entity_pos;
    uint8_t view_direction;

    if (!recovery || !solver || !out_route) return false;
    for (uint8_t index = 0u; index < solver->num_boxes && index < MAX_BOXES; index++) {
        visited_boxes[index] = recovery->rescan_box_done[index] != 0u;
    }
    for (uint8_t index = 0u; index < solver->num_targets && index < MAX_TARGETS; index++) {
        visited_targets[index] = recovery->rescan_target_done[index] != 0u;
    }

    if (!sokoban_plan_rescan_route(
            solver,
            recovery->recognition_player,
            recovery->recognition_heading,
            visited_boxes,
            visited_targets,
            solver->num_boxes,
            solver->num_targets,
            route_path,
            &route_len,
            waypoints,
            pauses,
            &waypoint_count,
            &final_pos,
            &final_heading
        )) {
        return false;
    }
    (void)final_pos;
    (void)final_heading;
    if (waypoint_count <= 0 || !recovery_decode_scan_waypoint_tag(
            waypoints[0].id, &kind, &slot)) {
        return false;
    }

    for (uint16_t index = 0u; index < route_len; index++) {
        if (direction_is_pause(route_path[index])) break;
        first_segment_len++;
    }
    if (first_segment_len >= route_len ||
        (kind == RECOVERY_IDENTIFY_BOX &&
         (slot >= solver->num_boxes || recovery->rescan_box_done[slot])) ||
        (kind == RECOVERY_IDENTIFY_TARGET &&
         (slot >= solver->num_targets || recovery->rescan_target_done[slot]))) {
        return false;
    }

    entity_pos = kind == RECOVERY_IDENTIFY_BOX
        ? solver->boxes[slot].pos
        : solver->targets[slot].pos;
    view_direction = direction_index_between(pauses[0], entity_pos);
    if (view_direction >= 4u) return false;
    if (first_segment_len > 0u) {
        memcpy(recovery->path, route_path, (size_t)first_segment_len * sizeof(Direction));
    }

    memset(out_route, 0, sizeof(*out_route));
    out_route->kind = kind;
    out_route->slot = slot;
    out_route->view_direction = view_direction;
    out_route->observation_pos = pauses[0];
    out_route->path_len = first_segment_len;
    out_route->end_heading = path_end_direction_index(
        recovery->path, first_segment_len, recovery->recognition_heading
    );
    out_route->score = recovery_scan_path_score(
        recovery->path, first_segment_len, recovery->recognition_heading
    );
    return true;
}

static bool recovery_recheck_scan_plan_next(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    RecoveryScanRoute* out_route
) {
    RecoveryScanRoute best_route;
    bool scan_targets = false;
    bool found = false;
    uint32_t best_total_score = 0xFFFFFFFFu;

    if (!recovery || !solver || !out_route) return false;

    /* Prefer the main scan planner on the current residual map.  The original
       route snapshot remains a fallback when dynamic planning cannot proceed. */
    if (recovery->use_dynamic_rescan_route) {
        if (recovery_recheck_scan_plan_dynamic(recovery, solver, out_route)) return true;
    }

    if (recovery->scan_route_valid) {
        while (recovery->scan_route_index < recovery->scan_route_count) {
            uint8_t route_index = recovery->scan_route_index++;
            uint8_t kind = recovery->scan_route_kind[route_index];
            uint8_t slot = recovery->scan_route_bound_slot[route_index];
            Position entity_pos;
            RecoveryScanRoute candidate;
            bool candidate_found = false;

            if (kind == RECOVERY_IDENTIFY_TARGET) {
                if (slot >= solver->num_targets || slot >= MAX_TARGETS ||
                    recovery->rescan_target_done[slot]) {
                    continue;
                }
                entity_pos = solver->targets[slot].pos;
            } else if (kind == RECOVERY_IDENTIFY_BOX) {
                if (slot >= solver->num_boxes || slot >= MAX_BOXES ||
                    recovery->rescan_box_done[slot]) {
                    continue;
                }
                entity_pos = solver->boxes[slot].pos;
            } else {
                continue;
            }

            memset(&candidate, 0, sizeof(candidate));
            if (recovery_recheck_scan_plan_snapshot_entity(
                    recovery,
                    solver,
                    route_index,
                    entity_pos,
                    g_recovery_scan_candidate_path,
                    &candidate)) {
                candidate_found = true;
            } else if (kind == RECOVERY_IDENTIFY_BOX) {
                /* A pushed box can be equidistant from two original scan slots.
                   If the nearest binding is currently unreachable, try a later
                   binding and swap it with the old slot.  This preserves the
                   mixed waypoint order without inventing a new entity order. */
                RecoveryScanRoute best_alternate;
                uint8_t best_alternate_slot = RECOVERY_INVALID_SLOT;
                bool alternate_found = false;

                memset(&best_alternate, 0, sizeof(best_alternate));
                best_alternate.score = 0xFFFFu;
                for (uint8_t alternate_slot = 0u;
                     alternate_slot < solver->num_boxes && alternate_slot < MAX_BOXES;
                     alternate_slot++) {
                    RecoveryScanRoute alternate;
                    int owner;

                    if (alternate_slot == slot || recovery->rescan_box_done[alternate_slot]) continue;
                    owner = recovery_recheck_scan_box_route_owner(recovery, route_index, alternate_slot);
                    if (owner >= 0 && owner < (int)route_index) continue;
                    memset(&alternate, 0, sizeof(alternate));
                    if (!recovery_recheck_scan_plan_snapshot_entity(
                            recovery,
                            solver,
                            route_index,
                            solver->boxes[alternate_slot].pos,
                            g_recovery_scan_candidate_path,
                            &alternate)) {
                        continue;
                    }
                    if (!alternate_found || recovery_scan_route_is_better(
                            alternate.path_len,
                            alternate.score,
                            best_alternate.path_len,
                            best_alternate.score)) {
                        if (alternate.path_len > 0u) {
                            memcpy(
                                g_recovery_scan_best_path,
                                g_recovery_scan_candidate_path,
                                (size_t)alternate.path_len * sizeof(Direction)
                            );
                        }
                        best_alternate = alternate;
                        best_alternate_slot = alternate_slot;
                        alternate_found = true;
                    }
                }
                if (alternate_found) {
                    int owner = recovery_recheck_scan_box_route_owner(
                        recovery, route_index, best_alternate_slot
                    );
                    if (owner >= 0) {
                        recovery->scan_route_bound_slot[owner] = slot;
                    }
                    recovery->scan_route_bound_slot[route_index] = best_alternate_slot;
                    slot = best_alternate_slot;
                    candidate = best_alternate;
                    if (candidate.path_len > 0u) {
                        memcpy(
                            g_recovery_scan_candidate_path,
                            g_recovery_scan_best_path,
                            (size_t)candidate.path_len * sizeof(Direction)
                        );
                    }
                    candidate_found = true;
                }
            }
            if (!candidate_found) {
                continue;
            }
            candidate.kind = kind;
            candidate.slot = slot;
            if (candidate.path_len > 0u) {
                memcpy(
                    recovery->path,
                    g_recovery_scan_candidate_path,
                    (size_t)candidate.path_len * sizeof(Direction)
                );
            }
            *out_route = candidate;
            return true;
        }
    }

    memset(&best_route, 0, sizeof(best_route));
    best_route.score = 0xFFFFu;
    for (uint8_t slot = 0; slot < solver->num_targets && slot < MAX_TARGETS; slot++) {
        if (!recovery->rescan_target_done[slot]) {
            scan_targets = true;
            break;
        }
    }

    for (uint8_t slot = 0;
         slot < (scan_targets ? solver->num_targets : solver->num_boxes) &&
         slot < (scan_targets ? MAX_TARGETS : MAX_BOXES);
         slot++) {
        RecoveryScanRoute candidate;
        uint16_t followup_score;
        uint32_t total_score;
        Position entity_pos;

        if (scan_targets ? recovery->rescan_target_done[slot] : recovery->rescan_box_done[slot]) continue;
        entity_pos = scan_targets ? solver->targets[slot].pos : solver->boxes[slot].pos;
        memset(&candidate, 0, sizeof(candidate));
        if (!recovery_scan_plan_entity(
                &solver->bmap,
                recovery->recognition_player,
                recovery->recognition_heading,
                entity_pos,
                0x0Fu,
                g_recovery_scan_candidate_path,
                MAX_PATH_LENGTH,
                &candidate)) {
            continue;
        }
        candidate.kind = scan_targets ? RECOVERY_IDENTIFY_TARGET : RECOVERY_IDENTIFY_BOX;
        candidate.slot = slot;
        followup_score = recovery_recheck_scan_followup_score(recovery, solver, &candidate);
        total_score = (uint32_t)candidate.score +
            (followup_score == 0xFFFFu ? 0u : followup_score);
        if (found && (total_score > best_total_score ||
                      (total_score == best_total_score && !recovery_scan_route_is_better(
                          candidate.path_len,
                          candidate.score,
                          best_route.path_len,
                          best_route.score)))) {
            continue;
        }
        if (candidate.path_len > 0u) {
            memcpy(recovery->path, g_recovery_scan_candidate_path, candidate.path_len * sizeof(Direction));
        }
        best_route = candidate;
        best_total_score = total_score;
        found = true;
    }

    if (!found) return false;
    *out_route = best_route;
    return true;
}

static bool recovery_recheck_scan_plan_pending_direction(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    uint8_t direction,
    RecoveryScanRoute* out_route
) {
    Position entity_position;

    if (!recovery || !solver || !out_route || direction >= 4u || !recovery->pending_id) return false;
    if (recovery->pending_kind == RECOVERY_IDENTIFY_TARGET) {
        if (recovery->pending_slot >= solver->num_targets) return false;
        entity_position = solver->targets[recovery->pending_slot].pos;
    } else if (recovery->pending_kind == RECOVERY_IDENTIFY_BOX) {
        if (recovery->pending_slot >= solver->num_boxes) return false;
        entity_position = solver->boxes[recovery->pending_slot].pos;
    } else {
        return false;
    }

    if (!recovery_scan_plan_entity(
            &solver->bmap,
            recovery->recognition_player,
            recovery->recognition_heading,
            entity_position,
            (uint8_t)(1u << direction),
            recovery->path,
            MAX_PATH_LENGTH,
            out_route)) {
        return false;
    }
    out_route->kind = recovery->pending_kind;
    out_route->slot = recovery->pending_slot;
    return true;
}

static bool recovery_recheck_scan_accept_route(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const RecoveryScanRoute* route
) {
    RecoveryNode observation_node;
    PathReplayOptions options = path_replay_default_options();

    if (!recovery || !solver || !route || route->path_len > MAX_PATH_LENGTH ||
        route->view_direction >= 4u || !recovery_position_valid(route->observation_pos)) {
        return false;
    }
    recovery_node_init_from_solver(solver, &observation_node);
    observation_node.player = recovery->recognition_player;
    options.mode = PATH_REPLAY_STRICT_VALIDATE;
    if (!recovery_node_to_replay_state(recovery, &observation_node, &g_recovery_replay_state) ||
        !recovery_replay_path(
            solver,
            &g_recovery_replay_state,
            recovery->path,
            route->path_len,
            &options,
            NULL)) {
        return false;
    }
    if (!pos_equal(g_recovery_replay_state.player, route->observation_pos)) {
        return false;
    }

    recovery->recognition_player = route->observation_pos;
    recovery->recognition_heading = route->end_heading;
    recovery->path_len = route->path_len;
    return true;
}

static SokobanRecoveryResult recovery_recheck_scan_need_id(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    Position observation_pos
) {
    SokobanRecoveryResult result;

    if (!recovery || !solver) {
        return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    }
    recovery->phase = RECOVERY_PHASE_WAIT_ID;
    if (recovery->path_len == 0u) {
        result = recovery_result(SOKOBAN_RECOVERY_NEED_ID, SOKOBAN_RECOVERY_NEED_ID);
    } else {
        result = recovery_path_result(recovery, SOKOBAN_RECOVERY_NEED_ID);
    }
    if (!recovery_fill_identification_metadata(recovery, solver, observation_pos, &result)) {
        return recovery_finish_with_error(recovery);
    }
    return result;
}

static void recovery_skip_pending_entity(SokobanRecovery* recovery) {
    if (!recovery || !recovery->pending_id) return;
    if (recovery->pending_kind == RECOVERY_IDENTIFY_BOX && recovery->pending_slot < MAX_BOXES) {
        recovery->box_recognized[recovery->pending_slot] = 0u;
        recovery->box_ids[recovery->pending_slot] = -1;
        if (recovery->use_recheck_scan_route) recovery->rescan_box_done[recovery->pending_slot] = 1u;
    } else if (recovery->pending_kind == RECOVERY_IDENTIFY_TARGET && recovery->pending_slot < MAX_TARGETS) {
        recovery->target_recognized[recovery->pending_slot] = 0u;
        recovery->target_ids[recovery->pending_slot] = -1;
        if (recovery->use_recheck_scan_route) recovery->rescan_target_done[recovery->pending_slot] = 1u;
    }
    recovery->pending_id = false;
    recovery->pending_directions = 0u;
    if (!recovery->use_recheck_scan_route) recovery->recognition_index++;
}

static int recovery_find_box_slot_at(const SokobanSolver* solver, Position pos) {
    if (!solver || !recovery_position_valid(pos)) return -1;
    for (uint8_t box = 0u; box < solver->num_boxes && box < MAX_BOXES; box++) {
        if (solver->boxes[box].is_active && pos_equal(solver->boxes[box].pos, pos)) {
            return (int)box;
        }
    }
    return -1;
}

static int recovery_find_target_slot_at(const SokobanSolver* solver, Position pos) {
    if (!solver || !recovery_position_valid(pos)) return -1;
    for (uint8_t target = 0u; target < solver->num_targets && target < MAX_TARGETS; target++) {
        if (solver->targets[target].is_active && pos_equal(solver->targets[target].pos, pos)) {
            return (int)target;
        }
    }
    return -1;
}

static bool recovery_build_recheck_cross_pairs(
    SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    uint8_t current_target_slots[2];

    if (!recovery || !solver || recovery->recheck_target_count != 2u ||
        recovery->recheck_initial_box_count > 1u || solver->num_boxes != 2u ||
        solver->num_targets != 2u) {
        return false;
    }
    recovery_clear_fixed_pairs(recovery);

    for (uint8_t target = 0u; target < 2u; target++) {
        int current_slot = recovery_find_target_slot_at(
            solver, recovery->recheck_target_positions[target]
        );

        if (current_slot < 0) return false;
        current_target_slots[target] = (uint8_t)current_slot;
    }
    if (current_target_slots[0] == current_target_slots[1]) return false;

    if (recovery->recheck_initial_box_count == 0u) {
        int first_box = recovery_find_box_slot_at(solver, recovery->recheck_landing_positions[0]);
        int second_box = recovery_find_box_slot_at(solver, recovery->recheck_landing_positions[1]);

        if (first_box < 0 || second_box < 0 || first_box == second_box) return false;
        recovery->fixed_target_slots[first_box] = current_target_slots[1];
        recovery->fixed_target_slots[second_box] = current_target_slots[0];
        return true;
    }

    {
        int visible_box = recovery_find_box_slot_at(solver, recovery->recheck_initial_boxes[0]);
        int hidden_box = -1;
        int hidden_origin = -1;

        if (visible_box < 0) return false;
        for (uint8_t target = 0u; target < 2u; target++) {
            int candidate = recovery_find_box_slot_at(
                solver, recovery->recheck_landing_positions[target]
            );

            if (candidate < 0 || candidate == visible_box) continue;
            if (hidden_box >= 0) return false;
            hidden_box = candidate;
            hidden_origin = (int)target;
        }
        if (hidden_box < 0 || hidden_origin < 0) return false;
        recovery->fixed_target_slots[hidden_box] = current_target_slots[1 - hidden_origin];
        recovery->fixed_target_slots[visible_box] = current_target_slots[hidden_origin];
    }
    return true;
}

static bool recovery_build_complement_pairs(
    SokobanRecovery* recovery,
    const SokobanSolver* solver
) {
    int matched_box = -1;
    int matched_target = -1;
    uint8_t matching_pairs = 0u;
    bool skipped_target = false;

    if (!recovery || !solver || recovery->mode != SOKOBAN_RECOVERY_IDENTIFIED ||
        !recovery->use_recheck_scan_route || solver->num_boxes != 2u ||
        solver->num_targets != 2u) {
        return false;
    }
    for (uint8_t target = 0u; target < 2u; target++) {
        if (recovery->rescan_target_done[target] && !recovery->target_recognized[target]) {
            skipped_target = true;
        }
    }
    if (!skipped_target) return false;

    for (uint8_t target = 0u; target < 2u; target++) {
        if (!recovery->target_recognized[target] || recovery->target_ids[target] < 0) continue;
        for (uint8_t box = 0u; box < 2u; box++) {
            if (!recovery->box_recognized[box] || recovery->box_ids[box] < 0 ||
                recovery->box_ids[box] != recovery->target_ids[target]) {
                continue;
            }
            matching_pairs++;
            matched_box = (int)box;
            matched_target = (int)target;
        }
    }
    if ((matching_pairs != 1u && matching_pairs != 2u) ||
        matched_box < 0 || matched_target < 0) {
        return false;
    }

    recovery_clear_fixed_pairs(recovery);
    recovery->fixed_target_slots[matched_box] = (uint8_t)matched_target;
    recovery->fixed_target_slots[1 - matched_box] = (uint8_t)(1 - matched_target);
    recovery->fixed_group = matching_pairs == 2u;
    return true;
}

static bool recovery_build_fixed_rules(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    RecoveryRules* rules
) {
    uint8_t paired_boxes = 0u;
    uint16_t paired_targets = 0u;

    if (!recovery || !solver || !rules) return false;
    memset(rules, 0, sizeof(*rules));
    rules->identified = true;
    rules->fixed_pairs = true;
    rules->fixed_group = recovery->fixed_group;
    memset(rules->fixed_target_slots, RECOVERY_INVALID_SLOT, sizeof(rules->fixed_target_slots));
    for (uint8_t target = 0u; target < solver->num_targets && target < MAX_TARGETS; target++) {
        rules->target_positions[target] = solver->targets[target].pos;
    }
    for (uint8_t box = 0u; box < solver->num_boxes && box < MAX_BOXES; box++) {
        uint8_t target = recovery->fixed_target_slots[box];

        if (target >= solver->num_targets || target >= MAX_TARGETS) continue;
        if ((paired_targets & (uint16_t)(1u << target)) != 0u) return false;
        rules->fixed_target_slots[box] = target;
        rules->movable_boxes |= (uint16_t)(1u << box);
        paired_targets |= (uint16_t)(1u << target);
        paired_boxes++;
    }
    return paired_boxes == solver->num_boxes && paired_boxes > 0u;
}

static SokobanRecoveryResult recovery_plan_fixed_pairs(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    Position player
) {
    RecoveryNode root;
    RecoveryRules rules;

    if (!recovery || !solver || !recovery_position_valid(player)) {
        return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    }
    recovery_node_init_from_solver(solver, &root);
    root.player = player;
    if (!recovery_build_fixed_rules(recovery, solver, &rules)) {
        return recovery_finish_with_error(recovery);
    }
    return recovery_plan_delivery(recovery, solver, &root, &rules, true);
}

static void recovery_build_identified_rules(
    const SokobanRecovery* recovery,
    const SokobanSolver* solver,
    RecoveryRules* rules
) {
    if (!recovery || !solver || !rules) return;
    memset(rules, 0, sizeof(*rules));
    rules->identified = true;
    memcpy(rules->box_ids, recovery->box_ids, sizeof(rules->box_ids));
    memcpy(rules->target_ids, recovery->target_ids, sizeof(rules->target_ids));

    for (int target = 0; target < solver->num_targets; target++) {
        rules->target_positions[target] = solver->targets[target].pos;
        if (!recovery->target_recognized[target]) continue;
        for (int box = 0; box < solver->num_boxes; box++) {
            if (!recovery->box_recognized[box]) continue;
            if (rules->box_ids[box] != rules->target_ids[target]) continue;
            rules->movable_boxes |= (uint16_t)(1u << box);
            rules->allowed_targets |= (uint16_t)(1u << target);
        }
    }
}

static SokobanRecoveryResult recovery_plan_identified(
    SokobanRecovery* recovery,
    SokobanSolver* solver
) {
    RecoveryNode root;
    RecoveryRules rules;

    if (!recovery || !solver) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    recovery_node_init_from_solver(solver, &root);
    root.player = recovery->recognition_player;
    recovery_build_identified_rules(recovery, solver, &rules);
    return recovery_plan_delivery(recovery, solver, &root, &rules, true);
}

static SokobanRecoveryResult recovery_plan_inferred_pair(
    SokobanRecovery* recovery,
    SokobanSolver* solver
) {
    RecoveryNode root;
    RecoveryRules rules;

    if (!recovery || !solver) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    /* A single remaining box and target form the only possible pair.  The
       residual loader has no IDs, so the geometric planner can deliver it. */
    recovery_node_init_from_solver(solver, &root);
    memset(&rules, 0, sizeof(rules));
    return recovery_plan_delivery(recovery, solver, &root, &rules, false);
}

static SokobanRecoveryResult recovery_prepare_recheck_scan_identification(
    SokobanRecovery* recovery,
    SokobanSolver* solver
) {
    if (!recovery || !solver) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);

    for (;;) {
        RecoveryScanRoute route;

        if (!recovery_recheck_scan_has_pending(recovery, solver)) {
            return recovery_plan_identified(recovery, solver);
        }

        if (recovery->pending_id) {
            for (uint8_t direction = 0; direction < 4u; direction++) {
                if ((recovery->pending_directions & (uint8_t)(1u << direction)) != 0u) continue;
                recovery->pending_directions |= (uint8_t)(1u << direction);
                memset(&route, 0, sizeof(route));
                if (!recovery_recheck_scan_plan_pending_direction(recovery, solver, direction, &route) ||
                    route.view_direction != direction ||
                    !recovery_recheck_scan_accept_route(recovery, solver, &route)) {
                    continue;
                }
                return recovery_recheck_scan_need_id(recovery, solver, route.observation_pos);
            }
            recovery_skip_pending_entity(recovery);
            if (recovery_build_complement_pairs(recovery, solver)) {
                return recovery_plan_fixed_pairs(
                    recovery, solver, recovery->recognition_player
                );
            }
            continue;
        }

        memset(&route, 0, sizeof(route));
        if (recovery_recheck_scan_plan_next(recovery, solver, &route)) {
            if (route.view_direction >= 4u) return recovery_finish_with_error(recovery);
            if (route.kind == RECOVERY_IDENTIFY_TARGET && route.slot < solver->num_targets) {
                recovery->pending_kind = RECOVERY_IDENTIFY_TARGET;
            } else if (route.kind == RECOVERY_IDENTIFY_BOX && route.slot < solver->num_boxes) {
                recovery->pending_kind = RECOVERY_IDENTIFY_BOX;
            } else {
                return recovery_finish_with_error(recovery);
            }
            recovery->pending_slot = route.slot;
            recovery->pending_directions = (uint8_t)(1u << route.view_direction);
            recovery->pending_id = true;
            if (recovery_recheck_scan_accept_route(recovery, solver, &route)) {
                return recovery_recheck_scan_need_id(recovery, solver, route.observation_pos);
            }
            continue;
        }

        if (!recovery_recheck_scan_select_first_pending(
                recovery,
                solver,
                &recovery->pending_kind,
                &recovery->pending_slot)) {
            return recovery_plan_identified(recovery, solver);
        }
        recovery->pending_directions = 0u;
        recovery->pending_id = true;
    }
}

static SokobanRecoveryResult recovery_prepare_next_identification(
    SokobanRecovery* recovery,
    SokobanSolver* solver
) {
    BitboardMap navigation_map;

    if (!recovery || !solver) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    if (recovery->use_recheck_scan_route) {
        return recovery_prepare_recheck_scan_identification(recovery, solver);
    }
    navigation_map = solver->bmap;

    while (recovery->recognition_index < recovery->recognition_count) {
        uint8_t kind;
        uint8_t slot;
        Position entity_pos;

        if (!recovery->pending_id) {
            recovery->pending_kind = recovery->recognition_kind[recovery->recognition_index];
            recovery->pending_slot = recovery->recognition_slot[recovery->recognition_index];
            recovery->pending_directions = 0u;
        }
        kind = recovery->pending_kind;
        slot = recovery->pending_slot;
        entity_pos = kind == RECOVERY_IDENTIFY_TARGET ? solver->targets[slot].pos : solver->boxes[slot].pos;

        for (uint8_t direction = 0; direction < 4u; direction++) {
            Direction view_direction;
            Position observation_pos;
            uint16_t path_len = 0;

            if ((recovery->pending_directions & (uint8_t)(1u << direction)) != 0u) continue;
            recovery->pending_directions |= (uint8_t)(1u << direction);
            view_direction = direction_from_index(direction);
            if (!recovery_offset(entity_pos, view_direction, -1, &observation_pos)) continue;
            if (!recovery_navigate(
                    solver,
                    &navigation_map,
                    recovery->recognition_player,
                    observation_pos,
                    recovery->path,
                    &path_len)) {
                continue;
            }
            {
                RecoveryNode observation_node;
                PathReplayOptions options = path_replay_default_options();

                recovery_node_init_from_solver(solver, &observation_node);
                observation_node.player = recovery->recognition_player;
                options.mode = PATH_REPLAY_STRICT_VALIDATE;
                if (!recovery_node_to_replay_state(
                        recovery, &observation_node, &g_recovery_replay_state) ||
                    !recovery_replay_path(
                        solver,
                        &g_recovery_replay_state,
                        recovery->path,
                        path_len,
                        &options,
                        NULL)) {
                    continue;
                }
            }

            recovery->recognition_player = observation_pos;
            recovery->path_len = path_len;
            recovery->pending_id = true;
            recovery->phase = RECOVERY_PHASE_WAIT_ID;
            if (path_len == 0u) {
                SokobanRecoveryResult result = recovery_result(SOKOBAN_RECOVERY_NEED_ID, SOKOBAN_RECOVERY_NEED_ID);
                if (!recovery_fill_identification_metadata(
                        recovery, solver, observation_pos, &result)) {
                    return recovery_finish_with_error(recovery);
                }
                return result;
            }
            {
                SokobanRecoveryResult result = recovery_path_result(recovery, SOKOBAN_RECOVERY_NEED_ID);
                if (!recovery_fill_identification_metadata(
                        recovery, solver, observation_pos, &result)) {
                    return recovery_finish_with_error(recovery);
                }
                return result;
            }
        }

        recovery->pending_id = true;
        recovery_skip_pending_entity(recovery);
    }

    return recovery_plan_identified(recovery, solver);
}

static SokobanRecoveryResult recovery_begin_identification(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    bool use_recheck_scan_route,
    bool use_dynamic_rescan_route
) {
    if (!recovery || !solver) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);

    memset(recovery->box_recognized, 0, sizeof(recovery->box_recognized));
    memset(recovery->target_recognized, 0, sizeof(recovery->target_recognized));
    recovery_clear_fixed_pairs(recovery);
    for (int i = 0; i < MAX_BOXES; i++) recovery->box_ids[i] = -1;
    for (int i = 0; i < MAX_TARGETS; i++) recovery->target_ids[i] = -1;
    recovery->recognition_index = 0;
    recovery->pending_id = false;
    recovery->pending_directions = 0u;
    recovery->recognition_player = solver->start_player;
    recovery->use_recheck_scan_route = use_recheck_scan_route;
    recovery->use_dynamic_rescan_route = use_recheck_scan_route && use_dynamic_rescan_route;
    recovery->recognition_heading = DIRECTION_INDEX_NONE;
    memset(recovery->rescan_box_done, 0, sizeof(recovery->rescan_box_done));
    memset(recovery->rescan_target_done, 0, sizeof(recovery->rescan_target_done));
    /* A new IDENTIFIED round starts its mixed-route cursor and bindings from
       scratch.  The route snapshot itself is retained across rounds, but no
       entity may inherit a consumed waypoint or stale slot binding. */
    recovery->scan_route_index = 0u;
    for (uint8_t route = 0u; route < MAX_BOXES + MAX_TARGETS; route++) {
        recovery->scan_route_bound_slot[route] = RECOVERY_INVALID_SLOT;
    }
    if (recovery->use_recheck_scan_route) {
        recovery->recognition_count = 0u;
        recovery_prepare_scan_route_bindings(recovery, solver);
    } else {
        recovery_build_identification_order(recovery, solver);
    }
    recovery->phase = RECOVERY_PHASE_PREPARE_ID;
    return recovery_prepare_next_identification(recovery, solver);
}

SokobanRecovery* sokoban_recovery_create(void) {
    sokoban_recovery_reset(&g_recovery);
    return &g_recovery;
}

void sokoban_recovery_reset(SokobanRecovery* recovery) {
    g_sokoban_recovery_need_return_path = 0u;
    if (!recovery) return;
    memset(recovery, 0, sizeof(*recovery));
    recovery->phase = RECOVERY_PHASE_IDLE;
    recovery_clear_fixed_pairs(recovery);
    recovery_clear_recheck_snapshot(recovery);
    for (int i = 0; i < MAX_BOXES; i++) recovery->box_ids[i] = -1;
    for (int i = 0; i < MAX_TARGETS; i++) recovery->target_ids[i] = -1;
    recovery->last_result = recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
}

SokobanRecoveryResult sokoban_recovery_get_result(const SokobanRecovery* recovery) {
    if (!recovery) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    return recovery->last_result;
}

bool sokoban_recovery_begin(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    SokobanRecoveryMode mode
) {
    g_sokoban_recovery_need_return_path = 0u;
    if (!recovery || !solver) return false;
    if (mode != SOKOBAN_RECOVERY_DIRECT && mode != SOKOBAN_RECOVERY_IDENTIFIED) return false;

    sokoban_recovery_reset(recovery);
    /* Recovery parks at the fixed loading-bay tile.  Only a wall in the
       original map selects the adjacent fallback; dynamic obstacles and an
       unreachable route remain strict return-path errors. */
    recovery->return_point = is_wall(solver, 5, 1) ?
        (Position){6u, 1u} : (Position){5u, 1u};
    recovery->mode = mode;
    if (mode == SOKOBAN_RECOVERY_IDENTIFIED) {
        (void)recovery_capture_scan_route(recovery, solver);
    }
    recovery->phase = RECOVERY_PHASE_WAIT_FIRST_OBSERVATION;
    recovery->active = true;
    recovery->last_result = recovery_result(
        SOKOBAN_RECOVERY_NEED_OBSERVATION,
        SOKOBAN_RECOVERY_NEED_OBSERVATION
    );
    return true;
}

SokobanRecoveryResult sokoban_recovery_submit_observation(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const char* map_string
) {
    SokobanRecoveryResult result;
    bool first_observation;
    bool second_observation;
    bool post_progress_observation;
    bool recheck_cross_pair_pending;
    bool return_after_no_progress = false;
    bool confirmed_exhausted_round = false;
    bool raw_box_count_at_least_three;
    bool raw_target_count_at_least_three;
    bool raw_box_count_mismatch;
    bool zero_box_single_target;
    bool low_target_excess_boxes;
    bool recheck_missing_box_still_hidden;
    uint8_t unresolved_suspect_count;
    uint8_t observed_box_count;
    uint8_t observed_target_count;

    if (!recovery || !solver || !recovery->active ||
        (recovery->phase != RECOVERY_PHASE_WAIT_FIRST_OBSERVATION &&
         recovery->phase != RECOVERY_PHASE_WAIT_SECOND_OBSERVATION)) {
        return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    }

    first_observation = recovery->phase == RECOVERY_PHASE_WAIT_FIRST_OBSERVATION;
    second_observation = !first_observation;
    post_progress_observation = recovery->progress_observation_pending;
    recheck_cross_pair_pending = second_observation && recovery->infer_recheck_cross_pair;
    if (!solver_load_residual_map_from_string(solver, map_string)) {
        return recovery_retry_observation(recovery);
    }

    observed_box_count = (uint8_t)solver->num_boxes;
    observed_target_count = (uint8_t)solver->num_targets;
    /* The multi-box eligibility rule is keyed to the physical box count.
       A three-box photo with fewer (or more) targets must return directly;
       the legacy one-/two-box recheck and retry rules remain below. */
    raw_box_count_at_least_three = observed_box_count >= 3u;
    /* Preserve the pre-existing conservative boundary for a target-heavy
       photo while the residual has at most two visible boxes.  In
       particular, zero boxes with three or more targets still returns. */
    raw_target_count_at_least_three = observed_target_count >= 3u;
    raw_box_count_mismatch = observed_box_count != observed_target_count;
    zero_box_single_target = observed_box_count == 0u && observed_target_count == 1u;
    recheck_missing_box_still_hidden = recovery->mode == SOKOBAN_RECOVERY_IDENTIFIED &&
        second_observation &&
        recovery->infer_recheck_cross_pair &&
        recovery_recheck_missing_box_is_still_hidden(recovery, solver);

    if (solver->num_boxes == 0u && solver->num_targets == 0u) {
        g_sokoban_recovery_need_return_path = 0u;
        recovery->phase = RECOVERY_PHASE_FINISHED;
        recovery->active = false;
        recovery_clear_progress_tracking(recovery);
        recovery->last_result = recovery_result(
            SOKOBAN_RECOVERY_COMPLETE,
            SOKOBAN_RECOVERY_COMPLETE
        );
        return recovery->last_result;
    }

    low_target_excess_boxes = !raw_box_count_at_least_three &&
        observed_target_count < 3u &&
        observed_box_count > observed_target_count;
    if (low_target_excess_boxes) {
        return recovery_retry_observation(recovery);
    }

    /* DIRECT recovery has no conservative hidden-box ledger: with fewer than
       three targets, a count mismatch is an invalid photo except for the
       explicit zero-box/single-target return boundary.  Reject retryable cases
       before consuming the pending progress snapshot so a later valid retake
       is still compared with the same just-executed path segment. */
    if (recovery->mode == SOKOBAN_RECOVERY_DIRECT &&
        !raw_box_count_at_least_three &&
        solver->num_targets < 3u &&
        !zero_box_single_target &&
        solver->num_boxes != solver->num_targets) {
        return recovery_retry_observation(recovery);
    }

    /* Count only valid raw camera layers.  A retryable observation returns
       above and therefore neither consumes the pending path snapshot nor
       changes the consecutive-same counter.  The counter compares adjacent
       observations, not every observation against the state before the last
       path segment, so a changed layout can establish a new stable streak. */
    if (post_progress_observation) {
        confirmed_exhausted_round =
            recovery_observation_confirms_exhaustion(recovery, solver);
        recovery->progress_observation_pending = false;
        recovery->confirmed_exhaustion_observation_valid = false;
    }
    return_after_no_progress = recovery_note_valid_observation(
        recovery, solver, post_progress_observation
    );

    /* Apply the current photo to the suspect ledger on every round, not only
       the historical WAIT_SECOND recheck phase.  A disappeared target clears
       its old virtual blocker; a still-visible suspect remains a blocker when
       the photo has fewer boxes than targets. */
    if (solver->num_boxes == solver->num_targets) {
        memset(recovery->unresolved_suspect_targets, 0,
               sizeof(recovery->unresolved_suspect_targets));
    } else {
        recovery_block_unresolved_suspect_targets(recovery, solver);
    }
    unresolved_suspect_count = recovery_unresolved_suspect_count(recovery);
    observed_box_count = (uint8_t)(solver->num_boxes + unresolved_suspect_count);
    observed_target_count = (uint8_t)solver->num_targets;

    /* The conservative recheck is the only place where one visible box is
       intentionally paired with two targets.  If the second raw photo still
       has that same single box, the missing box was not exposed and is
       already trapped in the dead corner represented by the remaining
       target.  Keep the suspect target blocked for the return path, then
       return immediately instead of falling through to NEED_ID. */
    if (recheck_missing_box_still_hidden) {
        recovery->infer_recheck_cross_pair = false;
        result = recovery_plan_return_only(recovery, solver, solver->start_player);
        recovery->last_result = result;
        return result;
    }

    /* These raw-count boundaries are terminal recovery outcomes, not bad
       photos.  Apply the latest photo to the suspect ledger first so a target
       that disappeared since recheck cannot survive as a ghost return blocker.
       A fully bound IDENTIFIED round also returns after one fresh photo exactly
       confirms its exhausted post-segment layout.  Keep one-box/one-target on
       the inferred-pair path because that geometry can still bypass stale IDs. */
    if (((raw_box_count_at_least_three || raw_target_count_at_least_three) &&
         raw_box_count_mismatch) ||
        zero_box_single_target || return_after_no_progress ||
        (confirmed_exhausted_round && solver->num_boxes > 1u)) {
        recovery->infer_recheck_cross_pair = false;
        result = recovery_plan_return_only(recovery, solver, solver->start_player);
        recovery->last_result = result;
        return result;
    }

    if (recovery->mode == SOKOBAN_RECOVERY_DIRECT) {
        RecoveryNode root;
        RecoveryRules rules;

        if (solver->num_boxes != solver->num_targets) {
            return recovery_retry_observation(recovery);
        }
        if (post_progress_observation) {
            recovery_reset_round_state(recovery, true);
        }
        memset(&rules, 0, sizeof(rules));
        recovery_node_init_from_solver(solver, &root);
        if (solver->num_boxes == 1u && solver->num_targets == 1u) {
            /* A singleton has a certain geometric pairing.  Do not accept a
               zero-delivery navigation prefix: an unpushable box would then
               produce another observation round and loop forever. */
            result = recovery_plan_inferred_pair(recovery, solver);
        } else {
            result = recovery_plan_delivery(recovery, solver, &root, &rules, true);
        }
        recovery->last_result = result;
        return result;
    }

    if (post_progress_observation) {
        if (second_observation) {
            /* A recheck photo that still contains a residual starts a fresh
               identification/planning round, but keeps the conservative
               suspect mask computed from that photo. */
            first_observation = true;
            if (!recheck_cross_pair_pending) {
                recovery_reset_round_state(recovery, false);
            }
        } else {
            recovery_reset_round_state(recovery, false);
        }
    } else if (second_observation && !recheck_cross_pair_pending) {
        recovery_reset_round_state(recovery, false);
        first_observation = true;
    }

    if (second_observation && recheck_cross_pair_pending) {
        recovery->infer_recheck_cross_pair = false;
        if (observed_box_count == 2u && observed_target_count == 2u &&
            recovery_build_recheck_cross_pairs(recovery, solver)) {
            result = recovery_plan_fixed_pairs(recovery, solver, solver->start_player);
        } else {
            recovery_reset_round_state(recovery, false);
            first_observation = true;
            if (observed_box_count == 1u && observed_target_count == 1u) {
                result = recovery_plan_inferred_pair(recovery, solver);
            } else if (observed_target_count == 2u && observed_box_count < 2u) {
                recovery->infer_recheck_cross_pair = true;
                result = recovery_plan_recheck(recovery, solver);
            } else {
                result = recovery_begin_identification(
                    recovery,
                    solver,
                    recovery->mode == SOKOBAN_RECOVERY_IDENTIFIED,
                    true
                );
            }
        }
    } else if (first_observation && observed_target_count == 2u &&
               observed_box_count < 2u) {
        recovery->infer_recheck_cross_pair = true;
        result = recovery_plan_recheck(recovery, solver);
    } else if (observed_box_count == 1u && observed_target_count == 1u) {
        recovery->infer_recheck_cross_pair = false;
        result = recovery_plan_inferred_pair(recovery, solver);
    } else {
        /* A failed recheck leaves no unique complement, so restore the normal
           ID-scanning path instead of weakening matching constraints. */
        recovery->infer_recheck_cross_pair = false;
        /* Entity proximity belongs to the current residual, not the original
           map whose route was captured when the recovery session began. */
        result = recovery_begin_identification(
            recovery,
            solver,
            recovery->mode == SOKOBAN_RECOVERY_IDENTIFIED,
            true
        );
    }
    recovery->last_result = result;
    return result;
}

SokobanRecoveryResult sokoban_recovery_submit_id(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    int id
) {
    SokobanRecoveryResult result;
    bool trusted_id;

    if (!recovery || !solver || !recovery->active ||
        recovery->phase != RECOVERY_PHASE_WAIT_ID || !recovery->pending_id) {
        return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    }

    trusted_id = id >= 0 && id <= 9;
    if (trusted_id) {
        if (recovery->pending_kind == RECOVERY_IDENTIFY_BOX && recovery->pending_slot < MAX_BOXES) {
            recovery->box_recognized[recovery->pending_slot] = 1u;
            recovery->box_ids[recovery->pending_slot] = (int8_t)id;
            solver->boxes[recovery->pending_slot].id = (int8_t)id;
            if (recovery->use_recheck_scan_route) {
                recovery->rescan_box_done[recovery->pending_slot] = 1u;
            }
        } else if (recovery->pending_kind == RECOVERY_IDENTIFY_TARGET && recovery->pending_slot < MAX_TARGETS) {
            recovery->target_recognized[recovery->pending_slot] = 1u;
            recovery->target_ids[recovery->pending_slot] = (int8_t)id;
            solver->targets[recovery->pending_slot].id = (int8_t)id;
            if (recovery->use_recheck_scan_route) {
                recovery->rescan_target_done[recovery->pending_slot] = 1u;
            }
        } else {
            return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
        }
        recovery->pending_id = false;
        recovery->pending_directions = 0u;
        if (!recovery->use_recheck_scan_route) recovery->recognition_index++;
    } else {
        /* Keep the entity pending; preparation tries every remaining view. */
    }

    if (trusted_id && recovery_build_complement_pairs(recovery, solver)) {
        result = recovery_plan_fixed_pairs(recovery, solver, recovery->recognition_player);
        recovery->last_result = result;
        return result;
    }

    recovery->phase = RECOVERY_PHASE_PREPARE_ID;
    result = recovery_prepare_next_identification(recovery, solver);
    recovery->last_result = result;
    return result;
}

static SokobanRecoveryResult recovery_step_finish_with_error(SokobanRecovery* recovery) {
    SokobanRecoveryResult result = recovery_finish_with_error(recovery);

    if (recovery) recovery->last_result = result;
    return result;
}

/* Publish the internal observation wait after the caller has executed a path. */
static SokobanRecoveryResult recovery_step_publish_observation_request(
    SokobanRecovery* recovery
) {
    if (!recovery) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    if (recovery->last_result.status == SOKOBAN_RECOVERY_RETRY_OBSERVATION) {
        return recovery->last_result;
    }
    if (recovery->last_result.status != SOKOBAN_RECOVERY_NEED_OBSERVATION &&
        recovery->last_result.status != SOKOBAN_RECOVERY_PATH_READY) {
        return recovery_step_finish_with_error(recovery);
    }

    recovery->last_result = recovery_result(
        SOKOBAN_RECOVERY_NEED_OBSERVATION,
        SOKOBAN_RECOVERY_NEED_OBSERVATION
    );
    return recovery->last_result;
}

/* Preserve the observation metadata when publishing an ID request after a path. */
static SokobanRecoveryResult recovery_step_publish_id_request(
    SokobanRecovery* recovery,
    SokobanSolver* solver
) {
    SokobanRecoveryResult result;

    if (!recovery || !solver) {
        return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    }
    result = recovery->last_result;
    if (!recovery_identification_metadata_matches(recovery, solver, &result)) {
        return recovery_step_finish_with_error(recovery);
    }
    if (result.status == SOKOBAN_RECOVERY_NEED_ID) return result;
    if (result.status != SOKOBAN_RECOVERY_PATH_READY ||
        result.next_status != SOKOBAN_RECOVERY_NEED_ID) {
        return recovery_step_finish_with_error(recovery);
    }

    result.status = SOKOBAN_RECOVERY_NEED_ID;
    result.next_status = SOKOBAN_RECOVERY_NEED_ID;
    result.path = NULL;
    result.path_len = 0u;
    recovery->last_result = result;
    return result;
}

/* Collapse a completed delivery or return path into its terminal result. */
static SokobanRecoveryResult recovery_step_publish_terminal_result(
    SokobanRecovery* recovery
) {
    SokobanRecoveryResult result;

    if (!recovery) return recovery_result(SOKOBAN_RECOVERY_ERROR, SOKOBAN_RECOVERY_ERROR);
    result = recovery->last_result;
    if (result.status == SOKOBAN_RECOVERY_COMPLETE ||
        result.status == SOKOBAN_RECOVERY_PARTIAL_RETURNED ||
        result.status == SOKOBAN_RECOVERY_ERROR) {
        return result;
    }
    if (result.status != SOKOBAN_RECOVERY_PATH_READY ||
        (result.next_status != SOKOBAN_RECOVERY_COMPLETE &&
         result.next_status != SOKOBAN_RECOVERY_PARTIAL_RETURNED)) {
        return recovery_step_finish_with_error(recovery);
    }

    result.status = result.next_status;
    result.path = NULL;
    result.path_len = 0u;
    result.observation_pos = recovery_invalid_position();
    result.observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE;
    result.entity_pos = recovery_invalid_position();
    result.view_direction = (Direction){0, 0};
    recovery->last_result = result;
    return result;
}

SokobanRecoveryResult sokoban_recovery_step(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const SokobanRecoveryInput* input
) {
    if (!recovery || !solver) {
        return recovery_step_finish_with_error(recovery);
    }

    /* A repeated call after PATH_READY acknowledges that the caller executed it. */
    if (recovery->last_result.status == SOKOBAN_RECOVERY_PATH_READY) {
        switch (recovery->phase) {
            case RECOVERY_PHASE_WAIT_FIRST_OBSERVATION:
            case RECOVERY_PHASE_WAIT_SECOND_OBSERVATION:
                return recovery_step_publish_observation_request(recovery);

            case RECOVERY_PHASE_WAIT_ID:
                return recovery_step_publish_id_request(recovery, solver);

            case RECOVERY_PHASE_FINISHED:
                return recovery_step_publish_terminal_result(recovery);

            default:
                return recovery_step_finish_with_error(recovery);
        }
    }

    switch (recovery->phase) {
        case RECOVERY_PHASE_WAIT_FIRST_OBSERVATION:
        case RECOVERY_PHASE_WAIT_SECOND_OBSERVATION:
            if (!recovery->active) return recovery_step_finish_with_error(recovery);
            if (input && input->has_observation) {
                if (!input->map_string) {
                    /* A missing/invalid photo is retryable in every round;
                       never turn an observation-quality failure into an early
                       return or terminal error. */
                    return recovery_retry_observation(recovery);
                }
                return sokoban_recovery_submit_observation(
                    recovery, solver, input->map_string
                );
            }
            return recovery_step_publish_observation_request(recovery);

        case RECOVERY_PHASE_WAIT_ID:
            if (!recovery->active || !recovery->pending_id) {
                return recovery_step_finish_with_error(recovery);
            }
            if (input && input->has_id) {
                return sokoban_recovery_submit_id(recovery, solver, input->id);
            }
            return recovery_step_publish_id_request(recovery, solver);

        case RECOVERY_PHASE_FINISHED:
            return recovery_step_publish_terminal_result(recovery);

        default:
            return recovery_step_finish_with_error(recovery);
    }
}
