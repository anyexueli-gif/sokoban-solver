#include "sokoban_solver.h"
#include "sokoban_scan.h"
#include "sokoban_flash.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>




static BitboardMap g_scan_initial_bmap;
static Position g_scan_initial_player;
static Entity g_scan_initial_boxes[MAX_BOXES];
static Entity g_scan_initial_bombs[MAX_BOMBS];
static int g_scan_initial_num_bombs;


#define SCAN_LEGACY_SMALL_BALANCED_ENTITY_COUNT 3
#define SCAN_CLEARANCE_SAFE_BOX_LIMIT 3
#define SCAN_VERIFIED_WAYPOINT_COUNT 4
#define SCAN_VERIFIED_MIN_FAST_LEN 50u
#define SCAN_CLEARANCE_CANDIDATE_LIMIT 15
#define SCAN_INITIAL_HEADING_DIR DIRECTION_INDEX_NONE
#define SCAN_FINAL_LEN_SLACK 4u
#define SCAN_STEP_SCORE 2u
#define SCAN_TURN_90_SCORE 7u
#define SCAN_BEND_SLOWDOWN_SCORE 7u
#define SCAN_VIEW_LOOKAHEAD_SCORE_SLACK 6u
#define SCAN_VIEW_LOOKAHEAD_CORNER_PENALTY 24u
#define SCAN_VIEW_LOOKAHEAD_BLOCKED_LINE_PENALTY 16u
#define SCAN_DEFER_HIGH_LB_TRIGGER_DELTA 6u
#define SCAN_DEFER_LOW_LB_SLACK 3u
#define SCAN_RETURN_REGION_MIN_AXIS_SEPARATION 8
#define SCAN_RETURN_REGION_MIN_SIDE_MARGIN4 4
#define SCAN_PAIR_BEAM_WIDTH 8
#define SCAN_PAIR_BEAM_ENTRY_TOP_K 3
#define SCAN_PAIR_BEAM_STATE_LIMIT 48
#ifndef SCAN_SPINE_PROGRESS_WEIGHT
#define SCAN_SPINE_PROGRESS_WEIGHT 8u
#endif
#ifndef SCAN_SPINE_STEP_PROGRESS_WEIGHT
#define SCAN_SPINE_STEP_PROGRESS_WEIGHT 2u
#endif


#ifndef ALLOC_IN_SDRAM_CACHE
#define ALLOC_IN_SDRAM_CACHE SOKOBAN_BSS_SECTION("SDRAM_CACHE")
#endif
#ifndef ALLOC_IN_SDRAM
#define ALLOC_IN_SDRAM ALLOC_IN_SDRAM_CACHE
#endif
#ifndef FAST_OCRAM_FUNC
#define FAST_OCRAM_FUNC __attribute__((section("OCRAM_CODE")))
#endif

#ifndef ALLOC_IN_DTCM
#define ALLOC_IN_DTCM ALLOC_IN_SDRAM
#endif



#define SCAN_BOMB_MAIN_ROUTE_PATH_BYTES ((MAX_PATH_LENGTH + 3u) / 4u)
typedef struct {
    uint16_t path_len;
    uint8_t packed_path[SCAN_BOMB_MAIN_ROUTE_PATH_BYTES];
} ScanBombMainRouteCandidate;

static Direction g_scan_hybrid_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static Direction g_scan_verified_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static ScanBombMainRouteCandidate g_scan_bomb_main_route_candidates[
    SOKOBAN_SCAN_BOMB_MAIN_ROUTE_CANDIDATE_LIMIT
] ALLOC_IN_SDRAM;
static Entity g_scan_bomb_best_waypoints[MAX_BOXES + MAX_TARGETS] ALLOC_IN_SDRAM;
static Position g_scan_bomb_best_pauses[MAX_BOXES + MAX_TARGETS] ALLOC_IN_SDRAM;
static PathReplayState g_scan_bomb_best_final_state ALLOC_IN_SDRAM;
static uint8_t g_scan_bomb_main_route_candidate_count = 0;
static int8_t g_scan_bomb_main_route_override_index = -1;
static Direction g_scan_eager_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
/* This must not alias solver->best_path in the verified scan branch. */
static Direction g_scan_compact_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static Entity g_scan_eager_waypoints[MAX_BOXES + MAX_TARGETS] ALLOC_IN_SDRAM;
static Position g_scan_eager_pauses[MAX_BOXES + MAX_TARGETS] ALLOC_IN_SDRAM;
static Entity g_scan_eager_desired_waypoints[MAX_BOXES + MAX_TARGETS] ALLOC_IN_SDRAM;
static bool g_scan_eager_emitted[MAX_BOXES + MAX_TARGETS] ALLOC_IN_SDRAM;
static SokobanSolver g_scan_verified_solver;
typedef struct {
    uint16_t dist[MAP_ROWS][MAP_COLS];
    uint8_t parent_dir[MAP_ROWS][MAP_COLS];
    Position queue[MAP_ROWS * MAP_COLS];
} ScanNavScratch;

typedef struct {
    uint16_t visited[MAP_ROWS][MAP_COLS];
    uint16_t dist[MAP_ROWS][MAP_COLS];
    Position parent[MAP_ROWS][MAP_COLS];
    Position queue[MAP_ROWS * MAP_COLS];
} ScanClearBfsScratch;

typedef union {
    ScanNavScratch nav;
    ScanNavScratch try_nav;
    ScanClearBfsScratch clear_bfs;
} ScanBfsWorkScratch;

typedef struct {
    ScanBfsWorkScratch work;
    Position clear_candidates[MAP_ROWS * MAP_COLS];
} ScanBfsScratch;

static ScanBfsScratch g_scan_bfs_scratch ALLOC_IN_SDRAM;

/*
 * Pure-navigation compaction needs one more dimension than the normal BFS:
 * at an equal step count, a lower turn count at the same arrival direction
 * dominates; at different step counts it does not.  Keep all predecessors in
 * SDRAM so a shorter-but-not-globally-shortest valid route can be rebuilt.
 */
#define SCAN_NAV_COMPACT_MAX_STEPS ((uint16_t)(MAX_PATH_LENGTH - 1u))
#define SCAN_NAV_COMPACT_PARENT_STEPS ((uint16_t)(MAX_PATH_LENGTH - 2u))
#define SCAN_NAV_COMPACT_PARENT_STATE_COUNT (4u * MAP_ROWS * MAP_COLS)
#define SCAN_NAV_COMPACT_PARENT_BYTES \
    ((SCAN_NAV_COMPACT_PARENT_STEPS * SCAN_NAV_COMPACT_PARENT_STATE_COUNT + 3u) / 4u)

typedef struct {
    uint16_t turns[2][4][MAP_ROWS][MAP_COLS];
    /* Two bits: the previous move direction for (arrival step, cell, direction). */
    uint8_t parent_bits[SCAN_NAV_COMPACT_PARENT_BYTES];
} ScanNavCompactDpScratch;

_Static_assert(sizeof(g_scan_bfs_scratch.work.nav.queue) >= MAP_ROWS * MAP_COLS * sizeof(Position),
               "scan nav queue scratch too small");
_Static_assert(sizeof(g_scan_bfs_scratch.work.clear_bfs.queue) >= MAP_ROWS * MAP_COLS * sizeof(Position),
               "scan clear queue scratch too small");
_Static_assert(sizeof(g_scan_bfs_scratch.clear_candidates) >= MAP_ROWS * MAP_COLS * sizeof(Position),
               "scan clear candidates scratch too small");
typedef struct {
    Direction best_push_path[MAX_SINGLE_PATH];
    Direction push_path[MAX_SINGLE_PATH];
    Direction nav_path[MAX_SINGLE_PATH];
} ScanClearScratch;

typedef struct {
    Direction nearest_entity_path[MAX_SINGLE_PATH];
    Direction best_path[MAX_SINGLE_PATH];
    Direction candidate_path[MAX_SINGLE_PATH];
} ScanTryScratch;

typedef union {
    Direction sequence_temp_path[MAX_SINGLE_PATH];
    ScanClearScratch clear;
    ScanTryScratch try_scan;
} ScanPathScratchWork;

typedef struct {
    Direction choose_candidate_path[MAX_SINGLE_PATH];
    Direction sequence_segment_best_path[MAX_SINGLE_PATH];
    ScanPathScratchWork work;
} ScanPathScratch;

static ScanPathScratch g_scan_path_scratch ALLOC_IN_SDRAM;
static uint16_t g_scan_nav_turn_cost[4][MAP_ROWS][MAP_COLS] ALLOC_IN_SDRAM;
static uint8_t g_scan_nav_turn_parent[4][MAP_ROWS][MAP_COLS] ALLOC_IN_SDRAM;
static Position g_scan_spine_turn_cache_target;
static uint8_t g_scan_spine_turn_cache_start_heading;
static bool g_scan_spine_turn_cache_valid;

typedef struct {
    bool planned_boxes[MAX_BOXES];
    bool planned_targets[MAX_TARGETS];
    BitboardMap sim_map;
    Position temp_boxes[MAX_BOXES];
    Position sim_boxes[MAX_BOXES];
    Direction best_to_path[MAX_SINGLE_PATH];
    Direction best_back_path[MAX_SINGLE_PATH];
    Direction temp_to_path[MAX_SINGLE_PATH];
    Direction temp_back_path[MAX_SINGLE_PATH];
} ScanExtendScratch;

static ScanExtendScratch g_scan_extend_scratch ALLOC_IN_SDRAM;

typedef struct {
    int box_idx;
    int target_idx;
} ScanPairedTask;

typedef struct {
    bool valid;
    Position view_pos;
    uint8_t heading;
    uint16_t len;
    uint16_t score;
    Direction path[MAX_SINGLE_PATH];
} ScanBeamEntryPath;

typedef struct {
    bool valid;
    uint16_t remaining_mask;
    Position pos;
    uint8_t heading;
    uint16_t score;
    uint16_t len;
    int waypoint_count;
    bool visited_boxes[MAX_BOXES];
    bool visited_targets[MAX_TARGETS];
    Direction path[MAX_PATH_LENGTH];
    Entity waypoints[MAX_BOXES + MAX_TARGETS];
    Position pauses[MAX_BOXES + MAX_TARGETS];
} ScanBeamState;

typedef struct {
    ScanPairedTask tasks[MAX_BOXES];
    ScanBeamState current[SCAN_PAIR_BEAM_WIDTH];
    ScanBeamState next[SCAN_PAIR_BEAM_STATE_LIMIT];
    ScanBeamState candidate;
    ScanBeamEntryPath first_entries[SCAN_PAIR_BEAM_ENTRY_TOP_K];
    ScanBeamEntryPath second_entries[SCAN_PAIR_BEAM_ENTRY_TOP_K];
    Direction planned_path[MAX_PATH_LENGTH];
    Entity planned_waypoints[MAX_BOXES + MAX_TARGETS];
    Position planned_pauses[MAX_BOXES + MAX_TARGETS];
    Direction greedy_path[MAX_PATH_LENGTH];
    Entity greedy_waypoints[MAX_BOXES + MAX_TARGETS];
    Position greedy_pauses[MAX_BOXES + MAX_TARGETS];
} ScanBeamScratch;


#ifndef SCAN_SPINE_STATE_LIMIT
#define SCAN_SPINE_STATE_LIMIT 4096
#endif
#define SCAN_SPINE_STATE_BUCKET_COUNT 4096u
#ifndef SCAN_SPINE_IMPROVEMENT_PRIORITY_SLACK
#define SCAN_SPINE_IMPROVEMENT_PRIORITY_SLACK 20u
#endif
#define SCAN_SPINE_NO_PARENT 0xFFFFu

typedef enum {
    SCAN_SPINE_ACTION_START = 0,
    SCAN_SPINE_ACTION_ADVANCE = 1,
    SCAN_SPINE_ACTION_REJOIN = 2,
    SCAN_SPINE_ACTION_SCAN_BOX = 3,
    SCAN_SPINE_ACTION_SCAN_TARGET = 4
} ScanSpineAction;

typedef struct {
    BitboardMap map;
    Position pos;
    Position boxes[MAX_BOXES];
    bool next_is_stateful;
} ScanSpineFrame;

typedef struct {
    uint16_t cost;
    uint16_t parent;
    uint16_t step;
    Position pos;
    uint16_t box_mask;
    uint16_t target_mask;
    uint8_t action;
    int8_t entity_idx;
    uint8_t heading;
    bool closed;
} ScanSpineState;
typedef struct {
    uint16_t head;
    uint16_t tail;
} ScanSpineStateBucket;

#define SCAN_SPINE_NAV_CACHE_SET_COUNT 128u
#define SCAN_SPINE_NAV_CACHE_WAYS 4u
#define SCAN_SPINE_NAV_CACHE_ENTRY_COUNT \
    (SCAN_SPINE_NAV_CACHE_SET_COUNT * SCAN_SPINE_NAV_CACHE_WAYS)
typedef struct {
    uint16_t obstacles[MAP_ROWS];
    uint8_t dist[MAP_ROWS][MAP_COLS];
    Position start;
    bool valid;
} ScanSpineNavCacheEntry;

typedef struct {
    ScanSpineFrame frames[MAX_PATH_LENGTH + 1];
    ScanSpineNavCacheEntry nav_cache[SCAN_SPINE_NAV_CACHE_ENTRY_COUNT];
    uint8_t nav_cache_next_way[SCAN_SPINE_NAV_CACHE_SET_COUNT];
    uint8_t nav_layer_offsets[MAP_ROWS * MAP_COLS];
    uint16_t nav_order_count;
    ScanSpineStateBucket state_buckets[SCAN_SPINE_STATE_BUCKET_COUNT];
    uint16_t state_heap[SCAN_SPINE_STATE_LIMIT];
    uint16_t state_heap_pos[SCAN_SPINE_STATE_LIMIT];
    uint8_t state_heap_remaining[SCAN_SPINE_STATE_LIMIT];
} ScanSpineScratch;


#ifndef SCAN_ANONYMOUS_BEAM_WIDTH
#define SCAN_ANONYMOUS_BEAM_WIDTH 512u
#endif
#define SCAN_ANONYMOUS_CANDIDATE_LIMIT (SCAN_ANONYMOUS_BEAM_WIDTH * 4u)
#define SCAN_ANONYMOUS_CANDIDATE_BASE SCAN_ANONYMOUS_BEAM_WIDTH
#define SCAN_ANONYMOUS_STORAGE_LIMIT \
    (SCAN_ANONYMOUS_BEAM_WIDTH + SCAN_ANONYMOUS_CANDIDATE_LIMIT)
#define SCAN_ANONYMOUS_PATH_BYTES ((MAX_PATH_LENGTH + 3u) / 4u)
#define SCAN_ANONYMOUS_HASH_SIZE 4096u
#define SCAN_ANONYMOUS_PROGRESS_WEIGHT 8u
#ifndef SCAN_ANONYMOUS_STOP_ON_FIRST_COMPLETE
#define SCAN_ANONYMOUS_STOP_ON_FIRST_COMPLETE 1
#endif
#define SCAN_ANONYMOUS_DISABLE_DISTANCE_BOUND 1

typedef struct {
    uint8_t path_bits[SCAN_ANONYMOUS_STORAGE_LIMIT][SCAN_ANONYMOUS_PATH_BYTES];
    uint8_t best_path_bits[SCAN_ANONYMOUS_PATH_BYTES];
    uint32_t run_score[SCAN_ANONYMOUS_STORAGE_LIMIT];
    uint16_t run_len[SCAN_ANONYMOUS_STORAGE_LIMIT];
    uint16_t hash[SCAN_ANONYMOUS_HASH_SIZE];
} ScanAnonymousScratch;

typedef union {
    ScanNavCompactDpScratch nav_compact_dp;
    ScanBeamScratch beam;
    ScanSpineScratch spine;
    ScanAnonymousScratch anonymous;
} ScanPhaseWorkspace;

static ScanPhaseWorkspace g_scan_phase_workspace ALLOC_IN_SDRAM;
static ScanSpineState g_scan_spine_states[SCAN_SPINE_STATE_LIMIT] ALLOC_IN_SDRAM;
static uint16_t g_scan_spine_order[SCAN_SPINE_STATE_LIMIT] ALLOC_IN_SDRAM;

typedef struct {
    SokobanSolver* solver;
    int prefix_len;
    int req_boxes;
    int req_targets;
    uint16_t count;
    bool favor_progress;
    bool free_terminal;
} ScanSpineHeapContext;
static ScanSpineHeapContext g_scan_spine_heap_context;

#define g_scan_nav_compact_dp (g_scan_phase_workspace.nav_compact_dp)
#define g_scan_beam_scratch (g_scan_phase_workspace.beam)
#define g_scan_spine_frames (g_scan_phase_workspace.spine.frames)
#define g_scan_spine_nav_cache (g_scan_phase_workspace.spine.nav_cache)
#define g_scan_spine_nav_cache_next_way (g_scan_phase_workspace.spine.nav_cache_next_way)
#define g_scan_spine_nav_layer_offsets (g_scan_phase_workspace.spine.nav_layer_offsets)
#define g_scan_spine_nav_order_count (g_scan_phase_workspace.spine.nav_order_count)
#define g_scan_spine_state_buckets (g_scan_phase_workspace.spine.state_buckets)
#define g_scan_spine_state_heap (g_scan_phase_workspace.spine.state_heap)
#define g_scan_spine_state_heap_pos (g_scan_phase_workspace.spine.state_heap_pos)
#define g_scan_spine_state_heap_remaining (g_scan_phase_workspace.spine.state_heap_remaining)
#define g_scan_anonymous_path_bits (g_scan_phase_workspace.anonymous.path_bits)
#define g_scan_anonymous_best_path_bits (g_scan_phase_workspace.anonymous.best_path_bits)
#define g_scan_anonymous_run_score (g_scan_phase_workspace.anonymous.run_score)
#define g_scan_anonymous_run_len (g_scan_phase_workspace.anonymous.run_len)
#define g_scan_anonymous_hash (g_scan_phase_workspace.anonymous.hash)

_Static_assert(SCAN_ANONYMOUS_STORAGE_LIMIT <= SCAN_SPINE_STATE_LIMIT,
               "anonymous scan storage must fit the shared spine state pool");
_Static_assert(MAP_ROWS * MAP_COLS < UINT8_MAX,
               "spine navigation cache distance must fit uint8_t");
_Static_assert(MAX_BOXES + MAX_TARGETS < UINT8_MAX,
               "spine heap remaining count must fit uint8_t");
_Static_assert((SCAN_SPINE_STATE_BUCKET_COUNT &
                (SCAN_SPINE_STATE_BUCKET_COUNT - 1u)) == 0u,
               "spine state bucket count must be a power of two");
_Static_assert(sizeof(ScanSpineScratch) <=
                   sizeof(ScanAnonymousScratch) +
                   SCAN_SPINE_STATE_LIMIT * sizeof(uint16_t),
               "spine scratch must stay within bounded ten-entity mask growth");
_Static_assert((SCAN_ANONYMOUS_HASH_SIZE & (SCAN_ANONYMOUS_HASH_SIZE - 1u)) == 0u,
               "anonymous scan hash size must be a power of two");
_Static_assert(sizeof(ScanPhaseWorkspace) - sizeof(ScanAnonymousScratch) <=
                   SCAN_SPINE_STATE_LIMIT * sizeof(uint16_t) +
                       _Alignof(ScanPhaseWorkspace) - 1u,
               "scan phase workspace must stay within bounded ten-entity mask growth");
_Static_assert(sizeof(((ScanNavCompactDpScratch*)0)->parent_bits) == SCAN_NAV_COMPACT_PARENT_BYTES,
               "compact navigation parent scratch size mismatch");
_Static_assert(sizeof(ScanPhaseWorkspace) <
                   sizeof(ScanNavCompactDpScratch) + sizeof(ScanBeamScratch) +
                   sizeof(ScanSpineFrame) * (MAX_PATH_LENGTH + 1u) +
                   sizeof(ScanAnonymousScratch),
               "scan phase workspace must reduce static storage");

static PathReplayOptions scan_replay_lenient_options(void) {
    PathReplayOptions options = path_replay_default_options();
    options.mode = PATH_REPLAY_LEGACY_LENIENT;
    options.preserve_dynamic_tunnels_on_blast = true;
    return options;
}

static PathReplayOptions scan_replay_hide_box_keep_target_options(void) {
    PathReplayOptions options = scan_replay_lenient_options();
    options.box_target_mode = PATH_REPLAY_BOX_TARGET_HIDE_BOX_KEEP_TARGET;
    return options;
}


static void scan_prefix_capture_initial_state(SokobanSolver* solver);
static bool scan_prefix_rebind_eager_observations_for_waypoints(
    SokobanSolver* solver,
    const Entity* source_waypoints,
    const Position* source_pauses,
    const Entity* desired_waypoints,
    int waypoint_count,
    const PathReplayOptions* replay_options
);
static bool scan_prefix_validate_first_arrival_observations(
    const SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    const Entity* waypoints,
    const Position* pause_positions,
    int waypoint_count,
    const PathReplayOptions* replay_options
);

static bool scan_replay_load_from_solver(
    PathReplayState* state,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
) {
    return path_replay_load_state(
        state,
        initial_map,
        initial_player,
        initial_boxes,
        initial_num_boxes,
        initial_bombs,
        initial_num_bombs
    );
}

static bool scan_replay_step_existing(
    const SokobanSolver* solver,
    PathReplayState* state,
    Direction d,
    const PathReplayOptions* options,
    PathReplayStepResult* out_step
) {
    PathReplayStepResult step = path_replay_step(solver, state, d, options);
    if (out_step) *out_step = step;
    return step.kind != PATH_REPLAY_STEP_ERROR && step.kind != PATH_REPLAY_STEP_STOPPED;
}
static bool scan_replay_apply_path_existing(
    const SokobanSolver* solver,
    PathReplayState* state,
    const Direction* path,
    uint16_t path_len,
    const PathReplayOptions* options
) {
    if (!solver || !state || (!path && path_len > 0)) return false;
    for (uint16_t i = 0; i < path_len; i++) {
        if (!scan_replay_step_existing(solver, state, path[i], options, NULL)) return false;
    }
    return true;
}

static void scan_replay_copy_positions(Position out_positions[MAX_BOXES], const PathReplayState* state) {
    for (int i = 0; i < MAX_BOXES; i++) out_positions[i] = state->boxes[i];
}

static void scan_replay_write_solver_state(SokobanSolver* solver, const PathReplayState* state) {
    solver->bmap = state->map;
    solver->start_player = state->player;
    for (int i = 0; i < solver->num_boxes && i < MAX_BOXES; i++) solver->boxes[i].pos = state->boxes[i];
    solver->num_bombs = (uint8_t)state->bomb_count;
    memset(solver->bombs, 0, sizeof(solver->bombs));
    if (state->bomb_count > 0) memcpy(solver->bombs, state->bombs, sizeof(Entity) * state->bomb_count);
}

static void scan_refresh_bomb_delay_events(SokobanSolver* solver) {
    if (!solver) return;
    solver_build_bomb_delay_events_from_state(
        solver, &g_scan_initial_bmap, g_scan_initial_player,
        g_scan_initial_boxes, solver->num_boxes,
        g_scan_initial_bombs, g_scan_initial_num_bombs
    );
}

_Static_assert(sizeof(g_scan_path_scratch.choose_candidate_path) >= MAX_SINGLE_PATH * sizeof(Direction),
               "scan choose path scratch too small");
_Static_assert(sizeof(g_scan_path_scratch.sequence_segment_best_path) >= MAX_SINGLE_PATH * sizeof(Direction),
               "scan sequence path scratch too small");
_Static_assert(sizeof(g_scan_path_scratch.work.clear.push_path) >= MAX_SINGLE_PATH * sizeof(Direction),
               "scan clear path scratch too small");
_Static_assert(sizeof(g_scan_path_scratch.work.try_scan.candidate_path) >= MAX_SINGLE_PATH * sizeof(Direction),
               "scan try path scratch too small");

#define g_scan_choose_candidate_path      (g_scan_path_scratch.choose_candidate_path)
#define g_scan_clear_best_push_path       (g_scan_path_scratch.work.clear.best_push_path)
#define g_scan_clear_push_path            (g_scan_path_scratch.work.clear.push_path)
#define g_scan_clear_nav_path             (g_scan_path_scratch.work.clear.nav_path)
#define g_scan_sequence_segment_best_path (g_scan_path_scratch.sequence_segment_best_path)
#define g_scan_sequence_temp_path         (g_scan_path_scratch.work.sequence_temp_path)
#define g_scan_nearest_entity_path        (g_scan_path_scratch.work.try_scan.nearest_entity_path)
#define g_scan_try_best_path              (g_scan_path_scratch.work.try_scan.best_path)
#define g_scan_try_candidate_path         (g_scan_path_scratch.work.try_scan.candidate_path)


static bool scan_cached_astar_single_box(SokobanSolver* solver, const BitboardMap* bmap,
                                         Position start_player, Position start_box, Position target_pos,
                                         uint8_t collision_mask, Direction* out_path, uint16_t* out_len,
                                         int macro_depth, AStarRouteType route_type) {
    hash_table_clear();
    return astar_solve_single_box_mask(solver->heap, solver->closed_list, bmap, start_player, start_box,
                                       target_pos, collision_mask, out_path, out_len, macro_depth, route_type);
}

static bool scan_cached_astar_navigate(SokobanSolver* solver, const BitboardMap* bmap, Position start,
                                       Position target, uint8_t collision_mask, Direction* out_path,
                                       uint16_t* out_len) {
    hash_table_clear();
    return astar_navigate_mask(solver->heap, solver->closed_list, bmap, start, target,
                               collision_mask, out_path, out_len);
}

static inline int8_t scan_waypoint_box_tag(int idx) {
    return (int8_t)(SCAN_WAYPOINT_BOX_TAG_BASE - idx);
}

static inline int8_t scan_waypoint_target_tag(int idx) {
    return (int8_t)(SCAN_WAYPOINT_TARGET_TAG_BASE - idx);
}

static bool scan_decode_waypoint_entity(const SokobanSolver* solver, int8_t tag, bool* out_is_box, int* out_idx) {
    if (!solver || !out_is_box || !out_idx) return false;

    int value = (int)tag;
    if (value <= SCAN_WAYPOINT_BOX_TAG_BASE && value > SCAN_WAYPOINT_BOX_TAG_BASE - MAX_BOXES) {
        int idx = SCAN_WAYPOINT_BOX_TAG_BASE - value;
        if (idx >= 0 && idx < solver->num_boxes) {
            *out_is_box = true;
            *out_idx = idx;
            return true;
        }
    }

    if (value <= SCAN_WAYPOINT_TARGET_TAG_BASE && value > SCAN_WAYPOINT_TARGET_TAG_BASE - MAX_TARGETS) {
        int idx = SCAN_WAYPOINT_TARGET_TAG_BASE - value;
        if (idx >= 0 && idx < solver->num_targets) {
            *out_is_box = false;
            *out_idx = idx;
            return true;
        }
    }

    return false;
}

static bool scan_find_waypoint_entity_by_position(const SokobanSolver* solver, Position pos, bool* out_is_box, int* out_idx) {
    if (!solver || !out_is_box || !out_idx) return false;

    for (int i = 0; i < solver->num_boxes; i++) {
        if (pos_equal(solver->boxes[i].pos, pos) || pos_equal(g_scan_initial_boxes[i].pos, pos)) {
            *out_is_box = true;
            *out_idx = i;
            return true;
        }
    }

    for (int i = 0; i < solver->num_targets; i++) {
        if (pos_equal(solver->targets[i].pos, pos)) {
            *out_is_box = false;
            *out_idx = i;
            return true;
        }
    }

    return false;
}


static bool scan_collect_id_balance(
    const int* b_counts,
    const int* t_counts,
    int* common_count,
    int* box_surplus,
    int* box_surplus_count,
    int* target_surplus,
    int* target_surplus_count
) {
    *common_count = 0;
    *box_surplus_count = 0;
    *target_surplus_count = 0;

    for (int id = 0; id < 10; id++) {
        *common_count += (b_counts[id] < t_counts[id]) ? b_counts[id] : t_counts[id];
        for (int n = t_counts[id]; n < b_counts[id]; n++) {
            if (*box_surplus_count >= MAX_BOXES) return false;
            box_surplus[(*box_surplus_count)++] = id;
        }
        for (int n = b_counts[id]; n < t_counts[id]; n++) {
            if (*target_surplus_count >= MAX_TARGETS) return false;
            target_surplus[(*target_surplus_count)++] = id;
        }
    }

    return true;
}



typedef struct {
    bool initial_small_balanced_case;
    bool initial_player_in_return_entry_region;
    bool path_has_return_region_shift;
    ScanVerificationLevel verification_level;
} ScanStrategyPolicy;

typedef struct {
    bool use_x_axis;
    int entry_sum;
    int middle_sum;
} ScanReturnRegionPattern;

typedef struct {
    const Position* box_positions;
    const Entity* box_entities;
    const bool* visited_boxes;
    int box_count;
    int scanned_boxes;
    int req_boxes;
    const Position* target_positions;
    const Entity* target_entities;
    const bool* visited_targets;
    int target_count;
    int scanned_targets;
    int req_targets;
    bool current_is_box;
    int current_idx;
} ScanNeighborLookahead;

static ScanNeighborLookahead g_scan_neighbor_lookahead;
static bool is_strict_scan_obstacle(BitboardMap* bmap, int x, int y);
static inline bool scan_current_small_balanced_case(const SokobanSolver* solver) {
    return solver &&
           solver->num_boxes == SCAN_LEGACY_SMALL_BALANCED_ENTITY_COUNT &&
           solver->num_targets == SCAN_LEGACY_SMALL_BALANCED_ENTITY_COUNT &&
           solver->num_bombs == SCAN_LEGACY_SMALL_BALANCED_ENTITY_COUNT;
}

static inline bool scan_initial_small_balanced_case(const SokobanSolver* solver) {
    return solver &&
           solver->num_boxes == SCAN_LEGACY_SMALL_BALANCED_ENTITY_COUNT &&
           solver->num_targets == SCAN_LEGACY_SMALL_BALANCED_ENTITY_COUNT &&
           g_scan_initial_num_bombs == SCAN_LEGACY_SMALL_BALANCED_ENTITY_COUNT;
}


static inline bool scan_should_try_clearance(const SokobanSolver* solver) {
    return solver &&
           solver->num_boxes > 0 &&
           solver->num_boxes <= SCAN_CLEARANCE_SAFE_BOX_LIMIT;
}

static uint16_t scan_entity_view_lower_bound(
    BitboardMap* bmap,
    const uint16_t dist[MAP_ROWS][MAP_COLS],
    Position entity,
    uint16_t hard_max_len
) {
    uint16_t best = 0xFFFF;
    for (int dir = 0; dir < 4; dir++) {
        int nx = (int)entity.x + DIRECTIONS[dir].dx;
        int ny = (int)entity.y + DIRECTIONS[dir].dy;
        if (is_strict_scan_obstacle(bmap, nx, ny)) continue;
        uint16_t d = dist[ny][nx];
        if (d == 0xFFFF || d > hard_max_len) continue;
        if (d < best) best = d;
    }
    return best;
}

static bool scan_should_defer_high_lb_box_candidate(
    const SokobanSolver* solver,
    BitboardMap* bmap,
    const uint16_t dist[MAP_ROWS][MAP_COLS],
    const Position* entities,
    int entity_count,
    const bool* visited,
    int scanned_count,
    int req_count,
    Position candidate_pos,
    uint16_t hard_max_len
) {
    if (g_enable_path_verification == SCAN_VERIFY_NONE) return false;
    if (!solver || !solver->is_scanning || !scan_current_small_balanced_case(solver)) return false;

    int need_boxes = req_count - scanned_count;
    if (need_boxes <= 0) return false;

    uint16_t candidate_lb = scan_entity_view_lower_bound(bmap, dist, candidate_pos, hard_max_len);
    if (candidate_lb == 0xFFFF) return false;

    uint16_t min_lb = 0xFFFF;
    uint16_t max_lb = 0;
    int reachable_count = 0;
    for (int bi = 0; bi < entity_count; bi++) {
        if (visited[bi]) continue;
        uint16_t lb = scan_entity_view_lower_bound(bmap, dist, entities[bi], hard_max_len);
        if (lb == 0xFFFF) continue;
        reachable_count++;
        if (lb < min_lb) min_lb = lb;
        if (lb > max_lb) max_lb = lb;
    }
    if (reachable_count <= 1 || min_lb == 0xFFFF) return false;
    if (max_lb < min_lb || (uint16_t)(max_lb - min_lb) < SCAN_DEFER_HIGH_LB_TRIGGER_DELTA) return false;
    if (candidate_lb <= (uint16_t)(min_lb + SCAN_DEFER_HIGH_LB_TRIGGER_DELTA)) return false;

    int low_lb_unvisited = 0;
    for (int bi = 0; bi < entity_count; bi++) {
        if (visited[bi] || pos_equal(entities[bi], candidate_pos)) continue;
        uint16_t lb = scan_entity_view_lower_bound(bmap, dist, entities[bi], hard_max_len);
        if (lb == 0xFFFF) continue;
        if (lb <= (uint16_t)(min_lb + SCAN_DEFER_LOW_LB_SLACK)) low_lb_unvisited++;
    }

    return low_lb_unvisited >= need_boxes;
}

static void scan_spine_reset_state(void);

void sokoban_scan_reset_state(void) {
    memset(&g_scan_initial_bmap, 0, sizeof(g_scan_initial_bmap));
    memset(&g_scan_initial_player, 0, sizeof(g_scan_initial_player));
    memset(g_scan_initial_boxes, 0, sizeof(g_scan_initial_boxes));
    memset(g_scan_initial_bombs, 0, sizeof(g_scan_initial_bombs));
    g_scan_initial_num_bombs = 0;

    memset(g_scan_hybrid_path, 0, sizeof(g_scan_hybrid_path));
    memset(g_scan_verified_path, 0, sizeof(g_scan_verified_path));
    memset(g_scan_bomb_main_route_candidates, 0, sizeof(g_scan_bomb_main_route_candidates));
    memset(g_scan_bomb_best_waypoints, 0, sizeof(g_scan_bomb_best_waypoints));
    memset(g_scan_bomb_best_pauses, 0, sizeof(g_scan_bomb_best_pauses));
    memset(&g_scan_bomb_best_final_state, 0, sizeof(g_scan_bomb_best_final_state));
    g_scan_bomb_main_route_candidate_count = 0;
    g_scan_bomb_main_route_override_index = -1;
    memset(g_scan_eager_path, 0, sizeof(g_scan_eager_path));
    memset(g_scan_compact_path, 0, sizeof(g_scan_compact_path));
    memset(g_scan_eager_waypoints, 0, sizeof(g_scan_eager_waypoints));
    memset(g_scan_eager_pauses, 0, sizeof(g_scan_eager_pauses));
    memset(g_scan_eager_desired_waypoints, 0, sizeof(g_scan_eager_desired_waypoints));
    memset(g_scan_eager_emitted, 0, sizeof(g_scan_eager_emitted));
    memset(&g_scan_verified_solver, 0, sizeof(g_scan_verified_solver));
    memset(&g_scan_phase_workspace, 0, sizeof(g_scan_phase_workspace));
    memset(&g_scan_bfs_scratch, 0, sizeof(g_scan_bfs_scratch));
    memset(&g_scan_path_scratch, 0, sizeof(g_scan_path_scratch));
    memset(g_scan_nav_turn_cost, 0xFF, sizeof(g_scan_nav_turn_cost));
    memset(g_scan_nav_turn_parent, 0xFF, sizeof(g_scan_nav_turn_parent));
    memset(&g_scan_extend_scratch, 0, sizeof(g_scan_extend_scratch));
    memset(&g_scan_neighbor_lookahead, 0, sizeof(g_scan_neighbor_lookahead));
    scan_spine_reset_state();
}

/**
 * 检查位置是否为严格扫描障碍物
 */
static bool is_strict_scan_obstacle(BitboardMap* bmap, int x, int y) {
    if (x < 0 || x >= MAP_COLS || y < 0 || y >= MAP_ROWS) return true;
    uint16_t mask = (1 << x);
    uint16_t all_obstacles = bmap->walls[y] | bmap->bombs[y]  | bmap->boxes[y];
    return (all_obstacles & mask) != 0;
}

// -------------------------------------------------------------------------
// 极简清道夫引擎：专治大门被堵，绝不绕远路。
// -------------------------------------------------------------------------

FAST_OCRAM_FUNC static bool find_best_scan_neighbor_and_path_with_lookahead(
    BitboardMap* bmap,
    Position start, Position entity,
    Position* best_neighbor, Direction* best_path, uint16_t* best_len,
    uint8_t preferred_heading, uint8_t* out_scan_heading, uint16_t* out_score,
    const ScanNeighborLookahead* lookahead, uint8_t excluded_scan_heading_mask
);
FAST_OCRAM_FUNC static void build_scan_nav_field(
    BitboardMap* sim_map, Position start_p,
    uint16_t dist[MAP_ROWS][MAP_COLS], uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    Position* q
);
FAST_OCRAM_FUNC static bool reconstruct_scan_nav_path(
    const uint16_t dist[MAP_ROWS][MAP_COLS], const uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    Position target, Direction* out_path, uint16_t* out_len
);
FAST_OCRAM_FUNC static bool reconstruct_scan_nav_turn_path(
    const uint16_t dist[MAP_ROWS][MAP_COLS], Position target,
    uint8_t preferred_heading, uint8_t scan_heading,
    Direction* out_path, uint16_t* out_len
);
static uint16_t scan_turn_score(uint8_t from_dir, uint8_t to_dir) {
    return (uint16_t)(direction_quarter_turns(from_dir, to_dir) * SCAN_TURN_90_SCORE);
}

static uint16_t scan_heading_change_count(uint8_t from_dir, uint8_t to_dir) {
    if (from_dir >= 4 || to_dir >= 4 || from_dir == to_dir) return 0;
    return 1;
}

static uint16_t scan_path_bend_score(const Direction* path, uint16_t len) {
    return (uint16_t)(path_direction_bend_count(path, len) * SCAN_BEND_SLOWDOWN_SCORE);
}

static uint16_t scan_route_turn_count(const Direction* path, uint16_t len, uint8_t start_heading) {
    if (!path || len == 0) return 0;

    uint16_t turns = 0;
    uint8_t prev_dir = start_heading;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t dir = direction_index(path[i]);
        if (dir >= 4u) continue;
        if (prev_dir < 4u && dir != prev_dir) turns++;
        prev_dir = dir;
    }
    return turns;
}

static uint16_t scan_path_move_count(const Direction* path, uint16_t len) {
    if (!path || len == 0) return 0;

    uint16_t moves = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (direction_index(path[i]) < 4u) moves++;
    }
    return moves;
}


static uint32_t scan_path_balance_score(const Direction* path, uint16_t len) {
    uint32_t moves = scan_path_move_count(path, len);
    uint32_t bends = path_direction_bend_count(path, len);
    return moves * SCAN_STEP_SCORE + bends * SCAN_BEND_SLOWDOWN_SCORE;
}

typedef struct {
    uint16_t moves;
    uint16_t bends;
    uint32_t straight_run_score;
} ScanPathShape;

/* Pauses are deliberately ignored: they do not interrupt the current run. */
static ScanPathShape scan_path_shape(const Direction* path, uint16_t len) {
    ScanPathShape shape;
    memset(&shape, 0, sizeof(shape));
    if (!path || len == 0) return shape;

    uint8_t previous_dir = DIRECTION_INDEX_NONE;
    uint16_t run_len = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t dir = direction_index(path[i]);
        if (dir >= 4u) continue;
        shape.moves++;
        if (previous_dir >= 4u || dir == previous_dir) {
            run_len++;
        } else {
            shape.straight_run_score += (uint32_t)run_len * run_len;
            shape.bends++;
            run_len = 1;
        }
        previous_dir = dir;
    }
    if (run_len > 0) {
        shape.straight_run_score += (uint32_t)run_len * run_len;
    }
    return shape;
}

static uint32_t scan_path_weighted_cost(const Direction* path, uint16_t len) {
    ScanPathShape shape = scan_path_shape(path, len);
    return (uint32_t)shape.moves * SCAN_STEP_SCORE +
           (uint32_t)shape.bends * SCAN_BEND_SLOWDOWN_SCORE;
}

static bool scan_path_continuity_is_better(
    const ScanPathShape* candidate, const ScanPathShape* baseline
) {
    if (!candidate || !baseline) return false;
    if (candidate->moves == 0 || baseline->moves == 0) {
        return candidate->moves > baseline->moves;
    }
    uint64_t candidate_scaled = (uint64_t)candidate->straight_run_score *
                                baseline->moves * baseline->moves;
    uint64_t baseline_scaled = (uint64_t)baseline->straight_run_score *
                               candidate->moves * candidate->moves;
    return candidate_scaled > baseline_scaled;
}

/* Stable generation order wins an exact tie; do not reshuffle equivalent plans. */
static bool scan_path_is_better(
    const Direction* candidate, uint16_t candidate_len,
    const Direction* baseline, uint16_t baseline_len
) {
    if (!candidate || !baseline) return false;
    uint32_t candidate_cost = scan_path_weighted_cost(candidate, candidate_len);
    uint32_t baseline_cost = scan_path_weighted_cost(baseline, baseline_len);
    if (candidate_cost != baseline_cost) return candidate_cost < baseline_cost;

    ScanPathShape candidate_shape = scan_path_shape(candidate, candidate_len);
    ScanPathShape baseline_shape = scan_path_shape(baseline, baseline_len);
    if (candidate_shape.moves != 0 && baseline_shape.moves != 0) {
        uint64_t candidate_scaled = (uint64_t)candidate_shape.straight_run_score *
                                    baseline_shape.moves * baseline_shape.moves;
        uint64_t baseline_scaled = (uint64_t)baseline_shape.straight_run_score *
                                   candidate_shape.moves * candidate_shape.moves;
        if (candidate_scaled != baseline_scaled) {
            return scan_path_continuity_is_better(&candidate_shape, &baseline_shape);
        }
    } else if (candidate_shape.moves != baseline_shape.moves) {
        return candidate_shape.moves > baseline_shape.moves;
    }
    if (candidate_shape.moves != baseline_shape.moves) {
        return candidate_shape.moves < baseline_shape.moves;
    }
    return false;
}

static bool scan_beam_should_replace_greedy(
    const Direction* beam_path, uint16_t beam_len,
    const Direction* greedy_path, uint16_t greedy_len
) {
    uint16_t beam_moves = scan_path_move_count(beam_path, beam_len);
    uint16_t greedy_moves = scan_path_move_count(greedy_path, greedy_len);
    uint16_t beam_bends = path_direction_bend_count(beam_path, beam_len);
    uint16_t greedy_bends = path_direction_bend_count(greedy_path, greedy_len);

    if (beam_moves >= greedy_moves && beam_bends >= greedy_bends &&
        (beam_moves > greedy_moves || beam_bends > greedy_bends)) {
        return false;
    }

    uint32_t beam_score = scan_path_balance_score(beam_path, beam_len);
    uint32_t greedy_score = scan_path_balance_score(greedy_path, greedy_len);
    if (beam_score != greedy_score) return beam_score < greedy_score;
    if (beam_bends != greedy_bends) return beam_bends < greedy_bends;
    return beam_moves <= greedy_moves;
}
static uint16_t scan_route_time_score(uint16_t path_len, const Direction* path, uint8_t start_heading) {
    uint32_t score = (uint32_t)path_len * SCAN_STEP_SCORE;
    if (path_len > 0) {
        score += scan_turn_score(start_heading, path_first_direction_index(path, path_len, start_heading));
        score += scan_path_bend_score(path, path_len);
    }
    return (score > 0xFFFEu) ? 0xFFFEu : (uint16_t)score;
}

static uint16_t scan_time_score(
    uint16_t path_len, const Direction* path,
    uint8_t preferred_heading, uint8_t scan_heading
) {
    uint32_t score = scan_route_time_score(path_len, path, preferred_heading);
    score += scan_turn_score(path_end_direction_index(path, path_len, preferred_heading), scan_heading);
    return (score > 0xFFFEu) ? 0xFFFEu : (uint16_t)score;
}


static bool scan_line_clear_between(BitboardMap* bmap, Position a, Position b) {
    if (a.x == b.x) {
        int step = (b.y > a.y) ? 1 : -1;
        for (int y = (int)a.y + step; y != (int)b.y; y += step) {
            if (is_strict_scan_obstacle(bmap, a.x, y)) return false;
        }
        return true;
    }

    if (a.y == b.y) {
        int step = (b.x > a.x) ? 1 : -1;
        for (int x = (int)a.x + step; x != (int)b.x; x += step) {
            if (is_strict_scan_obstacle(bmap, x, a.y)) return false;
        }
        return true;
    }

    return false;
}

static Position scan_lookahead_position(const Position* positions, const Entity* entities, int idx) {
    if (positions) return positions[idx];
    return entities[idx].pos;
}

FAST_OCRAM_FUNC static uint16_t scan_followup_entity_score(
    BitboardMap* bmap, Position view_pos, uint8_t heading_after_scan, Position entity
) {
    uint16_t best = 0xFFFF;

    for (int dir = 0; dir < 4; dir++) {
        int nx = (int)entity.x + DIRECTIONS[dir].dx;
        int ny = (int)entity.y + DIRECTIONS[dir].dy;
        if (is_strict_scan_obstacle(bmap, nx, ny)) continue;

        Position next_view = {(uint8_t)nx, (uint8_t)ny};
        uint8_t next_scan_heading = direction_index_between(next_view, entity);
        if (pos_equal(view_pos, next_view)) {
            uint16_t same_pos_score = scan_turn_score(heading_after_scan, next_scan_heading);
            if (same_pos_score < best) best = same_pos_score;
            continue;
        }
        uint16_t dist = manhattan_distance(view_pos, next_view);
        uint8_t travel_heading = direction_axis_index(view_pos, next_view);
        uint32_t score = (uint32_t)dist * SCAN_STEP_SCORE;

        if (travel_heading < 4) {
            if (!scan_line_clear_between(bmap, view_pos, next_view)) {
                score += SCAN_VIEW_LOOKAHEAD_BLOCKED_LINE_PENALTY;
            }
            score += scan_turn_score(heading_after_scan, travel_heading);
            score += scan_turn_score(travel_heading, next_scan_heading);
        } else {
            score += SCAN_VIEW_LOOKAHEAD_CORNER_PENALTY;
        }

        if (score < best) best = (score > 0xFFFEu) ? 0xFFFEu : (uint16_t)score;
    }

    return best;
}

static uint16_t scan_view_followup_score(
    BitboardMap* bmap, Position view_pos, uint8_t heading_after_scan,
    const ScanNeighborLookahead* lookahead
) {
    if (!lookahead) return 0xFFFF;

    uint16_t best = 0xFFFF;
    int after_boxes = lookahead->scanned_boxes + (lookahead->current_is_box ? 1 : 0);
    int after_targets = lookahead->scanned_targets + (!lookahead->current_is_box ? 1 : 0);

    if ((lookahead->box_positions || lookahead->box_entities) && lookahead->visited_boxes && after_boxes < lookahead->req_boxes) {
        for (int i = 0; i < lookahead->box_count; i++) {
            if (lookahead->visited_boxes[i]) continue;
            if (lookahead->current_is_box && i == lookahead->current_idx) continue;
            Position entity = scan_lookahead_position(lookahead->box_positions, lookahead->box_entities, i);
            uint16_t score = scan_followup_entity_score(bmap, view_pos, heading_after_scan, entity);
            if (score < best) best = score;
        }
    }

    if ((lookahead->target_positions || lookahead->target_entities) && lookahead->visited_targets && after_targets < lookahead->req_targets) {
        for (int i = 0; i < lookahead->target_count; i++) {
            if (lookahead->visited_targets[i]) continue;
            if (!lookahead->current_is_box && i == lookahead->current_idx) continue;
            Position entity = scan_lookahead_position(lookahead->target_positions, lookahead->target_entities, i);
            uint16_t score = scan_followup_entity_score(bmap, view_pos, heading_after_scan, entity);
            if (score < best) best = score;
        }
    }

    return best;
}

static bool scan_rank_is_better(
    uint16_t candidate_len, uint16_t candidate_score,
    uint16_t best_len, uint16_t best_score
) {
    if (best_score == 0xFFFF) return true;
    if (candidate_score < best_score) return true;
    if (candidate_score == best_score && candidate_len < best_len) return true;
    return false;
}

static bool scan_path_pushes_box_to_target(
    const BitboardMap* bmap, Position start,
    const Direction* path, uint16_t path_len
) {
    if (!bmap || !path) return true;

    BitboardMap sim_map = *bmap;
    Position sim_p = start;
    for (uint16_t i = 0; i < path_len; i++) {
        Direction d = path[i];
        if (d.dx == 0 && d.dy == 0) continue;

        int nx = (int)sim_p.x + d.dx;
        int ny = (int)sim_p.y + d.dy;
        if (!is_in_bounds(nx, ny)) return true;
        Position next_p = {(uint8_t)nx, (uint8_t)ny};

        if (get_bit(sim_map.boxes, next_p.x, next_p.y)) {
            int bx = nx + d.dx;
            int by = ny + d.dy;
            if (!is_in_bounds(bx, by)) return true;
            Position next_box = {(uint8_t)bx, (uint8_t)by};

            if (get_bit(sim_map.targets, next_box.x, next_box.y)) return true;
            if ((sim_map.walls[next_box.y] | sim_map.bombs[next_box.y] | sim_map.boxes[next_box.y]) & (1u << next_box.x)) {
                return true;
            }
            move_box_bit(&sim_map, next_p, next_box);
        }

        sim_p = next_p;
    }

    return false;
}
FAST_OCRAM_FUNC static bool choose_best_scan_neighbor_from_field(
    BitboardMap* bmap,
    const uint16_t dist[MAP_ROWS][MAP_COLS], const uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    Position entity, uint16_t hard_max_len, uint8_t preferred_heading,
    Position* best_neighbor, Direction* best_path, uint16_t* best_len,
    uint8_t* out_scan_heading, uint16_t* out_score,
    const ScanNeighborLookahead* lookahead, uint8_t excluded_scan_heading_mask
) {

    bool found = false;
    uint16_t best_score = 0xFFFF;
    uint16_t best_followup = 0xFFFF;
    uint16_t best_route_turns = 0xFFFF;
    uint16_t best_scan_heading_changes = 0xFFFF;
    uint16_t chosen_len = 0xFFFF;
    uint8_t chosen_heading = DIRECTION_INDEX_NONE;
    Position chosen = {0, 0};
    Direction* candidate_path = g_scan_choose_candidate_path;

    for (int dir = 0; dir < 4; dir++) {
        int nx = (int)entity.x + DIRECTIONS[dir].dx;
        int ny = (int)entity.y + DIRECTIONS[dir].dy;
        if (is_strict_scan_obstacle(bmap, nx, ny)) continue;
        if (dist[ny][nx] == 0xFFFF || dist[ny][nx] > hard_max_len) continue;

        Position candidate = {(uint8_t)nx, (uint8_t)ny};
        uint8_t scan_heading = direction_index_between(candidate, entity);
        if (scan_heading < 4u &&
            (excluded_scan_heading_mask & (uint8_t)(1u << scan_heading)) != 0u) {
            continue;
        }
        uint16_t candidate_len = 0;
        if (!reconstruct_scan_nav_path(dist, parent_dir, candidate, candidate_path, &candidate_len)) continue;

        uint8_t candidate_heading = path_end_direction_index(candidate_path, candidate_len, preferred_heading);
        uint16_t candidate_score = scan_time_score(candidate_len, candidate_path, preferred_heading, candidate_heading);
        uint16_t candidate_followup = scan_view_followup_score(bmap, candidate, candidate_heading, lookahead);
        uint16_t candidate_route_turns = scan_route_turn_count(candidate_path, candidate_len, preferred_heading);
        uint16_t candidate_scan_heading_changes = scan_heading_change_count(preferred_heading, candidate_heading);
        bool candidate_better = false;
        bool lookahead_ranked = false;
        if (lookahead && best_score != 0xFFFF && candidate_followup != 0xFFFF && best_followup != 0xFFFF) {
            uint32_t candidate_two_step = (uint32_t)candidate_score + candidate_followup;
            uint32_t best_two_step = (uint32_t)best_score + best_followup;
            lookahead_ranked = true;
            if (candidate_two_step < best_two_step) {
                candidate_better = true;
            } else if (candidate_two_step == best_two_step) {
                candidate_better = scan_rank_is_better(candidate_len, candidate_score, chosen_len, best_score);
            }
        }
        if (!lookahead_ranked) {
            candidate_better = scan_rank_is_better(candidate_len, candidate_score, chosen_len, best_score);
        }
        if (!candidate_better && best_score != 0xFFFF && candidate_score == best_score && candidate_len == chosen_len) {
            if (candidate_scan_heading_changes < best_scan_heading_changes) candidate_better = true;
        }
        if (!candidate_better && lookahead && best_score != 0xFFFF && candidate_followup < best_followup) {
            uint16_t score_delta = (candidate_score > best_score) ? (uint16_t)(candidate_score - best_score) : 0;
            uint16_t followup_gain = (best_followup == 0xFFFF)
                ? 0xFFFF
                : (uint16_t)(best_followup - candidate_followup);
            if ((candidate_route_turns <= best_route_turns && score_delta <= SCAN_VIEW_LOOKAHEAD_SCORE_SLACK) ||
                (followup_gain > score_delta && candidate_followup <= SCAN_VIEW_LOOKAHEAD_CORNER_PENALTY)) {
                candidate_better = true;
            }
        }
        if (candidate_better) {
            found = true;
            best_score = candidate_score;
            best_followup = candidate_followup;
            best_route_turns = candidate_route_turns;
            best_scan_heading_changes = candidate_scan_heading_changes;
            chosen_len = candidate_len;
            chosen_heading = candidate_heading;
            chosen = candidate;
            if (candidate_len > 0) memcpy(best_path, candidate_path, candidate_len * sizeof(Direction));
        }
    }

    if (!found) return false;
    if (chosen_len > 0) {
        uint16_t turn_len = 0;
        if (reconstruct_scan_nav_turn_path(
                dist, chosen, preferred_heading, chosen_heading,
                candidate_path, &turn_len
            ) && turn_len == chosen_len) {
            uint16_t old_turns = scan_route_turn_count(best_path, chosen_len, preferred_heading);
            uint16_t new_turns = scan_route_turn_count(candidate_path, turn_len, preferred_heading);
            if (new_turns < old_turns) {
                memcpy(best_path, candidate_path, turn_len * sizeof(Direction));
            }
        }
    }
    *best_neighbor = chosen;
    *best_len = chosen_len;
    if (out_scan_heading) *out_scan_heading = chosen_heading;
    if (out_score) *out_score = best_score;
    return true;
}
static bool clear_blocking_box_for_scan(
    SokobanSolver* solver, BitboardMap* bmap, Position* current_pos,
    bool* visited_boxes, bool* visited_targets,
    Direction* out_path, uint16_t* total_len,
    uint16_t min_dist_threshold
) {
    
    ScanClearBfsScratch* clear = &g_scan_bfs_scratch.work.clear_bfs;
    uint16_t (*visited)[MAP_COLS] = clear->visited;
    uint16_t (*dist)[MAP_COLS] = clear->dist;
    Position (*parent)[MAP_COLS] = clear->parent;
    Position* q = clear->queue;
    memset(clear->visited, 0, sizeof(clear->visited));
    memset(clear->dist, 0xFF, sizeof(clear->dist));

    int head = 0, tail = 0;
    q[tail++] = *current_pos;
    visited[current_pos->y][current_pos->x] = 1;
    dist[current_pos->y][current_pos->x] = 0;

    Position target_entity_pos = {255, 255};
    bool found_entity = false;

    // 1. 通过广度优先搜索找到最近的未扫描实体。
    while (head < tail) {
        Position curr = q[head++];
        uint16_t curr_d = dist[curr.y][curr.x];

        bool is_unscanned = false;
        for (int i = 0; i < solver->num_boxes; i++) {
            if (!visited_boxes[i] && pos_equal(curr, solver->boxes[i].pos)) { is_unscanned = true; break; }
        }
        if (!is_unscanned) {
            for (int i = 0; i < solver->num_targets; i++) {
                if (!visited_targets[i] && pos_equal(curr, solver->targets[i].pos)) { is_unscanned = true; break; }
            }
        }

        if (is_unscanned) {
            // 抄近道判定：如果踹门所需的代价（距离+惩罚）比外面的目标还要远，就放弃
            if (curr_d + 10 >= min_dist_threshold) return false;
            target_entity_pos = curr;
            found_entity = true;
            break;
        }

        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx, ny = curr.y + DIRECTIONS[d].dy;
            if (nx > 0 && nx < MAP_COLS - 1 && ny > 0 && ny < MAP_ROWS - 1) {
                // 普通可通行格继续扩展。
                if (!visited[ny][nx] && ((bmap->walls[ny] & (1 << nx)) == 0) && ((bmap->bombs[ny] & (1 << nx)) == 0)) {
                    visited[ny][nx] = 1;
                    dist[ny][nx] = curr_d + 1;
                    parent[ny][nx] = curr;
                    q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
                }
            }
        }
    }

    if (!found_entity) return false;

    Position blocker_box = {255, 255};
    {
        Position cursor = target_entity_pos;
        while (!pos_equal(cursor, *current_pos)) {
            if ((bmap->boxes[cursor.y] & (1 << cursor.x)) != 0) {
                blocker_box = cursor;
            }
            cursor = parent[cursor.y][cursor.x];
        }
    }

    if (blocker_box.x == 255) return false;

    Position* candidates = g_scan_bfs_scratch.clear_candidates;
    int num_candidates = 0;
    for (int y = 1; y < MAP_ROWS - 1; y++) {
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if (x == blocker_box.x && y == blocker_box.y) continue;
            if (((bmap->walls[y] | bmap->bombs[y] | bmap->boxes[y]) & (1 << x)) == 0) {
                if ((bmap->targets[y] & (1 << x)) != 0) continue;
                if ((bmap->deadlocks[y] & (1 << x)) != 0) continue;
                candidates[num_candidates++] = (Position){(uint8_t)x, (uint8_t)y};
            }
        }
    }

    for (int i = 0; i < num_candidates - 1; i++) {
        for (int j = i + 1; j < num_candidates; j++) {
            int di = abs(blocker_box.x - candidates[i].x) + abs(blocker_box.y - candidates[i].y);
            int dj = abs(blocker_box.x - candidates[j].x) + abs(blocker_box.y - candidates[j].y);
            if (di > dj) {
                Position tmp = candidates[i]; candidates[i] = candidates[j]; candidates[j] = tmp;
            }
        }
    }

    bool best_found = false;
    Direction* best_push_path = g_scan_clear_best_push_path;
    uint16_t best_push_len = 0xFFFF;

    int test_limit = (num_candidates < SCAN_CLEARANCE_CANDIDATE_LIMIT) ? num_candidates : SCAN_CLEARANCE_CANDIDATE_LIMIT;
    for (int i = 0; i < test_limit; i++) {
        BitboardMap temp_map = *bmap;
        clear_bit(temp_map.boxes, blocker_box.x, blocker_box.y);

        uint16_t backup_max_steps = g_astar_max_steps;
        g_astar_max_steps = best_push_len;
        Direction* push_path = g_scan_clear_push_path;
        uint16_t push_len = 0;
        bool can_push = scan_cached_astar_single_box(
            solver, &temp_map,
            *current_pos, blocker_box, candidates[i],
            MASK_WALL | MASK_BOMB | MASK_BOX,
            push_path, &push_len, ASTAR_NO_MACRO_DEPTH, ROUTE_BOX_NORMAL
        );
        g_astar_max_steps = backup_max_steps;

        if (can_push && push_len < best_push_len) {
            if (scan_path_pushes_box_to_target(bmap, *current_pos, push_path, push_len)) continue;
            Position sim_p = *current_pos;
            for (int s = 0; s < push_len; s++) {
                sim_p.x += push_path[s].dx;
                sim_p.y += push_path[s].dy;
            }

            set_bit(temp_map.boxes, candidates[i].x, candidates[i].y);

            // 验证：箱子推到这里后，小车能抵达目标实体旁边的合法观察位。
            Position temp_view;
            Direction* nav_path = g_scan_clear_nav_path;
            uint16_t nav_len = 0;
            bool can_nav = find_best_scan_neighbor_and_path_with_lookahead(
                &temp_map, sim_p, target_entity_pos,
                &temp_view, nav_path, &nav_len,
                DIRECTION_INDEX_NONE, NULL, NULL, NULL, 0u
            );

            if (can_nav) {
                best_found = true;
                best_push_len = push_len;
                memcpy(best_push_path, push_path, push_len * sizeof(Direction));
            }
        }
    }

    if (!best_found) return false;
    if (*total_len + best_push_len >= MAX_PATH_LENGTH) return false;

    // 采纳最优的最短直推方案
    memcpy(&out_path[*total_len], best_push_path, best_push_len * sizeof(Direction));
    
    
    *total_len += best_push_len;

    Position sim_p = *current_pos;
    for (int i = 0; i < best_push_len; i++) {
        Direction d = best_push_path[i];
        Position next_p = {sim_p.x + d.dx, sim_p.y + d.dy};
        if (get_bit(bmap->boxes, next_p.x, next_p.y)) {
            Position next_box = {next_p.x + d.dx, next_p.y + d.dy};
            move_box_bit(bmap, next_p, next_box);
            for (int b = 0; b < solver->num_boxes; b++) {
                if (pos_equal(solver->boxes[b].pos, next_p)) {
                    solver->boxes[b].pos = next_box;
                    break;
                }
            }
        }
        sim_p = next_p;
    }

    *current_pos = sim_p;
    solver_refresh_deadlocks(solver);
    return true;
}
/**
 * 评估实体周围的合法方向，寻找最短扫描站位。
 * 使用一次 BFS 距离场统一评估候选观察位，避免重复调用 A*。
 */
FAST_OCRAM_FUNC static bool find_best_scan_neighbor_and_path_with_lookahead(
    BitboardMap* bmap,
    Position start, Position entity,
    Position* best_neighbor, Direction* best_path, uint16_t* best_len,
    uint8_t preferred_heading, uint8_t* out_scan_heading, uint16_t* out_score,
    const ScanNeighborLookahead* lookahead, uint8_t excluded_scan_heading_mask
) {
    ScanNavScratch* nav = &g_scan_bfs_scratch.work.nav;
    uint16_t (*dist)[MAP_COLS] = nav->dist;
    uint8_t (*parent_dir)[MAP_COLS] = nav->parent_dir;
    build_scan_nav_field(bmap, start, dist, parent_dir, nav->queue);

    return choose_best_scan_neighbor_from_field(
        bmap, dist, parent_dir, entity, 0xFFFFu, preferred_heading,
        best_neighbor, best_path, best_len, out_scan_heading, out_score,
        lookahead, excluded_scan_heading_mask
    );
}

// Beam planner helpers for paired scan tasks.
static bool scan_beam_state_is_better(const ScanBeamState* candidate, const ScanBeamState* best) {
    if (!candidate || !candidate->valid) return false;
    if (!best || !best->valid) return true;
    if (candidate->score < best->score) return true;
    if (candidate->score > best->score) return false;
    if (candidate->len < best->len) return true;
    if (candidate->len > best->len) return false;
    return candidate->waypoint_count > best->waypoint_count;
}

static bool scan_beam_entry_is_better(const ScanBeamEntryPath* candidate, const ScanBeamEntryPath* best) {
    if (!candidate || !candidate->valid) return false;
    if (!best || !best->valid) return true;
    if (candidate->score < best->score) return true;
    if (candidate->score > best->score) return false;
    if (candidate->len < best->len) return true;
    if (candidate->len > best->len) return false;
    return false;
}

static bool scan_beam_same_state_key(const ScanBeamState* a, const ScanBeamState* b) {
    return a && b &&
           a->remaining_mask == b->remaining_mask &&
           a->heading == b->heading &&
           pos_equal(a->pos, b->pos);
}

static void scan_beam_insert_entry(ScanBeamEntryPath entries[SCAN_PAIR_BEAM_ENTRY_TOP_K], const ScanBeamEntryPath* candidate) {
    if (!candidate || !candidate->valid) return;

    int replace_idx = -1;
    for (int i = 0; i < SCAN_PAIR_BEAM_ENTRY_TOP_K; i++) {
        if (!entries[i].valid) {
            if (replace_idx < 0) replace_idx = i;
            continue;
        }
        if (entries[i].heading == candidate->heading && pos_equal(entries[i].view_pos, candidate->view_pos)) {
            if (scan_beam_entry_is_better(candidate, &entries[i])) entries[i] = *candidate;
            return;
        }
    }

    if (replace_idx < 0) {
        replace_idx = 0;
        for (int i = 1; i < SCAN_PAIR_BEAM_ENTRY_TOP_K; i++) {
            if (scan_beam_entry_is_better(&entries[replace_idx], &entries[i])) replace_idx = i;
        }
        if (!scan_beam_entry_is_better(candidate, &entries[replace_idx])) return;
    }

    entries[replace_idx] = *candidate;
}

static int scan_beam_collect_entries_from_field(
    BitboardMap* bmap,
    const uint16_t dist[MAP_ROWS][MAP_COLS],
    const uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    Position start,
    Position entity,
    uint16_t hard_max_len,
    uint8_t preferred_heading,
    int radius,
    ScanBeamEntryPath entries[SCAN_PAIR_BEAM_ENTRY_TOP_K]
) {
    if (!bmap || !entries) return 0;
    memset(entries, 0, sizeof(ScanBeamEntryPath) * SCAN_PAIR_BEAM_ENTRY_TOP_K);
    if (radius >= 0 && manhattan_distance(start, entity) > radius) return 0;

    Direction* turn_path = g_scan_choose_candidate_path;
    for (int dir = 0; dir < 4; dir++) {
        int nx = (int)entity.x + DIRECTIONS[dir].dx;
        int ny = (int)entity.y + DIRECTIONS[dir].dy;
        if (is_strict_scan_obstacle(bmap, nx, ny)) continue;
        if (dist[ny][nx] == 0xFFFF || dist[ny][nx] > hard_max_len) continue;

        ScanBeamEntryPath candidate;
        memset(&candidate, 0, sizeof(candidate));
        candidate.view_pos = (Position){(uint8_t)nx, (uint8_t)ny};
        if (!reconstruct_scan_nav_path(dist, parent_dir, candidate.view_pos, candidate.path, &candidate.len)) continue;

        candidate.heading = path_end_direction_index(candidate.path, candidate.len, preferred_heading);
        candidate.score = scan_route_time_score(candidate.len, candidate.path, preferred_heading);
        candidate.valid = true;

        if (candidate.len > 0) {
            uint16_t turn_len = 0;
            if (reconstruct_scan_nav_turn_path(
                    dist, candidate.view_pos, preferred_heading, candidate.heading,
                    turn_path, &turn_len
                ) && turn_len == candidate.len) {
                uint16_t turn_score = scan_route_time_score(turn_len, turn_path, preferred_heading);
                if (turn_score <= candidate.score) {
                    memcpy(candidate.path, turn_path, turn_len * sizeof(Direction));
                    candidate.score = turn_score;
                    candidate.heading = path_end_direction_index(candidate.path, candidate.len, preferred_heading);
                }
            }
        }

        scan_beam_insert_entry(entries, &candidate);
    }

    int count = 0;
    for (int i = 0; i < SCAN_PAIR_BEAM_ENTRY_TOP_K; i++) {
        if (entries[i].valid) count++;
    }
    return count;
}

static bool scan_collect_paired_tasks(
    const SokobanSolver* solver,
    const bool* visited_boxes,
    const bool* visited_targets,
    int target_box_count,
    int target_target_count,
    ScanPairedTask* tasks,
    int* out_task_count,
    int* out_tasks_needed
) {
    if (!solver || !visited_boxes || !visited_targets || !tasks || !out_task_count || !out_tasks_needed) return false;
    if (!solver->strict_target_mode || solver->num_boxes != solver->num_targets) return false;

    int boxes_scanned = 0;
    int targets_scanned = 0;
    for (int i = 0; i < solver->num_boxes; i++) if (visited_boxes[i]) boxes_scanned++;
    for (int i = 0; i < solver->num_targets; i++) if (visited_targets[i]) targets_scanned++;

    int boxes_needed = target_box_count - boxes_scanned;
    int targets_needed = target_target_count - targets_scanned;
    if (boxes_needed <= 0 || boxes_needed != targets_needed || boxes_needed > MAX_BOXES) return false;

    int box_for_id[10];
    int target_for_id[10];
    for (int i = 0; i < 10; i++) {
        box_for_id[i] = -1;
        target_for_id[i] = -1;
    }

    for (int i = 0; i < solver->num_boxes; i++) {
        int id = solver->boxes[i].id;
        if (id < 0 || id >= 10) return false;
        if (box_for_id[id] >= 0) return false;
        box_for_id[id] = i;
    }

    for (int i = 0; i < solver->num_targets; i++) {
        int id = solver->targets[i].id;
        if (id < 0 || id >= 10) return false;
        if (target_for_id[id] >= 0) return false;
        target_for_id[id] = i;
    }

    int task_count = 0;
    for (int id = 0; id < 10; id++) {
        int box_idx = box_for_id[id];
        int target_idx = target_for_id[id];
        if ((box_idx < 0) != (target_idx < 0)) return false;
        if (box_idx < 0) continue;

        bool box_done = visited_boxes[box_idx];
        bool target_done = visited_targets[target_idx];
        if (box_done && target_done) continue;
        if (box_done != target_done) return false;
        if (task_count >= MAX_BOXES) return false;

        tasks[task_count].box_idx = box_idx;
        tasks[task_count].target_idx = target_idx;
        task_count++;
    }

    if (task_count < boxes_needed) return false;
    *out_task_count = task_count;
    *out_tasks_needed = boxes_needed;
    return true;
}

static void scan_beam_insert_state(ScanBeamState* states, int* state_count, const ScanBeamState* candidate) {
    if (!states || !state_count || !candidate || !candidate->valid) return;

    for (int i = 0; i < *state_count; i++) {
        if (scan_beam_same_state_key(&states[i], candidate)) {
            if (scan_beam_state_is_better(candidate, &states[i])) states[i] = *candidate;
            return;
        }
    }

    if (*state_count < SCAN_PAIR_BEAM_STATE_LIMIT) {
        states[(*state_count)++] = *candidate;
        return;
    }

    int worst_idx = 0;
    for (int i = 1; i < *state_count; i++) {
        if (scan_beam_state_is_better(&states[worst_idx], &states[i])) worst_idx = i;
    }
    if (scan_beam_state_is_better(candidate, &states[worst_idx])) states[worst_idx] = *candidate;
}

static void scan_beam_sort_and_prune(ScanBeamState* states, int* state_count) {
    if (!states || !state_count) return;
    for (int i = 0; i < *state_count - 1; i++) {
        int best_idx = i;
        for (int j = i + 1; j < *state_count; j++) {
            if (scan_beam_state_is_better(&states[j], &states[best_idx])) best_idx = j;
        }
        if (best_idx != i) {
            g_scan_beam_scratch.candidate = states[i];
            states[i] = states[best_idx];
            states[best_idx] = g_scan_beam_scratch.candidate;
        }
    }
    if (*state_count > SCAN_PAIR_BEAM_WIDTH) *state_count = SCAN_PAIR_BEAM_WIDTH;
}

static bool scan_beam_build_candidate(
    const ScanBeamState* state,
    const ScanPairedTask* task,
    const Position* box_positions,
    const Position* target_positions,
    const ScanBeamEntryPath* first,
    const ScanBeamEntryPath* second,
    bool first_is_box,
    uint16_t next_mask,
    ScanBeamState* out_state
) {
    if (!state || !task || !box_positions || !target_positions || !first || !second || !out_state) return false;
    if (!first->valid || !second->valid) return false;
    if (state->waypoint_count + 2 > MAX_BOXES + MAX_TARGETS) return false;

    uint32_t next_len = (uint32_t)state->len + first->len + second->len + 2u;
    if (next_len >= MAX_PATH_LENGTH) return false;

    *out_state = *state;
    out_state->remaining_mask = next_mask;

    if (first->len > 0) {
        memcpy(&out_state->path[out_state->len], first->path, first->len * sizeof(Direction));
        out_state->len = (uint16_t)(out_state->len + first->len);
    }
    out_state->path[out_state->len].dx = 0;
    out_state->path[out_state->len].dy = 0;
    out_state->len++;

    int first_idx = first_is_box ? task->box_idx : task->target_idx;
    out_state->waypoints[out_state->waypoint_count].pos = first_is_box ? box_positions[first_idx] : target_positions[first_idx];
    out_state->waypoints[out_state->waypoint_count].id = first_is_box ? scan_waypoint_box_tag(first_idx) : scan_waypoint_target_tag(first_idx);
    out_state->waypoints[out_state->waypoint_count].is_active = true;
    out_state->pauses[out_state->waypoint_count] = first->view_pos;
    if (first_is_box) out_state->visited_boxes[first_idx] = true;
    else out_state->visited_targets[first_idx] = true;
    out_state->waypoint_count++;

    if (second->len > 0) {
        memcpy(&out_state->path[out_state->len], second->path, second->len * sizeof(Direction));
        out_state->len = (uint16_t)(out_state->len + second->len);
    }
    out_state->path[out_state->len].dx = 0;
    out_state->path[out_state->len].dy = 0;
    out_state->len++;

    int second_idx = first_is_box ? task->target_idx : task->box_idx;
    out_state->waypoints[out_state->waypoint_count].pos = first_is_box ? target_positions[second_idx] : box_positions[second_idx];
    out_state->waypoints[out_state->waypoint_count].id = first_is_box ? scan_waypoint_target_tag(second_idx) : scan_waypoint_box_tag(second_idx);
    out_state->waypoints[out_state->waypoint_count].is_active = true;
    out_state->pauses[out_state->waypoint_count] = second->view_pos;
    if (first_is_box) out_state->visited_targets[second_idx] = true;
    else out_state->visited_boxes[second_idx] = true;
    out_state->waypoint_count++;

    uint32_t next_score = (uint32_t)state->score + first->score + second->score;
    out_state->score = (next_score > 0xFFFEu) ? 0xFFFEu : (uint16_t)next_score;
    out_state->pos = second->view_pos;
    out_state->heading = second->heading;
    out_state->valid = true;
    return true;
}

static void scan_beam_expand_task_order(
    BitboardMap* bmap,
    const Position* box_positions,
    const Position* target_positions,
    const ScanBeamState* state,
    const ScanPairedTask* task,
    int task_idx,
    bool first_is_box,
    int radius,
    uint16_t hard_max_len,
    ScanBeamState* next_states,
    int* next_count
) {
    if (!bmap || !box_positions || !target_positions || !state || !task || !next_states || !next_count) return;

    Position first_entity = first_is_box ? box_positions[task->box_idx] : target_positions[task->target_idx];
    Position second_entity = first_is_box ? target_positions[task->target_idx] : box_positions[task->box_idx];

    ScanNavScratch* nav = &g_scan_bfs_scratch.work.nav;
    uint16_t (*dist)[MAP_COLS] = nav->dist;
    uint8_t (*parent_dir)[MAP_COLS] = nav->parent_dir;
    build_scan_nav_field(bmap, state->pos, dist, parent_dir, nav->queue);

    int first_count = scan_beam_collect_entries_from_field(
        bmap, dist, parent_dir, state->pos, first_entity, hard_max_len, state->heading,
        radius, g_scan_beam_scratch.first_entries
    );
    if (first_count <= 0) return;

    for (int i = 0; i < SCAN_PAIR_BEAM_ENTRY_TOP_K; i++) {
        ScanBeamEntryPath* first = &g_scan_beam_scratch.first_entries[i];
        if (!first->valid) continue;

        build_scan_nav_field(bmap, first->view_pos, dist, parent_dir, nav->queue);
        int second_count = scan_beam_collect_entries_from_field(
            bmap, dist, parent_dir, first->view_pos, second_entity, hard_max_len, first->heading,
            radius, g_scan_beam_scratch.second_entries
        );
        if (second_count <= 0) continue;

        for (int j = 0; j < SCAN_PAIR_BEAM_ENTRY_TOP_K; j++) {
            ScanBeamEntryPath* second = &g_scan_beam_scratch.second_entries[j];
            if (!second->valid) continue;

            uint16_t next_mask = (uint16_t)(state->remaining_mask & (uint16_t)(~(1u << task_idx)));
            if (scan_beam_build_candidate(
                    state, task, box_positions, target_positions,
                    first, second, first_is_box, next_mask,
                    &g_scan_beam_scratch.candidate
                )) {
                scan_beam_insert_state(next_states, next_count, &g_scan_beam_scratch.candidate);
            }
        }
    }
}

static bool scan_plan_paired_tasks_beam(
    SokobanSolver* solver,
    BitboardMap* bmap,
    Position start_pos,
    uint8_t start_heading,
    const Position* box_positions,
    const Position* target_positions,
    bool* visited_boxes,
    bool* visited_targets,
    int target_box_count,
    int target_target_count,
    int radius,
    int max_segment_len,
    Direction* out_path,
    uint16_t* out_len,
    Entity* out_waypoints,
    Position* out_pause_positions,
    int* out_waypoint_count,
    Position* out_final_pos,
    uint8_t* out_final_heading
) {
    if (!solver || !bmap || !box_positions || !target_positions || !visited_boxes || !visited_targets ||
        !out_path || !out_len || !out_waypoints || !out_pause_positions || !out_waypoint_count || !out_final_pos) {
        return false;
    }

    int task_count = 0;
    int tasks_needed = 0;
    if (!scan_collect_paired_tasks(
            solver, visited_boxes, visited_targets,
            target_box_count, target_target_count,
            g_scan_beam_scratch.tasks, &task_count, &tasks_needed
        )) {
        return false;
    }
    if (task_count <= 0 || tasks_needed <= 0 || task_count > MAX_BOXES) return false;

    uint16_t hard_max_len = 0xFFFFu;
    if (max_segment_len >= 0) {
        hard_max_len = (max_segment_len > 0xFFFE) ? 0xFFFEu : (uint16_t)max_segment_len;
    }

    memset(g_scan_beam_scratch.current, 0, sizeof(g_scan_beam_scratch.current));
    memset(g_scan_beam_scratch.next, 0, sizeof(g_scan_beam_scratch.next));

    ScanBeamState* start = &g_scan_beam_scratch.current[0];
    memset(start, 0, sizeof(*start));
    start->valid = true;
    start->remaining_mask = (uint16_t)((1u << task_count) - 1u);
    start->pos = start_pos;
    start->heading = start_heading;
    memcpy(start->visited_boxes, visited_boxes, sizeof(bool) * MAX_BOXES);
    memcpy(start->visited_targets, visited_targets, sizeof(bool) * MAX_TARGETS);

    int current_count = 1;
    for (int depth = 0; depth < tasks_needed; depth++) {
        memset(g_scan_beam_scratch.next, 0, sizeof(g_scan_beam_scratch.next));
        int next_count = 0;

        for (int s = 0; s < current_count; s++) {
            ScanBeamState* state = &g_scan_beam_scratch.current[s];
            if (!state->valid) continue;

            for (int task_idx = 0; task_idx < task_count; task_idx++) {
                if ((state->remaining_mask & (uint16_t)(1u << task_idx)) == 0) continue;
                ScanPairedTask* task = &g_scan_beam_scratch.tasks[task_idx];
                if (state->visited_boxes[task->box_idx] || state->visited_targets[task->target_idx]) continue;

                scan_beam_expand_task_order(
                    bmap, box_positions, target_positions, state, task, task_idx,
                    true, radius, hard_max_len, g_scan_beam_scratch.next, &next_count
                );
                scan_beam_expand_task_order(
                    bmap, box_positions, target_positions, state, task, task_idx,
                    false, radius, hard_max_len, g_scan_beam_scratch.next, &next_count
                );
            }
        }

        if (next_count <= 0) return false;
        scan_beam_sort_and_prune(g_scan_beam_scratch.next, &next_count);
        current_count = next_count;
        for (int i = 0; i < current_count; i++) {
            g_scan_beam_scratch.current[i] = g_scan_beam_scratch.next[i];
        }
    }

    ScanBeamState* best = NULL;
    for (int i = 0; i < current_count; i++) {
        ScanBeamState* state = &g_scan_beam_scratch.current[i];
        if (!state->valid || state->waypoint_count != tasks_needed * 2) continue;
        if (!best || scan_beam_state_is_better(state, best)) best = state;
    }
    if (!best) return false;

    memcpy(out_path, best->path, best->len * sizeof(Direction));
    memcpy(out_waypoints, best->waypoints, best->waypoint_count * sizeof(Entity));
    memcpy(out_pause_positions, best->pauses, best->waypoint_count * sizeof(Position));
    memcpy(visited_boxes, best->visited_boxes, sizeof(bool) * MAX_BOXES);
    memcpy(visited_targets, best->visited_targets, sizeof(bool) * MAX_TARGETS);
    *out_len = best->len;
    *out_waypoint_count = best->waypoint_count;
    *out_final_pos = best->pos;
    if (out_final_heading) *out_final_heading = best->heading;
    return true;
}

// Original greedy scan sequence; kept as the fallback path.
FAST_OCRAM_FUNC static bool compute_scan_sequence_greedy(
    SokobanSolver* solver, Position start_pos,
    bool* visited_boxes, bool* visited_targets,
    int target_box_count, int target_target_count,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pause_positions,
    int* out_waypoint_count, Position* out_final_pos,
    bool use_lookahead, bool allow_clearance_push
) {
    int boxes_scanned = 0, targets_scanned = 0;
    for (int i = 0; i < solver->num_boxes; i++) if (visited_boxes[i]) boxes_scanned++;
    for (int i = 0; i < solver->num_targets; i++) if (visited_targets[i]) targets_scanned++;

    Position current_pos = start_pos;
    uint8_t scan_heading = SCAN_INITIAL_HEADING_DIR;
    uint16_t total_len = 0;
    *out_waypoint_count = 0;

    while (boxes_scanned < target_box_count || targets_scanned < target_target_count) {
        int best_box_idx = -1, best_target_idx = -1;
        uint16_t min_dist = 0xFFFF;
        uint16_t best_scan_score = 0xFFFF;
        Position segment_best_neighbor;
        Direction* segment_best_path = g_scan_sequence_segment_best_path;
        uint16_t segment_best_len = 0;
        uint8_t segment_best_heading = scan_heading;
        bool is_picking_box = false;

        if (boxes_scanned < target_box_count) {
            for (int i = 0; i < solver->num_boxes; i++) {
                if (visited_boxes[i]) continue;
                Position best_neighbor; Direction* temp_path = g_scan_sequence_temp_path; uint16_t temp_len;
                uint8_t temp_heading = scan_heading;
                uint16_t temp_score = 0xFFFF;
                const ScanNeighborLookahead* lookahead = NULL;
                if (use_lookahead) {
                    g_scan_neighbor_lookahead = (ScanNeighborLookahead){
                        NULL, solver->boxes, visited_boxes, solver->num_boxes, boxes_scanned, target_box_count,
                        NULL, solver->targets, visited_targets, solver->num_targets, targets_scanned, target_target_count,
                        true, i
                    };
                    lookahead = &g_scan_neighbor_lookahead;
                }
                if (find_best_scan_neighbor_and_path_with_lookahead(&solver->bmap, current_pos, solver->boxes[i].pos, &best_neighbor, temp_path, &temp_len, scan_heading, &temp_heading, &temp_score, lookahead, 0u)) {
                    if (scan_rank_is_better(temp_len, temp_score, min_dist, best_scan_score)) {
                        min_dist = temp_len; best_scan_score = temp_score; best_box_idx = i; best_target_idx = -1; is_picking_box = true;
                        segment_best_neighbor = best_neighbor; segment_best_len = temp_len; segment_best_heading = temp_heading;
                        memcpy(segment_best_path, temp_path, temp_len * sizeof(Direction));
                    }
                }
            }
        }

        if (targets_scanned < target_target_count) {
            for (int i = 0; i < solver->num_targets; i++) {
                if (visited_targets[i]) continue;
                Position best_neighbor; Direction* temp_path = g_scan_sequence_temp_path; uint16_t temp_len;
                uint8_t temp_heading = scan_heading;
                uint16_t temp_score = 0xFFFF;
                const ScanNeighborLookahead* lookahead = NULL;
                if (use_lookahead) {
                    g_scan_neighbor_lookahead = (ScanNeighborLookahead){
                        NULL, solver->boxes, visited_boxes, solver->num_boxes, boxes_scanned, target_box_count,
                        NULL, solver->targets, visited_targets, solver->num_targets, targets_scanned, target_target_count,
                        false, i
                    };
                    lookahead = &g_scan_neighbor_lookahead;
                }
                if (find_best_scan_neighbor_and_path_with_lookahead(&solver->bmap, current_pos, solver->targets[i].pos, &best_neighbor, temp_path, &temp_len, scan_heading, &temp_heading, &temp_score, lookahead, 0u)) {
                    if (scan_rank_is_better(temp_len, temp_score, min_dist, best_scan_score)) {
                        min_dist = temp_len; best_scan_score = temp_score; best_target_idx = i; best_box_idx = -1; is_picking_box = false;
                        segment_best_neighbor = best_neighbor; segment_best_len = temp_len; segment_best_heading = temp_heading;
                        memcpy(segment_best_path, temp_path, temp_len * sizeof(Direction));
                    }
                }
            }
        }

        // 智能清道夫：门内实体明显更近时，优先推开挡路箱抄近道。
        if (allow_clearance_push && scan_should_try_clearance(solver) && clear_blocking_box_for_scan(solver, &solver->bmap, &current_pos,
                                        visited_boxes, visited_targets, out_path, &total_len, min_dist)) {
            continue;
        }

        if (best_box_idx == -1 && best_target_idx == -1) {
            break;
        }

        if (total_len + segment_best_len + 1 < MAX_PATH_LENGTH) {
            if (segment_best_len > 0) {
                memcpy(&out_path[total_len], segment_best_path, segment_best_len * sizeof(Direction));
                total_len += segment_best_len;
            }
            out_path[total_len].dx = 0; out_path[total_len].dy = 0; total_len++;
            
            if (*out_waypoint_count < MAX_BOXES + MAX_TARGETS) {
                if (is_picking_box) {
                    out_waypoints[*out_waypoint_count].pos = solver->boxes[best_box_idx].pos;
                    out_waypoints[*out_waypoint_count].id = scan_waypoint_box_tag(best_box_idx);
                    visited_boxes[best_box_idx] = true;
                    boxes_scanned++;
                } else {
                    out_waypoints[*out_waypoint_count].pos = solver->targets[best_target_idx].pos;
                    out_waypoints[*out_waypoint_count].id = scan_waypoint_target_tag(best_target_idx);
                    visited_targets[best_target_idx] = true;
                    targets_scanned++;
                }
                out_waypoints[*out_waypoint_count].is_active = true;
                out_pause_positions[*out_waypoint_count] = segment_best_neighbor; (*out_waypoint_count)++;
            }
            current_pos = segment_best_neighbor;
            scan_heading = segment_best_heading;
        } else break;
    }

    *out_len = total_len; *out_final_pos = current_pos;
    return true;
}


FAST_OCRAM_FUNC static bool compute_scan_sequence(
    SokobanSolver* solver, Position start_pos,
    bool* visited_boxes, bool* visited_targets,
    int target_box_count, int target_target_count,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pause_positions,
    int* out_waypoint_count, Position* out_final_pos,
    bool allow_clearance_push
) {
    Position box_positions[MAX_BOXES];
    Position target_positions[MAX_TARGETS];
    Entity saved_boxes[MAX_BOXES];
    for (int i = 0; i < MAX_BOXES; i++) box_positions[i] = solver->boxes[i].pos;
    for (int i = 0; i < MAX_TARGETS; i++) target_positions[i] = solver->targets[i].pos;
    memcpy(saved_boxes, solver->boxes, sizeof(saved_boxes));

    bool beam_visited_boxes[MAX_BOXES];
    bool beam_visited_targets[MAX_TARGETS];
    memcpy(beam_visited_boxes, visited_boxes, sizeof(beam_visited_boxes));
    memcpy(beam_visited_targets, visited_targets, sizeof(beam_visited_targets));

    uint16_t beam_len = 0;
    int beam_waypoint_count = 0;
    Position beam_final_pos = start_pos;
    uint8_t beam_final_heading = SCAN_INITIAL_HEADING_DIR;
    bool beam_ok = scan_plan_paired_tasks_beam(
        solver, &solver->bmap, start_pos, SCAN_INITIAL_HEADING_DIR,
        box_positions, target_positions,
        beam_visited_boxes, beam_visited_targets,
        target_box_count, target_target_count,
        -1, -1,
        g_scan_beam_scratch.planned_path, &beam_len,
        g_scan_beam_scratch.planned_waypoints, g_scan_beam_scratch.planned_pauses,
        &beam_waypoint_count, &beam_final_pos,
        &beam_final_heading
    );

    BitboardMap saved_bmap = solver->bmap;
    bool greedy_visited_boxes[MAX_BOXES];
    bool greedy_visited_targets[MAX_TARGETS];
    memcpy(greedy_visited_boxes, visited_boxes, sizeof(greedy_visited_boxes));
    memcpy(greedy_visited_targets, visited_targets, sizeof(greedy_visited_targets));

    uint16_t greedy_len = 0;
    int greedy_waypoint_count = 0;
    Position greedy_final_pos = start_pos;
    solver->bmap = saved_bmap;
    bool greedy_ok = compute_scan_sequence_greedy(
        solver, start_pos,
        greedy_visited_boxes, greedy_visited_targets,
        target_box_count, target_target_count,
        g_scan_beam_scratch.greedy_path, &greedy_len,
        g_scan_beam_scratch.greedy_waypoints, g_scan_beam_scratch.greedy_pauses,
        &greedy_waypoint_count, &greedy_final_pos,
        true, allow_clearance_push
    );
    BitboardMap greedy_bmap = solver->bmap;
    Entity greedy_boxes[MAX_BOXES];
    memcpy(greedy_boxes, solver->boxes, sizeof(greedy_boxes));

    if (beam_ok && (!greedy_ok || scan_beam_should_replace_greedy(
            g_scan_beam_scratch.planned_path, beam_len,
            g_scan_beam_scratch.greedy_path, greedy_len
        ))) {
        if (beam_len > 0) memcpy(g_scan_beam_scratch.greedy_path, g_scan_beam_scratch.planned_path, beam_len * sizeof(Direction));
        if (beam_waypoint_count > 0) {
            memcpy(g_scan_beam_scratch.greedy_waypoints, g_scan_beam_scratch.planned_waypoints, beam_waypoint_count * sizeof(Entity));
            memcpy(g_scan_beam_scratch.greedy_pauses, g_scan_beam_scratch.planned_pauses, beam_waypoint_count * sizeof(Position));
        }
        memcpy(greedy_visited_boxes, beam_visited_boxes, sizeof(bool) * MAX_BOXES);
        memcpy(greedy_visited_targets, beam_visited_targets, sizeof(bool) * MAX_TARGETS);
        greedy_len = beam_len;
        greedy_waypoint_count = beam_waypoint_count;
        greedy_final_pos = beam_final_pos;
        greedy_bmap = saved_bmap;
        memcpy(greedy_boxes, saved_boxes, sizeof(greedy_boxes));
        greedy_ok = true;
        (void)beam_final_heading;
    }

    bool safe_visited_boxes[MAX_BOXES];
    bool safe_visited_targets[MAX_TARGETS];
    memcpy(safe_visited_boxes, visited_boxes, sizeof(safe_visited_boxes));
    memcpy(safe_visited_targets, visited_targets, sizeof(safe_visited_targets));

    uint16_t safe_len = 0;
    int safe_waypoint_count = 0;
    Position safe_final_pos = start_pos;
    solver->bmap = saved_bmap;
    memcpy(solver->boxes, saved_boxes, sizeof(saved_boxes));
    bool safe_ok = compute_scan_sequence_greedy(
        solver, start_pos,
        safe_visited_boxes, safe_visited_targets,
        target_box_count, target_target_count,
        out_path, &safe_len,
        out_waypoints, out_pause_positions,
        &safe_waypoint_count, &safe_final_pos,
        false, allow_clearance_push
    );
    BitboardMap safe_bmap = solver->bmap;

    if (safe_ok && (!greedy_ok || !scan_beam_should_replace_greedy(
            g_scan_beam_scratch.greedy_path, greedy_len,
            out_path, safe_len
        ))) {
        memcpy(visited_boxes, safe_visited_boxes, sizeof(bool) * MAX_BOXES);
        memcpy(visited_targets, safe_visited_targets, sizeof(bool) * MAX_TARGETS);
        solver->bmap = safe_bmap;
        *out_len = safe_len;
        *out_waypoint_count = safe_waypoint_count;
        *out_final_pos = safe_final_pos;
        return true;
    }

    if (greedy_ok) {
        if (greedy_len > 0) memcpy(out_path, g_scan_beam_scratch.greedy_path, greedy_len * sizeof(Direction));
        if (greedy_waypoint_count > 0) {
            memcpy(out_waypoints, g_scan_beam_scratch.greedy_waypoints, greedy_waypoint_count * sizeof(Entity));
            memcpy(out_pause_positions, g_scan_beam_scratch.greedy_pauses, greedy_waypoint_count * sizeof(Position));
        }
        memcpy(visited_boxes, greedy_visited_boxes, sizeof(bool) * MAX_BOXES);
        memcpy(visited_targets, greedy_visited_targets, sizeof(bool) * MAX_TARGETS);
        solver->bmap = greedy_bmap;
        memcpy(solver->boxes, greedy_boxes, sizeof(greedy_boxes));
        *out_len = greedy_len;
        *out_waypoint_count = greedy_waypoint_count;
        *out_final_pos = greedy_final_pos;
        return true;
    }

    solver->bmap = saved_bmap;
    memcpy(solver->boxes, saved_boxes, sizeof(saved_boxes));
    return false;
}

/**
 * 扩展扫描路径：在当前路径中插入补扫绕行。
 */
FAST_OCRAM_FUNC bool sokoban_extend_scan_path(SokobanSolver* solver, bool need_box, bool need_target, int current_idx) {
    if (!solver || current_idx < 0 || current_idx >= solver->scan_waypoint_count ||
        solver->scan_waypoint_count >= SOKOBAN_SCAN_MAX_WAYPOINTS) {
        return false;
    }

    bool retry_is_box = false;
    int retry_entity_idx = -1;
    bool retry_entity_found = scan_decode_waypoint_entity(
        solver, solver->scan_waypoints[current_idx].id, &retry_is_box, &retry_entity_idx
    );
    if (!retry_entity_found) {
        retry_entity_found = scan_find_waypoint_entity_by_position(
            solver, solver->scan_waypoints[current_idx].pos, &retry_is_box, &retry_entity_idx
        );
    }
    if (retry_entity_found &&
        ((retry_is_box && !need_box) || (!retry_is_box && !need_target))) {
        retry_entity_found = false;
    }
    uint8_t retry_heading_mask = 0u;

    int pauses_seen = 0;
    int current_insert_idx = -1;
    for (int i = 0; i < solver->best_path_len; i++) {
        if (solver->best_path[i].dx == 0 && solver->best_path[i].dy == 0) {
            if (pauses_seen == current_idx) { current_insert_idx = i + 1; break; }
            pauses_seen++;
        }
    }
    if (current_insert_idx == -1) return false;
    PathReplayOptions extend_preview_options = scan_replay_hide_box_keep_target_options();

    // 预演整条路径，识别尚未纳入扫描计划的实体。
    bool* planned_boxes = g_scan_extend_scratch.planned_boxes;
    bool* planned_targets = g_scan_extend_scratch.planned_targets;
    memset(planned_boxes, 0, MAX_BOXES * sizeof(planned_boxes[0]));
    memset(planned_targets, 0, MAX_TARGETS * sizeof(planned_targets[0]));
    {
        Position* temp_boxes = g_scan_extend_scratch.temp_boxes;
        PathReplayState temp_state;
        if (!scan_replay_load_from_solver(
                &temp_state,
                &g_scan_initial_bmap,
                g_scan_initial_player,
                g_scan_initial_boxes,
                solver->num_boxes,
                g_scan_initial_bombs,
                g_scan_initial_num_bombs)) {
            return false;
        }
        scan_replay_copy_positions(temp_boxes, &temp_state);
        int pause_idx = 0;
        for (int j = 0; j < solver->best_path_len; j++) {
            if (solver->best_path[j].dx == 0 && solver->best_path[j].dy == 0) {
                if (pause_idx >= solver->scan_waypoint_count) return false;
                Position wp = solver->scan_waypoints[pause_idx].pos;
                bool wp_is_box = false;
                int wp_idx = -1;
                bool wp_found = scan_decode_waypoint_entity(
                    solver, solver->scan_waypoints[pause_idx].id, &wp_is_box, &wp_idx
                );
                if (wp_found) {
                    if (wp_is_box) planned_boxes[wp_idx] = true;
                    else planned_targets[wp_idx] = true;

                    if (retry_entity_found && pause_idx <= current_idx &&
                        wp_is_box == retry_is_box && wp_idx == retry_entity_idx) {
                        Position entity_pos = wp_is_box ? temp_boxes[wp_idx] : solver->targets[wp_idx].pos;
                        uint8_t heading = direction_index_between(temp_state.player, entity_pos);
                        if (heading < 4u) retry_heading_mask |= (uint8_t)(1u << heading);
                    }
                } else {
                    for (int b = 0; b < solver->num_boxes; b++) {
                        if (pos_equal(temp_boxes[b], wp)) planned_boxes[b] = true;
                    }
                    for (int t = 0; t < solver->num_targets; t++) {
                        if (pos_equal(solver->targets[t].pos, wp)) planned_targets[t] = true;
                    }
                }
                pause_idx++;
            }
            
            Direction d = solver->best_path[j];
            if (!scan_replay_step_existing(solver, &temp_state, d, &extend_preview_options, NULL)) return false;
            scan_replay_copy_positions(temp_boxes, &temp_state);
        }
    }

    bool has_unplanned_box = false;
    bool has_unplanned_target = false;
    for (int b = 0; b < solver->num_boxes; b++) {
        if (!planned_boxes[b]) has_unplanned_box = true;
    }
    for (int t = 0; t < solver->num_targets; t++) {
        if (!planned_targets[t]) has_unplanned_target = true;
    }
    bool retry_box_fallback = need_box && retry_entity_found && retry_is_box &&
                              !has_unplanned_box && retry_heading_mask != 0u;
    bool retry_target_fallback = need_target && retry_entity_found && !retry_is_box &&
                                 !has_unplanned_target && retry_heading_mask != 0u;

    // 2. 在后续路径中寻找补扫绕行的最小代价插入点。
    int best_insert_j = -1;
    uint16_t global_min_detour = 0xFFFF;
    Direction* best_to_path = g_scan_extend_scratch.best_to_path;
    uint16_t best_to_len = 0;
    Direction* best_back_path = g_scan_extend_scratch.best_back_path;
    uint16_t best_back_len = 0;
    Position best_view_pos;
    Position best_detour_target;
    bool best_detour_is_box = false;
    int best_detour_entity_idx = -1;

    BitboardMap* sim_map = &g_scan_extend_scratch.sim_map;
    Position* sim_boxes = g_scan_extend_scratch.sim_boxes;
    PathReplayState sim_state;
    if (!scan_replay_load_from_solver(
            &sim_state,
            &g_scan_initial_bmap,
            g_scan_initial_player,
            g_scan_initial_boxes,
            solver->num_boxes,
            g_scan_initial_bombs,
            g_scan_initial_num_bombs)) {
        return false;
    }
    *sim_map = sim_state.map;
    Position curr_p = sim_state.player;
    scan_replay_copy_positions(sim_boxes, &sim_state);
    for (int j = 0; j <= solver->best_path_len; j++) {
        // 回放到当前插入点之前，得到模拟位置。
        if (j >= current_insert_idx) {
            
            // 检查尚未扫描的箱子，尝试在路径第 j 步插入最短补扫绕行。
            if (need_box) {
                for (int b = 0; b < solver->num_boxes; b++) {
                    bool retry_candidate = retry_box_fallback && b == retry_entity_idx;
                    if (!planned_boxes[b] || retry_candidate) {
                        Position candidate_target = sim_boxes[b];
                        if (!is_in_bounds(candidate_target.x, candidate_target.y)) continue;
                        Position temp_view;
                        Direction* temp_to_path = g_scan_extend_scratch.temp_to_path;
                        uint16_t temp_to_len;
                        uint8_t excluded_headings = retry_candidate ? retry_heading_mask : 0u;
                        if (find_best_scan_neighbor_and_path_with_lookahead(
                                sim_map, curr_p, candidate_target,
                                &temp_view, temp_to_path, &temp_to_len,
                                DIRECTION_INDEX_NONE, NULL, NULL, NULL, excluded_headings
                            )) {
                            Direction* temp_back_path = g_scan_extend_scratch.temp_back_path;
                            uint16_t temp_back_len = 0;
                            if (scan_cached_astar_navigate(solver, sim_map, temp_view, curr_p, MASK_WALL | MASK_BOMB | MASK_BOX, temp_back_path, &temp_back_len)) {
                                uint16_t total_detour = temp_to_len + temp_back_len;
                                if (total_detour < global_min_detour) {
                                    global_min_detour = total_detour;
                                    best_insert_j = j;  // 记录当前最优插入位置。
                                    best_to_len = temp_to_len;
                                    memcpy(best_to_path, temp_to_path, temp_to_len * sizeof(Direction));
                                    best_back_len = temp_back_len;
                                    memcpy(best_back_path, temp_back_path, temp_back_len * sizeof(Direction));
                                    best_view_pos = temp_view;
                                    best_detour_target = candidate_target;
                                    best_detour_is_box = true;
                                    best_detour_entity_idx = b;
                                }
                            }
                        }
                    }
                }
            }

            // 若存在未扫描目标点，按相同方式评估插入代价。
            if (need_target) {
                for (int t = 0; t < solver->num_targets; t++) {
                    bool retry_candidate = retry_target_fallback && t == retry_entity_idx;
                    if (!planned_targets[t] || retry_candidate) {
                        Position candidate_target = solver->targets[t].pos;
                        Position temp_view;
                        Direction* temp_to_path = g_scan_extend_scratch.temp_to_path;
                        uint16_t temp_to_len;
                        uint8_t excluded_headings = retry_candidate ? retry_heading_mask : 0u;
                        if (find_best_scan_neighbor_and_path_with_lookahead(
                                sim_map, curr_p, candidate_target,
                                &temp_view, temp_to_path, &temp_to_len,
                                DIRECTION_INDEX_NONE, NULL, NULL, NULL, excluded_headings
                            )) {
                            Direction* temp_back_path = g_scan_extend_scratch.temp_back_path;
                            uint16_t temp_back_len = 0;
                            if (scan_cached_astar_navigate(solver, sim_map, temp_view, curr_p, MASK_WALL | MASK_BOMB | MASK_BOX, temp_back_path, &temp_back_len)) {
                                uint16_t total_detour = temp_to_len + temp_back_len;
                                if (total_detour < global_min_detour) {
                                    global_min_detour = total_detour;
                                    best_insert_j = j;
                                    best_to_len = temp_to_len;
                                    memcpy(best_to_path, temp_to_path, temp_to_len * sizeof(Direction));
                                    best_back_len = temp_back_len;
                                    memcpy(best_back_path, temp_back_path, temp_back_len * sizeof(Direction));
                                    best_view_pos = temp_view;
                                    best_detour_target = candidate_target;
                                    best_detour_is_box = false;
                                    best_detour_entity_idx = t;
                                }
                            }
                        }
                    }
                }
            }
        }

        // 若绕行代价为 0，说明当前位置已经能完成补扫，可提前结束。
        if (global_min_detour == 0) break; 

        // 推进模拟状态到 j+1 步，保证后续插入评估使用真实地图。
        if (j < solver->best_path_len) {
            if (!scan_replay_step_existing(solver, &sim_state, solver->best_path[j], &extend_preview_options, NULL)) return false;
            *sim_map = sim_state.map;
            curr_p = sim_state.player;
            scan_replay_copy_positions(sim_boxes, &sim_state);
        }
    }

    if (best_insert_j == -1) return false;

    // 3. 在最优插入点重组主路径，并插入补扫子路径。
    int detour_total_len = best_to_len + 1 + best_back_len;
    uint16_t extended_path_len = (uint16_t)(solver->best_path_len + detour_total_len);
    if (extended_path_len >= MAX_PATH_LENGTH) return false;

    Direction* extended_path = g_scan_verified_path;
    memcpy(extended_path, solver->best_path,
           (size_t)best_insert_j * sizeof(Direction));

    int p = best_insert_j;
    for (int i = 0; i < best_to_len; i++) extended_path[p++] = best_to_path[i];
    extended_path[p].dx = 0;
    extended_path[p].dy = 0;
    p++;
    for (int i = 0; i < best_back_len; i++) extended_path[p++] = best_back_path[i];
    memcpy(&extended_path[p],
           &solver->best_path[best_insert_j],
           (size_t)(solver->best_path_len - best_insert_j) * sizeof(Direction));
    p += solver->best_path_len - best_insert_j;
    if (p != extended_path_len) return false;

    // 4. 同步更新扫描航点队列，保持路径与停靠点索引一致。
    PathReplayState extended_final_state;
    if (!scan_replay_load_from_solver(
            &extended_final_state,
            &g_scan_initial_bmap,
            g_scan_initial_player,
            g_scan_initial_boxes,
            solver->num_boxes,
            g_scan_initial_bombs,
            g_scan_initial_num_bombs)) {
        return false;
    }
    if (!scan_replay_apply_path_existing(
            solver,
            &extended_final_state,
            extended_path,
            extended_path_len,
            &extend_preview_options)) {
        return false;
    }

    int target_waypoint_idx = 0;
    int pauses_before_j = 0;
    for (int i = 0; i < best_insert_j; i++) {
        if (solver->best_path[i].dx == 0 && solver->best_path[i].dy == 0) pauses_before_j++;
    }
    target_waypoint_idx = pauses_before_j;
    if (target_waypoint_idx > solver->scan_waypoint_count) return false;

    memmove(&solver->scan_waypoints[target_waypoint_idx + 1],
            &solver->scan_waypoints[target_waypoint_idx],
            (solver->scan_waypoint_count - target_waypoint_idx) * sizeof(Entity));
    
    memmove(&solver->scan_player_pause_positions[target_waypoint_idx + 1],
            &solver->scan_player_pause_positions[target_waypoint_idx],
            (solver->scan_waypoint_count - target_waypoint_idx) * sizeof(Position));

    solver->scan_waypoints[target_waypoint_idx].pos = best_detour_target;
    if (best_detour_is_box) {
        solver->scan_waypoints[target_waypoint_idx].id = scan_waypoint_box_tag(best_detour_entity_idx);
    } else {
        solver->scan_waypoints[target_waypoint_idx].id = scan_waypoint_target_tag(best_detour_entity_idx);
    }
    solver->scan_waypoints[target_waypoint_idx].is_active = true;
    solver->scan_player_pause_positions[target_waypoint_idx] = best_view_pos;

    solver->scan_waypoint_count++;

    // 重新回放更新后的完整扫描路径，确保求解器终点与插入绕行后的终点一致。
    memcpy(solver->best_path, extended_path,
           (size_t)extended_path_len * sizeof(Direction));
    solver->best_path_len = extended_path_len;
    solver->best_steps = extended_path_len;
    scan_replay_write_solver_state(solver, &extended_final_state);
    scan_refresh_bomb_delay_events(solver);
    return true;
}
/* 无炸弹识别统一使用观察覆盖模块，并完整识别全部箱子和目的地。 */


FAST_OCRAM_FUNC static bool scan_plan_observation_coverage(
    SokobanSolver* solver, Position start_pos,
    int req_boxes, int req_targets, uint16_t incumbent_cost,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pause_positions,
    int* out_waypoint_count, Position* out_final_pos
);

FAST_OCRAM_FUNC static bool scan_generate_observation_coverage_path(
    SokobanSolver* solver,
    int req_boxes,
    int req_targets,
    bool allow_clearance_push
) {
    if (!solver || req_boxes < 0 || req_targets < 0 ||
        req_boxes > solver->num_boxes || req_targets > solver->num_targets) {
        return false;
    }

    bool visited_boxes[MAX_BOXES] = {0};
    bool visited_targets[MAX_TARGETS] = {0};

    uint16_t out_len = 0;
    int out_waypoints = 0;
    Position final_pos = solver->start_player;
    int required_waypoints = req_boxes + req_targets;

    BitboardMap original_bmap = solver->bmap;
    Entity original_boxes[MAX_BOXES];
    Position original_start = solver->start_player;
    memcpy(original_boxes, solver->boxes, sizeof(original_boxes));

    /* Establish a real fallback first; its score is a generic upper bound. */
    bool greedy_ok = compute_scan_sequence(
        solver, original_start,
        visited_boxes, visited_targets,
        req_boxes, req_targets,
        solver->best_path, &out_len,
        solver->scan_waypoints, solver->scan_player_pause_positions,
        &out_waypoints, &final_pos,
        allow_clearance_push
    );

    uint16_t greedy_len = out_len;
    int greedy_waypoints = out_waypoints;
    Position greedy_final_pos = final_pos;
    bool greedy_complete = greedy_ok &&
        greedy_waypoints == required_waypoints;
    uint16_t greedy_cost = 0xFFFFu;
    BitboardMap greedy_bmap = original_bmap;
    Entity greedy_boxes[MAX_BOXES];
    if (greedy_ok) {
        greedy_bmap = solver->bmap;
        memcpy(greedy_boxes, solver->boxes, sizeof(greedy_boxes));
        if (greedy_complete) {
            greedy_cost = (uint16_t)scan_path_weighted_cost(
                solver->best_path, greedy_len
            );
        }
        if (greedy_len > 0) {
            memcpy(g_scan_verified_path, solver->best_path,
                   greedy_len * sizeof(Direction));
        }
        if (greedy_waypoints > 0) {
            memcpy(g_scan_eager_desired_waypoints, solver->scan_waypoints,
                   greedy_waypoints * sizeof(Entity));
            memcpy(g_scan_bomb_best_pauses,
                   solver->scan_player_pause_positions,
                   greedy_waypoints * sizeof(Position));
        }
    }

    /* Coverage search is read-only with respect to the original map state. */
    solver->bmap = original_bmap;
    memcpy(solver->boxes, original_boxes, sizeof(original_boxes));
    solver->start_player = original_start;

    uint16_t coverage_len = 0;
    int coverage_waypoints = 0;
    Position coverage_final_pos = original_start;
    bool coverage_ok = scan_plan_observation_coverage(
        solver, original_start,
        req_boxes, req_targets, greedy_cost,
        g_scan_hybrid_path, &coverage_len,
        g_scan_eager_waypoints, g_scan_eager_pauses,
        &coverage_waypoints, &coverage_final_pos
    );

    bool use_coverage = coverage_ok &&
        coverage_waypoints == required_waypoints &&
        (!greedy_complete ||
         scan_path_is_better(
             g_scan_hybrid_path, coverage_len,
             solver->best_path, greedy_len));

    if (use_coverage) {
        solver->bmap = original_bmap;
        memcpy(solver->boxes, original_boxes, sizeof(original_boxes));
        if (coverage_len > 0) {
            memcpy(solver->best_path, g_scan_hybrid_path,
                   coverage_len * sizeof(Direction));
        }
        memcpy(solver->scan_waypoints, g_scan_eager_waypoints,
               coverage_waypoints * sizeof(Entity));
        memcpy(solver->scan_player_pause_positions, g_scan_eager_pauses,
               coverage_waypoints * sizeof(Position));
        solver->best_path_len = coverage_len;
        solver->best_steps = coverage_len;
        solver->scan_waypoint_count = coverage_waypoints;
        solver->start_player = coverage_final_pos;
        solver_refresh_deadlocks(solver);
        return true;
    }

    if (greedy_complete) {
        solver->bmap = greedy_bmap;
        memcpy(solver->boxes, greedy_boxes, sizeof(greedy_boxes));
        if (greedy_len > 0) {
            memcpy(solver->best_path, g_scan_verified_path,
                   greedy_len * sizeof(Direction));
        }
        memcpy(solver->scan_waypoints, g_scan_eager_desired_waypoints,
               greedy_waypoints * sizeof(Entity));
        memcpy(solver->scan_player_pause_positions,
               g_scan_bomb_best_pauses,
               greedy_waypoints * sizeof(Position));
        solver->best_path_len = greedy_len;
        solver->best_steps = greedy_len;
        solver->scan_waypoint_count = greedy_waypoints;
        solver->start_player = greedy_final_pos;
        return true;
    }

    return false;
}
FAST_OCRAM_FUNC bool sokoban_generate_scan_path(SokobanSolver* solver) {
    if (!solver || solver->num_bombs != 0) return false;

    /* Every no-bomb identified solve uses the same full N/N coverage entry. */
    return scan_generate_observation_coverage_path(
        solver, solver->num_boxes, solver->num_targets, true
    );
}
static bool should_delay_scan(const Direction* main_path, int current_step_idx, int cutoff_idx, Position current_pos, Position entity_pos) {
    if (current_step_idx < 0) return false;

    int lookahead_dist = manhattan_distance(current_pos, entity_pos);
    Position lookahead_pos = current_pos;
    for (int k = current_step_idx; k <= cutoff_idx; k++) {
        lookahead_pos.x += main_path[k].dx;
        lookahead_pos.y += main_path[k].dy;
        if (manhattan_distance(lookahead_pos, entity_pos) < lookahead_dist) {
            return true;
        }
    }
    return false;
}

FAST_OCRAM_FUNC static void build_scan_nav_field(
    BitboardMap* sim_map, Position start_p,
    uint16_t dist[MAP_ROWS][MAP_COLS], uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    Position* q
) {
    int head = 0, tail = 0;

    memset(dist, 0xFF, sizeof(uint16_t) * MAP_ROWS * MAP_COLS);
    memset(parent_dir, 0xFF, sizeof(uint8_t) * MAP_ROWS * MAP_COLS);

    if (start_p.x >= MAP_COLS || start_p.y >= MAP_ROWS) return;
    dist[start_p.y][start_p.x] = 0;
    q[tail++] = start_p;

    while (head < tail) {
        Position curr = q[head++];
        uint16_t curr_d = dist[curr.y][curr.x];

        for (int d = 0; d < 4; d++) {
            int nx = (int)curr.x + DIRECTIONS[d].dx;
            int ny = (int)curr.y + DIRECTIONS[d].dy;
            if (nx < 0 || nx >= MAP_COLS || ny < 0 || ny >= MAP_ROWS) continue;
            if (dist[ny][nx] != 0xFFFF) continue;
            if (((sim_map->walls[ny] | sim_map->bombs[ny] | sim_map->boxes[ny]) & (1u << nx)) != 0) continue;

            dist[ny][nx] = (uint16_t)(curr_d + 1);
            parent_dir[ny][nx] = (uint8_t)d;
            q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }
}

FAST_OCRAM_FUNC static void scan_spine_build_nav_order(
    const uint16_t dist[MAP_ROWS][MAP_COLS], Position* queue
) {
    memset(g_scan_spine_nav_layer_offsets, 0,
           sizeof(g_scan_spine_nav_layer_offsets));
    for (int y = 0; y < MAP_ROWS; y++) {
        for (int x = 0; x < MAP_COLS; x++) {
            uint16_t step = dist[y][x];
            if (step != UINT16_MAX) g_scan_spine_nav_layer_offsets[step]++;
        }
    }
    uint16_t total = 0u;
    for (uint16_t step = 0u; step < MAP_ROWS * MAP_COLS; step++) {
        uint8_t count = g_scan_spine_nav_layer_offsets[step];
        g_scan_spine_nav_layer_offsets[step] = (uint8_t)total;
        total = (uint16_t)(total + count);
    }
    for (int y = 0; y < MAP_ROWS; y++) {
        for (int x = 0; x < MAP_COLS; x++) {
            uint16_t step = dist[y][x];
            if (step == UINT16_MAX) continue;
            queue[g_scan_spine_nav_layer_offsets[step]++] =
                (Position){(uint8_t)x, (uint8_t)y};
        }
    }
    g_scan_spine_nav_order_count = total;
}
static void scan_spine_nav_cache_reset(void) {
    for (uint32_t i = 0; i < SCAN_SPINE_NAV_CACHE_ENTRY_COUNT; i++) {
        g_scan_spine_nav_cache[i].valid = false;
    }
    memset(g_scan_spine_nav_cache_next_way, 0,
           sizeof(g_scan_spine_nav_cache_next_way));
}

/* Spine expansion consumes only distances. Final route reconstruction still
 * uses build_scan_nav_field() so cached hits cannot change parent directions. */
FAST_OCRAM_FUNC static void scan_spine_build_nav_dist_cached(
    BitboardMap* map, Position start,
    uint16_t dist[MAP_ROWS][MAP_COLS],
    uint8_t parent_dir[MAP_ROWS][MAP_COLS], Position* queue
) {
    g_scan_spine_turn_cache_valid = false;
    uint16_t obstacles[MAP_ROWS];
    uint32_t hash = 2166136261u;
    for (int y = 0; y < MAP_ROWS; y++) {
        obstacles[y] = (uint16_t)(map->walls[y] | map->bombs[y] | map->boxes[y]);
        hash ^= obstacles[y];
        hash *= 16777619u;
    }
    hash ^= start.x;
    hash *= 16777619u;
    hash ^= start.y;

    uint32_t set = hash & (SCAN_SPINE_NAV_CACHE_SET_COUNT - 1u);
    ScanSpineNavCacheEntry* entries =
        &g_scan_spine_nav_cache[set * SCAN_SPINE_NAV_CACHE_WAYS];
    for (uint32_t way = 0; way < SCAN_SPINE_NAV_CACHE_WAYS; way++) {
        ScanSpineNavCacheEntry* entry = &entries[way];
        if (!entry->valid || !pos_equal(entry->start, start) ||
            memcmp(entry->obstacles, obstacles, sizeof(obstacles)) != 0) {
            continue;
        }
        for (int y = 0; y < MAP_ROWS; y++) {
            for (int x = 0; x < MAP_COLS; x++) {
                uint8_t value = entry->dist[y][x];
                dist[y][x] = (value == UINT8_MAX) ? UINT16_MAX : value;
            }
        }
        scan_spine_build_nav_order(dist, queue);
        return;
    }

    build_scan_nav_field(map, start, dist, parent_dir, queue);
    uint8_t replace_way = g_scan_spine_nav_cache_next_way[set];
    for (uint8_t way = 0; way < SCAN_SPINE_NAV_CACHE_WAYS; way++) {
        if (!entries[way].valid) {
            replace_way = way;
            break;
        }
    }
    g_scan_spine_nav_cache_next_way[set] =
        (uint8_t)((replace_way + 1u) & (SCAN_SPINE_NAV_CACHE_WAYS - 1u));
    ScanSpineNavCacheEntry* entry = &entries[replace_way];
    memcpy(entry->obstacles, obstacles, sizeof(obstacles));
    entry->start = start;
    entry->valid = true;
    for (int y = 0; y < MAP_ROWS; y++) {
        for (int x = 0; x < MAP_COLS; x++) {
            uint16_t value = dist[y][x];
            entry->dist[y][x] = (value == UINT16_MAX) ? UINT8_MAX : (uint8_t)value;
        }
    }
    scan_spine_build_nav_order(dist, queue);
}

FAST_OCRAM_FUNC static bool reconstruct_scan_nav_path(
    const uint16_t dist[MAP_ROWS][MAP_COLS], const uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    Position target, Direction* out_path, uint16_t* out_len
) {
    uint16_t len = dist[target.y][target.x];
    if (len == 0xFFFF || len >= MAX_SINGLE_PATH) return false;

    Position curr = target;
    for (int i = (int)len - 1; i >= 0; i--) {
        uint8_t d = parent_dir[curr.y][curr.x];
        if (d >= 4) return false;
        out_path[i] = DIRECTIONS[d];
        int px = (int)curr.x - DIRECTIONS[d].dx;
        int py = (int)curr.y - DIRECTIONS[d].dy;
        if (px < 0 || px >= MAP_COLS || py < 0 || py >= MAP_ROWS) return false;
        curr = (Position){(uint8_t)px, (uint8_t)py};
    }

    *out_len = len;
    return true;
}

FAST_OCRAM_FUNC static bool reconstruct_scan_nav_turn_path(
    const uint16_t dist[MAP_ROWS][MAP_COLS], Position target,
    uint8_t preferred_heading, uint8_t scan_heading,
    Direction* out_path, uint16_t* out_len
) {
    uint16_t len = dist[target.y][target.x];
    if (len == 0xFFFF || len >= MAX_SINGLE_PATH) return false;
    if (len == 0) {
        *out_len = 0;
        return true;
    }

    memset(g_scan_nav_turn_cost, 0xFF, sizeof(g_scan_nav_turn_cost));
    memset(g_scan_nav_turn_parent, 0xFF, sizeof(g_scan_nav_turn_parent));

    for (uint16_t step = 1; step <= len; step++) {
        for (int y = 1; y < MAP_ROWS - 1; y++) {
            for (int x = 1; x < MAP_COLS - 1; x++) {
                if (dist[y][x] != step) continue;

                for (int dir = 0; dir < 4; dir++) {
                    int px = x - DIRECTIONS[dir].dx;
                    int py = y - DIRECTIONS[dir].dy;
                    if (px < 0 || px >= MAP_COLS || py < 0 || py >= MAP_ROWS) continue;
                    if (dist[py][px] + 1u != step) continue;

                    if (step == 1) {
                        uint16_t cost = scan_turn_score(preferred_heading, (uint8_t)dir);
                        if (cost < g_scan_nav_turn_cost[dir][y][x]) {
                            g_scan_nav_turn_cost[dir][y][x] = cost;
                            g_scan_nav_turn_parent[dir][y][x] = 4u;
                        }
                    } else {
                        for (int prev_dir = 0; prev_dir < 4; prev_dir++) {
                            uint16_t prev_cost = g_scan_nav_turn_cost[prev_dir][py][px];
                            if (prev_cost == 0xFFFF) continue;
                            uint32_t cost = (uint32_t)prev_cost;
                            if (prev_dir != dir) cost += SCAN_BEND_SLOWDOWN_SCORE;
                            if (cost < g_scan_nav_turn_cost[dir][y][x]) {
                                g_scan_nav_turn_cost[dir][y][x] = (uint16_t)cost;
                                g_scan_nav_turn_parent[dir][y][x] = (uint8_t)prev_dir;
                            }
                        }
                    }
                }
            }
        }
    }

    int best_dir = -1;
    uint16_t best_score = 0xFFFF;
    for (int dir = 0; dir < 4; dir++) {
        uint16_t route_score = g_scan_nav_turn_cost[dir][target.y][target.x];
        if (route_score == 0xFFFF) continue;
        uint32_t score = (uint32_t)route_score + scan_turn_score((uint8_t)dir, scan_heading);
        if (score < best_score) {
            best_score = (uint16_t)score;
            best_dir = dir;
        }
    }
    if (best_dir < 0) return false;

    Position curr = target;
    int dir = best_dir;
    for (int i = (int)len - 1; i >= 0; i--) {
        if (dir < 0 || dir >= 4) return false;
        out_path[i] = DIRECTIONS[dir];
        uint8_t parent = g_scan_nav_turn_parent[dir][curr.y][curr.x];
        int px = (int)curr.x - DIRECTIONS[dir].dx;
        int py = (int)curr.y - DIRECTIONS[dir].dy;
        if (px < 0 || px >= MAP_COLS || py < 0 || py >= MAP_ROWS) return false;
        curr = (Position){(uint8_t)px, (uint8_t)py};
        if (parent == 4u) {
            if (i != 0) return false;
            break;
        }
        if (parent > 3u) return false;
        dir = parent;
    }

    *out_len = len;
    return true;
}
// 在半径和绕行预算内挑选最近的未扫描实体。
static bool scan_nav_compact_cell_is_open(const BitboardMap* map, int x, int y) {
    if (!map || x < 0 || x >= MAP_COLS || y < 0 || y >= MAP_ROWS) return false;
    return ((map->walls[y] | map->bombs[y] | map->boxes[y]) & (1u << x)) == 0;
}

static uint32_t scan_nav_compact_parent_index(
    uint16_t arrival_step, Position pos, uint8_t direction
) {
    return ((uint32_t)(arrival_step - 2u) * SCAN_NAV_COMPACT_PARENT_STATE_COUNT) +
           (((uint32_t)direction * MAP_ROWS + pos.y) * MAP_COLS + pos.x);
}

static void scan_nav_compact_set_parent(
    uint16_t arrival_step, Position pos, uint8_t direction, uint8_t previous_direction
) {
    uint32_t index = scan_nav_compact_parent_index(arrival_step, pos, direction);
    uint32_t byte_index = index >> 2u;
    uint8_t shift = (uint8_t)((index & 3u) << 1u);
    uint8_t mask = (uint8_t)(3u << shift);
    g_scan_nav_compact_dp.parent_bits[byte_index] =
        (uint8_t)((g_scan_nav_compact_dp.parent_bits[byte_index] & (uint8_t)~mask) |
                  ((previous_direction & 3u) << shift));
}

static uint8_t scan_nav_compact_get_parent(
    uint16_t arrival_step, Position pos, uint8_t direction
) {
    uint32_t index = scan_nav_compact_parent_index(arrival_step, pos, direction);
    return (uint8_t)((g_scan_nav_compact_dp.parent_bits[index >> 2u] >>
                      ((index & 3u) << 1u)) & 3u);
}

/*
 * Find the shortest pure-navigation replacement that preserves the incoming
 * and final movement directions without adding turns.  A plain shortest-path
 * field is insufficient here: its route can exceed the turn budget while a
 * slightly longer route is still shorter than the emitted segment.
 */
static bool scan_find_turn_bounded_shorter_navigation_impl(
    const BitboardMap* map,
    Position start,
    Position end,
    uint8_t incoming_heading,
    uint8_t required_final_heading,
    uint16_t original_len,
    uint16_t original_turns,
    Direction* out_path,
    uint16_t* out_len
) {
    if (!map || !out_path || !out_len || start.x >= MAP_COLS || start.y >= MAP_ROWS ||
        end.x >= MAP_COLS || end.y >= MAP_ROWS || required_final_heading >= 4 ||
        original_len < 2 || !scan_nav_compact_cell_is_open(map, start.x, start.y) ||
        !scan_nav_compact_cell_is_open(map, end.x, end.y)) {
        return false;
    }

    uint16_t max_steps = (uint16_t)(original_len - 1u);
    if (max_steps > SCAN_NAV_COMPACT_MAX_STEPS) max_steps = SCAN_NAV_COMPACT_MAX_STEPS;
    if (max_steps == 0) return false;

    /* A reverse ordinary BFS is only a safe lower bound; DP owns turn state. */
    ScanNavScratch* nav = &g_scan_bfs_scratch.work.nav;
    build_scan_nav_field((BitboardMap*)map, end, nav->dist, nav->parent_dir, nav->queue);
    if (nav->dist[start.y][start.x] == 0xFFFF || nav->dist[start.y][start.x] > max_steps) {
        return false;
    }

    memset(g_scan_nav_compact_dp.turns, 0xFF, sizeof(g_scan_nav_compact_dp.turns));
    uint8_t current_layer = 0;
    uint8_t next_layer = 1;

    /* Seed t=1 separately so the no-heading state never enters the DP table. */
    for (uint8_t direction = 0; direction < 4; direction++) {
        int nx = (int)start.x + DIRECTIONS[direction].dx;
        int ny = (int)start.y + DIRECTIONS[direction].dy;
        if (!scan_nav_compact_cell_is_open(map, nx, ny)) continue;
        uint16_t remaining = nav->dist[ny][nx];
        if (remaining == 0xFFFF || 1u + remaining > max_steps) continue;

        uint16_t turns = (incoming_heading < 4 && incoming_heading != direction) ? 1u : 0u;
        if (turns <= original_turns) {
            g_scan_nav_compact_dp.turns[current_layer][direction][ny][nx] = turns;
        }
    }

    uint16_t found_len = 0;
    for (uint16_t step = 1; step <= max_steps; step++) {
        if (g_scan_nav_compact_dp.turns[current_layer][required_final_heading][end.y][end.x] <=
            original_turns) {
            found_len = step;
            break;
        }
        if (step == max_steps) break;

        memset(g_scan_nav_compact_dp.turns[next_layer], 0xFF,
               sizeof(g_scan_nav_compact_dp.turns[next_layer]));
        for (int y = 0; y < MAP_ROWS; y++) {
            for (int x = 0; x < MAP_COLS; x++) {
                for (uint8_t previous_direction = 0; previous_direction < 4; previous_direction++) {
                    uint16_t previous_turns =
                        g_scan_nav_compact_dp.turns[current_layer][previous_direction][y][x];
                    if (previous_turns == 0xFFFF) continue;

                    for (uint8_t direction = 0; direction < 4; direction++) {
                        int nx = x + DIRECTIONS[direction].dx;
                        int ny = y + DIRECTIONS[direction].dy;
                        if (!scan_nav_compact_cell_is_open(map, nx, ny)) continue;
                        uint16_t remaining = nav->dist[ny][nx];
                        if (remaining == 0xFFFF || step + 1u + remaining > max_steps) continue;

                        uint16_t candidate_turns = (uint16_t)previous_turns +
                                                   ((previous_direction != direction) ? 1u : 0u);
                        if (candidate_turns > original_turns || candidate_turns >=
                                g_scan_nav_compact_dp.turns[next_layer][direction][ny][nx]) {
                            continue;
                        }

                        Position child = {(uint8_t)nx, (uint8_t)ny};
                        g_scan_nav_compact_dp.turns[next_layer][direction][ny][nx] = candidate_turns;
                        scan_nav_compact_set_parent(
                            (uint16_t)(step + 1u), child, direction, previous_direction
                        );
                    }
                }
            }
        }

        uint8_t swap_layer = current_layer;
        current_layer = next_layer;
        next_layer = swap_layer;
    }

    if (found_len == 0) return false;

    Position cursor = end;
    uint8_t direction = required_final_heading;
    for (uint16_t step = found_len; step > 1; step--) {
        out_path[step - 1u] = DIRECTIONS[direction];
        uint8_t previous_direction = scan_nav_compact_get_parent(step, cursor, direction);
        int px = (int)cursor.x - DIRECTIONS[direction].dx;
        int py = (int)cursor.y - DIRECTIONS[direction].dy;
        if (px < 0 || px >= MAP_COLS || py < 0 || py >= MAP_ROWS || previous_direction >= 4) {
            return false;
        }
        cursor = (Position){(uint8_t)px, (uint8_t)py};
        direction = previous_direction;
    }

    out_path[0] = DIRECTIONS[direction];
    int start_x = (int)cursor.x - DIRECTIONS[direction].dx;
    int start_y = (int)cursor.y - DIRECTIONS[direction].dy;
    if (start_x != start.x || start_y != start.y) return false;

    *out_len = found_len;
    return true;
}

static inline bool scan_find_turn_bounded_shorter_navigation(
    const BitboardMap* map,
    Position start,
    Position end,
    uint8_t incoming_heading,
    uint8_t required_final_heading,
    uint16_t original_len,
    uint16_t original_turns,
    Direction* out_path,
    uint16_t* out_len
) {
    bool result = scan_find_turn_bounded_shorter_navigation_impl(
        map, start, end, incoming_heading, required_final_heading,
        original_len, original_turns, out_path, out_len
    );
    return result;
}

FAST_OCRAM_FUNC static bool scan_nearest_entity(
    SokobanSolver* solver, BitboardMap* sim_map, Position scan_curr,
    const uint16_t dist[MAP_ROWS][MAP_COLS], const uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    const Position* entities, int entity_count, bool* visited, int scanned_count, int req_count, bool is_box_scan,
    const Position* box_positions, bool* visited_boxes, int scanned_boxes, int req_boxes,
    const Position* target_positions, bool* visited_targets, int scanned_targets, int req_targets,
    int radius, int max_detour, int current_step_idx, int cutoff_idx, uint8_t scan_heading,
    bool use_lookahead,
    int* out_best_idx, Direction* out_path, uint16_t* out_len, Position* out_view_pos,
    uint8_t* out_view_heading, uint16_t* out_score
) {
    if (scanned_count >= req_count) return false;

    bool found = false;
    int best_idx = -1;
    Position best_view_pos = scan_curr;
    uint16_t best_detour = 0xFFFF;
    uint16_t best_score = 0xFFFF;
    uint8_t best_heading = scan_heading;
    uint16_t hard_max_len = (max_detour < 0) ? 0 : (uint16_t)((max_detour > 0xFFFE) ? 0xFFFE : max_detour);

    for (int i = 0; i < entity_count; i++) {
        if (visited[i] || manhattan_distance(scan_curr, entities[i]) > radius) continue;
        if (is_box_scan && scan_should_defer_high_lb_box_candidate(
            solver, sim_map, dist, entities, entity_count, visited,
            scanned_count, req_count, entities[i], hard_max_len
        )) {
            continue;
        }

        Position entity_view_pos = scan_curr;
        Direction* entity_path = g_scan_nearest_entity_path;
        uint16_t entity_detour = 0xFFFF;
        uint8_t entity_heading = scan_heading;
        uint16_t entity_score = 0xFFFF;

        const ScanNeighborLookahead* lookahead = NULL;
        if (use_lookahead) {
            g_scan_neighbor_lookahead = (ScanNeighborLookahead){
                box_positions, NULL, visited_boxes, solver->num_boxes, scanned_boxes, req_boxes,
                target_positions, NULL, visited_targets, solver->num_targets, scanned_targets, req_targets,
                is_box_scan, i
            };
            lookahead = &g_scan_neighbor_lookahead;
        }
        if (!choose_best_scan_neighbor_from_field(
                sim_map, dist, parent_dir, entities[i], hard_max_len, scan_heading,
                &entity_view_pos, entity_path, &entity_detour, &entity_heading, &entity_score,
                lookahead, 0u
            )) {
            continue;
        }

        if (entity_detour == 0xFFFF) continue;
        if (entity_detour > 0 && should_delay_scan(solver->best_path, current_step_idx, cutoff_idx, scan_curr, entities[i])) continue;
        if (entity_detour > hard_max_len) continue;

        if (scan_rank_is_better(entity_detour, entity_score, best_detour, best_score)) {
            best_detour = entity_detour;
            best_score = entity_score;
            best_idx = i;
            best_view_pos = entity_view_pos;
            best_heading = entity_heading;
            if (entity_detour > 0) memcpy(out_path, entity_path, entity_detour * sizeof(Direction));
            found = true;
        }
    }

    if (!found) return false;
    *out_best_idx = best_idx;
    *out_len = best_detour;
    *out_view_pos = best_view_pos;
    if (out_view_heading) *out_view_heading = best_heading;
    if (out_score) *out_score = best_score;
    return true;
}

static bool scan_append_plan_to_hybrid(
    SokobanSolver* solver,
    Direction* hybrid_path,
    uint16_t* hybrid_len,
    const Direction* plan_path,
    uint16_t plan_len,
    const Entity* waypoints,
    const Position* pause_positions,
    int waypoint_count
) {
    if (!solver || !hybrid_path || !hybrid_len || !plan_path || !waypoints || !pause_positions) return false;
    if (waypoint_count <= 0) return false;
    if ((uint32_t)(*hybrid_len) + plan_len >= MAX_PATH_LENGTH) return false;
    if (solver->scan_waypoint_count + waypoint_count > MAX_BOXES + MAX_TARGETS) return false;

    if (plan_len > 0) {
        memcpy(&hybrid_path[*hybrid_len], plan_path, plan_len * sizeof(Direction));
        *hybrid_len = (uint16_t)(*hybrid_len + plan_len);
    }

    for (int i = 0; i < waypoint_count; i++) {
        int wp_idx = solver->scan_waypoint_count + i;
        solver->scan_waypoints[wp_idx] = waypoints[i];
        solver->scan_player_pause_positions[wp_idx] = pause_positions[i];
    }
    solver->scan_waypoint_count += waypoint_count;
    return true;
}

FAST_OCRAM_FUNC static bool scan_plan_entities_greedy(
    SokobanSolver* solver, BitboardMap* sim_map, Position curr_p,
    Position* sim_boxes, bool* visited_boxes, bool* visited_targets,
    int* scanned_boxes_count, int* scanned_targets_count,
    int req_boxes, int req_targets,
    int radius, int max_detour,
    int current_step_idx, int cutoff_idx,
    uint8_t start_heading, bool use_lookahead,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pause_positions,
    int* out_waypoint_count, Position* out_final_pos,
    uint8_t* out_final_heading
) {
    if (!solver || !sim_map || !sim_boxes || !visited_boxes || !visited_targets ||
        !scanned_boxes_count || !scanned_targets_count || !out_path || !out_len ||
        !out_waypoints || !out_pause_positions || !out_waypoint_count || !out_final_pos) {
        return false;
    }

    Position scan_curr = curr_p;
    uint8_t scan_heading = start_heading;
    bool chained_any = false;
    uint16_t total_len = 0;
    int waypoint_count = 0;
    Position target_positions[MAX_TARGETS];

    for (int i = 0; i < solver->num_targets; i++) {
        target_positions[i] = solver->targets[i].pos;
    }

    while (waypoint_count < MAX_BOXES + MAX_TARGETS) {
        int best_idx = -1;
        bool best_is_box = false;
        uint16_t best_len = 0xFFFF;
        uint16_t best_score = 0xFFFF;
        uint8_t best_heading = scan_heading;
        Position best_view_pos = scan_curr;
        Direction* best_path = g_scan_try_best_path;

        int candidate_idx = -1;
        uint16_t candidate_len = 0xFFFF;
        uint16_t candidate_score = 0xFFFF;
        uint8_t candidate_heading = scan_heading;
        Position candidate_view_pos;
        Direction* candidate_path = g_scan_try_candidate_path;
        ScanNavScratch* try_nav = &g_scan_bfs_scratch.work.try_nav;
        uint16_t (*dist)[MAP_COLS] = try_nav->dist;
        uint8_t (*parent_dir)[MAP_COLS] = try_nav->parent_dir;

        build_scan_nav_field(sim_map, scan_curr, dist, parent_dir, try_nav->queue);

        if (scan_nearest_entity(
                solver, sim_map, scan_curr,
                dist, parent_dir,
                sim_boxes, solver->num_boxes, visited_boxes, *scanned_boxes_count, req_boxes, true,
                sim_boxes, visited_boxes, *scanned_boxes_count, req_boxes,
                target_positions, visited_targets, *scanned_targets_count, req_targets,
                radius, max_detour, current_step_idx, cutoff_idx, scan_heading,
                use_lookahead,
                &candidate_idx, candidate_path, &candidate_len, &candidate_view_pos,
                &candidate_heading, &candidate_score
            ) && scan_rank_is_better(candidate_len, candidate_score, best_len, best_score)) {
            best_idx = candidate_idx;
            best_is_box = true;
            best_len = candidate_len;
            best_score = candidate_score;
            best_heading = candidate_heading;
            best_view_pos = candidate_view_pos;
            memcpy(best_path, candidate_path, candidate_len * sizeof(Direction));
        }

        candidate_idx = -1;
        candidate_len = 0xFFFF;
        candidate_score = 0xFFFF;
        candidate_heading = scan_heading;
        if (scan_nearest_entity(
                solver, sim_map, scan_curr,
                dist, parent_dir,
                target_positions, solver->num_targets, visited_targets, *scanned_targets_count, req_targets, false,
                sim_boxes, visited_boxes, *scanned_boxes_count, req_boxes,
                target_positions, visited_targets, *scanned_targets_count, req_targets,
                radius, max_detour, current_step_idx, cutoff_idx, scan_heading,
                use_lookahead,
                &candidate_idx, candidate_path, &candidate_len, &candidate_view_pos,
                &candidate_heading, &candidate_score
            ) && scan_rank_is_better(candidate_len, candidate_score, best_len, best_score)) {
            best_idx = candidate_idx;
            best_is_box = false;
            best_len = candidate_len;
            best_score = candidate_score;
            best_heading = candidate_heading;
            best_view_pos = candidate_view_pos;
            memcpy(best_path, candidate_path, candidate_len * sizeof(Direction));
        }

        if (best_idx == -1) break;
        if (total_len + best_len + 1 >= MAX_PATH_LENGTH) break;

        if (best_len > 0) {
            memcpy(&out_path[total_len], best_path, best_len * sizeof(Direction));
            total_len = (uint16_t)(total_len + best_len);
        }

        out_path[total_len].dx = 0;
        out_path[total_len].dy = 0;
        total_len++;

        if (best_is_box) {
            out_waypoints[waypoint_count].pos = sim_boxes[best_idx];
            out_waypoints[waypoint_count].id = scan_waypoint_box_tag(best_idx);
            visited_boxes[best_idx] = true;
            (*scanned_boxes_count)++;
        } else {
            out_waypoints[waypoint_count].pos = target_positions[best_idx];
            out_waypoints[waypoint_count].id = scan_waypoint_target_tag(best_idx);
            visited_targets[best_idx] = true;
            (*scanned_targets_count)++;
        }

        out_waypoints[waypoint_count].is_active = true;
        out_pause_positions[waypoint_count] = best_view_pos;
        waypoint_count++;
        scan_curr = best_view_pos;
        scan_heading = best_heading;
        chained_any = true;
    }

    if (!chained_any) return false;
    *out_len = total_len;
    *out_waypoint_count = waypoint_count;
    *out_final_pos = scan_curr;
    if (out_final_heading) *out_final_heading = scan_heading;
    return true;
}
// 串联多个扫描点，把扫描暂停动作以 dx=0, dy=0 的哨兵步写入混合路径。
FAST_OCRAM_FUNC static bool try_scan_entities_impl(
    SokobanSolver* solver, BitboardMap* sim_map, Position curr_p,
    Position* sim_boxes, bool* visited_boxes, bool* visited_targets,
    int* scanned_boxes_count, int* scanned_targets_count,
    int req_boxes, int req_targets,
    int radius, int max_detour,
    int current_step_idx, int cutoff_idx,
    Direction* hybrid_path, uint16_t* hybrid_len, Position* out_scan_curr,
    uint8_t* io_scan_heading
) {
    Position scan_curr = curr_p;
    uint8_t scan_heading = io_scan_heading ? *io_scan_heading : SCAN_INITIAL_HEADING_DIR;
    Position target_positions[MAX_TARGETS];

    for (int i = 0; i < solver->num_targets; i++) {
        target_positions[i] = solver->targets[i].pos;
    }

    bool beam_visited_boxes[MAX_BOXES];
    bool beam_visited_targets[MAX_TARGETS];
    memcpy(beam_visited_boxes, visited_boxes, sizeof(beam_visited_boxes));
    memcpy(beam_visited_targets, visited_targets, sizeof(beam_visited_targets));

    uint16_t beam_len = 0;
    int beam_waypoint_count = 0;
    Position beam_final_pos = scan_curr;
    uint8_t beam_final_heading = scan_heading;
    bool beam_ok = scan_plan_paired_tasks_beam(
        solver, sim_map, scan_curr, scan_heading,
        sim_boxes, target_positions,
        beam_visited_boxes, beam_visited_targets,
        req_boxes, req_targets,
        radius, max_detour,
        g_scan_beam_scratch.planned_path, &beam_len,
        g_scan_beam_scratch.planned_waypoints, g_scan_beam_scratch.planned_pauses,
        &beam_waypoint_count, &beam_final_pos, &beam_final_heading
    ) && beam_waypoint_count > 0;

    bool greedy_visited_boxes[MAX_BOXES];
    bool greedy_visited_targets[MAX_TARGETS];
    memcpy(greedy_visited_boxes, visited_boxes, sizeof(greedy_visited_boxes));
    memcpy(greedy_visited_targets, visited_targets, sizeof(greedy_visited_targets));

    int greedy_scanned_boxes = *scanned_boxes_count;
    int greedy_scanned_targets = *scanned_targets_count;
    uint16_t greedy_len = 0;
    int greedy_waypoint_count = 0;
    Position greedy_final_pos = scan_curr;
    uint8_t greedy_final_heading = scan_heading;
    bool greedy_ok = scan_plan_entities_greedy(
        solver, sim_map, scan_curr,
        sim_boxes, greedy_visited_boxes, greedy_visited_targets,
        &greedy_scanned_boxes, &greedy_scanned_targets,
        req_boxes, req_targets,
        radius, max_detour,
        current_step_idx, cutoff_idx,
        scan_heading, true,
        g_scan_beam_scratch.greedy_path, &greedy_len,
        g_scan_beam_scratch.greedy_waypoints, g_scan_beam_scratch.greedy_pauses,
        &greedy_waypoint_count, &greedy_final_pos,
        &greedy_final_heading
    );

    if (beam_ok && (!greedy_ok || scan_beam_should_replace_greedy(
            g_scan_beam_scratch.planned_path, beam_len,
            g_scan_beam_scratch.greedy_path, greedy_len
        ))) {
        if (beam_len > 0) memcpy(g_scan_beam_scratch.greedy_path, g_scan_beam_scratch.planned_path, beam_len * sizeof(Direction));
        if (beam_waypoint_count > 0) {
            memcpy(g_scan_beam_scratch.greedy_waypoints, g_scan_beam_scratch.planned_waypoints, beam_waypoint_count * sizeof(Entity));
            memcpy(g_scan_beam_scratch.greedy_pauses, g_scan_beam_scratch.planned_pauses, beam_waypoint_count * sizeof(Position));
        }
        memcpy(greedy_visited_boxes, beam_visited_boxes, sizeof(bool) * MAX_BOXES);
        memcpy(greedy_visited_targets, beam_visited_targets, sizeof(bool) * MAX_TARGETS);
        greedy_len = beam_len;
        greedy_waypoint_count = beam_waypoint_count;
        greedy_final_pos = beam_final_pos;
        greedy_final_heading = beam_final_heading;
        greedy_ok = true;
    }

    bool safe_visited_boxes[MAX_BOXES];
    bool safe_visited_targets[MAX_TARGETS];
    memcpy(safe_visited_boxes, visited_boxes, sizeof(safe_visited_boxes));
    memcpy(safe_visited_targets, visited_targets, sizeof(safe_visited_targets));

    int safe_scanned_boxes = *scanned_boxes_count;
    int safe_scanned_targets = *scanned_targets_count;
    uint16_t safe_len = 0;
    int safe_waypoint_count = 0;
    Position safe_final_pos = scan_curr;
    uint8_t safe_final_heading = scan_heading;
    bool safe_ok = scan_plan_entities_greedy(
        solver, sim_map, scan_curr,
        sim_boxes, safe_visited_boxes, safe_visited_targets,
        &safe_scanned_boxes, &safe_scanned_targets,
        req_boxes, req_targets,
        radius, max_detour,
        current_step_idx, cutoff_idx,
        scan_heading, false,
        g_scan_beam_scratch.planned_path, &safe_len,
        g_scan_beam_scratch.planned_waypoints, g_scan_beam_scratch.planned_pauses,
        &safe_waypoint_count, &safe_final_pos,
        &safe_final_heading
    );

    if (safe_ok && (!greedy_ok || !scan_beam_should_replace_greedy(
            g_scan_beam_scratch.greedy_path, greedy_len,
            g_scan_beam_scratch.planned_path, safe_len
        ))) {
        if (scan_append_plan_to_hybrid(
                solver, hybrid_path, hybrid_len,
                g_scan_beam_scratch.planned_path, safe_len,
                g_scan_beam_scratch.planned_waypoints, g_scan_beam_scratch.planned_pauses,
                safe_waypoint_count
            )) {
            memcpy(visited_boxes, safe_visited_boxes, sizeof(bool) * MAX_BOXES);
            memcpy(visited_targets, safe_visited_targets, sizeof(bool) * MAX_TARGETS);
            *scanned_boxes_count = safe_scanned_boxes;
            *scanned_targets_count = safe_scanned_targets;
            *out_scan_curr = safe_final_pos;
            if (io_scan_heading) *io_scan_heading = safe_final_heading;
            return true;
        }
    }

    if (greedy_ok) {
        if (scan_append_plan_to_hybrid(
                solver, hybrid_path, hybrid_len,
                g_scan_beam_scratch.greedy_path, greedy_len,
                g_scan_beam_scratch.greedy_waypoints, g_scan_beam_scratch.greedy_pauses,
                greedy_waypoint_count
            )) {
            memcpy(visited_boxes, greedy_visited_boxes, sizeof(bool) * MAX_BOXES);
            memcpy(visited_targets, greedy_visited_targets, sizeof(bool) * MAX_TARGETS);

            int new_scanned_boxes = 0;
            int new_scanned_targets = 0;
            for (int i = 0; i < solver->num_boxes; i++) if (visited_boxes[i]) new_scanned_boxes++;
            for (int i = 0; i < solver->num_targets; i++) if (visited_targets[i]) new_scanned_targets++;
            *scanned_boxes_count = new_scanned_boxes;
            *scanned_targets_count = new_scanned_targets;

            *out_scan_curr = greedy_final_pos;
            if (io_scan_heading) *io_scan_heading = greedy_final_heading;
            return true;
        }
    }

    return false;
}

FAST_OCRAM_FUNC static inline bool try_scan_entities(
    SokobanSolver* solver, BitboardMap* sim_map, Position curr_p,
    Position* sim_boxes, bool* visited_boxes, bool* visited_targets,
    int* scanned_boxes_count, int* scanned_targets_count,
    int req_boxes, int req_targets,
    int radius, int max_detour,
    int current_step_idx, int cutoff_idx,
    Direction* hybrid_path, uint16_t* hybrid_len, Position* out_scan_curr,
    uint8_t* io_scan_heading
) {
    bool result = try_scan_entities_impl(
        solver, sim_map, curr_p, sim_boxes,
        visited_boxes, visited_targets,
        scanned_boxes_count, scanned_targets_count,
        req_boxes, req_targets, radius, max_detour,
        current_step_idx, cutoff_idx,
        hybrid_path, hybrid_len, out_scan_curr, io_scan_heading
    );
    return result;
}

/* Build a residual-map route with the normal mixed greedy/beam scan planner. */
FAST_OCRAM_FUNC bool sokoban_plan_rescan_route(
    const SokobanSolver* solver,
    Position start_pos,
    uint8_t start_heading,
    const bool* visited_boxes,
    const bool* visited_targets,
    int req_boxes,
    int req_targets,
    Direction* out_path,
    uint16_t* out_len,
    Entity* out_waypoints,
    Position* out_pause_positions,
    int* out_waypoint_count,
    Position* out_final_pos,
    uint8_t* out_final_heading
) {
    SokobanSolver work;
    BitboardMap sim_map;
    Position sim_boxes[MAX_BOXES];
    bool local_visited_boxes[MAX_BOXES];
    bool local_visited_targets[MAX_TARGETS];
    int scanned_boxes = 0;
    int scanned_targets = 0;
    uint16_t route_len = 0u;
    Position final_pos = start_pos;
    uint8_t final_heading = start_heading;
    PathReplayOptions replay_options = scan_replay_lenient_options();

    if (!solver || !visited_boxes || !visited_targets || !out_path || !out_len ||
        !out_waypoints || !out_pause_positions || !out_waypoint_count ||
        !out_final_pos || !out_final_heading || req_boxes < 0 || req_targets < 0 ||
        req_boxes > solver->num_boxes || req_targets > solver->num_targets) {
        return false;
    }

    memcpy(local_visited_boxes, visited_boxes, sizeof(local_visited_boxes));
    memcpy(local_visited_targets, visited_targets, sizeof(local_visited_targets));
    for (int i = 0; i < solver->num_boxes && i < MAX_BOXES; i++) {
        sim_boxes[i] = solver->boxes[i].pos;
        if (local_visited_boxes[i]) scanned_boxes++;
    }
    for (int i = 0; i < solver->num_targets && i < MAX_TARGETS; i++) {
        if (local_visited_targets[i]) scanned_targets++;
    }

    if (scanned_boxes >= req_boxes && scanned_targets >= req_targets) return false;

    work = *solver;
    work.start_player = start_pos;
    work.strict_target_mode = false;
    work.scan_waypoint_count = 0;
    work.scan_current_index = 0;
    work.best_path = out_path;
    work.best_path_len = 0u;
    work.best_steps = 0xFFFFu;
    sim_map = work.bmap;
    scan_prefix_capture_initial_state(&work);

    if (!try_scan_entities(
            &work, &sim_map, start_pos, sim_boxes,
            local_visited_boxes, local_visited_targets,
            &scanned_boxes, &scanned_targets,
            req_boxes, req_targets,
            99, 999, -1, -1,
            out_path, &route_len, &final_pos, &final_heading
        )) {
        return false;
    }
    if (work.scan_waypoint_count <= 0 ||
        work.scan_waypoint_count > MAX_BOXES + MAX_TARGETS ||
        route_len >= MAX_PATH_LENGTH) {
        return false;
    }

    work.best_path_len = route_len;
    work.best_steps = route_len;
    if (!scan_prefix_rebind_eager_observations_for_waypoints(
            &work,
            work.scan_waypoints,
            work.scan_player_pause_positions,
            work.scan_waypoints,
            work.scan_waypoint_count,
            &replay_options) ||
        !scan_prefix_validate_first_arrival_observations(
            &work,
            work.best_path,
            work.best_path_len,
            work.scan_waypoints,
            work.scan_player_pause_positions,
            work.scan_waypoint_count,
            &replay_options)) {
        return false;
    }
    route_len = work.best_path_len;

    memcpy(out_waypoints, work.scan_waypoints,
           (size_t)work.scan_waypoint_count * sizeof(Entity));
    memcpy(out_pause_positions, work.scan_player_pause_positions,
           (size_t)work.scan_waypoint_count * sizeof(Position));
    *out_len = route_len;
    *out_waypoint_count = work.scan_waypoint_count;
    *out_final_pos = final_pos;
    *out_final_heading = final_heading;
    return true;
}
FAST_OCRAM_FUNC static bool scan_choose_best_rejoin_after_detour(
    SokobanSolver* solver, const BitboardMap* sim_map,
    Position scan_curr, Position curr_p,
    int current_step_idx, int cutoff_idx,
    Direction* out_path, uint16_t* out_len,
    Position* out_rejoin_pos, int* out_rejoin_next_idx
) {
    static Direction candidate_path[MAX_SINGLE_PATH];
    bool found = false;
    int best_score = 1000000;
    int best_skip = -1;
    uint16_t best_len = 0xFFFF;
    Position best_pos = curr_p;
    Position candidate_pos = curr_p;
    int upper_next_idx = cutoff_idx + 1;

    for (int next_idx = current_step_idx; next_idx <= upper_next_idx; next_idx++) {
        uint16_t candidate_len = 0;
        if (scan_cached_astar_navigate(
                solver, sim_map, scan_curr, candidate_pos,
                MASK_WALL | MASK_BOMB | MASK_BOX,
                candidate_path, &candidate_len
            )) {
            int skip = next_idx - current_step_idx;
            int score = (int)candidate_len - skip;
            if (!found || score < best_score || (score == best_score && skip > best_skip)) {
                if (candidate_len < MAX_SINGLE_PATH) {
                    if (candidate_len > 0) {
                        memcpy(out_path, candidate_path, candidate_len * sizeof(Direction));
                    }
                    found = true;
                    best_score = score;
                    best_skip = skip;
                    best_len = candidate_len;
                    best_pos = candidate_pos;
                    if (out_rejoin_next_idx) *out_rejoin_next_idx = next_idx;
                }
            }
        }

        if (next_idx >= cutoff_idx) break;
        Direction d = solver->best_path[next_idx];
        Position next_pos = {
            (uint8_t)(candidate_pos.x + d.dx),
            (uint8_t)(candidate_pos.y + d.dy)
        };
        if (get_bit(sim_map->bombs, next_pos.x, next_pos.y) ||
            get_bit(sim_map->boxes, next_pos.x, next_pos.y)) {
            break;
        }
        candidate_pos = next_pos;
    }

    if (!found) return false;
    if (out_len) *out_len = best_len;
    if (out_rejoin_pos) *out_rejoin_pos = best_pos;
    return true;
}
/* Spine-only tie-break order; the global solver order remains U,D,L,R. */
static const uint8_t g_scan_spine_dir_order[4] = {2u, 3u, 0u, 1u};

static void scan_spine_reset_state(void) {
    memset(g_scan_spine_states, 0, sizeof(g_scan_spine_states));
    memset(g_scan_spine_order, 0, sizeof(g_scan_spine_order));
}
static uint16_t scan_spine_state_bucket_index(
    bool legacy_mode, uint16_t step, Position pos,
    uint16_t box_mask, uint16_t target_mask, uint8_t heading
) {
    uint32_t key = 2166136261u;
    key = (key ^ step) * 16777619u;
    key = (key ^ ((uint32_t)pos.y * MAP_COLS + pos.x)) * 16777619u;
    key = (key ^ (legacy_mode ? box_mask : heading)) * 16777619u;
    if (legacy_mode) key = (key ^ target_mask) * 16777619u;
    return (uint16_t)(key & (SCAN_SPINE_STATE_BUCKET_COUNT - 1u));
}

static void scan_spine_state_index_reset(void) {
    memset(g_scan_spine_state_buckets, 0xFF, sizeof(g_scan_spine_state_buckets));
}

static void scan_spine_state_index_insert(
    const ScanSpineState* state, uint16_t state_idx, bool legacy_mode
) {
    uint16_t bucket_idx = scan_spine_state_bucket_index(
        legacy_mode, state->step, state->pos,
        state->box_mask, state->target_mask, state->heading
    );
    ScanSpineStateBucket* bucket = &g_scan_spine_state_buckets[bucket_idx];
    g_scan_spine_order[state_idx] = SCAN_SPINE_NO_PARENT;
    if (bucket->head == SCAN_SPINE_NO_PARENT) {
        bucket->head = state_idx;
    } else {
        g_scan_spine_order[bucket->tail] = state_idx;
    }
    bucket->tail = state_idx;
}

static int scan_spine_popcount_u16(uint16_t value) {
    int count = 0;
    while (value) {
        count += value & 1u;
        value >>= 1;
    }
    return count;
}

static inline void scan_anonymous_set_path_dir(uint8_t* bits, uint16_t index, uint8_t dir) {
    uint8_t shift = (uint8_t)((index & 3u) * 2u);
    uint8_t mask = (uint8_t)(3u << shift);
    bits[index >> 2] = (uint8_t)((bits[index >> 2] & (uint8_t)(~mask)) |
                                 (uint8_t)((dir & 3u) << shift));
}

static inline uint8_t scan_anonymous_get_path_dir(const uint8_t* bits, uint16_t index) {
    return (uint8_t)((bits[index >> 2] >> ((index & 3u) * 2u)) & 3u);
}

static inline void scan_anonymous_apply_visible(
    const SokobanSolver* solver, Position pos,
    int req_boxes, int req_targets,
    uint16_t* box_mask, uint16_t* target_mask
) {
    int remaining = req_boxes - scan_spine_popcount_u16(*box_mask);
    for (int b = 0; b < solver->num_boxes && remaining > 0; b++) {
        uint16_t bit = (uint16_t)(1u << b);
        Position entity_pos = solver->boxes[b].pos;
        if ((*box_mask & bit) == 0u && solver->boxes[b].is_active &&
            is_in_bounds(entity_pos.x, entity_pos.y) &&
            manhattan_distance(pos, entity_pos) == 1u) {
            *box_mask = (uint16_t)(*box_mask | bit);
            remaining--;
        }
    }

    remaining = req_targets - scan_spine_popcount_u16(*target_mask);
    for (int t = 0; t < solver->num_targets && remaining > 0; t++) {
        uint16_t bit = (uint16_t)(1u << t);
        Position entity_pos = solver->targets[t].pos;
        if ((*target_mask & bit) == 0u && solver->targets[t].is_active &&
            is_in_bounds(entity_pos.x, entity_pos.y) &&
            manhattan_distance(pos, entity_pos) == 1u) {
            *target_mask = (uint16_t)(*target_mask | bit);
            remaining--;
        }
    }
}

static inline bool scan_anonymous_same_key(
    const ScanSpineState* state, Position pos, uint8_t heading,
    uint16_t box_mask, uint16_t target_mask
) {
    return state->heading == heading && state->box_mask == box_mask &&
           state->target_mask == target_mask && pos_equal(state->pos, pos);
}

FAST_OCRAM_FUNC static bool scan_anonymous_store_candidate(
    int parent_idx, uint8_t dir, Position pos, uint8_t heading,
    uint16_t box_mask, uint16_t target_mask,
    uint16_t cost, uint16_t step,
    uint32_t run_score, uint16_t run_len,
    int* candidate_count
) {
    uint32_t pos_idx = (uint32_t)pos.y * MAP_COLS + pos.x;
    uint32_t key = pos_idx | ((uint32_t)heading << 8) |
                   ((uint32_t)box_mask << 11) | ((uint32_t)target_mask << 21);
    uint32_t hash_idx = (key * 2654435761u) & (SCAN_ANONYMOUS_HASH_SIZE - 1u);

    for (uint32_t probe = 0; probe < SCAN_ANONYMOUS_HASH_SIZE; probe++) {
        uint16_t slot = g_scan_anonymous_hash[hash_idx];
        if (slot == SCAN_SPINE_NO_PARENT) {
            if (*candidate_count >= (int)SCAN_ANONYMOUS_CANDIDATE_LIMIT) return false;
            slot = (uint16_t)(SCAN_ANONYMOUS_CANDIDATE_BASE + (uint32_t)(*candidate_count));
            (*candidate_count)++;
            g_scan_anonymous_hash[hash_idx] = slot;

            ScanSpineState* child = &g_scan_spine_states[slot];
            memset(child, 0, sizeof(*child));
            child->cost = cost;
            child->parent = (uint16_t)parent_idx;
            child->step = step;
            child->pos = pos;
            child->box_mask = box_mask;
            child->target_mask = target_mask;
            child->heading = heading;
            memcpy(g_scan_anonymous_path_bits[slot],
                   g_scan_anonymous_path_bits[parent_idx],
                   SCAN_ANONYMOUS_PATH_BYTES);
            scan_anonymous_set_path_dir(g_scan_anonymous_path_bits[slot],
                                        (uint16_t)(step - 1u), dir);
            g_scan_anonymous_run_score[slot] = run_score;
            g_scan_anonymous_run_len[slot] = run_len;
            return true;
        }

        ScanSpineState* existing = &g_scan_spine_states[slot];
        if (scan_anonymous_same_key(existing, pos, heading, box_mask, target_mask)) {
            if (cost < existing->cost ||
                (cost == existing->cost && run_score > g_scan_anonymous_run_score[slot])) {
                existing->cost = cost;
                existing->parent = (uint16_t)parent_idx;
                existing->step = step;
                memcpy(g_scan_anonymous_path_bits[slot],
                       g_scan_anonymous_path_bits[parent_idx],
                       SCAN_ANONYMOUS_PATH_BYTES);
                scan_anonymous_set_path_dir(g_scan_anonymous_path_bits[slot],
                                            (uint16_t)(step - 1u), dir);
                g_scan_anonymous_run_score[slot] = run_score;
                g_scan_anonymous_run_len[slot] = run_len;
            }
            return true;
        }
        hash_idx = (hash_idx + 1u) & (SCAN_ANONYMOUS_HASH_SIZE - 1u);
    }
    return false;
}

static inline int scan_anonymous_remaining(
    const ScanSpineState* state, int req_boxes, int req_targets
) {
    int remaining = req_boxes - scan_spine_popcount_u16(state->box_mask) +
                    req_targets - scan_spine_popcount_u16(state->target_mask);
    return remaining > 0 ? remaining : 0;
}

static inline bool scan_anonymous_state_is_better(
    uint16_t a_idx, uint16_t b_idx, int req_boxes, int req_targets
) {
    const ScanSpineState* a = &g_scan_spine_states[a_idx];
    const ScanSpineState* b = &g_scan_spine_states[b_idx];
    int a_remaining = scan_anonymous_remaining(a, req_boxes, req_targets);
    int b_remaining = scan_anonymous_remaining(b, req_boxes, req_targets);
    uint32_t a_priority = (uint32_t)a->cost +
                          (uint32_t)a_remaining * SCAN_ANONYMOUS_PROGRESS_WEIGHT;
    uint32_t b_priority = (uint32_t)b->cost +
                          (uint32_t)b_remaining * SCAN_ANONYMOUS_PROGRESS_WEIGHT;
    if (a_priority != b_priority) return a_priority < b_priority;
    if (a_remaining != b_remaining) return a_remaining < b_remaining;
    if (a->cost != b->cost) return a->cost < b->cost;
    if (g_scan_anonymous_run_score[a_idx] != g_scan_anonymous_run_score[b_idx]) {
        return g_scan_anonymous_run_score[a_idx] > g_scan_anonymous_run_score[b_idx];
    }
    return a_idx < b_idx;
}

FAST_OCRAM_FUNC static int scan_anonymous_select_frontier(
    int candidate_count, int req_boxes, int req_targets
) {
    int selected_count = 0;
    for (int i = 0; i < candidate_count; i++) {
        uint16_t candidate = (uint16_t)(SCAN_ANONYMOUS_CANDIDATE_BASE + (uint32_t)i);
        if (selected_count < (int)SCAN_ANONYMOUS_BEAM_WIDTH) {
            int pos = selected_count++;
            g_scan_spine_order[pos] = candidate;
            while (pos > 0) {
                int parent = (pos - 1) / 2;
                if (!scan_anonymous_state_is_better(
                        g_scan_spine_order[parent], g_scan_spine_order[pos],
                        req_boxes, req_targets)) break;
                uint16_t tmp = g_scan_spine_order[parent];
                g_scan_spine_order[parent] = g_scan_spine_order[pos];
                g_scan_spine_order[pos] = tmp;
                pos = parent;
            }
        } else if (scan_anonymous_state_is_better(
                       candidate, g_scan_spine_order[0], req_boxes, req_targets)) {
            g_scan_spine_order[0] = candidate;
            int pos = 0;
            while (true) {
                int left = pos * 2 + 1;
                if (left >= selected_count) break;
                int right = left + 1;
                int worst = left;
                if (right < selected_count && scan_anonymous_state_is_better(
                        g_scan_spine_order[left], g_scan_spine_order[right],
                        req_boxes, req_targets)) {
                    worst = right;
                }
                if (!scan_anonymous_state_is_better(
                        g_scan_spine_order[pos], g_scan_spine_order[worst],
                        req_boxes, req_targets)) break;
                uint16_t tmp = g_scan_spine_order[pos];
                g_scan_spine_order[pos] = g_scan_spine_order[worst];
                g_scan_spine_order[worst] = tmp;
                pos = worst;
            }
        }
    }

    for (int i = 1; i < selected_count; i++) {
        uint16_t key = g_scan_spine_order[i];
        int j = i;
        while (j > 0 && scan_anonymous_state_is_better(
                key, g_scan_spine_order[j - 1], req_boxes, req_targets)) {
            g_scan_spine_order[j] = g_scan_spine_order[j - 1];
            j--;
        }
        g_scan_spine_order[j] = key;
    }

    for (int i = 0; i < selected_count; i++) {
        uint16_t source = g_scan_spine_order[i];
        g_scan_spine_states[i] = g_scan_spine_states[source];
        memcpy(g_scan_anonymous_path_bits[i], g_scan_anonymous_path_bits[source],
               SCAN_ANONYMOUS_PATH_BYTES);
        g_scan_anonymous_run_score[i] = g_scan_anonymous_run_score[source];
        g_scan_anonymous_run_len[i] = g_scan_anonymous_run_len[source];
    }
    return selected_count;
}

FAST_OCRAM_FUNC static bool scan_anonymous_emit_visible(
    const SokobanSolver* solver, Position pos,
    int req_boxes, int req_targets,
    uint16_t* box_mask, uint16_t* target_mask,
    Direction* out_path, uint16_t* path_len,
    Entity* out_waypoints, Position* out_pauses, int* waypoint_count
) {
    uint16_t old_boxes = *box_mask;
    uint16_t old_targets = *target_mask;
    scan_anonymous_apply_visible(
        solver, pos, req_boxes, req_targets, box_mask, target_mask
    );

    for (int b = 0; b < solver->num_boxes; b++) {
        uint16_t bit = (uint16_t)(1u << b);
        if (((*box_mask ^ old_boxes) & bit) == 0u) continue;
        if (*path_len >= MAX_PATH_LENGTH || *waypoint_count >= MAX_BOXES + MAX_TARGETS) return false;
        out_path[(*path_len)++] = (Direction){0, 0};
        out_waypoints[*waypoint_count] = solver->boxes[b];
        out_waypoints[*waypoint_count].id = scan_waypoint_box_tag(b);
        out_waypoints[*waypoint_count].is_active = true;
        out_pauses[*waypoint_count] = pos;
        (*waypoint_count)++;
    }
    for (int t = 0; t < solver->num_targets; t++) {
        uint16_t bit = (uint16_t)(1u << t);
        if (((*target_mask ^ old_targets) & bit) == 0u) continue;
        if (*path_len >= MAX_PATH_LENGTH || *waypoint_count >= MAX_BOXES + MAX_TARGETS) return false;
        out_path[(*path_len)++] = (Direction){0, 0};
        out_waypoints[*waypoint_count] = solver->targets[t];
        out_waypoints[*waypoint_count].id = scan_waypoint_target_tag(t);
        out_waypoints[*waypoint_count].is_active = true;
        out_pauses[*waypoint_count] = pos;
        (*waypoint_count)++;
    }
    return true;
}

/* A conservative lower bound for the remaining anonymous observations. */
static uint16_t scan_anonymous_entity_view_distance(
    Position from, Position entity
) {
    /* The nearest geometric neighbour is one Manhattan step closer whenever
     * the entity and player differ.  For coincident cells, every neighbour is
     * one step away.  This is equivalent to the four-direction loop above,
     * including map-edge entities, while avoiding four bounds checks per state. */
    uint16_t distance = manhattan_distance(from, entity);
    return distance == 0u ? 1u : (uint16_t)(distance - 1u);
}

static uint16_t scan_anonymous_remaining_move_lower_bound(
    const SokobanSolver* solver, const ScanSpineState* state,
    int req_boxes, int req_targets
) {
    if (!solver || !state) return 0u;

    int missing_boxes = req_boxes - scan_spine_popcount_u16(state->box_mask);
    int missing_targets = req_targets - scan_spine_popcount_u16(state->target_mask);
    if (missing_boxes < 0) missing_boxes = 0;
    if (missing_targets < 0) missing_targets = 0;

    uint16_t box_distances[MAX_BOXES];
    int box_count = 0;
    for (int b = 0; b < solver->num_boxes; b++) {
        if (state->box_mask & (uint16_t)(1u << b)) continue;
        uint16_t distance = scan_anonymous_entity_view_distance(
            state->pos, solver->boxes[b].pos
        );
        int insert = box_count;
        while (insert > 0 && box_distances[insert - 1] > distance) {
            box_distances[insert] = box_distances[insert - 1];
            insert--;
        }
        if (box_count < MAX_BOXES) {
            box_distances[insert] = distance;
            box_count++;
        }
    }

    uint16_t target_distances[MAX_TARGETS];
    int target_count = 0;
    for (int t = 0; t < solver->num_targets; t++) {
        if (state->target_mask & (uint16_t)(1u << t)) continue;
        uint16_t distance = scan_anonymous_entity_view_distance(
            state->pos, solver->targets[t].pos
        );
        int insert = target_count;
        while (insert > 0 && target_distances[insert - 1] > distance) {
            target_distances[insert] = target_distances[insert - 1];
            insert--;
        }
        if (target_count < MAX_TARGETS) {
            target_distances[insert] = distance;
            target_count++;
        }
    }

    uint16_t max_distance = 0u;
    if (missing_boxes > 0 && missing_boxes <= box_count &&
        box_distances[missing_boxes - 1] > max_distance) {
        max_distance = box_distances[missing_boxes - 1];
    }
    if (missing_targets > 0 && missing_targets <= target_count &&
        target_distances[missing_targets - 1] > max_distance) {
        max_distance = target_distances[missing_targets - 1];
    }

    uint32_t bound = (uint32_t)max_distance * SCAN_STEP_SCORE;
    return bound >= 0xFFFFu ? 0xFFFEu : (uint16_t)bound;
}

FAST_OCRAM_FUNC static bool scan_plan_observation_coverage(
    SokobanSolver* solver, Position start_pos,
    int req_boxes, int req_targets, uint16_t incumbent_cost,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pause_positions,
    int* out_waypoint_count, Position* out_final_pos
) {
    if (!solver || !out_path || !out_len || !out_waypoints || !out_pause_positions ||
        !out_waypoint_count || !out_final_pos || solver->num_bombs != 0 ||
        solver->strict_target_mode || req_boxes < 0 || req_targets < 0 ||
        req_boxes > solver->num_boxes || req_targets > solver->num_targets) {
        return false;
    }
    for (int i = 0; i < solver->num_boxes; i++) {
        if (solver->boxes[i].id != -1) return false;
    }
    for (int i = 0; i < solver->num_targets; i++) {
        if (solver->targets[i].id != -1) return false;
    }

    int required_waypoints = req_boxes + req_targets;
    if (required_waypoints <= 0 || required_waypoints > MAX_BOXES + MAX_TARGETS ||
        required_waypoints >= MAX_PATH_LENGTH) {
        return false;
    }

    ScanSpineState* start = &g_scan_spine_states[0];
    memset(start, 0, sizeof(*start));
    start->parent = SCAN_SPINE_NO_PARENT;
    start->pos = start_pos;
    start->heading = SCAN_INITIAL_HEADING_DIR;
    scan_anonymous_apply_visible(
        solver, start_pos, req_boxes, req_targets,
        &start->box_mask, &start->target_mask
    );
    memset(g_scan_anonymous_path_bits[0], 0, SCAN_ANONYMOUS_PATH_BYTES);
    g_scan_anonymous_run_score[0] = 0;
    g_scan_anonymous_run_len[0] = 0;

    bool best_found = false;
    uint16_t best_cost = 0xFFFFu;
    uint16_t best_move_len = 0;
    uint32_t best_run_score = 0;
    Position best_final_pos = start_pos;
    int current_count = 1;
    uint16_t max_moves = (uint16_t)(MAX_PATH_LENGTH - required_waypoints - 1);
    if (incumbent_cost != 0xFFFFu) {
        uint16_t cost_limited_moves = (uint16_t)(incumbent_cost / SCAN_STEP_SCORE);
        if (max_moves > cost_limited_moves) max_moves = cost_limited_moves;
    }

    if (scan_spine_popcount_u16(start->box_mask) >= req_boxes &&
        scan_spine_popcount_u16(start->target_mask) >= req_targets) {
        best_found = true;
        best_cost = 0;
        memcpy(g_scan_anonymous_best_path_bits, g_scan_anonymous_path_bits[0],
               SCAN_ANONYMOUS_PATH_BYTES);
    }

    for (uint16_t step = 1; step <= max_moves && current_count > 0; step++) {
        memset(g_scan_anonymous_hash, 0xFF, sizeof(g_scan_anonymous_hash));
        int candidate_count = 0;

        for (int i = 0; i < current_count; i++) {
            ScanSpineState* parent = &g_scan_spine_states[i];
            uint16_t active_limit = best_found ? best_cost : incumbent_cost;
            if (active_limit != 0xFFFFu) {
#if !defined(SCAN_ANONYMOUS_DISABLE_DISTANCE_BOUND)
                uint16_t lower_bound = scan_anonymous_remaining_move_lower_bound(
                    solver, parent, req_boxes, req_targets
                );
                if ((uint32_t)parent->cost + lower_bound > active_limit) continue;
#endif
            }

            for (uint8_t dir = 0; dir < 4u; dir++) {
                int nx = (int)parent->pos.x + DIRECTIONS[dir].dx;
                int ny = (int)parent->pos.y + DIRECTIONS[dir].dy;
                if (is_strict_scan_obstacle(&solver->bmap, nx, ny)) continue;

                Position next_pos = {(uint8_t)nx, (uint8_t)ny};
                uint16_t box_mask = parent->box_mask;
                uint16_t target_mask = parent->target_mask;
                scan_anonymous_apply_visible(
                    solver, next_pos, req_boxes, req_targets,
                    &box_mask, &target_mask
                );

                uint16_t cost = (uint16_t)(parent->cost + SCAN_STEP_SCORE);
                if (parent->heading < 4u && parent->heading != dir) {
                    cost = (uint16_t)(cost + SCAN_BEND_SLOWDOWN_SCORE);
                }
                if (active_limit != 0xFFFFu && cost > active_limit) continue;
                uint16_t run_len;
                uint32_t run_score;
                if (parent->heading == dir) {
                    uint16_t old_run = g_scan_anonymous_run_len[i];
                    run_len = (uint16_t)(old_run + 1u);
                    run_score = g_scan_anonymous_run_score[i] -
                                (uint32_t)old_run * old_run +
                                (uint32_t)run_len * run_len;
                } else {
                    run_len = 1u;
                    run_score = g_scan_anonymous_run_score[i] + 1u;
                }

                bool complete = scan_spine_popcount_u16(box_mask) >= req_boxes &&
                                scan_spine_popcount_u16(target_mask) >= req_targets;
                if (complete) {
                    bool better = !best_found || cost < best_cost;
                    if (!better && cost == best_cost && best_move_len > 0) {
                        uint64_t candidate_scaled = (uint64_t)run_score *
                                                    best_move_len * best_move_len;
                        uint64_t best_scaled = (uint64_t)best_run_score * step * step;
                        better = candidate_scaled > best_scaled ||
                                 (candidate_scaled == best_scaled && step < best_move_len);
                    }
                    if (better) {
                        best_found = true;
                        best_cost = cost;
                        best_move_len = step;
                        best_run_score = run_score;
                        best_final_pos = next_pos;
                        memcpy(g_scan_anonymous_best_path_bits,
                               g_scan_anonymous_path_bits[i],
                               SCAN_ANONYMOUS_PATH_BYTES);
                        scan_anonymous_set_path_dir(
                            g_scan_anonymous_best_path_bits, (uint16_t)(step - 1u), dir
                        );
                    }
                    continue;
                }

                if (!scan_anonymous_store_candidate(
                        i, dir, next_pos, dir, box_mask, target_mask,
                        cost, step, run_score, run_len, &candidate_count)) {
                    return false;
                }
            }
        }

#if SCAN_ANONYMOUS_STOP_ON_FIRST_COMPLETE
        if (best_found) break;
#endif

        current_count = scan_anonymous_select_frontier(
            candidate_count, req_boxes, req_targets
        );
    }

    if (!best_found) return false;

    uint16_t path_len = 0;
    int waypoint_count = 0;
    uint16_t box_mask = 0;
    uint16_t target_mask = 0;
    Position current_pos = start_pos;
    if (!scan_anonymous_emit_visible(
            solver, current_pos, req_boxes, req_targets,
            &box_mask, &target_mask,
            out_path, &path_len, out_waypoints, out_pause_positions, &waypoint_count)) {
        return false;
    }

    for (uint16_t i = 0; i < best_move_len; i++) {
        uint8_t dir = scan_anonymous_get_path_dir(g_scan_anonymous_best_path_bits, i);
        int nx = (int)current_pos.x + DIRECTIONS[dir].dx;
        int ny = (int)current_pos.y + DIRECTIONS[dir].dy;
        if (path_len >= MAX_PATH_LENGTH || is_strict_scan_obstacle(&solver->bmap, nx, ny)) {
            return false;
        }
        out_path[path_len++] = DIRECTIONS[dir];
        current_pos = (Position){(uint8_t)nx, (uint8_t)ny};
        if (!scan_anonymous_emit_visible(
                solver, current_pos, req_boxes, req_targets,
                &box_mask, &target_mask,
                out_path, &path_len, out_waypoints, out_pause_positions, &waypoint_count)) {
            return false;
        }
    }

    if (waypoint_count != required_waypoints ||
        scan_spine_popcount_u16(box_mask) != req_boxes ||
        scan_spine_popcount_u16(target_mask) != req_targets ||
        !pos_equal(current_pos, best_final_pos) || path_len >= MAX_PATH_LENGTH) {
        return false;
    }

    *out_len = path_len;
    *out_waypoint_count = waypoint_count;
    *out_final_pos = current_pos;
    return true;
}

/*
 * Pauses are zero-cost spine actions.  Account for entities that can be
 * observed immediately at the current cell when ranking the fixed state
 * pool; otherwise a state beside two useful entities looks no better than a
 * state beside one and the bounded search systematically misses good chains.
 */
static uint16_t scan_spine_priority_remaining(
    const SokobanSolver* solver,
    const ScanSpineFrame* frame,
    const ScanSpineState* state,
    int req_boxes,
    int req_targets
) {
    if (!solver || !frame || !state) return 0xFFFFu;

    int missing_boxes = req_boxes - scan_spine_popcount_u16(state->box_mask);
    int missing_targets = req_targets - scan_spine_popcount_u16(state->target_mask);
    if (missing_boxes < 0) missing_boxes = 0;
    if (missing_targets < 0) missing_targets = 0;

    for (int b = 0; b < solver->num_boxes && missing_boxes > 0; b++) {
        if (state->box_mask & (uint16_t)(1u << b)) continue;
        Position pos = frame->boxes[b];
        if (pos.x != 0xFFu && manhattan_distance(state->pos, pos) == 1) {
            missing_boxes--;
        }
    }

    for (int t = 0; t < solver->num_targets && missing_targets > 0; t++) {
        if (state->target_mask & (uint16_t)(1u << t)) continue;
        Position pos = solver->targets[t].pos;
        if (is_in_bounds(pos.x, pos.y) &&
            get_bit(frame->map.targets, pos.x, pos.y) &&
            !get_bit(frame->map.bombs, pos.x, pos.y) &&
            manhattan_distance(state->pos, pos) == 1) {
            missing_targets--;
        }
    }

    return (uint16_t)(missing_boxes + missing_targets);
}
FAST_OCRAM_FUNC static void scan_spine_heap_rank(
    uint16_t state_idx, uint32_t* out_priority, uint16_t* out_remaining
) {
    const ScanSpineState* state = &g_scan_spine_states[state_idx];
    uint16_t remaining = g_scan_spine_state_heap_remaining[state_idx];
    uint32_t terminal_bias = 0u;
    if (g_scan_spine_heap_context.favor_progress) {
        if (g_scan_spine_heap_context.free_terminal) {
            terminal_bias = (uint32_t)(
                g_scan_spine_heap_context.prefix_len - state->step
            ) * SCAN_SPINE_STEP_PROGRESS_WEIGHT;
        } else {
            terminal_bias = manhattan_distance(
                state->pos,
                g_scan_spine_frames[g_scan_spine_heap_context.prefix_len].pos
            );
        }
    }
    *out_remaining = remaining;
    *out_priority = (uint32_t)state->cost +
        (uint32_t)remaining * SCAN_SPINE_PROGRESS_WEIGHT + terminal_bias;
}

FAST_OCRAM_FUNC static bool scan_spine_heap_less(uint16_t a, uint16_t b) {
    uint32_t a_priority;
    uint32_t b_priority;
    uint16_t a_remaining;
    uint16_t b_remaining;
    scan_spine_heap_rank(a, &a_priority, &a_remaining);
    scan_spine_heap_rank(b, &b_priority, &b_remaining);
    if (a_priority != b_priority) return a_priority < b_priority;
    if (g_scan_spine_heap_context.favor_progress && a_remaining != b_remaining) {
        return a_remaining < b_remaining;
    }
    return a < b;
}

FAST_OCRAM_FUNC static void scan_spine_heap_swap(uint16_t a, uint16_t b) {
    uint16_t a_state = g_scan_spine_state_heap[a];
    uint16_t b_state = g_scan_spine_state_heap[b];
    g_scan_spine_state_heap[a] = b_state;
    g_scan_spine_state_heap[b] = a_state;
    g_scan_spine_state_heap_pos[a_state] = b;
    g_scan_spine_state_heap_pos[b_state] = a;
}

FAST_OCRAM_FUNC static void scan_spine_heap_sift_up(uint16_t pos) {
    while (pos > 0u) {
        uint16_t parent = (uint16_t)((pos - 1u) >> 1);
        if (!scan_spine_heap_less(
                g_scan_spine_state_heap[pos], g_scan_spine_state_heap[parent])) {
            break;
        }
        scan_spine_heap_swap(pos, parent);
        pos = parent;
    }
}

FAST_OCRAM_FUNC static bool scan_spine_heap_offer(uint16_t state_idx) {
    const ScanSpineState* state = &g_scan_spine_states[state_idx];
    uint16_t remaining = 0u;
    if (g_scan_spine_heap_context.favor_progress) {
        if (g_scan_spine_heap_context.free_terminal) {
            remaining = scan_spine_priority_remaining(
                g_scan_spine_heap_context.solver,
                &g_scan_spine_frames[state->step], state,
                g_scan_spine_heap_context.req_boxes,
                g_scan_spine_heap_context.req_targets
            );
        } else {
            int missing_boxes = g_scan_spine_heap_context.req_boxes -
                                scan_spine_popcount_u16(state->box_mask);
            int missing_targets = g_scan_spine_heap_context.req_targets -
                                  scan_spine_popcount_u16(state->target_mask);
            if (missing_boxes < 0) missing_boxes = 0;
            if (missing_targets < 0) missing_targets = 0;
            remaining = (uint16_t)(missing_boxes + missing_targets);
        }
    }
    g_scan_spine_state_heap_remaining[state_idx] = (uint8_t)remaining;

    uint16_t pos = g_scan_spine_state_heap_pos[state_idx];
    if (pos == SCAN_SPINE_NO_PARENT) {
        if (g_scan_spine_heap_context.count >= SCAN_SPINE_STATE_LIMIT) return false;
        pos = g_scan_spine_heap_context.count++;
        g_scan_spine_state_heap[pos] = state_idx;
        g_scan_spine_state_heap_pos[state_idx] = pos;
    }
    scan_spine_heap_sift_up(pos);
    return true;
}

FAST_OCRAM_FUNC static int scan_spine_heap_pop(void) {
    while (g_scan_spine_heap_context.count > 0u) {
        uint16_t result = g_scan_spine_state_heap[0];
        uint16_t last = g_scan_spine_state_heap[--g_scan_spine_heap_context.count];
        g_scan_spine_state_heap_pos[result] = SCAN_SPINE_NO_PARENT;
        if (g_scan_spine_heap_context.count > 0u) {
            uint16_t pos = 0u;
            g_scan_spine_state_heap[0] = last;
            g_scan_spine_state_heap_pos[last] = 0u;
            for (;;) {
                uint16_t left = (uint16_t)(pos * 2u + 1u);
                if (left >= g_scan_spine_heap_context.count) break;
                uint16_t right = (uint16_t)(left + 1u);
                uint16_t best = left;
                if (right < g_scan_spine_heap_context.count &&
                    scan_spine_heap_less(g_scan_spine_state_heap[right],
                                         g_scan_spine_state_heap[left])) {
                    best = right;
                }
                if (!scan_spine_heap_less(g_scan_spine_state_heap[best],
                                          g_scan_spine_state_heap[pos])) {
                    break;
                }
                scan_spine_heap_swap(pos, best);
                pos = best;
            }
        }
        if (!g_scan_spine_states[result].closed) return (int)result;
    }
    return -1;
}

static void scan_spine_heap_reset(
    SokobanSolver* solver, int prefix_len, int req_boxes, int req_targets,
    bool favor_progress, bool free_terminal
) {
    memset(g_scan_spine_state_heap_pos, 0xFF, sizeof(g_scan_spine_state_heap_pos));
    g_scan_spine_heap_context.solver = solver;
    g_scan_spine_heap_context.prefix_len = prefix_len;
    g_scan_spine_heap_context.req_boxes = req_boxes;
    g_scan_spine_heap_context.req_targets = req_targets;
    g_scan_spine_heap_context.count = 0u;
    g_scan_spine_heap_context.favor_progress = favor_progress;
    g_scan_spine_heap_context.free_terminal = free_terminal;
}

static uint16_t scan_spine_route_cost_from_path(
    const Direction* path, uint16_t len, uint8_t start_heading, uint8_t* out_heading
) {
    uint32_t cost = 0;
    uint8_t heading = start_heading;
    uint8_t previous_dir = start_heading;
    if (!path) return 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t dir = direction_index(path[i]);
        if (dir >= 4u) continue;
        cost += SCAN_STEP_SCORE;
        if (previous_dir < 4u && dir != previous_dir) {
            cost += SCAN_BEND_SLOWDOWN_SCORE;
        }
        previous_dir = dir;
        heading = (uint8_t)dir;
    }
    if (out_heading) *out_heading = heading;
    return (cost >= 0xFFFFu) ? 0xFFFEu : (uint16_t)cost;
}

/*
 * Reconstruct a spine navigation route with the requested 2M+7B metric.
 *
 * The ordinary navigation BFS deliberately keeps only one parent per cell.
 * That is sufficient for distance, but it makes equal-length routes depend on
 * the global U/D/L/R expansion order and can add needless bends.  The spine
 * has its own metric, so keep a direction-aware DP over the shortest-path DAG
 * here.  This helper is intentionally separate from the legacy navigation
 * compactor: changing that compactor would change no-bomb scan behaviour.
 */
FAST_OCRAM_FUNC static bool scan_spine_reconstruct_nav_route(
    const uint16_t dist[MAP_ROWS][MAP_COLS],
    const uint8_t parent_dir[MAP_ROWS][MAP_COLS],
    Position target,
    uint8_t start_heading,
    uint8_t requested_heading,
    Direction* out_path,
    uint16_t* out_len,
    uint8_t* out_heading,
    uint16_t* out_cost
) {
    if (!dist || !parent_dir || !out_path || !out_len || !out_heading || !out_cost ||
        target.x >= MAP_COLS || target.y >= MAP_ROWS) {
        return false;
    }
    (void)parent_dir;
    uint16_t len = dist[target.y][target.x];
    if (len == 0xFFFF || len >= MAX_SINGLE_PATH) return false;
    if (len == 0) {
        *out_len = 0;
        *out_heading = start_heading;
        *out_cost = 0;
        return true;
    }

    /*
     * Direction order is local to the spine.  Keep it aligned with the
     * route-quality oracle (L,R,U,D) without changing the global DIRECTIONS
     * order used by the solver and ordinary scan paths.
     */
    if (!g_scan_spine_turn_cache_valid ||
        !pos_equal(g_scan_spine_turn_cache_target, target) ||
        g_scan_spine_turn_cache_start_heading != start_heading) {
        memset(g_scan_nav_turn_cost, 0xFF, sizeof(g_scan_nav_turn_cost));
        memset(g_scan_nav_turn_parent, 0xFF, sizeof(g_scan_nav_turn_parent));

        for (uint16_t qi = 0u; qi < g_scan_spine_nav_order_count; qi++) {
            Position cell = g_scan_bfs_scratch.work.nav.queue[qi];
            int x = cell.x;
            int y = cell.y;
            uint16_t step = dist[y][x];
            if (step == 0u) continue;
            if (step > len) break;
            for (int oi = 0; oi < 4; oi++) {
                uint8_t dir = g_scan_spine_dir_order[oi];
                int px = x - DIRECTIONS[dir].dx;
                int py = y - DIRECTIONS[dir].dy;
                if (px < 0 || px >= MAP_COLS || py < 0 || py >= MAP_ROWS) continue;
                if (dist[py][px] + 1u != step) continue;

                uint16_t best_turns = 0xFFFFu;
                uint8_t best_parent = 0xFFu;
                if (step == 1u) {
                    best_turns = (start_heading < 4u && start_heading != dir) ? 1u : 0u;
                    best_parent = 4u;
                } else {
                    for (int pi = 0; pi < 4; pi++) {
                        uint8_t prev_dir = g_scan_spine_dir_order[pi];
                        uint16_t prev_turns = g_scan_nav_turn_cost[prev_dir][py][px];
                        if (prev_turns == 0xFFFFu) continue;
                        uint16_t turns = (uint16_t)(prev_turns + (prev_dir != dir ? 1u : 0u));
                        if (turns < best_turns) {
                            best_turns = turns;
                            best_parent = prev_dir;
                        }
                    }
                }
                if (best_turns != 0xFFFFu) {
                    g_scan_nav_turn_cost[dir][y][x] = best_turns;
                    g_scan_nav_turn_parent[dir][y][x] = best_parent;
                }
            }
        }
        g_scan_spine_turn_cache_target = target;
        g_scan_spine_turn_cache_start_heading = start_heading;
        g_scan_spine_turn_cache_valid = true;
    }

    int best_dir = -1;
    uint16_t best_turns = 0xFFFFu;
    if (requested_heading < 4u) {
        best_dir = requested_heading;
        best_turns = g_scan_nav_turn_cost[best_dir][target.y][target.x];
    } else {
        for (int oi = 0; oi < 4; oi++) {
            uint8_t dir = g_scan_spine_dir_order[oi];
            uint16_t turns = g_scan_nav_turn_cost[dir][target.y][target.x];
            if (turns < best_turns) {
                best_turns = turns;
                best_dir = dir;
            }
        }
    }
    if (best_dir < 0) return false;

    Position curr = target;
    int dir = best_dir;
    for (int i = (int)len - 1; i >= 0; i--) {
        if (dir < 0 || dir >= 4) return false;
        out_path[i] = DIRECTIONS[dir];
        uint8_t parent = g_scan_nav_turn_parent[dir][curr.y][curr.x];
        int px = (int)curr.x - DIRECTIONS[dir].dx;
        int py = (int)curr.y - DIRECTIONS[dir].dy;
        if (px < 0 || px >= MAP_COLS || py < 0 || py >= MAP_ROWS) return false;
        curr = (Position){(uint8_t)px, (uint8_t)py};
        if (parent == 4u) {
            if (i != 0) return false;
            break;
        }
        if (parent >= 4u) return false;
        dir = parent;
    }

    uint8_t heading = start_heading;
    uint16_t cost = scan_spine_route_cost_from_path(out_path, len, start_heading, &heading);
    if (cost == 0xFFFFu) return false;
    *out_len = len;
    *out_heading = heading;
    *out_cost = cost;
    return true;
}

FAST_OCRAM_FUNC static bool scan_spine_build_frames(
    SokobanSolver* solver, int cutoff_idx,
    ScanSpineFrame* frames, int* out_frame_count
) {
    if (!solver || cutoff_idx < 0 || cutoff_idx >= solver->best_path_len) return false;
    int prefix_len = cutoff_idx + 1;
    if (prefix_len >= MAX_PATH_LENGTH) return false;

    PathReplayState replay_state;
    if (!scan_replay_load_from_solver(
            &replay_state,
            &solver->bmap,
            solver->start_player,
            solver->boxes,
            solver->num_boxes,
            solver->bombs,
            solver->num_bombs)) {
        return false;
    }
    PathReplayOptions replay_options = scan_replay_lenient_options();

    for (int idx = 0; idx <= prefix_len; idx++) {
        frames[idx].map = replay_state.map;
        frames[idx].pos = replay_state.player;
        memcpy(frames[idx].boxes, replay_state.boxes, sizeof(frames[idx].boxes));
        frames[idx].next_is_stateful = false;

        if (idx >= prefix_len) break;

        Direction d = solver->best_path[idx];
        int nx = (int)replay_state.player.x + d.dx;
        int ny = (int)replay_state.player.y + d.dy;
        if (!is_in_bounds(nx, ny)) return false;
        Position next_p = {(uint8_t)nx, (uint8_t)ny};
        frames[idx].next_is_stateful = direction_index(d) < 4u &&
                                       (get_bit(replay_state.map.bombs, next_p.x, next_p.y) ||
                                        get_bit(replay_state.map.boxes, next_p.x, next_p.y));

        if (!scan_replay_step_existing(solver, &replay_state, d, &replay_options, NULL)) return false;
    }

    *out_frame_count = prefix_len + 1;
    return true;
}

FAST_OCRAM_FUNC static bool scan_spine_relax_state(
    ScanSpineState* states, int* state_count,
    int parent_idx, uint16_t new_cost,
    uint16_t step, Position pos, uint16_t box_mask, uint16_t target_mask,
    bool legacy_mode, uint8_t heading, ScanSpineAction action, int8_t entity_idx
) {
    int existing = -1;
    uint16_t bucket_idx = scan_spine_state_bucket_index(
        legacy_mode, step, pos, box_mask, target_mask, heading
    );
    if (legacy_mode) {
        if (new_cost >= MAX_PATH_LENGTH) return true;
        for (uint16_t cursor = g_scan_spine_state_buckets[bucket_idx].head;
             cursor != SCAN_SPINE_NO_PARENT; cursor = g_scan_spine_order[cursor]) {
            int i = (int)cursor;
            if (states[i].step == step && pos_equal(states[i].pos, pos) &&
                states[i].box_mask == box_mask && states[i].target_mask == target_mask) {
                existing = i;
                break;
            }
        }
        if (existing >= 0) {
            if (!states[existing].closed && new_cost < states[existing].cost) {
                states[existing].cost = new_cost;
                states[existing].parent = (uint16_t)parent_idx;
                states[existing].action = (uint8_t)action;
                states[existing].entity_idx = entity_idx;
                if (!scan_spine_heap_offer((uint16_t)existing)) return false;
            }
            return true;
        }
    } else {
        if (new_cost >= 0xFFFFu) return true;
        for (uint16_t cursor = g_scan_spine_state_buckets[bucket_idx].head;
             cursor != SCAN_SPINE_NO_PARENT; cursor = g_scan_spine_order[cursor]) {
            int i = (int)cursor;
            if (states[i].step != step || !pos_equal(states[i].pos, pos) ||
                states[i].heading != heading) {
                continue;
            }

            bool existing_covers_new =
                (states[i].box_mask & box_mask) == box_mask &&
                (states[i].target_mask & target_mask) == target_mask;
            if (existing_covers_new && states[i].cost <= new_cost) {
                return true;
            }

            bool new_covers_existing =
                (box_mask & states[i].box_mask) == states[i].box_mask &&
                (target_mask & states[i].target_mask) == states[i].target_mask;
            if (new_covers_existing && new_cost <= states[i].cost) {
                states[i].closed = true;
            }
            if (states[i].box_mask == box_mask && states[i].target_mask == target_mask) {
                existing = i;
            }
        }
        if (existing >= 0) {
            if (new_cost < states[existing].cost) {
                states[existing].cost = new_cost;
                states[existing].parent = (uint16_t)parent_idx;
                states[existing].action = (uint8_t)action;
                states[existing].entity_idx = entity_idx;
                states[existing].closed = false;
                if (!scan_spine_heap_offer((uint16_t)existing)) return false;
            }
            return true;
        }
    }

    if (*state_count >= SCAN_SPINE_STATE_LIMIT) return false;
    int idx = (*state_count)++;
    states[idx].cost = new_cost;
    states[idx].parent = (uint16_t)parent_idx;
    states[idx].step = step;
    states[idx].pos = pos;
    states[idx].box_mask = box_mask;
    states[idx].target_mask = target_mask;
    states[idx].action = (uint8_t)action;
    states[idx].entity_idx = entity_idx;
    states[idx].heading = heading;
    states[idx].closed = false;
    scan_spine_state_index_insert(&states[idx], (uint16_t)idx, legacy_mode);
    return scan_spine_heap_offer((uint16_t)idx);
}

static bool scan_spine_append_directions(
    Direction* out_path, uint16_t* out_len,
    const Direction* segment, uint16_t segment_len
) {
    if ((uint32_t)(*out_len) + segment_len >= MAX_PATH_LENGTH) return false;
    if (segment_len > 0) {
        memcpy(&out_path[*out_len], segment, segment_len * sizeof(Direction));
        *out_len = (uint16_t)(*out_len + segment_len);
    }
    return true;
}

FAST_OCRAM_FUNC static bool scan_spine_reconstruct_plan(
    SokobanSolver* solver, const ScanSpineFrame* frames,
    const ScanSpineState* states, int final_idx,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pauses, int* out_waypoint_count
) {
    int order_count = 0;
    int idx = final_idx;
    while (idx >= 0 && order_count < SCAN_SPINE_STATE_LIMIT) {
        g_scan_spine_order[order_count++] = (uint16_t)idx;
        if (states[idx].parent == SCAN_SPINE_NO_PARENT) break;
        idx = states[idx].parent;
    }
    if (order_count <= 1 || idx < 0) return false;

    *out_len = 0;
    *out_waypoint_count = 0;

    for (int oi = order_count - 2; oi >= 0; oi--) {
        const ScanSpineState* child = &states[g_scan_spine_order[oi]];
        const ScanSpineState* parent = &states[child->parent];
        Direction segment[MAX_SINGLE_PATH];
        uint16_t segment_len = 0;

        if (child->action == SCAN_SPINE_ACTION_ADVANCE) {
            if (*out_len >= MAX_PATH_LENGTH) return false;
            out_path[(*out_len)++] = solver->best_path[parent->step];
            continue;
        }

        if (child->action == SCAN_SPINE_ACTION_REJOIN ||
            child->action == SCAN_SPINE_ACTION_SCAN_BOX ||
            child->action == SCAN_SPINE_ACTION_SCAN_TARGET) {
            ScanNavScratch* nav = &g_scan_bfs_scratch.work.nav;
            build_scan_nav_field((BitboardMap*)&frames[parent->step].map, parent->pos,
                                 nav->dist, nav->parent_dir, nav->queue);
            scan_spine_build_nav_order(nav->dist, nav->queue);
            g_scan_spine_turn_cache_valid = false;
            if (child->heading >= 4u) {
                if (!reconstruct_scan_nav_path(
                        nav->dist, nav->parent_dir, child->pos,
                        segment, &segment_len)) {
                    return false;
                }
            } else {
                uint8_t segment_heading = parent->heading;
                uint16_t segment_cost = 0;
                if (!scan_spine_reconstruct_nav_route(
                        nav->dist, nav->parent_dir, child->pos, parent->heading,
                        child->heading,
                        segment, &segment_len, &segment_heading, &segment_cost)) {
                    return false;
                }
                (void)segment_heading;
                (void)segment_cost;
            }
            if (!scan_spine_append_directions(out_path, out_len, segment, segment_len)) return false;
        }

        if (child->action == SCAN_SPINE_ACTION_SCAN_BOX || child->action == SCAN_SPINE_ACTION_SCAN_TARGET) {
            if (*out_len + 1 >= MAX_PATH_LENGTH) return false;
            if (*out_waypoint_count >= MAX_BOXES + MAX_TARGETS) return false;
            out_path[*out_len] = (Direction){0, 0};
            (*out_len)++;

            int entity_idx = child->entity_idx;
            if (child->action == SCAN_SPINE_ACTION_SCAN_BOX) {
                if (entity_idx < 0 || entity_idx >= solver->num_boxes) return false;
                out_waypoints[*out_waypoint_count].pos = frames[parent->step].boxes[entity_idx];
                out_waypoints[*out_waypoint_count].id = scan_waypoint_box_tag(entity_idx);
            } else {
                if (entity_idx < 0 || entity_idx >= solver->num_targets) return false;
                out_waypoints[*out_waypoint_count].pos = solver->targets[entity_idx].pos;
                out_waypoints[*out_waypoint_count].id = scan_waypoint_target_tag(entity_idx);
            }
            out_waypoints[*out_waypoint_count].is_active = true;
            out_pauses[*out_waypoint_count] = child->pos;
            (*out_waypoint_count)++;
        }
    }

    return *out_waypoint_count > 0;
}

FAST_OCRAM_FUNC static bool scan_spine_plan_impl(
    SokobanSolver* solver, int cutoff_idx, int req_boxes, int req_targets,
    bool favor_progress, bool free_terminal,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pauses, int* out_waypoint_count
) {
    int frame_count = 0;
    if (!scan_spine_build_frames(solver, cutoff_idx, g_scan_spine_frames, &frame_count)) return false;
    int prefix_len = frame_count - 1;

    ScanSpineState* states = g_scan_spine_states;
    scan_spine_state_index_reset();
    scan_spine_heap_reset(solver, prefix_len, req_boxes, req_targets,
                           favor_progress, free_terminal);
    int state_count = 1;
    states[0].cost = 0;
    states[0].parent = SCAN_SPINE_NO_PARENT;
    states[0].step = 0;
    states[0].pos = g_scan_spine_frames[0].pos;
    states[0].box_mask = 0;
    states[0].target_mask = 0;
    states[0].action = SCAN_SPINE_ACTION_START;
    states[0].entity_idx = -1;
    states[0].heading = SCAN_INITIAL_HEADING_DIR;
    states[0].closed = false;
    scan_spine_state_index_insert(&states[0], 0u, !free_terminal);
    if (!scan_spine_heap_offer(0u)) return false;

    int final_idx = -1;
    bool overflow = false;

    while (!overflow) {
        int best_idx = scan_spine_heap_pop();
        if (best_idx < 0) break;
        uint32_t best_priority = states[best_idx].cost;
        if (favor_progress) {
            uint16_t best_remaining;
            scan_spine_heap_rank((uint16_t)best_idx, &best_priority, &best_remaining);
        }
        if (free_terminal && final_idx >= 0 &&
            best_priority > (uint32_t)states[final_idx].cost +
                                SCAN_SPINE_IMPROVEMENT_PRIORITY_SLACK) {
            break;
        }

        ScanSpineState* curr = &states[best_idx];
        curr->closed = true;
        if (scan_spine_popcount_u16(curr->box_mask) >= req_boxes &&
            scan_spine_popcount_u16(curr->target_mask) >= req_targets &&
            curr->step == prefix_len &&
            (free_terminal || pos_equal(curr->pos, g_scan_spine_frames[prefix_len].pos))) {
            if (final_idx < 0 || curr->cost < states[final_idx].cost) {
                final_idx = best_idx;
            }
            if (!free_terminal) break;
            continue;
        }

        const ScanSpineFrame* frame = &g_scan_spine_frames[curr->step];
        ScanNavScratch* nav = &g_scan_bfs_scratch.work.nav;
        scan_spine_build_nav_dist_cached((BitboardMap*)&frame->map, curr->pos,
                                         nav->dist, nav->parent_dir, nav->queue);

        for (int rejoin = curr->step; rejoin <= prefix_len; rejoin++) {
            if (free_terminal && rejoin != curr->step && rejoin != prefix_len &&
                !g_scan_spine_frames[rejoin].next_is_stateful) {
                continue;
            }
            Position target_pos = g_scan_spine_frames[rejoin].pos;
            uint16_t route_len = nav->dist[target_pos.y][target_pos.x];
            if (route_len != 0xFFFF &&
                !(rejoin == curr->step && pos_equal(curr->pos, target_pos))) {
                if (!free_terminal) {
                    if (!scan_spine_relax_state(
                            states, &state_count, best_idx, (uint16_t)(curr->cost + route_len),
                            (uint16_t)rejoin, target_pos, curr->box_mask, curr->target_mask,
                            true, DIRECTION_INDEX_NONE, SCAN_SPINE_ACTION_REJOIN, -1
                        )) {
                        overflow = true;
                    }
                } else {
                    for (uint8_t requested_heading = 0; requested_heading < 4u; requested_heading++) {
                        uint16_t candidate_len = route_len;
                        uint16_t route_cost = 0;
                        uint8_t route_heading = curr->heading;
                        if (!scan_spine_reconstruct_nav_route(
                                nav->dist, nav->parent_dir, target_pos, curr->heading,
                                requested_heading, g_scan_compact_path, &candidate_len,
                                &route_heading, &route_cost)) {
                            continue;
                        }
                        if (!scan_spine_relax_state(
                                states, &state_count, best_idx, (uint16_t)(curr->cost + route_cost),
                                (uint16_t)rejoin, target_pos, curr->box_mask, curr->target_mask,
                                false, route_heading, SCAN_SPINE_ACTION_REJOIN, -1
                            )) {
                            overflow = true;
                            break;
                        }
                    }
                }
                if (overflow) break;
            }
            if (rejoin >= prefix_len || g_scan_spine_frames[rejoin].next_is_stateful) break;
        }
        if (overflow) break;

        if (curr->step < prefix_len && pos_equal(curr->pos, frame->pos)) {
            Position next_pos = g_scan_spine_frames[curr->step + 1].pos;
            Direction advance = solver->best_path[curr->step];
            uint8_t advance_dir = direction_index(advance);
            uint8_t next_heading = free_terminal ? curr->heading : DIRECTION_INDEX_NONE;
            uint16_t advance_cost = free_terminal ? 0u : 1u;
            if (free_terminal && advance_dir < 4u) {
                advance_cost = SCAN_STEP_SCORE;
                if (curr->heading < 4 && curr->heading != (uint8_t)advance_dir) {
                    advance_cost = (uint16_t)(advance_cost + SCAN_BEND_SLOWDOWN_SCORE);
                }
                next_heading = (uint8_t)advance_dir;
            }
            if (!scan_spine_relax_state(
                    states, &state_count, best_idx, (uint16_t)(curr->cost + advance_cost),
                    (uint16_t)(curr->step + 1), next_pos, curr->box_mask, curr->target_mask,
                    !free_terminal, next_heading, SCAN_SPINE_ACTION_ADVANCE, -1
                )) {
                overflow = true;
                break;
            }
        }

        for (int b = 0;
             b < solver->num_boxes &&
             (!free_terminal || scan_spine_popcount_u16(curr->box_mask) < req_boxes);
             b++) {
            if (curr->box_mask & (uint16_t)(1u << b)) continue;
            Position entity_pos = frame->boxes[b];
            if (entity_pos.x == 0xFF || manhattan_distance(curr->pos, entity_pos) > 4) continue;
            for (int oi = 0; oi < 4; oi++) {
                int d = free_terminal ? g_scan_spine_dir_order[oi] : oi;
                int vx = (int)entity_pos.x + DIRECTIONS[d].dx;
                int vy = (int)entity_pos.y + DIRECTIONS[d].dy;
                if (!is_in_bounds(vx, vy)) continue;
                uint16_t route_len = nav->dist[vy][vx];
                if (route_len == 0xFFFF || route_len > 3) continue;
                Position view_pos = {(uint8_t)vx, (uint8_t)vy};
                if (!free_terminal) {
                    if (!scan_spine_relax_state(
                            states, &state_count, best_idx, (uint16_t)(curr->cost + route_len + 1u),
                            curr->step, view_pos, (uint16_t)(curr->box_mask | (uint16_t)(1u << b)), curr->target_mask,
                            true, DIRECTION_INDEX_NONE, SCAN_SPINE_ACTION_SCAN_BOX, (int8_t)b
                        )) {
                        overflow = true;
                    }
                } else {
                    for (uint8_t requested_heading = 0; requested_heading < 4u; requested_heading++) {
                        uint16_t candidate_len = route_len;
                        uint16_t route_cost = 0;
                        uint8_t route_heading = curr->heading;
                        if (!scan_spine_reconstruct_nav_route(
                                nav->dist, nav->parent_dir, view_pos, curr->heading,
                                requested_heading, g_scan_compact_path, &candidate_len,
                                &route_heading, &route_cost)) {
                            continue;
                        }
                        if (!scan_spine_relax_state(
                                states, &state_count, best_idx, (uint16_t)(curr->cost + route_cost),
                                curr->step, view_pos, (uint16_t)(curr->box_mask | (uint16_t)(1u << b)), curr->target_mask,
                                false, route_heading, SCAN_SPINE_ACTION_SCAN_BOX, (int8_t)b
                            )) {
                            overflow = true;
                            break;
                        }
                    }
                }
                if (overflow) break;
            }
            if (overflow) break;
        }
        if (overflow) break;

        for (int t = 0;
             t < solver->num_targets &&
             (!free_terminal || scan_spine_popcount_u16(curr->target_mask) < req_targets);
             t++) {
            if (curr->target_mask & (uint16_t)(1u << t)) continue;
            Position entity_pos = solver->targets[t].pos;
            if (manhattan_distance(curr->pos, entity_pos) > 4) continue;
            for (int oi = 0; oi < 4; oi++) {
                int d = free_terminal ? g_scan_spine_dir_order[oi] : oi;
                int vx = (int)entity_pos.x + DIRECTIONS[d].dx;
                int vy = (int)entity_pos.y + DIRECTIONS[d].dy;
                if (!is_in_bounds(vx, vy)) continue;
                uint16_t route_len = nav->dist[vy][vx];
                if (route_len == 0xFFFF || route_len > 3) continue;
                Position view_pos = {(uint8_t)vx, (uint8_t)vy};
                if (!free_terminal) {
                    if (!scan_spine_relax_state(
                            states, &state_count, best_idx, (uint16_t)(curr->cost + route_len + 1u),
                            curr->step, view_pos, curr->box_mask, (uint16_t)(curr->target_mask | (uint16_t)(1u << t)),
                            true, DIRECTION_INDEX_NONE, SCAN_SPINE_ACTION_SCAN_TARGET, (int8_t)t
                        )) {
                        overflow = true;
                    }
                } else {
                    for (uint8_t requested_heading = 0; requested_heading < 4u; requested_heading++) {
                        uint16_t candidate_len = route_len;
                        uint16_t route_cost = 0;
                        uint8_t route_heading = curr->heading;
                        if (!scan_spine_reconstruct_nav_route(
                                nav->dist, nav->parent_dir, view_pos, curr->heading,
                                requested_heading, g_scan_compact_path, &candidate_len,
                                &route_heading, &route_cost)) {
                            continue;
                        }
                        if (!scan_spine_relax_state(
                                states, &state_count, best_idx, (uint16_t)(curr->cost + route_cost),
                                curr->step, view_pos, curr->box_mask, (uint16_t)(curr->target_mask | (uint16_t)(1u << t)),
                                false, route_heading, SCAN_SPINE_ACTION_SCAN_TARGET, (int8_t)t
                            )) {
                            overflow = true;
                            break;
                        }
                    }
                }
                if (overflow) break;
            }
            if (overflow) break;
        }
    }

    /* A complete-looking state is not enough if the bounded pool was cut
     * short while generating its frontier.  Let the established hybrid path
     * own that fallback instead of submitting a partial search result. */
    if (overflow || final_idx < 0) return false;
    return scan_spine_reconstruct_plan(
        solver, g_scan_spine_frames, states, final_idx,
        out_path, out_len, out_waypoints, out_pauses, out_waypoint_count
    );
}

FAST_OCRAM_FUNC static inline bool scan_spine_plan(
    SokobanSolver* solver, int cutoff_idx, int req_boxes, int req_targets,
    bool favor_progress, bool free_terminal,
    Direction* out_path, uint16_t* out_len,
    Entity* out_waypoints, Position* out_pauses, int* out_waypoint_count
) {
    bool result = scan_spine_plan_impl(
        solver, cutoff_idx, req_boxes, req_targets,
        favor_progress, free_terminal,
        out_path, out_len, out_waypoints, out_pauses, out_waypoint_count
    );
    return result;
}
// 生成扫描路径：先求一条主解，再在爆破前插入必要的箱子/目标观察绕行。

static void scan_prefix_capture_initial_state(SokobanSolver* solver) {
    g_scan_initial_bmap = solver->bmap;
    g_scan_initial_player = solver->start_player;
    memcpy(g_scan_initial_boxes, solver->boxes, sizeof(solver->boxes));
    memcpy(g_scan_initial_bombs, solver->bombs, sizeof(solver->bombs));
    g_scan_initial_num_bombs = solver->num_bombs;
}

static bool scan_bomb_main_route_pack(
    ScanBombMainRouteCandidate* candidate,
    const Direction* path,
    uint16_t path_len
) {
    if (!candidate || !path || path_len == 0 || path_len >= MAX_PATH_LENGTH) {
        return false;
    }
    memset(candidate, 0, sizeof(*candidate));
    for (uint16_t i = 0; i < path_len; i++) {
        uint8_t code = direction_index(path[i]);
        if (code >= 4u) return false;
        uint16_t byte_idx = (uint16_t)(i >> 2);
        uint8_t shift = (uint8_t)((i & 0x03u) << 1);
        candidate->packed_path[byte_idx] = (uint8_t)(
            candidate->packed_path[byte_idx] | (uint8_t)(code << shift)
        );
    }
    candidate->path_len = path_len;
    return true;
}

static bool scan_bomb_main_route_unpack(
    uint8_t index,
    Direction* out_path,
    uint16_t* out_len
) {
    if (!out_path || !out_len ||
        index >= g_scan_bomb_main_route_candidate_count) {
        return false;
    }
    const ScanBombMainRouteCandidate* candidate =
        &g_scan_bomb_main_route_candidates[index];
    if (candidate->path_len == 0 || candidate->path_len >= MAX_PATH_LENGTH) {
        return false;
    }
    for (uint16_t i = 0; i < candidate->path_len; i++) {
        uint16_t byte_idx = (uint16_t)(i >> 2);
        uint8_t shift = (uint8_t)((i & 0x03u) << 1);
        uint8_t code = (uint8_t)(
            (candidate->packed_path[byte_idx] >> shift) & 0x03u
        );
        out_path[i] = direction_from_index(code);
    }
    *out_len = candidate->path_len;
    return true;
}

static void scan_prefix_capture_bomb_main_route_candidates(SokobanSolver* solver) {
    g_scan_bomb_main_route_candidate_count = 0;
    if (!solver || !solver->best_path ||
        solver->best_path_len == 0 || solver->best_path_len >= MAX_PATH_LENGTH) {
        return;
    }

    uint16_t saved_len = solver->best_path_len;
    uint16_t saved_steps = solver->best_steps;
    memcpy(g_scan_verified_path, solver->best_path, saved_len * sizeof(Direction));

    uint8_t source_count = solver_get_scan_bomb_main_route_candidate_count();
    if (source_count > SOKOBAN_SCAN_BOMB_MAIN_ROUTE_CANDIDATE_LIMIT) {
        source_count = SOKOBAN_SCAN_BOMB_MAIN_ROUTE_CANDIDATE_LIMIT;
    }
    for (uint8_t i = 0; i < source_count; i++) {
        uint16_t candidate_len = 0;
        if (!solver_copy_scan_bomb_main_route_candidate(
                i, solver->best_path, &candidate_len)) {
            continue;
        }
        ScanBombMainRouteCandidate* candidate =
            &g_scan_bomb_main_route_candidates[g_scan_bomb_main_route_candidate_count];
        if (scan_bomb_main_route_pack(candidate, solver->best_path, candidate_len)) {
            g_scan_bomb_main_route_candidate_count++;
        }
    }

    memcpy(solver->best_path, g_scan_verified_path, saved_len * sizeof(Direction));
    solver->best_path_len = saved_len;
    solver->best_steps = saved_steps;
}

FAST_OCRAM_FUNC static bool scan_prefix_solve_main_route(SokobanSolver* solver, bool* out_orig_strict) {
    solver->is_scanning = true;
    solver->scan_current_index = 0;
    solver->scan_waypoint_count = 0;

    bool orig_strict = solver->strict_target_mode;
    if (out_orig_strict) *out_orig_strict = orig_strict;
    solver->strict_target_mode = false;
    solver->best_path_len = 0;
    solver->best_steps = 0xFFFF;

    if (g_scan_bomb_main_route_override_index >= 0) {
        uint16_t override_len = 0;
        bool loaded = scan_bomb_main_route_unpack(
            (uint8_t)g_scan_bomb_main_route_override_index,
            solver->best_path,
            &override_len
        );
        if (loaded) {
            solver->best_path_len = override_len;
            solver->best_steps = override_len;
        }
        solver->strict_target_mode = orig_strict;
        return loaded;
    }

    if (!solver_solve_robust(solver)) {
        solver->strict_target_mode = orig_strict;
        return false;
    }
    scan_prefix_capture_bomb_main_route_candidates(solver);

    solver->strict_target_mode = orig_strict;
    return true;
}

FAST_OCRAM_FUNC static bool scan_prefix_find_last_blast_cutoff(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options,
    int* out_cutoff_idx
) {
    if (!out_cutoff_idx) return false;
    *out_cutoff_idx = -1;

    PathReplayState cutoff_state;
    if (!scan_replay_load_from_solver(
            &cutoff_state,
            &solver->bmap,
            solver->start_player,
            solver->boxes,
            solver->num_boxes,
            solver->bombs,
            solver->num_bombs)) {
        return false;
    }

    for (int i = 0; i < solver->best_path_len; i++) {
        PathReplayStepResult step;
        if (!scan_replay_step_existing(solver, &cutoff_state, solver->best_path[i], replay_options, &step)) return false;
        if (step.kind == PATH_REPLAY_STEP_BLASTED_WALL) *out_cutoff_idx = i;
    }
    return true;
}

FAST_OCRAM_FUNC static bool scan_prefix_build_hybrid_path(
    SokobanSolver* solver,
    int cutoff_idx,
    int req_boxes,
    int req_targets,
    const PathReplayOptions* replay_options,
    Direction* hybrid_path,
    uint16_t* out_hybrid_len
) {
    uint16_t hybrid_len = 0;
    PathReplayState hybrid_state;
    if (!scan_replay_load_from_solver(
            &hybrid_state,
            &solver->bmap,
            solver->start_player,
            solver->boxes,
            solver->num_boxes,
            solver->bombs,
            solver->num_bombs)) {
        return false;
    }

    BitboardMap sim_map = hybrid_state.map;
    Position curr_p = hybrid_state.player;
    static Position sim_boxes[MAX_BOXES];
    scan_replay_copy_positions(sim_boxes, &hybrid_state);
    static bool visited_boxes[MAX_BOXES];
    memset(visited_boxes, 0, sizeof(visited_boxes));
    static bool visited_targets[MAX_TARGETS];
    memset(visited_targets, 0, sizeof(visited_targets));

    int scanned_boxes_count = 0;
    int scanned_targets_count = 0;
    uint8_t hybrid_heading = SCAN_INITIAL_HEADING_DIR;

    for (int i = 0; i <= cutoff_idx; i++) {
        bool quota_met = (scanned_boxes_count >= req_boxes && scanned_targets_count >= req_targets);

        if (!quota_met) {
            Position scan_curr;
            bool chained_any = try_scan_entities(
                solver, &sim_map, curr_p, sim_boxes,
                visited_boxes, visited_targets,
                &scanned_boxes_count, &scanned_targets_count,
                req_boxes, req_targets,
                4, 3, i, cutoff_idx,
                hybrid_path, &hybrid_len, &scan_curr,
                &hybrid_heading
            );

            if (chained_any && !pos_equal(scan_curr, curr_p)) {
                static Direction return_path[MAX_SINGLE_PATH];
                uint16_t return_len = 0;
                Position rejoin_p = curr_p;
                int rejoin_next_idx = i;

                if (scan_choose_best_rejoin_after_detour(
                        solver, &sim_map, scan_curr, curr_p, i, cutoff_idx,
                        return_path, &return_len, &rejoin_p, &rejoin_next_idx
                    )) {
                    if (hybrid_len + return_len < MAX_PATH_LENGTH) {
                        memcpy(&hybrid_path[hybrid_len], return_path, return_len * sizeof(Direction));
                        hybrid_len += return_len;
                        hybrid_heading = path_end_direction_index(return_path, return_len, hybrid_heading);
                    }
                    curr_p = rejoin_p;
                    hybrid_state.player = rejoin_p;
                    i = rejoin_next_idx - 1;
                    continue;
                }
            }
        }

        Direction d = solver->best_path[i];

        if (hybrid_len < MAX_PATH_LENGTH) {
            hybrid_path[hybrid_len++] = d;
            uint8_t move_heading = direction_index(d);
            if (move_heading < 4u) hybrid_heading = move_heading;
        }

        if (!scan_replay_step_existing(solver, &hybrid_state, d, replay_options, NULL)) return false;
        sim_map = hybrid_state.map;
        curr_p = hybrid_state.player;
        scan_replay_copy_positions(sim_boxes, &hybrid_state);
    }

    Position pos_before_fallback = curr_p;
    if (scanned_boxes_count < req_boxes || scanned_targets_count < req_targets) {
        Position scan_curr;
        if (try_scan_entities(
                solver, &sim_map, curr_p, sim_boxes,
                visited_boxes, visited_targets,
                &scanned_boxes_count, &scanned_targets_count,
                req_boxes, req_targets,
                99, 999, -1, cutoff_idx,
                hybrid_path, &hybrid_len, &scan_curr,
                &hybrid_heading
            )) {
            static Direction return_path[MAX_SINGLE_PATH];
            uint16_t return_len = 0;
            if (scan_cached_astar_navigate(solver, &sim_map, scan_curr, pos_before_fallback, MASK_WALL | MASK_BOMB | MASK_BOX, return_path, &return_len)) {
                if (hybrid_len + return_len < MAX_PATH_LENGTH) {
                    memcpy(&hybrid_path[hybrid_len], return_path, return_len * sizeof(Direction));
                    hybrid_len += return_len;
                    hybrid_heading = path_end_direction_index(return_path, return_len, hybrid_heading);
                }
            }
            curr_p = pos_before_fallback;
        }
    }

    if (out_hybrid_len) *out_hybrid_len = hybrid_len;
    return true;
}

FAST_OCRAM_FUNC static void scan_prefix_try_spine_replacement(
    SokobanSolver* solver,
    int cutoff_idx,
    int req_boxes,
    int req_targets,
    Direction* hybrid_path,
    uint16_t* hybrid_len
) {
    if (!solver || !hybrid_path || !hybrid_len) return;
    scan_spine_nav_cache_reset();

    uint16_t spine_plan_len = 0;
    int spine_plan_count = 0;
    if (scan_spine_plan(
            solver, cutoff_idx, req_boxes, req_targets, false, false,
            g_scan_eager_path, &spine_plan_len,
            g_scan_eager_waypoints, g_scan_eager_pauses, &spine_plan_count
        ) && spine_plan_count > 0 && spine_plan_len < *hybrid_len) {
        memcpy(hybrid_path, g_scan_eager_path, spine_plan_len * sizeof(Direction));
        *hybrid_len = spine_plan_len;
        solver->scan_waypoint_count = spine_plan_count;
        memcpy(
            solver->scan_waypoints,
            g_scan_eager_waypoints,
            spine_plan_count * sizeof(Entity)
        );
        memcpy(
            solver->scan_player_pause_positions,
            g_scan_eager_pauses,
            spine_plan_count * sizeof(Position)
        );
    }

    spine_plan_len = 0;
    spine_plan_count = 0;
    if (scan_spine_plan(
            solver, cutoff_idx, req_boxes, req_targets, true, false,
            g_scan_eager_path, &spine_plan_len,
            g_scan_eager_waypoints, g_scan_eager_pauses, &spine_plan_count
        ) && spine_plan_count > 0 &&
        spine_plan_count == solver->scan_waypoint_count &&
        spine_plan_len < *hybrid_len &&
        scan_path_move_count(g_scan_eager_path, spine_plan_len) <
            scan_path_move_count(hybrid_path, *hybrid_len) &&
        path_direction_bend_count(g_scan_eager_path, spine_plan_len) <=
            path_direction_bend_count(hybrid_path, *hybrid_len)) {
        memcpy(hybrid_path, g_scan_eager_path, spine_plan_len * sizeof(Direction));
        *hybrid_len = spine_plan_len;
        solver->scan_waypoint_count = spine_plan_count;
        memcpy(
            solver->scan_waypoints,
            g_scan_eager_waypoints,
            spine_plan_count * sizeof(Entity)
        );
        memcpy(
            solver->scan_player_pause_positions,
            g_scan_eager_pauses,
            spine_plan_count * sizeof(Position)
        );
    }
}

FAST_OCRAM_FUNC static bool scan_prefix_build_spine_candidate(
    SokobanSolver* solver,
    int cutoff_idx,
    int req_boxes,
    int req_targets,
    const Direction* hybrid_path,
    uint16_t hybrid_len,
    Direction* spine_plan_path,
    uint16_t* spine_plan_len,
    Entity* spine_plan_waypoints,
    Position* spine_plan_pauses,
    int* spine_plan_count
) {
    if (!solver || !hybrid_path || !spine_plan_path || !spine_plan_len ||
        !spine_plan_waypoints || !spine_plan_pauses || !spine_plan_count) {
        return false;
    }
    *spine_plan_len = 0;
    *spine_plan_count = 0;
    if (!scan_spine_plan(
            solver, cutoff_idx, req_boxes, req_targets, true, true,
            spine_plan_path, spine_plan_len,
            spine_plan_waypoints, spine_plan_pauses, spine_plan_count
        )) {
        return false;
    }
    return *spine_plan_count > 0 &&
           *spine_plan_count == solver->scan_waypoint_count &&
           scan_path_is_better(spine_plan_path, *spine_plan_len, hybrid_path, hybrid_len);
}

static void scan_prefix_write_path_to_solver(
    SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    bool orig_strict
) {
    solver->strict_target_mode = orig_strict;
    memcpy(solver->best_path, path, path_len * sizeof(Direction));
    solver->best_path_len = path_len;
    solver->best_steps = path_len;
}

static void scan_prefix_snapshot_waypoints(
    SokobanSolver* solver,
    Entity* out_waypoints,
    Position* out_pauses,
    int* out_count
) {
    int count = solver->scan_waypoint_count;
    if (out_count) *out_count = count;
    memcpy(out_waypoints, solver->scan_waypoints, count * sizeof(Entity));
    memcpy(out_pauses, solver->scan_player_pause_positions, count * sizeof(Position));
}

static void scan_prefix_rebind_waypoints_after_dewater(
    SokobanSolver* solver,
    const Entity* orig_waypoints,
    const Position* orig_pauses,
    int orig_count
) {
    int new_scan_count = 0;
    Position footprint_p = g_scan_initial_player;
    static bool used_wp[MAX_BOXES + MAX_TARGETS];
    memset(used_wp, 0, sizeof(used_wp));

    for (int i = 0; i < solver->best_path_len; i++) {
        Direction d = solver->best_path[i];
        if (d.dx == 0 && d.dy == 0) {
            for (int k = 0; k < orig_count; k++) {
                if (!used_wp[k] && pos_equal(orig_pauses[k], footprint_p)) {
                    solver->scan_waypoints[new_scan_count] = orig_waypoints[k];
                    solver->scan_player_pause_positions[new_scan_count] = orig_pauses[k];
                    used_wp[k] = true;
                    new_scan_count++;
                    break;
                }
            }
        } else {
            footprint_p.x += d.dx;
            footprint_p.y += d.dy;
        }
    }
    solver->scan_waypoint_count = new_scan_count;
}

FAST_OCRAM_FUNC static void scan_prefix_restore_if_dewater_invalid(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options,
    const Direction* pre_squeeze_path,
    uint16_t pre_squeeze_len,
    const Entity* orig_waypoints,
    const Position* orig_pauses,
    int orig_count
) {
    bool squeeze_illegal_box_goal = false;
    /* A shorter post-optimized route must retain every planned observation. */
    bool squeeze_invalid_scan_pause = solver->scan_waypoint_count != orig_count;
    int check_pause_idx = 0;
    PathReplayState check_state;
    if (!scan_replay_load_from_solver(
            &check_state,
            &g_scan_initial_bmap,
            g_scan_initial_player,
            g_scan_initial_boxes,
            solver->num_boxes,
            g_scan_initial_bombs,
            g_scan_initial_num_bombs)) {
        squeeze_invalid_scan_pause = true;
    }

    for (int i = 0; !squeeze_invalid_scan_pause && i < solver->best_path_len; i++) {
        Direction d = solver->best_path[i];
        if (d.dx == 0 && d.dy == 0) {
            if (check_pause_idx >= solver->scan_waypoint_count) {
                squeeze_invalid_scan_pause = true;
                break;
            }

            Position scan_target = solver->scan_waypoints[check_pause_idx].pos;
            bool wp_is_box = false;
            int wp_idx = -1;
            if (scan_decode_waypoint_entity(solver, solver->scan_waypoints[check_pause_idx].id, &wp_is_box, &wp_idx)) {
                if (wp_is_box) {
                    if (wp_idx < 0 || wp_idx >= solver->num_boxes || check_state.boxes[wp_idx].x == 0xFF) {
                        squeeze_invalid_scan_pause = true;
                        break;
                    }
                    scan_target = check_state.boxes[wp_idx];
                } else {
                    if (wp_idx < 0 || wp_idx >= solver->num_targets) {
                        squeeze_invalid_scan_pause = true;
                        break;
                    }
                    scan_target = solver->targets[wp_idx].pos;
                    if (!is_in_bounds(scan_target.x, scan_target.y) ||
                        !get_bit(check_state.map.targets, scan_target.x, scan_target.y)) {
                        squeeze_invalid_scan_pause = true;
                        break;
                    }
                }
            }

            if (manhattan_distance(check_state.player, scan_target) != 1) {
                squeeze_invalid_scan_pause = true;
                break;
            }

            solver->scan_waypoints[check_pause_idx].pos = scan_target;
            solver->scan_player_pause_positions[check_pause_idx] = check_state.player;
            check_pause_idx++;
            continue;
        }

        PathReplayStepResult step;
        if (!scan_replay_step_existing(solver, &check_state, d, replay_options, &step)) {
            squeeze_invalid_scan_pause = true;
            break;
        }
        if (step.kind == PATH_REPLAY_STEP_PUSHED_BOX && step.box_absorbed) {
            squeeze_illegal_box_goal = true;
            break;
        }
    }
    if (!squeeze_invalid_scan_pause && check_pause_idx != solver->scan_waypoint_count) {
        squeeze_invalid_scan_pause = true;
    }
    if (squeeze_illegal_box_goal || squeeze_invalid_scan_pause) {
        memcpy(solver->best_path, pre_squeeze_path, pre_squeeze_len * sizeof(Direction));
        solver->best_path_len = pre_squeeze_len;
        solver->best_steps = pre_squeeze_len;
        solver->scan_waypoint_count = orig_count;
        memcpy(solver->scan_waypoints, orig_waypoints, orig_count * sizeof(Entity));
        memcpy(solver->scan_player_pause_positions, orig_pauses, orig_count * sizeof(Position));
    }
}

static bool scan_prefix_waypoint_position_at_state(
    const SokobanSolver* solver,
    const Entity* waypoint,
    const PathReplayState* state,
    Position* out_pos
) {
    if (!solver || !waypoint || !state || !out_pos) return false;

    Position pos = waypoint->pos;
    bool is_box = false;
    int entity_idx = -1;
    if (scan_decode_waypoint_entity(solver, waypoint->id, &is_box, &entity_idx)) {
        if (is_box) {
            if (entity_idx < 0 || entity_idx >= solver->num_boxes || state->boxes[entity_idx].x == 0xFF) return false;
            pos = state->boxes[entity_idx];
        } else {
            if (entity_idx < 0 || entity_idx >= solver->num_targets) return false;
            pos = solver->targets[entity_idx].pos;
            if (!is_in_bounds(pos.x, pos.y) || !get_bit(state->map.targets, pos.x, pos.y)) return false;
        }
    }

    *out_pos = pos;
    return true;
}

static bool scan_prefix_waypoint_is_observable_at_state(
    const SokobanSolver* solver,
    const Entity* waypoint,
    const PathReplayState* state,
    Position* out_pos
) {
    if (!out_pos) return false;

    Position pos;
    if (!scan_prefix_waypoint_position_at_state(solver, waypoint, state, &pos)) return false;
    if (!is_in_bounds(pos.x, pos.y)) return false;
    if (get_bit(state->map.targets, pos.x, pos.y) &&
        get_bit(state->map.bombs, pos.x, pos.y)) {
        return false;
    }

    *out_pos = pos;
    return true;
}

static bool scan_prefix_validate_bound_path_impl(
    const SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    const Entity* waypoints,
    const Position* pause_positions,
    int waypoint_count,
    const PathReplayOptions* replay_options,
    bool require_observable_waypoints,
    PathReplayState* out_final_state
) {
    if (!solver || !path || !waypoints || !pause_positions || !replay_options || waypoint_count < 0 ||
        waypoint_count > MAX_BOXES + MAX_TARGETS || path_len >= MAX_PATH_LENGTH) {
        return false;
    }

    PathReplayState state;
    if (!scan_replay_load_from_solver(
            &state,
            &g_scan_initial_bmap, g_scan_initial_player,
            g_scan_initial_boxes, solver->num_boxes,
            g_scan_initial_bombs, g_scan_initial_num_bombs)) {
        return false;
    }

    int pause_idx = 0;
    for (uint16_t i = 0; i < path_len; i++) {
        Direction d = path[i];
        if (d.dx == 0 && d.dy == 0) {
            Position target_pos;
            if (pause_idx >= waypoint_count ||
                !pos_equal(pause_positions[pause_idx], state.player) ||
                !(require_observable_waypoints
                    ? scan_prefix_waypoint_is_observable_at_state(
                        solver, &waypoints[pause_idx], &state, &target_pos)
                    : scan_prefix_waypoint_position_at_state(
                        solver, &waypoints[pause_idx], &state, &target_pos)) ||
                manhattan_distance(state.player, target_pos) != 1) {
                return false;
            }
            pause_idx++;
            continue;
        }

        if (!scan_replay_step_existing(solver, &state, d, replay_options, NULL)) return false;
    }

    if (pause_idx != waypoint_count) return false;
    if (out_final_state) *out_final_state = state;
    return true;
}

static bool scan_prefix_validate_bound_path(
    const SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    const Entity* waypoints,
    const Position* pause_positions,
    int waypoint_count,
    const PathReplayOptions* replay_options,
    PathReplayState* out_final_state
) {
    return scan_prefix_validate_bound_path_impl(
        solver, path, path_len, waypoints, pause_positions, waypoint_count,
        replay_options, true, out_final_state
    );
}

/* A quota entity must be paused at its first legal adjacent observation. */
static bool scan_prefix_validate_first_arrival_observations(
    const SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    const Entity* waypoints,
    const Position* pause_positions,
    int waypoint_count,
    const PathReplayOptions* replay_options
) {
    if (!solver || !path || !waypoints || !pause_positions || !replay_options ||
        waypoint_count <= 0 || waypoint_count > MAX_BOXES + MAX_TARGETS ||
        path_len >= MAX_PATH_LENGTH) {
        return false;
    }

    bool tracked[MAX_BOXES + MAX_TARGETS] = {false};
    bool paused[MAX_BOXES + MAX_TARGETS] = {false};
    for (int k = 0; k < waypoint_count; k++) {
        bool is_box = false;
        int entity_idx = -1;
        if (scan_decode_waypoint_entity(solver, waypoints[k].id, &is_box, &entity_idx) &&
            ((is_box && entity_idx >= 0 && entity_idx < solver->num_boxes) ||
             (!is_box && entity_idx >= 0 && entity_idx < solver->num_targets))) {
            tracked[k] = true;
        }
    }

    PathReplayState state;
    if (!scan_replay_load_from_solver(
            &state,
            &g_scan_initial_bmap, g_scan_initial_player,
            g_scan_initial_boxes, solver->num_boxes,
            g_scan_initial_bombs, g_scan_initial_num_bombs)) {
        return false;
    }

    int pause_idx = 0;
    for (uint16_t i = 0; i <= path_len; i++) {
        bool first_visible = false;
        for (int k = 0; k < waypoint_count; k++) {
            if (!tracked[k] || paused[k]) continue;
            Position entity_pos;
            if (scan_prefix_waypoint_is_observable_at_state(
                    solver, &waypoints[k], &state, &entity_pos) &&
                manhattan_distance(state.player, entity_pos) == 1) {
                first_visible = true;
                break;
            }
        }

        if (i == path_len) {
            return !first_visible && pause_idx == waypoint_count;
        }

        Direction d = path[i];
        if (direction_is_pause(d)) {
            if (pause_idx >= waypoint_count) return false;
            if (!pos_equal(pause_positions[pause_idx], state.player)) return false;
            Position target_pos;
            if (!scan_prefix_waypoint_is_observable_at_state(
                    solver, &waypoints[pause_idx], &state, &target_pos) ||
                manhattan_distance(state.player, target_pos) != 1) {
                return false;
            }
            paused[pause_idx++] = true;
            continue;
        }

        if (first_visible || !scan_replay_step_existing(solver, &state, d, replay_options, NULL)) {
            return false;
        }
    }
    return false;
}

static bool scan_prefix_states_equal(const PathReplayState* a, const PathReplayState* b) {
    return a && b &&
           pos_equal(a->player, b->player) &&
           a->bomb_count == b->bomb_count &&
           memcmp(&a->map, &b->map, sizeof(a->map)) == 0 &&
           memcmp(a->boxes, b->boxes, sizeof(a->boxes)) == 0 &&
           memcmp(a->bombs, b->bombs, sizeof(a->bombs)) == 0;
}

/* Free-terminal candidates may end at different player cells. */
static bool scan_prefix_physical_states_equal(const PathReplayState* a, const PathReplayState* b) {
    return a && b &&
           a->bomb_count == b->bomb_count &&
           memcmp(&a->map, &b->map, sizeof(a->map)) == 0 &&
           memcmp(a->boxes, b->boxes, sizeof(a->boxes)) == 0 &&
           memcmp(a->bombs, b->bombs, sizeof(a->bombs)) == 0;
}

/* Keep only cancellations that preserve both every scan pause and the replayed state. */
static void scan_prefix_remove_state_neutral_inverse_pairs(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options
) {
    if (!solver || !replay_options || solver->best_path_len < 2 ||
        solver->scan_waypoint_count <= 0 || solver->scan_waypoint_count > MAX_BOXES + MAX_TARGETS) {
        return;
    }

    PathReplayState baseline_final;
    if (!scan_prefix_validate_bound_path(
            solver, solver->best_path, solver->best_path_len,
            solver->scan_waypoints, solver->scan_player_pause_positions, solver->scan_waypoint_count,
            replay_options, &baseline_final)) {
        return;
    }

    bool removed = true;
    while (removed) {
        removed = false;
        for (uint16_t i = 0; i + 1 < solver->best_path_len; i++) {
            Direction first = solver->best_path[i];
            Direction second = solver->best_path[i + 1];
            if ((first.dx == 0 && first.dy == 0) || (second.dx == 0 && second.dy == 0) ||
                first.dx + second.dx != 0 || first.dy + second.dy != 0) {
                continue;
            }

            PathReplayState pair_state;
            if (!scan_replay_load_from_solver(
                    &pair_state,
                    &g_scan_initial_bmap, g_scan_initial_player,
                    g_scan_initial_boxes, solver->num_boxes,
                    g_scan_initial_bombs, g_scan_initial_num_bombs)) {
                return;
            }
            bool pair_prefix_ok = true;
            for (uint16_t j = 0; j < i; j++) {
                if (!scan_replay_step_existing(solver, &pair_state, solver->best_path[j], replay_options, NULL)) {
                    pair_prefix_ok = false;
                    break;
                }
            }
            PathReplayStepResult first_step;
            PathReplayStepResult second_step;
            if (!pair_prefix_ok ||
                !scan_replay_step_existing(solver, &pair_state, first, replay_options, &first_step) ||
                first_step.kind != PATH_REPLAY_STEP_MOVED ||
                !scan_replay_step_existing(solver, &pair_state, second, replay_options, &second_step) ||
                second_step.kind != PATH_REPLAY_STEP_MOVED) {
                continue;
            }

            uint16_t candidate_len = (uint16_t)(solver->best_path_len - 2u);
            if (candidate_len == 0 || candidate_len >= MAX_PATH_LENGTH) continue;
            if (i > 0) memcpy(g_scan_eager_path, solver->best_path, i * sizeof(Direction));
            if (i + 2 < solver->best_path_len) {
                memcpy(&g_scan_eager_path[i], &solver->best_path[i + 2],
                       (solver->best_path_len - i - 2u) * sizeof(Direction));
            }

            PathReplayState candidate_final;
            if (!scan_prefix_validate_bound_path(
                    solver, g_scan_eager_path, candidate_len,
                    solver->scan_waypoints, solver->scan_player_pause_positions, solver->scan_waypoint_count,
                    replay_options, &candidate_final) ||
                !scan_prefix_states_equal(&baseline_final, &candidate_final)) {
                continue;
            }

            memcpy(solver->best_path, g_scan_eager_path, candidate_len * sizeof(Direction));
            solver->best_path_len = candidate_len;
            solver->best_steps = candidate_len;
            baseline_final = candidate_final;
            removed = true;
            break;
        }
    }
}

/*
 * Eager observation rebinding can move a pause to an earlier adjacent cell.
 * A later post-path bridge can then become a dominated walking-only segment:
 * it still reaches the same next pause, but is longer even with its incoming
 * and final arrival directions held fixed.  Keep stateful actions out of this
 * pass and only accept a replacement after the normal bound-path/state checks.
 */
static bool scan_prefix_shorten_dominated_navigation_segments(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options
) {
    if (!solver || !replay_options || solver->best_path_len < 2 ||
        solver->scan_waypoint_count <= 0 || solver->scan_waypoint_count > MAX_BOXES + MAX_TARGETS) {
        return false;
    }

    PathReplayState baseline_final;
    if (!scan_prefix_validate_bound_path(
            solver, solver->best_path, solver->best_path_len,
            solver->scan_waypoints, solver->scan_player_pause_positions, solver->scan_waypoint_count,
            replay_options, &baseline_final)) {
        return false;
    }

    bool changed = false;
    bool shortened = true;
    while (shortened) {
        shortened = false;

        PathReplayState state;
        if (!scan_replay_load_from_solver(
                &state,
                &g_scan_initial_bmap, g_scan_initial_player,
                g_scan_initial_boxes, solver->num_boxes,
                g_scan_initial_bombs, g_scan_initial_num_bombs)) {
            return changed;
        }

        uint16_t segment_start = 0;
        Position segment_start_player = state.player;
        bool segment_is_walk_only = true;

        for (uint16_t i = 0; i <= solver->best_path_len; i++) {
            bool segment_end = (i == solver->best_path_len) ||
                               (solver->best_path[i].dx == 0 && solver->best_path[i].dy == 0);
            if (segment_end) {
                uint16_t segment_len = (uint16_t)(i - segment_start);
                if (segment_is_walk_only && segment_len > 1) {
                    Direction final_dir = solver->best_path[i - 1];
                    uint8_t final_dir_idx = direction_index(final_dir);
                    uint8_t segment_heading = path_end_direction_index(
                        solver->best_path, segment_start, DIRECTION_INDEX_NONE
                    );
                    uint16_t original_bends = scan_route_turn_count(
                        &solver->best_path[segment_start], segment_len, segment_heading
                    );
                    uint16_t replacement_len = 0;
                    bool have_replacement = false;
                    int predecessor_x = (int)state.player.x - final_dir.dx;
                    int predecessor_y = (int)state.player.y - final_dir.dy;

                    if (final_dir_idx < 4u && is_in_bounds(predecessor_x, predecessor_y)) {
                        /* Preserve the existing shortest-route selection first. */
                        ScanNavScratch* nav = &g_scan_bfs_scratch.work.nav;
                        Position predecessor = {(uint8_t)predecessor_x, (uint8_t)predecessor_y};
                        build_scan_nav_field(&state.map, segment_start_player,
                                             nav->dist, nav->parent_dir, nav->queue);
                        uint16_t predecessor_len = nav->dist[predecessor.y][predecessor.x];

                        if (predecessor_len != 0xFFFF && predecessor_len + 1u < segment_len) {
                            uint16_t navigation_len = 0;
                            if (predecessor_len + 1u < MAX_SINGLE_PATH &&
                                reconstruct_scan_nav_turn_path(
                                    nav->dist, predecessor, segment_heading, (uint8_t)final_dir_idx,
                                    g_scan_compact_path, &navigation_len
                                ) && navigation_len == predecessor_len) {
                                g_scan_compact_path[navigation_len] = final_dir;
                                replacement_len = (uint16_t)(navigation_len + 1u);
                                uint16_t replacement_bends = scan_route_turn_count(
                                    g_scan_compact_path, replacement_len, segment_heading
                                );
                                have_replacement = replacement_bends <= original_bends;
                            }

                            /*
                             * The globally shortest route can be too bendy, or exceed
                             * the legacy single-navigation buffer.  Search all shorter
                             * output lengths under the same bend budget before giving up.
                             */
                            if (!have_replacement) {
                                have_replacement = scan_find_turn_bounded_shorter_navigation(
                                    &state.map, segment_start_player, state.player,
                                    segment_heading, (uint8_t)final_dir_idx,
                                    segment_len, original_bends,
                                    g_scan_compact_path, &replacement_len
                                );
                            }
                        }
                    }

                    if (have_replacement) {
                        uint16_t candidate_len = (uint16_t)(
                            solver->best_path_len - segment_len + replacement_len
                        );
                        if (candidate_len > 0 && candidate_len < MAX_PATH_LENGTH) {
                            if (segment_start > 0) {
                                memcpy(g_scan_eager_path, solver->best_path,
                                       segment_start * sizeof(Direction));
                            }
                            memcpy(&g_scan_eager_path[segment_start], g_scan_compact_path,
                                   replacement_len * sizeof(Direction));
                            if (i < solver->best_path_len) {
                                memcpy(&g_scan_eager_path[segment_start + replacement_len],
                                       &solver->best_path[i],
                                       (solver->best_path_len - i) * sizeof(Direction));
                            }

                            PathReplayState candidate_final;
                            if (scan_prefix_validate_bound_path(
                                    solver, g_scan_eager_path, candidate_len,
                                    solver->scan_waypoints, solver->scan_player_pause_positions,
                                    solver->scan_waypoint_count, replay_options, &candidate_final) &&
                                scan_prefix_states_equal(&baseline_final, &candidate_final)) {
                                memcpy(solver->best_path, g_scan_eager_path,
                                       candidate_len * sizeof(Direction));
                                solver->best_path_len = candidate_len;
                                solver->best_steps = candidate_len;
                                baseline_final = candidate_final;
                                changed = true;
                                shortened = true;
                                break;
                            }
                        }
                    }
                }

                if (i == solver->best_path_len) break;
                segment_start = (uint16_t)(i + 1u);
                segment_start_player = state.player;
                segment_is_walk_only = true;
                continue;
            }

            PathReplayStepResult step;
            if (!scan_replay_step_existing(solver, &state, solver->best_path[i], replay_options, &step)) {
                return changed;
            }
            if (step.kind != PATH_REPLAY_STEP_MOVED) segment_is_walk_only = false;
        }
    }

    return changed;
}

static bool scan_prefix_append_eager_observations(
    const SokobanSolver* solver,
    const PathReplayState* state,
    const Entity* desired_waypoints,
    int waypoint_count,
    uint16_t* io_path_len,
    int* io_waypoint_count
) {
    if (!solver || !state || !desired_waypoints || !io_path_len || !io_waypoint_count) return false;

    while (true) {
        int eager_idx = -1;
        Position eager_pos;
        for (int k = 0; k < waypoint_count; k++) {
            if (g_scan_eager_emitted[k]) continue;
            if (!scan_prefix_waypoint_is_observable_at_state(solver, &desired_waypoints[k], state, &eager_pos)) continue;
            if (manhattan_distance(state->player, eager_pos) == 1) {
                eager_idx = k;
                break;
            }
        }
        if (eager_idx < 0) return true;
        if (*io_path_len >= MAX_PATH_LENGTH || *io_waypoint_count >= MAX_BOXES + MAX_TARGETS) return false;

        g_scan_eager_path[(*io_path_len)++] = (Direction){0, 0};
        g_scan_eager_waypoints[*io_waypoint_count] = desired_waypoints[eager_idx];
        g_scan_eager_waypoints[*io_waypoint_count].pos = eager_pos;
        g_scan_eager_pauses[*io_waypoint_count] = state->player;
        g_scan_eager_emitted[eager_idx] = true;
        (*io_waypoint_count)++;
    }
}

/*
 * A stateful tail can uncover a target and finish with the player standing on
 * that target.  The target is then observable, but not from the current cell.
 * Add the smallest state-neutral out-and-back walk needed to observe it.  The
 * deterministic direction order and strict replay checks keep this generic:
 * no map identity, coordinates, tags, or saved path fragments are consulted.
 */
static bool scan_prefix_append_terminal_observation_detours(
    const SokobanSolver* solver,
    PathReplayState* state,
    const Entity* desired_waypoints,
    int waypoint_count,
    const PathReplayOptions* replay_options,
    uint16_t* io_path_len,
    int* io_waypoint_count
) {
    if (!solver || !state || !desired_waypoints || !replay_options ||
        !io_path_len || !io_waypoint_count) {
        return false;
    }

    while (*io_waypoint_count < waypoint_count) {
        int overlapped_idx = -1;
        Position overlapped_pos;
        for (int k = 0; k < waypoint_count; k++) {
            if (g_scan_eager_emitted[k]) continue;
            if (!scan_prefix_waypoint_is_observable_at_state(
                    solver, &desired_waypoints[k], state, &overlapped_pos)) {
                continue;
            }
            if (pos_equal(state->player, overlapped_pos)) {
                overlapped_idx = k;
                break;
            }
        }
        if (overlapped_idx < 0 || (uint32_t)*io_path_len + 3u >= MAX_PATH_LENGTH) {
            return false;
        }

        bool found_roundtrip = false;
        Direction outward = {0, 0};
        PathReplayState roundtrip_state;
        for (int d = 0; d < 4; d++) {
            Direction candidate = DIRECTIONS[d];
            Direction reverse = {(int8_t)-candidate.dx, (int8_t)-candidate.dy};
            Position waypoint_pos;
            PathReplayStepResult step;

            roundtrip_state = *state;
            if (!scan_replay_step_existing(
                    solver, &roundtrip_state, candidate, replay_options, &step) ||
                step.kind != PATH_REPLAY_STEP_MOVED ||
                !scan_prefix_waypoint_is_observable_at_state(
                    solver, &desired_waypoints[overlapped_idx], &roundtrip_state, &waypoint_pos) ||
                manhattan_distance(roundtrip_state.player, waypoint_pos) != 1 ||
                !scan_replay_step_existing(
                    solver, &roundtrip_state, reverse, replay_options, &step) ||
                step.kind != PATH_REPLAY_STEP_MOVED ||
                !scan_prefix_states_equal(&roundtrip_state, state)) {
                continue;
            }

            outward = candidate;
            found_roundtrip = true;
            break;
        }
        if (!found_roundtrip) return false;

        Direction inward = {(int8_t)-outward.dx, (int8_t)-outward.dy};
        PathReplayStepResult step;
        g_scan_eager_path[(*io_path_len)++] = outward;
        if (!scan_replay_step_existing(solver, state, outward, replay_options, &step) ||
            step.kind != PATH_REPLAY_STEP_MOVED ||
            !scan_prefix_append_eager_observations(
                solver, state, desired_waypoints, waypoint_count,
                io_path_len, io_waypoint_count) ||
            !g_scan_eager_emitted[overlapped_idx] ||
            (uint32_t)*io_path_len + 1u >= MAX_PATH_LENGTH) {
            return false;
        }

        g_scan_eager_path[(*io_path_len)++] = inward;
        if (!scan_replay_step_existing(solver, state, inward, replay_options, &step) ||
            step.kind != PATH_REPLAY_STEP_MOVED ||
            !scan_prefix_states_equal(state, &roundtrip_state)) {
            return false;
        }
    }

    return true;
}

/* Rebind semantic scan tags at the first valid view along the replayed route. */
static bool scan_prefix_rebind_eager_observations_for_waypoints(
    SokobanSolver* solver,
    const Entity* source_waypoints,
    const Position* source_pauses,
    const Entity* desired_waypoints,
    int waypoint_count,
    const PathReplayOptions* replay_options
) {
    if (!solver || !source_waypoints || !source_pauses || !desired_waypoints || !replay_options ||
        waypoint_count <= 0 || waypoint_count > MAX_BOXES + MAX_TARGETS ||
        solver->best_path_len == 0 || solver->best_path_len >= MAX_PATH_LENGTH) {
        return false;
    }

    PathReplayState original_final;
    if (!scan_prefix_validate_bound_path_impl(
            solver, solver->best_path, solver->best_path_len,
            source_waypoints, source_pauses, waypoint_count,
            replay_options, false, &original_final)) {
        return false;
    }

    PathReplayState state;
    if (!scan_replay_load_from_solver(
            &state,
            &g_scan_initial_bmap, g_scan_initial_player,
            g_scan_initial_boxes, solver->num_boxes,
            g_scan_initial_bombs, g_scan_initial_num_bombs)) {
        return false;
    }

    memset(g_scan_eager_emitted, 0, sizeof(g_scan_eager_emitted));
    uint16_t out_len = 0;
    int out_waypoint_count = 0;
    int source_pause_idx = 0;

    for (uint16_t i = 0; i < solver->best_path_len; i++) {
        if (!scan_prefix_append_eager_observations(
                solver, &state, desired_waypoints, waypoint_count, &out_len, &out_waypoint_count)) {
            return false;
        }

        Direction d = solver->best_path[i];
        if (d.dx == 0 && d.dy == 0) {
            if (source_pause_idx >= waypoint_count) return false;
            source_pause_idx++;
            continue;
        }

        if (out_len >= MAX_PATH_LENGTH ||
            !scan_replay_step_existing(solver, &state, d, replay_options, NULL)) {
            return false;
        }
        g_scan_eager_path[out_len++] = d;
    }

    if (!scan_prefix_append_eager_observations(
            solver, &state, desired_waypoints, waypoint_count, &out_len, &out_waypoint_count)) {
        return false;
    }
    if (source_pause_idx != waypoint_count ||
        !scan_prefix_append_terminal_observation_detours(
            solver, &state, desired_waypoints, waypoint_count, replay_options,
            &out_len, &out_waypoint_count) ||
        out_waypoint_count != waypoint_count) {
        return false;
    }
    PathReplayState rebound_final;
    if (!scan_prefix_validate_bound_path(
            solver, g_scan_eager_path, out_len,
            g_scan_eager_waypoints, g_scan_eager_pauses, out_waypoint_count,
            replay_options, &rebound_final) ||
        !scan_prefix_states_equal(&original_final, &rebound_final)) {
        return false;
    }

    memcpy(solver->best_path, g_scan_eager_path, out_len * sizeof(Direction));
    solver->best_path_len = out_len;
    solver->best_steps = out_len;
    memcpy(solver->scan_waypoints, g_scan_eager_waypoints, out_waypoint_count * sizeof(Entity));
    memcpy(solver->scan_player_pause_positions, g_scan_eager_pauses, out_waypoint_count * sizeof(Position));
    return true;
}

/* A scan tag is semantic state, so emit it at the first valid view along the final route. */
static void scan_prefix_rebind_eager_observations(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options
) {
    if (!solver) return;
    (void)scan_prefix_rebind_eager_observations_for_waypoints(
        solver,
        solver->scan_waypoints,
        solver->scan_player_pause_positions,
        solver->scan_waypoints,
        solver->scan_waypoint_count,
        replay_options
    );
}

static bool scan_prefix_collect_first_visible_move_ranks(
    const SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    const PathReplayOptions* replay_options,
    uint16_t* box_ranks,
    uint16_t* target_ranks
) {
    if (!solver || !path || !replay_options || !box_ranks || !target_ranks) return false;

    for (int i = 0; i < MAX_BOXES; i++) box_ranks[i] = UINT16_MAX;
    for (int i = 0; i < MAX_TARGETS; i++) target_ranks[i] = UINT16_MAX;

    PathReplayState state;
    if (!scan_replay_load_from_solver(
            &state,
            &g_scan_initial_bmap, g_scan_initial_player,
            g_scan_initial_boxes, solver->num_boxes,
            g_scan_initial_bombs, g_scan_initial_num_bombs)) {
        return false;
    }

    uint16_t move_rank = 0;
    for (uint16_t path_idx = 0; path_idx <= path_len; path_idx++) {
        for (int i = 0; i < solver->num_boxes; i++) {
            if (box_ranks[i] == UINT16_MAX && state.boxes[i].x != 0xFF &&
                manhattan_distance(state.player, state.boxes[i]) == 1) {
                box_ranks[i] = move_rank;
            }
        }
        for (int i = 0; i < solver->num_targets; i++) {
            Position target_pos = solver->targets[i].pos;
            if (target_ranks[i] == UINT16_MAX && is_in_bounds(target_pos.x, target_pos.y) &&
                get_bit(state.map.targets, target_pos.x, target_pos.y) &&
                manhattan_distance(state.player, target_pos) == 1) {
                target_ranks[i] = move_rank;
            }
        }

        if (path_idx == path_len) break;
        Direction d = path[path_idx];
        if (d.dx == 0 && d.dy == 0) continue;
        if (!scan_replay_step_existing((SokobanSolver*)solver, &state, d, replay_options, NULL)) return false;
        if (move_rank != UINT16_MAX) move_rank++;
    }
    return true;
}

static bool scan_prefix_select_earliest_visible_entities(
    int entity_count,
    int entity_capacity,
    int selected_count,
    const uint16_t* ranks,
    const bool* original_selected,
    const int* source_ordinals,
    bool* out_selected
) {
    if (!ranks || !original_selected || !source_ordinals || !out_selected ||
        entity_capacity < 0 || entity_count < 0 || entity_count > entity_capacity ||
        selected_count < 0 || selected_count > entity_count) {
        return false;
    }

    int eligible_count = 0;
    for (int i = 0; i < entity_count; i++) {
        out_selected[i] = false;
        if (ranks[i] != UINT16_MAX) eligible_count++;
    }
    if (eligible_count < selected_count) return false;

    for (int picked = 0; picked < selected_count; picked++) {
        int best_idx = -1;
        for (int i = 0; i < entity_count; i++) {
            if (out_selected[i] || ranks[i] == UINT16_MAX) continue;
            if (best_idx < 0 || ranks[i] < ranks[best_idx] ||
                (ranks[i] == ranks[best_idx] && original_selected[i] && !original_selected[best_idx]) ||
                (ranks[i] == ranks[best_idx] && original_selected[i] == original_selected[best_idx] &&
                 source_ordinals[i] < source_ordinals[best_idx])) {
                best_idx = i;
            }
        }
        if (best_idx < 0) return false;
        out_selected[best_idx] = true;
    }
    return true;
}

static bool scan_prefix_build_earliest_visible_waypoints(
    const SokobanSolver* solver,
    const PathReplayOptions* replay_options,
    Entity* out_waypoints,
    int waypoint_count,
    bool* out_changed
) {
    if (!solver || !replay_options || !out_waypoints || !out_changed ||
        waypoint_count <= 0 || waypoint_count > MAX_BOXES + MAX_TARGETS) {
        return false;
    }

    bool original_boxes[MAX_BOXES] = {false};
    bool original_targets[MAX_TARGETS] = {false};
    bool desired_boxes[MAX_BOXES] = {false};
    bool desired_targets[MAX_TARGETS] = {false};
    int box_ordinals[MAX_BOXES];
    int target_ordinals[MAX_TARGETS];
    uint16_t box_ranks[MAX_BOXES];
    uint16_t target_ranks[MAX_TARGETS];
    int selected_box_count = 0;
    int selected_target_count = 0;

    for (int i = 0; i < MAX_BOXES; i++) box_ordinals[i] = MAX_BOXES + MAX_TARGETS + i;
    for (int i = 0; i < MAX_TARGETS; i++) target_ordinals[i] = MAX_BOXES + MAX_TARGETS + i;
    memcpy(out_waypoints, solver->scan_waypoints, waypoint_count * sizeof(Entity));

    for (int k = 0; k < waypoint_count; k++) {
        bool is_box = false;
        int entity_idx = -1;
        if (!scan_decode_waypoint_entity(solver, solver->scan_waypoints[k].id, &is_box, &entity_idx)) continue;
        if (is_box) {
            if (entity_idx < 0 || entity_idx >= solver->num_boxes || original_boxes[entity_idx]) return false;
            original_boxes[entity_idx] = true;
            box_ordinals[entity_idx] = k;
            selected_box_count++;
        } else {
            if (entity_idx < 0 || entity_idx >= solver->num_targets || original_targets[entity_idx]) return false;
            original_targets[entity_idx] = true;
            target_ordinals[entity_idx] = k;
            selected_target_count++;
        }
    }

    if (!scan_prefix_collect_first_visible_move_ranks(
            solver, solver->best_path, solver->best_path_len, replay_options, box_ranks, target_ranks) ||
        !scan_prefix_select_earliest_visible_entities(
            solver->num_boxes, MAX_BOXES, selected_box_count,
            box_ranks, original_boxes, box_ordinals, desired_boxes) ||
        !scan_prefix_select_earliest_visible_entities(
            solver->num_targets, MAX_TARGETS, selected_target_count,
            target_ranks, original_targets, target_ordinals, desired_targets)) {
        return false;
    }

    int next_new_box = 0;
    int next_new_target = 0;
    *out_changed = false;
    for (int k = 0; k < waypoint_count; k++) {
        bool is_box = false;
        int entity_idx = -1;
        if (!scan_decode_waypoint_entity(solver, solver->scan_waypoints[k].id, &is_box, &entity_idx)) continue;

        if (is_box && original_boxes[entity_idx] && !desired_boxes[entity_idx]) {
            while (next_new_box < solver->num_boxes &&
                   (!desired_boxes[next_new_box] || original_boxes[next_new_box])) {
                next_new_box++;
            }
            if (next_new_box >= solver->num_boxes) return false;
            out_waypoints[k].id = scan_waypoint_box_tag(next_new_box);
            out_waypoints[k].pos = g_scan_initial_boxes[next_new_box].pos;
            out_waypoints[k].is_active = true;
            next_new_box++;
            *out_changed = true;
        } else if (!is_box && original_targets[entity_idx] && !desired_targets[entity_idx]) {
            while (next_new_target < solver->num_targets &&
                   (!desired_targets[next_new_target] || original_targets[next_new_target])) {
                next_new_target++;
            }
            if (next_new_target >= solver->num_targets) return false;
            out_waypoints[k].id = scan_waypoint_target_tag(next_new_target);
            out_waypoints[k].pos = solver->targets[next_new_target].pos;
            out_waypoints[k].is_active = true;
            next_new_target++;
            *out_changed = true;
        }
    }
    return true;
}

static void scan_prefix_rebind_earliest_visible_observations(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options
) {
    if (!solver || !replay_options || solver->scan_waypoint_count <= 0 ||
        solver->scan_waypoint_count > MAX_BOXES + MAX_TARGETS) {
        return;
    }

    bool changed = false;
    if (!scan_prefix_build_earliest_visible_waypoints(
            solver, replay_options, g_scan_eager_desired_waypoints,
            solver->scan_waypoint_count, &changed) || !changed) {
        return;
    }

    (void)scan_prefix_rebind_eager_observations_for_waypoints(
        solver,
        solver->scan_waypoints,
        solver->scan_player_pause_positions,
        g_scan_eager_desired_waypoints,
        solver->scan_waypoint_count,
        replay_options
    );
}

FAST_OCRAM_FUNC static bool scan_prefix_write_final_state(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options
) {
    PathReplayState final_state;
    if (!scan_prefix_validate_bound_path(
            solver, solver->best_path, solver->best_path_len,
            solver->scan_waypoints, solver->scan_player_pause_positions,
            solver->scan_waypoint_count, replay_options, &final_state)) {
        return false;
    }
    scan_replay_write_solver_state(solver, &final_state);
    scan_refresh_bomb_delay_events(solver);
    solver_refresh_deadlocks(solver);
    return true;
}

/*
 * Eager observation rebinding can move the final pause ahead of navigation
 * that originally led to that observation.  A free-terminal candidate no
 * longer needs such a suffix once every remaining step is ordinary movement
 * and dropping it preserves the complete physical map state.  Keep the
 * stricter endpoint-preserving behavior for all non-free-terminal routes.
 */
FAST_OCRAM_FUNC static bool scan_prefix_trim_free_terminal_navigation_suffix(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options
) {
    if (!solver || !replay_options || solver->best_path_len < 2 ||
        solver->scan_waypoint_count <= 0 ||
        solver->scan_waypoint_count > MAX_BOXES + MAX_TARGETS) {
        return false;
    }

    int last_pause = -1;
    for (int i = 0; i < solver->best_path_len; i++) {
        if (direction_is_pause(solver->best_path[i])) last_pause = i;
    }
    if (last_pause < 0 || last_pause + 1 >= solver->best_path_len) return false;

    uint16_t truncated_len = (uint16_t)(last_pause + 1);
    PathReplayState full_final;
    PathReplayState suffix_state;
    if (!scan_prefix_validate_bound_path(
            solver, solver->best_path, solver->best_path_len,
            solver->scan_waypoints, solver->scan_player_pause_positions,
            solver->scan_waypoint_count, replay_options, &full_final) ||
        !scan_prefix_validate_bound_path(
            solver, solver->best_path, truncated_len,
            solver->scan_waypoints, solver->scan_player_pause_positions,
            solver->scan_waypoint_count, replay_options, &suffix_state) ||
        !scan_prefix_physical_states_equal(&full_final, &suffix_state)) {
        return false;
    }

    for (uint16_t i = truncated_len; i < solver->best_path_len; i++) {
        Direction d = solver->best_path[i];
        PathReplayStepResult step;
        if (direction_index(d) >= 4u ||
            !scan_replay_step_existing(solver, &suffix_state, d, replay_options, &step) ||
            step.kind != PATH_REPLAY_STEP_MOVED) {
            return false;
        }
    }
    if (!scan_prefix_states_equal(&full_final, &suffix_state)) return false;

    solver->best_path_len = truncated_len;
    solver->best_steps = truncated_len;
    return true;
}

static bool scan_prefix_finalize_semantic_route(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options,
    bool free_terminal
) {
    if (!solver || !replay_options) return false;
    for (int pass = 0; pass < MAX_BOXES + MAX_TARGETS; pass++) {
        scan_prefix_rebind_eager_observations(solver, replay_options);
        scan_prefix_remove_state_neutral_inverse_pairs(solver, replay_options);
        scan_prefix_rebind_earliest_visible_observations(solver, replay_options);
        scan_prefix_remove_state_neutral_inverse_pairs(solver, replay_options);
        if (!scan_prefix_shorten_dominated_navigation_segments(solver, replay_options)) break;
    }
    if (free_terminal) {
        (void)scan_prefix_trim_free_terminal_navigation_suffix(solver, replay_options);
    }
    return scan_prefix_write_final_state(solver, replay_options);
}

static bool scan_prefix_waypoint_quotas_equal(
    const SokobanSolver* solver,
    const Entity* baseline_waypoints,
    int baseline_count,
    const Entity* candidate_waypoints,
    int candidate_count
) {
    if (!solver || !baseline_waypoints || !candidate_waypoints ||
        baseline_count < 0 || baseline_count > MAX_BOXES + MAX_TARGETS ||
        candidate_count != baseline_count) {
        return false;
    }

    int baseline_boxes = 0;
    int candidate_boxes = 0;
    for (int i = 0; i < baseline_count; i++) {
        bool baseline_is_box = false;
        bool candidate_is_box = false;
        int entity_idx = -1;
        if (!scan_decode_waypoint_entity(
                solver, baseline_waypoints[i].id, &baseline_is_box, &entity_idx) ||
            !scan_decode_waypoint_entity(
                solver, candidate_waypoints[i].id, &candidate_is_box, &entity_idx)) {
            return false;
        }
        if (baseline_is_box) baseline_boxes++;
        if (candidate_is_box) candidate_boxes++;
    }
    return baseline_boxes == candidate_boxes;
}

static bool scan_prefix_restore_initial_solver_state(SokobanSolver* solver) {
    if (!solver) return false;

    PathReplayState initial_state;
    if (!scan_replay_load_from_solver(
            &initial_state,
            &g_scan_initial_bmap, g_scan_initial_player,
            g_scan_initial_boxes, solver->num_boxes,
            g_scan_initial_bombs, g_scan_initial_num_bombs)) {
        return false;
    }
    scan_replay_write_solver_state(solver, &initial_state);
    solver_refresh_deadlocks(solver);
    return true;
}

static bool scan_prefix_finalize_candidate(
    SokobanSolver* solver,
    const PathReplayOptions* replay_options,
    const Direction* path,
    uint16_t path_len,
    const Entity* waypoints,
    const Position* pauses,
    int waypoint_count,
    bool orig_strict,
    PathReplayState* out_final_state
) {
    if (!solver || !replay_options || !path || !waypoints || !pauses || !out_final_state ||
        path_len == 0 || path_len >= MAX_PATH_LENGTH ||
        waypoint_count <= 0 || waypoint_count > MAX_BOXES + MAX_TARGETS) {
        return false;
    }

    scan_prefix_write_path_to_solver(solver, path, path_len, orig_strict);
    solver->scan_waypoint_count = waypoint_count;
    memcpy(solver->scan_waypoints, waypoints, waypoint_count * sizeof(Entity));
    memcpy(solver->scan_player_pause_positions, pauses, waypoint_count * sizeof(Position));

    solver_optimize_post_path(solver);
    scan_prefix_rebind_waypoints_after_dewater(solver, waypoints, pauses, waypoint_count);
    scan_prefix_restore_if_dewater_invalid(
        solver, replay_options, path, path_len, waypoints, pauses, waypoint_count
    );
    if (!scan_prefix_finalize_semantic_route(solver, replay_options, true)) return false;
    return scan_prefix_validate_bound_path(
        solver, solver->best_path, solver->best_path_len,
        solver->scan_waypoints, solver->scan_player_pause_positions,
        solver->scan_waypoint_count, replay_options, out_final_state
    );
}

static void scan_prefix_restore_final_candidate(
    SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    const Entity* waypoints,
    const Position* pauses,
    int waypoint_count,
    bool orig_strict,
    const PathReplayState* final_state
) {
    if (!solver || !path || !waypoints || !pauses || !final_state) return;

    scan_prefix_write_path_to_solver(solver, path, path_len, orig_strict);
    solver->scan_waypoint_count = waypoint_count;
    memcpy(solver->scan_waypoints, waypoints, waypoint_count * sizeof(Entity));
    memcpy(solver->scan_player_pause_positions, pauses, waypoint_count * sizeof(Position));
    scan_replay_write_solver_state(solver, final_state);
    scan_refresh_bomb_delay_events(solver);
    solver_refresh_deadlocks(solver);
}

FAST_OCRAM_FUNC static bool solver_generate_scan_path_selected_main_route(SokobanSolver* solver) {
    if (!solver) return false;

    scan_prefix_capture_initial_state(solver);

    if (solver->num_bombs == 0) {
        solver->is_scanning = true;
        solver->scan_current_index = 0;
        if (!sokoban_generate_scan_path(solver)) return false;
        PathReplayOptions replay_options = scan_replay_lenient_options();
        if (!scan_prefix_finalize_semantic_route(solver, &replay_options, false)) return false;
        return solver->scan_waypoint_count > 0;
    }

    bool orig_strict = false;
    if (!scan_prefix_solve_main_route(solver, &orig_strict)) return false;

    PathReplayOptions replay_options = scan_replay_lenient_options();

    int cutoff_idx = -1;
    if (!scan_prefix_find_last_blast_cutoff(solver, &replay_options, &cutoff_idx)) return false;

    int req_boxes = (solver->num_boxes > 1) ? solver->num_boxes - 1 : solver->num_boxes;
    int req_targets = (solver->num_targets > 1) ? solver->num_targets - 1 : solver->num_targets;

    Direction* hybrid_path = g_scan_hybrid_path;
    uint16_t hybrid_len = 0;
    if (!scan_prefix_build_hybrid_path(
            solver, cutoff_idx, req_boxes, req_targets,
            &replay_options, hybrid_path, &hybrid_len)) {
        return false;
    }

    static Direction spine_path[MAX_PATH_LENGTH];
    static Entity spine_waypoints[MAX_BOXES + MAX_TARGETS];
    static Position spine_pauses[MAX_BOXES + MAX_TARGETS];
    uint16_t spine_len = 0;
    int spine_waypoint_count = 0;
    scan_prefix_try_spine_replacement(
        solver, cutoff_idx, req_boxes, req_targets, hybrid_path, &hybrid_len
    );
    bool have_spine_candidate = scan_prefix_build_spine_candidate(
        solver, cutoff_idx, req_boxes, req_targets,
        hybrid_path, hybrid_len,
        spine_path, &spine_len,
        spine_waypoints, spine_pauses, &spine_waypoint_count
    );
    const Direction* pre_squeeze_path = hybrid_path;
    uint16_t pre_squeeze_len = hybrid_len;
    scan_prefix_write_path_to_solver(solver, hybrid_path, hybrid_len, orig_strict);

    static Entity hybrid_waypoints[MAX_BOXES + MAX_TARGETS];
    static Position hybrid_pauses[MAX_BOXES + MAX_TARGETS];
    int hybrid_waypoint_count = 0;
    scan_prefix_snapshot_waypoints(
        solver, hybrid_waypoints, hybrid_pauses, &hybrid_waypoint_count
    );

    solver_optimize_post_path(solver);
    scan_prefix_rebind_waypoints_after_dewater(
        solver, hybrid_waypoints, hybrid_pauses, hybrid_waypoint_count
    );
    scan_prefix_restore_if_dewater_invalid(
        solver, &replay_options, pre_squeeze_path, pre_squeeze_len,
        hybrid_waypoints, hybrid_pauses, hybrid_waypoint_count
    );
    if (!scan_prefix_finalize_semantic_route(solver, &replay_options, false)) return false;

    PathReplayState hybrid_final_state;
    if (!scan_prefix_validate_bound_path(
            solver, solver->best_path, solver->best_path_len,
            solver->scan_waypoints, solver->scan_player_pause_positions,
            solver->scan_waypoint_count, &replay_options, &hybrid_final_state)) {
        return false;
    }

    uint16_t hybrid_final_len = solver->best_path_len;
    int hybrid_final_waypoint_count = solver->scan_waypoint_count;
    memcpy(hybrid_path, solver->best_path, hybrid_final_len * sizeof(Direction));
    scan_prefix_snapshot_waypoints(
        solver, hybrid_waypoints, hybrid_pauses, &hybrid_final_waypoint_count
    );
    if (!have_spine_candidate) return hybrid_final_waypoint_count > 0;

    if (!scan_prefix_restore_initial_solver_state(solver)) {
        return hybrid_final_waypoint_count > 0;
    }

    PathReplayState spine_final_state;
    if (!scan_prefix_finalize_candidate(
            solver, &replay_options,
            spine_path, spine_len,
            spine_waypoints, spine_pauses, spine_waypoint_count,
            orig_strict, &spine_final_state)) {
        scan_prefix_restore_final_candidate(
            solver,
            hybrid_path, hybrid_final_len,
            hybrid_waypoints, hybrid_pauses, hybrid_final_waypoint_count,
            orig_strict, &hybrid_final_state
        );
        return hybrid_final_waypoint_count > 0;
    }

    if (!scan_prefix_validate_first_arrival_observations(
            solver,
            solver->best_path, solver->best_path_len,
            solver->scan_waypoints, solver->scan_player_pause_positions,
            solver->scan_waypoint_count, &replay_options)) {
        scan_prefix_restore_final_candidate(
            solver,
            hybrid_path, hybrid_final_len,
            hybrid_waypoints, hybrid_pauses, hybrid_final_waypoint_count,
            orig_strict, &hybrid_final_state
        );
        return hybrid_final_waypoint_count > 0;
    }

    bool use_spine_candidate =
        scan_prefix_physical_states_equal(&hybrid_final_state, &spine_final_state) &&
        scan_prefix_waypoint_quotas_equal(
            solver,
            hybrid_waypoints, hybrid_final_waypoint_count,
            solver->scan_waypoints, solver->scan_waypoint_count
        ) &&
        scan_path_is_better(
            solver->best_path, solver->best_path_len,
            hybrid_path, hybrid_final_len
        );

    if (!use_spine_candidate) {
        scan_prefix_restore_final_candidate(
            solver,
            hybrid_path, hybrid_final_len,
            hybrid_waypoints, hybrid_pauses, hybrid_final_waypoint_count,
            orig_strict, &hybrid_final_state
        );
    }
    return solver->scan_waypoint_count > 0;
}

FAST_OCRAM_FUNC static bool solver_generate_scan_path_once(SokobanSolver* solver) {
    if (!solver) return false;

    g_scan_bomb_main_route_override_index = -1;
    if (!solver_generate_scan_path_selected_main_route(solver)) return false;
    if (g_scan_bomb_main_route_candidate_count == 0) return true;

    PathReplayOptions replay_options = scan_replay_lenient_options();
    if (!scan_prefix_validate_bound_path(
            solver, solver->best_path, solver->best_path_len,
            solver->scan_waypoints, solver->scan_player_pause_positions,
            solver->scan_waypoint_count, &replay_options,
            &g_scan_bomb_best_final_state)) {
        return true;
    }

    bool orig_strict = solver->strict_target_mode;
    uint16_t best_len = solver->best_path_len;
    int best_waypoint_count = solver->scan_waypoint_count;
    memcpy(g_scan_verified_path, solver->best_path, best_len * sizeof(Direction));
    memcpy(g_scan_bomb_best_waypoints, solver->scan_waypoints,
           best_waypoint_count * sizeof(Entity));
    memcpy(g_scan_bomb_best_pauses, solver->scan_player_pause_positions,
           best_waypoint_count * sizeof(Position));

    for (uint8_t i = 0; i < g_scan_bomb_main_route_candidate_count; i++) {
        if (!scan_prefix_restore_initial_solver_state(solver)) break;
        g_scan_bomb_main_route_override_index = (int8_t)i;
        bool candidate_ok =
            solver_generate_scan_path_selected_main_route(solver);
        g_scan_bomb_main_route_override_index = -1;
        if (!candidate_ok) continue;
        if (!scan_prefix_waypoint_quotas_equal(
                solver,
                g_scan_bomb_best_waypoints, best_waypoint_count,
                solver->scan_waypoints, solver->scan_waypoint_count)) {
            continue;
        }
        if (!scan_path_is_better(
                solver->best_path, solver->best_path_len,
                g_scan_verified_path, best_len)) {
            continue;
        }

        PathReplayState candidate_final_state;
        if (!scan_prefix_validate_bound_path(
                solver, solver->best_path, solver->best_path_len,
                solver->scan_waypoints, solver->scan_player_pause_positions,
                solver->scan_waypoint_count, &replay_options,
                &candidate_final_state)) {
            continue;
        }

        best_len = solver->best_path_len;
        best_waypoint_count = solver->scan_waypoint_count;
        memcpy(g_scan_verified_path, solver->best_path,
               best_len * sizeof(Direction));
        memcpy(g_scan_bomb_best_waypoints, solver->scan_waypoints,
               best_waypoint_count * sizeof(Entity));
        memcpy(g_scan_bomb_best_pauses, solver->scan_player_pause_positions,
               best_waypoint_count * sizeof(Position));
        g_scan_bomb_best_final_state = candidate_final_state;
    }

    g_scan_bomb_main_route_override_index = -1;
    scan_prefix_restore_final_candidate(
        solver,
        g_scan_verified_path, best_len,
        g_scan_bomb_best_waypoints, g_scan_bomb_best_pauses,
        best_waypoint_count, orig_strict, &g_scan_bomb_best_final_state
    );
    return best_waypoint_count > 0;
}

static int scan_return_region_axis_sum(Position a, Position b, bool use_x_axis) {
    return use_x_axis ? ((int)a.x + (int)b.x) : ((int)a.y + (int)b.y);
}

static int scan_return_region_side_margin4(const ScanReturnRegionPattern* pattern, Position pos) {
    if (!pattern) return 0;
    int coord = pattern->use_x_axis ? (int)pos.x : (int)pos.y;
    int boundary4 = pattern->entry_sum + pattern->middle_sum;
    int coord4 = coord * 4;
    return abs(coord4 - boundary4);
}

static bool scan_return_region_position_on_entry_side(const ScanReturnRegionPattern* pattern, Position pos) {
    if (!pattern) return false;
    int coord = pattern->use_x_axis ? (int)pos.x : (int)pos.y;
    int boundary4 = pattern->entry_sum + pattern->middle_sum;
    int coord4 = coord * 4;
    if (pattern->middle_sum > pattern->entry_sum) {
        return coord4 <= boundary4;
    }
    return coord4 >= boundary4;
}

static bool scan_build_return_region_pattern_from_points(
    Position w0,
    Position w1,
    Position w2,
    Position w3,
    ScanReturnRegionPattern* out_pattern
) {
    int entry_x = scan_return_region_axis_sum(w0, w3, true);
    int middle_x = scan_return_region_axis_sum(w1, w2, true);
    int entry_y = scan_return_region_axis_sum(w0, w3, false);
    int middle_y = scan_return_region_axis_sum(w1, w2, false);
    int sep_x = abs(middle_x - entry_x);
    int sep_y = abs(middle_y - entry_y);

    bool use_x_axis = sep_x > sep_y;
    int sep = use_x_axis ? sep_x : sep_y;
    if (sep < SCAN_RETURN_REGION_MIN_AXIS_SEPARATION) return false;

    ScanReturnRegionPattern pattern;
    pattern.use_x_axis = use_x_axis;
    pattern.entry_sum = use_x_axis ? entry_x : entry_y;
    pattern.middle_sum = use_x_axis ? middle_x : middle_y;
    if (pattern.entry_sum == pattern.middle_sum) return false;

    if (!scan_return_region_position_on_entry_side(&pattern, w0)) return false;
    if (!scan_return_region_position_on_entry_side(&pattern, w3)) return false;
    if (scan_return_region_position_on_entry_side(&pattern, w1)) return false;
    if (scan_return_region_position_on_entry_side(&pattern, w2)) return false;
    if (scan_return_region_side_margin4(&pattern, w0) <= SCAN_RETURN_REGION_MIN_SIDE_MARGIN4) return false;
    if (scan_return_region_side_margin4(&pattern, w1) <= SCAN_RETURN_REGION_MIN_SIDE_MARGIN4) return false;
    if (scan_return_region_side_margin4(&pattern, w2) <= SCAN_RETURN_REGION_MIN_SIDE_MARGIN4) return false;
    if (scan_return_region_side_margin4(&pattern, w3) <= SCAN_RETURN_REGION_MIN_SIDE_MARGIN4) return false;

    if (out_pattern) *out_pattern = pattern;
    return true;
}

static bool scan_build_return_region_pattern(const SokobanSolver* solver, ScanReturnRegionPattern* out_pattern) {
    if (!solver || solver->scan_waypoint_count < SCAN_VERIFIED_WAYPOINT_COUNT) return false;

    int max_start = solver->scan_waypoint_count - SCAN_VERIFIED_WAYPOINT_COUNT;
    for (int start = 0; start <= max_start; start++) {
        Position w0 = solver->scan_waypoints[start].pos;
        Position w1 = solver->scan_waypoints[start + 1].pos;
        Position w2 = solver->scan_waypoints[start + 2].pos;
        Position w3 = solver->scan_waypoints[start + 3].pos;
        if (scan_build_return_region_pattern_from_points(w0, w1, w2, w3, out_pattern)) {
            return true;
        }
    }

    return false;
}

static ScanVerificationLevel scan_select_verification_level(const ScanStrategyPolicy* policy, uint16_t fast_len) {
    if (!policy) return SCAN_VERIFY_NONE;
    if (!policy->path_has_return_region_shift ||
        !policy->initial_player_in_return_entry_region ||
        fast_len < SCAN_VERIFIED_MIN_FAST_LEN) {
        return SCAN_VERIFY_NONE;
    }

    if (policy->initial_small_balanced_case) {
        return SCAN_VERIFY_STRICT;
    }

    return SCAN_VERIFY_LIGHT;
}

static ScanStrategyPolicy scan_build_strategy_policy(
    const SokobanSolver* solver,
    uint16_t fast_len
) {
    ScanStrategyPolicy policy;
    memset(&policy, 0, sizeof(policy));

    ScanReturnRegionPattern return_pattern;
    policy.initial_small_balanced_case = scan_initial_small_balanced_case(solver);
    policy.path_has_return_region_shift = scan_build_return_region_pattern(solver, &return_pattern);
    policy.initial_player_in_return_entry_region =
        policy.path_has_return_region_shift &&
        scan_return_region_position_on_entry_side(&return_pattern, g_scan_initial_player);
    policy.verification_level = scan_select_verification_level(&policy, fast_len);
    return policy;
}

FAST_OCRAM_FUNC static bool solver_generate_scan_path_core(SokobanSolver* solver) {
    if (!solver) return false;
    ScanVerificationLevel saved_verification = g_enable_path_verification;
    g_enable_path_verification = SCAN_VERIFY_NONE;
    bool ok = solver_generate_scan_path_once(solver);
    if (!ok) {
        g_enable_path_verification = saved_verification;
        return false;
    }

    uint16_t fast_len = solver->best_path_len;
    ScanStrategyPolicy scan_policy = scan_build_strategy_policy(solver, fast_len);
    ScanVerificationLevel verification_level = scan_policy.verification_level;
    bool should_try_verified_scan = verification_level != SCAN_VERIFY_NONE;

    uint16_t fast_verified_scan_len = 0xFFFF;
    if (should_try_verified_scan) {
        fast_verified_scan_len = fast_len;
        SokobanSolver* verified_state = &g_scan_verified_solver;
        Direction* verified_path = g_scan_verified_path;
        *verified_state = *solver;
        verified_state->best_path = verified_path;
        verified_state->bmap = g_scan_initial_bmap;
        verified_state->start_player = g_scan_initial_player;
        memcpy(verified_state->boxes, g_scan_initial_boxes, sizeof(verified_state->boxes));
        memcpy(verified_state->bombs, g_scan_initial_bombs, sizeof(verified_state->bombs));
        verified_state->num_bombs = g_scan_initial_num_bombs;
        verified_state->scan_waypoint_count = 0;
        verified_state->scan_current_index = 0;
        verified_state->best_path_len = 0;
        verified_state->best_steps = 0xFFFF;
        solver_refresh_deadlocks(verified_state);

        g_enable_path_verification = verification_level;
        bool verified_ok = solver_generate_scan_path_once(verified_state);
        g_enable_path_verification = SCAN_VERIFY_NONE;
        uint16_t verified_scan_len = verified_ok ? verified_state->best_path_len : 0xFFFF;
        if (verified_ok && verified_state->scan_waypoint_count == solver->scan_waypoint_count &&
            verified_scan_len < fast_verified_scan_len && verified_state->best_path_len < fast_len) {
            solver->bmap = verified_state->bmap;
            solver->destroyed_walls_mask = verified_state->destroyed_walls_mask;
            solver->start_player = verified_state->start_player;
            memcpy(solver->boxes, verified_state->boxes, sizeof(solver->boxes));
            memcpy(solver->targets, verified_state->targets, sizeof(solver->targets));
            memcpy(solver->bombs, verified_state->bombs, sizeof(solver->bombs));
            solver->num_boxes = verified_state->num_boxes;
            solver->num_targets = verified_state->num_targets;
            solver->num_bombs = verified_state->num_bombs;
            solver->strict_target_mode = verified_state->strict_target_mode;
            solver->is_scanning = verified_state->is_scanning;
            solver->backup_needed_box = verified_state->backup_needed_box;
            solver->backup_needed_target = verified_state->backup_needed_target;
            solver->backup_activated = verified_state->backup_activated;
            solver->scan_current_index = verified_state->scan_current_index;
            solver->scan_waypoint_count = verified_state->scan_waypoint_count;
            memcpy(solver->scan_player_pause_positions, verified_state->scan_player_pause_positions, sizeof(solver->scan_player_pause_positions));
            memcpy(solver->scan_waypoints, verified_state->scan_waypoints, sizeof(solver->scan_waypoints));
            solver->best_path_len = verified_state->best_path_len;
            solver->best_steps = verified_state->best_steps;
            memcpy(solver->best_path, verified_state->best_path, verified_state->best_path_len * sizeof(Direction));
            solver_refresh_deadlocks(solver);
            fast_len = solver->best_path_len;
        }
    }

    /* Once every observation is complete, a trailing navigation-only suffix
     * cannot contribute to the scan state. Drop it only when strict replay
     * proves that the physical final state is unchanged. */
    PathReplayOptions terminal_replay_options = scan_replay_lenient_options();
    if (scan_prefix_trim_free_terminal_navigation_suffix(
            solver, &terminal_replay_options)) {
        if (!scan_prefix_write_final_state(solver, &terminal_replay_options)) {
            g_enable_path_verification = saved_verification;
            return false;
        }
    }

    g_enable_path_verification = saved_verification;
    return true;
}


FAST_OCRAM_FUNC bool solver_generate_scan_path(SokobanSolver* solver) {
    if (!solver) return false;

    SokobanScanCacheKey key;
    bool has_scan_cache_key = false;
    SokobanScanCachePayload* payload = NULL;

    if (g_sokoban_flash_cache_enabled != 0u) {
        payload = scan_cache_flash_payload();
        has_scan_cache_key = solver_build_scan_cache_key(solver, &key);
        if (has_scan_cache_key &&
            scan_cache_flash_find(&key, payload) &&
            solver_try_apply_scan_cache(solver, payload)) {
            return true;
        }
    }

    if (!solver_generate_scan_path_core(solver)) {
        return false;
    }

    scan_refresh_bomb_delay_events(solver);

    if (g_sokoban_flash_cache_enabled != 0u && has_scan_cache_key && payload) {
        if (solver_export_scan_cache(solver, payload)) {
            (void)scan_cache_flash_store(&key, payload);
        }
    }

    return true;
}

static uint32_t scan_cache_crc32_bytes(const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static uint32_t scan_cache_compute_key_crc(const SokobanScanCacheKey* key) {
    SokobanScanCacheKey temp = *key;
    temp.key_crc = 0;
    return scan_cache_crc32_bytes(&temp, sizeof(temp));
}

bool solver_build_scan_cache_key(const SokobanSolver* solver, SokobanScanCacheKey* out_key) {
    if (g_sokoban_flash_cache_enabled == 0u) return false;
    if (!solver || !out_key || solver->num_boxes < SOKOBAN_FLASH_CACHE_MIN_BOXES) return false;
    memset(out_key, 0, sizeof(*out_key));
    out_key->policy_version = SOKOBAN_SCAN_CACHE_POLICY_VERSION;
    out_key->cache_kind = SOKOBAN_FLASH_CACHE_KIND_SCAN;
    /* 扫描阶段不区分直接/识别求解，ID 仍未可靠提交。 */
    out_key->solve_mode = 0u;
    out_key->rows = MAP_ROWS;
    out_key->cols = MAP_COLS;
    out_key->num_boxes = solver->num_boxes;
    out_key->num_targets = solver->num_targets;
    out_key->num_bombs = solver->num_bombs;
    out_key->start_player = solver->start_player;
    memcpy(out_key->walls, solver->bmap.walls, sizeof(out_key->walls));
    memcpy(out_key->targets, solver->bmap.targets, sizeof(out_key->targets));
    memcpy(out_key->boxes, solver->bmap.boxes, sizeof(out_key->boxes));
    memcpy(out_key->bombs, solver->bmap.bombs, sizeof(out_key->bombs));
    /* 扫描缓存只按物理局面复用，不把本轮尚未可靠提交的 ID 带入键。 */
    memset(out_key->box_ids, 0xFF, sizeof(out_key->box_ids));
    memset(out_key->target_ids, 0xFF, sizeof(out_key->target_ids));
    out_key->key_crc = scan_cache_compute_key_crc(out_key);
    return true;
}

static bool scan_cache_replay_payload(
    const SokobanSolver* solver,
    const SokobanScanCachePayload* payload,
    BitboardMap* out_map,
    Position* out_player,
    Position out_boxes[MAX_BOXES],
    Entity out_bombs[MAX_BOMBS],
    int* out_num_bombs
) {
    if (!solver || !payload || !out_map || !out_player || !out_boxes || !out_bombs || !out_num_bombs) return false;
    if (payload->path_len == 0 || payload->path_len >= MAX_PATH_LENGTH) return false;
    if (payload->waypoint_count > SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS) return false;

    PathReplayState replay_state;
    if (!path_replay_load_state(
            &replay_state,
            &solver->bmap,
            solver->start_player,
            solver->boxes,
            solver->num_boxes,
            solver->bombs,
            solver->num_bombs)) {
        return false;
    }

    PathReplayOptions replay_options = {0};
    replay_options.mode = PATH_REPLAY_STRICT_VALIDATE;
    uint8_t pauses_seen = 0;
    for (uint16_t i = 0; i < payload->path_len; i++) {
        Direction d = payload->path[i];
        if (direction_is_pause(d)) {
            if (pauses_seen >= payload->waypoint_count) return false;
            if (!pos_equal(payload->pause_positions[pauses_seen], replay_state.player)) return false;
            pauses_seen++;
        }

        PathReplayStepResult step = path_replay_step(solver, &replay_state, d, &replay_options);
        if (step.kind == PATH_REPLAY_STEP_ERROR || step.kind == PATH_REPLAY_STEP_STOPPED) return false;
    }

    if (pauses_seen != payload->waypoint_count) return false;
    *out_map = replay_state.map;
    *out_player = replay_state.player;
    for (int i = 0; i < MAX_BOXES; i++) out_boxes[i] = replay_state.boxes[i];
    memset(out_bombs, 0, sizeof(Entity) * MAX_BOMBS);
    if (replay_state.bomb_count > 0) memcpy(out_bombs, replay_state.bombs, sizeof(Entity) * replay_state.bomb_count);
    *out_num_bombs = replay_state.bomb_count;
    return true;
}
bool solver_try_apply_scan_cache(SokobanSolver* solver, const SokobanScanCachePayload* payload) {
    if (g_sokoban_flash_cache_enabled == 0u) return false;
    if (!solver || !payload || !solver->best_path) return false;
    if (payload->waypoint_count == 0 || payload->waypoint_count > SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS) return false;

    BitboardMap initial_map = solver->bmap;
    Position initial_player = solver->start_player;
    Entity initial_boxes[MAX_BOXES];
    Entity initial_bombs[MAX_BOMBS];
    memcpy(initial_boxes, solver->boxes, sizeof(initial_boxes));
    memcpy(initial_bombs, solver->bombs, sizeof(initial_bombs));
    int initial_num_bombs = solver->num_bombs;

    BitboardMap final_map;
    Position final_player;
    Position final_boxes[MAX_BOXES];
    Entity final_bombs[MAX_BOMBS];
    int final_num_bombs = 0;
    if (!scan_cache_replay_payload(solver, payload, &final_map, &final_player,
                                   final_boxes, final_bombs, &final_num_bombs)) {
        return false;
    }
    if (!pos_equal(final_player, payload->end_player)) return false;
    if (memcmp(final_map.walls, payload->after_walls, sizeof(payload->after_walls)) != 0) return false;

    g_scan_initial_bmap = initial_map;
    g_scan_initial_player = initial_player;
    memcpy(g_scan_initial_boxes, initial_boxes, sizeof(g_scan_initial_boxes));
    memcpy(g_scan_initial_bombs, initial_bombs, sizeof(g_scan_initial_bombs));
    g_scan_initial_num_bombs = initial_num_bombs;
    memcpy(solver->best_path, payload->path, payload->path_len * sizeof(Direction));
    solver->best_path_len = payload->path_len;
    solver->best_steps = payload->path_len;
    solver->is_scanning = true;
    solver->scan_current_index = 0;
    solver->scan_waypoint_count = payload->waypoint_count;
    memcpy(solver->scan_waypoints, payload->waypoints, payload->waypoint_count * sizeof(Entity));
    memcpy(solver->scan_player_pause_positions, payload->pause_positions, payload->waypoint_count * sizeof(Position));

    solver->bmap = final_map;
    solver->start_player = final_player;
    for (int i = 0; i < solver->num_boxes && i < MAX_BOXES; i++) solver->boxes[i].pos = final_boxes[i];
    solver->num_bombs = (uint8_t)final_num_bombs;
    memset(solver->bombs, 0, sizeof(solver->bombs));
    if (final_num_bombs > 0) memcpy(solver->bombs, final_bombs, sizeof(Entity) * final_num_bombs);
    scan_refresh_bomb_delay_events(solver);
    solver_refresh_deadlocks(solver);
    return true;
}

bool solver_export_scan_cache(const SokobanSolver* solver, SokobanScanCachePayload* out_payload) {
    if (g_sokoban_flash_cache_enabled == 0u) return false;
    if (!solver || !out_payload || !solver->best_path) return false;
    if (solver->num_boxes < SOKOBAN_FLASH_CACHE_MIN_BOXES) return false;
    if (solver->best_path_len == 0 || solver->best_path_len >= MAX_PATH_LENGTH) return false;
    if (solver->scan_waypoint_count <= 0 || solver->scan_waypoint_count > SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS) return false;

    memset(out_payload, 0, sizeof(*out_payload));
    out_payload->path_len = solver->best_path_len;
    memcpy(out_payload->path, solver->best_path, solver->best_path_len * sizeof(Direction));
    out_payload->waypoint_count = (uint8_t)solver->scan_waypoint_count;
    memcpy(out_payload->waypoints, solver->scan_waypoints, solver->scan_waypoint_count * sizeof(Entity));
    memcpy(out_payload->pause_positions, solver->scan_player_pause_positions, solver->scan_waypoint_count * sizeof(Position));
    out_payload->end_player = solver->start_player;
    memcpy(out_payload->after_walls, solver->bmap.walls, sizeof(out_payload->after_walls));
    return true;
}
// 根据已经识别的 ID 自动补全剩余箱子和目标。无法可靠推断时保持未知。
FAST_OCRAM_FUNC bool sokoban_auto_assign_remaining_ids(SokobanSolver* solver) {
    if (!solver || solver->num_boxes != solver->num_targets) return false;
    int b_counts[10] = {0};
    int t_counts[10] = {0};
    int missing_box_indices[MAX_BOXES];
    int missing_target_indices[MAX_TARGETS];
    int missing_b = 0;
    int missing_t = 0;

    for (int i = 0; i < solver->num_boxes; i++) {
        int id = solver->boxes[i].id;
        if (id == -1) {
            if (missing_b >= MAX_BOXES) return false;
            missing_box_indices[missing_b++] = i;
        } else if (id >= 0 && id < 10) {
            b_counts[id]++;
        } else {
            return false;
        }
    }

    for (int i = 0; i < solver->num_targets; i++) {
        int id = solver->targets[i].id;
        if (id == -1) {
            if (missing_t >= MAX_TARGETS) return false;
            missing_target_indices[missing_t++] = i;
        } else if (id >= 0 && id < 10) {
            t_counts[id]++;
        } else {
            return false;
        }
    }

    int common_count = 0;
    int box_surplus[MAX_BOXES];
    int target_surplus[MAX_TARGETS];
    int box_surplus_count = 0;
    int target_surplus_count = 0;

    if (!scan_collect_id_balance(
            b_counts, t_counts, &common_count,
            box_surplus, &box_surplus_count,
            target_surplus, &target_surplus_count
        )) {
        return false;
    }

    int mb = 0;
    int mt = 0;
    int sb = 0;
    int st = 0;

    while (sb < box_surplus_count && mt < missing_t) {
        int id = box_surplus[sb++];
        solver->targets[missing_target_indices[mt++]].id = (int8_t)id;
        t_counts[id]++;
    }

    while (st < target_surplus_count && mb < missing_b) {
        int id = target_surplus[st++];
        solver->boxes[missing_box_indices[mb++]].id = (int8_t)id;
        b_counts[id]++;
    }

    if (!scan_collect_id_balance(
            b_counts, t_counts, &common_count,
            box_surplus, &box_surplus_count,
            target_surplus, &target_surplus_count
        )) {
        return false;
    }

    if (box_surplus_count > 0 || target_surplus_count > 0) {
        if (missing_b != 0 || missing_t != 0) return false;
        if (box_surplus_count != target_surplus_count) return false;
        if (common_count == 0) return false;
        for (int i = 0; i < box_surplus_count; i++) {
            int from_id = target_surplus[i];
            int to_id = box_surplus[i];
            bool rewritten = false;
            for (int t = 0; t < solver->num_targets; t++) {
                if (solver->targets[t].id == from_id) {
                    solver->targets[t].id = (int8_t)to_id;
                    rewritten = true;
                    break;
                }
            }
            if (!rewritten) return false;
            t_counts[from_id]--;
            t_counts[to_id]++;
        }
    }

    while (mb < missing_b && mt < missing_t) {
        int chosen_id = -1;
        for (int id = 0; id < 10; id++) {
            if (b_counts[id] == 0 && t_counts[id] == 0) {
                chosen_id = id;
                break;
            }
        }
        if (chosen_id < 0) return false;

        solver->boxes[missing_box_indices[mb++]].id = (int8_t)chosen_id;
        solver->targets[missing_target_indices[mt++]].id = (int8_t)chosen_id;
        b_counts[chosen_id]++;
        t_counts[chosen_id]++;
    }

    if (mb != missing_b || mt != missing_t) return false;

    memset(b_counts, 0, sizeof(b_counts));
    memset(t_counts, 0, sizeof(t_counts));
    for (int i = 0; i < solver->num_boxes; i++) {
        int id = solver->boxes[i].id;
        if (id < 0 || id >= 10) return false;
        b_counts[id]++;
    }
    for (int i = 0; i < solver->num_targets; i++) {
        int id = solver->targets[i].id;
        if (id < 0 || id >= 10) return false;
        t_counts[id]++;
    }
    for (int id = 0; id < 10; id++) {
        if (b_counts[id] != t_counts[id]) {

            return false;
        }
    }    return true;
}

static bool scan_restore_solver_state_at_pause(SokobanSolver* solver, int pause_index) {
    if (!solver || pause_index < 0 || pause_index >= solver->scan_waypoint_count) return false;

    PathReplayState replay_state;
    if (!path_replay_load_state(
            &replay_state,
            &g_scan_initial_bmap,
            g_scan_initial_player,
            g_scan_initial_boxes,
            solver->num_boxes,
            g_scan_initial_bombs,
            g_scan_initial_num_bombs)) {
        return false;
    }

    PathReplayOptions replay_options = {0};
    replay_options.mode = PATH_REPLAY_LEGACY_LENIENT;
    int pauses_seen = 0;
    uint16_t replay_len = 0;
    for (uint16_t i = 0; i < solver->best_path_len; i++) {
        Direction d = solver->best_path[i];
        if (direction_is_pause(d)) {
            if (pauses_seen == pause_index) {
                solver->bmap = replay_state.map;
                solver->start_player = replay_state.player;
                for (int b = 0; b < solver->num_boxes && b < MAX_BOXES; b++) {
                    solver->boxes[b].pos = replay_state.boxes[b];
                }
                solver->num_bombs = (uint8_t)replay_state.bomb_count;
                memset(solver->bombs, 0, sizeof(solver->bombs));
                if (replay_state.bomb_count > 0) {
                    memcpy(solver->bombs, replay_state.bombs, sizeof(Entity) * replay_state.bomb_count);
                }
                solver->best_path_len = replay_len;
                solver->best_steps = replay_len;
                scan_refresh_bomb_delay_events(solver);
                solver_refresh_deadlocks(solver);
                return true;
            }
            pauses_seen++;
            continue;
        }

        PathReplayStepResult step = path_replay_step(solver, &replay_state, d, &replay_options);
        if (step.kind == PATH_REPLAY_STEP_ERROR || step.kind == PATH_REPLAY_STEP_STOPPED) return false;
        replay_len = (uint16_t)(i + 1);
    }

    return false;
}
// 写入一个扫描点 ID：id=-1 表示无 ID 模式并结束扫描，id=-2 表示当前点还需要扩展补扫。
FAST_OCRAM_FUNC int solver_assign_next_scan_id(SokobanSolver* solver, int id) {
    if (!solver || !solver->is_scanning || id < -2 || id > 9) return -1;

    // 选择无 ID 模式时恢复到当前扫描暂停状态，避免小车坐标停留在旧终点。
    if (id == -1) {
        int pause_idx = solver->scan_current_index;

        if (!scan_restore_solver_state_at_pause(solver, pause_idx) &&
            pause_idx >= 0 && pause_idx < solver->scan_waypoint_count) {
            solver->start_player = solver->scan_player_pause_positions[pause_idx];
            solver_refresh_deadlocks(solver);
        }
        solver->is_scanning = false;
        solver->strict_target_mode = false;
        solver->scan_current_index = 0;
        solver->scan_waypoint_count = 0;
        return 0;
    }

    int idx = solver->scan_current_index;
    if (idx < 0 || idx >= solver->scan_waypoint_count) return -1;
    Position target_pos = solver->scan_waypoints[idx].pos;

    bool is_box = false;
    int entity_idx = -1;
    bool entity_found = scan_decode_waypoint_entity(solver, solver->scan_waypoints[idx].id, &is_box, &entity_idx);
    if (!entity_found) {
        entity_found = scan_find_waypoint_entity_by_position(solver, target_pos, &is_box, &entity_idx);
    }



    if (entity_found && id >= 0) {
        if (is_box) {
            solver->boxes[entity_idx].id = (int8_t)id;
        } else {
            solver->targets[entity_idx].id = (int8_t)id;
        }

    }

    // 当前观察点信息不足时触发补扫，扫描模块会在当前位置附近继续扩展。
    if (id == -2 &&
        (!entity_found || !sokoban_extend_scan_path(solver, is_box, !is_box, idx))) {
        return -1;
    }

    solver->scan_current_index++;

    // 所有扫描点录入完成后切回严格模式，让后续求解按识别出的 ID 约束运行。
    if (solver->scan_current_index >= solver->scan_waypoint_count) {
        solver->is_scanning = false;
        if (!sokoban_auto_assign_remaining_ids(solver)) {
            return -1;
        }
        solver->strict_target_mode = true;
        solver->identified_solve_mode = true;
        return 2;
    }
    return 1;
}
