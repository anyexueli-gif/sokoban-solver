#include "sokoban_solver.h"
#include "sokoban_scan.h"
#include "sokoban_flash.h"

#include <stdlib.h>
#include <string.h>


// 文件职责：求解器核心，包含状态哈希、剪枝、炸弹策略、深度优先验证和对外接口。



#define FAST_RAM_FUNC __attribute__((section("ITCM_NonCacheable")))
#define FAST_OCRAM_FUNC __attribute__((section("OCRAM_CODE")))
#define ALLOC_IN_OCRAM SOKOBAN_BSS_SECTION("OCRAM_CACHE")
#define ALLOC_IN_DTCM  SOKOBAN_BSS_SECTION("RW_m_data")
#define ALLOC_IN_SDRAM_CACHE SOKOBAN_BSS_SECTION("SDRAM_CACHE")
#define ALLOC_IN_SDRAM_NOCACHE __attribute__((section("SDRAM_NonCacheable")))
#define ALLOC_IN_SDRAM         ALLOC_IN_SDRAM_CACHE
bool g_sandbox_mode = false;
uint32_t g_sandbox_budget_limit = 0;
/* 5+ 箱子地图的扫描/直接缓存默认启用；上层仍可显式置 0 禁用。 */
uint8_t g_sokoban_flash_cache_enabled = 0u;
static uint16_t g_O1_blast_mask[MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_O1_deadlock_clear[MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_O1_ray_left[MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_O1_ray_right[MAP_COLS] ALLOC_IN_OCRAM;
ScanVerificationLevel g_enable_path_verification = SCAN_VERIFY_NONE;
static void init_O1_lookup_tables(void) {
    static bool inited = false;
    if (inited) return;

#if MAP_COLS > 16
#error "Bitboard rows are uint16_t, so MAP_COLS must not exceed 16."
#endif

    const uint32_t valid_cols_mask = (MAP_COLS == 16) ? 0xFFFFu : ((1u << MAP_COLS) - 1u);
    const uint32_t non_left_wall_mask = valid_cols_mask & ~1u;
    const uint32_t non_right_wall_mask = (1u << (MAP_COLS - 1)) - 1u;

    for (int x = 0; x < MAP_COLS; x++) {
        uint32_t blast = (x == 0) ? 0x03u : (0x07u << (x - 1));
        g_O1_blast_mask[x] = (uint16_t)(blast & valid_cols_mask);

        uint32_t dead_clear;
        if (x == 0) dead_clear = 0x07u;
        else if (x == 1) dead_clear = 0x0Fu;
        else dead_clear = 0x1Fu << (x - 2);
        g_O1_deadlock_clear[x] = (uint16_t)(~(dead_clear & valid_cols_mask));

        g_O1_ray_left[x] = (uint16_t)(((1u << x) - 1u) & non_left_wall_mask);
        g_O1_ray_right[x] = (uint16_t)((~((1u << (x + 1)) - 1u)) & non_right_wall_mask);
    }
    inited = true;
}

static SokobanSolver g_solver_instance;
static AStarNode g_heap_buffer[1];
static uint16_t g_closed_parent_index[MAX_CLOSED_SIZE] ALLOC_IN_OCRAM;
static uint8_t g_closed_dir_and_steps[MAX_CLOSED_SIZE] ALLOC_IN_OCRAM;
static ClosedNode g_closed_buffer = { g_closed_parent_index, g_closed_dir_and_steps };
static Direction g_best_path_buffer[MAX_PATH_LENGTH];
uint16_t g_macro_dist_field[MAX_MACRO_DEPTH][MAP_ROWS][MAP_COLS] ALLOC_IN_DTCM;
static uint16_t g_target_dist_field[MAX_TARGETS][MAP_ROWS][MAP_COLS] ALLOC_IN_DTCM;
static bool g_use_target_dist_heuristic = false;

static Direction g_dfs_path[MAX_BOXES][MAX_SINGLE_PATH];
static Direction g_dfs_full_path_buffer[MAX_PATH_LENGTH];

uint16_t g_astar_max_steps = 0xFFFF;
static bool g_dfs_first_solution_only = false;
static bool g_bomb_seed_first_solution_only = false;
static bool g_bomb_seed_solution_committed = false;
static bool g_assignment_bounded_refinement = false;
static uint16_t g_assignment_exclusive_upper_bound = UINT16_MAX;
static bool g_assignment_refinement_disabled = false;
static bool g_bomb_state_beam_tail_only = false;
static bool g_bomb_state_beam_full_fallback_active = false;
static bool g_d1_tail_search = false;
static uint32_t g_d1_tail_dfs_nodes = 0;
static bool g_force_maneuver_rescue = false;
static bool g_enable_push_reach_filter = false;
static bool g_enable_topology_soft_order = false;


#define HARD_BUDGET_RECURSION 20000u
#define FAST_BUDGET_RECURSION 7000u
#define SQ_D1_TAIL_DFS_MAX_NODES 32u
#define COMPONENT_FAIL_CACHE_SIZE SOKOBAN_PARAM_COMPONENT_FAIL_CACHE_SIZE

#define CACHE_GET_IDX(hash, seed, size) (((uint32_t)(hash) * (uint32_t)(seed)) & ((uint32_t)(size) - 1u))
#define CACHE_GET_MIXED_IDX(hash, key, seed, size) (((uint32_t)(hash) ^ ((uint32_t)(key) * (uint32_t)(seed))) & ((uint32_t)(size) - 1u))

#define SOLVER_LARGE_BOX_COUNT_THRESHOLD 3
#define SOLVER_POST_OPT_MIN_PATH_LEN 60u
#define SOLVER_POST_OPT_RERUN_MAX_PATH_LEN 70u
#define SOLVER_POST_OPT_MAX_PASSES 15
#define SOKOBAN_SCAN_BOMB_ASSIGNMENT_BEAM_WIDTH 4u
#define SOKOBAN_SCAN_BOMB_BEAM_TAIL_RANK_WEIGHT 200u
#define SOKOBAN_SCAN_BOMB_BEAM_TAIL_LIMIT 2u
#define BOMB_MANEUVER_ROOT_ATTEMPT_LIMIT 4
#define BOMB_TOPOLOGY_WIDE_ATTEMPT_LIMIT 8
#define BOMB_POLICY_LOW_CONFIDENCE 50

static inline bool solver_is_root_append_entry(const SokobanSolver* solver, int depth) {
    return depth == 0 && solver &&
           !solver->is_scanning &&
           solver->scan_waypoint_count == 0 &&
           solver->scan_current_index == 0;
}
static inline bool solver_has_large_box_set(const SokobanSolver* solver) {
    return solver && solver->num_boxes > SOLVER_LARGE_BOX_COUNT_THRESHOLD;
}

static inline bool solver_can_try_bomb_post_opt(const SokobanSolver* solver, uint16_t path_len) {
    return solver &&
           path_len >= SOLVER_POST_OPT_MIN_PATH_LEN &&
           path_len < MAX_PATH_LENGTH &&
           solver->num_bombs > 0;
}

static inline uint16_t map_blocked_row(const BitboardMap* bmap, int y) {
    return (uint16_t)(bmap->walls[y] | bmap->boxes[y] | bmap->bombs[y]);
}

static inline bool map_is_obstructed(const BitboardMap* bmap, int x, int y) {
    return (map_blocked_row(bmap, y) & bit_mask_at(x)) != 0;
}

typedef struct {
    uint32_t key;
    uint16_t best_steps;
    uint8_t bombs_left;
    uint8_t valid;
} ComponentFailEntry;


static uint32_t g_solve_attempt_recursive_calls = 0;
static uint32_t g_solve_budget_limit = FAST_BUDGET_RECURSION;
static bool g_solve_budget_exhausted = false;
static bool g_solve_ever_budget_exhausted = false;
static ComponentFailEntry g_component_fail_cache[COMPONENT_FAIL_CACHE_SIZE] ALLOC_IN_OCRAM;


static inline void solve_budget_counters_reset(void) {
    g_solve_attempt_recursive_calls = 0;
    g_solve_budget_exhausted = false;
    g_solve_ever_budget_exhausted = false;
}

static inline void solve_attempt_budget_reset(void) {
    g_solve_attempt_recursive_calls = 0;
    g_solve_budget_exhausted = false;
}


static inline uint32_t solve_dynamic_budget(const SokobanSolver* solver) {
    (void)solver;
    uint32_t limit = g_solve_budget_limit;
    if (g_sandbox_mode && g_sandbox_budget_limit > 0 && g_sandbox_budget_limit < limit) {
        limit = g_sandbox_budget_limit;
    }
    return (limit > HARD_BUDGET_RECURSION) ? HARD_BUDGET_RECURSION : limit;
}

static inline bool solve_budget_enter(const SokobanSolver* solver) {
    uint32_t calls = ++g_solve_attempt_recursive_calls;
    if (calls > solve_dynamic_budget(solver)) {
        g_solve_budget_exhausted = true;
        g_solve_ever_budget_exhausted = true;
        return false;
    }
    return true;
}


#if MAP_COLS > 16 || MAP_ROWS > 16
#error "Z_IDX assumes MAP_COLS <= 16 and MAP_ROWS <= 16."
#endif
#define Z_IDX(x, y) ((((uint16_t)(y)) << 4) | (uint16_t)(x))

static uint32_t ZOBRIST_PLAYER[256];
static uint32_t ZOBRIST_BOX[256];
static uint32_t ZOBRIST_BOMB[256];
static uint32_t ZOBRIST_WALL[256];
static bool g_zobrist_inited = false;

static uint16_t g_destructible_mask[MAP_ROWS] ALLOC_IN_OCRAM;
#define TOPO_COMPONENT_NONE 0xFFu
#define TOPO_MAX_COMPONENTS 96u

typedef struct {
    uint8_t component_id[MAP_ROWS][MAP_COLS];
    uint8_t component_count;
    uint8_t player_component;
    uint8_t component_cells[TOPO_MAX_COMPONENTS];
    uint8_t component_targets[TOPO_MAX_COMPONENTS];
    uint8_t component_boxes[TOPO_MAX_COMPONENTS];
    uint16_t corridor_mask[MAP_ROWS];
    uint16_t articulation_mask[MAP_ROWS];
    uint16_t destructible_bridge_mask[MAP_ROWS];
    uint16_t open_cells;
    uint16_t corridor_cells;
    uint16_t articulation_cells;
    uint16_t destructible_bridge_walls;
    uint16_t target_cluster_span;
    uint8_t target_component_count;
    uint8_t box_component_count;
    uint8_t bomb_bridge_count;
    uint8_t bomb_bottleneck_hits[MAX_BOMBS];
    uint8_t bomb_open_gain[MAX_BOMBS];
} TopologyFeatures;

static TopologyFeatures g_topology_features;
static bool g_topology_split_visited[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static Position g_topology_queue[MAP_ROWS * MAP_COLS] ALLOC_IN_OCRAM;
static bool g_topology_features_valid = false;


#define TT_STATUS_EMPTY     0
#define TT_STATUS_DEADEND   1
#define TT_STATUS_SEARCHING 2
#define TT_STATUS_SOLVED    3

#define TT_DOMAIN_UNIVERSE  0
#define TT_DOMAIN_DFS       1

typedef struct {
    uint32_t hash_key;
    uint16_t best_steps;
    uint8_t  bombs_left;
    uint8_t  status;
} __attribute__((packed)) HashEntry;

typedef struct {
    uint32_t state_hash;
    uint16_t reach_key;
    uint16_t reserved;
} BombReachFailEntry;

#define BOMB_REACH_FAIL_CACHE_SIZE SOKOBAN_BOMB_REACH_FAIL_CACHE_CAPACITY
static BombReachFailEntry g_bomb_reach_fail_cache[BOMB_REACH_FAIL_CACHE_SIZE] ALLOC_IN_OCRAM;

static inline uint16_t bomb_reach_fail_key(Position bomb, Position wall) {
    return (uint16_t)((bomb.x & 0x0F) |
                      ((bomb.y & 0x0F) << 4) |
                      ((wall.x & 0x0F) << 8) |
                      ((wall.y & 0x0F) << 12));
}

#define BOMB_REACH_FAIL_CTX_SCAN   0x0001u
#define BOMB_REACH_FAIL_CTX_STRICT 0x0002u

static inline uint16_t bomb_reach_fail_context(const SokobanSolver* solver) {
    uint16_t ctx = 0;
    if (solver && solver->is_scanning) ctx |= BOMB_REACH_FAIL_CTX_SCAN;
    if (solver && solver->strict_target_mode) ctx |= BOMB_REACH_FAIL_CTX_STRICT;
    return ctx;
}

static inline bool bomb_reach_fail_cached(uint32_t state_hash, uint16_t reach_key, uint16_t context) {
    uint32_t idx = CACHE_GET_MIXED_IDX(state_hash, reach_key, 2654435761u, BOMB_REACH_FAIL_CACHE_SIZE);
    BombReachFailEntry e = g_bomb_reach_fail_cache[idx];
    return e.state_hash == state_hash && e.reach_key == reach_key && e.reserved == context;
}

static inline void bomb_reach_fail_store(uint32_t state_hash, uint16_t reach_key, uint16_t context) {
    uint32_t idx = CACHE_GET_MIXED_IDX(state_hash, reach_key, 2654435761u, BOMB_REACH_FAIL_CACHE_SIZE);
    g_bomb_reach_fail_cache[idx] = (BombReachFailEntry){state_hash, reach_key, context};
}

static inline void bomb_reach_caches_reset(void) {
    memset(g_bomb_reach_fail_cache, 0, sizeof(g_bomb_reach_fail_cache));
}
#define L1_DFS_HASH_SIZE SOKOBAN_PARAM_L1_HASH_SIZE
#define L1_UNIVERSE_HASH_SIZE SOKOBAN_PARAM_L1_UNIVERSE_HASH_SIZE

static HashEntry g_l1_hash_dfs[L1_DFS_HASH_SIZE][2] ALLOC_IN_OCRAM;
static HashEntry g_l1_hash_universe[L1_UNIVERSE_HASH_SIZE][2] ALLOC_IN_OCRAM;


static inline void clear_transposition_table(void) {
    memset(g_l1_hash_dfs, 0, sizeof(g_l1_hash_dfs));
    memset(g_l1_hash_universe, 0, sizeof(g_l1_hash_universe));
}

static inline void clear_transposition_table_domain(int domain) {
    if (domain == TT_DOMAIN_DFS) {
        memset(g_l1_hash_dfs, 0, sizeof(g_l1_hash_dfs));
    } else {
        memset(g_l1_hash_universe, 0, sizeof(g_l1_hash_universe));
    }
}

static inline HashEntry (*l1_hash_slots_for_domain(int domain, uint32_t hash))[2] {
    if (domain == TT_DOMAIN_DFS) {
        return &g_l1_hash_dfs[hash & (L1_DFS_HASH_SIZE - 1u)];
    }
    return &g_l1_hash_universe[hash & (L1_UNIVERSE_HASH_SIZE - 1u)];
}

static int lookup_transposition_table(int domain, uint32_t hash, uint16_t metric, uint8_t bombs) {
    HashEntry (*slots)[2] = l1_hash_slots_for_domain(domain, hash);
    HashEntry e0 = (*slots)[0];
    HashEntry e1 = (*slots)[1];

    if (e0.hash_key == hash && e0.bombs_left == bombs) {
        if (domain == TT_DOMAIN_DFS && e0.best_steps <= metric) {
            return e0.status;
        }
        if (domain == TT_DOMAIN_UNIVERSE && e0.best_steps >= metric) {
            return e0.status;
        }
    }

    if (e1.hash_key == hash && e1.bombs_left == bombs) {
        if (domain == TT_DOMAIN_DFS && e1.best_steps <= metric) {
            return e1.status;
        }
        if (domain == TT_DOMAIN_UNIVERSE && e1.best_steps >= metric) {
            return e1.status;
        }
    }

    return TT_STATUS_EMPTY;
}

static inline bool universe_tt_prefers_new(uint8_t status, uint16_t steps, const HashEntry* old) {
    if (old->status == TT_STATUS_EMPTY) return true;

    if (status == TT_STATUS_DEADEND || status == TT_STATUS_SEARCHING) {
        if (old->status == TT_STATUS_SOLVED) return true;
        return steps > old->best_steps;
    }

    if (status == TT_STATUS_SOLVED) {
        return old->status == TT_STATUS_SOLVED && steps < old->best_steps;
    }

    return false;
}

static void store_transposition_table(int domain, uint32_t hash, uint16_t steps, uint8_t bombs, uint8_t status) {
    HashEntry (*slots)[2] = l1_hash_slots_for_domain(domain, hash);
    HashEntry* slot0 = &(*slots)[0];
    HashEntry* slot1 = &(*slots)[1];
    HashEntry entry = (HashEntry){hash, steps, bombs, status};

    if (domain == TT_DOMAIN_UNIVERSE) {
        if (slot0->hash_key == hash && slot0->bombs_left == bombs) {
            *slot0 = entry;
            return;
        }
        if (slot1->hash_key == hash && slot1->bombs_left == bombs) {
            *slot1 = entry;
            return;
        }

        if (universe_tt_prefers_new(status, steps, slot0)) {
            *slot0 = entry;
        } else if (universe_tt_prefers_new(status, steps, slot1)) {
            *slot1 = entry;
        }
        return;
    }

    if (slot0->hash_key == hash && slot0->bombs_left == bombs) {
        if (slot0->status == TT_STATUS_EMPTY || steps < slot0->best_steps) *slot0 = entry;
        return;
    }
    if (slot1->hash_key == hash && slot1->bombs_left == bombs) {
        if (slot1->status == TT_STATUS_EMPTY || steps < slot1->best_steps) *slot1 = entry;
        return;
    }

    if (slot0->status == TT_STATUS_EMPTY) {
        *slot0 = entry;
    } else if (slot1->status == TT_STATUS_EMPTY) {
        *slot1 = entry;
    } else if (slot0->best_steps >= slot1->best_steps) {
        if (steps < slot0->best_steps) *slot0 = entry;
    } else {
        if (steps < slot1->best_steps) *slot1 = entry;
    }
}
#define MATCH_HEURISTIC_INF 0x3FFFu
#define MATCH_HEURISTIC_STATE_COUNT (1u << MAX_BOXES)
static uint16_t g_match_heuristic_dp[MATCH_HEURISTIC_STATE_COUNT] ALLOC_IN_OCRAM;
static uint16_t g_match_heuristic_next[MATCH_HEURISTIC_STATE_COUNT] ALLOC_IN_OCRAM;

static inline bool heuristic_target_matches_box(const SokobanSolver* solver, int box_idx, int target_idx) {
    return !solver->strict_target_mode ||
           solver->targets[target_idx].id == -1 ||
           solver->boxes[box_idx].id == -1 ||
           solver->targets[target_idx].id == solver->boxes[box_idx].id;
}

static inline uint16_t heuristic_pair_cost(const SokobanSolver* solver, Position box, uint8_t box_idx, Position target, uint8_t target_idx) {
    if (!heuristic_target_matches_box(solver, box_idx, target_idx)) {
        return MATCH_HEURISTIC_INF;
    }
    if (g_use_target_dist_heuristic && target_idx < MAX_TARGETS) {
        uint16_t dist = g_target_dist_field[target_idx][box.y][box.x];
        return (dist == 0xFFFF) ? MATCH_HEURISTIC_INF : dist;
    }
    return manhattan_distance(box, target);
}

static inline uint16_t heuristic_add_cost(uint16_t a, uint16_t b) {
    if (a >= MATCH_HEURISTIC_INF || b >= MATCH_HEURISTIC_INF) return MATCH_HEURISTIC_INF;
    uint32_t sum = (uint32_t)a + b;
    return (sum >= MATCH_HEURISTIC_INF) ? MATCH_HEURISTIC_INF : (uint16_t)sum;
}

static inline uint16_t normalize_matching_heuristic(uint16_t value) {
    return (value >= MATCH_HEURISTIC_INF) ? 0xFFFF : value;
}

FAST_OCRAM_FUNC static uint16_t __attribute__((noinline)) compute_perfect_remaining_heuristic(
    const SokobanSolver* solver,
    Position* boxes,
    uint8_t* box_indices,
    Position* targets,
    uint8_t* target_indices,
    int num
) {
    if (num == 0) return 0;
    if (num < 0 || num > MAX_BOXES || num > MAX_TARGETS || num > 10) return 0xFFFF;

    const uint16_t full_mask = (uint16_t)((1u << num) - 1u);
    uint16_t* dp = g_match_heuristic_dp;
    uint16_t* next = g_match_heuristic_next;
    for (uint16_t mask = 0; mask <= full_mask; mask++) dp[mask] = MATCH_HEURISTIC_INF;
    dp[0] = 0;

    for (int box_i = 0; box_i < num; box_i++) {
        for (uint16_t mask = 0; mask <= full_mask; mask++) next[mask] = MATCH_HEURISTIC_INF;

        for (uint16_t mask = 0; mask <= full_mask; mask++) {
            uint16_t base = dp[mask];
            if (base >= MATCH_HEURISTIC_INF) continue;

            for (int target_i = 0; target_i < num; target_i++) {
                uint16_t bit = (uint16_t)(1u << target_i);
                if ((mask & bit) != 0) continue;

                uint16_t pair = heuristic_pair_cost(
                    solver,
                    boxes[box_i],
                    box_indices[box_i],
                    targets[target_i],
                    target_indices[target_i]
                );
                uint16_t candidate = heuristic_add_cost(base, pair);
                uint16_t new_mask = (uint16_t)(mask | bit);
                if (candidate < next[new_mask]) next[new_mask] = candidate;
            }
        }

        for (uint16_t mask = 0; mask <= full_mask; mask++) dp[mask] = next[mask];
    }

    return normalize_matching_heuristic(dp[full_mask]);
}

static bool collect_remaining_box_target_pairs(
    const SokobanSolver* solver,
    Position* rem_boxes,
    uint8_t* rem_box_indices,
    Position* rem_targets,
    uint8_t* rem_target_indices,
    int* out_num_boxes,
    int* out_num_targets,
    Position player,
    bool collect_player_distance,
    uint16_t* out_min_player_to_box
) {
    bool target_used[MAX_TARGETS] = {false};
    int num_boxes = 0;
    int num_targets = 0;
    uint16_t min_player_to_box = 0xFFFF;

    if (!solver || !rem_boxes || !rem_box_indices || !rem_targets || !rem_target_indices ||
        !out_num_boxes || !out_num_targets || !out_min_player_to_box) {
        return false;
    }

    for (int i = 0; i < solver->num_boxes; i++) {
        Position box = solver->boxes[i].pos;
        bool locked_on_target = false;

        for (int t = 0; t < solver->num_targets; t++) {
            if (!target_used[t] && pos_equal(solver->targets[t].pos, box) && heuristic_target_matches_box(solver, i, t)) {
                target_used[t] = true;
                locked_on_target = true;
                break;
            }
        }

        if (locked_on_target) continue;

        if (collect_player_distance) {
            uint16_t p_dist = manhattan_distance(player, box);
            if (p_dist < min_player_to_box) min_player_to_box = p_dist;
        }

        rem_boxes[num_boxes] = box;
        rem_box_indices[num_boxes] = (uint8_t)i;
        num_boxes++;
    }

    for (int t = 0; t < solver->num_targets; t++) {
        if (!target_used[t]) {
            rem_targets[num_targets] = solver->targets[t].pos;
            rem_target_indices[num_targets] = (uint8_t)t;
            num_targets++;
        }
    }

    *out_num_boxes = num_boxes;
    *out_num_targets = num_targets;
    *out_min_player_to_box = min_player_to_box;
    return true;
}

static uint16_t remaining_box_assignment_lower_bound(const SokobanSolver* solver) {
    Position rem_boxes[MAX_BOXES];
    Position rem_targets[MAX_TARGETS];
    uint8_t rem_box_indices[MAX_BOXES];
    uint8_t rem_target_indices[MAX_TARGETS];
    int num_boxes = 0;
    int num_targets = 0;
    uint16_t ignored_player_dist = 0xFFFF;

    if (!collect_remaining_box_target_pairs(
            solver,
            rem_boxes,
            rem_box_indices,
            rem_targets,
            rem_target_indices,
            &num_boxes,
            &num_targets,
            (Position){0, 0},
            false,
            &ignored_player_dist)) {
        return 0xFFFF;
    }

    if (num_boxes != num_targets) return 0xFFFF;
    return compute_perfect_remaining_heuristic(
        solver,
        rem_boxes,
        rem_box_indices,
        rem_targets,
        rem_target_indices,
        num_boxes
    );
}
static uint16_t remaining_solution_lower_bound(const SokobanSolver* solver, Position player) {
    Position rem_boxes[MAX_BOXES];
    Position rem_targets[MAX_TARGETS];
    uint8_t rem_box_indices[MAX_BOXES];
    uint8_t rem_target_indices[MAX_TARGETS];
    int num_boxes = 0;
    int num_targets = 0;
    uint16_t min_player_to_box = 0xFFFF;

    if (!collect_remaining_box_target_pairs(
            solver,
            rem_boxes,
            rem_box_indices,
            rem_targets,
            rem_target_indices,
            &num_boxes,
            &num_targets,
            player,
            true,
            &min_player_to_box)) {
        return 0xFFFF;
    }

    if (num_boxes != num_targets) return 0xFFFF;
    if (num_boxes == 0) return 0;

    uint16_t lower = compute_perfect_remaining_heuristic(
        solver,
        rem_boxes,
        rem_box_indices,
        rem_targets,
        rem_target_indices,
        num_boxes
    );
    if (lower == 0xFFFF) return 0xFFFF;

    if (min_player_to_box != 0xFFFF && min_player_to_box > 0) {
        lower = (uint16_t)(lower + min_player_to_box - 1);
    }
    return lower;
}
static uint32_t xorshift32(uint32_t* state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return *state = x;
}

static void init_zobrist(void) {
    if (g_zobrist_inited) return;
    uint32_t seed = 20260425u;

    for (int i = 0; i < 256; i++) {
        ZOBRIST_PLAYER[i] = xorshift32(&seed);
        ZOBRIST_BOX[i]    = xorshift32(&seed);
        ZOBRIST_BOMB[i]   = xorshift32(&seed);
        ZOBRIST_WALL[i]   = xorshift32(&seed);
    }
    g_zobrist_inited = true;
}

static void init_destructible_walls(const BitboardMap* bmap) {
    memset(g_destructible_mask, 0, sizeof(g_destructible_mask));
    for (int y = 1; y < MAP_ROWS - 1; y++) {
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if ((bmap->walls[y] & (1 << x)) != 0) {
                g_destructible_mask[y] |= (uint16_t)(1u << x);
            }
        }
    }
}


static void compute_static_deadlocks(BitboardMap* bmap) {
    memset(bmap->deadlocks, 0, sizeof(bmap->deadlocks));
    for (int y = 1; y < MAP_ROWS - 1; y++) {
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if ((bmap->walls[y] & (1 << x)) != 0 || (bmap->targets[y] & (1 << x)) != 0) continue;

            bool wall_up   = (bmap->walls[y - 1] & (1 << x)) != 0;
            bool wall_down = (bmap->walls[y + 1] & (1 << x)) != 0;
            bool wall_left = (bmap->walls[y] & (1 << (x - 1))) != 0;
            bool wall_right= (bmap->walls[y] & (1 << (x + 1))) != 0;

            if ((wall_up || wall_down) && (wall_left || wall_right)) {
                bmap->deadlocks[y] |= (1 << x);
            }
        }
    }
    sokoban_refresh_dynamic_tunnels(bmap);
}




static inline bool topology_is_open_cell(const BitboardMap* bmap, int x, int y) {
    return is_in_bounds(x, y) && ((bmap->walls[y] & bit_mask_at(x)) == 0);
}

static int topology_open_degree(const BitboardMap* bmap, int x, int y) {
    int degree = 0;
    for (int d = 0; d < 4; d++) {
        int nx = x + DIRECTIONS[d].dx;
        int ny = y + DIRECTIONS[d].dy;
        if (topology_is_open_cell(bmap, nx, ny)) degree++;
    }
    return degree;
}

static void topology_flood_component(
    const BitboardMap* bmap,
    TopologyFeatures* topo,
    int sx, int sy,
    uint8_t comp_id
) {
    Position* q = g_topology_queue;
    int head = 0, tail = 0;

    topo->component_id[sy][sx] = comp_id;
    q[tail++] = (Position){(uint8_t)sx, (uint8_t)sy};

    while (head < tail) {
        Position curr = q[head++];
        if (topo->component_cells[comp_id] < 0xFFu) topo->component_cells[comp_id]++;

        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (!topology_is_open_cell(bmap, nx, ny)) continue;
            if (topo->component_id[ny][nx] != TOPO_COMPONENT_NONE) continue;
            topo->component_id[ny][nx] = comp_id;
            q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }
}

static void topology_mark_components(const SokobanSolver* solver, TopologyFeatures* topo) {
    const BitboardMap* bmap = &solver->bmap;
    memset(topo->component_id, TOPO_COMPONENT_NONE, sizeof(topo->component_id));
    topo->component_count = 0;
    topo->open_cells = 0;

    for (int y = 0; y < MAP_ROWS; y++) {
        for (int x = 0; x < MAP_COLS; x++) {
            if (!topology_is_open_cell(bmap, x, y)) continue;
            topo->open_cells++;
            if (topo->component_id[y][x] != TOPO_COMPONENT_NONE) continue;
            if (topo->component_count >= TOPO_MAX_COMPONENTS) continue;
            topology_flood_component(bmap, topo, x, y, topo->component_count);
            topo->component_count++;
        }
    }

    topo->player_component = TOPO_COMPONENT_NONE;
    if (is_in_bounds(solver->start_player.x, solver->start_player.y)) {
        topo->player_component = topo->component_id[solver->start_player.y][solver->start_player.x];
    }
}

static void topology_count_entities(const SokobanSolver* solver, TopologyFeatures* topo) {
    bool target_component_seen[TOPO_MAX_COMPONENTS] = {0};
    bool box_component_seen[TOPO_MAX_COMPONENTS] = {0};
    int min_tx = MAP_COLS, min_ty = MAP_ROWS, max_tx = -1, max_ty = -1;

    for (int i = 0; i < solver->num_targets; i++) {
        if (!solver->targets[i].is_active) continue;
        Position p = solver->targets[i].pos;
        if (!is_in_bounds(p.x, p.y)) continue;
        uint8_t comp = topo->component_id[p.y][p.x];
        if (comp != TOPO_COMPONENT_NONE && comp < topo->component_count) {
            if (topo->component_targets[comp] < 0xFFu) topo->component_targets[comp]++;
            target_component_seen[comp] = true;
        }
        if (p.x < min_tx) min_tx = p.x;
        if (p.x > max_tx) max_tx = p.x;
        if (p.y < min_ty) min_ty = p.y;
        if (p.y > max_ty) max_ty = p.y;
    }

    for (int i = 0; i < solver->num_boxes; i++) {
        if (!solver->boxes[i].is_active) continue;
        Position p = solver->boxes[i].pos;
        if (!is_in_bounds(p.x, p.y)) continue;
        uint8_t comp = topo->component_id[p.y][p.x];
        if (comp != TOPO_COMPONENT_NONE && comp < topo->component_count) {
            if (topo->component_boxes[comp] < 0xFFu) topo->component_boxes[comp]++;
            box_component_seen[comp] = true;
        }
    }

    topo->target_component_count = 0;
    topo->box_component_count = 0;
    for (int i = 0; i < topo->component_count; i++) {
        if (target_component_seen[i]) topo->target_component_count++;
        if (box_component_seen[i]) topo->box_component_count++;
    }

    topo->target_cluster_span = 0;
    if (max_tx >= 0) {
        topo->target_cluster_span = (uint16_t)((max_tx - min_tx) + (max_ty - min_ty));
    }
}

static void topology_mark_corridors(const BitboardMap* bmap, TopologyFeatures* topo) {
    memset(topo->corridor_mask, 0, sizeof(topo->corridor_mask));
    topo->corridor_cells = 0;

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if (!topology_is_open_cell(bmap, x, y)) continue;
            bool wall_up = !topology_is_open_cell(bmap, x, y - 1);
            bool wall_down = !topology_is_open_cell(bmap, x, y + 1);
            bool wall_left = !topology_is_open_cell(bmap, x - 1, y);
            bool wall_right = !topology_is_open_cell(bmap, x + 1, y);
            int degree = topology_open_degree(bmap, x, y);
            if (degree <= 2 || (wall_up && wall_down) || (wall_left && wall_right)) {
                topo->corridor_mask[y] |= bit_mask_at(x);
                topo->corridor_cells++;
            }
        }
    }
}

static bool topology_neighbors_split_without_cell(const BitboardMap* bmap, int x, int y) {
    Position neighbors[4];
    int neighbor_count = 0;
    bool (*visited)[MAP_COLS] = g_topology_split_visited;
    memset(g_topology_split_visited, 0, sizeof(g_topology_split_visited));
    Position* q = g_topology_queue;
    int head = 0, tail = 0;

    for (int d = 0; d < 4; d++) {
        int nx = x + DIRECTIONS[d].dx;
        int ny = y + DIRECTIONS[d].dy;
        if (topology_is_open_cell(bmap, nx, ny)) {
            neighbors[neighbor_count++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }
    if (neighbor_count < 2) return false;

    visited[neighbors[0].y][neighbors[0].x] = true;
    q[tail++] = neighbors[0];
    while (head < tail) {
        Position curr = q[head++];
        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (nx == x && ny == y) continue;
            if (!topology_is_open_cell(bmap, nx, ny)) continue;
            if (visited[ny][nx]) continue;
            visited[ny][nx] = true;
            q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }

    for (int i = 1; i < neighbor_count; i++) {
        if (!visited[neighbors[i].y][neighbors[i].x]) return true;
    }
    return false;
}

static void topology_mark_articulations(const BitboardMap* bmap, TopologyFeatures* topo) {
    memset(topo->articulation_mask, 0, sizeof(topo->articulation_mask));
    topo->articulation_cells = 0;

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if (!topology_is_open_cell(bmap, x, y)) continue;
            if (topology_open_degree(bmap, x, y) < 2) continue;
            if (topology_neighbors_split_without_cell(bmap, x, y)) {
                topo->articulation_mask[y] |= bit_mask_at(x);
                topo->articulation_cells++;
            }
        }
    }
}

static void topology_mark_destructible_bridges(const BitboardMap* bmap, TopologyFeatures* topo) {
    memset(topo->destructible_bridge_mask, 0, sizeof(topo->destructible_bridge_mask));
    topo->destructible_bridge_walls = 0;

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        uint16_t row = bmap->walls[y] & g_destructible_mask[y];
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if ((row & bit_mask_at(x)) == 0) continue;
            uint8_t comps[4];
            int comp_count = 0;
            for (int d = 0; d < 4; d++) {
                int nx = x + DIRECTIONS[d].dx;
                int ny = y + DIRECTIONS[d].dy;
                if (!topology_is_open_cell(bmap, nx, ny)) continue;
                uint8_t comp = topo->component_id[ny][nx];
                if (comp == TOPO_COMPONENT_NONE) continue;
                bool seen = false;
                for (int i = 0; i < comp_count; i++) {
                    if (comps[i] == comp) { seen = true; break; }
                }
                if (!seen) comps[comp_count++] = comp;
            }
            if (comp_count >= 2) {
                topo->destructible_bridge_mask[y] |= bit_mask_at(x);
                topo->destructible_bridge_walls++;
            }
        }
    }
}

static void topology_score_bombs(const SokobanSolver* solver, TopologyFeatures* topo) {
    topo->bomb_bridge_count = 0;
    memset(topo->bomb_bottleneck_hits, 0, sizeof(topo->bomb_bottleneck_hits));
    memset(topo->bomb_open_gain, 0, sizeof(topo->bomb_open_gain));

    for (int i = 0; i < solver->num_bombs; i++) {
        if (!solver->bombs[i].is_active) continue;
        Position b = solver->bombs[i].pos;
        uint8_t open_gain = 0;
        uint8_t bridge_hits = 0;
        for (int y = b.y - 1; y <= b.y + 1; y++) {
            if (y <= 0 || y >= MAP_ROWS - 1) continue;
            for (int x = b.x - 1; x <= b.x + 1; x++) {
                if (x <= 0 || x >= MAP_COLS - 1) continue;
                uint16_t bit = bit_mask_at(x);
                if ((solver->bmap.walls[y] & bit) == 0 || (g_destructible_mask[y] & bit) == 0) continue;
                if (open_gain < 0xFFu) open_gain++;
                if ((topo->destructible_bridge_mask[y] & bit) != 0 && bridge_hits < 0xFFu) bridge_hits++;
            }
        }
        topo->bomb_open_gain[i] = open_gain;
        topo->bomb_bottleneck_hits[i] = bridge_hits;
        if (bridge_hits > 0) topo->bomb_bridge_count++;
    }
}

static void topology_extract(const SokobanSolver* solver) {
    if (!solver) return;

    TopologyFeatures* topo = &g_topology_features;
    memset(topo, 0, sizeof(*topo));
    topology_mark_components(solver, topo);
    topology_count_entities(solver, topo);
    topology_mark_corridors(&solver->bmap, topo);
    topology_mark_articulations(&solver->bmap, topo);
    topology_mark_destructible_bridges(&solver->bmap, topo);
    topology_score_bombs(solver, topo);
    g_topology_features_valid = true;
}

static inline bool topology_profile_may_use_soft_order(const SokobanSolver* solver) {
    return solver_has_large_box_set(solver);
}

static bool topology_profile_allows_soft_order(const SokobanSolver* solver) {
    if (!solver || solver->is_scanning || !topology_profile_may_use_soft_order(solver)) return false;
    if (!g_topology_features_valid) return false;

    const TopologyFeatures* topo = &g_topology_features;
    return topo->target_component_count > 1 &&
           topo->box_component_count == 1 &&
           topo->destructible_bridge_walls > 0 &&
           topo->bomb_bridge_count > 0;
}
static inline void rebuild_active_boxes_layer(
    BitboardMap* dest_map,
    const Position* current_boxes,
    const bool* box_assigned,
    int num_boxes
) {
    memset(dest_map->boxes, 0, sizeof(dest_map->boxes));
    for (int k = 0; k < num_boxes; k++) {
        if (!box_assigned[k]) {
            set_box_bit(dest_map, current_boxes[k].x, current_boxes[k].y);
        }
    }
}

#define MAX_BOMB_OPTIONS SOKOBAN_PARAM_MAX_BOMB_OPTIONS
#define MAX_DETONATION_POINTS 300

static Direction g_simple_path_pool[MAX_BOMBS][MAX_PATH_LENGTH] ALLOC_IN_OCRAM;
static Position g_target_walls_pool[MAX_BOMB_OPTIONS];
static Direction g_bomb_path_pool[MAX_BOMBS][MAX_SINGLE_PATH];

#define MAX_BOMB_CANDIDATES SOKOBAN_PARAM_MAX_BOMB_CANDIDATES
#define MAX_BASE_BOMB_ATTEMPTS 15

#define MAX_RESCUE_BOMB_ATTEMPTS 3
#define MAX_PROXIMITY_BOMB_ATTEMPTS 2
#define MAX_STRUCT_BOMB_ATTEMPTS 6
#define MAX_REACH_ALL_PREFETCH MAX_BOMB_CANDIDATES
#define BOMB_INHERIT_SOFT_MARGIN 4
#define SOKOBAN_SCAN_INHERIT_SOFT_MARGIN 0

#define MAX_ROOT_LOW_LB_APPEND 3
#define MAX_WIDE_ROOT_LOW_LB_APPEND 1



#define BOMB_TOPO_BRIDGE_BONUS 250
#define BOMB_TOPO_OPPOSITE_BRIDGE_BONUS 150
#define BOMB_TOPO_OPEN_DEGREE_BONUS 50
#define BOMB_TOPO_BRIDGE_BONUS_CAP 700
#define MAX_LIGHT_EVAC_DEPTH 1
#define LIGHT_EVAC_TOP_WINDOW 10
#define LIGHT_EVAC_MAX_PUSHES 2
#define MAX_LIGHT_EVAC_PLANS SOKOBAN_PARAM_MAX_LIGHT_EVAC_PLANS
#define LIGHT_EVAC_ROUTE_STEP_LIMIT 64
#define SCAN_LIGHT_EVAC_SKIP_MARGIN 4
#define SOKOBAN_SCAN_MACRO_UNIVERSE_BUDGET 800u



#define PACKED_PATH_BYTES(step_count) (((step_count) + 3u) / 4u)
typedef uint8_t PackedDirByte;

static inline uint8_t solver_direction_code(Direction dir) {
    uint8_t idx = direction_index(dir);
    return (idx < 4u) ? idx : 0u;
}

static inline Direction solver_direction_from_code(uint8_t code) {
    return direction_from_index(code);
}

static void packed_path_clear(PackedDirByte* packed) {
    if (!packed) return;
    memset(packed, 0, PACKED_PATH_BYTES(MAX_SINGLE_PATH));
}

static void packed_path_store_from_direction(PackedDirByte* packed, const Direction* path, uint16_t len) {
    if (!packed || !path || len > MAX_SINGLE_PATH) return;
    packed_path_clear(packed);
    for (uint16_t i = 0; i < len; i++) {
        uint8_t code = solver_direction_code(path[i]);
        uint16_t byte_idx = (uint16_t)(i >> 2);
        uint8_t shift = (uint8_t)((i & 0x03u) << 1);
        packed[byte_idx] = (uint8_t)((packed[byte_idx] & (uint8_t)~(0x03u << shift)) | (uint8_t)(code << shift));
    }
}

static void packed_path_load_to_direction(const PackedDirByte* packed, Direction* path, uint16_t len) {
    if (!packed || !path || len > MAX_SINGLE_PATH) return;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t byte_idx = (uint16_t)(i >> 2);
        uint8_t shift = (uint8_t)((i & 0x03u) << 1);
        path[i] = solver_direction_from_code((uint8_t)((packed[byte_idx] >> shift) & 0x03u));
    }
}

static Direction packed_path_step(const PackedDirByte* packed, uint16_t step) {
    uint16_t byte_idx = (uint16_t)(step >> 2);
    uint8_t shift = (uint8_t)((step & 0x03u) << 1);
    return solver_direction_from_code((uint8_t)((packed[byte_idx] >> shift) & 0x03u));
}

static PackedDirByte g_bomb_reach_all_path_pool[MAX_BOMBS][MAX_BOMB_CANDIDATES][PACKED_PATH_BYTES(MAX_SINGLE_PATH)] ALLOC_IN_OCRAM;
static uint16_t g_bomb_reach_all_len_pool[MAX_BOMBS][MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;

#define BOMB_CAND_FLAG_MANEUVER_EVALUATED 0x01u
#define BOMB_CAND_FLAG_TRIED              0x02u
#define BOMB_CAND_FLAG_ROUTE_PRECOMPUTED  0x04u

typedef struct {
    uint8_t b_idx;
    Position wall_pos;
    int16_t score;
    int16_t topology_score;
    int16_t applied_top_bonus;
    int16_t applied_short_bonus;
    int16_t applied_maneuver_bonus;
    uint16_t path_lower_bound;
    uint16_t route_len;
    uint8_t topology_bucket;
    uint8_t flags;
    uint8_t route_slot;
} BombWallCandidate;
typedef char BombWallCandidate_size_must_fit_budget[(sizeof(BombWallCandidate) <= 24u) ? 1 : -1];

static inline int16_t bomb_candidate_i16(int value) {
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static inline bool bomb_candidate_flag(const BombWallCandidate* cand, uint8_t flag) {
    return cand && ((cand->flags & flag) != 0);
}

static inline void bomb_candidate_set_flag(BombWallCandidate* cand, uint8_t flag, bool enabled) {
    if (!cand) return;
    if (enabled) cand->flags = (uint8_t)(cand->flags | flag);
    else cand->flags = (uint8_t)(cand->flags & (uint8_t)~flag);
}

static inline bool bomb_candidate_maneuver_evaluated(const BombWallCandidate* cand) {
    return bomb_candidate_flag(cand, BOMB_CAND_FLAG_MANEUVER_EVALUATED);
}

static inline bool bomb_candidate_tried(const BombWallCandidate* cand) {
    return bomb_candidate_flag(cand, BOMB_CAND_FLAG_TRIED);
}

static inline bool bomb_candidate_route_precomputed(const BombWallCandidate* cand) {
    return bomb_candidate_flag(cand, BOMB_CAND_FLAG_ROUTE_PRECOMPUTED);
}

static inline void bomb_candidate_set_maneuver_evaluated(BombWallCandidate* cand, bool enabled) {
    bomb_candidate_set_flag(cand, BOMB_CAND_FLAG_MANEUVER_EVALUATED, enabled);
}

static inline void bomb_candidate_set_tried(BombWallCandidate* cand, bool enabled) {
    bomb_candidate_set_flag(cand, BOMB_CAND_FLAG_TRIED, enabled);
}

static inline void bomb_candidate_set_route_precomputed(BombWallCandidate* cand, bool enabled) {
    bomb_candidate_set_flag(cand, BOMB_CAND_FLAG_ROUTE_PRECOMPUTED, enabled);
}

static inline bool bomb_candidate_pruned_by_local_best(const BombWallCandidate* cand,
                                                       uint16_t assignment_lower,
                                                       uint16_t local_best) {
    if (!cand || local_best == 0xFFFFu || cand->path_lower_bound == 0xFFFFu) return false;
    if (cand->path_lower_bound >= local_best) return true;
    return assignment_lower != 0xFFFFu &&
           (uint32_t)cand->path_lower_bound + assignment_lower >= local_best;
}

static inline uint16_t bomb_candidate_branch_attempt_key(const BombWallCandidate* cand) {
    if (!cand) return 0xFFFFu;
    return (uint16_t)(((uint16_t)cand->b_idx & 0x0Fu) |
                      (((uint16_t)cand->wall_pos.x & 0x0Fu) << 4) |
                      (((uint16_t)cand->wall_pos.y & 0x0Fu) << 8));
}

static bool bomb_strategy_branch_attempt_seen(const uint16_t* keys, int count, const BombWallCandidate* cand) {
    uint16_t key = bomb_candidate_branch_attempt_key(cand);
    for (int i = 0; i < count; i++) {
        if (keys[i] == key) return true;
    }
    return false;
}

static void bomb_strategy_branch_attempt_remember(uint16_t* keys, int* count, const BombWallCandidate* cand) {
    if (!keys || !count || *count >= MAX_BOMB_CANDIDATES) return;
    if (bomb_strategy_branch_attempt_seen(keys, *count, cand)) return;
    keys[*count] = bomb_candidate_branch_attempt_key(cand);
    (*count)++;
}

typedef struct {
    uint8_t box_idx;
    Position old_box;
    Position target_parking;
    Position player_after;
    Direction evac_path[MAX_SINGLE_PATH];
    uint16_t evac_len;
    uint8_t push_count;
    int score;
} LightEvacPlan;

static inline bool light_evac_context_allowed(const SokobanSolver* solver) {
    if (!solver) return false;
    if (solver->scan_waypoint_count != 0 || solver->scan_current_index != 0) return false;
    if (!solver->is_scanning) return true;
    return !solver->strict_target_mode;
}

static bool light_evac_direct_line_blocked_by_single_box(const SokobanSolver* solver, Position bomb, Position wall, uint16_t* push_hint);
static int g_bomb_ghost_topology_bonus[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static int g_bomb_ghost_shortcut_bonus[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_dist_p[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_dist_b[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_dist_t[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_player_nav_dist[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_bfs_dist[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_box_puddle_walls[MAP_ROWS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_puddle_core_walls[MAP_ROWS] ALLOC_IN_OCRAM;
static uint16_t g_bomb_candidate_wall_mask[MAP_ROWS] ALLOC_IN_OCRAM;
static Position g_bomb_candidate_wall_cells[MAP_ROWS * MAP_COLS] ALLOC_IN_OCRAM;
static BombWallCandidate g_bomb_top_candidates[MAX_BOMBS][MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static BombWallCandidate g_bomb_light_evac_candidates[MAX_BOMBS][LIGHT_EVAC_TOP_WINDOW] ALLOC_IN_OCRAM;
static int g_bomb_phase_order[MAX_BOMBS][MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static uint8_t g_bomb_phase_used[MAX_BOMBS][MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static uint8_t g_bomb_phase_epoch[MAX_BOMBS] ALLOC_IN_OCRAM;
static Position g_bomb_reach_targets[MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static uint8_t g_bomb_reach_slots[MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static BitboardMap g_bomb_temp_map;
static BitboardMap g_bomb_route_base_map[MAX_BOMBS];
static BitboardMap g_bomb_route_map;
static Direction g_solve_best_path_before[MAX_BOMBS][MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static LightEvacPlan g_light_evac_plan_pool[MAX_LIGHT_EVAC_PLANS] ALLOC_IN_SDRAM;
static Direction g_light_evac_saved_best_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static Direction g_light_evac_saved_simple_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static uint8_t g_light_evac_recursion_guard = 0;
typedef struct {
    uint32_t key;
    uint16_t best_len[2];
    int cand_idx[2];
} BlastDedupeBucket;

static uint32_t g_bomb_dedupe_keys[MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static uint8_t g_bomb_dedupe_counts[MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static BlastDedupeBucket g_bomb_dedupe_buckets[MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;
static bool g_bomb_dedupe_keep[MAX_BOMB_CANDIDATES] ALLOC_IN_OCRAM;

typedef struct {
    BitboardMap temp_map;
    BitboardMap orig_map;
    Entity orig_bombs[MAX_BOMBS];
} BombBranchScratch;

static BombBranchScratch g_bomb_branch_scratch[MAX_BOMBS];

typedef struct {
    BitboardMap snapshot_map;
    BitboardMap evac_route_map;
    LightEvacPlan temp_plan;
    BombWallCandidate moved_cand;
    Entity snapshot_boxes[MAX_BOXES];
    Entity snapshot_bombs[MAX_BOMBS];
    uint8_t blocker_indices[MAX_BOXES];
} BombLightScratch;

static BombLightScratch g_bomb_light_scratch[MAX_BOMBS];

static void __attribute__((noinline)) light_evac_restore_entry_state(
    SokobanSolver* solver,
    const BitboardMap* snapshot_map,
    const Entity* snapshot_boxes,
    const Entity* snapshot_bombs,
    Position snapshot_player,
    uint8_t snapshot_num_boxes,
    uint8_t snapshot_num_bombs,
    uint32_t snapshot_destroyed,
    uint16_t snapshot_best_steps,
    uint16_t snapshot_best_path_len,
    const Direction* saved_best_path
) {
    solver->bmap = *snapshot_map;
    memcpy(solver->boxes, snapshot_boxes, sizeof(solver->boxes));
    memcpy(solver->bombs, snapshot_bombs, sizeof(solver->bombs));
    solver->start_player = snapshot_player;
    solver->num_boxes = snapshot_num_boxes;
    solver->num_bombs = snapshot_num_bombs;
    solver->destroyed_walls_mask = snapshot_destroyed;
    solver->best_steps = snapshot_best_steps;
    solver->best_path_len = snapshot_best_path_len;
    if (solver->best_path) {
        memcpy(solver->best_path, saved_best_path, MAX_PATH_LENGTH * sizeof(Direction));
    }
}

typedef struct {
    uint16_t global_dynamic_obs[MAP_ROWS];
    bool ghost_target_filled[MAX_TARGETS];
    int ghost_target_deadlocks[MAX_TARGETS];
    int ghost_target_weight[MAX_TARGETS];
    uint16_t dynamic_obs[MAP_ROWS];
    uint16_t unified_obs[MAP_ROWS];
    uint16_t entity_layer[MAP_ROWS];
    Position entities_to_check[MAX_BOXES + MAX_BOMBS];
    uint16_t all_obs[MAP_ROWS];
    uint16_t player_obs[MAP_ROWS];
    bool low_used[MAX_BOMB_CANDIDATES];
    uint16_t branch_attempt_keys[MAX_BOMB_CANDIDATES];
} BombStrategyScratch;

typedef struct {
    int num_candidates;
    int num_light_evac_candidates;
    uint16_t min_candidate_path_lower;
} BombStrategyBuildResult;

static BombStrategyScratch g_bomb_strategy_scratch[MAX_BOMBS];

typedef struct {
    Position current_boxes[MAX_BOXES];
    bool box_assigned[MAX_BOXES];
    bool target_filled[MAX_TARGETS];
    int current_mapping[MAX_BOXES];
    bool target_used[MAX_TARGETS];
    Position current_boxes_eval[MAX_BOXES];
} SolveInternalScratch;

static SolveInternalScratch g_solve_internal_scratch[MAX_BOMBS];

static Position find_component_rep_on_map(const BitboardMap* bmap, Position start);
static void build_bomb_route_base_map(const SokobanSolver* solver, BitboardMap* route_map);
static void prepare_route_map_targets(const SokobanSolver* solver, BitboardMap* route_map,
                                      int active_box_idx, bool is_bomb_entity,
                                      bool mark_deadlocks);

static int topology_soft_blast_bonus(const SokobanSolver* solver, int depth, int cx, int cy) {
    if (!g_enable_topology_soft_order || !g_topology_features_valid || !solver) return 0;
    if (depth != 0 || solver->is_scanning) return 0;

    int bonus = 0;
    for (int y = cy - 1; y <= cy + 1; y++) {
        if (y <= 0 || y >= MAP_ROWS - 1) continue;
        for (int x = cx - 1; x <= cx + 1; x++) {
            if (x <= 0 || x >= MAP_COLS - 1) continue;
            uint16_t bit = bit_mask_at(x);
            if ((g_topology_features.destructible_bridge_mask[y] & bit) != 0) {
                bonus += BOMB_TOPO_BRIDGE_BONUS;
            }
        }
    }

    if (bonus > BOMB_TOPO_BRIDGE_BONUS_CAP) bonus = BOMB_TOPO_BRIDGE_BONUS_CAP;
    return bonus;
}


static int score_blast_component_bridge(
    const BitboardMap* bmap,
    const uint16_t all_obs[MAP_ROWS],
    const uint16_t player_dist[MAP_ROWS][MAP_COLS],
    int cx, int cy
) {
    int bonus = 0;

    for (int wy = cy - 1; wy <= cy + 1; wy++) {
        if (wy <= 0 || wy >= MAP_ROWS - 1) continue;
        for (int wx = cx - 1; wx <= cx + 1; wx++) {
            if (wx <= 0 || wx >= MAP_COLS - 1) continue;
            uint16_t wall_bit = bit_mask_at(wx);
            if ((bmap->walls[wy] & wall_bit) == 0 || (g_destructible_mask[wy] & wall_bit) == 0) continue;

            int reachable_dirs = 0;
            int unreachable_dirs = 0;
            int open_dirs = 0;
            bool reach_h = false, reach_v = false;
            bool unreach_h = false, unreach_v = false;

            for (int d = 0; d < 4; d++) {
                int nx = wx + DIRECTIONS[d].dx;
                int ny = wy + DIRECTIONS[d].dy;
                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                if ((all_obs[ny] & bit_mask_at(nx)) != 0) continue;

                open_dirs++;
                bool is_reachable = (player_dist[ny][nx] != 0xFFFF);
                bool horizontal = (DIRECTIONS[d].dx != 0);
                if (is_reachable) {
                    reachable_dirs++;
                    if (horizontal) reach_h = true; else reach_v = true;
                } else {
                    unreachable_dirs++;
                    if (horizontal) unreach_h = true; else unreach_v = true;
                }
            }

            if (reachable_dirs > 0 && unreachable_dirs > 0) {
                bonus += BOMB_TOPO_BRIDGE_BONUS;
                if ((reach_h && unreach_h) || (reach_v && unreach_v)) {
                    bonus += BOMB_TOPO_OPPOSITE_BRIDGE_BONUS;
                }
            } else if (open_dirs >= 3 && unreachable_dirs >= 2) {
                bonus += BOMB_TOPO_OPEN_DEGREE_BONUS;
            }
        }
    }

    return (bonus > BOMB_TOPO_BRIDGE_BONUS_CAP) ? BOMB_TOPO_BRIDGE_BONUS_CAP : bonus;
}

static inline uint32_t mix_u32(uint32_t h, uint32_t v) {
    h ^= v + 0x9E3779B9u + (h << 6) + (h >> 2);
    return h;
}

static uint32_t compute_blast_dedupe_key(const SokobanSolver* solver, const BombWallCandidate* cand, const Direction* route_path) {
    Position bomb = solver->bombs[cand->b_idx].pos;
    Position new_player = solver->start_player;
    for (uint16_t step = 0; step < cand->route_len && step < MAX_SINGLE_PATH; step++) {
        new_player.x = (uint8_t)(new_player.x + route_path[step].dx);
        new_player.y = (uint8_t)(new_player.y + route_path[step].dy);
    }

    BitboardMap post_map = solver->bmap;
    clear_bit(post_map.bombs, bomb.x, bomb.y);

    uint32_t key = 0x811C9DC5u;
    key = mix_u32(key, (uint32_t)cand->b_idx);

    uint16_t blast_mask = g_O1_blast_mask[cand->wall_pos.x];
    for (int row = cand->wall_pos.y - 1; row <= cand->wall_pos.y + 1; row++) {
        uint16_t destroyed = 0;
        if (row > 0 && row < MAP_ROWS - 1) {
            destroyed = post_map.walls[row] & blast_mask & g_destructible_mask[row];
            post_map.walls[row] &= (uint16_t)(~destroyed);
        }
        key = mix_u32(key, ((uint32_t)(uint8_t)row << 16) | destroyed);
    }

    for (int row = 0; row < MAP_ROWS; row++) {
        key = mix_u32(key, ((uint32_t)post_map.bombs[row] << 16) | post_map.boxes[row]);
    }

    Position rep = find_component_rep_on_map(&post_map, new_player);
    key = mix_u32(key, (uint32_t)POS_TO_IDX(rep));
    return key;
}

static uint32_t compute_blast_footprint_key(const SokobanSolver* solver, const BombWallCandidate* cand) {
    uint32_t key = 0x9E3779B9u;
    key = mix_u32(key, (uint32_t)cand->b_idx);

    uint16_t blast_mask = g_O1_blast_mask[cand->wall_pos.x];
    for (int row = cand->wall_pos.y - 1; row <= cand->wall_pos.y + 1; row++) {
        uint16_t destroyed = 0;
        if (row > 0 && row < MAP_ROWS - 1) {
            destroyed = solver->bmap.walls[row] & blast_mask & g_destructible_mask[row];
        }
        key = mix_u32(key, ((uint32_t)(uint8_t)row << 16) | destroyed);
    }
    return key;
}

static void __attribute__((noinline)) dedupe_bomb_candidates_by_footprint(const SokobanSolver* solver, BombWallCandidate* candidates, int* num_candidates) {
    enum { KEEP_PER_FOOTPRINT = 3 };
    uint32_t* keys = g_bomb_dedupe_keys;
    uint8_t* counts = g_bomb_dedupe_counts;
    int key_count = 0;
    int write = 0;

    for (int i = 0; i < *num_candidates; i++) {
        uint32_t key = compute_blast_footprint_key(solver, &candidates[i]);
        int key_idx = -1;
        for (int k = 0; k < key_count; k++) {
            if (keys[k] == key) {
                key_idx = k;
                break;
            }
        }
        if (key_idx < 0) {
            if (key_count >= MAX_BOMB_CANDIDATES) continue;
            key_idx = key_count++;
            keys[key_idx] = key;
            counts[key_idx] = 0;
        }
        if (counts[key_idx] >= KEEP_PER_FOOTPRINT) continue;
        counts[key_idx]++;
        if (write != i) candidates[write] = candidates[i];
        write++;
    }

    *num_candidates = write;
}


static void __attribute__((noinline)) dedupe_bomb_candidates_by_blast(const SokobanSolver* solver, int depth, BombWallCandidate* candidates, int* num_candidates) {
    BlastDedupeBucket* buckets = g_bomb_dedupe_buckets;
    int bucket_count = 0;
    bool* keep = g_bomb_dedupe_keep;
    memset(keep, 0, MAX_BOMB_CANDIDATES * sizeof(keep[0]));

    for (int i = 0; i < *num_candidates; i++) {
        BombWallCandidate* cand = &candidates[i];
        if (!bomb_candidate_route_precomputed(cand)) {
            keep[i] = true;
            continue;
        }
        if (cand->route_len == 0xFFFF || cand->route_len > MAX_SINGLE_PATH) {
            continue;
        }

        Direction* route_path = g_bomb_path_pool[depth];
        packed_path_load_to_direction(g_bomb_reach_all_path_pool[depth][cand->route_slot], route_path, cand->route_len);
        uint32_t key = compute_blast_dedupe_key(solver, cand, route_path);
        int bucket_idx = -1;
        for (int b = 0; b < bucket_count; b++) {
            if (buckets[b].key == key) {
                bucket_idx = b;
                break;
            }
        }
        if (bucket_idx < 0) {
            if (bucket_count >= MAX_BOMB_CANDIDATES) continue;
            bucket_idx = bucket_count++;
            buckets[bucket_idx].key = key;
            buckets[bucket_idx].best_len[0] = 0xFFFF;
            buckets[bucket_idx].best_len[1] = 0xFFFF;
            buckets[bucket_idx].cand_idx[0] = -1;
            buckets[bucket_idx].cand_idx[1] = -1;
        }

        uint16_t rank_len = cand->route_len;
        int replace_slot = -1;
        if (rank_len < buckets[bucket_idx].best_len[0] ||
            (rank_len == buckets[bucket_idx].best_len[0] && cand->score > candidates[buckets[bucket_idx].cand_idx[0]].score)) {
            replace_slot = 0;
        } else if (rank_len < buckets[bucket_idx].best_len[1] ||
                   (rank_len == buckets[bucket_idx].best_len[1] &&
                    (buckets[bucket_idx].cand_idx[1] < 0 || cand->score > candidates[buckets[bucket_idx].cand_idx[1]].score))) {
            replace_slot = 1;
        }

        if (replace_slot == 0) {
            buckets[bucket_idx].best_len[1] = buckets[bucket_idx].best_len[0];
            buckets[bucket_idx].cand_idx[1] = buckets[bucket_idx].cand_idx[0];
            buckets[bucket_idx].best_len[0] = rank_len;
            buckets[bucket_idx].cand_idx[0] = i;
        } else if (replace_slot == 1) {
            buckets[bucket_idx].best_len[1] = rank_len;
            buckets[bucket_idx].cand_idx[1] = i;
        }
    }

    for (int b = 0; b < bucket_count; b++) {
        for (int s = 0; s < 2; s++) {
            int idx = buckets[b].cand_idx[s];
            if (idx >= 0) keep[idx] = true;
        }
    }
    int write = 0;
    for (int i = 0; i < *num_candidates; i++) {
        if (!keep[i]) continue;
        if (write != i) candidates[write] = candidates[i];
        write++;
    }
    *num_candidates = write;
}

static uint16_t g_bfs_visited[MAP_ROWS][MAP_COLS];
static uint16_t g_bfs_run_epoch = 0;
static Position g_bfs_queue[MAP_ROWS * MAP_COLS];

#define PUSH_FILTER_MAX_QUEUE 512
#define PUSH_FILTER_STATE_COUNT (MAP_ROWS * MAP_COLS)
#define PUSH_FILTER_SEEN_BITS (PUSH_FILTER_STATE_COUNT * PUSH_FILTER_STATE_COUNT)
#define PUSH_FILTER_SEEN_WORDS ((PUSH_FILTER_SEEN_BITS + 31u) / 32u)
static uint32_t g_push_filter_seen_bits[PUSH_FILTER_SEEN_WORDS] ALLOC_IN_OCRAM;
static uint16_t g_push_filter_box_q[PUSH_FILTER_MAX_QUEUE] ALLOC_IN_OCRAM;
static uint16_t g_push_filter_player_q[PUSH_FILTER_MAX_QUEUE] ALLOC_IN_OCRAM;

static inline uint32_t push_filter_seen_index(uint16_t box_idx, uint16_t player_idx) {
    return ((uint32_t)box_idx * (uint32_t)PUSH_FILTER_STATE_COUNT) + (uint32_t)player_idx;
}

static inline bool push_filter_seen_test(uint16_t box_idx, uint16_t player_idx) {
    uint32_t idx = push_filter_seen_index(box_idx, player_idx);
    return (g_push_filter_seen_bits[idx >> 5] & (1u << (idx & 31u))) != 0;
}

static inline void push_filter_seen_set(uint16_t box_idx, uint16_t player_idx) {
    uint32_t idx = push_filter_seen_index(box_idx, player_idx);
    g_push_filter_seen_bits[idx >> 5] |= (1u << (idx & 31u));
}

static BitboardMap g_dfs_temp_map[MAX_BOXES];
static inline void prepare_dfs_temp_maps(const SokobanSolver* solver) {
    for (int i = 0; i < MAX_BOXES; i++) {
        g_dfs_temp_map[i] = solver->bmap;
    }
}

static Position g_dfs_available_targets[MAX_BOXES][MAX_TARGETS];
typedef struct {
    int box_order[MAX_BOXES];
    uint16_t box_macro_dist[MAX_BOXES];
    uint16_t box_player_dist[MAX_BOXES];
    Position heuristic_boxes[MAX_BOXES];
    Position heuristic_targets[MAX_TARGETS];
    uint8_t heuristic_box_indices[MAX_BOXES];
    uint8_t heuristic_target_indices[MAX_TARGETS];
    uint8_t available_target_indices[MAX_TARGETS];
    uint16_t push_reach_mask[MAP_ROWS];
    Position candidate_boxes[MAX_BOXES];
    Position candidate_targets[MAX_TARGETS];
    uint8_t candidate_box_indices[MAX_BOXES];
    uint8_t candidate_target_indices[MAX_TARGETS];
    uint16_t orig_deadlocks[MAP_ROWS];
    uint16_t orig_targets[MAP_ROWS];
} DfsFrameScratch;
static DfsFrameScratch g_dfs_frame_scratch[MAX_BOXES] ALLOC_IN_DTCM;

#ifndef SOKOBAN_ASSIGNMENT_BEAM_WIDTH
#define SOKOBAN_ASSIGNMENT_BEAM_WIDTH 4u
#endif
#if SOKOBAN_ASSIGNMENT_BEAM_WIDTH < 1u || SOKOBAN_ASSIGNMENT_BEAM_WIDTH > 128u
#error "SOKOBAN_ASSIGNMENT_BEAM_WIDTH must be in [1, 128]."
#endif
#if SOKOBAN_SCAN_BOMB_ASSIGNMENT_BEAM_WIDTH < 1u || SOKOBAN_SCAN_BOMB_ASSIGNMENT_BEAM_WIDTH > SOKOBAN_ASSIGNMENT_BEAM_WIDTH
#error "SOKOBAN_SCAN_BOMB_ASSIGNMENT_BEAM_WIDTH must fit the assignment beam storage."
#endif
#if MAX_BOXES > 16 || MAX_TARGETS > 16
#error "Assignment beam masks require MAX_BOXES and MAX_TARGETS to be at most 16."
#endif

#define ASSIGNMENT_BEAM_PATH_BYTES PACKED_PATH_BYTES(MAX_PATH_LENGTH)

typedef struct {
    Position player;
    uint16_t assigned_mask;
    uint16_t filled_mask;
    uint16_t steps;
    uint16_t lower_bound;
    uint32_t rank;
    uint32_t stable_order;
    PackedDirByte path[ASSIGNMENT_BEAM_PATH_BYTES];
} AssignmentBeamState;

typedef enum {
    ASSIGNMENT_BEAM_INELIGIBLE,
    ASSIGNMENT_BEAM_EXHAUSTED,
    ASSIGNMENT_BEAM_COMPLETE,
    ASSIGNMENT_BEAM_FATAL
} AssignmentBeamResult;

static AssignmentBeamState
    g_assignment_beam_states[2][SOKOBAN_ASSIGNMENT_BEAM_WIDTH] ALLOC_IN_SDRAM;
static uint16_t g_assignment_beam_active_width = SOKOBAN_ASSIGNMENT_BEAM_WIDTH;
static bool g_assignment_batch_enabled = false;
static AStarBatchResult g_assignment_batch_result ALLOC_IN_SDRAM;
static BitboardMap g_assignment_beam_box_map ALLOC_IN_OCRAM;
static BitboardMap g_assignment_beam_map ALLOC_IN_OCRAM;
static BitboardMap g_assignment_beam_replay_map ALLOC_IN_OCRAM;
static Entity g_assignment_beam_replay_boxes[MAX_BOXES] ALLOC_IN_OCRAM;
static PathReplayResult g_assignment_beam_replay_result ALLOC_IN_OCRAM;

static void __attribute__((noinline)) build_distance_field_for_depth(const BitboardMap* bmap, const Entity* targets, int num_targets, int depth) {
    if (depth < 0 || depth >= MAX_MACRO_DEPTH) return;

    uint16_t (*dist_map)[MAP_COLS] = g_macro_dist_field[depth];
    memset(dist_map, 0xFF, sizeof(g_macro_dist_field[depth]));

    Position* q = g_bfs_queue;
    int head = 0, tail = 0;

    for (int i = 0; i < num_targets; i++) {
        Position target = targets[i].pos;
        if (!targets[i].is_active || !is_in_bounds(target.x, target.y)) continue;
        if (dist_map[target.y][target.x] == 0) continue;

        dist_map[target.y][target.x] = 0;
        q[tail++] = target;
    }

    while (head < tail) {
        Position curr = q[head++];
        uint16_t curr_dist = dist_map[curr.y][curr.x];

        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (!is_in_bounds(nx, ny)) continue;

            uint16_t bit = bit_mask_at(nx);
            if ((bmap->walls[ny] & bit) != 0) continue;
            if ((bmap->deadlocks[ny] & bit) != 0) continue;

            if (dist_map[ny][nx] == 0xFFFF) {
                dist_map[ny][nx] = (uint16_t)(curr_dist + 1);
                if (tail < MAP_ROWS * MAP_COLS) {
                    q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
                }
            }
        }
    }
}
#define MAX_MACRO 4
static Direction g_evac_path_pool[MAX_BOMBS][MAX_MACRO][MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static uint16_t g_current_macro_depth = 0;
static bool g_allow_macro_evacuation = false;
#define UNIVERSE_EVAC_HASH_SALT 0x9E3779B9u
static bool g_allow_super_evacuation = false;
static uint16_t g_current_super_depth = 0;
#define MAX_SUPER_MACRO 3
#define UNIVERSE_SUPER_HASH_SALT 0x7A5C3B1Fu
typedef char SuperEvacPathPool_must_fit_macro_pool[(MAX_SUPER_MACRO <= MAX_MACRO) ? 1 : -1];
#define MAX_POCKET_UNBLOCK_DEPTH 2
#define MAX_POCKET_UNBLOCK_CANDIDATES 12
#define MAX_POCKET_PARKINGS 24
#define PLAYER_POCKET_MAX_REACH 16
#define MAX_EVAC_CANDIDATES 32
#define MAX_PARKINGS 10
#define MAX_SUPER_EVAC_CANDIDATES 16
#define MAX_SUPER_PARKINGS 8
#define EVAC_SCRATCH_PARKINGS 16

/* 该临时缓冲只保存不跨递归复用的疏散计算数据。 */
typedef struct {
    Direction route_path[MAX_SINGLE_PATH];
    Direction apply_path[MAX_SINGLE_PATH];
    Position parkings[EVAC_SCRATCH_PARKINGS];
    Position reach_queue[MAP_ROWS * MAP_COLS];
} EvacScratch;

typedef union {
    EvacScratch evac;
} SolverScratch;

static SolverScratch g_solver_scratch;
_Static_assert(sizeof(((EvacScratch*)0)->parkings) >= MAX_PARKINGS * sizeof(Position),
               "EvacScratch parkings too small for smart_evac");
_Static_assert(sizeof(((EvacScratch*)0)->parkings) >= MAX_SUPER_PARKINGS * sizeof(Position),
               "EvacScratch parkings too small for super_evac");
_Static_assert(sizeof(((EvacScratch*)0)->route_path) >= MAX_SINGLE_PATH * sizeof(Direction),
               "EvacScratch route_path too small");
_Static_assert(sizeof(((EvacScratch*)0)->apply_path) >= MAX_SINGLE_PATH * sizeof(Direction),
               "EvacScratch apply_path too small");
static Direction g_pocket_path_pool[MAX_POCKET_UNBLOCK_DEPTH][MAX_PATH_LENGTH] ALLOC_IN_SDRAM;
static uint16_t g_current_pocket_depth = 0;

static inline bool solver_is_target_cell(const SokobanSolver* solver, Position pos) {
    return (solver->bmap.targets[pos.y] & bit_mask_at(pos.x)) != 0;
}

static inline uint16_t macro_distance_to_target(int macro_depth, Position pos) {
    if (macro_depth < 0 || macro_depth >= MAX_MACRO_DEPTH) return 0xFFFF;
    return g_macro_dist_field[macro_depth][pos.y][pos.x];
}
static void __attribute__((noinline)) build_target_distance_fields(const BitboardMap* bmap, const Entity* targets, int num_targets) {
    memset(g_target_dist_field, 0xFF, sizeof(g_target_dist_field));

    for (int ti = 0; ti < num_targets && ti < MAX_TARGETS; ti++) {
        if (!targets[ti].is_active) continue;
        Position target = targets[ti].pos;
        if (!is_in_bounds(target.x, target.y)) continue;

        Position* q = g_bfs_queue;
        int head = 0;
        int tail = 0;
        g_target_dist_field[ti][target.y][target.x] = 0;
        q[tail++] = target;

        while (head < tail) {
            Position curr = q[head++];
            uint16_t curr_dist = g_target_dist_field[ti][curr.y][curr.x];

            for (int d = 0; d < 4; d++) {
                int px = curr.x - DIRECTIONS[d].dx;
                int py = curr.y - DIRECTIONS[d].dy;
                int stance_x = px - DIRECTIONS[d].dx;
                int stance_y = py - DIRECTIONS[d].dy;
                if (!is_in_bounds(px, py) || !is_in_bounds(stance_x, stance_y)) continue;

                uint16_t prev_bit = bit_mask_at(px);
                if ((bmap->walls[py] & prev_bit) != 0) continue;
                if ((bmap->deadlocks[py] & prev_bit) != 0) continue;

                uint16_t stance_bit = bit_mask_at(stance_x);
                if ((bmap->walls[stance_y] & stance_bit) != 0) continue;
                if (g_target_dist_field[ti][py][px] != 0xFFFF) continue;

                g_target_dist_field[ti][py][px] = (uint16_t)(curr_dist + 1);
                if (tail < MAP_ROWS * MAP_COLS) {
                    q[tail++] = (Position){(uint8_t)px, (uint8_t)py};
                }
            }
        }
    }
}


static inline uint32_t hash_move_player(uint32_t hash, Position old_pos, Position new_pos) {
    return hash ^ ZOBRIST_PLAYER[Z_IDX(old_pos.x, old_pos.y)] ^ ZOBRIST_PLAYER[Z_IDX(new_pos.x, new_pos.y)];
}

static inline uint32_t hash_move_box(uint32_t hash, Position old_pos, Position new_pos, bool is_target) {
    hash ^= ZOBRIST_BOX[Z_IDX(old_pos.x, old_pos.y)];
    if (!is_target) {
        hash ^= ZOBRIST_BOX[Z_IDX(new_pos.x, new_pos.y)];
    }
    return hash;
}

static uint32_t compute_universe_hash(const SokobanSolver* solver) {
    init_zobrist();
    uint32_t hash = ZOBRIST_PLAYER[Z_IDX(solver->start_player.x, solver->start_player.y)];
    for (int k = 0; k < solver->num_boxes; k++) {
        if (solver->boxes[k].is_active) {
            Position box_pos = solver->boxes[k].pos;
            if (!get_bit(solver->bmap.targets, box_pos.x, box_pos.y)) {
                hash ^= ZOBRIST_BOX[Z_IDX(box_pos.x, box_pos.y)];
            }
        }
    }
    for (int k = 0; k < solver->num_bombs; k++) {
        Position bomb_pos = solver->bombs[k].pos;
        hash ^= ZOBRIST_BOMB[Z_IDX(bomb_pos.x, bomb_pos.y)];
    }

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        uint16_t row_walls = solver->bmap.walls[y] & g_destructible_mask[y];
        while (row_walls) {
            int x = __builtin_ctz(row_walls);
            hash ^= ZOBRIST_WALL[Z_IDX(x, y)];
            row_walls &= (row_walls - 1);
        }
    }

    if (g_allow_macro_evacuation) {
        hash ^= UNIVERSE_EVAC_HASH_SALT;
    } else if (g_allow_super_evacuation) {
        hash ^= UNIVERSE_SUPER_HASH_SALT;
    }
    return hash;
}
static Position find_component_rep_on_map(const BitboardMap* bmap, Position start) {
    Position rep = start;
    if (rep.x <= 0 || rep.x >= MAP_COLS - 1 || rep.y <= 0 || rep.y >= MAP_ROWS - 1) return rep;

    uint16_t start_bit = (uint16_t)(1u << rep.x);
    if ((map_blocked_row(bmap, rep.y) & start_bit) != 0) {
        return rep;
    }

    const uint16_t interior_cols_mask = (uint16_t)(((uint16_t)1u << (MAP_COLS - 1)) - 2u);
    uint16_t blocked[MAP_ROWS];
    uint16_t reach[MAP_ROWS] = {0};
    uint16_t next[MAP_ROWS];

    for (int y = 0; y < MAP_ROWS; y++) {
        blocked[y] = map_blocked_row(bmap, y);
    }

    reach[rep.y] = start_bit;
    do {
        bool changed = false;
        memcpy(next, reach, sizeof(next));

        for (int y = 1; y < MAP_ROWS - 1; y++) {
            uint16_t expanded = reach[y];
            expanded |= (uint16_t)((expanded << 1) | (expanded >> 1));
            expanded |= reach[y - 1] | reach[y + 1];
            expanded &= interior_cols_mask;
            expanded &= (uint16_t)(~blocked[y]);

            if (expanded != reach[y]) {
                next[y] = expanded;
                changed = true;
            }
        }

        memcpy(reach, next, sizeof(reach));
        if (!changed) break;
    } while (true);

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        uint16_t row = reach[y];
        if (row != 0) {
            int x = __builtin_ctz((unsigned int)row);
            return (Position){(uint8_t)x, (uint8_t)y};
        }
    }

    return rep;
}

static inline uint32_t compute_component_fail_key(const SokobanSolver* solver, uint32_t current_hash) {
    Position rep = find_component_rep_on_map(&solver->bmap, solver->start_player);
    uint32_t key = current_hash;
    key ^= ZOBRIST_PLAYER[Z_IDX(solver->start_player.x, solver->start_player.y)];
    key ^= ZOBRIST_PLAYER[Z_IDX(rep.x, rep.y)];
    key ^= (uint32_t)solver->num_bombs * 0x85EBCA6Bu;
    return key;
}

static inline bool component_fail_cached(uint32_t key, uint16_t best_steps, uint8_t bombs_left) {
    uint32_t idx = CACHE_GET_IDX(key, 2654435761u, COMPONENT_FAIL_CACHE_SIZE);
    ComponentFailEntry e = g_component_fail_cache[idx];
    bool same_key = e.valid && e.key == key && e.bombs_left == bombs_left;
    if (!same_key) return false;
    return e.best_steps >= best_steps;
}

static inline void component_fail_store(uint32_t key, uint16_t best_steps, uint8_t bombs_left) {
    
    uint32_t idx = CACHE_GET_IDX(key, 2654435761u, COMPONENT_FAIL_CACHE_SIZE);
    g_component_fail_cache[idx] = (ComponentFailEntry){key, best_steps, bombs_left, 1};
}

static inline uint32_t clear_solver_explosion_wall_row_O1(SokobanSolver* solver, int wy, uint16_t blast_mask) {
    uint32_t hash_delta = 0;
    uint16_t destroyed_bits = solver->bmap.walls[wy] & blast_mask & g_destructible_mask[wy];

    if (!destroyed_bits) return 0;
    solver->bmap.walls[wy] &= (uint16_t)(~destroyed_bits);

    while (destroyed_bits != 0) {
        int wx = __builtin_ctz((unsigned int)destroyed_bits);
        destroyed_bits &= (uint16_t)(destroyed_bits - 1);
        hash_delta ^= ZOBRIST_WALL[Z_IDX(wx, wy)];
    }
    return hash_delta;
}

static inline uint32_t clear_solver_explosion_walls(SokobanSolver* solver, Position center) {
    uint32_t hash_delta = 0;
    uint16_t blast_mask = g_O1_blast_mask[center.x];

    uint16_t deadlock_clear_mask = g_O1_deadlock_clear[center.x];
    for (int dy = center.y - 2; dy <= center.y + 2; dy++) {
        if (dy >= 0 && dy < MAP_ROWS) {
            solver->bmap.deadlocks[dy] &= deadlock_clear_mask;
        }
    }

    if (center.y > 1) hash_delta ^= clear_solver_explosion_wall_row_O1(solver, center.y - 1, blast_mask);
    if (center.y > 0 && center.y < MAP_ROWS - 1) hash_delta ^= clear_solver_explosion_wall_row_O1(solver, center.y, blast_mask);
    if (center.y < MAP_ROWS - 2) hash_delta ^= clear_solver_explosion_wall_row_O1(solver, center.y + 1, blast_mask);

    return hash_delta;
}

void solver_sim_clear_explosion_walls(BitboardMap* bmap, Position center) {
    uint16_t blast_mask = g_O1_blast_mask[center.x];
    if (center.y > 1) bmap->walls[center.y - 1] &= (uint16_t)(~(blast_mask & g_destructible_mask[center.y - 1]));
    if (center.y > 0 && center.y < MAP_ROWS - 1) bmap->walls[center.y] &= (uint16_t)(~(blast_mask & g_destructible_mask[center.y]));
    if (center.y < MAP_ROWS - 2) bmap->walls[center.y + 1] &= (uint16_t)(~(blast_mask & g_destructible_mask[center.y + 1]));
}

static int path_replay_clamp_count(int count, int max_count, const void* source) {
    if (!source) return 0;
    if (count < 0) return 0;
    if (count > max_count) return max_count;
    return count;
}

PathReplayOptions path_replay_default_options(void) {
    PathReplayOptions options;
    options.mode = PATH_REPLAY_LEGACY_LENIENT;
    options.box_target_mode = PATH_REPLAY_BOX_TARGET_SOLVER_RULE;
    options.preserve_dynamic_tunnels_on_blast = false;
    options.marked_box_idx = -1;
    options.marked_target_pos = (Position){0xFF, 0xFF};
    return options;
}

static bool path_replay_is_strict(const PathReplayOptions* options) {
    return options && options->mode == PATH_REPLAY_STRICT_VALIDATE;
}

static Position path_replay_invalid_pos(void) {
    return (Position){0xFF, 0xFF};
}

void path_replay_init_state(PathReplayState* state) {
    memset(state, 0, sizeof(*state));
    for (int i = 0; i < MAX_BOXES; i++) state->boxes[i] = path_replay_invalid_pos();
}

bool path_replay_load_state(
    PathReplayState* state,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
) {
    if (!state || !initial_map) return false;
    path_replay_init_state(state);
    state->map = *initial_map;
    state->player = initial_player;

    int box_count = path_replay_clamp_count(initial_num_boxes, MAX_BOXES, initial_boxes);
    for (int i = 0; i < box_count; i++) state->boxes[i] = initial_boxes[i].pos;

    state->bomb_count = path_replay_clamp_count(initial_num_bombs, MAX_BOMBS, initial_bombs);
    for (int i = 0; i < state->bomb_count; i++) state->bombs[i] = initial_bombs[i];
    return true;
}

void path_replay_init_result(PathReplayResult* out) {
    memset(out, 0, sizeof(*out));
    path_replay_init_state(&out->final_state);
}

static PathReplayStepResult path_replay_step_result(PathReplayStepKind kind, PathReplayError error) {
    PathReplayStepResult result;
    result.kind = kind;
    result.player_pos = path_replay_invalid_pos();
    result.entity_from = path_replay_invalid_pos();
    result.entity_to = path_replay_invalid_pos();
    result.entity_index = -1;
    result.box_absorbed = false;
    result.error = error;
    return result;
}

static int path_replay_find_bomb_entity(const Entity* bombs, int bomb_count, Position pos) {
    for (int b = 0; b < bomb_count; b++) {
        if (pos_equal(bombs[b].pos, pos)) return b;
    }
    return -1;
}

static void path_replay_remove_bomb_entity(Entity* bombs, int* bomb_count, int bomb_idx) {
    if (!bombs || !bomb_count || bomb_idx < 0 || bomb_idx >= *bomb_count) return;
    for (int k = bomb_idx; k < *bomb_count - 1; k++) bombs[k] = bombs[k + 1];
    (*bomb_count)--;
}

static bool path_replay_should_absorb_box(
    const SokobanSolver* solver,
    const PathReplayState* state,
    const PathReplayOptions* options,
    int box_idx,
    Position target_pos
) {
    bool is_target = get_bit(state->map.targets, target_pos.x, target_pos.y);
    PathReplayBoxTargetMode mode = options ? options->box_target_mode : PATH_REPLAY_BOX_TARGET_SOLVER_RULE;
    switch (mode) {
        case PATH_REPLAY_BOX_TARGET_HIDE_BOX_KEEP_TARGET:
        case PATH_REPLAY_BOX_TARGET_SET_BOX_ON_TARGET:
        case PATH_REPLAY_BOX_TARGET_MARKED_CLEAR_TARGET_HIDE_BOX:
            return false;
        case PATH_REPLAY_BOX_TARGET_SOLVER_RULE:
        default:
            return is_target && solver_should_absorb_box(solver, box_idx, target_pos);
    }
}

static void path_replay_apply_box_push(
    const SokobanSolver* solver,
    PathReplayState* state,
    const PathReplayOptions* options,
    int box_idx,
    Position from,
    Position to,
    bool* out_absorbed
) {
    PathReplayBoxTargetMode mode = options ? options->box_target_mode : PATH_REPLAY_BOX_TARGET_SOLVER_RULE;

    if (mode == PATH_REPLAY_BOX_TARGET_HIDE_BOX_KEEP_TARGET) {
        clear_bit(state->map.boxes, from.x, from.y);
        if (!get_bit(state->map.targets, to.x, to.y)) set_bit(state->map.boxes, to.x, to.y);
        if (box_idx >= 0) state->boxes[box_idx] = to;
        if (out_absorbed) *out_absorbed = get_bit(state->map.targets, to.x, to.y);
        return;
    }

    if (mode == PATH_REPLAY_BOX_TARGET_MARKED_CLEAR_TARGET_HIDE_BOX) {
        bool clear_marked_target =
            box_idx == (options ? options->marked_box_idx : -1) &&
            pos_equal(to, options ? options->marked_target_pos : path_replay_invalid_pos()) &&
            get_bit(state->map.targets, to.x, to.y);
        clear_bit(state->map.boxes, from.x, from.y);
        if (clear_marked_target) {
            clear_bit(state->map.targets, to.x, to.y);
        } else {
            set_bit(state->map.boxes, to.x, to.y);
        }
        if (box_idx >= 0) state->boxes[box_idx] = clear_marked_target ? path_replay_invalid_pos() : to;
        if (out_absorbed) *out_absorbed = clear_marked_target;
        return;
    }

    bool absorbed = path_replay_should_absorb_box(solver, state, options, box_idx, to);
    apply_tracked_box_push_bits(&state->map, state->boxes, box_idx, from, to, absorbed);
    if (out_absorbed) *out_absorbed = absorbed;
}
PathReplayStepResult path_replay_step(
    const SokobanSolver* solver,
    PathReplayState* state,
    Direction d,
    const PathReplayOptions* options
) {
    if (!solver || !state) return path_replay_step_result(PATH_REPLAY_STEP_ERROR, PATH_REPLAY_ERROR_INVALID_ARGUMENT);
    bool strict = path_replay_is_strict(options);

    if (direction_is_pause(d)) {
        PathReplayStepResult result = path_replay_step_result(PATH_REPLAY_STEP_PAUSED, PATH_REPLAY_ERROR_NONE);
        result.player_pos = state->player;
        return result;
    }

    if (!direction_is_cardinal(d)) {
        if (strict) return path_replay_step_result(PATH_REPLAY_STEP_ERROR, PATH_REPLAY_ERROR_INVALID_STEP);
        PathReplayStepResult result = path_replay_step_result(PATH_REPLAY_STEP_IGNORED, PATH_REPLAY_ERROR_NONE);
        result.player_pos = state->player;
        return result;
    }

    Position curr_p = state->player;
    Position next_p = {(uint8_t)(curr_p.x + d.dx), (uint8_t)(curr_p.y + d.dy)};
    if (!is_in_bounds(next_p.x, next_p.y)) {
        return path_replay_step_result(strict ? PATH_REPLAY_STEP_ERROR : PATH_REPLAY_STEP_STOPPED,
                                       strict ? PATH_REPLAY_ERROR_INVALID_STEP : PATH_REPLAY_ERROR_NONE);
    }

    if (strict && get_bit(state->map.walls, next_p.x, next_p.y)) {
        return path_replay_step_result(PATH_REPLAY_STEP_ERROR, PATH_REPLAY_ERROR_BLOCKED);
    }

    PathReplayStepResult result = path_replay_step_result(PATH_REPLAY_STEP_MOVED, PATH_REPLAY_ERROR_NONE);
    result.player_pos = next_p;

    if (get_bit(state->map.bombs, next_p.x, next_p.y)) {
        Position next_bomb = {(uint8_t)(next_p.x + d.dx), (uint8_t)(next_p.y + d.dy)};
        if (!is_in_bounds(next_bomb.x, next_bomb.y)) {
            return path_replay_step_result(strict ? PATH_REPLAY_STEP_ERROR : PATH_REPLAY_STEP_STOPPED,
                                           strict ? PATH_REPLAY_ERROR_INVALID_STEP : PATH_REPLAY_ERROR_NONE);
        }
        if (strict && !get_bit(state->map.walls, next_bomb.x, next_bomb.y)) {
            uint16_t bit = bit_mask_at(next_bomb.x);
            if (((state->map.boxes[next_bomb.y] | state->map.bombs[next_bomb.y]) & bit) != 0) {
                return path_replay_step_result(PATH_REPLAY_STEP_ERROR, PATH_REPLAY_ERROR_BLOCKED);
            }
        }

        clear_bit(state->map.bombs, next_p.x, next_p.y);
        int b_entity_idx = path_replay_find_bomb_entity(state->bombs, state->bomb_count, next_p);
        result.entity_from = next_p;
        result.entity_to = next_bomb;

        if (get_bit(state->map.walls, next_bomb.x, next_bomb.y)) {
            solver_sim_clear_explosion_walls(&state->map, next_bomb);
            if (!options || !options->preserve_dynamic_tunnels_on_blast) sokoban_refresh_dynamic_tunnels(&state->map);
            path_replay_remove_bomb_entity(state->bombs, &state->bomb_count, b_entity_idx);
            result.kind = PATH_REPLAY_STEP_BLASTED_WALL;
        } else {
            set_bit(state->map.bombs, next_bomb.x, next_bomb.y);
            if (b_entity_idx != -1) state->bombs[b_entity_idx].pos = next_bomb;
            result.kind = PATH_REPLAY_STEP_PUSHED_BOMB;
        }
    } else if (get_bit(state->map.boxes, next_p.x, next_p.y)) {
        Position next_box = {(uint8_t)(next_p.x + d.dx), (uint8_t)(next_p.y + d.dy)};
        if (!is_in_bounds(next_box.x, next_box.y)) {
            return path_replay_step_result(strict ? PATH_REPLAY_STEP_ERROR : PATH_REPLAY_STEP_STOPPED,
                                           strict ? PATH_REPLAY_ERROR_INVALID_STEP : PATH_REPLAY_ERROR_NONE);
        }
        if (strict) {
            uint16_t bit = bit_mask_at(next_box.x);
            if ((map_blocked_row(&state->map, next_box.y) & bit) != 0) {
                return path_replay_step_result(PATH_REPLAY_STEP_ERROR, PATH_REPLAY_ERROR_BLOCKED);
            }
        }
        int box_idx = tracked_position_index(state->boxes, solver->num_boxes, next_p);
        result.entity_index = box_idx;
        path_replay_apply_box_push(solver, state, options, box_idx, next_p, next_box, &result.box_absorbed);
        result.kind = PATH_REPLAY_STEP_PUSHED_BOX;
        result.entity_from = next_p;
        result.entity_to = next_box;
    }

    state->player = next_p;
    return result;
}

bool path_replay_run(
    const SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs,
    const Direction* path,
    uint16_t path_len,
    const PathReplayOptions* options,
    PathReplayResult* out
) {
    if (out) path_replay_init_result(out);
    if (!solver || !initial_map || !path || !out) {
        if (out) out->error = PATH_REPLAY_ERROR_INVALID_ARGUMENT;
        return false;
    }

    PathReplayOptions effective_options = options ? *options : path_replay_default_options();
    if (!path_replay_load_state(&out->final_state, initial_map, initial_player, initial_boxes, initial_num_boxes, initial_bombs, initial_num_bombs)) {
        out->error = PATH_REPLAY_ERROR_INVALID_ARGUMENT;
        return false;
    }

    out->ok = true;
    out->error = PATH_REPLAY_ERROR_NONE;

    for (uint16_t i = 0; i < path_len; i++) {
        PathReplayStepResult step = path_replay_step(solver, &out->final_state, path[i], &effective_options);
        if (step.kind == PATH_REPLAY_STEP_ERROR) {
            out->ok = false;
            out->error = step.error;
            return false;
        }
        if (step.kind == PATH_REPLAY_STEP_STOPPED) break;

        out->consumed_len = (uint16_t)(i + 1);

    }
    return true;
}

static void solver_clear_bomb_delay_events(SokobanSolver* solver) {
    if (!solver) return;
    solver->bomb_delay_count = 0;
    memset(solver->bomb_delay_events, 0, sizeof(solver->bomb_delay_events));
}


static void solver_record_bomb_delay_event(
    SokobanSolver* solver,
    uint16_t path_index,
    Position player_pos,
    Position blast_center,
    Position next_pos
) {
    if (!solver || solver->bomb_delay_count >= MAX_BOMB_DELAY_EVENTS) return;
    BombDelayEvent* event = &solver->bomb_delay_events[solver->bomb_delay_count++];
    event->path_index = path_index;
    event->player_pos = player_pos;
    event->blast_center = blast_center;
    event->next_pos = next_pos;
}

uint8_t solver_get_bomb_delay_count(const SokobanSolver* solver) {
    return solver ? solver->bomb_delay_count : 0;
}

bool solver_get_bomb_delay_event(const SokobanSolver* solver, int index, BombDelayEvent* out_event) {
    if (!solver || !out_event || index < 0 || index >= solver->bomb_delay_count) return false;
    *out_event = solver->bomb_delay_events[index];
    return true;
}

bool solver_get_bomb_delay_at_path_index(const SokobanSolver* solver, uint16_t path_index, BombDelayEvent* out_event) {
    if (!solver) return false;
    for (int i = 0; i < solver->bomb_delay_count; i++) {
        if (solver->bomb_delay_events[i].path_index == path_index) {
            if (out_event) *out_event = solver->bomb_delay_events[i];
            return true;
        }
    }
    return false;
}

void solver_build_bomb_delay_events_from_state(
    SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
) {
    solver_clear_bomb_delay_events(solver);
    if (!solver || !initial_map || !solver->best_path) return;

    PathReplayState state;
    if (!path_replay_load_state(&state, initial_map, initial_player,
                                initial_boxes, initial_num_boxes,
                                initial_bombs, initial_num_bombs)) {
        return;
    }

    PathReplayOptions options = path_replay_default_options();
    options.mode = PATH_REPLAY_LEGACY_LENIENT;
    options.preserve_dynamic_tunnels_on_blast = true;

    for (uint16_t i = 0; i < solver->best_path_len; i++) {
        PathReplayStepResult step = path_replay_step(solver, &state, solver->best_path[i], &options);
        if (step.kind == PATH_REPLAY_STEP_ERROR || step.kind == PATH_REPLAY_STEP_STOPPED) break;

        if (step.kind != PATH_REPLAY_STEP_BLASTED_WALL) continue;

        Position player_after_blast = step.player_pos;
        Position blast_center = step.entity_to;
        uint16_t delay_idx = (uint16_t)(i + 1);
        Position next_pos = player_after_blast;

        if (delay_idx < solver->best_path_len) {
            Direction next_step = solver->best_path[delay_idx];
            if (direction_is_cardinal(next_step)) {
                next_pos.x = (uint8_t)(player_after_blast.x + next_step.dx);
                next_pos.y = (uint8_t)(player_after_blast.y + next_step.dy);
            } else if (!direction_is_pause(next_step)) {
                continue;
            }
        }

        if (is_in_bounds(next_pos.x, next_pos.y)) {
            solver_record_bomb_delay_event(solver, delay_idx, player_after_blast, blast_center, next_pos);
        }
    }
}
static bool g_edge_target_L, g_edge_target_R, g_edge_target_U, g_edge_target_D;

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
FAST_RAM_FUNC static bool sokoban_solve_internal(SokobanSolver* solver, int depth, uint32_t current_hash);
static bool __attribute__((noinline)) try_bomb_strategy_simple(SokobanSolver* solver, int depth, uint32_t current_hash);
FAST_OCRAM_FUNC static bool sq_apply_path_steps(
    const SokobanSolver* solver,
    BitboardMap* sim_map,
    Position* curr_p,
    Position* sim_boxes,
    Entity* sim_bombs,
    int* bomb_count,
    const Direction* path,
    uint16_t path_len
);
FAST_OCRAM_FUNC static bool sq_verify_full_path(
    const SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs,
    const Direction* path,
    uint16_t path_len
);


static inline bool is_fatal_deadlock_with_bombs(int bx, int by, const BitboardMap* bmap, int bombs_left) {
    if ((bmap->targets[by] & (1 << bx)) != 0) return false;
    if ((bx == 1 || bx == MAP_COLS - 2) && (by == 1 || by == MAP_ROWS - 2)) return true;

    if (bx == 1 && !g_edge_target_L) return true;
    if (bx == MAP_COLS - 2 && !g_edge_target_R) return true;
    if (by == 1 && !g_edge_target_U) return true;
    if (by == MAP_ROWS - 2 && !g_edge_target_D) return true;

    if (bombs_left > 0) return false;
    return (bmap->deadlocks[by] & (1 << bx)) != 0;
}

static inline bool has_any_deadlock(const SokobanSolver* solver, const BitboardMap* bmap, const Position* current_boxes, bool* box_assigned) {
    for (int i = 0; i < solver->num_boxes; i++) {
        if (box_assigned && box_assigned[i]) continue;
        Position bp = current_boxes ? current_boxes[i] : solver->boxes[i].pos;

        if ((bmap->targets[bp.y] & (1 << bp.x)) != 0) continue;
        if (is_fatal_deadlock_with_bombs(bp.x, bp.y, bmap, solver->num_bombs)) return true;

        if (solver->num_bombs == 0) {
            int bx = bp.x, by = bp.y;
            uint16_t active_curr = bmap->boxes[by];
            uint16_t active_up   = bmap->boxes[by-1];
            uint16_t active_dn   = bmap->boxes[by+1];

            uint16_t obs_curr = bmap->walls[by] | bmap->bombs[by] | active_curr | (1 << bx);
            uint16_t obs_up   = bmap->walls[by-1] | bmap->bombs[by-1] | active_up;
            uint16_t obs_dn   = bmap->walls[by+1] | bmap->bombs[by+1] | active_dn;

            if ((obs_curr & (1 << (bx-1))) && (obs_up & (1 << bx)) && (obs_up & (1 << (bx-1)))) return true;
            if ((obs_curr & (1 << (bx+1))) && (obs_up & (1 << bx)) && (obs_up & (1 << (bx+1)))) return true;
            if ((obs_curr & (1 << (bx-1))) && (obs_dn & (1 << bx)) && (obs_dn & (1 << (bx-1)))) return true;
            if ((obs_curr & (1 << (bx+1))) && (obs_dn & (1 << bx)) && (obs_dn & (1 << (bx+1)))) return true;
        }
    }
    return false;
}

static inline uint16_t first_push_player_lower_bound(const BitboardMap* temp_map, Position player, Position box, Position target) {
    if (pos_equal(box, target)) return 0;

    uint16_t best = 0xFFFFu;
    for (int d = 0; d < 4; d++) {
        int stance_x = box.x - DIRECTIONS[d].dx;
        int stance_y = box.y - DIRECTIONS[d].dy;
        int dest_x = box.x + DIRECTIONS[d].dx;
        int dest_y = box.y + DIRECTIONS[d].dy;
        if (!is_in_bounds(stance_x, stance_y) || !is_in_bounds(dest_x, dest_y)) continue;

        uint16_t stance_bit = bit_mask_at(stance_x);
        uint16_t dest_bit = bit_mask_at(dest_x);
        if ((map_blocked_row(temp_map, stance_y) & stance_bit) != 0) continue;
        if ((map_blocked_row(temp_map, dest_y) & dest_bit) != 0) continue;

        uint16_t dist = manhattan_distance(player, (Position){(uint8_t)stance_x, (uint8_t)stance_y});
        if (dist < best) best = dist;
    }
    return best;
}
static inline bool box_can_reach_target_fast(const BitboardMap* temp_map, Position start, Position target) {
    if (start.x == target.x && start.y == target.y) return true;

    uint16_t target_bit = bit_mask_at(target.x);
    if (((temp_map->walls[target.y] | temp_map->bombs[target.y] | temp_map->boxes[target.y]) & target_bit) != 0) {
        return false;
    }

    const uint16_t interior_cols_mask = (uint16_t)(((uint16_t)1u << (MAP_COLS - 1)) - 2u);
    uint16_t blocked[MAP_ROWS];
    uint16_t frontier[MAP_ROWS] = {0};
    uint16_t next[MAP_ROWS];

    for (int y = 0; y < MAP_ROWS; y++) {
        blocked[y] = (uint16_t)(temp_map->walls[y] | temp_map->bombs[y] | temp_map->boxes[y]);
    }

    blocked[start.y] &= (uint16_t)(~bit_mask_at(start.x));
    frontier[start.y] = bit_mask_at(start.x);

    do {
        bool changed = false;
        memcpy(next, frontier, sizeof(next));

        for (int y = 1; y < MAP_ROWS - 1; y++) {
            uint16_t expanded = frontier[y];
            expanded |= (uint16_t)((expanded << 1) | (expanded >> 1));
            expanded |= frontier[y - 1] | frontier[y + 1];
            expanded &= interior_cols_mask;
            expanded &= (uint16_t)(~blocked[y]);

            if (expanded != frontier[y]) {
                next[y] = expanded;
                changed = true;
            }
        }

        if ((next[target.y] & target_bit) != 0) return true;

        memcpy(frontier, next, sizeof(frontier));
        if (!changed) break;
    } while (true);

    return false;
}

static void build_player_reach_mask_with_box(const BitboardMap* bmap, Position start, Position box, uint16_t reach[MAP_ROWS]) {
    memset(reach, 0, sizeof(uint16_t) * MAP_ROWS);
    if (!is_in_bounds(start.x, start.y)) return;

    uint16_t blocked[MAP_ROWS];
    for (int y = 0; y < MAP_ROWS; y++) {
        blocked[y] = map_blocked_row(bmap, y);
    }
    blocked[box.y] |= bit_mask_at(box.x);
    if ((blocked[start.y] & bit_mask_at(start.x)) != 0) return;

    Position* queue = g_bfs_queue;
    int head = 0;
    int tail = 0;
    reach[start.y] = bit_mask_at(start.x);
    queue[tail++] = start;

    while (head < tail) {
        Position current = queue[head++];
        for (int d = 0; d < 4; d++) {
            int nx = (int)current.x + DIRECTIONS[d].dx;
            int ny = (int)current.y + DIRECTIONS[d].dy;
            if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;

            uint16_t bit = bit_mask_at(nx);
            if ((blocked[ny] & bit) != 0 || (reach[ny] & bit) != 0) continue;
            reach[ny] |= bit;
            if (tail < MAP_ROWS * MAP_COLS) queue[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }
}

static void build_box_push_reach_mask(const BitboardMap* bmap, Position start_player, Position start_box,
                                      uint16_t box_reach[MAP_ROWS]) {
    memset(box_reach, 0, sizeof(uint16_t) * MAP_ROWS);

    memset(g_push_filter_seen_bits, 0, sizeof(g_push_filter_seen_bits));

    int head = 0;
    int tail = 0;
    uint16_t start_box_idx = POS_TO_IDX(start_box);
    uint16_t start_player_idx = POS_TO_IDX(start_player);
    push_filter_seen_set(start_box_idx, start_player_idx);
    g_push_filter_box_q[tail] = start_box_idx;
    g_push_filter_player_q[tail] = start_player_idx;
    tail++;
    box_reach[start_box.y] |= bit_mask_at(start_box.x);

    uint16_t reach[MAP_ROWS];
    while (head < tail) {
        Position box = IDX_TO_POS(g_push_filter_box_q[head]);
        Position player = IDX_TO_POS(g_push_filter_player_q[head]);
        head++;

        build_player_reach_mask_with_box(bmap, player, box, reach);

        /* A fixed box has identical legal pushes from every player cell in one component. */
        uint16_t box_idx = POS_TO_IDX(box);
        for (int y = 0; y < MAP_ROWS; y++) {
            uint16_t row = reach[y];
            while (row != 0) {
                int x = __builtin_ctz((unsigned int)row);
                push_filter_seen_set(box_idx, (uint16_t)(y * MAP_COLS + x));
                row &= (uint16_t)(row - 1u);
            }
        }

        for (int d = 0; d < 4; d++) {
            int stance_x = box.x - DIRECTIONS[d].dx;
            int stance_y = box.y - DIRECTIONS[d].dy;
            int dest_x = box.x + DIRECTIONS[d].dx;
            int dest_y = box.y + DIRECTIONS[d].dy;
            if (!is_in_bounds(stance_x, stance_y) || !is_in_bounds(dest_x, dest_y)) continue;
            if ((reach[stance_y] & bit_mask_at(stance_x)) == 0) continue;

            uint16_t dest_bit = bit_mask_at(dest_x);
            if ((map_blocked_row(bmap, dest_y) & dest_bit) != 0) continue;

            bool dest_is_target = (bmap->targets[dest_y] & dest_bit) != 0;
            if (!dest_is_target && (bmap->deadlocks[dest_y] & dest_bit) != 0) continue;

            Position dest = {(uint8_t)dest_x, (uint8_t)dest_y};
            uint16_t next_box_idx = POS_TO_IDX(dest);
            uint16_t next_player_idx = POS_TO_IDX(box);
            if (push_filter_seen_test(next_box_idx, next_player_idx)) continue;
            push_filter_seen_set(next_box_idx, next_player_idx);
            box_reach[dest_y] |= dest_bit;
            if (tail >= PUSH_FILTER_MAX_QUEUE) return;
            g_push_filter_box_q[tail] = next_box_idx;
            g_push_filter_player_q[tail] = next_player_idx;
            tail++;
        }
    }
}
FAST_RAM_FUNC static void solve_permutation_dfs(SokobanSolver* solver, int depth, int macro_depth, Position current_player, Position* current_boxes, bool* box_assigned, bool* target_filled, Direction* current_path, uint16_t current_steps, uint32_t current_hash) {
    
    
    if (g_d1_tail_search && g_d1_tail_dfs_nodes++ >= SQ_D1_TAIL_DFS_MAX_NODES) return;

    if (g_dfs_first_solution_only && solver->best_steps != 0xFFFF) return;

    if (depth == solver->num_boxes) {
        if (current_steps < solver->best_steps) {
            solver->best_steps = current_steps;
            solver->best_path_len = current_steps;
            memcpy(solver->best_path, current_path, current_steps * sizeof(Direction));
        }
        return;
    }
    if (current_steps >= solver->best_steps) {
        return;
    }
    if (depth >= MAX_BOXES) {
        return;
    }

    BitboardMap* temp_map = &g_dfs_temp_map[depth];
    DfsFrameScratch* frame = &g_dfs_frame_scratch[depth];

    if (!g_sandbox_mode) {
        int tt_status = lookup_transposition_table(TT_DOMAIN_DFS, current_hash, current_steps, solver->num_bombs);
        if (tt_status != TT_STATUS_EMPTY) {
            return;
        }
        store_transposition_table(TT_DOMAIN_DFS, current_hash, current_steps, solver->num_bombs, TT_STATUS_DEADEND);
    }

    uint8_t collision_mask = MASK_WALL | MASK_BOMB | MASK_BOX;

    int* box_order = frame->box_order;
    uint16_t* box_macro_dist = frame->box_macro_dist;
    uint16_t* box_player_dist = frame->box_player_dist;
    const bool has_large_box_set = solver_has_large_box_set(solver);
    int avail_count = 0;
    for (int i = 0; i < solver->num_boxes; i++) {
        if (!box_assigned[i]) {
            box_order[avail_count++] = i;
            box_player_dist[i] = manhattan_distance(current_player, current_boxes[i]);
            box_macro_dist[i] = macro_distance_to_target(macro_depth, current_boxes[i]);
        }
    }

    for (int a = 0; a < avail_count - 1; a++) {
        for (int b = a + 1; b < avail_count; b++) {
            int box_a_idx = box_order[a];
            int box_b_idx = box_order[b];

            int dist_p_a = box_player_dist[box_a_idx];
            int dist_p_b = box_player_dist[box_b_idx];

            uint16_t raw_dist_t_a = box_macro_dist[box_a_idx];
            uint16_t raw_dist_t_b = box_macro_dist[box_b_idx];
            uint16_t dist_t_a = (raw_dist_t_a == 0xFFFF) ? 999 : raw_dist_t_a;
            uint16_t dist_t_b = (raw_dist_t_b == 0xFFFF) ? 999 : raw_dist_t_b;

            int score_a = dist_p_a * 10 + dist_t_a * 15;
            int score_b = dist_p_b * 10 + dist_t_b * 15;



            if (score_a > score_b) {
                int temp = box_order[a];
                box_order[a] = box_order[b];
                box_order[b] = temp;
            }
        }
    }

    Position* heuristic_boxes = frame->heuristic_boxes;
    Position* heuristic_targets = frame->heuristic_targets;
    uint8_t* heuristic_box_indices = frame->heuristic_box_indices;
    uint8_t* heuristic_target_indices = frame->heuristic_target_indices;
    int heuristic_num_boxes = 0;
    int heuristic_num_targets = 0;
    uint16_t min_player_to_box = 0xFFFF;

    for (int i = 0; i < solver->num_boxes; i++) {
        if (!box_assigned[i]) {
            uint16_t p_dist = box_player_dist[i];
            if (p_dist < min_player_to_box) min_player_to_box = p_dist;

            heuristic_boxes[heuristic_num_boxes] = current_boxes[i];
            heuristic_box_indices[heuristic_num_boxes] = (uint8_t)i;
            heuristic_num_boxes++;
        }
    }

    for (int t = 0; t < solver->num_targets; t++) {
        if (!target_filled[t]) {
            heuristic_targets[heuristic_num_targets] = solver->targets[t].pos;
            heuristic_target_indices[heuristic_num_targets] = (uint8_t)t;
            heuristic_num_targets++;
        }
    }

    if (heuristic_num_boxes != heuristic_num_targets) {
        return;
    }

    bool saved_target_dist_heuristic = g_use_target_dist_heuristic;
    g_use_target_dist_heuristic = true;
    uint16_t estimated_remaining_cost = compute_perfect_remaining_heuristic(
        solver,
        heuristic_boxes,
        heuristic_box_indices,
        heuristic_targets,
        heuristic_target_indices,
        heuristic_num_boxes
    );
    g_use_target_dist_heuristic = saved_target_dist_heuristic;

    if (estimated_remaining_cost == 0xFFFF) {
        return;
    }

    if (min_player_to_box != 0xFFFF && min_player_to_box > 0) {
        estimated_remaining_cost = (uint16_t)(estimated_remaining_cost + min_player_to_box - 1);
    }

    if ((uint32_t)current_steps + estimated_remaining_cost >= solver->best_steps) {
        return;
    }
    for (int k = 0; k < avail_count; k++) {
        int i = box_order[k];

        clear_bit(temp_map->boxes, current_boxes[i].x, current_boxes[i].y);

        if (box_macro_dist[i] == 0xFFFF) {
            set_box_bit(temp_map, current_boxes[i].x, current_boxes[i].y);
            continue;
        }

        Position* available_targets = g_dfs_available_targets[depth];
        uint8_t* available_target_indices = frame->available_target_indices;
        int num_available = 0;
        for (int t = 0; t < solver->num_targets; t++) {
            if (!target_filled[t]) {
                if (solver->strict_target_mode) {
                    if (solver->targets[t].id == solver->boxes[i].id ||
                        solver->boxes[i].id == -1 ||
                        solver->targets[t].id == -1) {
                        available_target_indices[num_available] = (uint8_t)t;
                        available_targets[num_available++] = solver->targets[t].pos;
                    }
                } else {
                    available_target_indices[num_available] = (uint8_t)t;
                    available_targets[num_available++] = solver->targets[t].pos;
                }
            }
        }

        for (int a = 0; a < num_available - 1; a++) {
            for (int b = a + 1; b < num_available; b++) {
                int dist_a = abs(available_targets[a].x - current_boxes[i].x) + abs(available_targets[a].y - current_boxes[i].y);
                int dist_b = abs(available_targets[b].x - current_boxes[i].x) + abs(available_targets[b].y - current_boxes[i].y);



                if (dist_a > dist_b) {
                    Position temp = available_targets[a];
                    available_targets[a] = available_targets[b];
                    available_targets[b] = temp;
                    uint8_t temp_idx = available_target_indices[a];
                    available_target_indices[a] = available_target_indices[b];
                    available_target_indices[b] = temp_idx;
                }
            }
        }

        uint16_t* push_reach_mask = frame->push_reach_mask;
        if (g_enable_push_reach_filter) {
            build_box_push_reach_mask(temp_map, current_player, current_boxes[i], push_reach_mask);
        }

        for (int t_idx = 0; t_idx < num_available; t_idx++) {
            
            Position target_pos = available_targets[t_idx];
            uint8_t target_real_idx = available_target_indices[t_idx];
            if (g_enable_push_reach_filter && (push_reach_mask[target_pos.y] & bit_mask_at(target_pos.x)) == 0) {
                
                continue;
            }

            uint16_t box_target_lower = g_target_dist_field[target_real_idx][current_boxes[i].y][current_boxes[i].x];
            if (box_target_lower == 0xFFFF) {
                
                continue;
            }
            if ((uint32_t)current_steps + box_target_lower >= solver->best_steps) {
                
                continue;
            }
            if (solver->best_steps != 0xFFFF) {
                uint16_t first_push_lower = first_push_player_lower_bound(temp_map, current_player, current_boxes[i], target_pos);
                if (first_push_lower == 0xFFFFu) {
                    
                    continue;
                }
                if ((uint32_t)current_steps + box_target_lower + first_push_lower >= solver->best_steps) {
                    
                    continue;
                }
            }

            if (has_large_box_set && solver->best_steps != 0xFFFF) {
                Position* candidate_boxes = frame->candidate_boxes;
                Position* candidate_targets = frame->candidate_targets;
                uint8_t* candidate_box_indices = frame->candidate_box_indices;
                uint8_t* candidate_target_indices = frame->candidate_target_indices;
                int candidate_num_boxes = 0;
                int candidate_num_targets = 0;

                for (int rb = 0; rb < solver->num_boxes; rb++) {
                    if (!box_assigned[rb] && rb != i) {
                        candidate_boxes[candidate_num_boxes] = current_boxes[rb];
                        candidate_box_indices[candidate_num_boxes] = (uint8_t)rb;
                        candidate_num_boxes++;
                    }
                }
                for (int rt = 0; rt < solver->num_targets; rt++) {
                    if (!target_filled[rt] && rt != target_real_idx) {
                        candidate_targets[candidate_num_targets] = solver->targets[rt].pos;
                        candidate_target_indices[candidate_num_targets] = (uint8_t)rt;
                        candidate_num_targets++;
                    }
                }

                if (candidate_num_boxes != candidate_num_targets) {
                    continue;
                }

                bool saved_candidate_target_dist = g_use_target_dist_heuristic;
                g_use_target_dist_heuristic = true;
                uint16_t candidate_remaining_cost = compute_perfect_remaining_heuristic(
                    solver,
                    candidate_boxes,
                    candidate_box_indices,
                    candidate_targets,
                    candidate_target_indices,
                    candidate_num_boxes
                );
                g_use_target_dist_heuristic = saved_candidate_target_dist;

                if (candidate_remaining_cost == 0xFFFF) {
                    
                    continue;
                }
                if ((uint32_t)current_steps + box_target_lower + candidate_remaining_cost >= solver->best_steps) {
                    
                    continue;
                }
            }

            if (box_macro_dist[i] == 0xFFFF) {
                continue;
            }
            if (!solver->is_scanning && !box_can_reach_target_fast(temp_map, current_boxes[i], target_pos)) {
                
                continue;
            }

            uint16_t* orig_deadlocks = frame->orig_deadlocks;
            uint16_t* orig_targets = frame->orig_targets;
            memcpy(orig_deadlocks, temp_map->deadlocks, sizeof(frame->orig_deadlocks));
            memcpy(orig_targets, temp_map->targets, sizeof(frame->orig_targets));

            for (int t = 0; t < solver->num_targets; t++) {
                if (target_filled[t]) {
                    temp_map->targets[solver->targets[t].pos.y] &= ~(1 << solver->targets[t].pos.x);
                } else if (!pos_equal(solver->targets[t].pos, target_pos)) {
                    bool is_dangerous = false;
                    if (!solver->strict_target_mode) {
                        is_dangerous = true;
                    } else {
                        int b_id = solver->boxes[i].id;
                        int t_id = solver->targets[t].id;
                        if (b_id == -1 || t_id == -1 || b_id == t_id) is_dangerous = true;
                    }

                    if (is_dangerous) {
                        temp_map->deadlocks[solver->targets[t].pos.y] |= (1 << solver->targets[t].pos.x);
                    }
                    temp_map->targets[solver->targets[t].pos.y] &= ~(1 << solver->targets[t].pos.x);
                }
            }

            Direction* path = g_dfs_path[depth];
            uint16_t len = 0;

            uint16_t astar_step_limit = 0xFFFF;
            if (solver->best_steps != 0xFFFF && solver->best_steps > current_steps) {
                astar_step_limit = (uint16_t)(solver->best_steps - current_steps);
            }
            g_astar_max_steps = astar_step_limit;

            hash_table_clear();
            
            bool success = astar_solve_single_box_mask(
                solver->heap, solver->closed_list, temp_map, current_player,
                current_boxes[i], target_pos,
                collision_mask, path, &len, macro_depth, ROUTE_BOX_NORMAL
            );
            memcpy(temp_map->deadlocks, orig_deadlocks, sizeof(frame->orig_deadlocks));
            memcpy(temp_map->targets, orig_targets, sizeof(frame->orig_targets));
            if (success && current_steps + len < MAX_PATH_LENGTH) {
                Position original_box_pos = current_boxes[i];
                current_boxes[i] = target_pos;
                box_assigned[i] = true;

                int real_target_idx = -1;

                for (int t = 0; t < solver->num_targets; t++) {
                    if (!target_filled[t] && pos_equal(solver->targets[t].pos, target_pos)) {
                        real_target_idx = t;
                        target_filled[t] = true;
                        break;
                    }
                }

                memcpy(&current_path[current_steps], path, len * sizeof(Direction));
                Position next_player = current_player;
                for (int s = 0; s < len; s++) { next_player.x += path[s].dx; next_player.y += path[s].dy; }

                uint32_t next_hash = current_hash
                                     ^ ZOBRIST_PLAYER[Z_IDX(current_player.x, current_player.y)]
                                     ^ ZOBRIST_PLAYER[Z_IDX(next_player.x, next_player.y)]
                                     ^ ZOBRIST_BOX[Z_IDX(original_box_pos.x, original_box_pos.y)];
                uint16_t next_steps = (uint16_t)(current_steps + len);
                if (depth + 1 == solver->num_boxes) {
                    if (next_steps < solver->best_steps) {
                        solver->best_steps = next_steps;
                        solver->best_path_len = next_steps;
                        memcpy(solver->best_path, current_path, next_steps * sizeof(Direction));
                    }
                } else {
                    if (depth + 1 < MAX_BOXES) {
                        g_dfs_temp_map[depth + 1] = *temp_map;
                    }
                    solve_permutation_dfs(solver, depth + 1, macro_depth, next_player, current_boxes, box_assigned, target_filled, current_path, next_steps, next_hash);
                }

                current_boxes[i] = original_box_pos;
                box_assigned[i] = false;

                if (real_target_idx != -1) target_filled[real_target_idx] = false;

                if (g_dfs_first_solution_only && solver->best_steps != 0xFFFF) {
                    set_box_bit(temp_map, current_boxes[i].x, current_boxes[i].y);
                    return;
                }
            }
        }
        set_box_bit(temp_map, current_boxes[i].x, current_boxes[i].y);
    }
}

static bool assignment_beam_path_store(
    PackedDirByte packed[ASSIGNMENT_BEAM_PATH_BYTES],
    const Direction* path,
    uint16_t len
) {
    if (!packed || (!path && len > 0) || len > MAX_PATH_LENGTH) return false;
    memset(packed, 0, ASSIGNMENT_BEAM_PATH_BYTES);
    for (uint16_t i = 0; i < len; i++) {
        uint8_t code = direction_index(path[i]);
        if (code >= 4u) return false;
        uint16_t byte_idx = (uint16_t)(i >> 2);
        uint8_t shift = (uint8_t)((i & 0x03u) << 1);
        packed[byte_idx] = (uint8_t)(packed[byte_idx] | (uint8_t)(code << shift));
    }
    return true;
}

static bool assignment_beam_path_append(
    PackedDirByte packed[ASSIGNMENT_BEAM_PATH_BYTES],
    uint16_t offset,
    const Direction* path,
    uint16_t len
) {
    if (!packed || (!path && len > 0) || (uint32_t)offset + len > MAX_PATH_LENGTH) return false;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t code = direction_index(path[i]);
        if (code >= 4u) return false;
        uint16_t step = (uint16_t)(offset + i);
        uint16_t byte_idx = (uint16_t)(step >> 2);
        uint8_t shift = (uint8_t)((step & 0x03u) << 1);
        packed[byte_idx] = (uint8_t)(
            (packed[byte_idx] & (uint8_t)~(0x03u << shift)) |
            (uint8_t)(code << shift)
        );
    }
    return true;
}

static void assignment_beam_path_load(
    const PackedDirByte packed[ASSIGNMENT_BEAM_PATH_BYTES],
    Direction* path,
    uint16_t len
) {
    if (!packed || !path || len > MAX_PATH_LENGTH) return;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t byte_idx = (uint16_t)(i >> 2);
        uint8_t shift = (uint8_t)((i & 0x03u) << 1);
        path[i] = direction_from_index((uint8_t)((packed[byte_idx] >> shift) & 0x03u));
    }
}

static bool assignment_beam_state_same(
    const AssignmentBeamState* lhs,
    const AssignmentBeamState* rhs
) {
    return lhs->assigned_mask == rhs->assigned_mask &&
           lhs->filled_mask == rhs->filled_mask &&
           pos_equal(lhs->player, rhs->player);
}

static bool assignment_beam_state_better(
    const AssignmentBeamState* lhs,
    const AssignmentBeamState* rhs
) {
    if (lhs->rank != rhs->rank) return lhs->rank < rhs->rank;
    if (lhs->steps != rhs->steps) return lhs->steps < rhs->steps;
    if (lhs->filled_mask != rhs->filled_mask) return lhs->filled_mask < rhs->filled_mask;
    if (lhs->assigned_mask != rhs->assigned_mask) return lhs->assigned_mask < rhs->assigned_mask;
    if (lhs->player.y != rhs->player.y) return lhs->player.y < rhs->player.y;
    if (lhs->player.x != rhs->player.x) return lhs->player.x < rhs->player.x;
    return lhs->stable_order < rhs->stable_order;
}

static void assignment_beam_consider(
    AssignmentBeamState states[SOKOBAN_ASSIGNMENT_BEAM_WIDTH],
    uint16_t* count,
    const AssignmentBeamState* candidate
) {
    if (!states || !count || !candidate) return;

    for (uint16_t i = 0; i < *count; i++) {
        if (!assignment_beam_state_same(&states[i], candidate)) continue;
        if (assignment_beam_state_better(candidate, &states[i])) states[i] = *candidate;
        return;
    }

    if (*count < g_assignment_beam_active_width) {
        states[*count] = *candidate;
        (*count)++;
        return;
    }

    uint16_t worst = 0;
    for (uint16_t i = 1; i < *count; i++) {
        if (assignment_beam_state_better(&states[worst], &states[i])) worst = i;
    }
    if (assignment_beam_state_better(candidate, &states[worst])) states[worst] = *candidate;
}

static void assignment_beam_sort(
    AssignmentBeamState states[SOKOBAN_ASSIGNMENT_BEAM_WIDTH],
    uint16_t count
) {
    for (uint16_t i = 1; i < count; i++) {
        AssignmentBeamState value = states[i];
        uint16_t j = i;
        while (j > 0 && assignment_beam_state_better(&value, &states[j - 1])) {
            states[j] = states[j - 1];
            j--;
        }
        states[j] = value;
    }
}

static uint16_t assignment_beam_remaining_lower_bound(
    const SokobanSolver* solver,
    const Position root_boxes[MAX_BOXES],
    uint16_t assigned_mask,
    uint16_t filled_mask,
    Position player
) {
    Position rem_boxes[MAX_BOXES];
    Position rem_targets[MAX_TARGETS];
    uint8_t rem_box_indices[MAX_BOXES];
    uint8_t rem_target_indices[MAX_TARGETS];
    int num_boxes = 0;
    int num_targets = 0;
    uint16_t min_player_to_box = 0xFFFF;

    for (int i = 0; i < solver->num_boxes; i++) {
        if ((assigned_mask & (uint16_t)(1u << i)) != 0) continue;
        rem_boxes[num_boxes] = root_boxes[i];
        rem_box_indices[num_boxes] = (uint8_t)i;
        num_boxes++;
        uint16_t player_dist = manhattan_distance(player, root_boxes[i]);
        if (player_dist < min_player_to_box) min_player_to_box = player_dist;
    }
    for (int t = 0; t < solver->num_targets; t++) {
        if ((filled_mask & (uint16_t)(1u << t)) != 0) continue;
        rem_targets[num_targets] = solver->targets[t].pos;
        rem_target_indices[num_targets] = (uint8_t)t;
        num_targets++;
    }
    if (num_boxes != num_targets) return 0xFFFF;
    if (num_boxes == 0) return 0;

    bool saved_target_dist_heuristic = g_use_target_dist_heuristic;
    g_use_target_dist_heuristic = true;
    uint16_t lower = compute_perfect_remaining_heuristic(
        solver,
        rem_boxes,
        rem_box_indices,
        rem_targets,
        rem_target_indices,
        num_boxes
    );
    g_use_target_dist_heuristic = saved_target_dist_heuristic;
    if (lower == 0xFFFF) return 0xFFFF;

    if (min_player_to_box != 0xFFFF && min_player_to_box > 0) {
        uint32_t with_player = (uint32_t)lower + min_player_to_box - 1u;
        if (with_player >= 0xFFFFu) return 0xFFFF;
        lower = (uint16_t)with_player;
    }
    return lower;
}

static void assignment_beam_rebuild_box_map(
    const SokobanSolver* solver,
    const Position root_boxes[MAX_BOXES],
    uint16_t assigned_mask,
    int active_box
) {
    g_assignment_beam_box_map = solver->bmap;
    memset(g_assignment_beam_box_map.boxes, 0, sizeof(g_assignment_beam_box_map.boxes));
    for (int i = 0; i < solver->num_boxes; i++) {
        if (i == active_box || (assigned_mask & (uint16_t)(1u << i)) != 0) continue;
        set_box_bit(&g_assignment_beam_box_map, root_boxes[i].x, root_boxes[i].y);
    }
}

static bool assignment_refinement_is_eligible(const SokobanSolver* solver) {
    return solver &&
           !g_assignment_refinement_disabled &&
           !solver->is_scanning &&
           !solver->strict_target_mode &&
           !g_sandbox_mode &&
           solver->num_boxes == MAX_BOXES;
}

static bool assignment_beam_first_is_eligible(const SokobanSolver* solver) {
    return solver &&
           !g_assignment_refinement_disabled &&
           (!solver->is_scanning || g_bomb_state_beam_tail_only) &&
           !g_sandbox_mode &&
           (solver->num_bombs == 0 || g_bomb_state_beam_tail_only) &&
           solver->num_boxes > 0 &&
           solver->num_boxes <= MAX_BOXES &&
           solver->num_targets == solver->num_boxes;
}

static bool assignment_beam_is_eligible(
    const SokobanSolver* solver,
    int initial_depth
) {
    bool generalized_beam =
        g_assignment_bounded_refinement && assignment_beam_first_is_eligible(solver);
    return (assignment_refinement_is_eligible(solver) || generalized_beam) &&
           !g_dfs_first_solution_only &&
           initial_depth < solver->num_boxes;
}

static bool assignment_beam_candidate_replay_valid(
    const SokobanSolver* solver,
    Position initial_player,
    const Position root_boxes[MAX_BOXES],
    uint16_t initially_assigned_mask,
    uint16_t initially_filled_mask,
    const Direction* path,
    uint16_t path_len
) {
    if (!solver || !root_boxes || !path || path_len >= MAX_PATH_LENGTH) return false;

    g_assignment_beam_replay_map = solver->bmap;
    for (int i = 0; i < solver->num_boxes; i++) {
        g_assignment_beam_replay_boxes[i] = solver->boxes[i];
        g_assignment_beam_replay_boxes[i].pos = root_boxes[i];
        if ((initially_assigned_mask & (uint16_t)(1u << i)) != 0) {
            clear_bit(
                g_assignment_beam_replay_map.boxes,
                root_boxes[i].x,
                root_boxes[i].y
            );
            g_assignment_beam_replay_boxes[i].pos = (Position){0xFF, 0xFF};
            g_assignment_beam_replay_boxes[i].is_active = false;
        }
    }
    for (int i = solver->num_boxes; i < MAX_BOXES; i++) {
        memset(&g_assignment_beam_replay_boxes[i], 0, sizeof(Entity));
        g_assignment_beam_replay_boxes[i].pos = (Position){0xFF, 0xFF};
    }
    for (int t = 0; t < solver->num_targets; t++) {
        if ((initially_filled_mask & (uint16_t)(1u << t)) == 0) continue;
        clear_bit(
            g_assignment_beam_replay_map.targets,
            solver->targets[t].pos.x,
            solver->targets[t].pos.y
        );
    }

    PathReplayOptions replay_options = path_replay_default_options();
    replay_options.mode = PATH_REPLAY_STRICT_VALIDATE;
    if (!path_replay_run(
            solver,
            &g_assignment_beam_replay_map,
            initial_player,
            g_assignment_beam_replay_boxes,
            solver->num_boxes,
            solver->bombs,
            solver->num_bombs,
            path,
            path_len,
            &replay_options,
            &g_assignment_beam_replay_result) ||
        !g_assignment_beam_replay_result.ok ||
        g_assignment_beam_replay_result.consumed_len != path_len) {
        return false;
    }

    for (int t = 0; t < solver->num_targets; t++) {
        Position target = solver->targets[t].pos;
        if (get_bit(g_assignment_beam_replay_result.final_state.map.targets, target.x, target.y)) {
            return false;
        }
    }
    return true;
}

static AssignmentBeamResult __attribute__((noinline)) solve_assignment_beam(
    SokobanSolver* solver,
    int initial_depth,
    int macro_depth,
    Position current_player,
    const Position root_boxes[MAX_BOXES],
    const bool box_assigned[MAX_BOXES],
    const bool target_filled[MAX_TARGETS],
    const Direction* current_path,
    uint16_t current_steps
) {
    if (!assignment_beam_is_eligible(solver, initial_depth)) {
        return ASSIGNMENT_BEAM_INELIGIBLE;
    }
    uint16_t saved_astar_max_steps = g_astar_max_steps;

    AssignmentBeamState* current = g_assignment_beam_states[0];
    AssignmentBeamState* next = g_assignment_beam_states[1];
    memset(current, 0, sizeof(g_assignment_beam_states[0]));
    memset(next, 0, sizeof(g_assignment_beam_states[1]));

    AssignmentBeamState root;
    memset(&root, 0, sizeof(root));
    root.player = current_player;
    root.steps = current_steps;
    for (int i = 0; i < solver->num_boxes; i++) {
        if (box_assigned[i]) root.assigned_mask |= (uint16_t)(1u << i);
    }
    for (int t = 0; t < solver->num_targets; t++) {
        if (target_filled[t]) root.filled_mask |= (uint16_t)(1u << t);
    }
    if (!assignment_beam_path_store(root.path, current_path, current_steps)) {
        return ASSIGNMENT_BEAM_EXHAUSTED;
    }
    root.lower_bound = assignment_beam_remaining_lower_bound(
        solver, root_boxes, root.assigned_mask, root.filled_mask, root.player
    );
    if (root.lower_bound == 0xFFFF) return ASSIGNMENT_BEAM_EXHAUSTED;
    root.rank = (uint32_t)root.steps + root.lower_bound;
    root.stable_order = 0;
    current[0] = root;
    uint16_t current_count = 1;
    uint32_t stable_order = 1;

    for (int assigned_count = initial_depth; assigned_count < solver->num_boxes; assigned_count++) {
        uint16_t next_count = 0;
        DfsFrameScratch* frame = &g_dfs_frame_scratch[assigned_count];

        for (uint16_t state_idx = 0; state_idx < current_count; state_idx++) {
            const AssignmentBeamState* state = &current[state_idx];
            if (g_assignment_exclusive_upper_bound != UINT16_MAX &&
                state->rank >= g_assignment_exclusive_upper_bound) {
                continue;
            }
            int avail_count = 0;

            for (int i = 0; i < solver->num_boxes; i++) {
                if ((state->assigned_mask & (uint16_t)(1u << i)) != 0) continue;
                frame->box_order[avail_count++] = i;
                frame->box_player_dist[i] = manhattan_distance(state->player, root_boxes[i]);
                frame->box_macro_dist[i] = macro_distance_to_target(macro_depth, root_boxes[i]);
            }

            for (int a = 0; a < avail_count - 1; a++) {
                for (int b = a + 1; b < avail_count; b++) {
                    int box_a = frame->box_order[a];
                    int box_b = frame->box_order[b];
                    uint16_t macro_a = frame->box_macro_dist[box_a] == 0xFFFF
                        ? 999u : frame->box_macro_dist[box_a];
                    uint16_t macro_b = frame->box_macro_dist[box_b] == 0xFFFF
                        ? 999u : frame->box_macro_dist[box_b];
                    uint32_t score_a = (uint32_t)frame->box_player_dist[box_a] * 10u +
                                       (uint32_t)macro_a * 15u;
                    uint32_t score_b = (uint32_t)frame->box_player_dist[box_b] * 10u +
                                       (uint32_t)macro_b * 15u;
                    if (score_a > score_b) {
                        int swap = frame->box_order[a];
                        frame->box_order[a] = frame->box_order[b];
                        frame->box_order[b] = swap;
                    }
                }
            }

            for (int order_idx = 0; order_idx < avail_count; order_idx++) {
                int box_idx = frame->box_order[order_idx];
                if (frame->box_macro_dist[box_idx] == 0xFFFF) continue;

                int target_count = 0;
                for (int t = 0; t < solver->num_targets; t++) {
                    if ((state->filled_mask & (uint16_t)(1u << t)) != 0) continue;
                    if (!heuristic_target_matches_box(solver, (uint8_t)box_idx, (uint8_t)t)) continue;
                    g_dfs_available_targets[assigned_count][target_count] = solver->targets[t].pos;
                    frame->available_target_indices[target_count] = (uint8_t)t;
                    target_count++;
                }

                for (int a = 0; a < target_count - 1; a++) {
                    for (int b = a + 1; b < target_count; b++) {
                        int dist_a = manhattan_distance(
                            g_dfs_available_targets[assigned_count][a], root_boxes[box_idx]
                        );
                        int dist_b = manhattan_distance(
                            g_dfs_available_targets[assigned_count][b], root_boxes[box_idx]
                        );
                        if (dist_a > dist_b) {
                            Position pos_swap = g_dfs_available_targets[assigned_count][a];
                            g_dfs_available_targets[assigned_count][a] =
                                g_dfs_available_targets[assigned_count][b];
                            g_dfs_available_targets[assigned_count][b] = pos_swap;
                            uint8_t idx_swap = frame->available_target_indices[a];
                            frame->available_target_indices[a] = frame->available_target_indices[b];
                            frame->available_target_indices[b] = idx_swap;
                        }
                    }
                }

                assignment_beam_rebuild_box_map(
                    solver, root_boxes, state->assigned_mask, box_idx
                );
                if (g_enable_push_reach_filter) {
                    build_box_push_reach_mask(
                        &g_assignment_beam_box_map,
                        state->player,
                        root_boxes[box_idx],
                        frame->push_reach_mask
                    );
                }

                if (g_assignment_batch_enabled && target_count > 1) {
                    Position batch_targets[MAX_TARGETS];
                    uint8_t batch_target_slots[MAX_TARGETS];
                    uint8_t batch_target_count = 0;
                    for (int target_order = 0; target_order < target_count; target_order++) {
                        uint8_t target_idx = frame->available_target_indices[target_order];
                        Position target_pos = solver->targets[target_idx].pos;
                        if (g_enable_push_reach_filter &&
                            (frame->push_reach_mask[target_pos.y] & bit_mask_at(target_pos.x)) == 0) {
                            continue;
                        }
                        if (g_target_dist_field[target_idx][root_boxes[box_idx].y][root_boxes[box_idx].x] == 0xFFFF) {
                            continue;
                        }
                        if (!g_enable_push_reach_filter &&
                            !box_can_reach_target_fast(
                                &g_assignment_beam_box_map, root_boxes[box_idx], target_pos)) {
                            continue;
                        }
                        batch_targets[batch_target_count] = target_pos;
                        batch_target_slots[batch_target_count] = target_idx;
                        batch_target_count++;
                    }

                    if (batch_target_count > 1) {
                        g_astar_max_steps = 0xFFFF;
                        AStarBatchStatus batch_status = astar_solve_single_box_targets_mask(
                            &g_assignment_beam_box_map,
                            state->player,
                            root_boxes[box_idx],
                            batch_targets,
                            batch_target_slots,
                            batch_target_count,
                            MASK_WALL | MASK_BOMB | MASK_BOX,
                            0,
                            &g_assignment_batch_result
                        );
                        if (batch_status == ASTAR_BATCH_INCOMPLETE) {
                            g_astar_max_steps = saved_astar_max_steps;
                            return ASSIGNMENT_BEAM_EXHAUSTED;
                        }
                        if (batch_status == ASTAR_BATCH_FATAL) {
                            g_astar_max_steps = saved_astar_max_steps;
                            return ASSIGNMENT_BEAM_FATAL;
                        }

                        for (uint8_t batch_index = 0;
                             batch_index < g_assignment_batch_result.candidate_count;
                             batch_index++) {
                            const AStarBatchCandidate* candidate =
                                &g_assignment_batch_result.candidates[batch_index];
                            if (candidate->status != ASTAR_BATCH_TARGET_FOUND) continue;
                            uint8_t target_idx = candidate->target_slot;
                            if (target_idx >= solver->num_targets) {
                                g_astar_max_steps = saved_astar_max_steps;
                                return ASSIGNMENT_BEAM_FATAL;
                            }
                            const Direction* segment = candidate->path;
                            uint16_t segment_len = candidate->path_len;
                            if ((uint32_t)state->steps + segment_len >= MAX_PATH_LENGTH) continue;

                            AssignmentBeamState child = *state;
                            child.steps = (uint16_t)(state->steps + segment_len);
                            child.assigned_mask |= (uint16_t)(1u << box_idx);
                            child.filled_mask |= (uint16_t)(1u << target_idx);
                            child.player = state->player;
                            for (uint16_t step = 0; step < segment_len; step++) {
                                child.player.x = (uint8_t)(child.player.x + segment[step].dx);
                                child.player.y = (uint8_t)(child.player.y + segment[step].dy);
                            }
                            if (!assignment_beam_path_append(
                                    child.path, state->steps, segment, segment_len)) {
                                continue;
                            }
                            child.lower_bound = assignment_beam_remaining_lower_bound(
                                solver,
                                root_boxes,
                                child.assigned_mask,
                                child.filled_mask,
                                child.player
                            );
                            if (child.lower_bound == 0xFFFF) continue;
                            child.rank = (uint32_t)child.steps + child.lower_bound;
                            child.stable_order = stable_order++;
                            assignment_beam_consider(next, &next_count, &child);
                        }
                        continue;
                    }
                }

                for (int target_order = 0; target_order < target_count; target_order++) {
                    uint8_t target_idx = frame->available_target_indices[target_order];
                    Position target_pos = solver->targets[target_idx].pos;
                    if (g_enable_push_reach_filter &&
                        (frame->push_reach_mask[target_pos.y] & bit_mask_at(target_pos.x)) == 0) {
                        continue;
                    }
                    if (g_target_dist_field[target_idx][root_boxes[box_idx].y][root_boxes[box_idx].x] == 0xFFFF) {
                        continue;
                    }
                    if (!box_can_reach_target_fast(
                            &g_assignment_beam_box_map, root_boxes[box_idx], target_pos)) {
                        continue;
                    }

                    g_assignment_beam_map = g_assignment_beam_box_map;
                    for (int t = 0; t < solver->num_targets; t++) {
                        if ((state->filled_mask & (uint16_t)(1u << t)) != 0) {
                            clear_bit(
                                g_assignment_beam_map.targets,
                                solver->targets[t].pos.x,
                                solver->targets[t].pos.y
                            );
                        } else if (t != target_idx) {
                            if (heuristic_target_matches_box(
                                    solver, (uint8_t)box_idx, (uint8_t)t)) {
                                set_bit(
                                    g_assignment_beam_map.deadlocks,
                                    solver->targets[t].pos.x,
                                    solver->targets[t].pos.y
                                );
                            }
                            clear_bit(
                                g_assignment_beam_map.targets,
                                solver->targets[t].pos.x,
                                solver->targets[t].pos.y
                            );
                        }
                    }

                    Direction* segment = g_dfs_path[assigned_count];
                    uint16_t segment_len = 0;
                    g_astar_max_steps = 0xFFFF;
                    hash_table_clear();
                    bool routed = astar_solve_single_box_mask(
                        solver->heap,
                        solver->closed_list,
                        &g_assignment_beam_map,
                        state->player,
                        root_boxes[box_idx],
                        target_pos,
                        MASK_WALL | MASK_BOMB | MASK_BOX,
                        segment,
                        &segment_len,
                        macro_depth,
                        ROUTE_BOX_NORMAL
                    );
                    if (!routed || (uint32_t)state->steps + segment_len >= MAX_PATH_LENGTH) continue;

                    AssignmentBeamState child = *state;
                    child.steps = (uint16_t)(state->steps + segment_len);
                    child.assigned_mask |= (uint16_t)(1u << box_idx);
                    child.filled_mask |= (uint16_t)(1u << target_idx);
                    child.player = state->player;
                    for (uint16_t step = 0; step < segment_len; step++) {
                        child.player.x = (uint8_t)(child.player.x + segment[step].dx);
                        child.player.y = (uint8_t)(child.player.y + segment[step].dy);
                    }
                    if (!assignment_beam_path_append(
                            child.path, state->steps, segment, segment_len)) {
                        continue;
                    }
                    child.lower_bound = assignment_beam_remaining_lower_bound(
                        solver,
                        root_boxes,
                        child.assigned_mask,
                        child.filled_mask,
                        child.player
                    );
                    if (child.lower_bound == 0xFFFF) continue;
                    child.rank = (uint32_t)child.steps + child.lower_bound;
                    child.stable_order = stable_order++;
                    assignment_beam_consider(next, &next_count, &child);
                }
            }
        }

        if (next_count == 0) {
            g_astar_max_steps = saved_astar_max_steps;
            return ASSIGNMENT_BEAM_EXHAUSTED;
        }
        assignment_beam_sort(next, next_count);
        AssignmentBeamState* swap = current;
        current = next;
        next = swap;
        current_count = next_count;
    }

    g_astar_max_steps = saved_astar_max_steps;
    assignment_beam_sort(current, current_count);
    for (uint16_t i = 0; i < current_count; i++) {
        const AssignmentBeamState* candidate = &current[i];
        assignment_beam_path_load(
            candidate->path,
            g_dfs_full_path_buffer,
            candidate->steps
        );
        if (!assignment_beam_candidate_replay_valid(
                solver,
                current_player,
                root_boxes,
                root.assigned_mask,
                root.filled_mask,
                g_dfs_full_path_buffer,
                candidate->steps)) {
            continue;
        }
        if (candidate->steps < solver->best_steps) {
            solver->best_steps = candidate->steps;
            solver->best_path_len = candidate->steps;
            memcpy(
                solver->best_path,
                g_dfs_full_path_buffer,
                candidate->steps * sizeof(Direction)
            );
        }
        return ASSIGNMENT_BEAM_COMPLETE;
    }
    return ASSIGNMENT_BEAM_EXHAUSTED;
}

static inline bool is_open_area(const BitboardMap* bmap, int x, int y) {
    if (x <= 0 || x >= MAP_COLS - 1 || y <= 0 || y >= MAP_ROWS - 1) return false;
    if ((bmap->walls[y] & (1 << x)) != 0) return false;

    if ((bmap->targets[y] & (1 << x)) != 0) return false;

    // ==========================================
    // ==========================================
    if ((bmap->h_tunnels[y] & (1 << x)) != 0) return false;
    if ((bmap->v_tunnels[y] & (1 << x)) != 0) return false;

    if ((bmap->h_tunnels[y] & (1 << (x - 1))) != 0) return false;
    if ((bmap->h_tunnels[y] & (1 << (x + 1))) != 0) return false;
    if ((bmap->v_tunnels[y - 1] & (1 << x)) != 0) return false;
    if ((bmap->v_tunnels[y + 1] & (1 << x)) != 0) return false;

    // ==========================================
    // ==========================================
    if (x == 1 && !g_edge_target_L) return false;
    if (x == MAP_COLS - 2 && !g_edge_target_R) return false;
    if (y == 1 && !g_edge_target_U) return false;
    if (y == MAP_ROWS - 2 && !g_edge_target_D) return false;

    if ((bmap->deadlocks[y] & (1 << x)) != 0) return false;

    return true;
}

typedef struct {
    uint8_t box_idx;
    Position target_parking;
    uint16_t push_len;
    int score;
} PocketUnblockCandidate;
static PocketUnblockCandidate g_pocket_unblock_candidates[MAX_POCKET_UNBLOCK_CANDIDATES] ALLOC_IN_OCRAM;

static int build_player_reach_mask(const BitboardMap* bmap, Position start, uint16_t reach[MAP_ROWS]) {
    memset(reach, 0, sizeof(uint16_t) * MAP_ROWS);
    if (start.x <= 0 || start.x >= MAP_COLS - 1 || start.y <= 0 || start.y >= MAP_ROWS - 1) return 0;
    if (map_is_obstructed(bmap, start.x, start.y)) return 0;

    Position* q = g_bfs_queue;
    int head = 0, tail = 0;
    int count = 0;
    q[tail++] = start;
    reach[start.y] |= bit_mask_at(start.x);

    while (head < tail) {
        Position curr = q[head++];
        count++;
        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
            uint16_t bit = bit_mask_at(nx);
            if ((reach[ny] & bit) != 0) continue;
            if ((map_blocked_row(bmap, ny) & bit) != 0) continue;
            reach[ny] |= bit;
            if (tail < MAP_ROWS * MAP_COLS) q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }
    return count;
}

static int score_player_reach_frontier(const BitboardMap* bmap, const uint16_t reach[MAP_ROWS], int* out_bombs, int* out_boxes, int* out_targets) {
    int tiles = 0;
    int bombs = 0;
    int boxes = 0;
    int targets = 0;
    uint16_t seen_bombs[MAP_ROWS] = {0};
    uint16_t seen_boxes[MAP_ROWS] = {0};

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        uint16_t row = reach[y];
        tiles += __builtin_popcount((unsigned int)row);
        targets += __builtin_popcount((unsigned int)(row & bmap->targets[y]));
        while (row) {
            int x = __builtin_ctz((unsigned int)row);
            row &= (uint16_t)(row - 1);
            for (int d = 0; d < 4; d++) {
                int nx = x + DIRECTIONS[d].dx;
                int ny = y + DIRECTIONS[d].dy;
                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                uint16_t bit = bit_mask_at(nx);
                if ((bmap->bombs[ny] & bit) != 0 && (seen_bombs[ny] & bit) == 0) {
                    seen_bombs[ny] |= bit;
                    bombs++;
                }
                if ((bmap->boxes[ny] & bit) != 0 && (seen_boxes[ny] & bit) == 0) {
                    seen_boxes[ny] |= bit;
                    boxes++;
                }
            }
        }
    }

    if (out_bombs) *out_bombs = bombs;
    if (out_boxes) *out_boxes = boxes;
    if (out_targets) *out_targets = targets;
    return tiles;
}

static bool is_pocket_parking_cell(const SokobanSolver* solver, int x, int y, Position moving_box) {
    const BitboardMap* bmap = &solver->bmap;
    if (x <= 0 || x >= MAP_COLS - 1 || y <= 0 || y >= MAP_ROWS - 1) return false;
    uint16_t bit = bit_mask_at(x);
    if ((bmap->walls[y] & bit) != 0) return false;
    if ((bmap->targets[y] & bit) != 0) return false;
    if ((bmap->bombs[y] & bit) != 0) return false;
    if ((bmap->boxes[y] & bit) != 0 && !(moving_box.x == x && moving_box.y == y)) return false;
    if ((bmap->deadlocks[y] & bit) != 0) return false;
    if (x == 1 && !g_edge_target_L) return false;
    if (x == MAP_COLS - 2 && !g_edge_target_R) return false;
    if (y == 1 && !g_edge_target_U) return false;
    if (y == MAP_ROWS - 2 && !g_edge_target_D) return false;
    return true;
}

static void insert_pocket_candidate(PocketUnblockCandidate* candidates, int* count, PocketUnblockCandidate cand) {
    if (*count < MAX_POCKET_UNBLOCK_CANDIDATES) {
        int idx = *count;
        while (idx > 0 && candidates[idx - 1].score < cand.score) {
            candidates[idx] = candidates[idx - 1];
            idx--;
        }
        candidates[idx] = cand;
        (*count)++;
        return;
    }

    if (candidates[MAX_POCKET_UNBLOCK_CANDIDATES - 1].score >= cand.score) return;
    int idx = MAX_POCKET_UNBLOCK_CANDIDATES - 1;
    while (idx > 0 && candidates[idx - 1].score < cand.score) {
        candidates[idx] = candidates[idx - 1];
        idx--;
    }
    candidates[idx] = cand;
}

// 扫描阶段审计：完整路径中不得出现 first_absorb_idx < last_blast_idx。
// 即第一次箱子被吸收推入目标，不能在最后一次炸墙之前发生。
static bool scan_audit_path_no_absorb_before_blast(
    const SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len
) {
    if (!solver || !path || path_len == 0) return true;
    if (!solver->is_scanning || solver->strict_target_mode) return true;

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
    replay_options.mode = PATH_REPLAY_LEGACY_LENIENT;
    uint16_t first_absorb_idx = 0xFFFF;
    uint16_t last_blast_idx = 0xFFFF;

    for (uint16_t i = 0; i < path_len; i++) {
        PathReplayStepResult step = path_replay_step(solver, &replay_state, path[i], &replay_options);
        if (step.kind == PATH_REPLAY_STEP_ERROR || step.kind == PATH_REPLAY_STEP_STOPPED) return false;
        if (step.kind == PATH_REPLAY_STEP_BLASTED_WALL) last_blast_idx = i;
        if (step.kind == PATH_REPLAY_STEP_PUSHED_BOX && step.box_absorbed && first_absorb_idx == 0xFFFF) {
            first_absorb_idx = i;
        }
    }

    if (first_absorb_idx != 0xFFFF && last_blast_idx != 0xFFFF &&
        first_absorb_idx < last_blast_idx) {
        return false;
    }
    return true;
}

static bool try_player_pocket_unblock(SokobanSolver* solver, int depth, uint32_t current_hash) {
    if (depth != 0 || g_current_pocket_depth != 0) return false;
    if (g_current_pocket_depth >= MAX_POCKET_UNBLOCK_DEPTH) return false;

    static uint16_t old_reach[MAP_ROWS] ALLOC_IN_OCRAM;
    int old_reach_tiles = build_player_reach_mask(&solver->bmap, solver->start_player, old_reach);
    if (old_reach_tiles <= 0 || old_reach_tiles > PLAYER_POCKET_MAX_REACH) return false;

    static bool frontier_box[MAX_BOXES] ALLOC_IN_OCRAM;
    memset(frontier_box, 0, sizeof(frontier_box));
    int frontier_count = 0;
    for (int y = 1; y < MAP_ROWS - 1; y++) {
        uint16_t row = old_reach[y];
        while (row) {
            int x = __builtin_ctz((unsigned int)row);
            row &= (uint16_t)(row - 1);
            for (int d = 0; d < 4; d++) {
                int nx = x + DIRECTIONS[d].dx;
                int ny = y + DIRECTIONS[d].dy;
                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                if ((solver->bmap.boxes[ny] & bit_mask_at(nx)) == 0) continue;
                for (int b = 0; b < solver->num_boxes; b++) {
                    if (solver->boxes[b].pos.x == nx && solver->boxes[b].pos.y == ny) {
                        if ((solver->bmap.targets[ny] & bit_mask_at(nx)) != 0) break;
                        if (!frontier_box[b]) {
                            frontier_box[b] = true;
                            frontier_count++;
                        }
                        break;
                    }
                }
            }
        }
    }
    if (frontier_count == 0) return false;

    PocketUnblockCandidate* candidates = g_pocket_unblock_candidates;
    int num_candidates = 0;
    int old_bombs = 0, old_boxes = 0, old_targets = 0;
    score_player_reach_frontier(&solver->bmap, old_reach, &old_bombs, &old_boxes, &old_targets);

    for (int box_idx = 0; box_idx < solver->num_boxes; box_idx++) {
        if (!frontier_box[box_idx]) continue;
        Position old_box = solver->boxes[box_idx].pos;

        static Position parkings[MAX_POCKET_PARKINGS] ALLOC_IN_OCRAM;
        static int parking_score[MAX_POCKET_PARKINGS] ALLOC_IN_OCRAM;
        int num_parkings = 0;
        for (int y = 1; y < MAP_ROWS - 1; y++) {
            for (int x = 1; x < MAP_COLS - 1; x++) {
                if (!is_pocket_parking_cell(solver, x, y, old_box)) continue;
                int score = (int)manhattan_distance(old_box, (Position){(uint8_t)x, (uint8_t)y});
                if ((old_reach[y] & bit_mask_at(x)) != 0) score += 40;
                if (num_parkings < MAX_POCKET_PARKINGS) {
                    int idx = num_parkings;
                    while (idx > 0 && parking_score[idx - 1] > score) {
                        parkings[idx] = parkings[idx - 1];
                        parking_score[idx] = parking_score[idx - 1];
                        idx--;
                    }
                    parkings[idx] = (Position){(uint8_t)x, (uint8_t)y};
                    parking_score[idx] = score;
                    num_parkings++;
                } else if (parking_score[MAX_POCKET_PARKINGS - 1] > score) {
                    int idx = MAX_POCKET_PARKINGS - 1;
                    while (idx > 0 && parking_score[idx - 1] > score) {
                        parkings[idx] = parkings[idx - 1];
                        parking_score[idx] = parking_score[idx - 1];
                        idx--;
                    }
                    parkings[idx] = (Position){(uint8_t)x, (uint8_t)y};
                    parking_score[idx] = score;
                }
            }
        }

        if (num_parkings == 0) continue;

        static BitboardMap route_map ALLOC_IN_OCRAM;
        route_map = solver->bmap;
        clear_bit(route_map.boxes, old_box.x, old_box.y);
        for (int p = 0; p < num_parkings; p++) {
            Direction* nudge_path = g_solver_scratch.evac.route_path;
            uint16_t path_len = 0;
            int dummy = -1;
            uint16_t backup_max_steps = g_astar_max_steps;
            g_astar_max_steps = 80;
            hash_table_clear();
            bool can_reach = astar_solve_with_mask(
                solver->heap, solver->closed_list, &route_map,
                solver->start_player, old_box, &parkings[p], 1,
                &dummy, MASK_WALL | MASK_BOMB | MASK_BOX,
                nudge_path, &path_len, ASTAR_NO_MACRO_DEPTH, ROUTE_BOX_NORMAL
            );
            g_astar_max_steps = backup_max_steps;
            if (!can_reach || path_len == 0) {
                continue;
            }
            Position new_player = solver->start_player;
            for (int s = 0; s < path_len; s++) {
                new_player.x = (uint8_t)(new_player.x + nudge_path[s].dx);
                new_player.y = (uint8_t)(new_player.y + nudge_path[s].dy);
            }

            static BitboardMap future_map ALLOC_IN_OCRAM;
            future_map = route_map;
            set_box_bit(&future_map, parkings[p].x, parkings[p].y);
            static uint16_t new_reach[MAP_ROWS] ALLOC_IN_OCRAM;
            int new_reach_tiles = build_player_reach_mask(&future_map, new_player, new_reach);
            if (new_reach_tiles <= old_reach_tiles + 3) continue;

            int new_bombs = 0, new_boxes = 0, new_targets = 0;
            score_player_reach_frontier(&future_map, new_reach, &new_bombs, &new_boxes, &new_targets);
            int score = ((new_reach_tiles - old_reach_tiles) * 1000) +
                        ((new_bombs - old_bombs) * 8000) +
                        ((new_targets - old_targets) * 3000) +
                        ((new_boxes - old_boxes) * 1000) -
                        ((int)path_len * 20) - parking_score[p];

            PocketUnblockCandidate cand;
            cand.box_idx = (uint8_t)box_idx;
            cand.target_parking = parkings[p];
            cand.push_len = path_len;
            cand.score = score;
            insert_pocket_candidate(candidates, &num_candidates, cand);
        }
    }

    if (num_candidates == 0) return false;

    bool found = false;
    uint16_t local_best = solver->best_steps;
    uint16_t pool_idx = g_current_pocket_depth;
    int attempts = (num_candidates > 4) ? 4 : num_candidates;

    for (int i = 0; i < attempts; i++) {
        PocketUnblockCandidate* cand = &candidates[i];
        if (local_best != 0xFFFF && cand->push_len >= local_best) continue;

        int box_idx = cand->box_idx;
        Position old_box = solver->boxes[box_idx].pos;
        Position old_player = solver->start_player;
        uint16_t old_steps = solver->best_steps;
        uint16_t old_path_len = solver->best_path_len;
        static BitboardMap old_map ALLOC_IN_OCRAM;
        old_map = solver->bmap;

        static BitboardMap route_map ALLOC_IN_OCRAM;
        route_map = solver->bmap;
        clear_bit(route_map.boxes, old_box.x, old_box.y);
        Direction* nudge_path = g_solver_scratch.evac.apply_path;
        uint16_t actual_push_len = 0;
        int dummy = -1;
        uint16_t backup_max_steps = g_astar_max_steps;
        g_astar_max_steps = 80;
        hash_table_clear();
        bool can_reach = astar_solve_with_mask(
            solver->heap, solver->closed_list, &route_map,
            old_player, old_box, &cand->target_parking, 1,
            &dummy, MASK_WALL | MASK_BOMB | MASK_BOX,
            nudge_path, &actual_push_len, ASTAR_NO_MACRO_DEPTH, ROUTE_BOX_NORMAL
        );
        g_astar_max_steps = backup_max_steps;
        if (!can_reach) {
            continue;
        }
        Position new_player = old_player;
        for (int s = 0; s < actual_push_len; s++) {
            new_player.x = (uint8_t)(new_player.x + nudge_path[s].dx);
            new_player.y = (uint8_t)(new_player.y + nudge_path[s].dy);
        }

        uint32_t next_hash = hash_move_player(current_hash, old_player, new_player);
        next_hash = hash_move_box(next_hash, old_box, cand->target_parking, false);

        clear_bit(solver->bmap.boxes, old_box.x, old_box.y);
        set_box_bit(&solver->bmap, cand->target_parking.x, cand->target_parking.y);
        solver->boxes[box_idx].pos = cand->target_parking;
        solver->start_player = new_player;
        if (local_best != 0xFFFF) solver->best_steps = (uint16_t)(local_best - actual_push_len);
        Direction* stable_prefix = g_pocket_path_pool[pool_idx];
        memcpy(stable_prefix, nudge_path, actual_push_len * sizeof(Direction));
        /* 应用路径缓冲只临时使用；递归前缀已复制到稳定路径池。 */

        g_current_pocket_depth++;
        if (sokoban_solve_internal(solver, depth, next_hash)) {
            uint16_t sub_len = solver->best_path_len;
            uint32_t total32 = (uint32_t)actual_push_len + sub_len;
            if (total32 < local_best && total32 < MAX_PATH_LENGTH) {
                uint16_t total = (uint16_t)total32;
                local_best = total;
                memcpy(&stable_prefix[actual_push_len], solver->best_path, sub_len * sizeof(Direction));
                found = true;
            }
        }
        g_current_pocket_depth--;

        solver->boxes[box_idx].pos = old_box;
        solver->start_player = old_player;
        solver->best_steps = old_steps;
        solver->best_path_len = old_path_len;
        solver->bmap = old_map;

        if (found) break;
    }

    if (found) {
        if (!scan_audit_path_no_absorb_before_blast(solver, g_pocket_path_pool[pool_idx], local_best)) {
            return false;
        }
        solver->best_steps = local_best;
        solver->best_path_len = local_best;
        memcpy(solver->best_path, g_pocket_path_pool[pool_idx], local_best * sizeof(Direction));
        return true;
    }
    return false;
}
typedef struct {
    bool is_bomb;
    uint8_t entity_idx;
    Position old_pos;
    Position target_parking;
    uint16_t push_len;
    int score;
} EvacMoveCandidate;

static EvacMoveCandidate g_evac_candidates[MAX_EVAC_CANDIDATES] ALLOC_IN_OCRAM;
typedef char SuperEvacCandidates_must_fit_shared_pool[(MAX_SUPER_EVAC_CANDIDATES <= MAX_EVAC_CANDIDATES) ? 1 : -1];

typedef enum {
    EVAC_KIND_SMART = 1,
    EVAC_KIND_SUPER = 2
} EvacKind;

typedef struct {
    EvacKind kind;
    int depth;
    int pool_depth;
    Direction* path_pool_row;
    uint16_t max_path_len;
} EvacOutput;

typedef bool (*EvacParkingFn)(const SokobanSolver* solver, int x, int y);

typedef struct {
    EvacKind kind;
    bool allow_bombs;
    bool use_bomb_route_base_map;
    uint8_t max_candidates;
    uint8_t max_parkings;
    uint8_t attempt_limit;
    uint16_t astar_step_limit;
    AStarRouteType route_type;
    EvacParkingFn is_parking;
} EvacConfig;

static EvacOutput evac_output_make(EvacKind kind, int depth, int pool_depth,
                                   Direction* path_pool_row, uint16_t max_path_len) {
    EvacOutput out;
    out.kind = kind;
    out.depth = depth;
    out.pool_depth = pool_depth;
    out.path_pool_row = path_pool_row;
    out.max_path_len = max_path_len;
    return out;
}

static bool evac_store_output_path(const EvacOutput* out,
                                   const Direction* nudge_path,
                                   uint16_t nudge_len,
                                   const Direction* solve_path,
                                   uint16_t solve_len) {
    if (!out || !out->path_pool_row || !nudge_path || !solve_path) return false;
    uint32_t total = (uint32_t)nudge_len + (uint32_t)solve_len;
    if (total >= out->max_path_len) return false;
    if (nudge_len > 0 && out->path_pool_row != nudge_path) {
        memcpy(out->path_pool_row, nudge_path, nudge_len * sizeof(Direction));
    }
    if (solve_len > 0) memcpy(&out->path_pool_row[nudge_len], solve_path, solve_len * sizeof(Direction));
    return true;
}

static void sort_evac_candidates(EvacMoveCandidate* candidates, int count) {
    if (!candidates || count <= 1) return;
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (candidates[i].score < candidates[j].score) {
                EvacMoveCandidate temp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = temp;
            }
        }
    }
}

static bool evac_try_route(SokobanSolver* solver,
                             const BitboardMap* route_map,
                             Position player,
                             Position entity_pos,
                             const Position* target,
                             uint16_t step_limit,
                             AStarRouteType route_type,
                             Direction* out_path,
                             uint16_t* out_len) {
    if (!solver || !route_map || !target || !out_path || !out_len) return false;
    int dummy = -1;
    *out_len = 0;
    uint16_t backup_max_steps = g_astar_max_steps;
    Position route_target = *target;
    g_astar_max_steps = step_limit;
    hash_table_clear();
    bool ok = astar_solve_with_mask(
        solver->heap, solver->closed_list, route_map,
        player, entity_pos, &route_target, 1,
        &dummy, MASK_WALL | MASK_BOMB | MASK_BOX,
        out_path, out_len, ASTAR_NO_MACRO_DEPTH, route_type
    );
    g_astar_max_steps = backup_max_steps;
    return ok;
}

static Position evac_path_end(Position start, const Direction* path, uint16_t path_len) {
    Position pos = start;
    for (uint16_t s = 0; s < path_len; s++) {
        pos.x = (uint8_t)(pos.x + path[s].dx);
        pos.y = (uint8_t)(pos.y + path[s].dy);
    }
    return pos;
}

static inline bool is_smart_evac_parking(const SokobanSolver* solver, int x, int y) {
    return solver && is_open_area(&solver->bmap, x, y);
}

static inline bool is_super_parking(const SokobanSolver* solver, int x, int y) {
    const BitboardMap* bmap = &solver->bmap;
    if (x <= 0 || x >= MAP_COLS - 1 || y <= 0 || y >= MAP_ROWS - 1) return false;
    if ((bmap->walls[y] & (1 << x)) != 0) return false;
    if ((bmap->targets[y] & (1 << x)) != 0) return false;
    if (x == 1 && !g_edge_target_L) return false;
    if (x == MAP_COLS - 2 && !g_edge_target_R) return false;
    if (y == 1 && !g_edge_target_U) return false;
    if (y == MAP_ROWS - 2 && !g_edge_target_D) return false;
    if (solver->num_bombs == 0 && ((bmap->deadlocks[y] & (1 << x)) != 0)) return false;
    return true;
}

static void evac_prepare_route_map(const SokobanSolver* solver,
                                   const EvacConfig* cfg,
                                   Position entity_pos,
                                   int entity_idx,
                                   bool is_bomb,
                                   BitboardMap* route_map) {
    if (cfg->use_bomb_route_base_map) build_bomb_route_base_map(solver, route_map);
    else *route_map = solver->bmap;
    if (is_bomb) clear_bit(route_map->bombs, entity_pos.x, entity_pos.y);
    else clear_bit(route_map->boxes, entity_pos.x, entity_pos.y);
    prepare_route_map_targets(solver, route_map, entity_idx, is_bomb, true);
}

static int smart_evac_reachable_score(const SokobanSolver* solver,
                                      const BitboardMap* route_map,
                                      Position target,
                                      const Direction* nudge_path,
                                      uint16_t path_len) {
    Position new_player = evac_path_end(solver->start_player, nudge_path, path_len);
    static BitboardMap future_map ALLOC_IN_OCRAM;
    future_map = *route_map;
    set_box_bit(&future_map, target.x, target.y);

    static uint16_t reach_visited[MAP_ROWS] ALLOC_IN_OCRAM;
    memset(reach_visited, 0, sizeof(reach_visited));
    Position* rq = g_solver_scratch.evac.reach_queue;
    int r_head = 0;
    int r_tail = 0;
    rq[r_tail++] = new_player;
    reach_visited[new_player.y] |= (1 << new_player.x);

    int reachable_tiles = 0;
    int reachable_bombs = 0;
    int reachable_targets = 0;
    int reachable_boxes = 0;

    while (r_head < r_tail) {
        Position curr = rq[r_head++];
        reachable_tiles++;
        if (get_bit(future_map.targets, curr.x, curr.y)) reachable_targets++;

        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
            uint16_t bit = bit_mask_at(nx);
            if ((reach_visited[ny] & bit) != 0) continue;
            reach_visited[ny] |= bit;
            if (get_bit(future_map.walls, nx, ny)) {
                continue;
            } else if (get_bit(future_map.bombs, nx, ny)) {
                reachable_bombs++;
            } else if (get_bit(future_map.boxes, nx, ny)) {
                reachable_boxes++;
            } else if (r_tail < MAP_ROWS * MAP_COLS) {
                rq[r_tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
            }
        }
    }

    return (reachable_bombs * 10000) +
           (reachable_targets * 5000) +
           (reachable_boxes * 1000) +
           (reachable_tiles * 10) -
           path_len;
}

static bool try_evacuation_generic(SokobanSolver* solver,
                                   int depth,
                                   uint32_t current_hash,
                                   const EvacConfig* cfg,
                                   const EvacOutput* output) {
    if (!solver || !cfg || !output || depth >= MAX_BOMBS) return false;

    EvacMoveCandidate* candidates = g_evac_candidates;
    int num_candidates = 0;
    int total_entities = solver->num_boxes + (cfg->allow_bombs ? solver->num_bombs : 0);

    for (int e = 0; e < total_entities; e++) {
        bool is_bomb = cfg->allow_bombs && e >= solver->num_boxes;
        int idx = is_bomb ? (e - solver->num_boxes) : e;
        Position old_pos = is_bomb ? solver->bombs[idx].pos : solver->boxes[idx].pos;
        if (!is_bomb && (solver->bmap.targets[old_pos.y] & (1 << old_pos.x)) != 0) continue;

        g_bfs_run_epoch++;
        if (g_bfs_run_epoch == 0) {
            memset(g_bfs_visited, 0, sizeof(g_bfs_visited));
            g_bfs_run_epoch = 1;
        }

        Position* q = g_bfs_queue;
        int head = 0;
        int tail = 0;
        q[tail++] = old_pos;
        g_bfs_visited[old_pos.y][old_pos.x] = g_bfs_run_epoch;

        Position* target_parkings = g_solver_scratch.evac.parkings;
        int num_parkings = 0;
        while (head < tail) {
            Position curr = q[head++];
            if (!pos_equal(curr, old_pos) && cfg->is_parking(solver, curr.x, curr.y)) {
                if (((solver->bmap.boxes[curr.y] | solver->bmap.bombs[curr.y]) & (1 << curr.x)) == 0) {
                    target_parkings[num_parkings++] = curr;
                    if (num_parkings >= cfg->max_parkings) break;
                }
            }

            for (int d = 0; d < 4; d++) {
                int nx = curr.x + DIRECTIONS[d].dx;
                int ny = curr.y + DIRECTIONS[d].dy;
                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                if (g_bfs_visited[ny][nx] == g_bfs_run_epoch) continue;
                if ((solver->bmap.walls[ny] & (1 << nx)) != 0) continue;
                g_bfs_visited[ny][nx] = g_bfs_run_epoch;
                if (tail < MAP_ROWS * MAP_COLS) q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
            }
        }
        if (num_parkings == 0) {
            continue;
        }

        static BitboardMap temp_map ALLOC_IN_OCRAM;
        evac_prepare_route_map(solver, cfg, old_pos, idx, is_bomb, &temp_map);
        for (int t = 0; t < num_parkings; t++) {
            if (num_candidates >= cfg->max_candidates) break;
            Position target = target_parkings[t];
            Direction* nudge_path = g_solver_scratch.evac.route_path;
            uint16_t path_len = 0;
            bool can_reach = evac_try_route(solver, &temp_map, solver->start_player, old_pos,
                                              &target, cfg->astar_step_limit, cfg->route_type,
                                              nudge_path, &path_len);
            if (!can_reach) continue;

            EvacMoveCandidate* cand = &candidates[num_candidates++];
            cand->is_bomb = is_bomb;
            cand->entity_idx = (uint8_t)idx;
            cand->old_pos = old_pos;
            cand->target_parking = target;
            cand->push_len = path_len;
            cand->score = (cfg->kind == EVAC_KIND_SMART)
                ? smart_evac_reachable_score(solver, &temp_map, target, nudge_path, path_len)
                : (1000 - path_len);
        }
    }

    
    if (num_candidates == 0) return false;
    sort_evac_candidates(candidates, num_candidates);

    int attempt_limit = (cfg->attempt_limit == 0 || cfg->attempt_limit > num_candidates)
        ? num_candidates
        : cfg->attempt_limit;
    bool found = false;
    uint16_t local_best = 0xFFFF;

    for (int i = 0; i < attempt_limit; i++) {
        EvacMoveCandidate* cand = &candidates[i];
        if (local_best != 0xFFFF && cand->push_len >= local_best) continue;
        

        int idx = cand->entity_idx;
        bool is_bomb = cand->is_bomb;
        Position old_pos = is_bomb ? solver->bombs[idx].pos : solver->boxes[idx].pos;
        Position old_player = solver->start_player;
        uint16_t old_steps = solver->best_steps;
        uint16_t old_path_len = solver->best_path_len;
        BitboardMap old_map = solver->bmap;

        static BitboardMap route_map ALLOC_IN_OCRAM;
        evac_prepare_route_map(solver, cfg, old_pos, idx, is_bomb, &route_map);
        Direction* nudge_path = g_solver_scratch.evac.apply_path;
        uint16_t actual_push_len = 0;
        bool can_reach = evac_try_route(solver, &route_map, old_player, old_pos,
                                          &cand->target_parking, cfg->astar_step_limit,
                                          cfg->route_type, nudge_path, &actual_push_len);
        if (!can_reach) {
            continue;
        }

        Position new_player = evac_path_end(old_player, nudge_path, actual_push_len);
        uint32_t next_hash = hash_move_player(current_hash, old_player, new_player);
        if (is_bomb) {
            next_hash ^= ZOBRIST_BOMB[Z_IDX(old_pos.x, old_pos.y)];
            next_hash ^= ZOBRIST_BOMB[Z_IDX(cand->target_parking.x, cand->target_parking.y)];
            clear_bit(solver->bmap.bombs, old_pos.x, old_pos.y);
            set_bit(solver->bmap.bombs, cand->target_parking.x, cand->target_parking.y);
            solver->bombs[idx].pos = cand->target_parking;
        } else {
            bool box_absorbed = solver_is_target_cell(solver, cand->target_parking) &&
                                solver_should_absorb_box(solver, idx, cand->target_parking);
            next_hash = hash_move_box(next_hash, old_pos, cand->target_parking, box_absorbed);
            clear_bit(solver->bmap.boxes, old_pos.x, old_pos.y);
            set_box_bit(&solver->bmap, cand->target_parking.x, cand->target_parking.y);
            solver->boxes[idx].pos = cand->target_parking;
        }
        solver->start_player = new_player;
        if (local_best != 0xFFFF) solver->best_steps = (uint16_t)(local_best - actual_push_len);
        if (!evac_store_output_path(output, nudge_path, actual_push_len, solver->best_path, 0)) {
            if (is_bomb) solver->bombs[idx].pos = old_pos;
            else solver->boxes[idx].pos = old_pos;
            solver->start_player = old_player;
            solver->best_steps = old_steps;
            solver->best_path_len = old_path_len;
            solver->bmap = old_map;
            continue;
        }

        /* 应用路径缓冲只临时使用；前缀已写入输出路径池行。 */

        if (cfg->kind == EVAC_KIND_SMART) g_current_macro_depth++;
        else g_current_super_depth++;
        bool solved = sokoban_solve_internal(solver, depth, next_hash);
        if (cfg->kind == EVAC_KIND_SMART) g_current_macro_depth--;
        else g_current_super_depth--;

        if (solved) {
            uint16_t sub_len = solver->best_path_len;
            uint16_t total = (uint16_t)(actual_push_len + sub_len);
            if (total < local_best && total < MAX_PATH_LENGTH &&
                evac_store_output_path(output, output->path_pool_row, actual_push_len, solver->best_path, sub_len)) {
                local_best = total;
                found = true;
                
                
            }
        }

        if (is_bomb) solver->bombs[idx].pos = old_pos;
        else solver->boxes[idx].pos = old_pos;
        solver->start_player = old_player;
        solver->best_steps = old_steps;
        solver->best_path_len = old_path_len;
        solver->bmap = old_map;
        if (found) break;
    }

    if (!found) return false;
    if (!scan_audit_path_no_absorb_before_blast(solver, output->path_pool_row, local_best)) {
        return false;
    }
    solver->best_steps = local_best;
    solver->best_path_len = local_best;
    memcpy(solver->best_path, output->path_pool_row, local_best * sizeof(Direction));
    return true;
}
static bool try_smart_evacuation(SokobanSolver* solver, int depth, int macro_depth, uint32_t current_hash) {
    if (depth >= MAX_BOMBS || macro_depth >= MAX_MACRO) return false;
    const EvacConfig cfg = {
        EVAC_KIND_SMART, false, true, MAX_EVAC_CANDIDATES, MAX_PARKINGS,
        0, 150, ROUTE_BOX_NORMAL, is_smart_evac_parking
    };
    EvacOutput out = evac_output_make(EVAC_KIND_SMART, depth, macro_depth,
                                      g_evac_path_pool[depth][macro_depth], MAX_PATH_LENGTH);
    bool ok = try_evacuation_generic(solver, depth, current_hash, &cfg, &out);
    return ok;
}

static bool try_super_evacuation(SokobanSolver* solver, int depth, int super_depth, uint32_t current_hash) {
    if (depth >= MAX_BOMBS || super_depth >= MAX_SUPER_MACRO) return false;
    const EvacConfig cfg = {
        EVAC_KIND_SUPER, true, false, MAX_SUPER_EVAC_CANDIDATES, MAX_SUPER_PARKINGS,
        6, 80, ROUTE_SUPER_EVAC, is_super_parking
    };
    EvacOutput out = evac_output_make(EVAC_KIND_SUPER, depth, super_depth,
                                      g_evac_path_pool[depth][super_depth], MAX_PATH_LENGTH);
    bool ok = try_evacuation_generic(solver, depth, current_hash, &cfg, &out);
    return ok;
}
static uint8_t next_bomb_phase_epoch(int depth) {
    if (depth < 0 || depth >= MAX_BOMBS) return 1u;
    uint8_t epoch = (uint8_t)(g_bomb_phase_epoch[depth] + 1u);
    if (epoch == 0u) {
        memset(g_bomb_phase_used[depth], 0, sizeof(g_bomb_phase_used[depth]));
        epoch = 1u;
    }
    g_bomb_phase_epoch[depth] = epoch;
    return epoch;
}

typedef bool (*BombCandidateLessFn)(const BombWallCandidate* lhs, const BombWallCandidate* rhs);

static bool bomb_candidate_rank_less(const BombWallCandidate* lhs, const BombWallCandidate* rhs) {
    return lhs->score < rhs->score ||
           (lhs->score == rhs->score && lhs->path_lower_bound > rhs->path_lower_bound);
}

static bool sorted_insert_bomb_candidate(BombWallCandidate* items, int* count, int max_count, BombWallCandidate item, BombCandidateLessFn less) {
    if (!items || !count || max_count <= 0 || !less) return false;
    int idx = *count;
    if (idx < max_count) {
        (*count)++;
    } else {
        if (items[max_count - 1].score >= item.score) return false;
        idx = max_count - 1;
    }

    while (idx > 0 && less(&items[idx - 1], &item)) {
        items[idx] = items[idx - 1];
        idx--;
    }
    items[idx] = item;
    return true;
}

static void insert_bomb_candidate_sorted(BombWallCandidate* candidates, int* count, int max_count, BombWallCandidate cand) {
    (void)sorted_insert_bomb_candidate(
        candidates, count, max_count, cand, bomb_candidate_rank_less
    );
}

static void sort_bomb_candidates_desc(BombWallCandidate* candidates, int count) {
    for (int i = 1; i < count; i++) {
        BombWallCandidate item = candidates[i];
        int j = i;
        while (j > 0 && bomb_candidate_rank_less(&candidates[j - 1], &item)) {
            candidates[j] = candidates[j - 1];
            j--;
        }
        candidates[j] = item;
    }
}
typedef struct {
    int candidate_count;
    int before_dedupe_count;
    int top_count;
    int shortcut_count;
    int maneuver_count;
    int reachable_count;
    int routed_count;
    int lb_min;
    int lb_max;
    int lb_span;
    int route_min;
    int route_max;
    int route_span;
    int dedupe_retention_pct;
} BombOrderContext;

typedef enum {
    BOMB_ORDER_SMALL,
    BOMB_ORDER_BASE,
    BOMB_ORDER_TOPOLOGY_WIDE,
    BOMB_ORDER_MANEUVER_WIDE
} BombOrderMode;

typedef struct {
    BombOrderMode order_mode;
    int confidence;
    int base_attempt_limit;
    int rescue_attempt_limit;
    int proximity_attempt_limit;
    int struct_attempt_limit;
    int root_low_lb_append;
    bool use_maneuver_wide_order;
    bool use_topology_wide_fast_limit;
    bool use_scan_low_lb_append;
    bool use_root_low_lb_budget;
    bool reserve_root_low_lb_append;
    bool allow_root_low_lb_append;
    bool enable_root_push_reach_filter;
    int low_lb_hidden_count;
    int low_lb_in_base_window;
    int low_lb_best_route_gap;
    uint16_t low_lb_max_path_lower;
} BombStrategyPolicy;

static BombOrderContext g_bomb_order_context_scratch[MAX_BOMBS] ALLOC_IN_DTCM;

typedef struct {
    BombWallCandidate* top_candidates;
    BombWallCandidate* light_evac_candidates;
    BitboardMap* route_base_map;
    BombOrderContext* order_ctx;
    BombStrategyPolicy policy;
    int num_candidates;
    int num_candidates_before_dedupe;
    int num_light_evac_candidates;
    uint16_t min_candidate_path_lower;
} BombCandidatePlan;

typedef struct {
    Direction* bomb_path;
    uint16_t* branch_attempt_keys;
    int branch_attempt_key_count;
    uint16_t assignment_lower;
    uint16_t local_best;
    int valid_attempts;
    bool found;
} BombAttemptSchedule;

static BombCandidatePlan g_bomb_candidate_plan_scratch[MAX_BOMBS] ALLOC_IN_DTCM;
static inline bool solver_should_expand_root_low_lb_budget(
    const SokobanSolver* solver,
    const BombOrderContext* ctx,
    const BombWallCandidate* candidates,
    int num_candidates,
    uint16_t min_candidate_path_lower,
    int depth,
    int base_attempt_limit
) {
    if (!solver || !ctx || !candidates) return false;
    if (!solver_is_root_append_entry(solver, depth)) return false;
    if (num_candidates < 10 || base_attempt_limit <= 0) return false;
    if (min_candidate_path_lower == 0xFFFF) return false;
    if (ctx->reachable_count < 6) return false;
    if (ctx->lb_span < 6) return false;
    if (candidates[0].path_lower_bound == 0xFFFF) return false;

    bool topo_dominant = (ctx->top_count * 2) >= ctx->candidate_count;
    if (topo_dominant) {
        bool topo_maneuver_bridge =
            ctx->shortcut_count == 0 &&
            ctx->maneuver_count >= 6 &&
            ctx->lb_span >= 30 &&
            ctx->route_span >= 40 &&
            ctx->candidate_count > 0 &&
            (ctx->candidate_count - ctx->top_count) <= 4;
        bool topo_dense_with_span =
            ctx->shortcut_count == 0 &&
            ctx->top_count >= 20 &&
            ctx->candidate_count > 0 &&
            (ctx->top_count * 4) >= (ctx->candidate_count * 3) &&
            ctx->maneuver_count >= 7 &&
            ctx->lb_span >= 30 &&
            ctx->route_span >= 40;
        bool topo_has_support =
            (ctx->shortcut_count > 0) ||
            ((ctx->maneuver_count * 2 + 1) >= ctx->candidate_count) ||
            topo_maneuver_bridge ||
            topo_dense_with_span;
        if (!topo_has_support) return false;
        if (ctx->route_span < 30) return false;
    } else {
        if (num_candidates <= base_attempt_limit) return false;
        if (ctx->route_span < 28) return false;
        if (candidates[0].path_lower_bound <= (uint16_t)(min_candidate_path_lower + 6)) return false;
    }

    int nearby_total = 0;
    uint16_t best_near_route = 0xFFFF;
    for (int i = 0; i < num_candidates; i++) {
        const BombWallCandidate* cand = &candidates[i];
        if (cand->route_len == 0xFFFF || cand->path_lower_bound == 0xFFFF) continue;
        if (cand->path_lower_bound > (uint16_t)(min_candidate_path_lower + 3)) continue;
        nearby_total++;
        if (cand->route_len < best_near_route) best_near_route = cand->route_len;
    }
    if (nearby_total == 0 || best_near_route == 0xFFFF) return false;
    if (!topo_dominant && ctx->route_min != 0xFFFF && best_near_route > (uint16_t)(ctx->route_min + 12)) return false;

    return true;
}

typedef struct {
    int hidden_count;
    int in_base_window;
    int best_route_gap;
} BombRootLowLbStats;

static inline bool solver_collect_hidden_root_low_lb_candidate_stats(
    const BombWallCandidate* candidates,
    int num_candidates,
    uint16_t min_candidate_path_lower,
    uint16_t route_min,
    int base_attempt_limit,
    int root_low_lb_append,
    BombRootLowLbStats* out_stats
) {
    BombRootLowLbStats stats;
    memset(&stats, 0, sizeof(stats));
    stats.best_route_gap = 0xFFFF;
    if (out_stats) *out_stats = stats;

    if (!candidates || num_candidates <= 0 || base_attempt_limit <= 0) return false;
    if (min_candidate_path_lower == 0xFFFF) return false;

    int base_window = base_attempt_limit - root_low_lb_append;
    if (base_window < 0) base_window = 0;
    if (base_window > num_candidates) base_window = num_candidates;

    int nearby_total = 0;
    int nearby_in_base_window = 0;
    int first_hidden_nearby = -1;
    uint16_t best_near_route = 0xFFFF;
    uint16_t max_near_lower = (uint16_t)(min_candidate_path_lower + 3);
    for (int i = 0; i < num_candidates; i++) {
        const BombWallCandidate* cand = &candidates[i];
        if (cand->route_len == 0xFFFF || cand->path_lower_bound == 0xFFFF) continue;
        if (cand->path_lower_bound > max_near_lower) continue;
        nearby_total++;
        if (cand->route_len < best_near_route) best_near_route = cand->route_len;
        if (i < base_window) {
            nearby_in_base_window++;
        } else if (first_hidden_nearby < 0) {
            first_hidden_nearby = i;
        }
    }

    stats.in_base_window = nearby_in_base_window;
    stats.hidden_count = nearby_total - nearby_in_base_window;
    if (route_min != 0xFFFF && best_near_route != 0xFFFF) {
        stats.best_route_gap = (best_near_route > route_min) ? (int)(best_near_route - route_min) : 0;
    }
    if (out_stats) *out_stats = stats;

    if (nearby_total <= nearby_in_base_window || first_hidden_nearby < 0) return false;
    if (num_candidates <= (base_window + root_low_lb_append + 4)) return false;
    if (first_hidden_nearby <= (base_window + 1)) return false;
    return true;
}


static BombOrderContext build_bomb_order_context(int num_candidates, int before_dedupe_count,
                                                 const BombWallCandidate* candidates) {
    BombOrderContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.candidate_count = num_candidates;
    ctx.before_dedupe_count = before_dedupe_count;
    ctx.lb_min = 0xFFFF;
    ctx.route_min = 0xFFFF;

    for (int i = 0; i < num_candidates; i++) {
        const BombWallCandidate* cand = &candidates[i];
        if (cand->applied_top_bonus > 0) ctx.top_count++;
        if (cand->applied_short_bonus > 0) ctx.shortcut_count++;
        if (cand->applied_maneuver_bonus > 0) ctx.maneuver_count++;

        if (cand->path_lower_bound != 0xFFFF) {
            ctx.reachable_count++;
            if (cand->path_lower_bound < ctx.lb_min) ctx.lb_min = cand->path_lower_bound;
            if (cand->path_lower_bound > ctx.lb_max) ctx.lb_max = cand->path_lower_bound;
        }
        if (cand->route_len != 0xFFFF) {
            ctx.routed_count++;
            if (cand->route_len < ctx.route_min) ctx.route_min = cand->route_len;
            if (cand->route_len > ctx.route_max) ctx.route_max = cand->route_len;
        }
    }

    if (ctx.lb_min != 0xFFFF) ctx.lb_span = ctx.lb_max - ctx.lb_min;
    if (ctx.route_min != 0xFFFF) ctx.route_span = ctx.route_max - ctx.route_min;
    ctx.dedupe_retention_pct = (before_dedupe_count > 0) ?
        (num_candidates * 100) / before_dedupe_count : 100;
    
    
    
    return ctx;
}

static int bomb_order_mode_signal_score(const SokobanSolver* solver, int depth,
                                        const BombOrderContext* ctx, BombOrderMode mode);

static BombOrderMode select_bomb_order_mode(const SokobanSolver* solver, int depth, const BombOrderContext* ctx) {
    if (!solver || !ctx || ctx->candidate_count <= 0) return BOMB_ORDER_BASE;
    if (solver->num_bombs < 2) return BOMB_ORDER_BASE;

    BombOrderMode best_mode = BOMB_ORDER_BASE;
    int best_score = bomb_order_mode_signal_score(solver, depth, ctx, BOMB_ORDER_BASE);

    int small_score = bomb_order_mode_signal_score(solver, depth, ctx, BOMB_ORDER_SMALL);
    if (small_score > best_score) {
        best_score = small_score;
        best_mode = BOMB_ORDER_SMALL;
    }

    int topology_score = bomb_order_mode_signal_score(solver, depth, ctx, BOMB_ORDER_TOPOLOGY_WIDE);
    if (topology_score > best_score) {
        best_score = topology_score;
        best_mode = BOMB_ORDER_TOPOLOGY_WIDE;
    }

    int maneuver_score = bomb_order_mode_signal_score(solver, depth, ctx, BOMB_ORDER_MANEUVER_WIDE);
    if (maneuver_score > best_score) {
        best_score = maneuver_score;
        best_mode = BOMB_ORDER_MANEUVER_WIDE;
    }

    return best_mode;
}

static int bomb_policy_clamp_percent(int value) {
    if (value < 0) return 0;
    if (value > 100) return 100;
    return value;
}

static int bomb_policy_ratio_to_threshold(int numerator, int denominator, int threshold_pct) {
    if (denominator <= 0 || threshold_pct <= 0) return 0;
    int pct = (numerator * 100) / denominator;
    return bomb_policy_clamp_percent((pct * 100) / threshold_pct);
}

static int bomb_policy_span_to_threshold(int value, int threshold) {
    if (threshold <= 0) return 0;
    return bomb_policy_clamp_percent((value * 100) / threshold);
}

static int bomb_policy_confidence(int best_score, int second_best_score) {
    /* confidence = (best - second) * 100 / max(best, 1). */
    if (best_score <= 0) return 0;
    if (second_best_score < 0) second_best_score = 0;
    if (second_best_score > best_score) second_best_score = best_score;
    return ((best_score - second_best_score) * 100) / best_score;
}

static int bomb_order_mode_signal_score(const SokobanSolver* solver, int depth,
                                        const BombOrderContext* ctx, BombOrderMode mode) {
    if (!solver || !ctx) return 0;
    int candidate_count = ctx->candidate_count;

    switch (mode) {
    case BOMB_ORDER_SMALL:
        return (candidate_count <= 4) ? 100 : 0;

    case BOMB_ORDER_TOPOLOGY_WIDE: {
        if (candidate_count <= 0) return 0;
        int low_maneuver_score = (ctx->maneuver_count <= 1)
            ? 100
            : bomb_policy_clamp_percent(100 - ((ctx->maneuver_count - 1) * 25));
        int top_score = bomb_policy_ratio_to_threshold(ctx->top_count, candidate_count, 67);
        int shortcut_score = bomb_policy_ratio_to_threshold(ctx->shortcut_count, candidate_count, 50);
        int score = low_maneuver_score;
        if (top_score < score) score = top_score;
        if (shortcut_score < score) score = shortcut_score;
        return score;
    }

    case BOMB_ORDER_MANEUVER_WIDE: {
        if (depth != 0 || candidate_count <= 0) return 0;
        int maneuver_score = bomb_policy_ratio_to_threshold(ctx->maneuver_count, candidate_count, 50);
        int count_score = bomb_policy_span_to_threshold(ctx->maneuver_count, 6);
        int lb_score = bomb_policy_span_to_threshold(ctx->lb_span, 20);
        int route_score = bomb_policy_span_to_threshold(ctx->route_span, 30);
        int clean_score = (ctx->top_count == 0 && ctx->shortcut_count == 0) ? 100 : 0;
        int score = maneuver_score;
        if (count_score < score) score = count_score;
        if (lb_score < score) score = lb_score;
        if (route_score < score) score = route_score;
        if (clean_score < score) score = clean_score;
        return score;
    }

    case BOMB_ORDER_BASE:
    default: {
        int score = 50;
        if (solver->num_bombs < 2) score += 50;
        if (candidate_count < 10) score += (10 - candidate_count) * 4;
        if (depth != 0) score += 20;
        if (ctx->top_count == 0 && ctx->shortcut_count == 0 && ctx->maneuver_count == 0) score += 15;
        return bomb_policy_clamp_percent(score);
    }
    }
}

static int bomb_strategy_policy_confidence(const SokobanSolver* solver, int depth,
                                           const BombOrderContext* ctx, BombOrderMode selected_mode) {
    int selected_score = bomb_order_mode_signal_score(solver, depth, ctx, selected_mode);
    int second_score = 0;
    const BombOrderMode modes[] = {
        BOMB_ORDER_SMALL,
        BOMB_ORDER_BASE,
        BOMB_ORDER_TOPOLOGY_WIDE,
        BOMB_ORDER_MANEUVER_WIDE
    };

    for (int i = 0; i < (int)(sizeof(modes) / sizeof(modes[0])); i++) {
        if (modes[i] == selected_mode) continue;
        int score = bomb_order_mode_signal_score(solver, depth, ctx, modes[i]);
        if (score > second_score) second_score = score;
    }

    return bomb_policy_confidence(selected_score, second_score);
}

static BombStrategyPolicy build_bomb_strategy_policy(
    const SokobanSolver* solver,
    int depth,
    const BombOrderContext* ctx,
    const BombWallCandidate* candidates,
    int num_candidates,
    uint16_t min_candidate_path_lower
) {
    BombStrategyPolicy policy;
    memset(&policy, 0, sizeof(policy));

    policy.order_mode = select_bomb_order_mode(solver, depth, ctx);
    policy.confidence = bomb_strategy_policy_confidence(solver, depth, ctx, policy.order_mode);
    bool high_confidence = policy.confidence >= BOMB_POLICY_LOW_CONFIDENCE;
    policy.use_maneuver_wide_order =
        high_confidence && policy.order_mode == BOMB_ORDER_MANEUVER_WIDE;
    policy.use_topology_wide_fast_limit =
        high_confidence && policy.order_mode == BOMB_ORDER_TOPOLOGY_WIDE;
    bool wide_order = policy.use_maneuver_wide_order || policy.use_topology_wide_fast_limit;
    bool wide_low_confidence =
        wide_order && policy.confidence <= (BOMB_POLICY_LOW_CONFIDENCE + 5);
    policy.use_scan_low_lb_append =
        solver->is_scanning && !solver->strict_target_mode &&
        solver->scan_waypoint_count == 0 && solver->scan_current_index == 0;
    policy.root_low_lb_append = policy.use_scan_low_lb_append ? 2 :
        (wide_low_confidence ? MAX_WIDE_ROOT_LOW_LB_APPEND : MAX_ROOT_LOW_LB_APPEND);
    policy.base_attempt_limit = (depth == 0 && policy.use_maneuver_wide_order) ? BOMB_MANEUVER_ROOT_ATTEMPT_LIMIT :
        (policy.use_topology_wide_fast_limit ? BOMB_TOPOLOGY_WIDE_ATTEMPT_LIMIT : MAX_BASE_BOMB_ATTEMPTS);

    bool feature_root_low_lb_budget = solver_should_expand_root_low_lb_budget(
        solver, ctx, candidates, num_candidates, min_candidate_path_lower,
        depth, policy.base_attempt_limit);
    bool root_low_lb_basic_append =
        num_candidates >= 10 && min_candidate_path_lower != 0xFFFF &&
        candidates[0].path_lower_bound != 0xFFFF &&
        candidates[0].path_lower_bound > (uint16_t)(min_candidate_path_lower + 6);
    policy.low_lb_max_path_lower = (min_candidate_path_lower == 0xFFFF)
        ? 0xFFFF
        : (uint16_t)(min_candidate_path_lower + 3);

    BombRootLowLbStats low_lb_stats;
    bool has_hidden_low_lb = false;
    if (root_low_lb_basic_append) {
        has_hidden_low_lb = solver_collect_hidden_root_low_lb_candidate_stats(
            candidates, num_candidates, min_candidate_path_lower, (uint16_t)ctx->route_min,
            policy.base_attempt_limit, policy.root_low_lb_append, &low_lb_stats);
    } else {
        memset(&low_lb_stats, 0, sizeof(low_lb_stats));
        low_lb_stats.best_route_gap = 0xFFFF;
    }
    policy.low_lb_hidden_count = low_lb_stats.hidden_count;
    policy.low_lb_in_base_window = low_lb_stats.in_base_window;
    policy.low_lb_best_route_gap = low_lb_stats.best_route_gap;

    bool feature_root_low_lb_append =
        solver_is_root_append_entry(solver, depth) &&
        root_low_lb_basic_append &&
        ctx->reachable_count >= 6 && ctx->lb_span >= 6 && ctx->route_span >= 20 &&
        has_hidden_low_lb;

    bool wide_allows_root_low_lb_append = wide_low_confidence && feature_root_low_lb_append;
    policy.use_root_low_lb_budget =
        depth == 0 && solver &&
        (feature_root_low_lb_budget || policy.use_scan_low_lb_append) &&
        !wide_order;
    policy.reserve_root_low_lb_append =
        depth == 0 && solver &&
        ((feature_root_low_lb_append && !wide_order) ||
         wide_allows_root_low_lb_append || policy.use_scan_low_lb_append);
    policy.allow_root_low_lb_append =
        policy.reserve_root_low_lb_append && root_low_lb_basic_append;
    if (policy.use_root_low_lb_budget) {
        policy.base_attempt_limit += policy.root_low_lb_append;
    }

    policy.proximity_attempt_limit = MAX_PROXIMITY_BOMB_ATTEMPTS;
    policy.rescue_attempt_limit = MAX_RESCUE_BOMB_ATTEMPTS;
    policy.struct_attempt_limit = MAX_STRUCT_BOMB_ATTEMPTS;
    policy.enable_root_push_reach_filter =
        (policy.order_mode == BOMB_ORDER_SMALL || policy.use_maneuver_wide_order) ||
        (policy.order_mode == BOMB_ORDER_BASE && ctx &&
         ctx->top_count == 0 && ctx->shortcut_count == 0 &&
         ctx->maneuver_count * 2 >= ctx->candidate_count);
    return policy;
}


typedef struct {
    int depth;
} BombReachPackEmitCtx;

static void bomb_reach_all_pack_emit(uint8_t slot, const Direction* path, uint16_t len, void* ctx) {
    BombReachPackEmitCtx* pack_ctx = (BombReachPackEmitCtx*)ctx;
    if (!pack_ctx || pack_ctx->depth < 0 || pack_ctx->depth >= MAX_BOMBS) return;
    if (slot >= MAX_BOMB_CANDIDATES || len > MAX_SINGLE_PATH) return;
    packed_path_store_from_direction(g_bomb_reach_all_path_pool[pack_ctx->depth][slot], path, len);
}

static bool bomb_route_final_lane_exists(const BitboardMap* route_map, Position wall) {
    if (!route_map) return false;
    for (int d = 0; d < 4; d++) {
        int pre_x = (int)wall.x - DIRECTIONS[d].dx;
        int pre_y = (int)wall.y - DIRECTIONS[d].dy;
        int stance_x = pre_x - DIRECTIONS[d].dx;
        int stance_y = pre_y - DIRECTIONS[d].dy;
        if (pre_x <= 0 || pre_x >= MAP_COLS - 1 || pre_y <= 0 || pre_y >= MAP_ROWS - 1) continue;
        if (stance_x <= 0 || stance_x >= MAP_COLS - 1 || stance_y <= 0 || stance_y >= MAP_ROWS - 1) continue;
        uint16_t pre_bit = bit_mask_at(pre_x);
        uint16_t stance_bit = bit_mask_at(stance_x);
        uint16_t pre_blocked = (uint16_t)(route_map->walls[pre_y] | route_map->boxes[pre_y] | route_map->bombs[pre_y]);
        uint16_t stance_blocked = (uint16_t)(route_map->walls[stance_y] | route_map->boxes[stance_y] | route_map->bombs[stance_y]);
        if ((pre_blocked & pre_bit) != 0) continue;
        if ((stance_blocked & stance_bit) != 0) continue;
        return true;
    }
    return false;
}

static int bomb_route_build_initial_reach(BitboardMap* route_map, Position player, Position bomb, uint16_t reach[MAP_ROWS]) {
    if (!route_map || !reach) return 0;
    if (bomb.x <= 0 || bomb.x >= MAP_COLS - 1 || bomb.y <= 0 || bomb.y >= MAP_ROWS - 1) return 0;

    uint16_t bomb_bit = bit_mask_at(bomb.x);
    bool bomb_was_set = (route_map->bombs[bomb.y] & bomb_bit) != 0;
    route_map->bombs[bomb.y] |= bomb_bit;
    int reachable_tiles = build_player_reach_mask(route_map, player, reach);
    if (!bomb_was_set) route_map->bombs[bomb.y] &= (uint16_t)(~bomb_bit);
    return reachable_tiles;
}

static bool bomb_route_initial_push_reachable_from_mask(
    const BitboardMap* route_map,
    const uint16_t reach[MAP_ROWS],
    Position bomb,
    Position target_wall
) {
    if (!route_map || !reach) return false;
    for (int d = 0; d < 4; d++) {
        int stance_x = (int)bomb.x - DIRECTIONS[d].dx;
        int stance_y = (int)bomb.y - DIRECTIONS[d].dy;
        int dest_x = (int)bomb.x + DIRECTIONS[d].dx;
        int dest_y = (int)bomb.y + DIRECTIONS[d].dy;
        if (stance_x <= 0 || stance_x >= MAP_COLS - 1 || stance_y <= 0 || stance_y >= MAP_ROWS - 1) continue;
        if (dest_x <= 0 || dest_x >= MAP_COLS - 1 || dest_y <= 0 || dest_y >= MAP_ROWS - 1) continue;

        uint16_t stance_bit = bit_mask_at(stance_x);
        if ((reach[stance_y] & stance_bit) == 0) continue;

        bool dest_is_target_wall = (dest_x == target_wall.x && dest_y == target_wall.y);
        if (!dest_is_target_wall) {
            uint16_t dest_bit = bit_mask_at(dest_x);
            if ((map_blocked_row(route_map, dest_y) & dest_bit) != 0) continue;
        }
        return true;
    }
    return false;
}

static bool bomb_route_initial_push_reachable(BitboardMap* route_map, Position player, Position bomb, Position target_wall) {
    uint16_t reach[MAP_ROWS];
    int reachable_tiles = bomb_route_build_initial_reach(route_map, player, bomb, reach);
    if (reachable_tiles <= 0) return false;
    return bomb_route_initial_push_reachable_from_mask(route_map, reach, bomb, target_wall);
}
static inline void precompute_bomb_routes_range(
    SokobanSolver* solver,
    int depth,
    const BitboardMap* bomb_route_base_map,
    BombWallCandidate* candidates,
    int num_candidates,
    int start_idx,
    int end_idx,
    uint16_t assignment_lower,
    uint16_t local_best
) {
    if (start_idx < 0) start_idx = 0;
    if (end_idx > num_candidates) end_idx = num_candidates;
    if (start_idx >= end_idx) return;
    for (int i = start_idx; i < end_idx; i++) {
        g_bomb_reach_all_len_pool[depth][i] = 0xFFFF;
        bomb_candidate_set_route_precomputed(&candidates[i], false);
        candidates[i].route_slot = (uint8_t)i;
        candidates[i].route_len = 0xFFFF;
    }

    bool allow_precompute_lb_prune = solver && solver->is_scanning;
    Position* reach_targets = g_bomb_reach_targets;
    uint8_t* reach_slots = g_bomb_reach_slots;
    for (uint8_t b_idx = 0; b_idx < solver->num_bombs; b_idx++) {
        Position bomb = solver->bombs[b_idx].pos;
        BitboardMap* route_map = &g_bomb_route_map;
        *route_map = *bomb_route_base_map;
        clear_bit(route_map->bombs, bomb.x, bomb.y);

        int reach_count = 0;
        uint16_t cheap_reach[MAP_ROWS];
        int cheap_reach_tiles = bomb_route_build_initial_reach(route_map, solver->start_player, bomb, cheap_reach);
        for (int i = start_idx; i < end_idx; i++) {
            if (candidates[i].b_idx != b_idx) continue;
            if (allow_precompute_lb_prune && bomb_candidate_pruned_by_local_best(&candidates[i], assignment_lower, local_best)) continue;
            bomb_candidate_set_route_precomputed(&candidates[i], true);
            bool has_final_lane = bomb_route_final_lane_exists(route_map, candidates[i].wall_pos);
            if (!has_final_lane) continue;
            bool has_initial_reach = cheap_reach_tiles > 0 &&
                bomb_route_initial_push_reachable_from_mask(route_map, cheap_reach, bomb, candidates[i].wall_pos);
            if (!has_initial_reach) continue;
            reach_targets[reach_count] = candidates[i].wall_pos;
            reach_slots[reach_count] = (uint8_t)i;
            reach_count++;
        }
        if (reach_count == 0) continue;

        uint16_t backup_max_steps = g_astar_max_steps;
        g_astar_max_steps = 100;
        BombReachPackEmitCtx pack_ctx = { depth };
        astar_bomb_reach_all_emit(solver->heap, solver->closed_list, route_map,
                                  solver->start_player, bomb,
                                  reach_targets, reach_slots, reach_count,
                                  bomb_reach_all_pack_emit, &pack_ctx,
                                  g_bomb_reach_all_len_pool[depth]);
        g_astar_max_steps = backup_max_steps;
    }

    for (int i = start_idx; i < end_idx; i++) {
        candidates[i].route_len = g_bomb_reach_all_len_pool[depth][i];
    }
}
static bool route_target_filled_by_box(const SokobanSolver* solver, Position target_pos) {
    for (int b = 0; b < solver->num_boxes; b++) {
        if (pos_equal(target_pos, solver->boxes[b].pos)) return true;
    }
    return false;
}

static bool route_target_matches_active_box(const SokobanSolver* solver, int active_box_idx, int target_idx) {
    if (!solver || active_box_idx < 0 || active_box_idx >= solver->num_boxes ||
        target_idx < 0 || target_idx >= solver->num_targets) {
        return false;
    }
    if (!solver->strict_target_mode) return true;
    int b_id = solver->boxes[active_box_idx].id;
    int t_id = solver->targets[target_idx].id;
    return b_id == -1 || t_id == -1 || b_id == t_id;
}

static void prepare_route_map_targets(const SokobanSolver* solver, BitboardMap* route_map,
                                      int active_box_idx, bool is_bomb_entity,
                                      bool mark_deadlocks) {
    if (!solver || !route_map) return;

    for (int k = 0; k < solver->num_targets; k++) {
        Position tp = solver->targets[k].pos;
        if (route_target_filled_by_box(solver, tp)) continue;

        if (mark_deadlocks && !is_bomb_entity && route_target_matches_active_box(solver, active_box_idx, k)) {
            route_map->deadlocks[tp.y] |= bit_mask_at(tp.x);
        }
        clear_bit(route_map->targets, tp.x, tp.y);
    }
}

static void build_bomb_route_base_map(const SokobanSolver* solver, BitboardMap* route_map) {
    if (!solver || !route_map) return;

    *route_map = solver->bmap;
    memset(route_map->boxes, 0, sizeof(route_map->boxes));
    for (int k = 0; k < solver->num_boxes; k++) {
        Position bp = solver->boxes[k].pos;
        if ((solver->bmap.targets[bp.y] & bit_mask_at(bp.x)) == 0) {
            set_box_bit(route_map, bp.x, bp.y);
        }
    }
    prepare_route_map_targets(solver, route_map, -1, true, false);
}
static inline bool light_evac_inner_cell(Position p) {
    return p.x > 0 && p.x < MAP_COLS - 1 && p.y > 0 && p.y < MAP_ROWS - 1;
}

static bool light_evac_box_is_non_target(const SokobanSolver* solver, int box_idx) {
    if (!solver || box_idx < 0 || box_idx >= solver->num_boxes) return false;
    if (!solver->boxes[box_idx].is_active) return false;
    Position bp = solver->boxes[box_idx].pos;
    if (!light_evac_inner_cell(bp)) return false;
    return (solver->bmap.targets[bp.y] & bit_mask_at(bp.x)) == 0;
}

static int light_evac_box_index_at(const SokobanSolver* solver, Position pos) {
    if (!solver || !light_evac_inner_cell(pos)) return -1;
    for (int i = 0; i < solver->num_boxes; i++) {
        if (light_evac_box_is_non_target(solver, i) && pos_equal(solver->boxes[i].pos, pos)) return i;
    }
    return -1;
}

static void light_evac_add_box_index(uint8_t* box_indices, int* count, int box_idx) {
    if (!box_indices || !count || box_idx < 0 || box_idx >= MAX_BOXES) return;
    for (int i = 0; i < *count; i++) {
        if (box_indices[i] == (uint8_t)box_idx) return;
    }
    if (*count < MAX_BOXES) box_indices[(*count)++] = (uint8_t)box_idx;
}

static void light_evac_add_boxes_near_point(
    const SokobanSolver* solver,
    Position anchor,
    int radius,
    uint8_t* box_indices,
    int* count
) {
    if (!solver || !box_indices || !count || !light_evac_inner_cell(anchor)) return;
    if (radius < 0) radius = 0;

    for (int i = 0; i < solver->num_boxes; i++) {
        if (!light_evac_box_is_non_target(solver, i)) continue;
        Position bp = solver->boxes[i].pos;
        if (abs((int)bp.x - (int)anchor.x) <= radius &&
            abs((int)bp.y - (int)anchor.y) <= radius) {
            light_evac_add_box_index(box_indices, count, i);
        }
    }
}

static void light_evac_add_boxes_on_aligned_line(
    const SokobanSolver* solver,
    Position from,
    Position wall,
    uint8_t* box_indices,
    int* count
) {
    if (!solver || !box_indices || !count || !light_evac_inner_cell(from)) return;

    if (from.x == wall.x) {
        int step = (wall.y > from.y) ? 1 : -1;
        for (int y = (int)from.y + step; y != (int)wall.y; y += step) {
            if (y <= 0 || y >= MAP_ROWS - 1) break;
            int idx = light_evac_box_index_at(solver, (Position){from.x, (uint8_t)y});
            light_evac_add_box_index(box_indices, count, idx);
        }
    } else if (from.y == wall.y) {
        int step = (wall.x > from.x) ? 1 : -1;
        for (int x = (int)from.x + step; x != (int)wall.x; x += step) {
            if (x <= 0 || x >= MAP_COLS - 1) break;
            int idx = light_evac_box_index_at(solver, (Position){(uint8_t)x, from.y});
            light_evac_add_box_index(box_indices, count, idx);
        }
    }
}

static void light_evac_collect_route_blocking_boxes(
    const SokobanSolver* solver,
    int depth,
    const BombWallCandidate* cand,
    uint8_t* box_indices,
    int* count
) {
    if (!solver || !cand || !box_indices || !count) return;
    if (depth < 0 || depth >= MAX_BOMBS || cand->b_idx >= solver->num_bombs) return;
    if (!bomb_candidate_route_precomputed(cand) || cand->route_len == 0xFFFF || cand->route_len >= MAX_SINGLE_PATH) return;
    if (cand->route_slot >= MAX_BOMB_CANDIDATES) return;

    Position player = solver->start_player;
    Position bomb = solver->bombs[cand->b_idx].pos;
    const PackedDirByte* route_path = g_bomb_reach_all_path_pool[depth][cand->route_slot];

    light_evac_add_boxes_near_point(solver, bomb, 2, box_indices, count);
    light_evac_add_boxes_on_aligned_line(solver, bomb, cand->wall_pos, box_indices, count);

    for (uint16_t step = 0; step < cand->route_len; step++) {
        Direction d = packed_path_step(route_path, step);
        if (direction_is_pause(d)) continue;
        if (!direction_is_cardinal(d)) break;

        int nx = (int)player.x + d.dx;
        int ny = (int)player.y + d.dy;
        if (!is_in_bounds(nx, ny)) break;
        Position next_player = {(uint8_t)nx, (uint8_t)ny};

        if (pos_equal(next_player, bomb)) {
            Position old_bomb = bomb;
            int bx = (int)bomb.x + d.dx;
            int by = (int)bomb.y + d.dy;
            if (!is_in_bounds(bx, by)) break;
            Position new_bomb = {(uint8_t)bx, (uint8_t)by};

            light_evac_add_boxes_near_point(solver, old_bomb, 2, box_indices, count);
            light_evac_add_boxes_near_point(solver, new_bomb, 2, box_indices, count);
            light_evac_add_boxes_on_aligned_line(solver, old_bomb, cand->wall_pos, box_indices, count);
            light_evac_add_boxes_on_aligned_line(solver, new_bomb, cand->wall_pos, box_indices, count);

            bomb = new_bomb;
        }

        player = next_player;
    }
}
static bool light_evac_push_line_clear(const SokobanSolver* solver, const BitboardMap* map, const BombWallCandidate* cand) {
    if (!solver || !map || !cand || cand->b_idx >= solver->num_bombs) return false;
    Position bomb = solver->bombs[cand->b_idx].pos;
    Position wall = cand->wall_pos;

    if (bomb.x == wall.x) {
        int step = (wall.y > bomb.y) ? 1 : -1;
        for (int y = (int)bomb.y + step; y != (int)wall.y; y += step) {
            if ((map->boxes[y] & bit_mask_at(bomb.x)) != 0) return false;
        }
    } else if (bomb.y == wall.y) {
        int step = (wall.x > bomb.x) ? 1 : -1;
        for (int x = (int)bomb.x + step; x != (int)wall.x; x += step) {
            if ((map->boxes[bomb.y] & bit_mask_at(x)) != 0) return false;
        }
    }
    return true;
}

static bool light_evac_direct_line_blocked_by_single_box(
    const SokobanSolver* solver,
    Position bomb,
    Position wall,
    uint16_t* push_hint
) {
    if (!solver) return false;
    if (bomb.x != wall.x && bomb.y != wall.y) return false;

    int dx = (wall.x > bomb.x) ? 1 : ((wall.x < bomb.x) ? -1 : 0);
    int dy = (wall.y > bomb.y) ? 1 : ((wall.y < bomb.y) ? -1 : 0);
    if (dx == 0 && dy == 0) return false;

    int blockers = 0;
    int x = (int)bomb.x + dx;
    int y = (int)bomb.y + dy;
    while (x != (int)wall.x || y != (int)wall.y) {
        if (x <= 0 || x >= MAP_COLS - 1 || y <= 0 || y >= MAP_ROWS - 1) return false;
        uint16_t bit = bit_mask_at(x);
        if ((solver->bmap.walls[y] & bit) != 0) return false;
        if ((solver->bmap.bombs[y] & bit) != 0) return false;
        if ((solver->bmap.boxes[y] & bit) != 0) {
            int box_idx = light_evac_box_index_at(solver, (Position){(uint8_t)x, (uint8_t)y});
            if (box_idx < 0) return false;
            blockers++;
            if (blockers > 1) return false;
        }
        x += dx;
        y += dy;
    }

    if (blockers != 1) return false;
    if (push_hint) {
        uint16_t dist = manhattan_distance(bomb, wall);
        *push_hint = (dist > 0) ? (uint16_t)(dist - 1) : 0;
    }
    return true;
}

static bool light_evac_final_stance_clear(const SokobanSolver* solver, const BitboardMap* map, const BombWallCandidate* cand) {
    if (!solver || !map || !cand || cand->b_idx >= solver->num_bombs) return false;

    Position bomb = solver->bombs[cand->b_idx].pos;
    BitboardMap temp_map = *map;
    if (is_in_bounds(bomb.x, bomb.y)) clear_bit(temp_map.bombs, bomb.x, bomb.y);
    Position target_wall = cand->wall_pos;
    for (int d = 0; d < 4; d++) {
        int pre_x = target_wall.x - DIRECTIONS[d].dx;
        int pre_y = target_wall.y - DIRECTIONS[d].dy;
        int stance_x = pre_x - DIRECTIONS[d].dx;
        int stance_y = pre_y - DIRECTIONS[d].dy;
        if (pre_x <= 0 || pre_x >= MAP_COLS - 1 || pre_y <= 0 || pre_y >= MAP_ROWS - 1) continue;
        if (stance_x <= 0 || stance_x >= MAP_COLS - 1 || stance_y <= 0 || stance_y >= MAP_ROWS - 1) continue;

        uint16_t pre_bit = bit_mask_at(pre_x);
        uint16_t stance_bit = bit_mask_at(stance_x);
        if ((map_blocked_row(&temp_map, pre_y) & pre_bit) != 0) continue;
        if ((map_blocked_row(&temp_map, stance_y) & stance_bit) != 0) continue;
        return true;
    }
    return false;
}

static int light_evac_collect_blocking_boxes(const SokobanSolver* solver, int depth, const BombWallCandidate* cand, uint8_t* box_indices) {
    if (!solver || !cand || !box_indices || cand->b_idx >= solver->num_bombs) return 0;

    int count = 0;
    Position bomb = solver->bombs[cand->b_idx].pos;
    Position wall = cand->wall_pos;

    for (int d = 0; d < 4; d++) {
        int pre_x = wall.x - DIRECTIONS[d].dx;
        int pre_y = wall.y - DIRECTIONS[d].dy;
        int stance_x = pre_x - DIRECTIONS[d].dx;
        int stance_y = pre_y - DIRECTIONS[d].dy;
        if (pre_x > 0 && pre_x < MAP_COLS - 1 && pre_y > 0 && pre_y < MAP_ROWS - 1) {
            int idx = light_evac_box_index_at(solver, (Position){(uint8_t)pre_x, (uint8_t)pre_y});
            light_evac_add_box_index(box_indices, &count, idx);
        }
        if (stance_x > 0 && stance_x < MAP_COLS - 1 && stance_y > 0 && stance_y < MAP_ROWS - 1) {
            int idx = light_evac_box_index_at(solver, (Position){(uint8_t)stance_x, (uint8_t)stance_y});
            light_evac_add_box_index(box_indices, &count, idx);
        }
    }

    if (bomb.x == wall.x) {
        int step = (wall.y > bomb.y) ? 1 : -1;
        for (int y = (int)bomb.y + step; y != (int)wall.y; y += step) {
            int idx = light_evac_box_index_at(solver, (Position){bomb.x, (uint8_t)y});
            light_evac_add_box_index(box_indices, &count, idx);
        }
    } else if (bomb.y == wall.y) {
        int step = (wall.x > bomb.x) ? 1 : -1;
        for (int x = (int)bomb.x + step; x != (int)wall.x; x += step) {
            int idx = light_evac_box_index_at(solver, (Position){(uint8_t)x, bomb.y});
            light_evac_add_box_index(box_indices, &count, idx);
        }
    }

    int min_x = (bomb.x < wall.x) ? bomb.x : wall.x;
    int max_x = (bomb.x > wall.x) ? bomb.x : wall.x;
    int min_y = (bomb.y < wall.y) ? bomb.y : wall.y;
    int max_y = (bomb.y > wall.y) ? bomb.y : wall.y;
    for (int i = 0; i < solver->num_boxes; i++) {
        if (!light_evac_box_is_non_target(solver, i)) continue;
        Position bp = solver->boxes[i].pos;
        bool near_endpoint = (abs((int)bp.x - (int)bomb.x) <= 2 && abs((int)bp.y - (int)bomb.y) <= 2) ||
                             (abs((int)bp.x - (int)wall.x) <= 2 && abs((int)bp.y - (int)wall.y) <= 2);
        bool near_line = false;
        if (bomb.x == wall.x) {
            near_line = abs((int)bp.x - (int)bomb.x) <= 2 &&
                        (int)bp.y >= min_y - 2 && (int)bp.y <= max_y + 2;
        } else if (bomb.y == wall.y) {
            near_line = abs((int)bp.y - (int)bomb.y) <= 2 &&
                        (int)bp.x >= min_x - 2 && (int)bp.x <= max_x + 2;
        }
        if (near_endpoint || near_line) light_evac_add_box_index(box_indices, &count, i);
    }

    light_evac_collect_route_blocking_boxes(solver, depth, cand, box_indices, &count);

    return count;
}
static bool light_evac_parking_legal(
    const SokobanSolver* solver,
    const BombWallCandidate* cand,
    int box_idx,
    Position parking
) {
    if (!solver || !cand || !light_evac_box_is_non_target(solver, box_idx) || !light_evac_inner_cell(parking)) return false;
    Position old_box = solver->boxes[box_idx].pos;
    if (pos_equal(old_box, parking)) return false;

    uint16_t bit = bit_mask_at(parking.x);
    if (((solver->bmap.walls[parking.y] | solver->bmap.boxes[parking.y] |
          solver->bmap.bombs[parking.y] | solver->bmap.targets[parking.y] |
          solver->bmap.deadlocks[parking.y]) & bit) != 0) {
        return false;
    }

    BitboardMap after_map = solver->bmap;
    clear_bit(after_map.boxes, old_box.x, old_box.y);
    set_bit(after_map.boxes, parking.x, parking.y);
    if (!light_evac_final_stance_clear(solver, &after_map, cand)) return false;
    if (!light_evac_push_line_clear(solver, &after_map, cand)) return false;
    return true;
}

static bool light_evac_analyze_route(
    const SokobanSolver* solver,
    int box_idx,
    const Direction* route,
    uint16_t route_len,
    Position* out_player,
    Position* out_box,
    uint8_t* out_push_count
) {
    if (!solver || !route || box_idx < 0 || box_idx >= solver->num_boxes || route_len == 0 || route_len >= MAX_SINGLE_PATH) return false;

    BitboardMap sim_map = solver->bmap;
    Position curr_p = solver->start_player;
    Position sim_boxes[MAX_BOXES];
    Entity sim_bombs[MAX_BOMBS];
    int bomb_count = solver->num_bombs;
    for (int i = 0; i < MAX_BOXES; i++) sim_boxes[i] = (Position){0xFF, 0xFF};
    for (int i = 0; i < solver->num_boxes && i < MAX_BOXES; i++) sim_boxes[i] = solver->boxes[i].pos;
    for (int i = 0; i < solver->num_bombs && i < MAX_BOMBS; i++) sim_bombs[i] = solver->bombs[i];

    uint8_t pushes = 0;
    for (uint16_t i = 0; i < route_len; i++) {
        Direction d = route[i];
        if (direction_is_pause(d)) continue;
        if (!direction_is_cardinal(d)) return false;

        int nx = (int)curr_p.x + d.dx;
        int ny = (int)curr_p.y + d.dy;
        if (!is_in_bounds(nx, ny)) return false;
        Position next_p = {(uint8_t)nx, (uint8_t)ny};
        if (get_bit(sim_map.bombs, next_p.x, next_p.y)) return false;

        bool pushes_box = get_bit(sim_map.boxes, next_p.x, next_p.y);
        if (pushes_box) {
            int pushed_idx = tracked_position_index(sim_boxes, solver->num_boxes, next_p);
            if (pushed_idx != box_idx) return false;
            Position before = sim_boxes[box_idx];
            if (!sq_apply_path_steps(solver, &sim_map, &curr_p, sim_boxes, sim_bombs, &bomb_count, &route[i], 1)) return false;
            if (!pos_equal(before, sim_boxes[box_idx])) pushes++;
            if (pushes > LIGHT_EVAC_MAX_PUSHES) return false;
        } else {
            if (!sq_apply_path_steps(solver, &sim_map, &curr_p, sim_boxes, sim_bombs, &bomb_count, &route[i], 1)) return false;
        }
    }

    if (sim_boxes[box_idx].x == 0xFF) return false;
    if (out_player) *out_player = curr_p;
    if (out_box) *out_box = sim_boxes[box_idx];
    if (out_push_count) *out_push_count = pushes;
    return true;
}

static bool light_evac_apply_plan(SokobanSolver* solver, const LightEvacPlan* plan) {
    if (!solver || !plan || plan->evac_len == 0 || plan->evac_len >= MAX_SINGLE_PATH) return false;
    

    BitboardMap sim_map = solver->bmap;
    Position curr_p = solver->start_player;
    Position sim_boxes[MAX_BOXES];
    Entity sim_bombs[MAX_BOMBS];
    int bomb_count = solver->num_bombs;
    for (int i = 0; i < MAX_BOXES; i++) sim_boxes[i] = (Position){0xFF, 0xFF};
    for (int i = 0; i < solver->num_boxes && i < MAX_BOXES; i++) sim_boxes[i] = solver->boxes[i].pos;
    for (int i = 0; i < solver->num_bombs && i < MAX_BOMBS; i++) sim_bombs[i] = solver->bombs[i];

    if (!sq_apply_path_steps(solver, &sim_map, &curr_p, sim_boxes, sim_bombs, &bomb_count,
                             plan->evac_path, plan->evac_len)) {
        return false;
    }
    if (plan->box_idx >= solver->num_boxes || !pos_equal(sim_boxes[plan->box_idx], plan->target_parking)) return false;

    solver->bmap = sim_map;
    solver->start_player = curr_p;
    solver->num_bombs = (uint8_t)bomb_count;
    for (int i = 0; i < solver->num_boxes && i < MAX_BOXES; i++) solver->boxes[i].pos = sim_boxes[i];
    for (int i = 0; i < solver->num_bombs && i < MAX_BOMBS; i++) solver->bombs[i] = sim_bombs[i];
    
    return true;
}

static void light_evac_insert_plan(LightEvacPlan* plans, int* count, const LightEvacPlan* plan) {
    if (!plans || !count || !plan) return;
    for (int i = 0; i < *count; i++) {
        if (plans[i].box_idx == plan->box_idx && pos_equal(plans[i].target_parking, plan->target_parking)) {
            if (plans[i].score >= plan->score) return;
            plans[i] = *plan;
            return;
        }
    }

    int idx = *count;
    if (idx < MAX_LIGHT_EVAC_PLANS) {
        (*count)++;
    } else if (plan->score > plans[MAX_LIGHT_EVAC_PLANS - 1].score) {
        idx = MAX_LIGHT_EVAC_PLANS - 1;
    } else {
        return;
    }

    while (idx > 0 && plans[idx - 1].score < plan->score) {
        plans[idx] = plans[idx - 1];
        idx--;
    }
    plans[idx] = *plan;
}

static int __attribute__((noinline)) build_light_evac_plans(SokobanSolver* solver, int depth, const BombWallCandidate* cand, LightEvacPlan* plans) {
    if (!solver || !cand || !plans || depth < 0 || depth >= MAX_BOMBS) return 0;
    

    BombLightScratch* light_scratch = &g_bomb_light_scratch[depth];
    uint8_t* box_indices = light_scratch->blocker_indices;
    int num_boxes = light_evac_collect_blocking_boxes(solver, depth, cand, box_indices);
    if (num_boxes <= 0) return 0;

    int count = 0;
    for (int bi = 0; bi < num_boxes; bi++) {
        int box_idx = box_indices[bi];
        if (!light_evac_box_is_non_target(solver, box_idx)) continue;
        Position old_box = solver->boxes[box_idx].pos;

        for (int d = 0; d < 4; d++) {
            for (int pushes = 1; pushes <= LIGHT_EVAC_MAX_PUSHES; pushes++) {
                int px = (int)old_box.x + (DIRECTIONS[d].dx * pushes);
                int py = (int)old_box.y + (DIRECTIONS[d].dy * pushes);
                if (px <= 0 || px >= MAP_COLS - 1 || py <= 0 || py >= MAP_ROWS - 1) continue;
                Position parking = {(uint8_t)px, (uint8_t)py};
                bool parking_ok = light_evac_parking_legal(solver, cand, box_idx, parking);
                if (!parking_ok) continue;

                LightEvacPlan* plan = &light_scratch->temp_plan;
                memset(plan, 0, sizeof(*plan));
                plan->box_idx = (uint8_t)box_idx;
                plan->old_box = old_box;
                plan->target_parking = parking;

                BitboardMap* route_map = &light_scratch->evac_route_map;
                *route_map = solver->bmap;
                clear_bit(route_map->boxes, old_box.x, old_box.y);

                uint16_t saved_astar = g_astar_max_steps;
                uint16_t route_len = 0;
                int dummy = -1;
                hash_table_clear();
                g_astar_max_steps = LIGHT_EVAC_ROUTE_STEP_LIMIT;
                bool routed = astar_solve_with_mask(
                    solver->heap, solver->closed_list, route_map,
                    solver->start_player, old_box, &parking, 1,
                    &dummy, MASK_WALL | MASK_BOMB | MASK_BOX,
                    plan->evac_path, &route_len, ASTAR_NO_MACRO_DEPTH, ROUTE_SUPER_EVAC
                );
                g_astar_max_steps = saved_astar;
                if (!routed || route_len == 0 || route_len >= MAX_SINGLE_PATH) {
                    continue;
                }

                Position player_after = {0xFF, 0xFF};
                Position box_after = {0xFF, 0xFF};
                uint8_t push_count = 0;
                if (!light_evac_analyze_route(solver, box_idx, plan->evac_path, route_len,
                                              &player_after, &box_after, &push_count)) {
                    continue;
                }
                if (push_count == 0 || push_count > LIGHT_EVAC_MAX_PUSHES) continue;
                if (!pos_equal(box_after, parking)) continue;

                plan->player_after = player_after;
                plan->evac_len = route_len;
                plan->push_count = push_count;
                plan->score = 1000 - (int)route_len - ((int)push_count * 20) - manhattan_distance(old_box, parking);
                light_evac_insert_plan(plans, &count, plan);
            }
        }
    }

    
    return count;
}
static int compute_candidate_maneuver_bonus(
    const SokobanSolver* solver,
    int x,
    int y,
    Position bomb,
    const uint16_t dist_P[MAP_ROWS][MAP_COLS]
) {
    uint16_t maneuver_dynamic_obs[MAP_ROWS];
    for (int r = 0; r < MAP_ROWS; r++) {
        maneuver_dynamic_obs[r] = (uint16_t)(solver->bmap.boxes[r] | solver->bmap.bombs[r]);
    }
    maneuver_dynamic_obs[bomb.y] = (uint16_t)(maneuver_dynamic_obs[bomb.y] & (uint16_t)~(1u << bomb.x));

    bool has_near_maneuver_entity = false;
    for (int ey = y - 2; ey <= y + 2 && !has_near_maneuver_entity; ey++) {
        for (int ex = x - 2; ex <= x + 2; ex++) {
            if (ey <= 0 || ey >= MAP_ROWS - 1 || ex <= 0 || ex >= MAP_COLS - 1) continue;
            uint16_t entity_bit = (uint16_t)(1u << ex);
            if ((maneuver_dynamic_obs[ey] & entity_bit) != 0) {
                has_near_maneuver_entity = true;
                break;
            }
        }
    }
    if (!has_near_maneuver_entity) return 0;

    bool blast_reach[3][3] = {false};
    Position q_blast[9];
    int head_b = 0;
    int tail_b = 0;

    for (int by = y - 1; by <= y + 1; by++) {
        for (int bx = x - 1; bx <= x + 1; bx++) {
            if (bx <= 0 || bx >= MAP_COLS - 1 || by <= 0 || by >= MAP_ROWS - 1) continue;

            uint16_t blast_bit = (uint16_t)(1u << bx);
            bool is_hard = ((solver->bmap.walls[by] & blast_bit) != 0 && (g_destructible_mask[by] & blast_bit) == 0);
            bool has_ent = ((maneuver_dynamic_obs[by] & blast_bit) != 0);
            if (is_hard || has_ent) continue;

            bool can_enter = false;
            for (int d = 0; d < 4; d++) {
                int px = bx + DIRECTIONS[d].dx;
                int py = by + DIRECTIONS[d].dy;
                if (px <= 0 || px >= MAP_COLS - 1 || py <= 0 || py >= MAP_ROWS - 1) continue;
                if (abs(px - x) <= 1 && abs(py - y) <= 1) continue;
                if (dist_P[py][px] != 0xFFFF) {
                    can_enter = true;
                    break;
                }
            }

            if (can_enter) {
                blast_reach[by - y + 1][bx - x + 1] = true;
                q_blast[tail_b++] = (Position){(uint8_t)bx, (uint8_t)by};
            }
        }
    }

    while (head_b < tail_b) {
        Position curr = q_blast[head_b++];
        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
            if (abs(nx - x) > 1 || abs(ny - y) > 1) continue;
            if (blast_reach[ny - y + 1][nx - x + 1]) continue;

            uint16_t bit = (uint16_t)(1u << nx);
            bool is_hard = ((solver->bmap.walls[ny] & bit) != 0 && (g_destructible_mask[ny] & bit) == 0);
            bool has_ent = ((maneuver_dynamic_obs[ny] & bit) != 0);
            if (is_hard || has_ent) continue;

            blast_reach[ny - y + 1][nx - x + 1] = true;
            q_blast[tail_b++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }

    if (tail_b == 0) return 0;

    int local_maneuver_bonus = 0;
    for (int ey = y - 2; ey <= y + 2; ey++) {
        for (int ex = x - 2; ex <= x + 2; ex++) {
            if (ey <= 0 || ey >= MAP_ROWS - 1 || ex <= 0 || ex >= MAP_COLS - 1) continue;

            uint16_t entity_bit = (uint16_t)(1u << ex);
            bool is_box = ((solver->bmap.boxes[ey] & entity_bit) != 0);
            bool is_bomb = ((solver->bmap.bombs[ey] & entity_bit) != 0);
            if (is_bomb && ex == bomb.x && ey == bomb.y) is_bomb = false;
            if (!is_box && !is_bomb) continue;

            if (is_box && ((solver->bmap.targets[ey] & entity_bit) != 0)) {
                bool box_done = true;
                if (solver->strict_target_mode) {
                    int b_id = -1;
                    int t_id = -1;
                    for (int k = 0; k < solver->num_boxes; k++) {
                        if (solver->boxes[k].pos.x == ex && solver->boxes[k].pos.y == ey) { b_id = solver->boxes[k].id; break; }
                    }
                    for (int k = 0; k < solver->num_targets; k++) {
                        if (solver->targets[k].pos.x == ex && solver->targets[k].pos.y == ey) { t_id = solver->targets[k].id; break; }
                    }
                    if (b_id != -1 && t_id != -1 && b_id != t_id) box_done = false;
                }
                if (box_done) continue;
            }

            for (int d = 0; d < 4; d++) {
                int py = ey - DIRECTIONS[d].dy;
                int px = ex - DIRECTIONS[d].dx;
                int ny = ey + DIRECTIONS[d].dy;
                int nx = ex + DIRECTIONS[d].dx;

                if (py <= 0 || py >= MAP_ROWS - 1 || px <= 0 || px >= MAP_COLS - 1) continue;
                if (ny <= 0 || ny >= MAP_ROWS - 1 || nx <= 0 || nx >= MAP_COLS - 1) continue;

                bool stance_in_blast = (abs(py - y) <= 1 && abs(px - x) <= 1);
                bool dest_in_blast = (abs(ny - y) <= 1 && abs(nx - x) <= 1);

                uint16_t stance_bit = (uint16_t)(1u << px);
                uint16_t dest_bit = (uint16_t)(1u << nx);
                bool stance_before = (dist_P[py][px] != 0xFFFF);
                bool dest_before_clear = ((solver->bmap.walls[ny] & dest_bit) == 0) &&
                                         ((maneuver_dynamic_obs[ny] & dest_bit) == 0);
                bool push_before_valid = stance_before && dest_before_clear;

                bool stance_after_valid = false;
                if ((maneuver_dynamic_obs[py] & stance_bit) == 0) {
                    stance_after_valid = stance_before || (stance_in_blast && blast_reach[py - y + 1][px - x + 1]);
                }

                bool dest_is_hard_wall = ((solver->bmap.walls[ny] & dest_bit) != 0 && (g_destructible_mask[ny] & dest_bit) == 0);
                bool dest_after_clear = false;
                if (!dest_is_hard_wall && ((maneuver_dynamic_obs[ny] & dest_bit) == 0)) {
                    dest_after_clear = dest_in_blast || ((solver->bmap.walls[ny] & dest_bit) == 0);
                }

                bool push_after_valid = stance_after_valid && dest_after_clear;
                if (!push_after_valid) continue;

                if (!push_before_valid) {
                    if (is_box) {
                        if (!dest_in_blast && ((solver->bmap.deadlocks[ny] & dest_bit) != 0)) continue;
                        local_maneuver_bonus += 4500;
                    } else if (is_bomb) {
                        local_maneuver_bonus += 2500;
                    }
                } else {
                    int nny = ny + DIRECTIONS[d].dy;
                    int nnx = nx + DIRECTIONS[d].dx;
                    bool forward_extension = false;

                    if (nny > 0 && nny < MAP_ROWS - 1 && nnx > 0 && nnx < MAP_COLS - 1) {
                        uint16_t next_bit = (uint16_t)(1u << nnx);
                        bool next_in_blast = (abs(nny - y) <= 1 && abs(nnx - x) <= 1);
                        bool next_was_soft_wall = ((solver->bmap.walls[nny] & next_bit) != 0 && (g_destructible_mask[nny] & next_bit) != 0);
                        if (next_in_blast && next_was_soft_wall) {
                            forward_extension = true;
                            local_maneuver_bonus += is_box ? 5000 : 3000;
                        }
                    }

                    if (!forward_extension) {
                        int side_dirs[2] = {(d + 1) % 4, (d + 3) % 4};
                        bool side_pocket_opened = false;

                        for (int s = 0; s < 2 && !side_pocket_opened; s++) {
                            int sd = side_dirs[s];
                            int check_x[2] = {ex + DIRECTIONS[sd].dx, nx + DIRECTIONS[sd].dx};
                            int check_y[2] = {ey + DIRECTIONS[sd].dy, ny + DIRECTIONS[sd].dy};

                            for (int cp = 0; cp < 2; cp++) {
                                int cx = check_x[cp];
                                int cy = check_y[cp];
                                if (cx <= 0 || cx >= MAP_COLS - 1 || cy <= 0 || cy >= MAP_ROWS - 1) continue;

                                uint16_t side_bit = (uint16_t)(1u << cx);
                                bool pt_in_blast = (abs(cy - y) <= 1 && abs(cx - x) <= 1);
                                bool pt_was_soft_wall = ((solver->bmap.walls[cy] & side_bit) != 0 && (g_destructible_mask[cy] & side_bit) != 0);
                                if (pt_in_blast && pt_was_soft_wall) {
                                    side_pocket_opened = true;
                                    break;
                                }
                            }
                        }

                        if (side_pocket_opened) {
                            local_maneuver_bonus += is_box ? 4000 : 2000;
                        }
                    }
                }
            }
        }
    }

    if (local_maneuver_bonus > 7500) local_maneuver_bonus = 7500;
    return local_maneuver_bonus;
}
static bool __attribute__((noinline)) try_bomb_candidate_branch(
    SokobanSolver* solver,
    int depth,
    uint32_t current_hash,
    const BitboardMap* bomb_route_base_map,
    const BombWallCandidate* cand,
    Direction* bomb_path,
    uint16_t assignment_lower,
    uint16_t* local_best,
    bool* found,
    int* valid_attempts,
    int max_valid_attempts,
    bool force_maneuver_rescue
) {
    if (g_solve_budget_exhausted) {
        return true;
    }

    if (bomb_candidate_pruned_by_local_best(cand, assignment_lower, *local_best)) {
        return false;
    }
    if (bomb_candidate_route_precomputed(cand) && cand->route_len == 0xFFFF) {
        return false;
    }

    uint8_t b_idx = cand->b_idx;
    Position target_wall = cand->wall_pos;
    Position bomb = solver->bombs[b_idx].pos;
    uint16_t reach_key = bomb_reach_fail_key(bomb, target_wall);
    uint16_t reach_context = bomb_reach_fail_context(solver);
    if (!bomb_candidate_route_precomputed(cand) && bomb_reach_fail_cached(current_hash, reach_key, reach_context)) {
        return false;
    }

    BombBranchScratch* branch_scratch = &g_bomb_branch_scratch[depth];
    BitboardMap* temp_map = &branch_scratch->temp_map;
    *temp_map = *bomb_route_base_map;
    clear_bit(temp_map->bombs, bomb.x, bomb.y);

    bool final_stance_possible = false;
    for (int d = 0; d < 4; d++) {
        int pre_x = target_wall.x - DIRECTIONS[d].dx;
        int pre_y = target_wall.y - DIRECTIONS[d].dy;
        int stance_x = pre_x - DIRECTIONS[d].dx;
        int stance_y = pre_y - DIRECTIONS[d].dy;
        if (pre_x <= 0 || pre_x >= MAP_COLS - 1 || pre_y <= 0 || pre_y >= MAP_ROWS - 1) continue;
        if (stance_x <= 0 || stance_x >= MAP_COLS - 1 || stance_y <= 0 || stance_y >= MAP_ROWS - 1) continue;
        uint16_t pre_bit = (uint16_t)(1u << pre_x);
        uint16_t stance_bit = (uint16_t)(1u << stance_x);
        if ((map_blocked_row(temp_map, pre_y) & pre_bit) != 0) continue;
        if ((map_blocked_row(temp_map, stance_y) & stance_bit) != 0) continue;
        final_stance_possible = true;
        break;
    }
    if (!final_stance_possible) {
        bomb_reach_fail_store(current_hash, reach_key, reach_context);
        return false;
    }

    if (!bomb_candidate_route_precomputed(cand)) {
        bool initial_push_reachable = bomb_route_initial_push_reachable(temp_map, solver->start_player, bomb, target_wall);
        if (!initial_push_reachable) {
            return false;
        }
    }

    uint16_t path_len = 0xFFFF;
    bool can_reach = false;
    if (bomb_candidate_route_precomputed(cand)) {
        if (cand->route_len == 0xFFFF) {
            bomb_reach_fail_store(current_hash, reach_key, reach_context);
            return false;
        }
        path_len = cand->route_len;
        packed_path_load_to_direction(g_bomb_reach_all_path_pool[depth][cand->route_slot], bomb_path, path_len);
        can_reach = true;
    } else {
        can_reach = false;
    }

    int dummy_idx = -1;
    uint16_t backup_max_steps = g_astar_max_steps;
    uint16_t bomb_step_budget = 100;
    if (*local_best != 0xFFFF && assignment_lower != 0xFFFF && *local_best > assignment_lower) {
        uint16_t improvement_budget = (uint16_t)(*local_best - assignment_lower);
        if (improvement_budget < bomb_step_budget) bomb_step_budget = improvement_budget;
    }
    if (!can_reach) {
        hash_table_clear();
        g_astar_max_steps = bomb_step_budget;
        can_reach = astar_solve_with_mask(solver->heap, solver->closed_list, temp_map, solver->start_player, bomb,
                                          &target_wall, 1, &dummy_idx, MASK_WALL | MASK_BOMB | MASK_BOX,
                                          bomb_path, &path_len, ASTAR_NO_MACRO_DEPTH, ROUTE_BOMB_ATTACK);
        g_astar_max_steps = backup_max_steps;

        if (!can_reach) {
            bomb_reach_fail_store(current_hash, reach_key, reach_context);
            return false;
        }
    }
    (*valid_attempts)++;

    if (*local_best != 0xFFFF && path_len >= *local_best) {
        return *valid_attempts >= max_valid_attempts;
    }

    uint16_t old_global_best = solver->best_steps;
    if (*local_best != 0xFFFF) solver->best_steps = (uint16_t)(*local_best - path_len);
    Position new_player = solver->start_player;
    for (uint16_t step = 0; step < path_len; step++) {
        new_player.x += bomb_path[step].dx;
        new_player.y += bomb_path[step].dy;
    }

    if (*local_best != 0xFFFF) {
        uint16_t remain_lower = remaining_solution_lower_bound(solver, new_player);
        if (remain_lower == 0xFFFF || (uint32_t)path_len + remain_lower >= *local_best) {
            solver->best_steps = old_global_best;
            return *valid_attempts >= max_valid_attempts;
        }
    }

    BitboardMap* orig_map = &branch_scratch->orig_map;
    *orig_map = solver->bmap;
    uint32_t orig_mask = solver->destroyed_walls_mask;
    Position orig_player = solver->start_player;
    uint8_t orig_num_bombs = solver->num_bombs;
    uint16_t orig_path_len = solver->best_path_len;
    Entity* orig_bombs = branch_scratch->orig_bombs;
    memcpy(orig_bombs, solver->bombs, sizeof(solver->bombs));

    uint32_t next_hash = hash_move_player(current_hash, solver->start_player, new_player);
    next_hash ^= ZOBRIST_BOMB[Z_IDX(bomb.x, bomb.y)];
    clear_bit(solver->bmap.bombs, bomb.x, bomb.y);
    next_hash ^= clear_solver_explosion_walls(solver, target_wall);
    sokoban_refresh_dynamic_tunnels(&solver->bmap);

    solver->start_player = new_player;
    for (uint8_t k = b_idx; k < solver->num_bombs - 1; k++) solver->bombs[k] = solver->bombs[k + 1];
    solver->num_bombs--;

    bool quick_deadlock = has_any_deadlock(solver, &solver->bmap, NULL, NULL);

    uint16_t backup_macro = g_current_macro_depth;
    g_current_macro_depth = 0;

    bool saved_force_maneuver_rescue = g_force_maneuver_rescue;
    if (force_maneuver_rescue) g_force_maneuver_rescue = true;

    if (!quick_deadlock && *local_best != 0xFFFF) {
        uint16_t post_lower = remaining_solution_lower_bound(solver, new_player);
        if (post_lower == 0xFFFF || (uint32_t)path_len + post_lower >= *local_best) {
            g_force_maneuver_rescue = saved_force_maneuver_rescue;
            g_current_macro_depth = backup_macro;
            solver->best_steps = old_global_best;
            solver->destroyed_walls_mask = orig_mask;
            solver->bmap = *orig_map;
            solver->start_player = orig_player;
            solver->num_bombs = orig_num_bombs;
            solver->best_path_len = orig_path_len;
            memcpy(solver->bombs, orig_bombs, sizeof(solver->bombs));
            return *valid_attempts >= max_valid_attempts;
        }
    }

    bool recurse_ok = false;
    if (!quick_deadlock) {
        recurse_ok = sokoban_solve_internal(solver, depth + 1, next_hash);
    }
    if (recurse_ok) {
        uint16_t sub_len = solver->best_path_len;
        uint16_t total_len = (uint16_t)(path_len + sub_len);

        if (total_len < *local_best && total_len < MAX_PATH_LENGTH) {
            *local_best = total_len;
            memcpy(g_simple_path_pool[depth], bomb_path, path_len * sizeof(Direction));
            memcpy(&g_simple_path_pool[depth][path_len], solver->best_path, sub_len * sizeof(Direction));
            *found = true;
        }
    }

    g_force_maneuver_rescue = saved_force_maneuver_rescue;
    g_current_macro_depth = backup_macro;

    solver->best_steps = old_global_best;
    solver->destroyed_walls_mask = orig_mask;
    solver->bmap = *orig_map;
    solver->start_player = orig_player;
    solver->num_bombs = orig_num_bombs;
    solver->best_path_len = orig_path_len;
    memcpy(solver->bombs, orig_bombs, sizeof(solver->bombs));

    if (g_bomb_seed_first_solution_only && *found) {
        return true;
    }
    return *valid_attempts >= max_valid_attempts;
}
static bool __attribute__((noinline)) try_bomb_candidate_with_light_evac(
    SokobanSolver* solver,
    int depth,
    uint32_t current_hash,
    const BitboardMap* bomb_route_base_map,
    const BombWallCandidate* cand,
    Direction* bomb_path,
    uint16_t assignment_lower,
    uint16_t* local_best,
    bool* found,
    int* valid_attempts,
    int max_valid_attempts,
    bool force_maneuver_rescue,
    int candidate_rank,
    int num_candidates
) {
    if (!solver || !bomb_route_base_map || !cand || !bomb_path || !local_best || !found || !valid_attempts) return false;

    int trigger_window = (num_candidates < 5) ? num_candidates : 5;
    bool in_trigger_window = candidate_rank >= 0 && candidate_rank < trigger_window;
    BombLightScratch* light_scratch = (depth >= 0 && depth < MAX_BOMBS) ? &g_bomb_light_scratch[depth] : NULL;
    bool has_local_blocker = false;
    if (light_scratch && cand->route_len == 0xFFFF) {
        uint8_t* blocker_indices = light_scratch->blocker_indices;
        has_local_blocker = light_evac_collect_blocking_boxes(solver, depth, cand, blocker_indices) > 0;
    }
    bool should_try_light =
        depth >= 0 && depth < MAX_BOMBS &&
        cand->route_len == 0xFFFF &&
        light_evac_context_allowed(solver) &&
        (in_trigger_window && has_local_blocker) &&
        g_light_evac_recursion_guard < MAX_LIGHT_EVAC_DEPTH;
    if (!should_try_light) {
        return false;
    }

    BitboardMap* snapshot_map = &light_scratch->snapshot_map;
    *snapshot_map = solver->bmap;
    Entity* snapshot_boxes = light_scratch->snapshot_boxes;
    Entity* snapshot_bombs = light_scratch->snapshot_bombs;
    Position snapshot_player = solver->start_player;
    uint8_t snapshot_num_boxes = solver->num_boxes;
    uint8_t snapshot_num_bombs = solver->num_bombs;
    uint32_t snapshot_destroyed = solver->destroyed_walls_mask;
    uint16_t snapshot_best_steps = solver->best_steps;
    uint16_t snapshot_best_path_len = solver->best_path_len;
    Direction* saved_best_path = g_light_evac_saved_best_path;
    Direction* saved_simple_path = g_light_evac_saved_simple_path;
    memcpy(snapshot_boxes, solver->boxes, sizeof(solver->boxes));
    memcpy(snapshot_bombs, solver->bombs, sizeof(solver->bombs));
    if (solver->best_path) {
        memcpy(saved_best_path, solver->best_path, MAX_PATH_LENGTH * sizeof(Direction));
    }
    memcpy(saved_simple_path, g_simple_path_pool[depth], MAX_PATH_LENGTH * sizeof(Direction));

#define LIGHT_EVAC_RESTORE_ENTRY() \
    light_evac_restore_entry_state( \
        solver, snapshot_map, snapshot_boxes, snapshot_bombs, snapshot_player, \
        snapshot_num_boxes, snapshot_num_bombs, snapshot_destroyed, \
        snapshot_best_steps, snapshot_best_path_len, saved_best_path)

    if (*found && *local_best != 0xFFFF) {
        memcpy(saved_simple_path, g_simple_path_pool[depth], MAX_PATH_LENGTH * sizeof(Direction));
    }
    LIGHT_EVAC_RESTORE_ENTRY();
    LightEvacPlan* plans = g_light_evac_plan_pool;
    int num_plans = build_light_evac_plans(solver, depth, cand, plans);
    if (num_plans <= 0) {
        LIGHT_EVAC_RESTORE_ENTRY();
        return false;
    }

    bool stop = false;
    uint16_t best_limit = *local_best;
    g_light_evac_recursion_guard++;
    for (int pi = 0; pi < num_plans; pi++) {
        const LightEvacPlan* plan = &plans[pi];
        LIGHT_EVAC_RESTORE_ENTRY();
        memcpy(g_simple_path_pool[depth], saved_simple_path, MAX_PATH_LENGTH * sizeof(Direction));

        if (best_limit != 0xFFFF && plan->evac_len >= best_limit) continue;
        if (!light_evac_apply_plan(solver, plan)) continue;

        uint32_t evac_hash = hash_move_player(current_hash, snapshot_player, plan->player_after);
        evac_hash = hash_move_box(evac_hash, plan->old_box, plan->target_parking, false);

        BitboardMap* evac_route_map = &light_scratch->evac_route_map;
        build_bomb_route_base_map(solver, evac_route_map);

        BombWallCandidate* moved_cand = &light_scratch->moved_cand;
        *moved_cand = *cand;
        bomb_candidate_set_route_precomputed(moved_cand, false);
        moved_cand->route_len = 0xFFFF;
        moved_cand->route_slot = 0;
        moved_cand->path_lower_bound = 0xFFFF;

        if (best_limit != 0xFFFF && best_limit <= plan->evac_len) continue;
        uint16_t branch_best = (best_limit == 0xFFFF)
            ? 0xFFFF
            : (uint16_t)(best_limit - plan->evac_len);
        bool branch_found = false;
        int branch_valid_attempts = *valid_attempts;

        bool branch_stop = try_bomb_candidate_branch(
            solver, depth, evac_hash, evac_route_map, moved_cand, bomb_path, assignment_lower,
            &branch_best, &branch_found, &branch_valid_attempts, max_valid_attempts, force_maneuver_rescue
        );
        *valid_attempts = branch_valid_attempts;

        if (branch_found && branch_best != 0xFFFF) {
            uint32_t total_len_u32 = (uint32_t)plan->evac_len + (uint32_t)branch_best;
            if (total_len_u32 < MAX_PATH_LENGTH && total_len_u32 < best_limit) {
                uint16_t branch_len = branch_best;
                uint16_t total_len = (uint16_t)total_len_u32;
                memmove(&g_simple_path_pool[depth][plan->evac_len],
                        g_simple_path_pool[depth],
                        branch_len * sizeof(Direction));
                memcpy(g_simple_path_pool[depth], plan->evac_path, plan->evac_len * sizeof(Direction));

                LIGHT_EVAC_RESTORE_ENTRY();
                if (sq_verify_full_path(
                        solver,
                        snapshot_map,
                        snapshot_player,
                        snapshot_boxes,
                        snapshot_num_boxes,
                        snapshot_bombs,
                        snapshot_num_bombs,
                        g_simple_path_pool[depth],
                        total_len)) {
                    *local_best = total_len;
                    *found = true;
                    best_limit = total_len;
                    memcpy(saved_simple_path, g_simple_path_pool[depth], MAX_PATH_LENGTH * sizeof(Direction));
                } else {
                    memcpy(g_simple_path_pool[depth], saved_simple_path, MAX_PATH_LENGTH * sizeof(Direction));
                }
            }
        }

        if (branch_stop) {
            stop = true;
            break;
        }
    }
    g_light_evac_recursion_guard--;

    LIGHT_EVAC_RESTORE_ENTRY();
    memcpy(g_simple_path_pool[depth], saved_simple_path, MAX_PATH_LENGTH * sizeof(Direction));
#undef LIGHT_EVAC_RESTORE_ENTRY
    return stop;
}
static inline int bomb_candidate_topology_cell_bonus(int (*bonus)[MAP_COLS], int y, int x) { return bonus[y][x]; }
static inline int bomb_candidate_shortcut_cell_bonus(int (*bonus)[MAP_COLS], int y, int x) { return bonus[y][x]; }

static bool __attribute__((noinline)) build_bomb_strategy_candidates(SokobanSolver* solver, int depth, BombStrategyBuildResult* out) {
    if (!solver || !out || depth < 0 || depth >= MAX_BOMBS || solver->num_bombs == 0) return false;

    BombStrategyScratch* strategy_scratch = &g_bomb_strategy_scratch[depth];
    Position* target_walls = g_target_walls_pool;
    int num_walls = 0;

    // ========================================================================
    // ========================================================================
    int (*ghost_topology_bonus)[MAP_COLS] = g_bomb_ghost_topology_bonus;
    memset(ghost_topology_bonus, 0, sizeof(g_bomb_ghost_topology_bonus));
    int (*ghost_shortcut_bonus)[MAP_COLS] = g_bomb_ghost_shortcut_bonus;
    memset(ghost_shortcut_bonus, 0, sizeof(g_bomb_ghost_shortcut_bonus));

    uint16_t* global_dynamic_obs = strategy_scratch->global_dynamic_obs;
    for (int r = 0; r < MAP_ROWS; r++) {
        global_dynamic_obs[r] = (uint16_t)(solver->bmap.boxes[r] | solver->bmap.bombs[r]);
    }

    uint16_t (*dist_P)[MAP_COLS] = g_bomb_dist_p;
    memset(dist_P, 0xFF, sizeof(g_bomb_dist_p));

    Position* q_P = g_bfs_queue;
    int head_P = 0;
    int tail_P = 0;
    q_P[tail_P++] = solver->start_player;
    dist_P[solver->start_player.y][solver->start_player.x] = 0;

    while (head_P < tail_P) {
        Position curr = q_P[head_P++];
        int d_curr = dist_P[curr.y][curr.x];

        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
            if (dist_P[ny][nx] != 0xFFFF) continue;

            uint16_t bit = (uint16_t)(1u << nx);
            bool is_hard_wall = ((solver->bmap.walls[ny] & bit) != 0 && (g_destructible_mask[ny] & bit) == 0);
            if (is_hard_wall) continue;
            if ((global_dynamic_obs[ny] & bit) != 0) continue;
            if ((solver->bmap.walls[ny] & bit) != 0) continue;

            dist_P[ny][nx] = (uint16_t)(d_curr + 1);
            q_P[tail_P++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }

    bool* ghost_target_filled = strategy_scratch->ghost_target_filled;
    int* ghost_target_deadlocks = strategy_scratch->ghost_target_deadlocks;
    int* ghost_target_weight = strategy_scratch->ghost_target_weight;
    memset(ghost_target_filled, 0, MAX_TARGETS * sizeof(ghost_target_filled[0]));
    memset(ghost_target_deadlocks, 0, MAX_TARGETS * sizeof(ghost_target_deadlocks[0]));
    memset(ghost_target_weight, 0, MAX_TARGETS * sizeof(ghost_target_weight[0]));

    for (int t = 0; t < solver->num_targets; t++) {
        Position target = solver->targets[t].pos;

        for (int j = 0; j < solver->num_boxes; j++) {
            if (pos_equal(solver->boxes[j].pos, target)) {
                ghost_target_filled[t] = true;
                break;
            }
        }

        int target_deadlocks = 0;
        for (int d = 0; d < 4; d++) {
            int nx = target.x + DIRECTIONS[d].dx;
            int ny = target.y + DIRECTIONS[d].dy;
            if (nx > 0 && nx < MAP_COLS - 1 && ny > 0 && ny < MAP_ROWS - 1) {
                uint16_t bit = (uint16_t)(1u << nx);
                if ((solver->bmap.walls[ny] & bit) != 0 && (g_destructible_mask[ny] & bit) == 0) {
                    target_deadlocks++;
                } else if ((solver->bmap.deadlocks[ny] & bit) != 0) {
                    target_deadlocks++;
                }
            } else {
                target_deadlocks++;
            }
        }

        ghost_target_deadlocks[t] = target_deadlocks;
        ghost_target_weight[t] = 1500 - (target_deadlocks * 350);
        if (ghost_target_weight[t] < 100) ghost_target_weight[t] = 100;
    }

    for (int i = 0; i < solver->num_boxes; i++) {
        Position box = solver->boxes[i].pos;
        if ((solver->bmap.targets[box.y] & (1u << box.x)) != 0) continue;

        bool has_eligible_target = false;
        for (int t = 0; t < solver->num_targets; t++) {
            if (ghost_target_filled[t]) continue;
            if (solver->strict_target_mode && solver->targets[t].id != -1 && solver->boxes[i].id != -1) {
                if (solver->targets[t].id != solver->boxes[i].id) continue;
            }
            has_eligible_target = true;
            break;
        }
        if (!has_eligible_target) continue;

        uint16_t* dynamic_obs = strategy_scratch->dynamic_obs;
        for (int r = 0; r < MAP_ROWS; r++) {
            dynamic_obs[r] = global_dynamic_obs[r];
        }
        dynamic_obs[box.y] = (uint16_t)(dynamic_obs[box.y] & (uint16_t)~(1u << box.x));

        uint16_t (*dist_B)[MAP_COLS] = g_bomb_dist_b;
        memset(dist_B, 0xFF, sizeof(g_bomb_dist_b));

        Position* q = g_bfs_queue;
        int head = 0;
        int tail = 0;

        q[tail++] = box;
        dist_B[box.y][box.x] = 0;

        while (head < tail) {
            Position curr = q[head++];
            int d_curr = dist_B[curr.y][curr.x];

            for (int d = 0; d < 4; d++) {
                int nx = curr.x + DIRECTIONS[d].dx;
                int ny = curr.y + DIRECTIONS[d].dy;
                int px = curr.x - DIRECTIONS[d].dx;
                int py = curr.y - DIRECTIONS[d].dy;

                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                if (px <= 0 || px >= MAP_COLS - 1 || py <= 0 || py >= MAP_ROWS - 1) continue;

                if (dist_B[ny][nx] == 0xFFFF) {
                    uint16_t push_bit = (uint16_t)(1u << px);
                    if ((solver->bmap.walls[py] & push_bit) != 0 && (g_destructible_mask[py] & push_bit) == 0) continue;
                    if ((dynamic_obs[py] & push_bit) != 0) continue;

                    uint16_t next_bit = (uint16_t)(1u << nx);
                    bool is_hard_wall = ((solver->bmap.walls[ny] & next_bit) != 0 && (g_destructible_mask[ny] & next_bit) == 0);
                    if (is_hard_wall) continue;

                    dist_B[ny][nx] = (uint16_t)(d_curr + 1);

                    if ((solver->bmap.walls[ny] & next_bit) != 0) continue;
                    if ((dynamic_obs[ny] & next_bit) != 0) continue;

                    q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
                }
            }
        }

        for (int t = 0; t < solver->num_targets; t++) {
            Position target = solver->targets[t].pos;

            if (ghost_target_filled[t]) continue;

            if (solver->strict_target_mode && solver->targets[t].id != -1 && solver->boxes[i].id != -1) {
                if (solver->targets[t].id != solver->boxes[i].id) continue;
            }

            int target_deadlocks = ghost_target_deadlocks[t];
            int ghost_weight = ghost_target_weight[t];

            uint16_t (*dist_T)[MAP_COLS] = g_bomb_dist_t;
            memset(dist_T, 0xFF, sizeof(g_bomb_dist_t));

            head = 0;
            tail = 0;
            q[tail++] = target;
            dist_T[target.y][target.x] = 0;

            while (head < tail) {
                Position curr = q[head++];
                int d_curr = dist_T[curr.y][curr.x];

                for (int d = 0; d < 4; d++) {
                    int nx = curr.x + DIRECTIONS[d].dx;
                    int ny = curr.y + DIRECTIONS[d].dy;

                    if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;

                    if (dist_T[ny][nx] == 0xFFFF) {
                        uint16_t next_bit = (uint16_t)(1u << nx);
                        bool is_hard_wall = ((solver->bmap.walls[ny] & next_bit) != 0 && (g_destructible_mask[ny] & next_bit) == 0);
                        if (is_hard_wall) continue;

                        dist_T[ny][nx] = (uint16_t)(d_curr + 1);

                        if ((solver->bmap.walls[ny] & next_bit) != 0) continue;
                        if ((dynamic_obs[ny] & next_bit) != 0) continue;

                        q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
                    }
                }
            }

            int normal_dist = dist_B[target.y][target.x];

            for (int wy = 1; wy < MAP_ROWS - 1; wy++) {
                for (int wx = 1; wx < MAP_COLS - 1; wx++) {
                    uint16_t wall_bit = (uint16_t)(1u << wx);
                    if ((solver->bmap.walls[wy] & wall_bit) != 0 && (g_destructible_mask[wy] & wall_bit) != 0) {
                        if (dist_B[wy][wx] != 0xFFFF && dist_T[wy][wx] != 0xFFFF) {
                            int shortcut_dist = (int)dist_B[wy][wx] + (int)dist_T[wy][wx];

                            if (normal_dist == 0xFFFF) {
                                int top_dist = shortcut_dist;
                                if (top_dist > 60) top_dist = 60;

                                int raw_top = 6500 - (top_dist * 40);
                                if (raw_top < 3500) raw_top = 3500;

                                ghost_topology_bonus[wy][wx] += raw_top;
                                if (ghost_topology_bonus[wy][wx] > 8000) {
                                    ghost_topology_bonus[wy][wx] = 8000;
                                }
                            } else if (shortcut_dist < normal_dist) {
                                bool is_adjacent_to_target = (abs(wx - target.x) + abs(wy - target.y) == 1);
                                if (target_deadlocks == 0 && is_adjacent_to_target) continue;

                                int saved_steps = normal_dist - shortcut_dist;
                                if (saved_steps > 6) {
                                    int raw_short = (saved_steps * ghost_weight) / 10;
                                    if (raw_short > 4500) raw_short = 4500;

                                    ghost_shortcut_bonus[wy][wx] += raw_short;
                                    if (ghost_shortcut_bonus[wy][wx] > 6000) {
                                        ghost_shortcut_bonus[wy][wx] = 6000;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // ========================================================================
    #define ADD_TARGET_WALL(w) do { \
        bool dup = false; \
        for(int k=0; k<num_walls; k++) { if(pos_equal(target_walls[k], (w))) {dup=true; break;} } \
        if(!dup && num_walls < MAX_BOMB_OPTIONS) { target_walls[num_walls++] = (w); } \
    } while(0)

    uint16_t* box_puddle_walls = g_bomb_box_puddle_walls;
    uint16_t* is_puddle_core_wall = g_bomb_puddle_core_walls;
    memset(box_puddle_walls, 0, sizeof(g_bomb_box_puddle_walls));
    memset(is_puddle_core_wall, 0, sizeof(g_bomb_puddle_core_walls));

    g_bfs_run_epoch++;
    if (g_bfs_run_epoch == 0) {
        memset(g_bfs_visited, 0, sizeof(g_bfs_visited));
        g_bfs_run_epoch = 1;
    }
    int head = 0, tail = 0;

    g_bfs_queue[tail++] = solver->start_player;
    g_bfs_visited[solver->start_player.y][solver->start_player.x] = g_bfs_run_epoch;

    for (int i = 0; i < solver->num_boxes; i++) {
        Position bp = solver->boxes[i].pos;
        if ((solver->bmap.targets[bp.y] & (1 << bp.x)) == 0) {
            if (g_bfs_visited[bp.y][bp.x] != g_bfs_run_epoch) {
                g_bfs_queue[tail++] = bp;
                g_bfs_visited[bp.y][bp.x] = g_bfs_run_epoch;
            }
        }
    }

    uint16_t* unified_obs = strategy_scratch->unified_obs;
    for (int r = 0; r < MAP_ROWS; r++) {
        unified_obs[r] = map_blocked_row(&solver->bmap, r);
    }

    while (head < tail) {
        Position curr = g_bfs_queue[head++];
        for (int d = 0; d < 4; d++) {
            int nx = curr.x + DIRECTIONS[d].dx;
            int ny = curr.y + DIRECTIONS[d].dy;
            if (nx > 0 && nx < MAP_COLS - 1 && ny > 0 && ny < MAP_ROWS - 1) {
                if ((solver->bmap.walls[ny] & (1 << nx)) != 0 && (g_destructible_mask[ny] & (1 << nx)) != 0) {
                    box_puddle_walls[ny] |= bit_mask_at(nx);
                } else if (g_bfs_visited[ny][nx] != g_bfs_run_epoch && (unified_obs[ny] & (1 << nx)) == 0) {
                    g_bfs_visited[ny][nx] = g_bfs_run_epoch;
                    g_bfs_queue[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
                }
            }
        }
    }

    for (int b = 0; b < solver->num_bombs; b++) {
        g_bfs_run_epoch++;
        if (g_bfs_run_epoch == 0) {
            memset(g_bfs_visited, 0, sizeof(g_bfs_visited));
            g_bfs_run_epoch = 1;
        }
        head = 0;
        tail = 0;

        Position bomb_pos = solver->bombs[b].pos;
        g_bfs_queue[tail++] = bomb_pos;
        g_bfs_visited[bomb_pos.y][bomb_pos.x] = g_bfs_run_epoch;

        while (head < tail) {
            Position curr = g_bfs_queue[head++];
            for (int d = 0; d < 4; d++) {
                int nx = curr.x + DIRECTIONS[d].dx;
                int ny = curr.y + DIRECTIONS[d].dy;
                if (nx > 0 && nx < MAP_COLS - 1 && ny > 0 && ny < MAP_ROWS - 1) {
                    if ((solver->bmap.walls[ny] & (1 << nx)) != 0 && (g_destructible_mask[ny] & (1 << nx)) != 0) {
                        ADD_TARGET_WALL(((Position){(uint8_t)nx, (uint8_t)ny}));
                        if ((box_puddle_walls[ny] & bit_mask_at(nx)) != 0) {
                            is_puddle_core_wall[ny] |= bit_mask_at(nx);
                        }
                    } else if (g_bfs_visited[ny][nx] != g_bfs_run_epoch && (unified_obs[ny] & (1 << nx)) == 0) {
                        g_bfs_visited[ny][nx] = g_bfs_run_epoch;
                        g_bfs_queue[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
                    }
                }
            }
        }
    }
    for (int b = 0; b < solver->num_bombs; b++) {
        Position p = solver->bombs[b].pos;

        if (p.y == 1 || p.y == MAP_ROWS - 2) {
            for (int cx = p.x - 1; cx > 0; cx--) {
                if ((solver->bmap.boxes[p.y] & (1 << cx)) != 0 || (solver->bmap.bombs[p.y] & (1 << cx)) != 0) break;
                if ((solver->bmap.walls[p.y] & (1 << cx)) != 0) {
                    if ((g_destructible_mask[p.y] & (1 << cx)) != 0) ADD_TARGET_WALL(((Position){(uint8_t)cx, p.y}));
                    break;
                }
            }
            for (int cx = p.x + 1; cx < MAP_COLS - 1; cx++) {
                if ((solver->bmap.boxes[p.y] & (1 << cx)) != 0 || (solver->bmap.bombs[p.y] & (1 << cx)) != 0) break;
                if ((solver->bmap.walls[p.y] & (1 << cx)) != 0) {
                    if ((g_destructible_mask[p.y] & (1 << cx)) != 0) ADD_TARGET_WALL(((Position){(uint8_t)cx, p.y}));
                    break;
                }
            }
        }

        if (p.x == 1 || p.x == MAP_COLS - 2) {
            for (int cy = p.y - 1; cy > 0; cy--) {
                if ((solver->bmap.boxes[cy] & (1 << p.x)) != 0 || (solver->bmap.bombs[cy] & (1 << p.x)) != 0) break;
                if ((solver->bmap.walls[cy] & (1 << p.x)) != 0) {
                    if ((g_destructible_mask[cy] & (1 << p.x)) != 0) ADD_TARGET_WALL(((Position){p.x, (uint8_t)cy}));
                    break;
                }
            }
            for (int cy = p.y + 1; cy < MAP_ROWS - 1; cy++) {
                if ((solver->bmap.boxes[cy] & (1 << p.x)) != 0 || (solver->bmap.bombs[cy] & (1 << p.x)) != 0) break;
                if ((solver->bmap.walls[cy] & (1 << p.x)) != 0) {
                    if ((g_destructible_mask[cy] & (1 << p.x)) != 0) ADD_TARGET_WALL(((Position){p.x, (uint8_t)cy}));
                    break;
                }
            }
        }
    }

    {
        uint16_t* entity_layer = strategy_scratch->entity_layer;
        for (int r = 0; r < MAP_ROWS; r++) {
            entity_layer[r] = solver->bmap.boxes[r] | solver->bmap.bombs[r];
        }

        Position* entities_to_check = strategy_scratch->entities_to_check;
        int num_entities = 0;
        for (int i = 0; i < solver->num_boxes; i++) entities_to_check[num_entities++] = solver->boxes[i].pos;
        for (int i = 0; i < solver->num_bombs; i++) entities_to_check[num_entities++] = solver->bombs[i].pos;

        for (int i = 0; i < num_entities; i++) {
            Position p = entities_to_check[i];

            bool wall_l = (solver->bmap.walls[p.y] & (1 << (p.x - 1))) != 0;
            bool wall_r = (solver->bmap.walls[p.y] & (1 << (p.x + 1))) != 0;
            bool wall_u = (solver->bmap.walls[p.y - 1] & (1 << p.x)) != 0;
            bool wall_d = (solver->bmap.walls[p.y + 1] & (1 << p.x)) != 0;

            bool block_l = wall_l || ((entity_layer[p.y] & (1 << (p.x - 1))) != 0);
            bool block_r = wall_r || ((entity_layer[p.y] & (1 << (p.x + 1))) != 0);
            bool block_u = wall_u || ((entity_layer[p.y - 1] & (1 << p.x)) != 0);
            bool block_d = wall_d || ((entity_layer[p.y + 1] & (1 << p.x)) != 0);

            if (wall_l && wall_r && (block_u || block_d)) {
                ADD_TARGET_WALL(((Position){p.x - 1, p.y}));
                ADD_TARGET_WALL(((Position){p.x + 1, p.y}));
            }
            if (wall_u && wall_d && (block_l || block_r)) {
                ADD_TARGET_WALL(((Position){p.x, p.y - 1}));
                ADD_TARGET_WALL(((Position){p.x, p.y + 1}));
            }
        }
    }

    for (int i = 0; i < solver->num_bombs; i++) {
        Position p = solver->bombs[i].pos;
        if (p.x == 1) {
            for (int y = p.y + 1; y < MAP_ROWS - 1; y++) {
                if ((solver->bmap.walls[y] & (1 << 1)) != 0) { ADD_TARGET_WALL(((Position){1, y})); break; }
                if (((solver->bmap.boxes[y] | solver->bmap.bombs[y]) & (1 << 1)) != 0) break;
            }
            for (int y = p.y - 1; y > 0; y--) {
                if ((solver->bmap.walls[y] & (1 << 1)) != 0) { ADD_TARGET_WALL(((Position){1, y})); break; }
                if (((solver->bmap.boxes[y] | solver->bmap.bombs[y]) & (1 << 1)) != 0) break;
            }
            if ((solver->bmap.walls[p.y] & (1 << 2)) != 0) ADD_TARGET_WALL(((Position){2, p.y}));
        }
        if (p.x == MAP_COLS - 2) {
            for (int y = p.y + 1; y < MAP_ROWS - 1; y++) {
                if ((solver->bmap.walls[y] & (1 << (MAP_COLS - 2))) != 0) { ADD_TARGET_WALL(((Position){MAP_COLS - 2, y})); break; }
                if (((solver->bmap.boxes[y] | solver->bmap.bombs[y]) & (1 << (MAP_COLS - 2))) != 0) break;
            }
            for (int y = p.y - 1; y > 0; y--) {
                if ((solver->bmap.walls[y] & (1 << (MAP_COLS - 2))) != 0) { ADD_TARGET_WALL(((Position){MAP_COLS - 2, y})); break; }
                if (((solver->bmap.boxes[y] | solver->bmap.bombs[y]) & (1 << (MAP_COLS - 2))) != 0) break;
            }
            if ((solver->bmap.walls[p.y] & (1 << (MAP_COLS - 3))) != 0) ADD_TARGET_WALL(((Position){MAP_COLS - 3, p.y}));
        }
        if (p.y == 1 || p.y == MAP_ROWS - 2) {
            int y = p.y;
            uint16_t all_obs = map_blocked_row(&solver->bmap, y);

            uint16_t obs_r = all_obs & g_O1_ray_right[p.x];
            if (obs_r) {
                int hit_x = __builtin_ctz(obs_r);
                if ((solver->bmap.walls[y] & (1 << hit_x)) != 0) ADD_TARGET_WALL(((Position){(uint8_t)hit_x, (uint8_t)y}));
            }

            uint16_t obs_l = all_obs & g_O1_ray_left[p.x];
            if (obs_l) {
                int hit_x = 31 - __builtin_clz(obs_l);
                if ((solver->bmap.walls[y] & (1 << hit_x)) != 0) ADD_TARGET_WALL(((Position){(uint8_t)hit_x, (uint8_t)y}));
            }

            if (y == 1 && (solver->bmap.walls[2] & (1 << p.x)) != 0) ADD_TARGET_WALL(((Position){p.x, 2}));
            if (y == MAP_ROWS - 2 && (solver->bmap.walls[MAP_ROWS - 3] & (1 << p.x)) != 0) ADD_TARGET_WALL(((Position){p.x, MAP_ROWS - 3}));
        }
    }

    if (num_walls == 0) return false;

    uint16_t* is_candidate_wall = g_bomb_candidate_wall_mask;
    memset(is_candidate_wall, 0, sizeof(g_bomb_candidate_wall_mask));
    for (int w = 0; w < num_walls; w++) {
        Position tw = target_walls[w];
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int nx = tw.x + dx, ny = tw.y + dy;
                if (nx > 0 && nx < MAP_COLS - 1 && ny > 0 && ny < MAP_ROWS - 1) {
                    if ((solver->bmap.walls[ny] & (1 << nx)) != 0) {
                        is_candidate_wall[ny] |= bit_mask_at(nx);
                    }
                }
            }
        }
    }

    Position* candidate_wall_cells = g_bomb_candidate_wall_cells;
    int num_candidate_wall_cells = 0;
    for (int y = 1; y < MAP_ROWS - 1; y++) {
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if ((is_candidate_wall[y] & bit_mask_at(x)) != 0 && (solver->bmap.walls[y] & (1 << x)) != 0) {
                candidate_wall_cells[num_candidate_wall_cells++] = (Position){(uint8_t)x, (uint8_t)y};
            }
        }
    }
    if (num_candidate_wall_cells == 0) return false;

    BombWallCandidate* top_candidates = g_bomb_top_candidates[depth];
    int num_candidates = 0;
    uint16_t min_candidate_path_lower = 0xFFFF;



    for (uint8_t b_idx = 0; b_idx < solver->num_bombs; b_idx++) {
        Position bomb = solver->bombs[b_idx].pos;
        BitboardMap* temp_map = &g_bomb_temp_map;
        *temp_map = solver->bmap;
        clear_bit(temp_map->bombs, bomb.x, bomb.y);
        memset(temp_map->boxes, 0, sizeof(temp_map->boxes));
        for (int k = 0; k < solver->num_boxes; k++) {
            if ((solver->bmap.targets[solver->boxes[k].pos.y] & (1 << solver->boxes[k].pos.x)) == 0) {
                set_box_bit(temp_map, solver->boxes[k].pos.x, solver->boxes[k].pos.y);
            }
        }

        uint16_t* all_obs = strategy_scratch->all_obs;
        for (int r = 0; r < MAP_ROWS; r++) {
            all_obs[r] = map_blocked_row(temp_map, r);
        }
        uint16_t (*player_nav_dist)[MAP_COLS] = g_bomb_player_nav_dist;
        memset(player_nav_dist, 0xFF, sizeof(g_bomb_player_nav_dist));
        uint16_t* player_obs = strategy_scratch->player_obs;
        for (int r = 0; r < MAP_ROWS; r++) {
            player_obs[r] = all_obs[r];
        }
        player_obs[bomb.y] |= (uint16_t)(1u << bomb.x);

        g_bfs_run_epoch++;
        int nav_head = 0;
        int nav_tail = 0;
        g_bfs_queue[nav_tail++] = solver->start_player;
        g_bfs_visited[solver->start_player.y][solver->start_player.x] = g_bfs_run_epoch;
        player_nav_dist[solver->start_player.y][solver->start_player.x] = 0;

        while (nav_head < nav_tail) {
            Position curr = g_bfs_queue[nav_head++];
            uint16_t cur_d = player_nav_dist[curr.y][curr.x];

            for (int d = 0; d < 4; d++) {
                int nx = curr.x + DIRECTIONS[d].dx;
                int ny = curr.y + DIRECTIONS[d].dy;
                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                if (g_bfs_visited[ny][nx] == g_bfs_run_epoch) continue;
                if ((player_obs[ny] & (1u << nx)) != 0) continue;

                g_bfs_visited[ny][nx] = g_bfs_run_epoch;
                player_nav_dist[ny][nx] = (uint16_t)(cur_d + 1);
                g_bfs_queue[nav_tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
            }
        }

        int player_to_bomb_route = 0xFFFF;
        for (int d = 0; d < 4; d++) {
            int sx = bomb.x + DIRECTIONS[d].dx;
            int sy = bomb.y + DIRECTIONS[d].dy;
            if (sx <= 0 || sx >= MAP_COLS - 1 || sy <= 0 || sy >= MAP_ROWS - 1) continue;
            if (player_nav_dist[sy][sx] < player_to_bomb_route) {
                player_to_bomb_route = player_nav_dist[sy][sx];
            }
        }
        if (player_to_bomb_route == 0xFFFF) {
            continue;
        }
        uint16_t (*bfs_dist)[MAP_COLS] = g_bomb_bfs_dist;
        memset(bfs_dist, 0xFF, sizeof(g_bomb_bfs_dist));

        g_bfs_run_epoch++;
        int head = 0, tail = 0;
        g_bfs_queue[tail++] = bomb;
        g_bfs_visited[bomb.y][bomb.x] = g_bfs_run_epoch;
        bfs_dist[bomb.y][bomb.x] = 0;

        while (head < tail) {
            Position curr = g_bfs_queue[head++];
            uint16_t cur_d = bfs_dist[curr.y][curr.x];
            for (int d = 0; d < 4; d++) {
                int nx = curr.x + DIRECTIONS[d].dx, ny = curr.y + DIRECTIONS[d].dy;
                if (nx > 0 && nx < MAP_COLS - 1 && ny > 0 && ny < MAP_ROWS - 1) {
                    if (g_bfs_visited[ny][nx] != g_bfs_run_epoch && (all_obs[ny] & (1 << nx)) == 0) {
                        g_bfs_visited[ny][nx] = g_bfs_run_epoch;
                        bfs_dist[ny][nx] = (uint16_t)(cur_d + 1);
                        g_bfs_queue[tail++] = (Position){nx, ny};
                    }
                }
            }
        }
        for (int cw = 0; cw < num_candidate_wall_cells; cw++) {
                int x = candidate_wall_cells[cw].x;
                int y = candidate_wall_cells[cw].y;

                int min_push_dist = 0xFFFF;
                for (int d = 0; d < 4; d++) {
                    int nx = x + DIRECTIONS[d].dx, ny = y + DIRECTIONS[d].dy;
                    if (nx > 0 && nx < MAP_COLS - 1 && ny > 0 && ny < MAP_ROWS - 1) {
                        if (g_bfs_visited[ny][nx] == g_bfs_run_epoch && bfs_dist[ny][nx] < min_push_dist) {
                            min_push_dist = bfs_dist[ny][nx];
                        }
                    }
                }
                if (min_push_dist == 0xFFFF) continue;

                int score = 0;
                bool opens_new_area = false;
                bool hit_puddle_core = false;
                int thin_wall_bonus = 0;
                int target_bonus = 0;
                int box_bonus = 0;

                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nx = x + dx, ny = y + dy;
                        if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;

                        int wall_count = 0;
                        if ((solver->bmap.walls[ny] & (1 << (nx - 1))) != 0) wall_count++;
                        if ((solver->bmap.walls[ny] & (1 << (nx + 1))) != 0) wall_count++;
                        if ((solver->bmap.walls[ny - 1] & (1 << nx)) != 0) wall_count++;
                        if ((solver->bmap.walls[ny + 1] & (1 << nx)) != 0) wall_count++;

                        bool is_direct_adjacent = (abs(nx - x) + abs(ny - y) == 1);

                        if ((solver->bmap.targets[ny] & (1 << nx)) != 0) {
                            target_bonus += 400;
                            if (is_direct_adjacent) {
                                if (wall_count >= 3) {
                                    target_bonus += 12000;
                                } else if (wall_count == 2) {
                                    target_bonus += 8000;
                                }
                            }
                        }

                        if ((solver->bmap.boxes[ny] & (1 << nx)) != 0) {
                            box_bonus += 300;
                            if (is_direct_adjacent) {
                                if (wall_count >= 3) {
                                    box_bonus += 12000;
                                } else if (wall_count == 2) {
                                    box_bonus += 6000;
                                }
                            }
                        }

                        if ((solver->bmap.walls[ny] & (1 << nx)) != 0 &&
                            (g_destructible_mask[ny] & (1 << nx)) != 0) {
                            if ((is_puddle_core_wall[ny] & bit_mask_at(nx)) != 0) hit_puddle_core = true;

                            for (int d = 0; d < 4; d++) {
                                int nnx = nx + DIRECTIONS[d].dx, nny = ny + DIRECTIONS[d].dy;
                                if ((all_obs[nny] & (1 << nnx)) == 0 &&
                                    g_bfs_visited[nny][nnx] != g_bfs_run_epoch) {
                                    opens_new_area = true;
                                }
                            }

                            bool left_open  = ((all_obs[ny] & (1 << (nx - 1))) == 0);
                            bool right_open = ((all_obs[ny] & (1 << (nx + 1))) == 0);
                            bool up_open    = ((all_obs[ny - 1] & (1 << nx)) == 0);
                            bool down_open  = ((all_obs[ny + 1] & (1 << nx)) == 0);
                            if ((left_open && right_open) || (up_open && down_open)) {
                                thin_wall_bonus += 1000;
                            }
                        }
                    }
                }

                if (opens_new_area) score += 6000;
                if (hit_puddle_core) score += 2000;
                score += thin_wall_bonus;
                score += target_bonus;
                score += box_bonus;

                if (bomb.x == x || bomb.y == y) score += 300;

                int top_area_max = 0;
                int short_area_max = 0;
                int component_bridge_bonus = 0;
                for (int gy = y - 1; gy <= y + 1; gy++) {
                    for (int gx = x - 1; gx <= x + 1; gx++) {
                        if (gx > 0 && gx < MAP_COLS - 1 && gy > 0 && gy < MAP_ROWS - 1) {
                            if (ghost_topology_bonus[gy][gx] > top_area_max) {
                                top_area_max = ghost_topology_bonus[gy][gx];
                            }
                            if (ghost_shortcut_bonus[gy][gx] > short_area_max) {
                                short_area_max = ghost_shortcut_bonus[gy][gx];
                            }
                        }
                    }
                }
                int local_maneuver_bonus = 0;
                int actual_maneuver_bonus = 0;
                if (top_area_max == 0) {
                    local_maneuver_bonus = compute_candidate_maneuver_bonus(solver, x, y, bomb, dist_P);
                    actual_maneuver_bonus = local_maneuver_bonus;
                    score += actual_maneuver_bonus;
                }

                score += top_area_max;
                if (!solver->is_scanning && top_area_max == 0 && target_bonus == 0 && box_bonus == 0 &&
                    !opens_new_area && !hit_puddle_core) {
                    component_bridge_bonus = score_blast_component_bridge(&solver->bmap, all_obs, dist_P, x, y);
                    score += component_bridge_bonus;
                }
                int actual_short_bonus = 0;
                if (target_bonus == 0 && box_bonus == 0) {
                    actual_short_bonus = short_area_max;
                    score += actual_short_bonus;
                }

                if (g_enable_topology_soft_order) {
                    int precomputed_topology_bonus = topology_soft_blast_bonus(solver, depth, x, y);
                    score += precomputed_topology_bonus;
                }

                int topology_score = score;
                uint8_t topology_bucket = 0;
                if (top_area_max > 0 || target_bonus >= 8000 || box_bonus >= 6000 ||
                    component_bridge_bonus >= BOMB_TOPO_BRIDGE_BONUS ||
                    (opens_new_area && (hit_puddle_core || thin_wall_bonus >= 1000 || target_bonus > 0 || box_bonus > 0))) {
                    topology_bucket = 2;
                } else if (opens_new_area || hit_puddle_core || component_bridge_bonus > 0 || actual_short_bonus > 0 || topology_score >= 2500) {
                    topology_bucket = 1;
                }

                int player_to_bomb = player_to_bomb_route;
                if (player_to_bomb == 0xFFFF) {
                    continue;
                }
                int path_lower_bound = min_push_dist + ((player_to_bomb > 0) ? (player_to_bomb - 1) : 0);
                if (path_lower_bound > 0xFFFF) path_lower_bound = 0xFFFF;
                if ((uint16_t)path_lower_bound < min_candidate_path_lower) {
                    min_candidate_path_lower = (uint16_t)path_lower_bound;
                }
                score -= (min_push_dist * 150);
                score -= (player_to_bomb * 50);

                BombWallCandidate new_cand = {
                    .b_idx = b_idx,
                    .wall_pos = (Position){(uint8_t)x, (uint8_t)y},
                    .score = bomb_candidate_i16(score),
                    .topology_bucket = topology_bucket,
                    .topology_score = bomb_candidate_i16(topology_score),
                    .applied_top_bonus = bomb_candidate_i16(top_area_max),
                    .applied_short_bonus = bomb_candidate_i16(actual_short_bonus),
                    .applied_maneuver_bonus = bomb_candidate_i16(actual_maneuver_bonus),
                    .flags = (top_area_max == 0) ? BOMB_CAND_FLAG_MANEUVER_EVALUATED : 0,
                    .path_lower_bound = (uint16_t)path_lower_bound,
                    .route_slot = 0,
                    .route_len = 0xFFFF
                };

                insert_bomb_candidate_sorted(top_candidates, &num_candidates, MAX_BOMB_CANDIDATES, new_cand);
        }
    }

    BombWallCandidate* light_evac_candidates = g_bomb_light_evac_candidates[depth];
    int num_light_evac_candidates = 0;
    if (light_evac_context_allowed(solver) && g_light_evac_recursion_guard < MAX_LIGHT_EVAC_DEPTH) {
#define ADD_LIGHT_EVAC_CANDIDATE(b_idx_expr, wall_expr, score_seed_expr) do { \
            uint8_t add_b_idx = (uint8_t)(b_idx_expr); \
            Position add_wall = (wall_expr); \
            bool duplicate = false; \
            for (int existing = 0; existing < num_light_evac_candidates; existing++) { \
                if (light_evac_candidates[existing].b_idx == add_b_idx && \
                    pos_equal(light_evac_candidates[existing].wall_pos, add_wall)) { \
                    duplicate = true; \
                    break; \
                } \
            } \
            if (!duplicate) { \
                Position add_bomb = solver->bombs[add_b_idx].pos; \
                int add_score = (score_seed_expr) - ((int)manhattan_distance(add_bomb, add_wall) * 150); \
                for (int gy = add_wall.y - 1; gy <= add_wall.y + 1; gy++) { \
                    for (int gx = add_wall.x - 1; gx <= add_wall.x + 1; gx++) { \
                        if (gx <= 0 || gx >= MAP_COLS - 1 || gy <= 0 || gy >= MAP_ROWS - 1) continue; \
                        add_score += bomb_candidate_topology_cell_bonus(ghost_topology_bonus, gy, gx); \
                        add_score += bomb_candidate_shortcut_cell_bonus(ghost_shortcut_bonus, gy, gx); \
                        if ((solver->bmap.targets[gy] & bit_mask_at(gx)) != 0) add_score += 400; \
                        if ((solver->bmap.boxes[gy] & bit_mask_at(gx)) != 0) add_score += 300; \
                    } \
                } \
                BombWallCandidate light_cand = { \
                    .b_idx = add_b_idx, \
                    .wall_pos = add_wall, \
                    .score = bomb_candidate_i16(add_score), \
                    .topology_bucket = 0, \
                    .topology_score = bomb_candidate_i16(add_score), \
                    .applied_top_bonus = 0, \
                    .applied_short_bonus = 0, \
                    .applied_maneuver_bonus = 0, \
                    .flags = BOMB_CAND_FLAG_MANEUVER_EVALUATED, \
                    .path_lower_bound = 0xFFFF, \
                    .route_slot = 0, \
                    .route_len = 0xFFFF \
                }; \
                insert_bomb_candidate_sorted(light_evac_candidates, &num_light_evac_candidates, \
                                             LIGHT_EVAC_TOP_WINDOW, light_cand); \
            } \
        } while (0)

        for (int b_idx = 0; b_idx < solver->num_bombs; b_idx++) {
            Position bomb = solver->bombs[b_idx].pos;
            for (int cw = 0; cw < num_candidate_wall_cells; cw++) {
                Position wall = candidate_wall_cells[cw];
                if (light_evac_direct_line_blocked_by_single_box(solver, bomb, wall, NULL)) {
                    ADD_LIGHT_EVAC_CANDIDATE(b_idx, wall, 0);
                }
            }

            for (int d = 0; d < 4; d++) {
                bool saw_single_non_target_box = false;
                int x = (int)bomb.x + DIRECTIONS[d].dx;
                int y = (int)bomb.y + DIRECTIONS[d].dy;
                while (x > 0 && x < MAP_COLS - 1 && y > 0 && y < MAP_ROWS - 1) {
                    uint16_t bit = bit_mask_at(x);
                    if ((solver->bmap.bombs[y] & bit) != 0) break;
                    if ((solver->bmap.boxes[y] & bit) != 0) {
                        int box_idx = light_evac_box_index_at(solver, (Position){(uint8_t)x, (uint8_t)y});
                        if (box_idx < 0 || saw_single_non_target_box) break;
                        saw_single_non_target_box = true;
                        x += DIRECTIONS[d].dx;
                        y += DIRECTIONS[d].dy;
                        continue;
                    }
                    if ((solver->bmap.walls[y] & bit) != 0) {
                        if (saw_single_non_target_box && (g_destructible_mask[y] & bit) != 0) {
                            ADD_LIGHT_EVAC_CANDIDATE(b_idx, ((Position){(uint8_t)x, (uint8_t)y}), 20000);
                        }
                        break;
                    }
                    x += DIRECTIONS[d].dx;
                    y += DIRECTIONS[d].dy;
                }
            }
        }
#undef ADD_LIGHT_EVAC_CANDIDATE
    }

    out->num_candidates = num_candidates;
    
    out->num_light_evac_candidates = num_light_evac_candidates;
    out->min_candidate_path_lower = min_candidate_path_lower;
    return true;
}

static bool __attribute__((noinline)) bomb_candidate_plan_prepare(
    SokobanSolver* solver,
    int depth,
    uint16_t assignment_lower,
    uint16_t local_best,
    BombCandidatePlan* plan
) {
    if (!solver || !plan || depth < 0 || depth >= MAX_BOMBS) return false;

    BombStrategyBuildResult build_result;
    if (!build_bomb_strategy_candidates(solver, depth, &build_result)) return false;

    plan->top_candidates = g_bomb_top_candidates[depth];
    plan->light_evac_candidates = g_bomb_light_evac_candidates[depth];
    plan->route_base_map = &g_bomb_route_base_map[depth];
    plan->order_ctx = &g_bomb_order_context_scratch[depth];
    plan->num_candidates = build_result.num_candidates;
    plan->num_light_evac_candidates = build_result.num_light_evac_candidates;
    plan->min_candidate_path_lower = build_result.min_candidate_path_lower;
    plan->num_candidates_before_dedupe = plan->num_candidates;

    build_bomb_route_base_map(solver, plan->route_base_map);

    if (!solver->is_scanning) {
        dedupe_bomb_candidates_by_footprint(solver, plan->top_candidates, &plan->num_candidates);
    }

    for (int i = 0; i < MAX_BOMB_CANDIDATES; i++) {
        g_bomb_reach_all_len_pool[depth][i] = 0xFFFF;
    }
    for (int i = 0; i < plan->num_candidates; i++) {
        bomb_candidate_set_route_precomputed(&plan->top_candidates[i], false);
        plan->top_candidates[i].route_slot = (uint8_t)i;
        plan->top_candidates[i].route_len = 0xFFFF;
    }

    precompute_bomb_routes_range(
        solver, depth, plan->route_base_map, plan->top_candidates, plan->num_candidates,
        0, MAX_REACH_ALL_PREFETCH, assignment_lower, local_best);

    plan->num_candidates_before_dedupe = plan->num_candidates;
    dedupe_bomb_candidates_by_blast(solver, depth, plan->top_candidates, &plan->num_candidates);

    *plan->order_ctx = build_bomb_order_context(
        plan->num_candidates, plan->num_candidates_before_dedupe, plan->top_candidates);
    plan->policy = build_bomb_strategy_policy(
        solver, depth, plan->order_ctx, plan->top_candidates,
        plan->num_candidates, plan->min_candidate_path_lower);

    if (depth == 0 &&
        plan->policy.enable_root_push_reach_filter &&
        solver_has_large_box_set(solver)) {
        g_enable_push_reach_filter = true;
    }
    return true;
}

static bool __attribute__((noinline)) bomb_attempt_schedule_run(
    SokobanSolver* solver,
    int depth,
    uint32_t current_hash,
    BombCandidatePlan* plan,
    BombAttemptSchedule* schedule,
    BombStrategyScratch* strategy_scratch
) {
    if (!solver || !plan || !schedule || !strategy_scratch) return false;
    if (depth >= MAX_BOMBS || solver->num_bombs == 0) return false;

    Direction* bomb_path = schedule->bomb_path;
    BombWallCandidate* top_candidates = plan->top_candidates;
    BombWallCandidate* light_evac_candidates = plan->light_evac_candidates;
    int num_candidates = plan->num_candidates;
    int num_light_evac_candidates = plan->num_light_evac_candidates;
    uint16_t min_candidate_path_lower = plan->min_candidate_path_lower;
    uint16_t (*dist_P)[MAP_COLS] = g_bomb_dist_p;
    bool found = schedule->found;
    uint16_t local_best = schedule->local_best;
    int valid_attempts = schedule->valid_attempts;
    uint16_t assignment_lower = schedule->assignment_lower;
    uint16_t* branch_attempt_keys = schedule->branch_attempt_keys;
    int branch_attempt_key_count = schedule->branch_attempt_key_count;
    BitboardMap* bomb_route_base_map = plan->route_base_map;
    BombStrategyPolicy policy = plan->policy;
    if (!policy.use_maneuver_wide_order) {
        int* phase_order = g_bomb_phase_order[depth];
        int phase_count = 0;
        uint8_t* phase_used = g_bomb_phase_used[depth];
        uint8_t phase_epoch = next_bomb_phase_epoch(depth);

#define ADD_BASE_PHASE_CANDIDATE(idx_expr) do { \
            int add_idx = (idx_expr); \
            if (add_idx >= 0 && add_idx < num_candidates && phase_used[add_idx] != phase_epoch) { \
                phase_used[add_idx] = phase_epoch; \
                phase_order[phase_count++] = add_idx; \
            } \
        } while (0)


        int base_window = policy.base_attempt_limit;
        if (policy.reserve_root_low_lb_append) {
            base_window -= policy.root_low_lb_append;
        }
        if (base_window < 0) base_window = 0;
        if (base_window > num_candidates) base_window = num_candidates;
        for (int i = 0; i < base_window; i++) {
            ADD_BASE_PHASE_CANDIDATE(i);
        }
        if (policy.allow_root_low_lb_append) {
            int appended = 0;
            bool* low_used = strategy_scratch->low_used;
            memset(low_used, 0, MAX_BOMB_CANDIDATES * sizeof(low_used[0]));
            while (appended < policy.root_low_lb_append) {
                int best_idx = -1;
                int best_rank = 0x7FFFFFFF;
                for (int i = 0; i < num_candidates; i++) {
                    if (phase_used[i] == phase_epoch || low_used[i]) continue;
                    if (top_candidates[i].route_len == 0xFFFF || top_candidates[i].path_lower_bound == 0xFFFF) continue;
                    if (top_candidates[i].path_lower_bound > policy.low_lb_max_path_lower) continue;
                    int rank = ((int)top_candidates[i].path_lower_bound * 100) +
                               ((int)top_candidates[i].route_len * 10) -
                               (top_candidates[i].score / 100);
                    if (rank < best_rank) {
                        best_rank = rank;
                        best_idx = i;
                    }
                }
                if (best_idx < 0) break;
                low_used[best_idx] = true;
                ADD_BASE_PHASE_CANDIDATE(best_idx);
                appended++;
            }
        }
        for (int i = base_window; i < num_candidates; i++) {
            ADD_BASE_PHASE_CANDIDATE(i);
        }
#undef ADD_BASE_PHASE_CANDIDATE

        for (int oi = 0; oi < phase_count; oi++) {
            int i = phase_order[oi];
            bomb_candidate_set_tried(&top_candidates[i], true);
            int before_attempts = valid_attempts;
            bool stop = try_bomb_candidate_branch(solver, depth, current_hash, bomb_route_base_map,
                                                  &top_candidates[i], bomb_path, assignment_lower,
                                                  &local_best, &found, &valid_attempts, policy.base_attempt_limit, false);
            if (valid_attempts > before_attempts) {
                bomb_strategy_branch_attempt_remember(branch_attempt_keys, &branch_attempt_key_count, &top_candidates[i]);
            }
            if (stop) {
                break;
            }
        }
    } else {
        int* phase_order = g_bomb_phase_order[depth];
        int phase_count = 0;
        uint8_t* phase_used = g_bomb_phase_used[depth];
        uint8_t phase_epoch = next_bomb_phase_epoch(depth);

#define ADD_PHASE_CANDIDATE(idx_expr) do { \
            int add_idx = (idx_expr); \
            if (add_idx >= 0 && add_idx < num_candidates && phase_used[add_idx] != phase_epoch) { \
                phase_used[add_idx] = phase_epoch; \
                phase_order[phase_count++] = add_idx; \
            } \
        } while (0)

        int bootstrap_idx = -1;
        int bootstrap_rank = 0x7FFFFFFF;
        int scan_limit = (num_candidates < policy.base_attempt_limit) ? num_candidates : policy.base_attempt_limit;

        for (int i = 1; i < scan_limit; i++) {
            const BombWallCandidate* cand = &top_candidates[i];
            if (cand->applied_maneuver_bonus != 0) continue;
            if (cand->route_len == 0xFFFF || cand->path_lower_bound == 0xFFFF) continue;
            int rank = ((int)cand->route_len * 100) + ((int)cand->path_lower_bound * 10) - (cand->score / 50);
            if (rank < bootstrap_rank) {
                bootstrap_rank = rank;
                bootstrap_idx = i;
            }
        }

        ADD_PHASE_CANDIDATE(bootstrap_idx);
        for (int i = 0; i < num_candidates; i++) {
            ADD_PHASE_CANDIDATE(i);
        }
#undef ADD_PHASE_CANDIDATE

        for (int oi = 0; oi < phase_count; oi++) {
            int i = phase_order[oi];
            bomb_candidate_set_tried(&top_candidates[i], true);
            int before_attempts = valid_attempts;
            bool stop = try_bomb_candidate_branch(solver, depth, current_hash, bomb_route_base_map,
                                                  &top_candidates[i], bomb_path, assignment_lower,
                                                  &local_best, &found, &valid_attempts, policy.base_attempt_limit, false);
            if (valid_attempts > before_attempts) {
                bomb_strategy_branch_attempt_remember(branch_attempt_keys, &branch_attempt_key_count, &top_candidates[i]);
            }
            if (stop) {
                break;
            }
        }
    }

    if (g_bomb_seed_first_solution_only && found) goto commit_found;

    bool scan_skip_light_evac = false;
    if (solver && solver->is_scanning && !solver->strict_target_mode && found &&
        assignment_lower != 0xFFFF && min_candidate_path_lower != 0xFFFF) {
        uint32_t scan_floor = (uint32_t)min_candidate_path_lower + assignment_lower;
        scan_skip_light_evac = ((uint32_t)local_best <= scan_floor + SCAN_LIGHT_EVAC_SKIP_MARGIN);
    }
    if (!scan_skip_light_evac && num_light_evac_candidates > 0) {
        int light_attempts = 0;
        for (int li = 0; li < num_light_evac_candidates && light_attempts < LIGHT_EVAC_TOP_WINDOW; li++) {
            uint16_t before_light_best = local_best;
            bool light_direct_stop = false;
            if (!bomb_strategy_branch_attempt_seen(
                    branch_attempt_keys, branch_attempt_key_count,
                    &light_evac_candidates[li])) {
                int before_attempts = light_attempts;
                light_direct_stop = try_bomb_candidate_branch(solver, depth, current_hash, bomb_route_base_map,
                                                              &light_evac_candidates[li], bomb_path, assignment_lower,
                                                              &local_best, &found, &light_attempts, LIGHT_EVAC_TOP_WINDOW, false);
                if (light_attempts > before_attempts) {
                    bomb_strategy_branch_attempt_remember(branch_attempt_keys, &branch_attempt_key_count, &light_evac_candidates[li]);
                }
            }
            if (light_direct_stop) {
                break;
            }
            if (try_bomb_candidate_with_light_evac(solver, depth, current_hash, bomb_route_base_map,
                                                   &light_evac_candidates[li], bomb_path, assignment_lower,
                                                   &local_best, &found, &light_attempts, LIGHT_EVAC_TOP_WINDOW, false,
                                                   li, num_light_evac_candidates)) {
                break;
            }
            if (before_light_best != 0xFFFF && local_best >= before_light_best) {
                local_best = before_light_best;
            }
        }
    }

    if (g_bomb_seed_first_solution_only && found) goto commit_found;

    bool base_good_enough = false;
    if (found && assignment_lower != 0xFFFF && min_candidate_path_lower != 0xFFFF) {
        uint32_t base_floor = (uint32_t)min_candidate_path_lower + assignment_lower;
        int remaining_boxes = 0;
        for (int bi = 0; bi < solver->num_boxes; bi++) {
            Position bp = solver->boxes[bi].pos;
            if ((solver->bmap.targets[bp.y] & (1 << bp.x)) == 0) {
                remaining_boxes++;
            }
        }
        uint32_t dynamic_margin = (uint32_t)(remaining_boxes * 6 + 5);
        base_good_enough = ((uint32_t)local_best <= base_floor + dynamic_margin);
    }
    if (!base_good_enough && found && depth == 0 && num_candidates > 0 && min_candidate_path_lower != 0xFFFF &&
        top_candidates[0].path_lower_bound > (uint16_t)(min_candidate_path_lower + 6)) {
        int proximity_attempts = 0;
        while (proximity_attempts < policy.proximity_attempt_limit) {
            int best_idx = -1;
            uint16_t best_lower = 0xFFFF;

            for (int i = 0; i < num_candidates; i++) {
                if (bomb_candidate_tried(&top_candidates[i])) continue;
                if (top_candidates[i].path_lower_bound >= best_lower) continue;
                if (assignment_lower != 0xFFFF &&
                    (uint32_t)top_candidates[i].path_lower_bound + assignment_lower >= local_best) {
                    continue;
                }
                best_lower = top_candidates[i].path_lower_bound;
                best_idx = i;
            }

            if (best_idx == -1) break;
            bomb_candidate_set_tried(&top_candidates[best_idx], true);
            if (bomb_strategy_branch_attempt_seen(branch_attempt_keys, branch_attempt_key_count, &top_candidates[best_idx])) {
                continue;
            }
            int before_attempts = proximity_attempts;
            bool stop = try_bomb_candidate_branch(solver, depth, current_hash, bomb_route_base_map,
                                                  &top_candidates[best_idx], bomb_path, assignment_lower,
                                                  &local_best, &found, &proximity_attempts, policy.proximity_attempt_limit, false);
            if (proximity_attempts > before_attempts) {
                bomb_strategy_branch_attempt_remember(branch_attempt_keys, &branch_attempt_key_count, &top_candidates[best_idx]);
            }
            if (stop) {
                break;
            }
        }
    }
    if (depth == 0 && !found && !base_good_enough) {
        int struct_attempts = 0;
        for (int bucket = 2;
             bucket >= 1 && struct_attempts < policy.struct_attempt_limit &&
             !(g_bomb_seed_first_solution_only && found);
             bucket--) {
            for (int i = 0; i < num_candidates && struct_attempts < policy.struct_attempt_limit; i++) {
                if (bomb_candidate_tried(&top_candidates[i])) continue;
                if (top_candidates[i].topology_bucket != bucket) continue;
                if (found) {
                    if (top_candidates[i].path_lower_bound >= local_best) continue;
                    if (assignment_lower != 0xFFFF && (uint32_t)top_candidates[i].path_lower_bound + assignment_lower >= local_best) continue;
                }

                bomb_candidate_set_tried(&top_candidates[i], true);
                if (bomb_strategy_branch_attempt_seen(branch_attempt_keys, branch_attempt_key_count, &top_candidates[i])) {
                    continue;
                }
                int before_attempts = struct_attempts;
                bool stop = try_bomb_candidate_branch(solver, depth, current_hash, bomb_route_base_map,
                                                      &top_candidates[i], bomb_path, assignment_lower,
                                                      &local_best, &found, &struct_attempts, policy.struct_attempt_limit, false);
                if (struct_attempts > before_attempts) {
                    bomb_strategy_branch_attempt_remember(branch_attempt_keys, &branch_attempt_key_count, &top_candidates[i]);
                }
                if (stop) {
                    break;
                }
            }
        }
    }

    if (g_bomb_seed_first_solution_only && found) goto commit_found;

    bool need_rescue = !base_good_enough;

    if (need_rescue) {
        BombWallCandidate* rescue_candidates = top_candidates;
        int num_rescue_candidates = 0;

        for (int i = 0; i < num_candidates; i++) {
            BombWallCandidate cand = top_candidates[i];
            if (bomb_candidate_tried(&cand)) continue;
            if (cand.applied_top_bonus != 0) continue;

            if (found) {
                if (cand.path_lower_bound >= local_best) continue;
                if (assignment_lower != 0xFFFF && (uint32_t)cand.path_lower_bound + assignment_lower >= local_best) continue;
            }

            int maneuver_bonus = cand.applied_maneuver_bonus;
            if (!bomb_candidate_maneuver_evaluated(&cand)) {
                Position bomb = solver->bombs[cand.b_idx].pos;
                maneuver_bonus = compute_candidate_maneuver_bonus(solver, cand.wall_pos.x, cand.wall_pos.y, bomb, dist_P);
                bomb_candidate_set_maneuver_evaluated(&cand, true);
                cand.applied_maneuver_bonus = bomb_candidate_i16(maneuver_bonus);
                cand.score = bomb_candidate_i16((int)cand.score + maneuver_bonus);
            }
            if (maneuver_bonus <= 0) continue;
            rescue_candidates[num_rescue_candidates++] = cand;
        }
        sort_bomb_candidates_desc(rescue_candidates, num_rescue_candidates);

        int rescue_attempts = 0;
        for (int i = 0; i < num_rescue_candidates; i++) {
            if (found) {
                if (rescue_candidates[i].path_lower_bound >= local_best) continue;
                if (assignment_lower != 0xFFFF && (uint32_t)rescue_candidates[i].path_lower_bound + assignment_lower >= local_best) continue;
            }

            bomb_candidate_set_tried(&rescue_candidates[i], true);
            if (bomb_strategy_branch_attempt_seen(branch_attempt_keys, branch_attempt_key_count, &rescue_candidates[i])) {
                continue;
            }
            
            int before_attempts = rescue_attempts;
            bool stop = try_bomb_candidate_branch(solver, depth, current_hash, bomb_route_base_map,
                                                  &rescue_candidates[i], bomb_path, assignment_lower,
                                                  &local_best, &found, &rescue_attempts, policy.rescue_attempt_limit, false);
            if (rescue_attempts > before_attempts) {
                bomb_strategy_branch_attempt_remember(branch_attempt_keys, &branch_attempt_key_count, &rescue_candidates[i]);
            }
            if (stop) {

                break;
            }

        }
    }
commit_found:
    if (found) {
        if (!scan_audit_path_no_absorb_before_blast(solver, g_simple_path_pool[depth], local_best)) {
            return false;
        }
        solver->best_path_len = local_best;
        solver->best_steps = local_best;
        memcpy(solver->best_path, g_simple_path_pool[depth], local_best * sizeof(Direction));
        return true;
    }
    return false;
}

static bool __attribute__((noinline)) try_bomb_strategy_simple(SokobanSolver* solver, int depth, uint32_t current_hash) {
    if (!solver || depth < 0 || depth >= MAX_BOMBS || solver->num_bombs == 0) return false;

    uint16_t local_best = 0xFFFF;
    if (solver->best_steps != 0xFFFF) {
        uint32_t inherit_margin = (solver->is_scanning && !solver->strict_target_mode)
            ? SOKOBAN_SCAN_INHERIT_SOFT_MARGIN
            : BOMB_INHERIT_SOFT_MARGIN;
        uint32_t soft_best = (uint32_t)solver->best_steps + inherit_margin;
        local_best = (soft_best >= 0xFFFFu) ? 0xFFFF : (uint16_t)soft_best;
    }

    BombCandidatePlan* plan = &g_bomb_candidate_plan_scratch[depth];
    if (!bomb_candidate_plan_prepare(solver, depth, remaining_box_assignment_lower_bound(solver), local_best, plan)) {
        return false;
    }

    BombAttemptSchedule schedule;
    memset(&schedule, 0, sizeof(schedule));
    schedule.bomb_path = g_bomb_path_pool[depth];
    schedule.branch_attempt_keys = g_bomb_strategy_scratch[depth].branch_attempt_keys;
    schedule.assignment_lower = remaining_box_assignment_lower_bound(solver);
    schedule.local_best = local_best;
    return bomb_attempt_schedule_run(solver, depth, current_hash, plan, &schedule, &g_bomb_strategy_scratch[depth]);
}

typedef enum {
    SCAN_PROVED_BAD,
    SCAN_PROVED_GOOD,
    SCAN_UNKNOWN
} ScanProofResult;

static bool scan_pair_may_need_validation(const SokobanSolver* solver, const Entity* box, const Entity* target) {
    if (!solver || !box || !target) return false;

    int box_id = box->id;
    int target_id = target->id;
    if (box_id == -1 || target_id == -1 || box_id == target_id) return true;
    if (box_id < 0 || box_id >= 10 || target_id < 0 || target_id >= 10) return false;

    int b_counts[10] = {0};
    int t_counts[10] = {0};
    for (int i = 0; i < solver->num_boxes; i++) {
        int id = solver->boxes[i].id;
        if (id >= 0 && id < 10) b_counts[id]++;
    }
    for (int i = 0; i < solver->num_targets; i++) {
        int id = solver->targets[i].id;
        if (id >= 0 && id < 10) t_counts[id]++;
    }

    int common_count = 0;
    for (int id = 0; id < 10; id++) {
        common_count += (b_counts[id] < t_counts[id]) ? b_counts[id] : t_counts[id];
    }
    if (common_count == 0) return false;

    return b_counts[box_id] > t_counts[box_id] && t_counts[target_id] > b_counts[target_id];
}

static bool scan_position_in_unknown_boxes(const SokobanSolver* solver, const Position* boxes, int x, int y) {
    for (int i = 0; i < solver->num_boxes; i++) {
        if (solver->boxes[i].id == -1 && boxes[i].x == x && boxes[i].y == y) return true;
    }
    return false;
}

FAST_OCRAM_FUNC static ScanProofResult __attribute__((noinline)) verify_blind_scan_robustness(const SokobanSolver* solver, const BitboardMap* base_map,
                                                    Position start_player, const Position* start_boxes) {
    bool box_assigned[MAX_BOXES] = {false};
    if (has_any_deadlock(solver, base_map, start_boxes, box_assigned)) {
        return SCAN_PROVED_BAD;
    }

    BitboardMap optimistic = *base_map;
    memset(optimistic.boxes, 0, sizeof(optimistic.boxes));
    memset(optimistic.bombs, 0, sizeof(optimistic.bombs));

    const uint16_t interior_cols_mask = (uint16_t)(((uint16_t)1u << (MAP_COLS - 1)) - 2u);
    uint16_t blocked[MAP_ROWS];
    for (int y = 0; y < MAP_ROWS; y++) {
        blocked[y] = optimistic.walls[y];
    }

    for (int j = 0; j < solver->num_targets; j++) {
        bool target_needed = false;
        for (int i = 0; i < solver->num_boxes; i++) {
            if (scan_pair_may_need_validation(solver, &solver->boxes[i], &solver->targets[j])) {
                target_needed = true;
                break;
            }
        }
        if (!target_needed) continue;

        Position target = solver->targets[j].pos;
        uint16_t target_bit = bit_mask_at(target.x);
        if ((blocked[target.y] & target_bit) != 0) {
            return SCAN_PROVED_BAD;
        }

        uint16_t reach[MAP_ROWS] = {0};
        uint16_t next[MAP_ROWS];
        reach[target.y] = target_bit;

        do {
            bool changed = false;
            memcpy(next, reach, sizeof(next));

            for (int y = 1; y < MAP_ROWS - 1; y++) {
                uint16_t expanded = reach[y];
                expanded |= (uint16_t)((expanded << 1) | (expanded >> 1));
                expanded |= reach[y - 1] | reach[y + 1];
                expanded &= interior_cols_mask;
                expanded &= (uint16_t)(~blocked[y]);

                if (expanded != reach[y]) {
                    next[y] = expanded;
                    changed = true;
                }
            }

            memcpy(reach, next, sizeof(reach));
            if (!changed) break;
        } while (true);

        for (int i = 0; i < solver->num_boxes; i++) {
            if (!scan_pair_may_need_validation(solver, &solver->boxes[i], &solver->targets[j])) continue;
            if ((reach[start_boxes[i].y] & bit_mask_at(start_boxes[i].x)) == 0) {
                return SCAN_PROVED_BAD;
            }
        }
    }

    int unknown_boxes = 0;
    int unknown_targets = 0;
    int min_x = MAP_COLS, max_x = -1, min_y = MAP_ROWS, max_y = -1;

    for (int i = 0; i < solver->num_boxes; i++) {
        if (solver->boxes[i].id != -1) continue;
        Position p = start_boxes[i];
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
        unknown_boxes++;
    }

    for (int j = 0; j < solver->num_targets; j++) {
        if (solver->targets[j].id != -1) continue;
        Position p = solver->targets[j].pos;
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
        unknown_targets++;
    }

    if (unknown_boxes == 0 || unknown_boxes != unknown_targets) {
        return SCAN_UNKNOWN;
    }

    min_x--; max_x++; min_y--; max_y++;
    if (min_x < 1) min_x = 1;
    if (min_y < 1) min_y = 1;
    if (max_x > MAP_COLS - 2) max_x = MAP_COLS - 2;
    if (max_y > MAP_ROWS - 2) max_y = MAP_ROWS - 2;

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;
    if (width < 4 || height < 4) {
        return SCAN_UNKNOWN;
    }

    if (start_player.x < min_x || start_player.x > max_x || start_player.y < min_y || start_player.y > max_y) {
        return SCAN_UNKNOWN;
    }

    int room_cells = 0;
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            uint16_t bit = bit_mask_at(x);
            if ((base_map->walls[y] & bit) != 0) return SCAN_UNKNOWN;
            if ((base_map->deadlocks[y] & bit) != 0) return SCAN_UNKNOWN;
            if ((base_map->h_tunnels[y] & bit) != 0) return SCAN_UNKNOWN;
            if ((base_map->v_tunnels[y] & bit) != 0) return SCAN_UNKNOWN;
            if ((base_map->bombs[y] & bit) != 0) return SCAN_UNKNOWN;
            if ((base_map->boxes[y] & bit) != 0 && !scan_position_in_unknown_boxes(solver, start_boxes, x, y)) {
                return SCAN_UNKNOWN;
            }
            room_cells++;
        }
    }

    if (room_cells < unknown_boxes + 4) {
        return SCAN_UNKNOWN;
    }

    return SCAN_PROVED_GOOD;
}
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static Direction g_verify_dummy_path[MAX_PATH_LENGTH] ALLOC_IN_SDRAM;

static inline void clear_dfs_transposition_table(void) {
    clear_transposition_table_domain(TT_DOMAIN_DFS);
}

static bool verify_permutation_cooperative(SokobanSolver* solver, BitboardMap* candidate_map,
                                           Position start_player, Position* candidate_boxes, int* current_mapping) {
    static SokobanSolver temp_solver;
    temp_solver = *solver;

    temp_solver.best_path = g_verify_dummy_path;
    temp_solver.bmap = *candidate_map;
    temp_solver.start_player = start_player;
    temp_solver.num_bombs = 0;
    memset(temp_solver.bombs, 0, sizeof(temp_solver.bombs));

    for (int i = 0; i < temp_solver.num_boxes; i++) {
        temp_solver.boxes[i].pos = candidate_boxes[i];
    }

    bool used_temp_ids[10] = {false};
    for (int i = 0; i < temp_solver.num_boxes; i++) {
        int id = temp_solver.boxes[i].id;
        if (id >= 0 && id < 10) used_temp_ids[id] = true;
    }
    int next_temp_id = 0;
    for (int i = 0; i < temp_solver.num_boxes; i++) {
        if (temp_solver.boxes[i].id == -1) {
            while (next_temp_id < 10 && used_temp_ids[next_temp_id]) next_temp_id++;
            if (next_temp_id >= 10) return false;
            temp_solver.boxes[i].id = (int8_t)next_temp_id;
            used_temp_ids[next_temp_id] = true;
        } else if (temp_solver.boxes[i].id < 0 || temp_solver.boxes[i].id >= 10) {
            return false;
        }
    }

    for (int i = 0; i < temp_solver.num_targets; i++) {
        temp_solver.targets[i].id = -1;
    }
    for (int i = 0; i < temp_solver.num_boxes; i++) {
        int target_idx = current_mapping[i];
        if (target_idx < 0 || target_idx >= temp_solver.num_targets) return false;
        temp_solver.targets[target_idx].id = temp_solver.boxes[i].id;
    }

    temp_solver.strict_target_mode = true;
    temp_solver.best_steps = 0xFFFF;
    temp_solver.best_path_len = 0;

    if (has_any_deadlock(&temp_solver, &temp_solver.bmap, NULL, NULL)) return false;

    clear_dfs_transposition_table();
    prepare_dfs_temp_maps(&temp_solver);

    Position current_boxes[MAX_BOXES];
    bool box_assigned[MAX_BOXES] = {false};
    bool target_filled[MAX_TARGETS] = {false};
    int initial_depth = 0;

    for (int i = 0; i < temp_solver.num_boxes; i++) {
        current_boxes[i] = temp_solver.boxes[i].pos;
        if ((temp_solver.bmap.targets[current_boxes[i].y] & (1 << current_boxes[i].x)) != 0) {
            bool can_lock = false;
            for (int t = 0; t < temp_solver.num_targets; t++) {
                if (temp_solver.targets[t].id == temp_solver.boxes[i].id && pos_equal(temp_solver.targets[t].pos, current_boxes[i])) {
                    can_lock = true;
                    break;
                }
            }
            if (can_lock) {
                box_assigned[i] = true;
                initial_depth++;
                for (int t = 0; t < temp_solver.num_targets; t++) {
                    if (!target_filled[t] && pos_equal(temp_solver.targets[t].pos, current_boxes[i])) {
                        target_filled[t] = true;
                        break;
                    }
                }
            }
        }
    }

    if (initial_depth < MAX_BOXES) {
        g_dfs_temp_map[initial_depth] = temp_solver.bmap;
        rebuild_active_boxes_layer(&g_dfs_temp_map[initial_depth], current_boxes, box_assigned, temp_solver.num_boxes);
    }

    uint16_t saved_astar_max_steps = g_astar_max_steps;
    g_astar_max_steps = 0xFFFF;
    bool saved_first_solution_only = g_dfs_first_solution_only;
    g_dfs_first_solution_only = true;
    uint32_t hash = compute_universe_hash(&temp_solver);
    solve_permutation_dfs(&temp_solver, initial_depth, 0, start_player, current_boxes, box_assigned, target_filled, g_dfs_full_path_buffer, 0, hash);
    g_dfs_first_solution_only = saved_first_solution_only;
    g_astar_max_steps = saved_astar_max_steps;

    return temp_solver.best_steps != 0xFFFF;
}

static bool __attribute__((noinline)) generate_and_verify_permutations(SokobanSolver* solver, BitboardMap* base_map, Position start_player,
                                             Position* start_boxes, int* current_mapping, bool* target_used, int box_idx) {
    if (box_idx == 0) {
        build_distance_field_for_depth(base_map, solver->targets, solver->num_targets, 0);
        build_target_distance_fields(base_map, solver->targets, solver->num_targets);

        bool all_boxes_unknown = true;
        for (int i = 0; i < solver->num_boxes; i++) {
            if (solver->boxes[i].id != -1) {
                all_boxes_unknown = false;
                break;
            }
        }

        if (all_boxes_unknown) {
            for (int i = 0; i < solver->num_boxes; i++) current_mapping[i] = i;
            return verify_permutation_cooperative(solver, base_map, start_player, start_boxes, current_mapping);
        }
    }

    if (box_idx == solver->num_boxes) {
        return verify_permutation_cooperative(solver, base_map, start_player, start_boxes, current_mapping);
    }

    for (int t = 0; t < solver->num_targets; t++) {
        if (!target_used[t]) {
            if (!scan_pair_may_need_validation(solver, &solver->boxes[box_idx], &solver->targets[t])) continue;

            target_used[t] = true;
            current_mapping[box_idx] = t;

            if (!generate_and_verify_permutations(solver, base_map, start_player, start_boxes,
                                                  current_mapping, target_used, box_idx + 1)) {
                target_used[t] = false;
                current_mapping[box_idx] = -1;
                return false;
            }

            target_used[t] = false;
            current_mapping[box_idx] = -1;
        }
    }
    return true;
}

FAST_RAM_FUNC static bool sokoban_solve_internal(SokobanSolver* solver, int depth, uint32_t current_hash) {
    if (!solve_budget_enter(solver)) return false;
    if (depth >= MAX_BOMBS || solver->num_boxes != solver->num_targets) return false;

    uint16_t initial_best = solver->best_steps;
    bool use_component_fail_cache =
        (!g_sandbox_mode && !g_bomb_state_beam_tail_only && initial_best == 0xFFFF);
    uint32_t component_fail_key = 0;

    if (!g_sandbox_mode && !g_bomb_state_beam_tail_only) {
        int tt_status = lookup_transposition_table(TT_DOMAIN_UNIVERSE, current_hash, initial_best, solver->num_bombs);
        if (tt_status == TT_STATUS_DEADEND) return false;
        if (tt_status == TT_STATUS_SEARCHING) return false;
    }

    if (use_component_fail_cache) {
        component_fail_key = compute_component_fail_key(solver, current_hash);
        if (component_fail_cached(component_fail_key, initial_best, solver->num_bombs)) {
            return false;
        }
    }

    if (has_any_deadlock(solver, &solver->bmap, NULL, NULL)) {
        if (!g_sandbox_mode && !g_bomb_state_beam_tail_only) {
            store_transposition_table(TT_DOMAIN_UNIVERSE, current_hash, initial_best, solver->num_bombs, TT_STATUS_DEADEND);
        }
        if (use_component_fail_cache) {
            component_fail_store(component_fail_key, initial_best, solver->num_bombs);
        }
        return false;
    }

    if (!g_sandbox_mode && !g_bomb_state_beam_tail_only) {
        store_transposition_table(TT_DOMAIN_UNIVERSE, current_hash, initial_best, solver->num_bombs, TT_STATUS_SEARCHING);
    }

    build_distance_field_for_depth(&solver->bmap, solver->targets, solver->num_targets, depth);
    build_target_distance_fields(&solver->bmap, solver->targets, solver->num_targets);

    hash_table_clear();

    bool found_any = false;
    bool pocket_unblocked = false;
    if (try_player_pocket_unblock(solver, depth, current_hash)) {
        found_any = true;
        pocket_unblocked = true;
    }

    if (!g_bomb_state_beam_tail_only && !pocket_unblocked && solver->num_bombs > 0) {
        if (try_bomb_strategy_simple(solver, depth, current_hash)) found_any = true;
    }
    if (!pocket_unblocked) {
    SolveInternalScratch* solve_scratch = &g_solve_internal_scratch[depth];
    Position* current_boxes = solve_scratch->current_boxes;
    bool* box_assigned = solve_scratch->box_assigned;
    bool* target_filled = solve_scratch->target_filled;
    for (int i = 0; i < solver->num_boxes; i++) current_boxes[i] = solver->boxes[i].pos;
    memset(box_assigned, 0, MAX_BOXES * sizeof(box_assigned[0]));
    memset(target_filled, 0, MAX_TARGETS * sizeof(target_filled[0]));

    int initial_depth = 0;
    for (int i = 0; i < solver->num_boxes; i++) {
        if ((solver->bmap.targets[current_boxes[i].y] & (1 << current_boxes[i].x)) != 0) {
            bool can_lock = true;
            if (solver->strict_target_mode) {
                can_lock = false;
                for (int t = 0; t < solver->num_targets; t++) {
                    if (solver->targets[t].id == solver->boxes[i].id && pos_equal(solver->targets[t].pos, current_boxes[i])) {
                        can_lock = true; break;
                    }
                }
            }
            if (can_lock) {
                box_assigned[i] = true;
                initial_depth++;
                for (int t = 0; t < solver->num_targets; t++) {
                    if (!target_filled[t] && pos_equal(solver->targets[t].pos, current_boxes[i])) {
                        target_filled[t] = true; break;
                    }
                }
            }
        }
    }

    uint16_t best_before = solver->best_steps;
    uint16_t best_len_before = solver->best_path_len;
    Direction* best_path_before = g_solve_best_path_before[depth];
    if (best_before != 0xFFFF) {
        memcpy(best_path_before, solver->best_path, best_len_before * sizeof(Direction));
    }

    prepare_dfs_temp_maps(solver);
    if (initial_depth < MAX_BOXES) {
        g_dfs_temp_map[initial_depth] = solver->bmap;
        rebuild_active_boxes_layer(&g_dfs_temp_map[initial_depth], current_boxes, box_assigned, solver->num_boxes);
    }
    AssignmentBeamResult assignment_beam_result = solve_assignment_beam(
        solver,
        initial_depth,
        depth,
        solver->start_player,
        current_boxes,
        box_assigned,
        target_filled,
        g_dfs_full_path_buffer,
        0
    );
    if (assignment_beam_result == ASSIGNMENT_BEAM_FATAL) return false;
    bool run_dfs_fallback =
        assignment_beam_result == ASSIGNMENT_BEAM_INELIGIBLE ||
        (assignment_beam_result == ASSIGNMENT_BEAM_EXHAUSTED &&
         !g_assignment_bounded_refinement);
    if (run_dfs_fallback) {
        if (assignment_beam_result == ASSIGNMENT_BEAM_EXHAUSTED) {
            clear_dfs_transposition_table();
            prepare_dfs_temp_maps(solver);
            if (initial_depth < MAX_BOXES) {
                g_dfs_temp_map[initial_depth] = solver->bmap;
                rebuild_active_boxes_layer(
                    &g_dfs_temp_map[initial_depth],
                    current_boxes,
                    box_assigned,
                    solver->num_boxes
                );
            }
        }
        solve_permutation_dfs(
            solver,
            initial_depth,
            depth,
            solver->start_player,
            current_boxes,
            box_assigned,
            target_filled,
            g_dfs_full_path_buffer,
            0,
            current_hash
        );
    }

    if (solver->best_steps < best_before) {
        if (g_enable_path_verification != SCAN_VERIFY_NONE && solver->is_scanning && !solver->strict_target_mode && solver->num_boxes > 1) {

            int* current_mapping = solve_scratch->current_mapping;
            bool* target_used = solve_scratch->target_used;
            Position* current_boxes_eval = solve_scratch->current_boxes_eval;
            memset(target_used, 0, MAX_TARGETS * sizeof(target_used[0]));
            for (int i = 0; i < MAX_BOXES; i++) current_mapping[i] = -1;
            for (int i = 0; i < solver->num_boxes; i++) current_boxes_eval[i] = current_boxes[i];

            ScanProofResult proof = verify_blind_scan_robustness(
                solver, &solver->bmap, solver->start_player, current_boxes_eval
            );

            bool all_pass = true;
            if (proof == SCAN_PROVED_BAD) {
                all_pass = false;
            } else if (proof == SCAN_UNKNOWN && g_enable_path_verification >= SCAN_VERIFY_STRICT) {
                all_pass = generate_and_verify_permutations(
                    solver, &solver->bmap, solver->start_player, current_boxes_eval, current_mapping, target_used, 0
                );
            }

            if (!all_pass) {
                solver->best_steps = best_before;
                solver->best_path_len = best_len_before;
                if (best_before != 0xFFFF) {
                    memcpy(solver->best_path, best_path_before, best_len_before * sizeof(Direction));
                }
            }
        }
    }
    }

    g_astar_max_steps = 0xFFFF;

    if (g_allow_macro_evacuation && solver->best_steps >= initial_best && solver->num_bombs > 0) {
        if (g_current_macro_depth < MAX_MACRO) {
            if (try_smart_evacuation(solver, depth, g_current_macro_depth, current_hash)) {
                found_any = true;
            }
        }
    }

    if (g_allow_super_evacuation && solver->best_steps >= initial_best && solver->num_bombs > 0) {
        if (g_current_super_depth < MAX_SUPER_MACRO) {
            if (try_super_evacuation(solver, depth, g_current_super_depth, current_hash)) {
                found_any = true;
            }
        }
    }

    bool success = (solver->best_steps < initial_best) || found_any;

    if (!g_sandbox_mode && !g_bomb_state_beam_tail_only) {
        if (!success) {
            store_transposition_table(TT_DOMAIN_UNIVERSE, current_hash, initial_best, solver->num_bombs, TT_STATUS_DEADEND);
            if (use_component_fail_cache && !g_solve_budget_exhausted) {
                component_fail_store(component_fail_key, initial_best, solver->num_bombs);
            }
        } else {
            store_transposition_table(TT_DOMAIN_UNIVERSE, current_hash, solver->best_steps, solver->num_bombs, TT_STATUS_SOLVED);
        }
    }

    return success;
}

#define BOMB_STATE_BEAM_MAX_WIDTH 16u
#define BOMB_STATE_BEAM_CHILD_CAP (BOMB_STATE_BEAM_MAX_WIDTH * 2u)
#define BOMB_STATE_BEAM_PATH_BYTES PACKED_PATH_BYTES(MAX_PATH_LENGTH)

typedef struct {
    BitboardMap bmap;
    Entity boxes[MAX_BOXES];
    Entity targets[MAX_TARGETS];
    Entity bombs[MAX_BOMBS];
    Position player;
    uint32_t destroyed_walls_mask;
    uint32_t universe_hash;
    uint32_t rank;
    uint32_t stable_order;
    uint16_t prefix_len;
    uint16_t lower_bound;
    int16_t candidate_score;
    uint8_t num_boxes;
    uint8_t num_targets;
    uint8_t num_bombs;
    PackedDirByte prefix[BOMB_STATE_BEAM_PATH_BYTES];
} BombStateBeamState;

typedef enum {
    BOMB_STATE_BEAM_SKIP = 0,
    BOMB_STATE_BEAM_OK,
    BOMB_STATE_BEAM_FATAL
} BombStateBeamMaterializeResult;

typedef enum {
    BOMB_STATE_BEAM_TAIL_NONE = 0,
    BOMB_STATE_BEAM_TAIL_FOUND,
    BOMB_STATE_BEAM_TAIL_INCOMPLETE,
    BOMB_STATE_BEAM_TAIL_FATAL
} BombStateBeamTailResult;

typedef struct {
    uint16_t walls[MAP_ROWS];
    uint16_t targets[MAP_ROWS];
    uint16_t bombs[MAP_ROWS];
    uint16_t boxes[MAP_ROWS];
    uint16_t h_tunnels[MAP_ROWS];
    uint16_t v_tunnels[MAP_ROWS];
} SqPostMapFrame;

typedef struct {
    SqPostMapFrame sq_post_map_history[MAX_PATH_LENGTH + 1];
} SqPostProcessReplayMapScratch;

typedef struct {
    Position sq_post_pos_history[MAX_PATH_LENGTH + 1];
    Position sq_post_box_history[MAX_PATH_LENGTH + 1][MAX_BOXES];
} SqPostProcessReplayStateScratch;

typedef struct {
    Direction tail_path[MAX_PATH_LENGTH];
    Direction best_candidate[MAX_PATH_LENGTH];
} SqD1PostProcessPathScratch;

typedef struct {
    SqD1PostProcessPathScratch d1;
} SqPostProcessPathScratch;

typedef struct {
    Direction relink_path[MAX_PATH_LENGTH];
    Direction bridge[MAX_SINGLE_PATH];
} SolverPostOptRelinkPathScratch;

typedef struct {
    Direction hybrid[MAX_PATH_LENGTH];
    Direction detour[MAX_SINGLE_PATH];
    Direction bridge[MAX_SINGLE_PATH];
} SolverPostOptTaskPathScratch;

typedef struct {
    Direction shortcut[MAX_SINGLE_PATH];
    Direction new_path[MAX_PATH_LENGTH];
} SolverPostOptShortcutPathScratch;

typedef union {
    SolverPostOptRelinkPathScratch relink;
    SolverPostOptTaskPathScratch task_reorder;
    SolverPostOptShortcutPathScratch shortcut;
} SolverPostOptPathScratch;

typedef struct {
    SqPostProcessPathScratch paths;
    Direction route_path[MAX_SINGLE_PATH];
} SqPostProcessPathWorkspace;

typedef union {
    SqPostProcessPathWorkspace sq;
    SolverPostOptPathScratch post;
} PostPathScratchWorkspace;

typedef struct {
    uint8_t turn_shortcut_cost[4][MAP_ROWS][MAP_COLS];
    uint8_t turn_shortcut_parent[4][MAP_ROWS][MAP_COLS];
    SqPostProcessReplayMapScratch replay_map;
    SqPostProcessReplayStateScratch replay_state;
    PostPathScratchWorkspace path_scratch;
    uint16_t scan_pause_prefix[MAX_PATH_LENGTH + 1];
} SolverPostProcessWorkspace;

typedef struct {
    uint16_t prefix_len;
    PackedDirByte prefix[BOMB_STATE_BEAM_PATH_BYTES];
} BombScanMainRouteCandidate;

typedef struct {
    BombStateBeamState current[BOMB_STATE_BEAM_MAX_WIDTH];
    BombStateBeamState children[BOMB_STATE_BEAM_CHILD_CAP];
    bool selected[BOMB_STATE_BEAM_CHILD_CAP];
    SokobanSolver tail_solver;
    PathReplayResult replay_result;
    Direction tail_path[MAX_PATH_LENGTH];
    Direction full_path[MAX_PATH_LENGTH];
    Direction incumbent_path[MAX_PATH_LENGTH];
    Direction saved_path[MAX_PATH_LENGTH];
    BombScanMainRouteCandidate scan_route_candidates[
        SOKOBAN_SCAN_BOMB_MAIN_ROUTE_CANDIDATE_LIMIT
    ];
    uint8_t scan_route_candidate_count;
} BombStateBeamWorkspace;

typedef union {
    BombStateBeamWorkspace bomb_state_beam;
    SolverPostProcessWorkspace post_process;
} SolverPhaseWorkspace;

static SolverPhaseWorkspace g_solver_phase_workspace ALLOC_IN_SDRAM;
static uint8_t g_bomb_state_beam_tail_order[BOMB_STATE_BEAM_MAX_WIDTH];

#define g_bomb_state_beam_current \
    (g_solver_phase_workspace.bomb_state_beam.current)
#define g_bomb_state_beam_children \
    (g_solver_phase_workspace.bomb_state_beam.children)
#define g_bomb_state_beam_selected \
    (g_solver_phase_workspace.bomb_state_beam.selected)
#define g_bomb_state_beam_tail_solver \
    (g_solver_phase_workspace.bomb_state_beam.tail_solver)
#define g_bomb_state_beam_replay_result \
    (g_solver_phase_workspace.bomb_state_beam.replay_result)
#define g_bomb_state_beam_tail_path \
    (g_solver_phase_workspace.bomb_state_beam.tail_path)
#define g_bomb_state_beam_full_path \
    (g_solver_phase_workspace.bomb_state_beam.full_path)
#define g_bomb_state_beam_incumbent_path \
    (g_solver_phase_workspace.bomb_state_beam.incumbent_path)
#define g_bomb_state_beam_saved_path \
    (g_solver_phase_workspace.bomb_state_beam.saved_path)
#define g_bomb_scan_main_route_candidates \
    (g_solver_phase_workspace.bomb_state_beam.scan_route_candidates)
#define g_bomb_scan_main_route_candidate_count \
    (g_solver_phase_workspace.bomb_state_beam.scan_route_candidate_count)

#define g_sq_turn_shortcut_cost \
    (g_solver_phase_workspace.post_process.turn_shortcut_cost)
#define g_sq_turn_shortcut_parent \
    (g_solver_phase_workspace.post_process.turn_shortcut_parent)
#define g_sq_post_replay_map_scratch \
    (g_solver_phase_workspace.post_process.replay_map)
#define g_sq_post_replay_state_scratch \
    (g_solver_phase_workspace.post_process.replay_state)
#define g_post_path_scratch \
    (g_solver_phase_workspace.post_process.path_scratch)
#define g_sq_post_scan_pause_prefix \
    (g_solver_phase_workspace.post_process.scan_pause_prefix)

#define g_sq_post_path_scratch          (g_post_path_scratch.sq.paths)
#define g_sq_post_route_path            (g_post_path_scratch.sq.route_path)
#define g_solver_post_opt_path_scratch  (g_post_path_scratch.post)

_Static_assert(sizeof(SqPostMapFrame) < sizeof(BitboardMap),
               "sq post map frame must stay compact");
_Static_assert(sizeof(SolverPostProcessWorkspace) > sizeof(BombStateBeamWorkspace),
               "post-process scratch must remain the largest solver phase workspace");
_Static_assert(sizeof(SolverPhaseWorkspace) >= sizeof(SolverPostProcessWorkspace) &&
                   sizeof(SolverPhaseWorkspace) <
                       sizeof(SolverPostProcessWorkspace) + _Alignof(SolverPhaseWorkspace),
               "solver phase workspace may add alignment padding only");
_Static_assert(sizeof(SolverPhaseWorkspace) <
                   sizeof(BombStateBeamWorkspace) + sizeof(SolverPostProcessWorkspace),
               "solver phase workspace must reduce static storage");
_Static_assert(sizeof(((SqPostProcessPathWorkspace*)0)->route_path) >=
                   MAX_SINGLE_PATH * sizeof(Direction),
               "sq post route path scratch too small");
_Static_assert(sizeof(((SqPostProcessPathWorkspace*)0)->paths.d1.tail_path) >=
                   MAX_PATH_LENGTH * sizeof(Direction),
               "sq d1 tail path scratch too small");
_Static_assert(sizeof(((SqPostProcessPathWorkspace*)0)->paths.d1.best_candidate) >=
                   MAX_PATH_LENGTH * sizeof(Direction),
               "sq d1 candidate path scratch too small");
_Static_assert(sizeof(((SolverPostOptPathScratch*)0)->relink.relink_path) >=
                   MAX_PATH_LENGTH * sizeof(Direction),
               "post opt relink path scratch too small");
_Static_assert(sizeof(((SolverPostOptPathScratch*)0)->task_reorder.hybrid) >=
                   MAX_PATH_LENGTH * sizeof(Direction),
               "post opt hybrid path scratch too small");
_Static_assert(sizeof(((SolverPostOptPathScratch*)0)->shortcut.new_path) >=
                   MAX_PATH_LENGTH * sizeof(Direction),
               "post opt shortcut path scratch too small");

_Static_assert(BOMB_STATE_BEAM_MAX_WIDTH <= BOMB_STATE_BEAM_CHILD_CAP,
               "bomb state beam child pool must hold one complete layer");

static uint16_t bomb_state_beam_width(uint8_t remaining_boxes, uint8_t remaining_bombs) {
    uint16_t box_term = remaining_boxes < 4u ? remaining_boxes : 4u;
    uint16_t width = (uint16_t)(4u + 2u * remaining_bombs + box_term);
    if (width < 4u) width = 4u;
    if (width > BOMB_STATE_BEAM_MAX_WIDTH) width = BOMB_STATE_BEAM_MAX_WIDTH;
    return width;
}

static void bomb_state_beam_capture_root(
    BombStateBeamState* state,
    const SokobanSolver* solver
) {
    memset(state, 0, sizeof(*state));
    state->bmap = solver->bmap;
    memcpy(state->boxes, solver->boxes, sizeof(state->boxes));
    memcpy(state->targets, solver->targets, sizeof(state->targets));
    memcpy(state->bombs, solver->bombs, sizeof(state->bombs));
    state->player = solver->start_player;
    state->destroyed_walls_mask = solver->destroyed_walls_mask;
    state->num_boxes = solver->num_boxes;
    state->num_targets = solver->num_targets;
    state->num_bombs = solver->num_bombs;
}

static void bomb_state_beam_restore_solver(
    SokobanSolver* solver,
    const BombStateBeamState* state
) {
    solver->bmap = state->bmap;
    memcpy(solver->boxes, state->boxes, sizeof(solver->boxes));
    memcpy(solver->targets, state->targets, sizeof(solver->targets));
    memcpy(solver->bombs, state->bombs, sizeof(solver->bombs));
    solver->start_player = state->player;
    solver->destroyed_walls_mask = state->destroyed_walls_mask;
    solver->num_boxes = state->num_boxes;
    solver->num_targets = state->num_targets;
    solver->num_bombs = state->num_bombs;
    solver->best_steps = 0xFFFF;
    solver->best_path_len = 0;
}

static void bomb_state_beam_store_step(
    PackedDirByte* packed,
    uint16_t step,
    Direction direction
) {
    uint16_t byte_idx = (uint16_t)(step >> 2);
    uint8_t shift = (uint8_t)((step & 0x03u) << 1);
    uint8_t code = solver_direction_code(direction);
    packed[byte_idx] = (uint8_t)(
        (packed[byte_idx] & (uint8_t)~(0x03u << shift)) |
        (uint8_t)(code << shift)
    );
}

static bool bomb_state_beam_append_path(
    BombStateBeamState* child,
    const BombStateBeamState* parent,
    const Direction* path,
    uint16_t path_len
) {
    uint32_t total_len = (uint32_t)parent->prefix_len + path_len;
    if (!child || !parent || !path || total_len >= MAX_PATH_LENGTH) return false;
    *child = *parent;
    for (uint16_t i = 0; i < path_len; i++) {
        if (!direction_is_cardinal(path[i])) return false;
        bomb_state_beam_store_step(child->prefix, (uint16_t)(parent->prefix_len + i), path[i]);
    }
    child->prefix_len = (uint16_t)total_len;
    return true;
}

static void bomb_state_beam_load_path(
    const BombStateBeamState* state,
    Direction* out
) {
    for (uint16_t i = 0; i < state->prefix_len; i++) {
        out[i] = packed_path_step(state->prefix, i);
    }
}

static void bomb_state_beam_record_scan_main_route(
    const SokobanSolver* root_solver,
    const BombStateBeamState* state
) {
    if (!root_solver || !state ||
        !root_solver->is_scanning || root_solver->strict_target_mode ||
        g_bomb_state_beam_full_fallback_active ||
        state->prefix_len == 0 || state->prefix_len >= MAX_PATH_LENGTH) {
        return;
    }

    uint16_t used_bytes = PACKED_PATH_BYTES(state->prefix_len);
    for (uint8_t i = 0; i < g_bomb_scan_main_route_candidate_count; i++) {
        const BombScanMainRouteCandidate* existing =
            &g_bomb_scan_main_route_candidates[i];
        if (existing->prefix_len == state->prefix_len &&
            memcmp(existing->prefix, state->prefix, used_bytes) == 0) {
            return;
        }
    }
    if (g_bomb_scan_main_route_candidate_count >=
        SOKOBAN_SCAN_BOMB_MAIN_ROUTE_CANDIDATE_LIMIT) {
        return;
    }

    BombScanMainRouteCandidate* candidate =
        &g_bomb_scan_main_route_candidates[g_bomb_scan_main_route_candidate_count++];
    candidate->prefix_len = state->prefix_len;
    memset(candidate->prefix, 0, sizeof(candidate->prefix));
    memcpy(candidate->prefix, state->prefix, used_bytes);
}

uint8_t solver_get_scan_bomb_main_route_candidate_count(void) {
    return g_bomb_scan_main_route_candidate_count;
}

bool solver_copy_scan_bomb_main_route_candidate(
    uint8_t index, Direction* out_path, uint16_t* out_len
) {
    if (!out_path || !out_len ||
        index >= g_bomb_scan_main_route_candidate_count) {
        return false;
    }
    const BombScanMainRouteCandidate* candidate =
        &g_bomb_scan_main_route_candidates[index];
    if (candidate->prefix_len == 0 || candidate->prefix_len >= MAX_PATH_LENGTH) {
        return false;
    }
    for (uint16_t i = 0; i < candidate->prefix_len; i++) {
        out_path[i] = packed_path_step(candidate->prefix, i);
    }
    *out_len = candidate->prefix_len;
    return true;
}

static bool bomb_state_beam_entity_equal(Entity a, Entity b) {
    return pos_equal(a.pos, b.pos) && a.id == b.id && a.is_active == b.is_active;
}

static bool bomb_state_beam_same_exact_state(
    const BombStateBeamState* a,
    const BombStateBeamState* b
) {
    if (a->num_boxes != b->num_boxes ||
        a->num_targets != b->num_targets ||
        a->num_bombs != b->num_bombs) return false;
    if (!pos_equal(a->player, b->player)) return false;
    if (a->destroyed_walls_mask != b->destroyed_walls_mask) return false;
    if (memcmp(&a->bmap, &b->bmap, sizeof(BitboardMap)) != 0) return false;
    for (uint8_t i = 0; i < a->num_boxes; i++) {
        if (!bomb_state_beam_entity_equal(a->boxes[i], b->boxes[i])) return false;
    }
    for (uint8_t i = 0; i < a->num_targets; i++) {
        if (!bomb_state_beam_entity_equal(a->targets[i], b->targets[i])) return false;
    }
    for (uint8_t i = 0; i < a->num_bombs; i++) {
        if (!bomb_state_beam_entity_equal(a->bombs[i], b->bombs[i])) return false;
    }
    return true;
}

static bool bomb_state_beam_same_topology(
    const BombStateBeamState* a,
    const BombStateBeamState* b
) {
    for (int y = 0; y < MAP_ROWS; y++) {
        uint16_t a_destructible = (uint16_t)(a->bmap.walls[y] & g_destructible_mask[y]);
        uint16_t b_destructible = (uint16_t)(b->bmap.walls[y] & g_destructible_mask[y]);
        if (a_destructible != b_destructible) return false;
    }
    return true;
}

static int bomb_state_beam_compare(
    const BombStateBeamState* a,
    const BombStateBeamState* b
) {
    if (a->rank != b->rank) return a->rank < b->rank ? -1 : 1;
    if (a->lower_bound != b->lower_bound) return a->lower_bound < b->lower_bound ? -1 : 1;
    if (a->prefix_len != b->prefix_len) return a->prefix_len < b->prefix_len ? -1 : 1;
    if (a->candidate_score != b->candidate_score) {
        return a->candidate_score > b->candidate_score ? -1 : 1;
    }
    if (a->stable_order != b->stable_order) return a->stable_order < b->stable_order ? -1 : 1;
    return 0;
}

static int bomb_state_beam_tail_compare(
    const BombStateBeamState* a,
    const BombStateBeamState* b
) {
    int64_t a_priority =
        (int64_t)a->rank * SOKOBAN_SCAN_BOMB_BEAM_TAIL_RANK_WEIGHT -
        (int64_t)a->candidate_score;
    int64_t b_priority =
        (int64_t)b->rank * SOKOBAN_SCAN_BOMB_BEAM_TAIL_RANK_WEIGHT -
        (int64_t)b->candidate_score;
    if (a_priority != b_priority) return a_priority < b_priority ? -1 : 1;
    return bomb_state_beam_compare(a, b);
}

static void bomb_state_beam_sort(BombStateBeamState* states, uint16_t count) {
    for (uint16_t i = 1; i < count; i++) {
        BombStateBeamState value = states[i];
        uint16_t j = i;
        while (j > 0 && bomb_state_beam_compare(&value, &states[j - 1]) < 0) {
            states[j] = states[j - 1];
            j--;
        }
        states[j] = value;
    }
}

static void bomb_state_beam_consider_child(
    BombStateBeamState* children,
    uint16_t* count,
    uint16_t capacity,
    const BombStateBeamState* candidate
) {
    for (uint16_t i = 0; i < *count; i++) {
        if (!bomb_state_beam_same_exact_state(&children[i], candidate)) continue;
        if (bomb_state_beam_compare(candidate, &children[i]) < 0) children[i] = *candidate;
        return;
    }
    if (*count < capacity) {
        children[(*count)++] = *candidate;
        return;
    }
    if (capacity == 0) return;

    uint16_t worst = 0;
    for (uint16_t i = 1; i < *count; i++) {
        if (bomb_state_beam_compare(&children[worst], &children[i]) < 0) worst = i;
    }
    if (bomb_state_beam_compare(candidate, &children[worst]) < 0) {
        children[worst] = *candidate;
    }
}

static uint16_t bomb_state_beam_select_layer(
    BombStateBeamState* current,
    BombStateBeamState* children,
    uint16_t child_count,
    uint16_t width
) {
    uint16_t next_count = 0;
    memset(g_bomb_state_beam_selected, 0, sizeof(g_bomb_state_beam_selected));
    bomb_state_beam_sort(children, child_count);

    for (uint16_t i = 0; i < child_count && next_count < width; i++) {
        bool topology_seen = false;
        for (uint16_t j = 0; j < next_count; j++) {
            if (bomb_state_beam_same_topology(&current[j], &children[i])) {
                topology_seen = true;
                break;
            }
        }
        if (topology_seen) continue;
        current[next_count++] = children[i];
        g_bomb_state_beam_selected[i] = true;
    }

    for (uint16_t i = 0; i < child_count && next_count < width; i++) {
        if (g_bomb_state_beam_selected[i]) continue;
        current[next_count++] = children[i];
    }
    return next_count;
}

static BombStateBeamMaterializeResult bomb_state_beam_materialize_candidate(
    SokobanSolver* solver,
    const BombStateBeamState* parent,
    int depth,
    const BitboardMap* route_base_map,
    const BombWallCandidate* candidate,
    uint32_t stable_order,
    BombStateBeamState* child
) {
    if (!solver || !parent || !route_base_map || !candidate || !child) {
        return BOMB_STATE_BEAM_SKIP;
    }
    if (candidate->b_idx >= parent->num_bombs) return BOMB_STATE_BEAM_SKIP;

    Position bomb = parent->bombs[candidate->b_idx].pos;
    Position target_wall = candidate->wall_pos;
    uint16_t reach_key = bomb_reach_fail_key(bomb, target_wall);
    uint16_t reach_context = bomb_reach_fail_context(solver);
    if (!bomb_candidate_route_precomputed(candidate) &&
        bomb_reach_fail_cached(parent->universe_hash, reach_key, reach_context)) {
        return BOMB_STATE_BEAM_SKIP;
    }

    BitboardMap* route_map = &g_bomb_branch_scratch[depth].temp_map;
    *route_map = *route_base_map;
    clear_bit(route_map->bombs, bomb.x, bomb.y);

    bool final_stance_possible = false;
    for (int d = 0; d < 4; d++) {
        int pre_x = target_wall.x - DIRECTIONS[d].dx;
        int pre_y = target_wall.y - DIRECTIONS[d].dy;
        int stance_x = pre_x - DIRECTIONS[d].dx;
        int stance_y = pre_y - DIRECTIONS[d].dy;
        if (pre_x <= 0 || pre_x >= MAP_COLS - 1 || pre_y <= 0 || pre_y >= MAP_ROWS - 1) continue;
        if (stance_x <= 0 || stance_x >= MAP_COLS - 1 || stance_y <= 0 || stance_y >= MAP_ROWS - 1) continue;
        if ((map_blocked_row(route_map, pre_y) & bit_mask_at(pre_x)) != 0) continue;
        if ((map_blocked_row(route_map, stance_y) & bit_mask_at(stance_x)) != 0) continue;
        final_stance_possible = true;
        break;
    }
    if (!final_stance_possible) {
        bomb_reach_fail_store(parent->universe_hash, reach_key, reach_context);
        return BOMB_STATE_BEAM_SKIP;
    }

    Direction* route_path = g_bomb_path_pool[depth];
    uint16_t route_len = 0xFFFF;
    if (bomb_candidate_route_precomputed(candidate)) {
        if (candidate->route_len == 0xFFFF || candidate->route_len > MAX_SINGLE_PATH) {
            return BOMB_STATE_BEAM_SKIP;
        }
        route_len = candidate->route_len;
        packed_path_load_to_direction(
            g_bomb_reach_all_path_pool[depth][candidate->route_slot],
            route_path,
            route_len
        );
    } else {
        if (!bomb_route_initial_push_reachable(route_map, parent->player, bomb, target_wall)) {
            return BOMB_STATE_BEAM_SKIP;
        }
        int dummy_idx = -1;
        uint16_t saved_astar_max_steps = g_astar_max_steps;
        hash_table_clear();
        g_astar_max_steps = 100u;
        bool routed = astar_solve_with_mask(
            solver->heap,
            solver->closed_list,
            route_map,
            parent->player,
            bomb,
            &target_wall,
            1,
            &dummy_idx,
            MASK_WALL | MASK_BOMB | MASK_BOX,
            route_path,
            &route_len,
            ASTAR_NO_MACRO_DEPTH,
            ROUTE_BOMB_ATTACK
        );
        g_astar_max_steps = saved_astar_max_steps;
        if (!routed) {
            bomb_reach_fail_store(parent->universe_hash, reach_key, reach_context);
            return BOMB_STATE_BEAM_SKIP;
        }
    }

    if ((uint32_t)parent->prefix_len + route_len >= MAX_PATH_LENGTH) {
        return BOMB_STATE_BEAM_SKIP;
    }

    PathReplayOptions replay_options = path_replay_default_options();
    replay_options.mode = PATH_REPLAY_STRICT_VALIDATE;
    if (!path_replay_run(
            solver,
            &parent->bmap,
            parent->player,
            parent->boxes,
            parent->num_boxes,
            parent->bombs,
            parent->num_bombs,
            route_path,
            route_len,
            &replay_options,
            &g_bomb_state_beam_replay_result) ||
        !g_bomb_state_beam_replay_result.ok ||
        g_bomb_state_beam_replay_result.consumed_len != route_len) {
        return BOMB_STATE_BEAM_SKIP;
    }

    PathReplayState* final_state = &g_bomb_state_beam_replay_result.final_state;
    if (final_state->bomb_count + 1 != parent->num_bombs ||
        get_bit(final_state->map.walls, target_wall.x, target_wall.y)) {
        return BOMB_STATE_BEAM_SKIP;
    }

    if (!bomb_state_beam_append_path(child, parent, route_path, route_len)) {
        return BOMB_STATE_BEAM_SKIP;
    }
    child->bmap = final_state->map;
    uint16_t deadlock_clear_mask = g_O1_deadlock_clear[target_wall.x];
    for (int y = target_wall.y - 2; y <= target_wall.y + 2; y++) {
        if (y >= 0 && y < MAP_ROWS) child->bmap.deadlocks[y] &= deadlock_clear_mask;
    }
    child->player = final_state->player;

    uint8_t box_count = 0;
    memset(child->boxes, 0, sizeof(child->boxes));
    for (uint8_t i = 0; i < parent->num_boxes; i++) {
        Position pos = final_state->boxes[i];
        if (pos.x == 0xFF && pos.y == 0xFF) continue;
        if (!is_in_bounds(pos.x, pos.y) ||
            !get_bit(final_state->map.boxes, pos.x, pos.y) ||
            box_count >= MAX_BOXES) {
            return BOMB_STATE_BEAM_SKIP;
        }
        child->boxes[box_count] = parent->boxes[i];
        child->boxes[box_count].pos = pos;
        child->boxes[box_count].is_active = true;
        box_count++;
    }

    uint8_t target_count = 0;
    memset(child->targets, 0, sizeof(child->targets));
    for (uint8_t i = 0; i < parent->num_targets; i++) {
        Position pos = parent->targets[i].pos;
        if (!is_in_bounds(pos.x, pos.y)) return BOMB_STATE_BEAM_SKIP;
        if (!get_bit(final_state->map.targets, pos.x, pos.y)) continue;
        if (target_count >= MAX_TARGETS) return BOMB_STATE_BEAM_SKIP;
        child->targets[target_count] = parent->targets[i];
        child->targets[target_count].is_active = true;
        target_count++;
    }

    int map_box_count = 0;
    int map_target_count = 0;
    for (int y = 0; y < MAP_ROWS; y++) {
        map_box_count += __builtin_popcount((unsigned int)final_state->map.boxes[y]);
        map_target_count += __builtin_popcount((unsigned int)final_state->map.targets[y]);
    }
    if (map_box_count != box_count ||
        map_target_count != target_count ||
        box_count != target_count) {
        return BOMB_STATE_BEAM_SKIP;
    }
    child->num_boxes = box_count;
    child->num_targets = target_count;

    child->num_bombs = (uint8_t)final_state->bomb_count;
    memset(child->bombs, 0, sizeof(child->bombs));
    for (uint8_t i = 0; i < child->num_bombs; i++) child->bombs[i] = final_state->bombs[i];
    child->candidate_score = candidate->score;
    child->stable_order = stable_order;

    bomb_state_beam_restore_solver(solver, child);
    child->universe_hash = compute_universe_hash(solver);
    child->lower_bound = remaining_solution_lower_bound(solver, child->player);
    child->rank = child->lower_bound == 0xFFFF
        ? UINT32_MAX
        : (uint32_t)child->prefix_len + child->lower_bound;
    return BOMB_STATE_BEAM_OK;
}

static BombStateBeamTailResult bomb_state_beam_try_tail(
    SokobanSolver* root_solver,
    const BombStateBeamState* root,
    const BombStateBeamState* state,
    uint16_t* incumbent_len
) {
    g_bomb_state_beam_tail_solver = *root_solver;
    bomb_state_beam_restore_solver(&g_bomb_state_beam_tail_solver, state);
    g_bomb_state_beam_tail_solver.best_path = g_bomb_state_beam_tail_path;

    bool saved_tail_only = g_bomb_state_beam_tail_only;
    bool saved_bounded_refinement = g_assignment_bounded_refinement;
    uint16_t saved_assignment_exclusive_upper_bound =
        g_assignment_exclusive_upper_bound;
    uint16_t saved_assignment_beam_width = g_assignment_beam_active_width;
    bool saved_refinement_disabled = g_assignment_refinement_disabled;
    bool saved_first_solution_only = g_dfs_first_solution_only;
    bool saved_bomb_first_solution_only = g_bomb_seed_first_solution_only;
    bool saved_push_reach_filter = g_enable_push_reach_filter;
    bool saved_allow_macro = g_allow_macro_evacuation;
    bool saved_allow_super = g_allow_super_evacuation;
    uint16_t saved_macro_depth = g_current_macro_depth;
    uint16_t saved_super_depth = g_current_super_depth;
    uint16_t saved_pocket_depth = g_current_pocket_depth;
    uint16_t saved_astar_max_steps = g_astar_max_steps;

    g_assignment_exclusive_upper_bound = UINT16_MAX;
    if (root_solver->is_scanning && !root_solver->strict_target_mode &&
        !g_bomb_state_beam_full_fallback_active &&
        *incumbent_len != UINT16_MAX && state->prefix_len < *incumbent_len) {
        g_assignment_exclusive_upper_bound =
            (uint16_t)(*incumbent_len - state->prefix_len);
    }
    g_bomb_state_beam_tail_only = true;
    g_assignment_bounded_refinement = true;
    if (root_solver->is_scanning &&
        !root_solver->strict_target_mode &&
        !g_bomb_state_beam_full_fallback_active) {
        g_assignment_beam_active_width = SOKOBAN_SCAN_BOMB_ASSIGNMENT_BEAM_WIDTH;
    }
    g_assignment_refinement_disabled = false;
    g_dfs_first_solution_only = false;
    g_bomb_seed_first_solution_only = false;
    g_enable_push_reach_filter = false;
    g_allow_macro_evacuation = false;
    g_allow_super_evacuation = false;
    g_current_macro_depth = 0;
    g_current_super_depth = 0;
    g_current_pocket_depth = 0;
    g_astar_max_steps = 0xFFFF;
    clear_dfs_transposition_table();
    solve_attempt_budget_reset();

    bool tail_ok = sokoban_solve_internal(
        &g_bomb_state_beam_tail_solver,
        0,
        state->universe_hash
    );
    bool exhausted = g_solve_budget_exhausted;

    g_bomb_state_beam_tail_only = saved_tail_only;
    g_assignment_bounded_refinement = saved_bounded_refinement;
    g_assignment_exclusive_upper_bound =
        saved_assignment_exclusive_upper_bound;
    g_assignment_beam_active_width = saved_assignment_beam_width;
    g_assignment_refinement_disabled = saved_refinement_disabled;
    g_dfs_first_solution_only = saved_first_solution_only;
    g_bomb_seed_first_solution_only = saved_bomb_first_solution_only;
    g_enable_push_reach_filter = saved_push_reach_filter;
    g_allow_macro_evacuation = saved_allow_macro;
    g_allow_super_evacuation = saved_allow_super;
    g_current_macro_depth = saved_macro_depth;
    g_current_super_depth = saved_super_depth;
    g_current_pocket_depth = saved_pocket_depth;
    g_astar_max_steps = saved_astar_max_steps;

    if (exhausted) return BOMB_STATE_BEAM_TAIL_INCOMPLETE;
    if (!tail_ok) return BOMB_STATE_BEAM_TAIL_NONE;
    if (g_bomb_state_beam_tail_solver.best_steps == 0xFFFF ||
        g_bomb_state_beam_tail_solver.best_path_len >= MAX_PATH_LENGTH) {
        return BOMB_STATE_BEAM_TAIL_NONE;
    }

    uint32_t total_len =
        (uint32_t)state->prefix_len + g_bomb_state_beam_tail_solver.best_path_len;
    if (total_len >= MAX_PATH_LENGTH) return BOMB_STATE_BEAM_TAIL_NONE;
    bomb_state_beam_load_path(state, g_bomb_state_beam_full_path);
    memcpy(
        &g_bomb_state_beam_full_path[state->prefix_len],
        g_bomb_state_beam_tail_solver.best_path,
        g_bomb_state_beam_tail_solver.best_path_len * sizeof(Direction)
    );

    bomb_state_beam_restore_solver(root_solver, root);
    if (!sq_verify_full_path(
            root_solver,
            &root->bmap,
            root->player,
            root->boxes,
            root->num_boxes,
            root->bombs,
            root->num_bombs,
            g_bomb_state_beam_full_path,
            (uint16_t)total_len) ||
        !scan_audit_path_no_absorb_before_blast(
            root_solver,
            g_bomb_state_beam_full_path,
            (uint16_t)total_len)) {
        return BOMB_STATE_BEAM_TAIL_NONE;
    }

    bomb_state_beam_record_scan_main_route(root_solver, state);

    if ((uint16_t)total_len < *incumbent_len) {
        *incumbent_len = (uint16_t)total_len;
        memcpy(
            g_bomb_state_beam_incumbent_path,
            g_bomb_state_beam_full_path,
            total_len * sizeof(Direction)
        );
    }
    return BOMB_STATE_BEAM_TAIL_FOUND;
}

static bool bomb_state_beam_is_eligible(const SokobanSolver* solver) {
    if (!solver || g_sandbox_mode ||
        solver->num_bombs == 0 || solver->num_bombs > MAX_BOMBS ||
        solver->num_boxes == 0 || solver->num_boxes > MAX_BOXES ||
        solver->num_boxes != solver->num_targets) {
        return false;
    }
    /* When all targets remain in one static component and a bomb opens a
       component bridge, the existing attempt plan is faster and near-baseline. */
    if (solver->is_scanning && !solver->strict_target_mode &&
        solver->num_boxes == 3u && g_topology_features_valid &&
        g_topology_features.component_count > 1u &&
        g_topology_features.target_component_count == 1u &&
        g_topology_features.bomb_bridge_count > 0u) {
        return false;
    }
    return true;
}

static bool bomb_state_beam_solve_impl(SokobanSolver* solver) {
    if (!bomb_state_beam_is_eligible(solver)) return false;

    BombStateBeamState root;
    bomb_state_beam_capture_root(&root, solver);
    root.universe_hash = compute_universe_hash(solver);
    root.lower_bound = remaining_solution_lower_bound(solver, root.player);
    root.rank = root.lower_bound;

    uint16_t saved_best_steps = solver->best_steps;
    uint16_t saved_best_path_len = solver->best_path_len;
    if (solver->best_path && saved_best_path_len < MAX_PATH_LENGTH) {
        memcpy(
            g_bomb_state_beam_saved_path,
            solver->best_path,
            saved_best_path_len * sizeof(Direction)
        );
    }
    bool saved_push_reach_filter = g_enable_push_reach_filter;
    uint16_t incumbent_len = 0xFFFF;
    uint32_t stable_order = 1u;
    bool incomplete = false;

    g_bomb_state_beam_current[0] = root;
    uint16_t current_count = 1u;
    BombStateBeamTailResult root_tail = bomb_state_beam_try_tail(
        solver,
        &root,
        &g_bomb_state_beam_current[0],
        &incumbent_len
    );
    if (root_tail == BOMB_STATE_BEAM_TAIL_INCOMPLETE ||
        root_tail == BOMB_STATE_BEAM_TAIL_FATAL) {
        incomplete = true;
    }

    bool bounded_scan_tails =
        solver->is_scanning &&
        !solver->strict_target_mode &&
        !g_bomb_state_beam_full_fallback_active;
    for (int layer = 0;
         !incomplete && current_count > 0 && layer < root.num_bombs;
         layer++) {
        uint8_t remaining_after = g_bomb_state_beam_current[0].num_bombs > 0
            ? (uint8_t)(g_bomb_state_beam_current[0].num_bombs - 1u)
            : 0u;
        uint8_t remaining_boxes = 0;
        for (uint16_t i = 0; i < current_count; i++) {
            if (g_bomb_state_beam_current[i].num_boxes > remaining_boxes) {
                remaining_boxes = g_bomb_state_beam_current[i].num_boxes;
            }
        }
        uint16_t next_width = bomb_state_beam_width(remaining_boxes, remaining_after);
        uint16_t child_capacity = (uint16_t)(next_width * 2u);
        if (child_capacity > BOMB_STATE_BEAM_CHILD_CAP) {
            child_capacity = BOMB_STATE_BEAM_CHILD_CAP;
        }
        uint16_t child_count = 0;

        for (uint16_t parent_idx = 0;
             !incomplete && parent_idx < current_count;
             parent_idx++) {
            BombStateBeamState* parent = &g_bomb_state_beam_current[parent_idx];
            bomb_state_beam_restore_solver(solver, parent);

            int scratch_depth = (int)root.num_bombs - (int)parent->num_bombs;
            if (scratch_depth < 0 || scratch_depth >= MAX_BOMBS) continue;

            bool parent_push_reach_filter = g_enable_push_reach_filter;
            g_enable_push_reach_filter = false;
            BombCandidatePlan* plan = &g_bomb_candidate_plan_scratch[scratch_depth];
            bool planned = bomb_candidate_plan_prepare(
                solver,
                scratch_depth,
                remaining_box_assignment_lower_bound(solver),
                0xFFFF,
                plan
            );
            g_enable_push_reach_filter = parent_push_reach_filter;
            if (!planned) continue;

            uint16_t parents_left = (uint16_t)(current_count - parent_idx);
            uint16_t slots_left = (uint16_t)(child_capacity - child_count);
            uint16_t quota = parents_left == 0 ? 0 : (uint16_t)(slots_left / parents_left);
            if (parents_left != 0 && (slots_left % parents_left) != 0) quota++;
            if (quota == 0 && plan->num_candidates > 0) quota = 1;
            uint16_t materialized = 0;

            for (int candidate_idx = 0;
                 !incomplete && candidate_idx < plan->num_candidates && materialized < quota;
                 candidate_idx++) {
                bomb_state_beam_restore_solver(solver, parent);
                BombStateBeamState child;
                BombStateBeamMaterializeResult result = bomb_state_beam_materialize_candidate(
                    solver,
                    parent,
                    scratch_depth,
                    plan->route_base_map,
                    &plan->top_candidates[candidate_idx],
                    stable_order++,
                    &child
                );
                if (result == BOMB_STATE_BEAM_FATAL) {
                    incomplete = true;
                    break;
                }
                if (result != BOMB_STATE_BEAM_OK) continue;
                bomb_state_beam_consider_child(
                    g_bomb_state_beam_children,
                    &child_count,
                    child_capacity,
                    &child
                );
                materialized++;
            }
        }

        if (incomplete || child_count == 0) break;
        current_count = bomb_state_beam_select_layer(
            g_bomb_state_beam_current,
            g_bomb_state_beam_children,
            child_count,
            next_width
        );

        uint16_t tail_count = current_count;
        for (uint16_t i = 0; i < current_count; i++) {
            g_bomb_state_beam_tail_order[i] = (uint8_t)i;
        }
        bool score_order_tails =
            bounded_scan_tails &&
            root.num_boxes >= 4u &&
            root.num_boxes <= MAX_BOXES &&
            root.num_bombs <= 2u;
        if (score_order_tails) {
            uint8_t score_head = 0;
            for (uint16_t i = 1; i < current_count; i++) {
                if (bomb_state_beam_tail_compare(
                        &g_bomb_state_beam_current[i],
                        &g_bomb_state_beam_current[score_head]) < 0) {
                    score_head = (uint8_t)i;
                }
            }
            if (score_head != 0) {
                g_bomb_state_beam_tail_order[0] = score_head;
                g_bomb_state_beam_tail_order[1] = 0;
            }
        }
        if (bounded_scan_tails &&
            tail_count > SOKOBAN_SCAN_BOMB_BEAM_TAIL_LIMIT) {
            tail_count = SOKOBAN_SCAN_BOMB_BEAM_TAIL_LIMIT;
        }
        for (uint16_t i = 0; !incomplete && i < tail_count; i++) {
            uint8_t state_idx = g_bomb_state_beam_tail_order[i];
            BombStateBeamTailResult tail = bomb_state_beam_try_tail(
                solver,
                &root,
                &g_bomb_state_beam_current[state_idx],
                &incumbent_len
            );
            if (tail == BOMB_STATE_BEAM_TAIL_INCOMPLETE ||
                tail == BOMB_STATE_BEAM_TAIL_FATAL) {
                incomplete = true;
            }
        }
    }

    bomb_state_beam_restore_solver(solver, &root);
    g_bomb_state_beam_tail_only = false;
    g_enable_push_reach_filter = saved_push_reach_filter;
    clear_transposition_table();
    bomb_reach_caches_reset();
    memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));
    solve_attempt_budget_reset();
    prepare_dfs_temp_maps(solver);

    if (!incomplete && incumbent_len != 0xFFFF) {
        solver->best_steps = incumbent_len;
        solver->best_path_len = incumbent_len;
        memcpy(
            solver->best_path,
            g_bomb_state_beam_incumbent_path,
            incumbent_len * sizeof(Direction)
        );
        g_bomb_seed_solution_committed = false;
        return true;
    }

    solver->best_steps = saved_best_steps;
    solver->best_path_len = saved_best_path_len;
    if (solver->best_path && saved_best_path_len < MAX_PATH_LENGTH) {
        memcpy(
            solver->best_path,
            g_bomb_state_beam_saved_path,
            saved_best_path_len * sizeof(Direction)
        );
    }
    g_bomb_seed_solution_committed = false;
    return false;
}

static bool bomb_state_beam_solve(SokobanSolver* solver) {
    if (!bomb_state_beam_is_eligible(solver)) return false;

    bool saved_full_fallback_active = g_bomb_state_beam_full_fallback_active;
    bool bounded_scan_tails =
        solver->is_scanning &&
        !solver->strict_target_mode &&
        !saved_full_fallback_active;
    bool result = bomb_state_beam_solve_impl(solver);
    if (!result && bounded_scan_tails) {
        g_bomb_scan_main_route_candidate_count = 0;
        /* A bounded miss is not an unsolvable proof. Re-run the original full beam once. */
        g_bomb_state_beam_full_fallback_active = true;
        result = bomb_state_beam_solve_impl(solver);
    }
    g_bomb_state_beam_full_fallback_active = saved_full_fallback_active;
    return result;
}
// ============================================================================
// ============================================================================

static void sq_store_map_frame(SqPostMapFrame* dst, const BitboardMap* src) {
    memcpy(dst->walls, src->walls, sizeof(dst->walls));
    memcpy(dst->targets, src->targets, sizeof(dst->targets));
    memcpy(dst->bombs, src->bombs, sizeof(dst->bombs));
    memcpy(dst->boxes, src->boxes, sizeof(dst->boxes));
    memcpy(dst->h_tunnels, src->h_tunnels, sizeof(dst->h_tunnels));
    memcpy(dst->v_tunnels, src->v_tunnels, sizeof(dst->v_tunnels));
}

static void sq_restore_map_frame(BitboardMap* dst, const SqPostMapFrame* src, const uint16_t deadlocks[MAP_ROWS]) {
    memcpy(dst->walls, src->walls, sizeof(dst->walls));
    memcpy(dst->targets, src->targets, sizeof(dst->targets));
    memcpy(dst->bombs, src->bombs, sizeof(dst->bombs));
    memcpy(dst->boxes, src->boxes, sizeof(dst->boxes));
    if (deadlocks) memcpy(dst->deadlocks, deadlocks, sizeof(dst->deadlocks));
    else memset(dst->deadlocks, 0, sizeof(dst->deadlocks));
    memcpy(dst->h_tunnels, src->h_tunnels, sizeof(dst->h_tunnels));
    memcpy(dst->v_tunnels, src->v_tunnels, sizeof(dst->v_tunnels));
}

static bool sq_frames_are_identical(const SqPostMapFrame* m1, const SqPostMapFrame* m2) {
    for (int i = 0; i < MAP_ROWS; i++) {
        if (m1->boxes[i] != m2->boxes[i]) return false;
        if (m1->bombs[i] != m2->bombs[i]) return false;
        if (m1->walls[i] != m2->walls[i]) return false;
    }
    return true;
}

static bool sq_prev_cardinal_dir(const Direction* path, int start_idx, Direction* out_dir) {
    for (int i = start_idx - 1; i >= 0; i--) {
        if (direction_is_cardinal(path[i])) {
            *out_dir = path[i];
            return true;
        }
    }
    return false;
}

static bool sq_next_cardinal_dir(const Direction* path, int start_idx, int path_len, Direction* out_dir) {
    for (int i = start_idx; i < path_len; i++) {
        if (direction_is_cardinal(path[i])) {
            *out_dir = path[i];
            return true;
        }
    }
    return false;
}

static int sq_count_turns_with_boundaries(
    const Direction* path,
    int path_len,
    bool has_prev,
    Direction prev_dir,
    bool has_next,
    Direction next_dir
) {
    int turns = 0;
    bool has_last = has_prev;
    Direction last_dir = prev_dir;

    for (int i = 0; i < path_len; i++) {
        Direction d = path[i];
        if (!direction_is_cardinal(d)) continue;
        if (has_last && !direction_equal(last_dir, d)) turns++;
        last_dir = d;
        has_last = true;
    }

    if (has_next && has_last && !direction_equal(last_dir, next_dir)) turns++;
    return turns;
}

static inline bool sq_walk_cell_free(const BitboardMap* map, Position p) {
    if (p.x <= 0 || p.x >= MAP_COLS - 1 || p.y <= 0 || p.y >= MAP_ROWS - 1) return false;
    return !map_is_obstructed(map, p.x, p.y);
}

static bool sq_emit_axis_walk(
    const BitboardMap* map,
    Position start,
    Position target,
    bool horizontal_first,
    Direction* out_path,
    uint16_t* out_len
) {
    Position p = start;
    uint16_t len = 0;

    for (int phase = 0; phase < 2; phase++) {
        bool horizontal = horizontal_first ? (phase == 0) : (phase != 0);
        if (horizontal) {
            int step = (target.x > p.x) ? 1 : ((target.x < p.x) ? -1 : 0);
            while (p.x != target.x) {
                Direction d = {(int8_t)step, 0};
                Position next = {(uint8_t)(p.x + step), p.y};
                if (len >= MAX_SINGLE_PATH || !sq_walk_cell_free(map, next)) return false;
                out_path[len++] = d;
                p = next;
            }
        } else {
            int step = (target.y > p.y) ? 1 : ((target.y < p.y) ? -1 : 0);
            while (p.y != target.y) {
                Direction d = {0, (int8_t)step};
                Position next = {p.x, (uint8_t)(p.y + step)};
                if (len >= MAX_SINGLE_PATH || !sq_walk_cell_free(map, next)) return false;
                out_path[len++] = d;
                p = next;
            }
        }
    }

    if (!pos_equal(p, target)) return false;
    *out_len = len;
    return true;
}

FAST_OCRAM_FUNC static bool sq_find_equal_len_turn_shortcut(
    const BitboardMap* map,
    Position start,
    Position target,
    const Direction* original_segment,
    int original_segment_len,
    bool has_prev,
    Direction prev_dir,
    bool has_next,
    Direction next_dir,
    Direction* out_path,
    uint16_t* out_len
) {
    if (!map || !original_segment || !out_path || !out_len) return false;
    if (original_segment_len <= 0 || original_segment_len > MAX_SINGLE_PATH) return false;
    

    int manhattan = abs((int)start.x - (int)target.x) + abs((int)start.y - (int)target.y);
    if (manhattan != original_segment_len) return false;

    int old_turns = sq_count_turns_with_boundaries(original_segment, original_segment_len, has_prev, prev_dir, has_next, next_dir);
    int best_turns = old_turns;
    uint16_t best_len = 0;
    bool found = false;

    static Direction candidate[MAX_SINGLE_PATH];
    for (int option = 0; option < 2; option++) {
        uint16_t candidate_len = 0;
        if (!sq_emit_axis_walk(map, start, target, option == 0, candidate, &candidate_len)) continue;
        if ((int)candidate_len != original_segment_len) continue;
        int new_turns = sq_count_turns_with_boundaries(candidate, candidate_len, has_prev, prev_dir, has_next, next_dir);
        if (new_turns < best_turns) {
            best_turns = new_turns;
            best_len = candidate_len;
            memcpy(out_path, candidate, best_len * sizeof(Direction));
            found = true;
        }
    }

    int dirs[2];
    int dir_count = 0;
    if (target.y < start.y) dirs[dir_count++] = 0;
    else if (target.y > start.y) dirs[dir_count++] = 1;
    if (target.x < start.x) dirs[dir_count++] = 2;
    else if (target.x > start.x) dirs[dir_count++] = 3;
    if (dir_count == 0) return found;

    memset(g_sq_turn_shortcut_cost, 0xFF, sizeof(g_sq_turn_shortcut_cost));
    memset(g_sq_turn_shortcut_parent, 0xFF, sizeof(g_sq_turn_shortcut_parent));

    for (int i = 0; i < dir_count; i++) {
        int d = dirs[i];
        Position next = {
            (uint8_t)(start.x + DIRECTIONS[d].dx),
            (uint8_t)(start.y + DIRECTIONS[d].dy)
        };
        if (!sq_walk_cell_free(map, next)) continue;
        int on_shortest_rect = abs((int)next.x - (int)start.x) + abs((int)next.y - (int)start.y) +
                               abs((int)next.x - (int)target.x) + abs((int)next.y - (int)target.y);
        if (on_shortest_rect != manhattan) continue;
        uint8_t cost = (has_prev && !direction_equal(prev_dir, DIRECTIONS[d])) ? 1u : 0u;
        if (cost < g_sq_turn_shortcut_cost[d][next.y][next.x]) {
            g_sq_turn_shortcut_cost[d][next.y][next.x] = cost;
            g_sq_turn_shortcut_parent[d][next.y][next.x] = 4u;
        }
    }

    for (int step = 1; step < original_segment_len; step++) {
        for (int y = 1; y < MAP_ROWS - 1; y++) {
            for (int x = 1; x < MAP_COLS - 1; x++) {
                int dist_from_start = abs(x - (int)start.x) + abs(y - (int)start.y);
                if (dist_from_start != step) continue;

                for (int cur_dir = 0; cur_dir < 4; cur_dir++) {
                    uint8_t base_cost = g_sq_turn_shortcut_cost[cur_dir][y][x];
                    if (base_cost == 0xFFu) continue;

                    for (int i = 0; i < dir_count; i++) {
                        int next_dir_code = dirs[i];
                        int nx = x + DIRECTIONS[next_dir_code].dx;
                        int ny = y + DIRECTIONS[next_dir_code].dy;
                        if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;

                        Position next = {(uint8_t)nx, (uint8_t)ny};
                        if (!sq_walk_cell_free(map, next)) continue;
                        int on_shortest_rect = abs(nx - (int)start.x) + abs(ny - (int)start.y) +
                                               abs(nx - (int)target.x) + abs(ny - (int)target.y);
                        if (on_shortest_rect != manhattan) continue;

                        uint8_t add_turn = (cur_dir != next_dir_code) ? 1u : 0u;
                        uint8_t new_cost = (uint8_t)(base_cost + add_turn);
                        if (new_cost < g_sq_turn_shortcut_cost[next_dir_code][ny][nx]) {
                            g_sq_turn_shortcut_cost[next_dir_code][ny][nx] = new_cost;
                            g_sq_turn_shortcut_parent[next_dir_code][ny][nx] = (uint8_t)cur_dir;
                        }
                    }
                }
            }
        }
    }

    int best_dir = -1;
    for (int d = 0; d < 4; d++) {
        uint8_t cost = g_sq_turn_shortcut_cost[d][target.y][target.x];
        if (cost == 0xFFu) continue;
        int final_turns = cost + ((has_next && !direction_equal(DIRECTIONS[d], next_dir)) ? 1 : 0);
        if (final_turns < best_turns) {
            best_turns = final_turns;
            best_dir = d;
        }
    }

    if (best_dir >= 0) {
        Position p = target;
        int dir = best_dir;
        int write_idx = original_segment_len - 1;
        bool ok = true;
        while (write_idx >= 0) {
            if (dir < 0 || dir >= 4) { ok = false; break; }
            out_path[write_idx] = DIRECTIONS[dir];
            uint8_t parent = g_sq_turn_shortcut_parent[dir][p.y][p.x];
            p.x = (uint8_t)(p.x - DIRECTIONS[dir].dx);
            p.y = (uint8_t)(p.y - DIRECTIONS[dir].dy);
            write_idx--;
            if (parent == 4u) {
                if (write_idx >= 0) ok = false;
                break;
            }
            if (parent > 3u) { ok = false; break; }
            dir = parent;
        }
        if (ok && write_idx < 0 && pos_equal(p, start)) {
            best_len = (uint16_t)original_segment_len;
            found = true;
        }
    }

    if (!found) return false;
    *out_len = best_len;
    return true;
}


FAST_OCRAM_FUNC static bool sq_apply_path_steps(
    const SokobanSolver* solver,
    BitboardMap* sim_map,
    Position* curr_p,
    Position* sim_boxes,
    Entity* sim_bombs,
    int* bomb_count,
    const Direction* path,
    uint16_t path_len
) {
    if (!solver || !sim_map || !curr_p || !sim_boxes || !sim_bombs || !bomb_count || !path) return false;

    Entity replay_boxes[MAX_BOXES];
    for (int i = 0; i < MAX_BOXES; i++) {
        replay_boxes[i].pos = (Position){0xFF, 0xFF};
        replay_boxes[i].id = -1;
        replay_boxes[i].is_active = false;
    }
    for (int i = 0; i < solver->num_boxes && i < MAX_BOXES; i++) {
        replay_boxes[i].pos = sim_boxes[i];
        replay_boxes[i].id = -1;
        replay_boxes[i].is_active = true;
    }

    PathReplayOptions replay_options = {0};
    replay_options.mode = PATH_REPLAY_STRICT_VALIDATE;
    PathReplayResult replay_result;
    if (!path_replay_run(
            solver,
            sim_map,
            *curr_p,
            replay_boxes,
            solver->num_boxes,
            sim_bombs,
            *bomb_count,
            path,
            path_len,
            &replay_options,
            &replay_result) ||
        !replay_result.ok) {
        return false;
    }

    *sim_map = replay_result.final_state.map;
    *curr_p = replay_result.final_state.player;
    for (int i = 0; i < MAX_BOXES; i++) sim_boxes[i] = replay_result.final_state.boxes[i];
    for (int i = 0; i < MAX_BOMBS; i++) sim_bombs[i] = replay_result.final_state.bombs[i];
    *bomb_count = replay_result.final_state.bomb_count;
    return true;
}

FAST_OCRAM_FUNC static bool sq_verify_full_path(
    const SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs,
    const Direction* path,
    uint16_t path_len
) {
    if (!solver || !initial_map || !path || path_len >= MAX_PATH_LENGTH) return false;

    BitboardMap sim_map = *initial_map;
    Position curr_p = initial_player;
    Position sim_boxes[MAX_BOXES];
    Entity sim_bombs[MAX_BOMBS];

    for (int i = 0; i < MAX_BOXES; i++) sim_boxes[i] = (Position){0xFF, 0xFF};
    int box_count = initial_num_boxes;
    if (box_count < 0) box_count = 0;
    if (box_count > MAX_BOXES) box_count = MAX_BOXES;
    if (!initial_boxes) box_count = 0;
    for (int i = 0; i < box_count; i++) sim_boxes[i] = initial_boxes[i].pos;

    int bomb_count = initial_num_bombs;
    if (bomb_count < 0) bomb_count = 0;
    if (bomb_count > MAX_BOMBS) bomb_count = MAX_BOMBS;
    if (!initial_bombs) bomb_count = 0;
    for (int i = 0; i < bomb_count; i++) sim_bombs[i] = initial_bombs[i];

    if (!sq_apply_path_steps(solver, &sim_map, &curr_p, sim_boxes, sim_bombs, &bomb_count, path, path_len)) return false;

    for (int t = 0; t < solver->num_targets; t++) {
        Position tp = solver->targets[t].pos;
        if (get_bit(sim_map.targets, tp.x, tp.y)) return false;
    }
    return true;
}
typedef enum {
    SQ_TASK_BOX_GOAL,
    SQ_TASK_BOMB_DETONATE
} SqTaskType;

typedef struct {
    SqTaskType type;
    int start_t;
    int end_t;
    Position entity_start;
    Position target_pos;
    int box_idx;
    int target_idx;
} SqPathTask;

#define SQ_D1_MAX_TASKS SOKOBAN_PARAM_SQ_D1_MAX_TASKS
#define SQ_D1_MAX_TRIES 10

typedef struct {
    SqPostMapFrame* map_history;
    Position* pos_history;
    Position (*box_history)[MAX_BOXES];
    uint16_t* scan_pause_prefix;
    const Direction* path;
    uint16_t path_len;
} SqPostPathContext;

typedef struct {
    int start_t;
    int end_t;
    Position entity_start_pos;
} SqPostMacroTask;

typedef struct {
    int task_idx;
    int offset;
    int len;
} SqPostTaskSegment;

#define SQ_POST_MAX_MACRO_TASKS SOKOBAN_PARAM_MAX_MACRO_TASKS

static int sq_find_target_idx(const SokobanSolver* solver, Position pos) {
    for (int t = 0; t < solver->num_targets; t++) {
        if (pos_equal(solver->targets[t].pos, pos)) return t;
    }
    return -1;
}

static int sq_restore_bomb_entities_from_map(Entity out[MAX_BOMBS], const BitboardMap* map) {
    int count = 0;
    if (map) {
        for (int y = 0; y < MAP_ROWS && count < MAX_BOMBS; y++) {
            uint16_t row = map->bombs[y];
            while (row != 0 && count < MAX_BOMBS) {
                int x = __builtin_ctz((unsigned int)row);
                row &= (uint16_t)(row - 1u);
                out[count].pos = (Position){(uint8_t)x, (uint8_t)y};
                out[count].id = (int8_t)count;
                out[count].is_active = true;
                count++;
            }
        }
    }
    for (int i = count; i < MAX_BOMBS; i++) {
        out[i].pos = (Position){0xFF, 0xFF};
        out[i].id = -1;
        out[i].is_active = false;
    }
    return count;
}

FAST_OCRAM_FUNC static bool sq_build_history(
    const SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs,
    const Direction* path,
    uint16_t path_len,
    SqPostMapFrame* map_history,
    Position* pos_history,
    Position box_history[][MAX_BOXES]
) {
    if (!solver || !initial_map || !path || !map_history || !pos_history || !box_history) return false;

    PathReplayState replay_state;
    if (!path_replay_load_state(
            &replay_state,
            initial_map,
            initial_player,
            initial_boxes,
            initial_num_boxes,
            initial_bombs,
            initial_num_bombs)) {
        return false;
    }

    PathReplayOptions replay_options = {0};
    replay_options.mode = PATH_REPLAY_STRICT_VALIDATE;
    sq_store_map_frame(&map_history[0], &replay_state.map);
    pos_history[0] = replay_state.player;
    memcpy(box_history[0], replay_state.boxes, sizeof(Position) * MAX_BOXES);
    for (uint16_t i = 0; i < path_len; i++) {
        PathReplayStepResult step = path_replay_step(solver, &replay_state, path[i], &replay_options);
        if (step.kind == PATH_REPLAY_STEP_ERROR || step.kind == PATH_REPLAY_STEP_STOPPED) return false;
        sq_store_map_frame(&map_history[i + 1], &replay_state.map);
        pos_history[i + 1] = replay_state.player;
        memcpy(box_history[i + 1], replay_state.boxes, sizeof(Position) * MAX_BOXES);
    }
    return true;
}
FAST_OCRAM_FUNC static bool sq_post_context_build(
    SqPostPathContext* ctx,
    const SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs,
    const Direction* path,
    uint16_t path_len
) {
    if (!ctx || !solver || !initial_map || !path || path_len >= MAX_PATH_LENGTH) return false;

    ctx->map_history = g_sq_post_replay_map_scratch.sq_post_map_history;
    ctx->pos_history = g_sq_post_replay_state_scratch.sq_post_pos_history;
    ctx->box_history = g_sq_post_replay_state_scratch.sq_post_box_history;
    ctx->scan_pause_prefix = g_sq_post_scan_pause_prefix;
    ctx->path = path;
    ctx->path_len = path_len;

    if (!sq_build_history(
            solver,
            initial_map,
            initial_player,
            initial_boxes,
            initial_num_boxes,
            initial_bombs,
            initial_num_bombs,
            path,
            path_len,
            ctx->map_history,
            ctx->pos_history,
            ctx->box_history)) {
        return false;
    }

    ctx->scan_pause_prefix[0] = 0;
    for (uint16_t i = 0; i < path_len; i++) {
        Direction d = path[i];
        ctx->scan_pause_prefix[i + 1] = (uint16_t)(ctx->scan_pause_prefix[i] + ((d.dx == 0 && d.dy == 0) ? 1 : 0));
    }
    return true;
}

static bool sq_post_all_targets_filled(const SokobanSolver* solver, const BitboardMap* map) {
    if (!solver || !map) return false;
    for (int k = 0; k < solver->num_targets; k++) {
        Position target = solver->targets[k].pos;
        if (get_bit(map->targets, target.x, target.y)) return false;
    }
    return true;
}

FAST_OCRAM_FUNC static bool sq_post_replay_task_step(
    const SokobanSolver* solver,
    BitboardMap* map,
    Position* player,
    Position boxes[MAX_BOXES],
    Direction d,
    bool allow_bomb_blast
) {
    if (!solver || !map || !player || !boxes) return false;
    if (d.dx == 0 && d.dy == 0) return true;

    Position next_p = {(uint8_t)(player->x + d.dx), (uint8_t)(player->y + d.dy)};
    Position next_ent = {(uint8_t)(next_p.x + d.dx), (uint8_t)(next_p.y + d.dy)};
    if (!is_in_bounds(next_p.x, next_p.y) || !is_in_bounds(next_ent.x, next_ent.y)) return false;

    bool is_bomb = get_bit(map->bombs, next_p.x, next_p.y);
    bool is_box = get_bit(map->boxes, next_p.x, next_p.y);

    if (get_bit(map->walls, next_p.x, next_p.y)) {
        return false;
    } else if (is_bomb || is_box) {
        if (get_bit(map->walls, next_ent.x, next_ent.y)) {
            if (!(is_bomb && allow_bomb_blast)) return false;
        } else if (get_bit(map->boxes, next_ent.x, next_ent.y) ||
                   get_bit(map->bombs, next_ent.x, next_ent.y)) {
            return false;
        }
    }

    if (is_bomb) {
        clear_bit(map->bombs, next_p.x, next_p.y);
        if (get_bit(map->walls, next_ent.x, next_ent.y)) {
            solver_sim_clear_explosion_walls(map, next_ent);
        } else {
            set_bit(map->bombs, next_ent.x, next_ent.y);
        }
    } else if (is_box) {
        solver_apply_tracked_box_push(solver, map, boxes, next_p, next_ent);
    }
    *player = next_p;
    return true;
}

FAST_OCRAM_FUNC static int sq_extract_path_tasks(
    const SokobanSolver* solver,
    const Direction* path,
    uint16_t path_len,
    const SqPostMapFrame* map_history,
    const Position* pos_history,
    Position box_history[][MAX_BOXES],
    SqPathTask* tasks
) {
    int num_tasks = 0;
    int current_task_start = -1;
    Position current_entity = {0xFF, 0xFF};
    Position initial_entity_pos = {0xFF, 0xFF};
    bool current_is_box = false;
    bool current_is_bomb = false;
    bool current_detonated = false;
    int last_push_t = -1;
    Position last_entity_pos = {0xFF, 0xFF};

    for (int i = 0; i < (int)path_len; i++) {
        Direction d = path[i];
        if (direction_is_pause(d) || !direction_is_cardinal(d)) continue;

        Position next_p = {(uint8_t)(pos_history[i].x + d.dx), (uint8_t)(pos_history[i].y + d.dy)};
        bool is_box = get_bit(map_history[i].boxes, next_p.x, next_p.y);
        bool is_bomb = get_bit(map_history[i].bombs, next_p.x, next_p.y);
        if (!is_box && !is_bomb) continue;

        Position pushed_to = {(uint8_t)(next_p.x + d.dx), (uint8_t)(next_p.y + d.dy)};
        bool same_task = current_task_start >= 0 && is_box == current_is_box && is_bomb == current_is_bomb && pos_equal(next_p, current_entity);
        if (!same_task) {
            if (current_task_start >= 0 && num_tasks < SQ_D1_MAX_TASKS) {
                if (current_is_bomb && current_detonated) {
                    tasks[num_tasks++] = (SqPathTask){SQ_TASK_BOMB_DETONATE, current_task_start, last_push_t, initial_entity_pos, last_entity_pos, -1, -1};
                } else if (current_is_box) {
                    int target_idx = sq_find_target_idx(solver, last_entity_pos);
                    int box_idx = tracked_position_index(box_history[current_task_start], solver->num_boxes, initial_entity_pos);
                    if (target_idx >= 0 && box_idx >= 0 && last_push_t + 1 <= (int)path_len &&
                        get_bit(map_history[last_push_t].targets, last_entity_pos.x, last_entity_pos.y) &&
                        !get_bit(map_history[last_push_t + 1].targets, last_entity_pos.x, last_entity_pos.y)) {
                        tasks[num_tasks++] = (SqPathTask){SQ_TASK_BOX_GOAL, current_task_start, last_push_t, initial_entity_pos, last_entity_pos, box_idx, target_idx};
                    }
                }
            }
            current_task_start = i;
            initial_entity_pos = next_p;
            current_is_box = is_box;
            current_is_bomb = is_bomb;
            current_detonated = false;
        }

        current_entity = pushed_to;
        last_entity_pos = pushed_to;
        last_push_t = i;
        if (is_bomb && get_bit(map_history[i].walls, pushed_to.x, pushed_to.y)) current_detonated = true;
    }

    if (current_task_start >= 0 && num_tasks < SQ_D1_MAX_TASKS) {
        if (current_is_bomb && current_detonated) {
            tasks[num_tasks++] = (SqPathTask){SQ_TASK_BOMB_DETONATE, current_task_start, last_push_t, initial_entity_pos, last_entity_pos, -1, -1};
        } else if (current_is_box) {
            int target_idx = sq_find_target_idx(solver, last_entity_pos);
            int box_idx = tracked_position_index(box_history[current_task_start], solver->num_boxes, initial_entity_pos);
            if (target_idx >= 0 && box_idx >= 0 && last_push_t + 1 <= (int)path_len &&
                get_bit(map_history[last_push_t].targets, last_entity_pos.x, last_entity_pos.y) &&
                !get_bit(map_history[last_push_t + 1].targets, last_entity_pos.x, last_entity_pos.y)) {
                tasks[num_tasks++] = (SqPathTask){SQ_TASK_BOX_GOAL, current_task_start, last_push_t, initial_entity_pos, last_entity_pos, box_idx, target_idx};
            }
        }
    }
    return num_tasks;
}

FAST_OCRAM_FUNC static bool sq_prepare_d1_tail_solver(
    const SokobanSolver* solver,
    const BitboardMap* after_map,
    Position after_player,
    const Position* after_boxes,
    const Entity* after_bombs,
    int after_bomb_count,
    int completed_box_idx,
    int completed_target_idx,
    SokobanSolver* out_solver
) {
    *out_solver = *solver;
    out_solver->bmap = *after_map;
    out_solver->start_player = after_player;

    int wb = 0;
    for (int i = 0; i < solver->num_boxes; i++) {
        if (i == completed_box_idx) continue;
        if (after_boxes[i].x == 0xFF) continue;
        out_solver->boxes[wb] = solver->boxes[i];
        out_solver->boxes[wb].pos = after_boxes[i];
        out_solver->boxes[wb].is_active = true;
        wb++;
    }
    out_solver->num_boxes = (uint8_t)wb;
    for (int i = wb; i < MAX_BOXES; i++) out_solver->boxes[i].is_active = false;

    int wt = 0;
    for (int t = 0; t < solver->num_targets; t++) {
        if (t == completed_target_idx) continue;
        Position tp = solver->targets[t].pos;
        if (!get_bit(out_solver->bmap.targets, tp.x, tp.y)) continue;
        out_solver->targets[wt++] = solver->targets[t];
    }
    out_solver->num_targets = (uint8_t)wt;
    for (int t = wt; t < MAX_TARGETS; t++) out_solver->targets[t].is_active = false;

    if (out_solver->num_boxes != out_solver->num_targets) return false;

    int bc = after_bomb_count;
    if (bc < 0) bc = 0;
    if (bc > MAX_BOMBS) bc = MAX_BOMBS;
    out_solver->num_bombs = (uint8_t)bc;
    for (int b = 0; b < bc; b++) out_solver->bombs[b] = after_bombs[b];
    for (int b = bc; b < MAX_BOMBS; b++) out_solver->bombs[b].is_active = false;

    compute_static_deadlocks(&out_solver->bmap);
    return true;
}

static uint16_t sq_d1_tail_manhattan_matching_lower_bound(const SokobanSolver* solver) {
    if (!solver || solver->num_boxes == 0 || solver->num_boxes != solver->num_targets) return 0;

    const int count = solver->num_boxes;
    const int state_count = 1 << count;
    static uint16_t dp[1 << MAX_TARGETS] ALLOC_IN_OCRAM;
    uint16_t min_player_to_box = 0xFFFFu;

    for (int mask = 0; mask < state_count; mask++) dp[mask] = 0xFFFFu;
    dp[0] = 0;

    for (int mask = 0; mask < state_count; mask++) {
        if (dp[mask] == 0xFFFFu) continue;
        int box_index = 0;
        for (int bits = mask; bits != 0; bits >>= 1) box_index += bits & 1;
        if (box_index >= count) continue;

        for (int target_index = 0; target_index < count; target_index++) {
            int target_bit = 1 << target_index;
            if ((mask & target_bit) != 0) continue;
            if (solver->strict_target_mode &&
                solver->targets[target_index].id != solver->boxes[box_index].id &&
                solver->boxes[box_index].id != -1 &&
                solver->targets[target_index].id != -1) {
                continue;
            }
            uint16_t distance = manhattan_distance(
                solver->boxes[box_index].pos, solver->targets[target_index].pos
            );
            uint32_t next_cost = (uint32_t)dp[mask] + distance;
            int next_mask = mask | target_bit;
            if (next_cost < dp[next_mask]) dp[next_mask] = (uint16_t)next_cost;
        }
    }

    uint16_t result = dp[state_count - 1];
    if (result == 0xFFFFu) return 0;
    for (int box_index = 0; box_index < count; box_index++) {
        uint16_t distance = manhattan_distance(solver->start_player, solver->boxes[box_index].pos);
        if (distance < min_player_to_box) min_player_to_box = distance;
    }
    if (min_player_to_box > 0 && min_player_to_box != 0xFFFFu) {
        uint32_t with_player = (uint32_t)result + min_player_to_box - 1u;
        result = (with_player >= 0xFFFFu) ? 0xFFFFu : (uint16_t)with_player;
    }
    return result;
}

FAST_OCRAM_FUNC static bool __attribute__((noinline)) sq_try_d1_reorder_box_goal(
    SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
) {
    if (!solver) return false;
    uint16_t old_len = solver->best_path_len;
    if (!solver_can_try_bomb_post_opt(solver, old_len)) return false;

    SqPostPathContext ctx;
    if (!sq_post_context_build(&ctx, solver, initial_map, initial_player, initial_boxes, initial_num_boxes,
                               initial_bombs, initial_num_bombs, solver->best_path, old_len)) {
        return false;
    }
    SqPostMapFrame* map_history = ctx.map_history;
    Position* pos_history = ctx.pos_history;
    Position (*box_history)[MAX_BOXES] = ctx.box_history;
    static SqPathTask tasks[SQ_D1_MAX_TASKS];
    Direction* route_path = g_sq_post_route_path;
    Direction* tail_path = g_sq_post_path_scratch.d1.tail_path;
    Direction* best_candidate = g_sq_post_path_scratch.d1.best_candidate;

    int num_tasks = sq_extract_path_tasks(solver, solver->best_path, old_len, map_history, pos_history, box_history, tasks);
    
    if (num_tasks <= 1) return false;

    uint16_t best_len = old_len;
    int tries = 0;

    for (int bi = 0; bi < num_tasks && tries < SQ_D1_MAX_TRIES; bi++) {
        if (tasks[bi].type != SQ_TASK_BOMB_DETONATE) continue;
        int key_step = tasks[bi].end_t + 1;
        if (key_step <= 0 || key_step >= (int)old_len) continue;

        for (int ti = bi + 1; ti < num_tasks && tries < SQ_D1_MAX_TRIES; ti++) {
            if (tasks[ti].type != SQ_TASK_BOX_GOAL) continue;

            bool has_later_bomb_before_box = false;
            for (int mid = bi + 1; mid < ti; mid++) {
                if (tasks[mid].type == SQ_TASK_BOMB_DETONATE) {
                    has_later_bomb_before_box = true;
                    break;
                }
            }
            if (!has_later_bomb_before_box) continue;

            int box_idx = tasks[ti].box_idx;
            int target_idx = tasks[ti].target_idx;
            Position box_pos = box_history[key_step][box_idx];
            Position target_pos = tasks[ti].target_pos;
            if (!pos_equal(box_pos, tasks[ti].entity_start)) continue;
            if (!get_bit(map_history[key_step].targets, target_pos.x, target_pos.y)) continue;
            if (!solver_should_absorb_box(solver, box_idx, target_pos)) continue;
            if (key_step + 2 >= best_len) continue;

            uint16_t route_budget = (uint16_t)(best_len - key_step - 2);
            if (route_budget == 0 || route_budget > MAX_SINGLE_PATH) route_budget = MAX_SINGLE_PATH;

            static BitboardMap route_map ALLOC_IN_OCRAM;
            sq_restore_map_frame(&route_map, &map_history[key_step], initial_map->deadlocks);
            clear_bit(route_map.boxes, box_pos.x, box_pos.y);
            static uint16_t saved_deadlocks[MAP_ROWS] ALLOC_IN_OCRAM;
            static uint16_t saved_targets[MAP_ROWS] ALLOC_IN_OCRAM;
            memcpy(saved_deadlocks, route_map.deadlocks, sizeof(saved_deadlocks));
            memcpy(saved_targets, route_map.targets, sizeof(saved_targets));
            for (int t = 0; t < solver->num_targets; t++) {
                Position tp = solver->targets[t].pos;
                if (t == target_idx) continue;
                if (get_bit(route_map.targets, tp.x, tp.y)) {
                    route_map.deadlocks[tp.y] |= bit_mask_at(tp.x);
                    clear_bit(route_map.targets, tp.x, tp.y);
                }
            }

            uint16_t saved_astar = g_astar_max_steps;
            g_astar_max_steps = route_budget;
            uint16_t route_len = 0;
            int dummy_idx = -1;
            hash_table_clear();
            bool routed = astar_solve_with_mask(
                solver->heap, solver->closed_list, &route_map, pos_history[key_step], box_pos,
                &target_pos, 1, &dummy_idx, MASK_WALL | MASK_BOMB | MASK_BOX,
                route_path, &route_len, ASTAR_NO_MACRO_DEPTH, ROUTE_BOX_NORMAL
            );
            g_astar_max_steps = saved_astar;
            memcpy(route_map.deadlocks, saved_deadlocks, sizeof(saved_deadlocks));
            memcpy(route_map.targets, saved_targets, sizeof(saved_targets));
            tries++;
            if (!routed || route_len == 0 || route_len >= MAX_SINGLE_PATH) continue;

            uint32_t prefix_insert_len = (uint32_t)key_step + route_len;
            if (prefix_insert_len + 2 >= best_len || prefix_insert_len >= MAX_PATH_LENGTH) continue;

            static BitboardMap after_map ALLOC_IN_OCRAM;
            sq_restore_map_frame(&after_map, &map_history[key_step], initial_map->deadlocks);
            Position after_player = pos_history[key_step];
            static Position after_boxes[MAX_BOXES] ALLOC_IN_OCRAM;
            static Entity after_bombs[MAX_BOMBS] ALLOC_IN_OCRAM;
            memcpy(after_boxes, box_history[key_step], sizeof(after_boxes));
            int after_bomb_count = sq_restore_bomb_entities_from_map(after_bombs, &after_map);
            if (!sq_apply_path_steps(solver, &after_map, &after_player, after_boxes, after_bombs,
                                     &after_bomb_count, route_path, route_len)) {
                continue;
            }
            if (get_bit(after_map.targets, target_pos.x, target_pos.y)) continue;

            static SokobanSolver temp_solver;
            if (!sq_prepare_d1_tail_solver(solver, &after_map, after_player, after_boxes, after_bombs,
                                           after_bomb_count, box_idx, target_idx, &temp_solver)) {
                continue;
            }

            uint16_t max_tail = (uint16_t)(best_len - prefix_insert_len - 2);
            uint16_t tail_len = 0;
            if (temp_solver.num_boxes > 0) {
                uint16_t tail_lower = sq_d1_tail_manhattan_matching_lower_bound(&temp_solver);
                if (tail_lower >= max_tail) continue;
                temp_solver.best_path = tail_path;
                temp_solver.best_path_len = 0;
                temp_solver.best_steps = max_tail;
                clear_transposition_table();
                hash_table_clear();
                prepare_dfs_temp_maps(&temp_solver);

                bool saved_sandbox = g_sandbox_mode;
                bool saved_d1_tail_search = g_d1_tail_search;
                uint32_t saved_d1_tail_dfs_nodes = g_d1_tail_dfs_nodes;
                bool saved_macro_evacuation = g_allow_macro_evacuation;
                bool saved_super_evacuation = g_allow_super_evacuation;
                uint16_t saved_astar_tail = g_astar_max_steps;
                g_sandbox_mode = true;
                /* D1 is a bounded tail improvement, not a new full DFS solve. */
                g_d1_tail_search = true;
                g_d1_tail_dfs_nodes = 0;
                g_allow_macro_evacuation = false;
                g_allow_super_evacuation = false;
                g_astar_max_steps = max_tail;
                uint32_t tail_hash = compute_universe_hash(&temp_solver);
                bool tail_ok = sokoban_solve_internal(&temp_solver, 0, tail_hash);
                g_astar_max_steps = saved_astar_tail;
                g_allow_macro_evacuation = saved_macro_evacuation;
                g_allow_super_evacuation = saved_super_evacuation;
                g_sandbox_mode = saved_sandbox;
                g_d1_tail_search = saved_d1_tail_search;
                g_d1_tail_dfs_nodes = saved_d1_tail_dfs_nodes;
                if (!tail_ok || temp_solver.best_path_len > max_tail) continue;
                tail_len = temp_solver.best_path_len;
            }

            uint16_t new_len = (uint16_t)(prefix_insert_len + tail_len);
            if (new_len + 1 >= best_len || new_len >= MAX_PATH_LENGTH) continue;

            memcpy(best_candidate, solver->best_path, key_step * sizeof(Direction));
            memcpy(&best_candidate[key_step], route_path, route_len * sizeof(Direction));
            if (tail_len > 0) memcpy(&best_candidate[key_step + route_len], tail_path, tail_len * sizeof(Direction));

            if (!sq_verify_full_path(solver, initial_map, initial_player, initial_boxes, initial_num_boxes,
                                     initial_bombs, initial_num_bombs, best_candidate, new_len)) {
                continue;
            }

            best_len = new_len;
            memcpy(solver->best_path, best_candidate, best_len * sizeof(Direction));
            solver->best_path_len = best_len;
            solver->best_steps = best_len;
        }
    }
    if (best_len < old_len) {
        return true;
    }
    return false;
}


#define SOLVER_TURN_SMOOTH_MAX_PASSES 12
#define SOLVER_TURN_SMOOTH_MAX_SEGMENT 96

FAST_OCRAM_FUNC static void solver_smooth_equal_len_walk_segments(SokobanSolver* solver) {
    if (!solver || solver->best_path_len == 0 || solver->best_path_len >= MAX_PATH_LENGTH) return;

    bool improved = true;
    int pass = 0;
    while (improved && pass < SOLVER_TURN_SMOOTH_MAX_PASSES) {
        improved = false;
        pass++;

        SqPostPathContext ctx;
        if (!sq_post_context_build(
                &ctx,
                solver,
                &solver->bmap,
                solver->start_player,
                solver->boxes,
                solver->num_boxes,
                solver->bombs,
                solver->num_bombs,
                solver->best_path,
                solver->best_path_len)) {
            return;
        }
        SqPostMapFrame* map_history = ctx.map_history;
        Position* pos_history = ctx.pos_history;
        Position (*box_history)[MAX_BOXES] = ctx.box_history;
        uint16_t* scan_pause_prefix = ctx.scan_pause_prefix;

        for (int t1 = 0; t1 < solver->best_path_len - 1 && !improved; t1++) {
            int max_t2 = t1 + SOLVER_TURN_SMOOTH_MAX_SEGMENT;
            if (max_t2 > solver->best_path_len) max_t2 = solver->best_path_len;
            for (int t2 = max_t2; t2 >= t1 + 3; t2--) {
                int segment_len = t2 - t1;
                int manhattan = abs(pos_history[t1].x - pos_history[t2].x) +
                                abs(pos_history[t1].y - pos_history[t2].y);
                if (manhattan != segment_len) continue;
                if (scan_pause_prefix[t2] != scan_pause_prefix[t1]) continue;
                if (!sq_frames_are_identical(&map_history[t1], &map_history[t2])) continue;
                if (memcmp(box_history[t1], box_history[t2], sizeof(box_history[t1])) != 0) continue;

                static BitboardMap shortcut_map ALLOC_IN_SDRAM;
                sq_restore_map_frame(&shortcut_map, &map_history[t1], solver->bmap.deadlocks);
                Direction* shortcut = g_solver_post_opt_path_scratch.shortcut.shortcut;
                uint16_t shortcut_len = 0;
                Direction prev_dir = {0, 0};
                Direction next_dir = {0, 0};
                bool has_prev = sq_prev_cardinal_dir(solver->best_path, t1, &prev_dir);
                bool has_next = sq_next_cardinal_dir(solver->best_path, t2, solver->best_path_len, &next_dir);

                if (!sq_find_equal_len_turn_shortcut(
                        &shortcut_map, pos_history[t1], pos_history[t2],
                        &solver->best_path[t1], segment_len,
                        has_prev, prev_dir, has_next, next_dir,
                        shortcut, &shortcut_len)) {
                    continue;
                }
                if ((int)shortcut_len != segment_len) continue;

                memcpy(&solver->best_path[t1], shortcut, shortcut_len * sizeof(Direction));
                solver->best_steps = solver->best_path_len;
                improved = true;
                break;
            }
        }
    }

}

FAST_OCRAM_FUNC void __attribute__((noinline)) solver_optimize_post_path(SokobanSolver* solver) {
    if (solver->best_path_len == 0 || solver->best_path_len >= MAX_PATH_LENGTH) return;
    bool improved = true;
    int pass = 0;
    while (improved && pass < SOLVER_POST_OPT_MAX_PASSES) {
        improved = false;
        pass++;

        SqPostPathContext ctx;
        if (!sq_post_context_build(
                &ctx,
                solver,
                &solver->bmap,
                solver->start_player,
                solver->boxes,
                solver->num_boxes,
                solver->bombs,
                solver->num_bombs,
                solver->best_path,
                solver->best_path_len)) {
            return;
        }
        SqPostMapFrame* map_history = ctx.map_history;
        Position* pos_history = ctx.pos_history;
        Position (*box_history)[MAX_BOXES] = ctx.box_history;
        uint16_t* scan_pause_prefix = ctx.scan_pause_prefix;

        for (int i = 0; i + 2 <= (int)solver->best_path_len && !improved; i++) {
            Direction a = solver->best_path[i];
            Direction b = solver->best_path[i + 1];

            if ((a.dx == 0 && a.dy == 0) || (b.dx == 0 && b.dy == 0)) continue;
            if (a.dx + b.dx != 0 || a.dy + b.dy != 0) continue;
            if (scan_pause_prefix[i + 2] != scan_pause_prefix[i]) continue;
            if (!pos_equal(pos_history[i], pos_history[i + 2])) continue;
            if (!sq_frames_are_identical(&map_history[i], &map_history[i + 2])) continue;
            if (memcmp(box_history[i], box_history[i + 2], sizeof(box_history[i])) != 0) continue;

            memmove(&solver->best_path[i],
                    &solver->best_path[i + 2],
                    (solver->best_path_len - i - 2) * sizeof(Direction));
            solver->best_path_len = (uint16_t)(solver->best_path_len - 2);
            solver->best_steps = solver->best_path_len;
            improved = true;
        }
        if (improved) continue;

        static SqPostMacroTask tasks[SQ_POST_MAX_MACRO_TASKS];
        int num_tasks = 0;
        int current_task_start = -1;
        Position current_entity = {0xFF, 0xFF};
        int last_push_t = -1;
        Position initial_entity_pos = {0xFF, 0xFF};

        for (int i = 0; i < solver->best_path_len; i++) {
            Direction d = solver->best_path[i];
            Position next_p = {pos_history[i].x + d.dx, pos_history[i].y + d.dy};
            bool is_scan = (d.dx == 0 && d.dy == 0);

            bool is_push = false;
            if (!is_scan) {
                is_push = get_bit(map_history[i].boxes, next_p.x, next_p.y) || get_bit(map_history[i].bombs, next_p.x, next_p.y);
            }

            if (is_push || is_scan) {
                if (current_task_start == -1) {
                    current_task_start = i;
                    initial_entity_pos = is_scan ? pos_history[i] : next_p;
                    current_entity = is_scan ? pos_history[i] : (Position){next_p.x + d.dx, next_p.y + d.dy};
                    last_push_t = i;
                } else {
                    if (!is_scan && next_p.x == current_entity.x && next_p.y == current_entity.y) {
                        current_entity.x += d.dx;
                        current_entity.y += d.dy;
                        last_push_t = i;
                    } else {
                        if (num_tasks < SQ_POST_MAX_MACRO_TASKS) {
                            tasks[num_tasks++] = (SqPostMacroTask){current_task_start, last_push_t, initial_entity_pos};
                        }
                        current_task_start = i;
                        initial_entity_pos = is_scan ? pos_history[i] : next_p;
                        current_entity = is_scan ? pos_history[i] : (Position){next_p.x + d.dx, next_p.y + d.dy};
                        last_push_t = i;
                    }
                }
            }
        }
        if (current_task_start != -1 && num_tasks < SQ_POST_MAX_MACRO_TASKS) {
            tasks[num_tasks++] = (SqPostMacroTask){current_task_start, last_push_t, initial_entity_pos};
        }

        {
            static BitboardMap relink_map ALLOC_IN_OCRAM;
            
            relink_map = solver->bmap;
            Position relink_p = solver->start_player;
            static Position relink_boxes[MAX_BOXES] ALLOC_IN_OCRAM;
            for (int b = 0; b < MAX_BOXES; b++) relink_boxes[b] = (Position){0xFF, 0xFF};
            for (int b = 0; b < solver->num_boxes; b++) relink_boxes[b] = solver->boxes[b].pos;

            Direction* relink_path = g_solver_post_opt_path_scratch.relink.relink_path;
            int relink_len = 0;
            bool relink_valid = true;

            for (int j = 0; j < num_tasks && relink_valid; j++) {
                Position expected_p = pos_history[tasks[j].start_t];

                if (!pos_equal(relink_p, expected_p)) {
                    Direction* bridge = g_solver_post_opt_path_scratch.relink.bridge;
                    uint16_t bridge_len = 0;
                    hash_table_clear();
                    
                    
                    if (!astar_navigate_mask(solver->heap, solver->closed_list, &relink_map, relink_p, expected_p,
                                             MASK_WALL | MASK_BOMB | MASK_BOX, bridge, &bridge_len)) {
                        
                        relink_valid = false;
                        break;
                    }
                    
                    if (relink_len + bridge_len >= MAX_PATH_LENGTH) {
                        relink_valid = false;
                        break;
                    }
                    for (uint16_t bi = 0; bi < bridge_len; bi++) {
                        relink_path[relink_len++] = bridge[bi];
                        relink_p.x += bridge[bi].dx;
                        relink_p.y += bridge[bi].dy;
                    }
                }

                for (int s = tasks[j].start_t; s <= tasks[j].end_t; s++) {
                    Direction d = solver->best_path[s];
                    bool is_scan = (d.dx == 0 && d.dy == 0);

                    if (relink_len >= MAX_PATH_LENGTH) {
                        relink_valid = false;
                        break;
                    }
                    relink_path[relink_len++] = d;

                    if (is_scan) continue;

                    if (!sq_post_replay_task_step(
                            solver,
                            &relink_map,
                            &relink_p,
                            relink_boxes,
                            d,
                            s == tasks[j].end_t)) {
                        relink_valid = false;
                        break;
                    }
                }

            }

            if (relink_valid && !solver->is_scanning) {
                relink_valid = sq_post_all_targets_filled(solver, &relink_map);
            }

            if (relink_valid && relink_len < solver->best_path_len) {
                memcpy(solver->best_path, relink_path, relink_len * sizeof(Direction));
                solver->best_path_len = (uint16_t)relink_len;
                solver->best_steps = (uint16_t)relink_len;
                improved = true;
            }
        }
        if (improved) continue;
        static bool task_time[MAX_PATH_LENGTH];
        memset(task_time, 0, sizeof(task_time));
        for (int k = 0; k < num_tasks; k++) {
            for (int t = tasks[k].start_t; t <= tasks[k].end_t && t < MAX_PATH_LENGTH; t++) {
                task_time[t] = true;
            }
        }

        for (int t = 0; t < solver->best_path_len && !improved; t++) {
            if (task_time[t]) continue;

            int first_future_task = -1;
            for (int j = 0; j < num_tasks; j++) {
                if (tasks[j].start_t >= t) {
                    first_future_task = j;
                    break;
                }
            }
            if (first_future_task == -1) continue;

            for (int k = 0; k < num_tasks && !improved; k++) {
                if (tasks[k].start_t <= t) continue;

                if (first_future_task > k) continue;

                int dist = abs(pos_history[t].x - tasks[k].entity_start_pos.x) + abs(pos_history[t].y - tasks[k].entity_start_pos.y);
                if (dist > 0 && dist <= 8) {
                    static BitboardMap ver_map ALLOC_IN_OCRAM;
                    
                    sq_restore_map_frame(&ver_map, &map_history[t], solver->bmap.deadlocks);
                    Position ver_p = pos_history[t];
                    static Position ver_boxes[MAX_BOXES] ALLOC_IN_OCRAM;
                    memcpy(ver_boxes, box_history[t], sizeof(ver_boxes));
                    Direction* hybrid = g_solver_post_opt_path_scratch.task_reorder.hybrid;
                    int h_len = 0;
                    bool valid = true;


                    for (int i = 0; i < t; i++) {
                        if (h_len >= MAX_PATH_LENGTH) { valid = false; break; }

                        hybrid[h_len++] = solver->best_path[i];
                    }
                    if (!valid) continue;

                    int push_len = tasks[k].end_t - tasks[k].start_t + 1;
                    int split_len = 0;
                    static BitboardMap temp_map ALLOC_IN_OCRAM;
                    temp_map = ver_map;
                    Position temp_p = ver_p;
                    static Position temp_boxes[MAX_BOXES] ALLOC_IN_OCRAM;
                    memcpy(temp_boxes, box_history[t], sizeof(temp_boxes));

                    Direction* detour = g_solver_post_opt_path_scratch.task_reorder.detour;
                    uint16_t detour_len = 0;
                    hash_table_clear();
                    
                    
                    if (!astar_navigate_mask(solver->heap, solver->closed_list, &temp_map, temp_p, pos_history[tasks[k].start_t], MASK_WALL | MASK_BOMB | MASK_BOX, detour, &detour_len)) {
                        
                        continue;
                    }
                    
                    for (int i = 0; i < detour_len; i++) {
                        temp_p.x += detour[i].dx;
                        temp_p.y += detour[i].dy;
                    }

                    for (int s = 0; s < push_len; s++) {
                        Direction d = solver->best_path[tasks[k].start_t + s];
                        bool is_scan = (d.dx == 0 && d.dy == 0);

                        if (is_scan) {
                            split_len++;
                            continue;
                        }

                        if (!sq_post_replay_task_step(
                                solver,
                                &temp_map,
                                &temp_p,
                                temp_boxes,
                                d,
                                s == push_len - 1)) {
                            break;
                        }
                        split_len++;
                    }

                    if (split_len == 0) continue;

                    static SqPostTaskSegment seq[SQ_POST_MAX_MACRO_TASKS + 1];
                    int num_seq = 0;

                    if (num_seq < SQ_POST_MAX_MACRO_TASKS + 1) seq[num_seq++] = (SqPostTaskSegment){k, 0, split_len};
                    for (int j = first_future_task; j < k && num_seq < SQ_POST_MAX_MACRO_TASKS + 1; j++) {
                        seq[num_seq++] = (SqPostTaskSegment){j, 0, tasks[j].end_t - tasks[j].start_t + 1};
                    }
                    if (split_len < push_len && num_seq < SQ_POST_MAX_MACRO_TASKS + 1) {
                        seq[num_seq++] = (SqPostTaskSegment){k, split_len, push_len - split_len};
                    }
                    for (int j = k + 1; j < num_tasks && num_seq < SQ_POST_MAX_MACRO_TASKS + 1; j++) {
                        seq[num_seq++] = (SqPostTaskSegment){j, 0, tasks[j].end_t - tasks[j].start_t + 1};
                    }

                    for (int idx = 0; idx < num_seq; idx++) {
                        int j = seq[idx].task_idx;
                        int s_offset = seq[idx].offset;
                        int s_len = seq[idx].len;

                        Position expected_p = pos_history[tasks[j].start_t + s_offset];

                        if (ver_p.x != expected_p.x || ver_p.y != expected_p.y) {
                            Direction* bridge = g_solver_post_opt_path_scratch.task_reorder.bridge;
                            uint16_t b_len = 0;
                            hash_table_clear();
                            
                            
                            if (!astar_navigate_mask(solver->heap, solver->closed_list, &ver_map, ver_p, expected_p, MASK_WALL | MASK_BOMB | MASK_BOX, bridge, &b_len)) {
                                
                                valid = false; break;
                            }
                            
                            for (int i = 0; i < b_len; i++) {
                                if (h_len >= MAX_PATH_LENGTH) { valid = false; break; }
                                hybrid[h_len++] = bridge[i];
                                ver_p.x += bridge[i].dx;
                                ver_p.y += bridge[i].dy;
                            }
                        }
                        if (!valid) break;

                        for (int s = 0; s < s_len; s++) {
                            int orig_step_idx = tasks[j].start_t + s_offset + s;
                            Direction d = solver->best_path[orig_step_idx];
                            bool is_scan = (d.dx == 0 && d.dy == 0);

                            if (is_scan) {
                                if (h_len >= MAX_PATH_LENGTH) { valid = false; break; }
                                hybrid[h_len++] = d;
                                continue;
                            }

                            if (!sq_post_replay_task_step(
                                    solver,
                                    &ver_map,
                                    &ver_p,
                                    ver_boxes,
                                    d,
                                    s_offset + s == tasks[j].end_t - tasks[j].start_t)) {
                                valid = false;
                                break;
                            }
                            if (h_len >= MAX_PATH_LENGTH) { valid = false; break; }
                            hybrid[h_len++] = d;
                        }
                        if (!valid) break;
                    }
                    if (!valid) continue;

                    if (!solver->is_scanning && !sq_post_all_targets_filled(solver, &ver_map)) continue;


                    if (valid) {
                        if (h_len < solver->best_path_len) {
                            solver->best_path_len = h_len;
                            solver->best_steps = h_len;
                            memcpy(solver->best_path, hybrid, h_len * sizeof(Direction));
                            improved = true;
                            break;
                        }
                    }
                }
            }
        }

        if (!improved) {
            for (int t1 = 0; t1 < solver->best_path_len - 1; t1++) {
                for (int t2 = solver->best_path_len; t2 >= t1 + 3; t2--) {
                    int original_segment_len = t2 - t1;
                    int manhattan = abs(pos_history[t1].x - pos_history[t2].x) +
                                    abs(pos_history[t1].y - pos_history[t2].y);
                    if (manhattan > original_segment_len) continue;

                    if (sq_frames_are_identical(&map_history[t1], &map_history[t2]) &&
                        memcmp(box_history[t1], box_history[t2], sizeof(box_history[t1])) == 0) {
                        if (scan_pause_prefix[t2] != scan_pause_prefix[t1]) continue;
                        

                        static BitboardMap shortcut_map ALLOC_IN_OCRAM;
                        sq_restore_map_frame(&shortcut_map, &map_history[t1], solver->bmap.deadlocks);
                        Direction* shortcut = g_solver_post_opt_path_scratch.shortcut.shortcut;
                        uint16_t shortcut_len = 0;
                        bool accept_shortcut = false;

                        if (manhattan == original_segment_len) {
                            Direction prev_dir = {0, 0};
                            Direction next_dir = {0, 0};
                            bool has_prev = sq_prev_cardinal_dir(solver->best_path, t1, &prev_dir);
                            bool has_next = sq_next_cardinal_dir(solver->best_path, t2, solver->best_path_len, &next_dir);
                            accept_shortcut = sq_find_equal_len_turn_shortcut(
                                &shortcut_map, pos_history[t1], pos_history[t2],
                                &solver->best_path[t1], original_segment_len,
                                has_prev, prev_dir, has_next, next_dir,
                                shortcut, &shortcut_len);
                        } else {
                            hash_table_clear();
                            
                            
                            if (astar_navigate_mask(solver->heap, solver->closed_list, &shortcut_map, pos_history[t1], pos_history[t2], MASK_WALL | MASK_BOMB | MASK_BOX, shortcut, &shortcut_len)) {
                                
                                accept_shortcut = shortcut_len < original_segment_len;
                            }
                        }

                        if (accept_shortcut) {
                            Direction* new_path = g_solver_post_opt_path_scratch.shortcut.new_path;
                            int new_len = 0;
                            for (int k = 0; k < t1; k++) new_path[new_len++] = solver->best_path[k];
                            for (int k = 0; k < shortcut_len; k++) new_path[new_len++] = shortcut[k];
                            for (int k = t2; k < solver->best_path_len; k++) new_path[new_len++] = solver->best_path[k];

                            memcpy(solver->best_path, new_path, new_len * sizeof(Direction));
                            solver->best_path_len = new_len;
                            solver->best_steps = new_len;
                            improved = true;
                            break;
                        }
                    }
                }
                if (improved) break;
            }
        }
    }

}


void __attribute__((noinline)) solver_optimize_bomb_mixed_path(
    SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
) {
    solver_optimize_post_path(solver);

    if (!g_bomb_seed_solution_committed) {
        sq_try_d1_reorder_box_goal(
            solver, initial_map, initial_player,
            initial_boxes, initial_num_boxes,
            initial_bombs, initial_num_bombs
        );
    }

}
static void solver_attach_static_buffers(SokobanSolver* solver) {
    solver->heap = g_heap_buffer;
    solver->closed_list = &g_closed_buffer;
    solver->best_path = g_best_path_buffer;
    solver->best_steps = 0xFFFF;
}

/*
 * Clear only the per-instance state while preserving any caller-supplied
 * scratch buffers.  The Driver uses copied solver instances during bounded
 * sub-searches, so replacing those pointers with the singleton buffers here
 * would make a reset corrupt the active parent search.
 */
static void solver_reset_instance_fields(SokobanSolver* solver) {
    AStarNode* heap;
    ClosedNode* closed_list;
    Direction* best_path;

    if (!solver) return;
    heap = solver->heap;
    closed_list = solver->closed_list;
    best_path = solver->best_path;
    memset(solver, 0, sizeof(*solver));
    solver->heap = heap;
    solver->closed_list = closed_list;
    solver->best_path = best_path;
    solver->best_steps = 0xFFFF;

    for (int i = 0; i < MAX_BOXES; i++) solver->boxes[i].id = -1;
    for (int i = 0; i < MAX_TARGETS; i++) solver->targets[i].id = -1;
    for (int i = 0; i < MAX_BOMBS; i++) solver->bombs[i].id = -1;
}

static void solver_reset_runtime_tables(void) {
    memset(&g_solver_phase_workspace, 0, sizeof(g_solver_phase_workspace));
    astar_reset_tables();
    clear_transposition_table();
    bomb_reach_caches_reset();
    memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));

    memset(g_heap_buffer, 0, sizeof(g_heap_buffer));
    memset(g_closed_parent_index, 0, sizeof(g_closed_parent_index));
    memset(g_closed_dir_and_steps, 0, sizeof(g_closed_dir_and_steps));
    memset(g_best_path_buffer, 0, sizeof(g_best_path_buffer));
    memset(g_macro_dist_field, 0xFF, sizeof(g_macro_dist_field));
    memset(g_target_dist_field, 0xFF, sizeof(g_target_dist_field));
    memset(g_dfs_path, 0, sizeof(g_dfs_path));
    memset(g_dfs_full_path_buffer, 0, sizeof(g_dfs_full_path_buffer));
    memset(g_destructible_mask, 0, sizeof(g_destructible_mask));
    memset(&g_topology_features, 0, sizeof(g_topology_features));
    memset(g_topology_split_visited, 0, sizeof(g_topology_split_visited));
    memset(g_topology_queue, 0, sizeof(g_topology_queue));
    memset(g_match_heuristic_dp, 0, sizeof(g_match_heuristic_dp));
    memset(g_match_heuristic_next, 0, sizeof(g_match_heuristic_next));

    memset(g_simple_path_pool, 0, sizeof(g_simple_path_pool));
    memset(g_target_walls_pool, 0, sizeof(g_target_walls_pool));
    memset(g_bomb_path_pool, 0, sizeof(g_bomb_path_pool));
    memset(g_bomb_reach_all_path_pool, 0, sizeof(g_bomb_reach_all_path_pool));
    memset(g_bomb_reach_all_len_pool, 0xFF, sizeof(g_bomb_reach_all_len_pool));
    memset(g_bomb_ghost_topology_bonus, 0, sizeof(g_bomb_ghost_topology_bonus));
    memset(g_bomb_ghost_shortcut_bonus, 0, sizeof(g_bomb_ghost_shortcut_bonus));
    memset(g_bomb_dist_p, 0xFF, sizeof(g_bomb_dist_p));
    memset(g_bomb_dist_b, 0xFF, sizeof(g_bomb_dist_b));
    memset(g_bomb_dist_t, 0xFF, sizeof(g_bomb_dist_t));
    memset(g_bomb_player_nav_dist, 0xFF, sizeof(g_bomb_player_nav_dist));
    memset(g_bomb_bfs_dist, 0xFF, sizeof(g_bomb_bfs_dist));
    memset(g_bomb_box_puddle_walls, 0, sizeof(g_bomb_box_puddle_walls));
    memset(g_bomb_puddle_core_walls, 0, sizeof(g_bomb_puddle_core_walls));
    memset(g_bomb_candidate_wall_mask, 0, sizeof(g_bomb_candidate_wall_mask));
    memset(g_bomb_candidate_wall_cells, 0, sizeof(g_bomb_candidate_wall_cells));
    memset(g_bomb_top_candidates, 0, sizeof(g_bomb_top_candidates));
    memset(g_bomb_light_evac_candidates, 0, sizeof(g_bomb_light_evac_candidates));
    memset(g_bomb_phase_order, 0, sizeof(g_bomb_phase_order));
    memset(g_bomb_phase_used, 0, sizeof(g_bomb_phase_used));
    memset(g_bomb_phase_epoch, 0, sizeof(g_bomb_phase_epoch));
    memset(g_bomb_reach_targets, 0, sizeof(g_bomb_reach_targets));
    memset(g_bomb_reach_slots, 0, sizeof(g_bomb_reach_slots));
    memset(&g_bomb_temp_map, 0, sizeof(g_bomb_temp_map));
    memset(g_bomb_route_base_map, 0, sizeof(g_bomb_route_base_map));
    memset(&g_bomb_route_map, 0, sizeof(g_bomb_route_map));
    memset(g_solve_best_path_before, 0, sizeof(g_solve_best_path_before));
    memset(g_light_evac_plan_pool, 0, sizeof(g_light_evac_plan_pool));
    memset(g_light_evac_saved_best_path, 0, sizeof(g_light_evac_saved_best_path));
    memset(g_light_evac_saved_simple_path, 0, sizeof(g_light_evac_saved_simple_path));
    memset(g_bomb_dedupe_keys, 0, sizeof(g_bomb_dedupe_keys));
    memset(g_bomb_dedupe_counts, 0, sizeof(g_bomb_dedupe_counts));
    memset(g_bomb_dedupe_buckets, 0, sizeof(g_bomb_dedupe_buckets));
    memset(g_bomb_dedupe_keep, 0, sizeof(g_bomb_dedupe_keep));
    memset(g_bomb_branch_scratch, 0, sizeof(g_bomb_branch_scratch));
    memset(g_bomb_light_scratch, 0, sizeof(g_bomb_light_scratch));
    memset(g_bomb_strategy_scratch, 0, sizeof(g_bomb_strategy_scratch));
    memset(g_bomb_order_context_scratch, 0, sizeof(g_bomb_order_context_scratch));
    memset(g_solve_internal_scratch, 0, sizeof(g_solve_internal_scratch));

    memset(g_bfs_visited, 0, sizeof(g_bfs_visited));
    memset(g_bfs_queue, 0, sizeof(g_bfs_queue));
    memset(g_push_filter_seen_bits, 0, sizeof(g_push_filter_seen_bits));
    memset(g_push_filter_box_q, 0, sizeof(g_push_filter_box_q));
    memset(g_push_filter_player_q, 0, sizeof(g_push_filter_player_q));
    memset(g_dfs_temp_map, 0, sizeof(g_dfs_temp_map));
    memset(g_dfs_available_targets, 0, sizeof(g_dfs_available_targets));
    memset(g_dfs_frame_scratch, 0, sizeof(g_dfs_frame_scratch));
    memset(g_assignment_beam_states, 0, sizeof(g_assignment_beam_states));
    memset(&g_assignment_beam_box_map, 0, sizeof(g_assignment_beam_box_map));
    memset(&g_assignment_beam_map, 0, sizeof(g_assignment_beam_map));
    memset(&g_assignment_beam_replay_map, 0, sizeof(g_assignment_beam_replay_map));
    memset(g_assignment_beam_replay_boxes, 0, sizeof(g_assignment_beam_replay_boxes));
    memset(&g_assignment_beam_replay_result, 0, sizeof(g_assignment_beam_replay_result));
    memset(g_evac_path_pool, 0, sizeof(g_evac_path_pool));
    memset(g_pocket_path_pool, 0, sizeof(g_pocket_path_pool));
    memset(g_verify_dummy_path, 0, sizeof(g_verify_dummy_path));
    memset(&g_solver_scratch, 0, sizeof(g_solver_scratch));
    memset(g_pocket_unblock_candidates, 0, sizeof(g_pocket_unblock_candidates));
    memset(g_evac_candidates, 0, sizeof(g_evac_candidates));
    memset(g_bomb_candidate_plan_scratch, 0, sizeof(g_bomb_candidate_plan_scratch));
    memset(g_bomb_state_beam_tail_order, 0, sizeof(g_bomb_state_beam_tail_order));
    g_assignment_beam_active_width = SOKOBAN_ASSIGNMENT_BEAM_WIDTH;

    g_astar_max_steps = 0xFFFF;
    g_dfs_first_solution_only = false;
    g_bomb_seed_first_solution_only = false;
    g_bomb_seed_solution_committed = false;
    g_assignment_bounded_refinement = false;
    g_assignment_refinement_disabled = false;
    g_bomb_state_beam_tail_only = false;
    g_bomb_state_beam_full_fallback_active = false;
    g_d1_tail_search = false;
    g_d1_tail_dfs_nodes = 0;
    g_force_maneuver_rescue = false;
    g_enable_push_reach_filter = false;
    g_enable_topology_soft_order = false;
    g_topology_features_valid = false;
    g_use_target_dist_heuristic = false;
    g_enable_path_verification = SCAN_VERIFY_NONE;
    g_sandbox_mode = false;
    g_sandbox_budget_limit = 0;
    g_solve_attempt_recursive_calls = 0;
    g_solve_budget_limit = FAST_BUDGET_RECURSION;
    g_solve_budget_exhausted = false;
    g_solve_ever_budget_exhausted = false;
    g_bfs_run_epoch = 0;
    g_current_macro_depth = 0;
    g_current_super_depth = 0;
    g_current_pocket_depth = 0;
    g_light_evac_recursion_guard = 0;
    g_allow_macro_evacuation = false;
    g_allow_super_evacuation = false;
    g_edge_target_L = false;
    g_edge_target_R = false;
    g_edge_target_U = false;
    g_edge_target_D = false;
}

/* A map load starts a new solver lifecycle.  Keep this separate from the
 * public destroy operation so residual recovery can retain its own session
 * object while replacing the physical solver map between observations. */
static void solver_begin_new_map(SokobanSolver* solver) {
    if (!solver) return;
    solver_reset_runtime_tables();
    sokoban_scan_reset_state();
    solver_reset_instance_fields(solver);
}
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------




typedef enum {
    SOLVE_ATTEMPT_BASE,
    SOLVE_ATTEMPT_MACRO_EVAC,
    SOLVE_ATTEMPT_SUPER_EVAC
} SolveAttemptKind;

typedef struct {
    SolveAttemptKind kind;
    uint32_t hash_salt;
    bool allow_macro;
    bool allow_super;
    bool reset_super_depth;
    bool clear_transposition_before;
    bool reset_push_reach_filter;
    bool use_scan_macro_budget;
} SolveAttemptSpec;

static const SolveAttemptSpec SOLVE_ATTEMPT_PLAN[] = {
    { SOLVE_ATTEMPT_BASE,       0u,                       false, false, false, false, true,  false },
    { SOLVE_ATTEMPT_MACRO_EVAC, UNIVERSE_EVAC_HASH_SALT,  true,  false, false, true,  false, true  },
    { SOLVE_ATTEMPT_SUPER_EVAC, UNIVERSE_SUPER_HASH_SALT, false, true,  true,  true,  false, false }
};

static bool solver_run_attempt_spec(SokobanSolver* solver, uint32_t initial_hash, const SolveAttemptSpec* spec) {
    if (!solver || !spec) return false;

    hash_table_clear();
    solver->best_steps = 0xFFFF;
    solver->best_path_len = 0;

    if (spec->clear_transposition_before) {
        clear_transposition_table();
    }

    g_allow_macro_evacuation = spec->allow_macro;
    g_allow_super_evacuation = spec->allow_super;
    if (spec->reset_super_depth) {
        g_current_super_depth = 0;
    }
    if (spec->reset_push_reach_filter) {
        g_enable_push_reach_filter = false;
    }

    uint32_t saved_budget = g_solve_budget_limit;
    if (spec->use_scan_macro_budget &&
        solver->is_scanning && !solver->strict_target_mode &&
        g_solve_budget_limit > SOKOBAN_SCAN_MACRO_UNIVERSE_BUDGET) {
        g_solve_budget_limit = SOKOBAN_SCAN_MACRO_UNIVERSE_BUDGET;
    }

    solve_attempt_budget_reset();
    if (spec->kind == SOLVE_ATTEMPT_BASE && bomb_state_beam_is_eligible(solver)) {
        if (bomb_state_beam_solve(solver)) {
            if (spec->use_scan_macro_budget) g_solve_budget_limit = saved_budget;
            return true;
        }
        solve_attempt_budget_reset();
    }
    bool use_assignment_beam_first = assignment_beam_first_is_eligible(solver);
    bool use_assignment_refinement = assignment_refinement_is_eligible(solver);
    bool success = false;
    if (use_assignment_beam_first) {
        g_assignment_bounded_refinement = true;
        success = sokoban_solve_internal(
            solver, 0, initial_hash ^ spec->hash_salt
        );
        g_assignment_bounded_refinement = false;

        bool beam_valid =
            success &&
            solver->best_steps != 0xFFFF &&
            solver->best_path_len < MAX_PATH_LENGTH &&
            sq_verify_full_path(
                solver,
                &solver->bmap,
                solver->start_player,
                solver->boxes,
                solver->num_boxes,
                solver->bombs,
                solver->num_bombs,
                solver->best_path,
                solver->best_path_len
            );
        if (!beam_valid) {
            solver->best_steps = 0xFFFF;
            solver->best_path_len = 0;
            clear_transposition_table();
            bomb_reach_caches_reset();
            memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));
            solve_attempt_budget_reset();
            g_assignment_refinement_disabled = true;
            success = sokoban_solve_internal(
                solver, 0, initial_hash ^ spec->hash_salt
            );
            g_assignment_refinement_disabled = false;
        }
    } else if (use_assignment_refinement) {
        g_dfs_first_solution_only = true;
        g_bomb_seed_first_solution_only = true;
        bool seed_success = sokoban_solve_internal(
            solver, 0, initial_hash ^ spec->hash_salt
        );
        g_bomb_seed_first_solution_only = false;
        g_dfs_first_solution_only = false;

        bool seed_valid =
            seed_success &&
            solver->best_steps != 0xFFFF &&
            solver->best_path_len < MAX_PATH_LENGTH &&
            sq_verify_full_path(
                solver,
                &solver->bmap,
                solver->start_player,
                solver->boxes,
                solver->num_boxes,
                solver->bombs,
                solver->num_bombs,
                solver->best_path,
                solver->best_path_len
            );
        if (seed_valid) {
            if (solver->num_bombs == 0) {
                clear_transposition_table();
                bomb_reach_caches_reset();
                memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));
                solve_attempt_budget_reset();
                g_assignment_bounded_refinement = true;
                (void)sokoban_solve_internal(
                    solver, 0, initial_hash ^ spec->hash_salt
                );
                g_assignment_bounded_refinement = false;
            }
            g_bomb_seed_solution_committed = solver->num_bombs > 0;
            success = true;
        } else {
            solver->best_steps = 0xFFFF;
            solver->best_path_len = 0;
            clear_transposition_table();
            bomb_reach_caches_reset();
            memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));
            solve_attempt_budget_reset();
            g_assignment_refinement_disabled = true;
            success = sokoban_solve_internal(
                solver, 0, initial_hash ^ spec->hash_salt
            );
            g_assignment_refinement_disabled = false;
        }
    } else {
        success = sokoban_solve_internal(solver, 0, initial_hash ^ spec->hash_salt);
    }

    if (spec->use_scan_macro_budget) {
        g_solve_budget_limit = saved_budget;
    }
    return success;
}

static uint32_t solver_flash_key_crc(const SokobanScanCacheKey* key) {
    SokobanScanCacheKey temp;
    const uint8_t* p;
    uint32_t crc = 0xFFFFFFFFu;
    if (!key) return 0;
    temp = *key;
    temp.key_crc = 0;
    p = (const uint8_t*)&temp;
    for (uint32_t i = 0; i < (uint32_t)sizeof(temp); i++) {
        crc ^= p[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static bool solver_run_attempt_plan(SokobanSolver* solver, uint32_t initial_hash) {
    for (int i = 0; i < (int)(sizeof(SOLVE_ATTEMPT_PLAN) / sizeof(SOLVE_ATTEMPT_PLAN[0])); i++) {
        if (solver_run_attempt_spec(solver, initial_hash, &SOLVE_ATTEMPT_PLAN[i])) {
            return true;
        }
    }
    return false;
}

FAST_OCRAM_FUNC bool solver_solve(SokobanSolver* solver) {
    init_O1_lookup_tables();
    if (!solver) return false;
    g_assignment_batch_enabled =
        ((solver->is_scanning && !solver->strict_target_mode && solver->num_bombs <= 2) ||
         (!solver->is_scanning && !solver->strict_target_mode &&
          !solver->identified_solve_mode));
    g_enable_topology_soft_order = false;
    g_topology_features_valid = false;
    solve_budget_counters_reset();

    clear_transposition_table();
    bomb_reach_caches_reset();
    memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));

    g_current_macro_depth = 0;
    g_current_super_depth = 0;
    init_zobrist();
    init_destructible_walls(&solver->bmap);
    solver->destroyed_walls_mask = 0;
    solver_clear_bomb_delay_events(solver);
    bool need_three_box_scan_topology =
        solver->is_scanning &&
        !solver->strict_target_mode &&
        solver->num_boxes == 3u;
    if ((topology_profile_may_use_soft_order(solver) ||
         need_three_box_scan_topology) &&
        !g_topology_features_valid) {
        topology_extract(solver);
    }
    g_enable_topology_soft_order = topology_profile_allows_soft_order(solver);

    prepare_dfs_temp_maps(solver);

    if (!(solver->is_scanning && !solver->strict_target_mode)) {
        if (!sokoban_auto_assign_remaining_ids(solver)) return false;
    }

    SokobanScanCacheKey direct_cache_key;
    SokobanScanCachePayload* direct_cache_payload = NULL;
    BitboardMap direct_cache_initial_map;
    Position direct_cache_initial_player;
    Entity direct_cache_initial_boxes[MAX_BOXES];
    Entity direct_cache_initial_bombs[MAX_BOMBS];
    int direct_cache_initial_num_boxes = 0;
    int direct_cache_initial_num_bombs = 0;
    bool direct_cache_eligible =
        g_sokoban_flash_cache_enabled != 0u &&
        !solver->is_scanning &&
        !solver->identified_solve_mode &&
        solver->num_boxes >= SOKOBAN_FLASH_CACHE_MIN_BOXES;
    if (direct_cache_eligible) {
        direct_cache_initial_map = solver->bmap;
        direct_cache_initial_player = solver->start_player;
        direct_cache_initial_num_boxes = solver->num_boxes;
        direct_cache_initial_num_bombs = solver->num_bombs;
        memcpy(direct_cache_initial_boxes, solver->boxes, sizeof(direct_cache_initial_boxes));
        memcpy(direct_cache_initial_bombs, solver->bombs, sizeof(direct_cache_initial_bombs));
        memset(&direct_cache_key, 0, sizeof(direct_cache_key));
        direct_cache_key.policy_version = SOKOBAN_SCAN_CACHE_POLICY_VERSION;
        direct_cache_key.cache_kind = SOKOBAN_FLASH_CACHE_KIND_DIRECT;
        direct_cache_key.solve_mode = solver->strict_target_mode ? 1u : 0u;
        direct_cache_key.rows = MAP_ROWS;
        direct_cache_key.cols = MAP_COLS;
        direct_cache_key.num_boxes = solver->num_boxes;
        direct_cache_key.num_targets = solver->num_targets;
        direct_cache_key.num_bombs = solver->num_bombs;
        direct_cache_key.start_player = solver->start_player;
        memcpy(direct_cache_key.walls, solver->bmap.walls, sizeof(direct_cache_key.walls));
        memcpy(direct_cache_key.targets, solver->bmap.targets, sizeof(direct_cache_key.targets));
        memcpy(direct_cache_key.boxes, solver->bmap.boxes, sizeof(direct_cache_key.boxes));
        memcpy(direct_cache_key.bombs, solver->bmap.bombs, sizeof(direct_cache_key.bombs));
        for (int i = 0; i < MAX_BOXES; i++) {
            direct_cache_key.box_ids[i] = (i < solver->num_boxes) ? solver->boxes[i].id : -1;
        }
        for (int i = 0; i < MAX_TARGETS; i++) {
            direct_cache_key.target_ids[i] = (i < solver->num_targets) ? solver->targets[i].id : -1;
        }
        direct_cache_key.key_crc = solver_flash_key_crc(&direct_cache_key);
        direct_cache_payload = scan_cache_flash_payload();
        if (scan_cache_flash_find_direct(&direct_cache_key, direct_cache_payload) &&
            direct_cache_payload->waypoint_count == 0 &&
            direct_cache_payload->path_len < MAX_PATH_LENGTH) {
            PathReplayResult cached_replay;
            PathReplayOptions cached_options = path_replay_default_options();
            cached_options.mode = PATH_REPLAY_STRICT_VALIDATE;
            if (path_replay_run(
                    solver, &solver->bmap, solver->start_player, solver->boxes, solver->num_boxes,
                    solver->bombs, solver->num_bombs, direct_cache_payload->path,
                    direct_cache_payload->path_len, &cached_options, &cached_replay) &&
                cached_replay.ok && cached_replay.consumed_len == direct_cache_payload->path_len) {
                bool targets_clear = true;
                for (int t = 0; t < solver->num_targets; t++) {
                    Position tp = solver->targets[t].pos;
                    if (get_bit(cached_replay.final_state.map.targets, tp.x, tp.y)) {
                        targets_clear = false;
                        break;
                    }
                }
                if (targets_clear) {
                    memcpy(solver->best_path, direct_cache_payload->path,
                           direct_cache_payload->path_len * sizeof(Direction));
                    solver->best_path_len = direct_cache_payload->path_len;
                    solver->best_steps = direct_cache_payload->path_len;
                    /* 缓存只提供已验证的正式路径，求解器状态仍保持在初始局面。 */
                    solver->bmap = direct_cache_initial_map;
                    solver->start_player = direct_cache_initial_player;
                    solver->num_boxes = (uint8_t)direct_cache_initial_num_boxes;
                    solver->num_bombs = (uint8_t)direct_cache_initial_num_bombs;
                    memcpy(solver->boxes, direct_cache_initial_boxes, sizeof(solver->boxes));
                    memcpy(solver->bombs, direct_cache_initial_bombs, sizeof(solver->bombs));
                    solver->destroyed_walls_mask = 0;
                    if (direct_cache_initial_num_bombs > 0) {
                        solver_build_bomb_delay_events_from_state(
                            solver, &direct_cache_initial_map, direct_cache_initial_player,
                            direct_cache_initial_boxes, direct_cache_initial_num_boxes,
                            direct_cache_initial_bombs, direct_cache_initial_num_bombs
                        );
                    } else {
                        solver_clear_bomb_delay_events(solver);
                    }
                    /* 缓存命中也必须清理本次求解可能修改的全局策略状态。 */
                    g_solve_budget_limit = FAST_BUDGET_RECURSION;
                    g_dfs_first_solution_only = false;
                    g_bomb_seed_first_solution_only = false;
                    g_bomb_seed_solution_committed = false;
                    g_assignment_bounded_refinement = false;
                    g_assignment_refinement_disabled = false;
                    g_bomb_state_beam_tail_only = false;
                    return true;
                }
            }
        }
    }


    bool had_bombs = (solver->num_bombs > 0);
    g_bomb_seed_solution_committed = false;
    static BitboardMap initial_map;
    initial_map = solver->bmap;
    Position initial_player = solver->start_player;
    static Entity initial_boxes[MAX_BOXES];
    static Entity initial_bombs[MAX_BOMBS];
    int initial_num_boxes = solver->num_boxes;
    int initial_num_bombs = solver->num_bombs;
    memcpy(initial_boxes, solver->boxes, sizeof(Entity) * solver->num_boxes);
    memcpy(initial_bombs, solver->bombs, sizeof(Entity) * solver->num_bombs);

    bool original_strict_mode = solver->strict_target_mode;
    static Entity original_targets[MAX_TARGETS];
    memcpy(original_targets, solver->targets, sizeof(Entity) * solver->num_targets);

    uint32_t initial_hash = compute_universe_hash(solver);
    g_bomb_scan_main_route_candidate_count = 0;
    g_solve_budget_limit = FAST_BUDGET_RECURSION;
    if (solver->is_scanning && !solver->strict_target_mode &&
        solver->num_bombs > 0 &&
        solver->num_boxes < SOKOBAN_FLASH_CACHE_MIN_BOXES) {
        g_solve_budget_limit = HARD_BUDGET_RECURSION;
    }
    bool success = solver_run_attempt_plan(solver, initial_hash);

    if (!success && !g_sandbox_mode && g_solve_budget_limit < HARD_BUDGET_RECURSION && g_solve_ever_budget_exhausted) {
        solver->bmap = initial_map;
        solver->start_player = initial_player;
        solver->num_boxes = (uint8_t)initial_num_boxes;
        solver->num_bombs = (uint8_t)initial_num_bombs;
        memcpy(solver->boxes, initial_boxes, sizeof(Entity) * initial_num_boxes);
        memcpy(solver->bombs, initial_bombs, sizeof(Entity) * initial_num_bombs);
        solver->strict_target_mode = original_strict_mode;
        memcpy(solver->targets, original_targets, sizeof(Entity) * solver->num_targets);
        solver->destroyed_walls_mask = 0;
        solver->best_steps = 0xFFFF;
        solver->best_path_len = 0;

        clear_transposition_table();
        memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));
        bomb_reach_caches_reset();
        g_solve_budget_limit = HARD_BUDGET_RECURSION;
        g_bomb_scan_main_route_candidate_count = 0;

        success = solver_run_attempt_plan(solver, initial_hash);
    }

    // 非扫描求解成功后整理物化路径。
    if (success) {
        if (!solver->is_scanning) {
            if (had_bombs) {
                solver_optimize_bomb_mixed_path(
                    solver, &initial_map, initial_player,
                    initial_boxes, initial_num_boxes,
                    initial_bombs, initial_num_bombs
                );
            }
            solver_smooth_equal_len_walk_segments(solver);
            if (had_bombs) {
                solver_build_bomb_delay_events_from_state(
                    solver, &initial_map, initial_player,
                    initial_boxes, initial_num_boxes,
                    initial_bombs, initial_num_bombs
                );
            }

            /* 直接求解缓存只保存最终正式路径，并在写入前再次严格重放。 */
            if (direct_cache_eligible && solver->best_path_len > 0 &&
                solver->best_path_len < MAX_PATH_LENGTH) {
                static SokobanScanCachePayload direct_store_payload;
                PathReplayResult direct_store_replay;
                PathReplayOptions direct_store_options = path_replay_default_options();
                direct_store_options.mode = PATH_REPLAY_STRICT_VALIDATE;
                memset(&direct_store_payload, 0, sizeof(direct_store_payload));
                direct_store_payload.path_len = solver->best_path_len;
                memcpy(direct_store_payload.path, solver->best_path,
                       solver->best_path_len * sizeof(Direction));
                if (path_replay_run(
                        solver, &initial_map, initial_player,
                        initial_boxes, initial_num_boxes,
                        initial_bombs, initial_num_bombs,
                        direct_store_payload.path, direct_store_payload.path_len,
                        &direct_store_options, &direct_store_replay) &&
                    direct_store_replay.ok &&
                    direct_store_replay.consumed_len == direct_store_payload.path_len) {
                    bool targets_clear = true;
                    for (int t = 0; t < solver->num_targets; t++) {
                        Position tp = solver->targets[t].pos;
                        if (get_bit(direct_store_replay.final_state.map.targets, tp.x, tp.y)) {
                            targets_clear = false;
                            break;
                        }
                    }
                    if (targets_clear) {
                        direct_store_payload.waypoint_count = 0;
                        (void)scan_cache_flash_store_direct(
                            &direct_cache_key, &direct_store_payload);
                    }
                }
            }
        }
    } else {
        solver->strict_target_mode = original_strict_mode;
        memcpy(solver->targets, original_targets, sizeof(Entity) * solver->num_targets);
    }
    g_solve_budget_limit = FAST_BUDGET_RECURSION;
    g_dfs_first_solution_only = false;
    g_bomb_seed_first_solution_only = false;
    g_bomb_seed_solution_committed = false;
    g_assignment_bounded_refinement = false;
    g_assignment_refinement_disabled = false;
    g_bomb_state_beam_tail_only = false;
    return success;
}

void solver_warmup(void) {
    init_O1_lookup_tables();
    init_zobrist();
    hash_table_clear();
    clear_transposition_table();
    bomb_reach_caches_reset();
    memset(g_component_fail_cache, 0, sizeof(g_component_fail_cache));

    MinHeap warm_heap;
    heap_init(&warm_heap, g_heap_buffer, MAX_HEAP_SIZE);
}
FAST_OCRAM_FUNC bool solver_solve_robust(SokobanSolver* solver) {
    return solver_solve(solver);
}

SokobanSolver* solver_create(void) {
    solver_warmup();
    solver_reset_runtime_tables();
    sokoban_scan_reset_state();
    memset(&g_solver_instance, 0, sizeof(SokobanSolver));
    solver_attach_static_buffers(&g_solver_instance);
    return &g_solver_instance;
}

bool solver_clear_scan_cache_flash(void) {
    if (g_sokoban_flash_cache_enabled == 0u) return false;
    return scan_cache_flash_clear();
}

void solver_destroy(SokobanSolver* solver) {
    if (!solver) return;

    solver_begin_new_map(solver);
    if (solver == &g_solver_instance) {
        solver_attach_static_buffers(&g_solver_instance);
    }
}


bool solver_load_map_from_string(SokobanSolver* solver, const char* map_string) {
    if (!solver) return false;
    solver_begin_new_map(solver);
    if (!map_string) return false;

    int row = 0, col = 0;
    bool has_ids = false;
    bool entity_overflow = false;

    for (const char* p = map_string; *p != '\0'; p++) {
        if (*p == '|') { row++; col = 0; continue; }
        if (row >= MAP_ROWS || col >= MAP_COLS) { col++; continue; }

        char c = *p;
        if (c == '#') {
            set_bit(solver->bmap.walls, col, row);
        }
        else if (c == '@') {
            solver->start_player.x = col; solver->start_player.y = row;
            solver->map_start_player = solver->start_player;
            solver->map_start_player_valid = true;
        }
        else if (c == 'B' && solver->num_bombs < MAX_BOMBS) {
            set_bit(solver->bmap.bombs, col, row);
            solver->bombs[solver->num_bombs].pos = (Position){col, row};
            solver->bombs[solver->num_bombs].is_active = true;
            solver->num_bombs++;
        }
        else if (c >= '0' && c <= '9') {
            has_ids = true;
            int box_id = c - '0';
            if (solver->num_boxes < MAX_BOXES) {
                set_box_bit(&solver->bmap, col, row);
                Entity* box = &solver->boxes[solver->num_boxes++];
                box->pos = (Position){col, row};
                box->id = box_id;
                box->is_active = true;
            } else {
                entity_overflow = true;
            }
        }
        else if (c >= 'a' && c <= 'j') {
            has_ids = true;
            int target_id = c - 'a';
            if (solver->num_targets < MAX_TARGETS) {
                set_bit(solver->bmap.targets, col, row);
                Entity* target = &solver->targets[solver->num_targets++];
                target->pos = (Position){col, row};
                target->id = target_id;
                target->is_active = true;
            } else {
                entity_overflow = true;
            }
        }
        else if (c >= 'k' && c <= 'z') {
            entity_overflow = true;
        }
        else if (c == '$') {
            if (solver->num_boxes < MAX_BOXES) {
                set_box_bit(&solver->bmap, col, row);
                Entity* box = &solver->boxes[solver->num_boxes++];
                box->pos = (Position){col, row};
                box->id = -1;
                box->is_active = true;
            } else {
                entity_overflow = true;
            }
        }
        else if (c == '*') {
            if (solver->num_boxes < MAX_BOXES && solver->num_targets < MAX_TARGETS) {
                set_box_bit(&solver->bmap, col, row);
                Entity* box = &solver->boxes[solver->num_boxes++];
                box->pos = (Position){col, row};
                box->id = -1;
                box->is_active = true;
                set_bit(solver->bmap.targets, col, row);
                Entity* target = &solver->targets[solver->num_targets++];
                target->pos = (Position){col, row};
                target->id = -1;
                target->is_active = true;
            } else {
                entity_overflow = true;
            }
        }
        else if (c == '.') {
            if (solver->num_targets < MAX_TARGETS) {
                set_bit(solver->bmap.targets, col, row);
                Entity* target = &solver->targets[solver->num_targets++];
                target->pos = (Position){col, row};
                target->id = -1;
                target->is_active = true;
            } else {
                entity_overflow = true;
            }
        }
        col++;
    }

    if (entity_overflow) {
        /* Do not expose a partially parsed map after a failed load. */
        solver_begin_new_map(solver);
        return false;
    }

    int cb_count = 0, ct_count = 0;
    Entity cb[MAX_BOXES], ct[MAX_TARGETS];
    for (int i = 0; i < solver->num_boxes; i++) if (solver->boxes[i].is_active) cb[cb_count++] = solver->boxes[i];
    memcpy(solver->boxes, cb, sizeof(Entity) * cb_count); solver->num_boxes = cb_count;
    for (int i = 0; i < solver->num_targets; i++) if (solver->targets[i].is_active) ct[ct_count++] = solver->targets[i];
    memcpy(solver->targets, ct, sizeof(Entity) * ct_count); solver->num_targets = ct_count;

    g_edge_target_L = g_edge_target_R = g_edge_target_U = g_edge_target_D = false;
    for (int y = 1; y < MAP_ROWS - 1; y++) {
        if ((solver->bmap.targets[y] & (1 << 1)) != 0) g_edge_target_L = true;
        if ((solver->bmap.targets[y] & (1 << (MAP_COLS - 2))) != 0) g_edge_target_R = true;
    }
    for (int x = 1; x < MAP_COLS - 1; x++) {
        if ((solver->bmap.targets[1] & (1 << x)) != 0) g_edge_target_U = true;
        if ((solver->bmap.targets[MAP_ROWS - 2] & (1 << x)) != 0) g_edge_target_D = true;
    }

    solver->strict_target_mode = has_ids;
    init_O1_lookup_tables();
    init_zobrist();
    init_destructible_walls(&solver->bmap);
    compute_static_deadlocks(&solver->bmap);
    bool valid_map = (solver->num_boxes == solver->num_targets && solver->num_boxes > 0);
    g_topology_features_valid = false;
    g_enable_topology_soft_order = false;

    if (!valid_map) {
        /* A failed normal load starts no usable solve lifecycle. */
        solver_begin_new_map(solver);
        return false;
    }
    return true;
}

Direction* solver_get_solution(SokobanSolver* solver, uint16_t* length) {
    if (!solver || !length) return NULL;
    *length = solver->best_path_len;
    return solver->best_path;
}

void solver_set_strict_target_mode(SokobanSolver* solver, bool strict_mode) {
    if (solver) solver->strict_target_mode = strict_mode;
}

void solver_set_identified_solve_mode(SokobanSolver* solver, bool identified) {
    if (solver) solver->identified_solve_mode = identified;
}


void solver_refresh_deadlocks(SokobanSolver* solver) {
    if (solver) {
        compute_static_deadlocks(&solver->bmap);
    }
}













static bool solver_validate_residual_map_text(const char* map_string) {
    int row = 0;
    int col = 0;
    int player_count = 0;
    int box_count = 0;
    int target_count = 0;
    int bomb_count = 0;

    if (!map_string) return false;
    for (const char* p = map_string; *p != '\0'; p++) {
        char c = *p;
        if (c == '|') {
            if (col != MAP_COLS || row >= MAP_ROWS - 1) return false;
            row++;
            col = 0;
            continue;
        }
        if (row >= MAP_ROWS || col >= MAP_COLS) return false;

        if (c == '@') {
            player_count++;
        } else if (c >= '0' && c <= '9') {
            box_count++;
        } else if (c >= 'a' && c <= 'j') {
            target_count++;
        } else if (c == '$') {
            box_count++;
        } else if (c == '.') {
            target_count++;
        } else if (c == 'B') {
            bomb_count++;
        } else if (c != '#' && c != ' ') {
            return false;
        }

        if (box_count > MAX_BOXES || target_count > MAX_TARGETS || bomb_count > MAX_BOMBS) return false;
        col++;
    }

    return row == MAP_ROWS - 1 && col == MAP_COLS && player_count == 1;
}

bool solver_load_residual_map_from_string(SokobanSolver* solver, const char* map_string) {
    if (!solver) return false;
    solver_destroy(solver);
    if (!solver_validate_residual_map_text(map_string)) return false;

    memset(&solver->bmap, 0, sizeof(BitboardMap));
    solver->destroyed_walls_mask = 0;
    solver->num_boxes = 0;
    solver->num_targets = 0;
    solver->num_bombs = 0;
    solver->identified_solve_mode = false;
    for (int i = 0; i < MAX_BOXES; i++) {
        solver->boxes[i].is_active = false;
        solver->boxes[i].id = -1;
    }
    for (int i = 0; i < MAX_TARGETS; i++) {
        solver->targets[i].is_active = false;
        solver->targets[i].id = -1;
    }
    for (int i = 0; i < MAX_BOMBS; i++) {
        solver->bombs[i].is_active = false;
        solver->bombs[i].id = -1;
    }

    int row = 0;
    int col = 0;
    for (const char* p = map_string; *p != '\0'; p++) {
        char c = *p;
        if (c == '|') {
            row++;
            col = 0;
            continue;
        }

        if (c == '#') {
            set_bit(solver->bmap.walls, col, row);
        } else if (c == '@') {
            solver->start_player = (Position){(uint8_t)col, (uint8_t)row};
            solver->map_start_player = solver->start_player;
            solver->map_start_player_valid = true;
        } else if (c == 'B') {
            set_bit(solver->bmap.bombs, col, row);
            solver->bombs[solver->num_bombs] = (Entity){(Position){(uint8_t)col, (uint8_t)row}, -1, true};
            solver->num_bombs++;
        } else if (c >= '0' && c <= '9') {
            set_box_bit(&solver->bmap, col, row);
            solver->boxes[solver->num_boxes] = (Entity){(Position){(uint8_t)col, (uint8_t)row}, (int8_t)(c - '0'), true};
            solver->num_boxes++;
        } else if (c >= 'a' && c <= 'j') {
            set_bit(solver->bmap.targets, col, row);
            solver->targets[solver->num_targets] = (Entity){(Position){(uint8_t)col, (uint8_t)row}, (int8_t)(c - 'a'), true};
            solver->num_targets++;
        } else if (c == '$') {
            set_box_bit(&solver->bmap, col, row);
            solver->boxes[solver->num_boxes] = (Entity){(Position){(uint8_t)col, (uint8_t)row}, -1, true};
            solver->num_boxes++;
        } else if (c == '.') {
            set_bit(solver->bmap.targets, col, row);
            solver->targets[solver->num_targets] = (Entity){(Position){(uint8_t)col, (uint8_t)row}, -1, true};
            solver->num_targets++;
        }
        col++;
    }

    g_edge_target_L = g_edge_target_R = g_edge_target_U = g_edge_target_D = false;
    for (int y = 1; y < MAP_ROWS - 1; y++) {
        if ((solver->bmap.targets[y] & (1 << 1)) != 0) g_edge_target_L = true;
        if ((solver->bmap.targets[y] & (1 << (MAP_COLS - 2))) != 0) g_edge_target_R = true;
    }
    for (int x = 1; x < MAP_COLS - 1; x++) {
        if ((solver->bmap.targets[1] & (1 << x)) != 0) g_edge_target_U = true;
        if ((solver->bmap.targets[MAP_ROWS - 2] & (1 << x)) != 0) g_edge_target_D = true;
    }

    solver->strict_target_mode = false;
    solver->is_scanning = false;
    solver->scan_waypoint_count = 0;
    solver->scan_current_index = 0;
    solver->best_path_len = 0;
    solver->best_steps = 0xFFFF;
    init_O1_lookup_tables();
    init_zobrist();
    init_destructible_walls(&solver->bmap);
    compute_static_deadlocks(&solver->bmap);
    g_topology_features_valid = false;
    g_enable_topology_soft_order = false;
    return true;
}
