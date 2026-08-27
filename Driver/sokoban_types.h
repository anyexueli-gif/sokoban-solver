#ifndef SOKOBAN_TYPES_H
#define SOKOBAN_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*
 * Compiler/linker placement for zero-initialized Driver storage.
 *
 * Keep this small portability shim with the common Driver types so every
 * implementation file can use the same custom BSS section names without a
 * separate memory-layout header.  The root scatter file maps these sections
 * to the intended SDRAM/OCRAM/DTCM regions.
 */
#if defined(__MINGW32__)
#define SOKOBAN_BSS_SECTION(region) \
    __attribute__((section(".bss." region ",\"bw\"#")))
#else
#define SOKOBAN_BSS_SECTION(region) \
    __attribute__((section(".bss." region)))
#endif

#define MAP_ROWS 16
#define MAP_COLS 12
#define MAX_BOXES 10
#define MAX_TARGETS 10
#define MAX_BOMBS 5
#define MAX_PATH_LENGTH 320
#define SOKOBAN_SCAN_CACHE_POLICY_VERSION 18u
#define SOKOBAN_FLASH_CACHE_MIN_BOXES 5u
#define SOKOBAN_SCAN_MAX_WAYPOINTS (4 * (MAX_BOXES + MAX_TARGETS))
#define SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS (MAX_BOXES + MAX_TARGETS)

/* 固定的单片机求解器配置。 */

#define SOKOBAN_PARAM_MAX_BOMB_CANDIDATES 50
#define SOKOBAN_PARAM_MAX_LIGHT_EVAC_PLANS 4
#define SOKOBAN_PARAM_MAX_BOMB_OPTIONS 80
#define SOKOBAN_PARAM_COMPONENT_FAIL_CACHE_SIZE 512u
#define SOKOBAN_PARAM_BOMB_REACH_FAIL_CACHE_SIZE 512u
#define SOKOBAN_BOMB_REACH_FAIL_CACHE_CAPACITY SOKOBAN_PARAM_BOMB_REACH_FAIL_CACHE_SIZE

#define SOKOBAN_PARAM_L1_HASH_SIZE 2048u
#define SOKOBAN_PARAM_L1_UNIVERSE_HASH_SIZE 1024u
#define SOKOBAN_PARAM_SQ_D1_MAX_TASKS 16
#define SOKOBAN_PARAM_MAX_MACRO_TASKS 16
#if (SOKOBAN_PARAM_COMPONENT_FAIL_CACHE_SIZE & (SOKOBAN_PARAM_COMPONENT_FAIL_CACHE_SIZE - 1u)) != 0
#error "SOKOBAN_PARAM_COMPONENT_FAIL_CACHE_SIZE must be a power of two."
#endif
#if (SOKOBAN_PARAM_BOMB_REACH_FAIL_CACHE_SIZE & (SOKOBAN_PARAM_BOMB_REACH_FAIL_CACHE_SIZE - 1u)) != 0
#error "SOKOBAN_PARAM_BOMB_REACH_FAIL_CACHE_SIZE must be a power of two."
#endif
#if (SOKOBAN_PARAM_L1_HASH_SIZE & (SOKOBAN_PARAM_L1_HASH_SIZE - 1u)) != 0
#error "SOKOBAN_PARAM_L1_HASH_SIZE must be a power of two."
#endif
#if (SOKOBAN_PARAM_L1_UNIVERSE_HASH_SIZE & (SOKOBAN_PARAM_L1_UNIVERSE_HASH_SIZE - 1u)) != 0
#error "SOKOBAN_PARAM_L1_UNIVERSE_HASH_SIZE must be a power of two."
#endif
#if SOKOBAN_PARAM_MAX_BOMB_CANDIDATES < 16
#error "SOKOBAN_PARAM_MAX_BOMB_CANDIDATES is too small for current bomb ordering windows."
#endif
#define MASK_EMPTY   0x00        // 0000 0000 - 空地
#define MASK_WALL    (1 << 0)    // 0000 0001 - 墙壁
#define MASK_TARGET  (1 << 1)    // 0000 0010 - 目标点
#define MASK_BOMB    (1 << 2)    // 0000 0100 - 炸弹
#define MASK_BOX     (1 << 3)    // 0000 1000 - 箱子（动态）

/**
 * 坐标点数据结构 (2 bytes)
 */
typedef struct {
    uint8_t x;
    uint8_t y;
} Position;

_Static_assert(sizeof(Position) == 2, "Position must stay 2 bytes");

/**
 * 统一的实体结构 (4 bytes)
 * 箱子、目标点、炸弹共用
 */
typedef struct {
    Position pos;      // 实体坐标
    int8_t id;         // ID号，-1 表示未分配，>=0 表示真实 ID
    bool is_active;    // 是否激活（替代原来判断 x == 0xFF 的方式）
} Entity;

/**
 * 方向结构
 */
typedef struct {
    int8_t dx;
    int8_t dy;
} Direction;

typedef enum {
    SOKOBAN_FLASH_CACHE_KIND_SCAN = 1,
    SOKOBAN_FLASH_CACHE_KIND_DIRECT = 2
} SokobanFlashCacheKind;

typedef struct {
    uint32_t key_crc;
    uint32_t policy_version;
    uint8_t cache_kind;
    uint8_t solve_mode;
    uint8_t rows;
    uint8_t cols;
    uint8_t num_bombs;
    uint8_t num_boxes;
    uint8_t num_targets;
    uint8_t reserved[3];
    Position start_player;
    uint16_t walls[MAP_ROWS];
    uint16_t targets[MAP_ROWS];
    uint16_t boxes[MAP_ROWS];
    uint16_t bombs[MAP_ROWS];
    int8_t box_ids[MAX_BOXES];
    int8_t target_ids[MAX_TARGETS];
} SokobanScanCacheKey;

typedef struct {
    uint16_t path_len;
    Direction path[MAX_PATH_LENGTH];
    uint8_t waypoint_count;
    Entity waypoints[SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS];
    Position pause_positions[SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS];
    Position end_player;
    uint16_t after_walls[MAP_ROWS];
} SokobanScanCachePayload;
typedef enum {
    SCAN_VERIFY_NONE = 0,
    SCAN_VERIFY_LIGHT = 1,
    SCAN_VERIFY_MEDIUM = 2,
    SCAN_VERIFY_STRICT = 3
} ScanVerificationLevel;

extern uint16_t g_astar_max_steps;
extern ScanVerificationLevel g_enable_path_verification;
extern bool g_sandbox_mode;
extern uint32_t g_sandbox_budget_limit;


struct SokobanSolver;
#define DIRECTION_INDEX_NONE 0xFFu

static inline bool direction_is_pause(Direction d) {
    return d.dx == 0 && d.dy == 0;
}

static inline bool direction_is_cardinal(Direction d) {
    return (d.dx == 0 && (d.dy == -1 || d.dy == 1)) ||
           (d.dy == 0 && (d.dx == -1 || d.dx == 1));
}

static inline bool direction_equal(Direction a, Direction b) {
    return a.dx == b.dx && a.dy == b.dy;
}

static inline uint8_t direction_index(Direction d) {
    if (d.dx == 0 && d.dy == -1) return 0;
    if (d.dx == 0 && d.dy == 1) return 1;
    if (d.dx == -1 && d.dy == 0) return 2;
    if (d.dx == 1 && d.dy == 0) return 3;
    return DIRECTION_INDEX_NONE;
}

static inline Direction direction_from_index(uint8_t idx) {
    static const Direction dirs[4] = {
        {0, -1},
        {0, 1},
        {-1, 0},
        {1, 0}
    };
    return dirs[idx & 0x03u];
}

static inline uint8_t direction_index_between(Position from, Position to) {
    Direction direction = {
        (int8_t)((int)to.x - (int)from.x),
        (int8_t)((int)to.y - (int)from.y)
    };
    return direction_index(direction);
}

static inline uint8_t direction_axis_index(Position from, Position to) {
    if (from.x == to.x) {
        if (to.y < from.y) return 0u;
        if (to.y > from.y) return 1u;
    } else if (from.y == to.y) {
        if (to.x < from.x) return 2u;
        if (to.x > from.x) return 3u;
    }
    return DIRECTION_INDEX_NONE;
}

static inline uint8_t direction_quarter_turns(uint8_t from_index, uint8_t to_index) {
    if (from_index >= 4u || to_index >= 4u || from_index == to_index) return 0u;
    return ((from_index ^ to_index) == 1u) ? 2u : 1u;
}

static inline uint8_t path_first_direction_index(
    const Direction* path,
    uint16_t path_len,
    uint8_t fallback_index
) {
    if (path) {
        for (uint16_t index = 0; index < path_len; index++) {
            uint8_t direction = direction_index(path[index]);
            if (direction < 4u) return direction;
        }
    }
    return fallback_index;
}

static inline uint8_t path_end_direction_index(
    const Direction* path,
    uint16_t path_len,
    uint8_t fallback_index
) {
    uint8_t result = fallback_index;
    if (path) {
        for (uint16_t index = 0; index < path_len; index++) {
            uint8_t direction = direction_index(path[index]);
            if (direction < 4u) result = direction;
        }
    }
    return result;
}

static inline uint16_t path_direction_bend_count(const Direction* path, uint16_t path_len) {
    uint8_t previous = DIRECTION_INDEX_NONE;
    uint16_t bends = 0u;

    if (!path) return 0u;
    for (uint16_t index = 0; index < path_len; index++) {
        uint8_t direction = direction_index(path[index]);
        if (direction >= 4u) continue;
        if (previous < 4u && previous != direction) bends++;
        previous = direction;
    }
    return bends;
}

/**
 * 判断两个位置是否相等
 */
static inline uint16_t position_key(Position p) {
    return (uint16_t)((uint16_t)p.x | ((uint16_t)p.y << 8));
}

static inline bool pos_equal(Position a, Position b) {
    return position_key(a) == position_key(b);
}

/**
 * 计算曼哈顿距离
 */
static inline uint16_t manhattan_distance(Position a, Position b) {
    int dx = (a.x > b.x) ? (a.x - b.x) : (b.x - a.x);
    int dy = (a.y > b.y) ? (a.y - b.y) : (b.y - a.y);
    return dx + dy;
}

/**
 * 检查坐标是否在地图范围内
 */
static inline bool is_in_bounds(int x, int y) {
    return x >= 0 && x < MAP_COLS && y >= 0 && y < MAP_ROWS;
}

static inline uint16_t bit_mask_at(int x) {
    return (uint16_t)(1u << x);
}

static inline void clear_bit(uint16_t* layer, int x, int y) {
    layer[y] &= (uint16_t)(~bit_mask_at(x));
}

static inline void set_bit(uint16_t* layer, int x, int y) {
    layer[y] |= bit_mask_at(x);
}

static inline bool get_bit(const uint16_t* layer, int x, int y) {
    return (layer[y] & bit_mask_at(x)) != 0;
}

/**
 * 空间降维的位棋盘 (Bitboard) 结构
 */
typedef struct {
    uint16_t walls[MAP_ROWS];
    uint16_t targets[MAP_ROWS];
    uint16_t bombs[MAP_ROWS];
    uint16_t boxes[MAP_ROWS];
    uint16_t deadlocks[MAP_ROWS];
    uint16_t h_tunnels[MAP_ROWS];
    uint16_t v_tunnels[MAP_ROWS];
} BitboardMap;

/* Refresh topology-only tunnel layers after the wall/target layout changes. */
static inline void sokoban_refresh_dynamic_tunnels(BitboardMap* bmap) {
    const uint16_t valid_cols_mask = (uint16_t)((1u << MAP_COLS) - 1u);
    const uint16_t edge_cols_mask = (uint16_t)(bit_mask_at(0) | bit_mask_at(MAP_COLS - 1));
    const uint16_t interior_cols_mask = (uint16_t)(valid_cols_mask & (uint16_t)(~edge_cols_mask));

    if (!bmap) return;
    memset(bmap->h_tunnels, 0, sizeof(bmap->h_tunnels));
    memset(bmap->v_tunnels, 0, sizeof(bmap->v_tunnels));

    for (int y = 1; y < MAP_ROWS - 1; y++) {
        uint16_t self_empty = (uint16_t)(~(bmap->walls[y] | bmap->targets[y])) & interior_cols_mask;
        uint16_t wall_up = bmap->walls[y - 1] & valid_cols_mask;
        uint16_t wall_down = bmap->walls[y + 1] & valid_cols_mask;
        uint16_t wall_left = (uint16_t)(bmap->walls[y] << 1) & valid_cols_mask;
        uint16_t wall_right = (uint16_t)(bmap->walls[y] >> 1);

        bmap->h_tunnels[y] = (uint16_t)(wall_up & wall_down & self_empty);
        bmap->v_tunnels[y] = (uint16_t)(wall_left & wall_right & self_empty);
    }
}




typedef enum {
    PATH_REPLAY_ERROR_NONE = 0,
    PATH_REPLAY_ERROR_INVALID_ARGUMENT = 1,
    PATH_REPLAY_ERROR_INVALID_STEP = 2,
    PATH_REPLAY_ERROR_BLOCKED = 3
} PathReplayError;

typedef enum {
    PATH_REPLAY_LEGACY_LENIENT = 0,
    PATH_REPLAY_STRICT_VALIDATE = 1
} PathReplayMode;

typedef enum {
    PATH_REPLAY_BOX_TARGET_SOLVER_RULE = 0,
    PATH_REPLAY_BOX_TARGET_HIDE_BOX_KEEP_TARGET,
    PATH_REPLAY_BOX_TARGET_SET_BOX_ON_TARGET,
    PATH_REPLAY_BOX_TARGET_MARKED_CLEAR_TARGET_HIDE_BOX
} PathReplayBoxTargetMode;

typedef enum {
    PATH_REPLAY_STEP_MOVED = 0,
    PATH_REPLAY_STEP_PAUSED,
    PATH_REPLAY_STEP_IGNORED,
    PATH_REPLAY_STEP_PUSHED_BOX,
    PATH_REPLAY_STEP_PUSHED_BOMB,
    PATH_REPLAY_STEP_BLASTED_WALL,
    PATH_REPLAY_STEP_STOPPED,
    PATH_REPLAY_STEP_ERROR
} PathReplayStepKind;

typedef struct {
    PathReplayMode mode;
    PathReplayBoxTargetMode box_target_mode;
    bool preserve_dynamic_tunnels_on_blast;
    int marked_box_idx;
    Position marked_target_pos;
} PathReplayOptions;

typedef struct {
    BitboardMap map;
    Position player;
    Position boxes[MAX_BOXES];
    Entity bombs[MAX_BOMBS];
    int bomb_count;
} PathReplayState;

typedef struct {
    PathReplayStepKind kind;
    Position player_pos;
    Position entity_from;
    Position entity_to;
    int entity_index;
    bool box_absorbed;
    PathReplayError error;
} PathReplayStepResult;

typedef struct {
    bool ok;
    PathReplayError error;
    uint16_t consumed_len;
    PathReplayState final_state;
} PathReplayResult;
PathReplayOptions path_replay_default_options(void);
void path_replay_init_state(PathReplayState* state);
bool path_replay_load_state(
    PathReplayState* state,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
);
void path_replay_init_result(PathReplayResult* out);
PathReplayStepResult path_replay_step(
    const struct SokobanSolver* solver,
    PathReplayState* state,
    Direction d,
    const PathReplayOptions* options
);
bool path_replay_run(
    const struct SokobanSolver* solver,
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
);
static inline uint16_t explosion_inner_mask_x(int x) {
    const uint16_t inner_col_mask = (uint16_t)((1u << (MAP_COLS - 1)) - 2u);

    if (x <= 0) return bit_mask_at(1);
    if (x >= MAP_COLS - 1) return bit_mask_at(MAP_COLS - 2);

    return (uint16_t)(((uint16_t)0x07 << (x - 1)) & inner_col_mask);
}

static inline void clear_explosion_walls(BitboardMap* bmap, Position center) {
    uint16_t clear_mask = (uint16_t)(~explosion_inner_mask_x(center.x));

    if (center.y > 1) bmap->walls[center.y - 1] &= clear_mask;
    if (center.y > 0 && center.y < MAP_ROWS - 1) bmap->walls[center.y] &= clear_mask;
    if (center.y < MAP_ROWS - 2) bmap->walls[center.y + 1] &= clear_mask;
}

static inline void set_box_bit(BitboardMap* bmap, int x, int y) {
    if (!get_bit(bmap->targets, x, y)) {
        set_bit(bmap->boxes, x, y);
    }
}

static inline void move_box_bit(BitboardMap* bmap, Position from, Position to) {
    clear_bit(bmap->boxes, from.x, from.y);
    set_box_bit(bmap, to.x, to.y);
}

static inline int tracked_position_index(const Position* positions, int count, Position pos) {
    for (int i = 0; i < count; i++) {
        if (pos_equal(positions[i], pos)) return i;
    }
    return -1;
}

static inline void apply_tracked_box_push_bits(
    BitboardMap* bmap,
    Position* tracked_positions,
    int tracked_idx,
    Position from,
    Position to,
    bool absorb_on_target
) {
    clear_bit(bmap->boxes, from.x, from.y);

    if (absorb_on_target && get_bit(bmap->targets, to.x, to.y)) {
        if (tracked_idx >= 0) tracked_positions[tracked_idx] = (Position){0xFF, 0xFF};
        clear_bit(bmap->targets, to.x, to.y);
        return;
    }

    set_bit(bmap->boxes, to.x, to.y);
    if (tracked_idx >= 0) tracked_positions[tracked_idx] = to;
}

#endif
