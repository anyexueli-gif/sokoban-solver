#include "astar.h"

#include <string.h>
#include <stdlib.h>

#define FAST_RAM_FUNC __attribute__((section("ITCM_NonCacheable")))
#define ALLOC_IN_OCRAM SOKOBAN_BSS_SECTION("OCRAM_CACHE")
#define ALLOC_IN_SDRAM_CACHE SOKOBAN_BSS_SECTION("SDRAM_CACHE")
#define ALLOC_IN_SDRAM ALLOC_IN_SDRAM_CACHE
#define ASTAR_STEP_COST 10
#define ASTAR_TURN_PENALTY 2
#define ASTAR_HEURISTIC_WEIGHT 10
#define ASTAR_DIR_MASK          0x07u
#define ASTAR_STEPS_SHIFT       3
#define ALLOC_IN_DTCM SOKOBAN_BSS_SECTION("RW_m_data")

#define ASTAR_TABLE_MEM ALLOC_IN_OCRAM

#define ASTAR_BUCKET_MEM ALLOC_IN_DTCM
#define MAX_F_COST              4096u
#define BUCKET_EMPTY            0xFFFFu
#define MAX_MAP_TILES           192
#define ASTAR_BOMB_ATTACK_DIST_STEP_LIMIT 100u
#define ASTAR_BOMB_ATTACK_DIST_MIN_RANGE 7u

#if (MAP_ROWS * MAP_COLS) > MAX_MAP_TILES
#error "MAX_MAP_TILES must cover every map tile."
#endif
// 方向数组定义
const Direction DIRECTIONS[4] = {
    {0, -1},  // 上
    {0, 1},   // 下
    {-1, 0},  // 左
    {1, 0}    // 右
};

#define IDX_POS_KEY(x, y) ((uint16_t)(((uint16_t)(x)) | ((uint16_t)(y) << 8)))
#define IDX_POS_ROW(y) \
    IDX_POS_KEY(0, y), IDX_POS_KEY(1, y), IDX_POS_KEY(2, y), IDX_POS_KEY(3, y), \
    IDX_POS_KEY(4, y), IDX_POS_KEY(5, y), IDX_POS_KEY(6, y), IDX_POS_KEY(7, y), \
    IDX_POS_KEY(8, y), IDX_POS_KEY(9, y), IDX_POS_KEY(10, y), IDX_POS_KEY(11, y)

static const uint16_t g_idx_pos_key[ASTAR_STATE_AXIS] = {
    IDX_POS_ROW(0),
    IDX_POS_ROW(1),
    IDX_POS_ROW(2),
    IDX_POS_ROW(3),
    IDX_POS_ROW(4),
    IDX_POS_ROW(5),
    IDX_POS_ROW(6),
    IDX_POS_ROW(7),
    IDX_POS_ROW(8),
    IDX_POS_ROW(9),
    IDX_POS_ROW(10),
    IDX_POS_ROW(11),
    IDX_POS_ROW(12),
    IDX_POS_ROW(13),
    IDX_POS_ROW(14),
    IDX_POS_ROW(15)
};

#undef IDX_POS_ROW
#undef IDX_POS_KEY

static inline Position idx_to_pos_fast(uint16_t idx) {
    uint16_t key = g_idx_pos_key[(uint8_t)idx];
    return (Position){(uint8_t)key, (uint8_t)(key >> 8)};
}

// 完美状态表：玩家下标与箱子下标组合，覆盖当前地图紧凑坐标状态。
static uint16_t g_fast_cost[MAX_MAP_TILES][MAX_MAP_TILES] ASTAR_TABLE_MEM;
static uint16_t g_fast_touched[ASTAR_TOUCHED_CAPACITY] ASTAR_TABLE_MEM;
static uint16_t g_fast_touched_count = 0;
static bool g_fast_touched_overflowed = false;
static bool g_fast_cost_initialized = false;


// 桶队列用于 A* 的 12 位有界 f 值范围。
static uint16_t g_bucket_heads[MAX_F_COST] ALLOC_IN_DTCM;
static uint32_t g_active_bucket_bits[MAX_F_COST / 32u] ALLOC_IN_DTCM;
static AStarNode g_bucket_nodes[MAX_HEAP_SIZE] ASTAR_BUCKET_MEM;
static uint16_t g_bucket_next[MAX_HEAP_SIZE] ASTAR_BUCKET_MEM;
static uint16_t g_bucket_alloc_idx = 0;
static uint16_t g_bucket_free_head = BUCKET_EMPTY;
static uint16_t g_bucket_current_min_f = 0;
static uint16_t g_astar_bomb_attack_dist[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static uint16_t g_astar_single_target_dist[MAP_ROWS][MAP_COLS] ALLOC_IN_OCRAM;
static Position g_astar_dist_queue[MAP_ROWS * MAP_COLS] ALLOC_IN_OCRAM;
static Direction g_astar_emit_path[MAX_SINGLE_PATH] ALLOC_IN_OCRAM;

#define ASTAR_BATCH_CLOSED_CAPACITY 16384u
#define ASTAR_BATCH_NO_PARENT UINT16_MAX
static uint16_t g_astar_batch_parent[ASTAR_BATCH_CLOSED_CAPACITY] ALLOC_IN_SDRAM;
static uint8_t g_astar_batch_dir_and_steps[ASTAR_BATCH_CLOSED_CAPACITY] ALLOC_IN_SDRAM;

/**
 * 清空状态表
 */
void hash_table_clear(void) {
    if (!g_fast_cost_initialized) {
        memset(g_fast_cost, 0xFF, sizeof(g_fast_cost));
        g_fast_cost_initialized = true;
        g_fast_touched_count = 0;
        g_fast_touched_overflowed = false;
        return;
    }

    if (g_fast_touched_overflowed) {
        memset(g_fast_cost, 0xFF, sizeof(g_fast_cost));
        g_fast_touched_overflowed = false;
    } else {
        for (uint16_t i = 0; i < g_fast_touched_count; i++) {
            uint16_t idx = g_fast_touched[i];
            g_fast_cost[idx / MAX_MAP_TILES][idx % MAX_MAP_TILES] = 0xFFFFu;
        }
    }
    g_fast_touched_count = 0;
}

void astar_reset_tables(void) {
    memset(g_fast_cost, 0xFF, sizeof(g_fast_cost));
    g_fast_cost_initialized = true;
    memset(g_fast_touched, 0, sizeof(g_fast_touched));
    g_fast_touched_count = 0;
    g_fast_touched_overflowed = false;

    memset(g_bucket_heads, 0xFF, sizeof(g_bucket_heads));
    memset(g_active_bucket_bits, 0, sizeof(g_active_bucket_bits));
    memset(g_bucket_nodes, 0, sizeof(g_bucket_nodes));
    memset(g_bucket_next, 0xFF, sizeof(g_bucket_next));
    g_bucket_alloc_idx = 0;
    g_bucket_free_head = BUCKET_EMPTY;
    g_bucket_current_min_f = MAX_F_COST;

    memset(g_astar_bomb_attack_dist, 0xFF, sizeof(g_astar_bomb_attack_dist));
    memset(g_astar_single_target_dist, 0xFF, sizeof(g_astar_single_target_dist));
    memset(g_astar_dist_queue, 0, sizeof(g_astar_dist_queue));
}

/**
 * 检查是否为更优路径，并写入当前世代
 */
static inline void remember_fast_touched(uint16_t p_idx, uint16_t b_idx) {
    if (g_fast_touched_overflowed) return;
    uint16_t idx = (uint16_t)(p_idx * MAX_MAP_TILES + b_idx);
    if (g_fast_touched_count < ASTAR_TOUCHED_CAPACITY) {
        g_fast_touched[g_fast_touched_count++] = idx;
        
    } else {
        g_fast_touched_overflowed = true;
        
    }
}

static inline bool is_better_path_and_add(uint16_t p_idx, uint16_t b_idx, uint16_t g_cost) {
    if (g_fast_cost[p_idx][b_idx] == 0xFFFFu) {
        remember_fast_touched(p_idx, b_idx);
        g_fast_cost[p_idx][b_idx] = g_cost;
        return true;
    }
    if (g_cost < g_fast_cost[p_idx][b_idx]) {
        g_fast_cost[p_idx][b_idx] = g_cost;
        return true;
    }
    return false;
}

static inline bool is_stale_path(uint16_t p_idx, uint16_t b_idx, uint16_t g_cost) {
    return g_fast_cost[p_idx][b_idx] == 0xFFFFu || g_cost > g_fast_cost[p_idx][b_idx];
}
// 桶队列实现。总代价字段为 12 位，优先级范围固定。
void heap_init(MinHeap* h, AStarNode* buffer, uint16_t max_size) {
    h->heap = buffer;
    h->heap_size = 0;
    h->max_size = max_size;

    g_bucket_alloc_idx = 0;
    g_bucket_free_head = BUCKET_EMPTY;
    g_bucket_current_min_f = MAX_F_COST;

    static bool first_init = true;
    if (first_init) {
        memset(g_bucket_heads, 0xFF, sizeof(g_bucket_heads));
        first_init = false;
    } else {
        for (uint16_t word = 0; word < (MAX_F_COST / 32u); word++) {
            uint32_t bits = g_active_bucket_bits[word];
            while (bits != 0) {
                uint16_t bit = (uint16_t)__builtin_ctz(bits);
                uint16_t bucket = (uint16_t)((word << 5) + bit);
                g_bucket_heads[bucket] = BUCKET_EMPTY;
                bits &= (bits - 1u);
            }
            g_active_bucket_bits[word] = 0;
        }
    }
}

FAST_RAM_FUNC void heap_push(MinHeap* h, const AStarNode* node) {
    
    if (h->heap_size >= h->max_size) return;
    
    if (node->f_cost >= MAX_F_COST) return;

    uint16_t idx;
    if (g_bucket_free_head != BUCKET_EMPTY) {
        idx = g_bucket_free_head;
        g_bucket_free_head = g_bucket_next[idx];
    } else {
        if (g_bucket_alloc_idx >= h->max_size) return;
        idx = g_bucket_alloc_idx++;
    }

    if (g_bucket_heads[node->f_cost] == BUCKET_EMPTY) {
        uint32_t bit = 1u << (node->f_cost & 31u);
        uint16_t word = (uint16_t)(node->f_cost >> 5);
        g_active_bucket_bits[word] |= bit;
    }

    g_bucket_nodes[idx] = *node;
    g_bucket_next[idx] = g_bucket_heads[node->f_cost];
    g_bucket_heads[node->f_cost] = idx;
    h->heap_size++;
    

    if (node->f_cost < g_bucket_current_min_f) {
        g_bucket_current_min_f = node->f_cost;
    }
}

FAST_RAM_FUNC bool heap_pop(MinHeap* h, AStarNode* out) {
    if (h->heap_size == 0) return false;
    

    while (g_bucket_current_min_f < MAX_F_COST &&
           g_bucket_heads[g_bucket_current_min_f] == BUCKET_EMPTY) {
        g_bucket_current_min_f++;
    }

    if (g_bucket_current_min_f >= MAX_F_COST) {
        h->heap_size = 0;
        return false;
    }

    uint16_t idx = g_bucket_heads[g_bucket_current_min_f];
    *out = g_bucket_nodes[idx];
    g_bucket_heads[g_bucket_current_min_f] = g_bucket_next[idx];

    g_bucket_next[idx] = g_bucket_free_head;
    g_bucket_free_head = idx;
    h->heap_size--;
    return true;
}
// A* 算法核心实现

/**
 * 检查位置是否被阻挡（位棋盘版本）
 * 参数 x：X坐标
 * 参数 y：Y坐标
 * 参数 bmap：位棋盘地图
 * 参数 collision_mask：碰撞掩码
 * 返回：true表示阻挡，false表示可通行
 */

static inline void bake_static_obstacles(uint16_t static_obs[MAP_ROWS], const BitboardMap* bmap, uint8_t collision_mask) {
    for (int r = 0; r < MAP_ROWS; r++) {
        uint16_t row = 0;

        if ((collision_mask & MASK_WALL) != 0) row |= bmap->walls[r];
        if ((collision_mask & MASK_BOMB) != 0) row |= bmap->bombs[r];
        if ((collision_mask & MASK_BOX) != 0) row |= bmap->boxes[r];

        static_obs[r] = row;
    }
}

static inline bool is_blocked_from_layer(const uint16_t static_obs[MAP_ROWS], Position pos) {
    return (static_obs[pos.y] & bit_mask_at(pos.x)) != 0;
}

static bool astar_reconstruct_path(const ClosedNode* closed_buffer, int current_index,
                                   Direction* out_path, uint16_t* out_len) {
    if (!closed_buffer || !closed_buffer->parent_index || !closed_buffer->dir_and_steps ||
        current_index < 0 || !out_path || !out_len) {
        return false;
    }

    int curr_idx = current_index;
    uint16_t path_length = 0;
    while (curr_idx != ASTAR_NO_PARENT && closed_buffer->parent_index[curr_idx] != ASTAR_NO_PARENT) {
        uint8_t steps = (uint8_t)(closed_buffer->dir_and_steps[curr_idx] >> ASTAR_STEPS_SHIFT);
        if ((uint32_t)path_length + steps > MAX_SINGLE_PATH) return false;
        path_length = (uint16_t)(path_length + steps);
        curr_idx = closed_buffer->parent_index[curr_idx];
    }

    *out_len = path_length;
    curr_idx = current_index;
    int write_idx = (int)path_length - 1;
    while (curr_idx != ASTAR_NO_PARENT && closed_buffer->parent_index[curr_idx] != ASTAR_NO_PARENT) {
        uint8_t steps = (uint8_t)(closed_buffer->dir_and_steps[curr_idx] >> ASTAR_STEPS_SHIFT);
        uint8_t dir_code = (uint8_t)(closed_buffer->dir_and_steps[curr_idx] & ASTAR_DIR_MASK);
        if (dir_code >= 4u) return false;
        Direction dir_obj = DIRECTIONS[dir_code];
        for (int s = 0; s < steps; s++) {
            if (write_idx < 0) return false;
            out_path[write_idx--] = dir_obj;
        }
        curr_idx = closed_buffer->parent_index[curr_idx];
    }

    return write_idx == -1;
}

static inline bool is_slide_tunnel_cell(const BitboardMap* bmap, Position pos, int dir) {
    uint16_t pos_bit = bit_mask_at(pos.x);
    if (dir == 2 || dir == 3) {
        return (bmap->h_tunnels[pos.y] & pos_bit) != 0;
    }
    return (bmap->v_tunnels[pos.y] & pos_bit) != 0;
}

static inline uint16_t min_target_manhattan(Position pos, Position* target_positions, int num_targets) {
    if (num_targets <= 1) {
        return manhattan_distance(pos, target_positions[0]);
    }

    uint16_t best = 0xFFFF;
    for (int t = 0; t < num_targets; t++) {
        uint16_t dist = manhattan_distance(pos, target_positions[t]);
        if (dist < best) {
            best = dist;
            if (best == 0) break;
        }
    }
    return best;
}

// A* 核心算法 - 通用实现

/**
 * 纯导航 A* 求解
 */
FAST_RAM_FUNC bool astar_navigate_mask(
    AStarNode* heap_buffer,
    ClosedNode* closed_buffer,
    const BitboardMap* bmap,
    Position start,
    Position target,
    uint8_t collision_mask,
    Direction* out_path,
    uint16_t* out_len
) {
    
    MinHeap heap;
    uint16_t static_obs[MAP_ROWS];
    heap_init(&heap, heap_buffer, MAX_HEAP_SIZE);
    bake_static_obstacles(static_obs, bmap, collision_mask);
    int closed_count = 0;
    int current_index = ASTAR_NO_PARENT;

    AStarNode start_node;
    start_node.player_idx = POS_TO_IDX(start);
    start_node.box_idx = 0;
    start_node.g_cost = 0;

    int dx = (start.x > target.x) ? (start.x - target.x) : (target.x - start.x);
    int dy = (start.y > target.y) ? (start.y - target.y) : (target.y - start.y);
    start_node.f_cost = (dx + dy) * ASTAR_HEURISTIC_WEIGHT;
    start_node.parent_index = ASTAR_NO_PARENT;
    start_node.dir_and_steps = (uint8_t)(4 | (1 << ASTAR_STEPS_SHIFT));

    is_better_path_and_add(start_node.player_idx, 0, 0);
    heap_push(&heap, &start_node);

    while (heap.heap_size > 0) {
        AStarNode current;
        heap_pop(&heap, &current);

        if (is_stale_path(current.player_idx, 0, current.g_cost)) continue;
        if (closed_count >= MAX_CLOSED_SIZE) break;

        closed_buffer->parent_index[closed_count] = current.parent_index;
        closed_buffer->dir_and_steps[closed_count] = current.dir_and_steps;
        current_index = closed_count;
        closed_count++;
        

        Position current_player = idx_to_pos_fast(current.player_idx);
        if (pos_equal(current_player, target)) goto found_nav_path;

        for (int dir = 0; dir < 4; dir++) {
            Position new_player = {current_player.x + DIRECTIONS[dir].dx, current_player.y + DIRECTIONS[dir].dy};

            if (is_blocked_from_layer(static_obs, new_player)) continue;

            uint16_t slide_steps = 0;
            Position slide_player = new_player;
            while (!pos_equal(slide_player, target) && is_slide_tunnel_cell(bmap, slide_player, dir)) {
                Position next_slide = {
                    (uint8_t)(slide_player.x + DIRECTIONS[dir].dx),
                    (uint8_t)(slide_player.y + DIRECTIONS[dir].dy)
                };
                if (is_blocked_from_layer(static_obs, next_slide)) break;

                slide_player = next_slide;
                slide_steps++;
            }
            new_player = slide_player;

            uint8_t current_dir = (uint8_t)(current.dir_and_steps & ASTAR_DIR_MASK);
            uint16_t turn_penalty = (current_dir != 4 && dir != current_dir) ? ASTAR_TURN_PENALTY : 0;
            uint16_t new_g = current.g_cost + ASTAR_STEP_COST + turn_penalty;
            new_g = (uint16_t)(new_g + (slide_steps * ASTAR_STEP_COST));
            uint8_t node_steps = (uint8_t)(1 + slide_steps);

            uint16_t new_p_idx = POS_TO_IDX(new_player);
            if (!is_better_path_and_add(new_p_idx, 0, new_g)) continue;

            AStarNode new_node;
            new_node.player_idx = new_p_idx;
            new_node.box_idx = 0;
            new_node.g_cost = new_g;
            int ndx = (new_player.x > target.x) ? (new_player.x - target.x) : (target.x - new_player.x);
            int ndy = (new_player.y > target.y) ? (new_player.y - target.y) : (target.y - new_player.y);
            new_node.f_cost = new_g + ((ndx + ndy) * ASTAR_HEURISTIC_WEIGHT);
            new_node.parent_index = current_index;
            new_node.dir_and_steps = (uint8_t)(dir | (node_steps << ASTAR_STEPS_SHIFT));
            if (heap.heap_size < MAX_HEAP_SIZE) heap_push(&heap, &new_node);
        }
    }

    return false;

found_nav_path:
    {
        return astar_reconstruct_path(closed_buffer, current_index, out_path, out_len);
    }
}

#define ASTAR_BOMB_REACH_ALL_MAX_TARGETS 64

FAST_RAM_FUNC int astar_bomb_reach_all_emit(
    AStarNode* heap_buffer,
    ClosedNode* closed_buffer,
    const BitboardMap* bmap,
    Position start_player,
    Position start_bomb,
    const Position* target_walls,
    const uint8_t* target_slots,
    int num_targets,
    AStarPathEmitFn emit,
    void* emit_ctx,
    uint16_t* out_lens
) {
    
    if (num_targets <= 0) return 0;
    if (num_targets > ASTAR_BOMB_REACH_ALL_MAX_TARGETS) num_targets = ASTAR_BOMB_REACH_ALL_MAX_TARGETS;

    int8_t target_index[MAP_ROWS][MAP_COLS];
    memset(target_index, 0xFF, sizeof(target_index));
    for (int i = 0; i < num_targets; i++) {
        Position t = target_walls[i];
        uint8_t slot = target_slots ? target_slots[i] : (uint8_t)i;
        out_lens[slot] = 0xFFFF;
        if (t.x > 0 && t.x < MAP_COLS - 1 && t.y > 0 && t.y < MAP_ROWS - 1) {
            target_index[t.y][t.x] = (int8_t)i;
        }
    }

    MinHeap heap;
    uint16_t static_obs[MAP_ROWS];
    heap_init(&heap, heap_buffer, MAX_HEAP_SIZE);
    bake_static_obstacles(static_obs, bmap, MASK_WALL | MASK_BOMB | MASK_BOX);
    hash_table_clear();

    int16_t hit_closed_idx[ASTAR_BOMB_REACH_ALL_MAX_TARGETS];
    uint8_t found_target[ASTAR_BOMB_REACH_ALL_MAX_TARGETS];
    for (int i = 0; i < num_targets; i++) {
        hit_closed_idx[i] = -1;
        found_target[i] = 0;
    }

    int closed_count = 0;
    int found_count = 0;

    AStarNode start_node;
    start_node.player_idx = POS_TO_IDX(start_player);
    start_node.box_idx = POS_TO_IDX(start_bomb);
    start_node.g_cost = 0;
    start_node.f_cost = 0;
    start_node.parent_index = ASTAR_NO_PARENT;
    start_node.dir_and_steps = (uint8_t)(4 | (1 << ASTAR_STEPS_SHIFT));

    is_better_path_and_add(start_node.player_idx, start_node.box_idx, 0);
    heap_push(&heap, &start_node);

    while (heap.heap_size > 0) {
        AStarNode current;
        heap_pop(&heap, &current);

        if (is_stale_path(current.player_idx, current.box_idx, current.g_cost)) continue;
        if (closed_count >= MAX_CLOSED_SIZE) break;

        closed_buffer->parent_index[closed_count] = current.parent_index;
        closed_buffer->dir_and_steps[closed_count] = current.dir_and_steps;
        int current_index = closed_count;
        closed_count++;
        

        Position current_player = idx_to_pos_fast(current.player_idx);
        Position current_box = idx_to_pos_fast(current.box_idx);

        int target_idx = target_index[current_box.y][current_box.x];
        if (target_idx >= 0) {
            if (!found_target[target_idx]) {
                found_target[target_idx] = 1;
                hit_closed_idx[target_idx] = (int16_t)current_index;
                found_count++;
                if (found_count >= num_targets) break;
            }
            continue;
        }

        for (int dir = 0; dir < 4; dir++) {
            Position new_player = {
                (uint8_t)(current_player.x + DIRECTIONS[dir].dx),
                (uint8_t)(current_player.y + DIRECTIONS[dir].dy)
            };
            Position new_box = current_box;
            uint16_t slide_steps = 0;

            if (is_blocked_from_layer(static_obs, new_player)) continue;

            if (pos_equal(new_player, current_box)) {
                new_box.x = (uint8_t)(new_box.x + DIRECTIONS[dir].dx);
                new_box.y = (uint8_t)(new_box.y + DIRECTIONS[dir].dy);

                int pushed_target_idx = target_index[new_box.y][new_box.x];
                bool box_blocked = is_blocked_from_layer(static_obs, new_box);
                if (pushed_target_idx >= 0) box_blocked = false;
                if (box_blocked) continue;

                Position slide_box = new_box;
                Position slide_player = new_player;
                while (is_slide_tunnel_cell(bmap, slide_box, dir)) {
                    Position next_slide_box = {
                        (uint8_t)(slide_box.x + DIRECTIONS[dir].dx),
                        (uint8_t)(slide_box.y + DIRECTIONS[dir].dy)
                    };
                    if (is_blocked_from_layer(static_obs, next_slide_box)) break;

                    slide_box = next_slide_box;
                    slide_player.x = (uint8_t)(slide_player.x + DIRECTIONS[dir].dx);
                    slide_player.y = (uint8_t)(slide_player.y + DIRECTIONS[dir].dy);
                    slide_steps++;
                }

                new_box = slide_box;
                new_player = slide_player;
            } else {
                Position slide_player = new_player;
                while (is_slide_tunnel_cell(bmap, slide_player, dir)) {
                    Position next_slide = {
                        (uint8_t)(slide_player.x + DIRECTIONS[dir].dx),
                        (uint8_t)(slide_player.y + DIRECTIONS[dir].dy)
                    };
                    if (pos_equal(next_slide, current_box)) break;
                    if (is_blocked_from_layer(static_obs, next_slide)) break;

                    slide_player = next_slide;
                    slide_steps++;
                }
                new_player = slide_player;
            }

            uint8_t current_dir = (uint8_t)(current.dir_and_steps & ASTAR_DIR_MASK);
            uint16_t turn_penalty = 0;
            if (current_dir != 4 && dir != current_dir) turn_penalty = ASTAR_TURN_PENALTY;

            uint16_t new_g = (uint16_t)(current.g_cost + ASTAR_STEP_COST + turn_penalty + (slide_steps * ASTAR_STEP_COST));
            if ((uint32_t)new_g >= ((uint32_t)g_astar_max_steps * ASTAR_STEP_COST)) continue;

            uint16_t new_p_idx = POS_TO_IDX(new_player);
            uint16_t new_b_idx = POS_TO_IDX(new_box);
            if (!is_better_path_and_add(new_p_idx, new_b_idx, new_g)) continue;

            AStarNode new_node;
            new_node.player_idx = new_p_idx;
            new_node.box_idx = new_b_idx;
            new_node.g_cost = new_g;
            new_node.f_cost = new_g;
            new_node.parent_index = current_index;
            new_node.dir_and_steps = (uint8_t)(dir | ((uint8_t)(1 + slide_steps) << ASTAR_STEPS_SHIFT));
            if (heap.heap_size < MAX_HEAP_SIZE) heap_push(&heap, &new_node);
        }
    }

    int emitted = 0;
    for (int t = 0; t < num_targets; t++) {
        if (hit_closed_idx[t] < 0) continue;

        uint16_t physical_path_length = 0;
        uint8_t slot = target_slots ? target_slots[t] : (uint8_t)t;
        Direction* out_path = g_astar_emit_path;
        if (!astar_reconstruct_path(closed_buffer, hit_closed_idx[t], out_path, &physical_path_length)) continue;

        out_lens[slot] = physical_path_length;
        if (emit) emit(slot, out_path, physical_path_length, emit_ctx);
        emitted++;
    }

    
    return emitted;
}
/**
 * A* 算法 - 支持两种推箱模式的通用实现
 * 使用位棋盘进行高效的碰撞检测和状态管理
 * 支持两种模式：mode 0=单箱单目标，mode 1=单箱多目标
 */
static void astar_bomb_reach_all_copy_emit(uint8_t slot, const Direction* path, uint16_t len, void* ctx) {
    Direction* out_paths = (Direction*)ctx;
    if (!out_paths || !path || len > MAX_SINGLE_PATH) return;
    memcpy(&out_paths[(uint16_t)slot * MAX_SINGLE_PATH], path, len * sizeof(Direction));
}

FAST_RAM_FUNC int astar_bomb_reach_all(
    AStarNode* heap_buffer,
    ClosedNode* closed_buffer,
    const BitboardMap* bmap,
    Position start_player,
    Position start_bomb,
    const Position* target_walls,
    const uint8_t* target_slots,
    int num_targets,
    Direction* out_paths,
    uint16_t* out_lens
) {
    return astar_bomb_reach_all_emit(heap_buffer, closed_buffer, bmap, start_player, start_bomb,
                                     target_walls, target_slots, num_targets,
                                     astar_bomb_reach_all_copy_emit, out_paths, out_lens);
}
static bool astar_batch_reconstruct(int current_index, Direction* out_path, uint16_t* out_len) {
    if (!out_path || !out_len || current_index < 0 ||
        (uint32_t)current_index >= ASTAR_BATCH_CLOSED_CAPACITY) {
        return false;
    }

    uint16_t path_length = 0;
    uint16_t current = (uint16_t)current_index;
    while (current != ASTAR_BATCH_NO_PARENT &&
           g_astar_batch_parent[current] != ASTAR_BATCH_NO_PARENT) {
        uint8_t steps = (uint8_t)(g_astar_batch_dir_and_steps[current] >> ASTAR_STEPS_SHIFT);
        if ((uint32_t)path_length + steps > MAX_SINGLE_PATH) return false;
        path_length = (uint16_t)(path_length + steps);
        current = g_astar_batch_parent[current];
    }

    *out_len = path_length;
    current = (uint16_t)current_index;
    int write_index = (int)path_length - 1;
    while (current != ASTAR_BATCH_NO_PARENT &&
           g_astar_batch_parent[current] != ASTAR_BATCH_NO_PARENT) {
        uint8_t packed = g_astar_batch_dir_and_steps[current];
        uint8_t steps = (uint8_t)(packed >> ASTAR_STEPS_SHIFT);
        uint8_t dir = (uint8_t)(packed & ASTAR_DIR_MASK);
        if (dir >= 4u) return false;
        for (uint8_t step = 0; step < steps; step++) {
            if (write_index < 0) return false;
            out_path[write_index--] = DIRECTIONS[dir];
        }
        current = g_astar_batch_parent[current];
    }
    return write_index == -1;
}

static int astar_batch_target_at(const int8_t target_by_tile[ASTAR_STATE_AXIS], Position box) {
    int8_t value = target_by_tile[POS_TO_IDX(box)];
    return value < 0 ? -1 : (int)value;
}

static void astar_batch_mark_incomplete(AStarBatchResult* out) {
    out->status = ASTAR_BATCH_INCOMPLETE;
    for (uint8_t i = 0; i < out->candidate_count; i++) {
        if (out->candidates[i].status != ASTAR_BATCH_TARGET_FOUND) {
            out->candidates[i].status = ASTAR_BATCH_TARGET_INCOMPLETE;
        }
    }
}

static bool astar_batch_push(MinHeap* heap, const AStarNode* node) {
    if (!heap || !node || heap->heap_size >= heap->max_size || node->f_cost >= MAX_F_COST) {
        return false;
    }
    heap_push(heap, node);
    return true;
}

FAST_RAM_FUNC AStarBatchStatus astar_solve_single_box_targets_mask(
    const BitboardMap* bmap,
    Position start_player,
    Position start_box,
    const Position* target_positions,
    const uint8_t* target_slots,
    uint8_t num_targets,
    uint8_t collision_mask,
    uint32_t capacity_limit,
    AStarBatchResult* out
) {
    if (!out) return ASTAR_BATCH_FATAL;
    memset(out, 0, sizeof(*out));
    out->status = ASTAR_BATCH_FATAL;
    out->candidate_count = num_targets > MAX_TARGETS ? MAX_TARGETS : num_targets;

    for (uint8_t i = 0; i < out->candidate_count; i++) {
        out->candidates[i].target_slot = target_slots ? target_slots[i] : i;
        out->candidates[i].status = ASTAR_BATCH_TARGET_INCOMPLETE;
        out->candidates[i].path_len = UINT16_MAX;
    }

    if (!bmap || !target_positions || num_targets == 0 || num_targets > MAX_TARGETS ||
        start_player.x == 0 || start_player.x >= MAP_COLS - 1 ||
        start_player.y == 0 || start_player.y >= MAP_ROWS - 1 ||
        start_box.x == 0 || start_box.x >= MAP_COLS - 1 ||
        start_box.y == 0 || start_box.y >= MAP_ROWS - 1) {
        return ASTAR_BATCH_FATAL;
    }

    uint32_t state_limit = capacity_limit;
    if (state_limit == 0 || state_limit > (uint32_t)ASTAR_STATE_AXIS * ASTAR_STATE_AXIS) {
        state_limit = (uint32_t)ASTAR_STATE_AXIS * ASTAR_STATE_AXIS;
    }
    uint32_t closed_limit = state_limit < ASTAR_BATCH_CLOSED_CAPACITY
        ? state_limit : ASTAR_BATCH_CLOSED_CAPACITY;

    BitboardMap route_map = *bmap;
    memset(route_map.targets, 0, sizeof(route_map.targets));
    int8_t target_by_tile[ASTAR_STATE_AXIS];
    memset(target_by_tile, 0xFF, sizeof(target_by_tile));
    for (uint8_t i = 0; i < num_targets; i++) {
        Position target = target_positions[i];
        if (target.x == 0 || target.x >= MAP_COLS - 1 ||
            target.y == 0 || target.y >= MAP_ROWS - 1) {
            return ASTAR_BATCH_FATAL;
        }
        uint8_t tile = POS_TO_IDX(target);
        if (target_by_tile[tile] >= 0) return ASTAR_BATCH_FATAL;
        target_by_tile[tile] = (int8_t)i;
        set_bit(route_map.targets, target.x, target.y);
    }

    MinHeap heap;
    uint16_t static_obs[MAP_ROWS];
    heap_init(&heap, NULL, MAX_HEAP_SIZE);
    bake_static_obstacles(static_obs, &route_map, collision_mask);
    hash_table_clear();

    memset(g_astar_single_target_dist, 0xFF, sizeof(g_astar_single_target_dist));
    uint16_t distance_head = 0;
    uint16_t distance_tail = 0;
    for (uint8_t i = 0; i < num_targets; i++) {
        Position target = target_positions[i];
        if ((static_obs[target.y] & bit_mask_at(target.x)) != 0) continue;
        if (g_astar_single_target_dist[target.y][target.x] != UINT16_MAX) continue;
        g_astar_single_target_dist[target.y][target.x] = 0;
        g_astar_dist_queue[distance_tail++] = target;
    }
    while (distance_head < distance_tail) {
        Position current = g_astar_dist_queue[distance_head++];
        uint16_t next_distance = (uint16_t)(g_astar_single_target_dist[current.y][current.x] + 1u);
        for (uint8_t dir = 0; dir < 4; dir++) {
            int nx = (int)current.x + DIRECTIONS[dir].dx;
            int ny = (int)current.y + DIRECTIONS[dir].dy;
            if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
            if (g_astar_single_target_dist[ny][nx] != UINT16_MAX) continue;
            if ((static_obs[ny] & bit_mask_at(nx)) != 0) continue;
            g_astar_single_target_dist[ny][nx] = next_distance;
            g_astar_dist_queue[distance_tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
        }
    }

    uint16_t start_h = g_astar_single_target_dist[start_box.y][start_box.x];
    if (start_h == UINT16_MAX) {
        out->status = ASTAR_BATCH_COMPLETE;
        for (uint8_t i = 0; i < num_targets; i++) {
            out->candidates[i].status = ASTAR_BATCH_TARGET_PROVED_NO_PATH;
        }
        return ASTAR_BATCH_COMPLETE;
    }

    bool edge_target_l = false;
    bool edge_target_r = false;
    bool edge_target_u = false;
    bool edge_target_d = false;
    for (uint8_t i = 0; i < num_targets; i++) {
        edge_target_l = edge_target_l || target_positions[i].x == 1;
        edge_target_r = edge_target_r || target_positions[i].x == MAP_COLS - 2;
        edge_target_u = edge_target_u || target_positions[i].y == 1;
        edge_target_d = edge_target_d || target_positions[i].y == MAP_ROWS - 2;
    }

    AStarNode start_node;
    memset(&start_node, 0, sizeof(start_node));
    start_node.player_idx = POS_TO_IDX(start_player);
    start_node.box_idx = POS_TO_IDX(start_box);
    start_node.g_cost = 0;
    start_node.f_cost = (uint16_t)(start_h * ASTAR_HEURISTIC_WEIGHT);
    start_node.parent_index = ASTAR_BATCH_NO_PARENT;
    start_node.dir_and_steps = (uint8_t)(4u | (1u << ASTAR_STEPS_SHIFT));
    if (!is_better_path_and_add(start_node.player_idx, start_node.box_idx, 0) ||
        !astar_batch_push(&heap, &start_node)) {
        astar_batch_mark_incomplete(out);
        return ASTAR_BATCH_INCOMPLETE;
    }

    uint32_t admitted_states = 1;
    int closed_count = 0;
    while (heap.heap_size > 0) {
        AStarNode current;
        if (!heap_pop(&heap, &current)) {
            astar_batch_mark_incomplete(out);
            return ASTAR_BATCH_INCOMPLETE;
        }
        if (is_stale_path(current.player_idx, current.box_idx, current.g_cost)) continue;
        if ((uint32_t)closed_count >= closed_limit) {
            astar_batch_mark_incomplete(out);
            return ASTAR_BATCH_INCOMPLETE;
        }

        g_astar_batch_parent[closed_count] = current.parent_index;
        g_astar_batch_dir_and_steps[closed_count] = current.dir_and_steps;
        int current_index = closed_count++;
        Position current_player = idx_to_pos_fast(current.player_idx);
        Position current_box = idx_to_pos_fast(current.box_idx);

        int target_index = astar_batch_target_at(target_by_tile, current_box);
        if (target_index >= 0) {
            AStarBatchCandidate* candidate = &out->candidates[target_index];
            if (candidate->status != ASTAR_BATCH_TARGET_FOUND) {
                uint16_t path_len = 0;
                if (!astar_batch_reconstruct(current_index, candidate->path, &path_len)) {
                    astar_batch_mark_incomplete(out);
                    return ASTAR_BATCH_INCOMPLETE;
                }
                candidate->status = ASTAR_BATCH_TARGET_FOUND;
                candidate->path_len = path_len;
                out->found_count++;
            }
            if (out->found_count == num_targets) {
                out->status = ASTAR_BATCH_COMPLETE;
                return ASTAR_BATCH_COMPLETE;
            }
            continue;
        }

        for (uint8_t dir = 0; dir < 4; dir++) {
            Position new_player = {
                (uint8_t)(current_player.x + DIRECTIONS[dir].dx),
                (uint8_t)(current_player.y + DIRECTIONS[dir].dy)
            };
            Position new_box = current_box;
            uint16_t slide_steps = 0;
            if (is_blocked_from_layer(static_obs, new_player)) continue;

            if (pos_equal(new_player, current_box)) {
                new_box.x = (uint8_t)(current_box.x + DIRECTIONS[dir].dx);
                new_box.y = (uint8_t)(current_box.y + DIRECTIONS[dir].dy);
                if (is_blocked_from_layer(static_obs, new_box)) continue;

                Position slide_box = new_box;
                Position slide_player = new_player;
                while (astar_batch_target_at(target_by_tile, slide_box) < 0 &&
                       is_slide_tunnel_cell(&route_map, slide_box, dir)) {
                    Position next_slide_box = {
                        (uint8_t)(slide_box.x + DIRECTIONS[dir].dx),
                        (uint8_t)(slide_box.y + DIRECTIONS[dir].dy)
                    };
                    if (is_blocked_from_layer(static_obs, next_slide_box)) break;
                    slide_box = next_slide_box;
                    slide_player.x = (uint8_t)(slide_player.x + DIRECTIONS[dir].dx);
                    slide_player.y = (uint8_t)(slide_player.y + DIRECTIONS[dir].dy);
                    slide_steps++;
                }
                new_box = slide_box;
                new_player = slide_player;

                bool is_on_target = astar_batch_target_at(target_by_tile, new_box) >= 0;
                if (!is_on_target) {
                    if ((route_map.deadlocks[new_box.y] & bit_mask_at(new_box.x)) != 0) continue;
                    if ((new_box.x == 1 && !edge_target_l) ||
                        (new_box.x == MAP_COLS - 2 && !edge_target_r) ||
                        (new_box.y == 1 && !edge_target_u) ||
                        (new_box.y == MAP_ROWS - 2 && !edge_target_d)) {
                        continue;
                    }

                    uint8_t bx = new_box.x;
                    uint8_t by = new_box.y;
                    uint16_t box_bit = bit_mask_at(bx);
                    uint16_t left_bit = bit_mask_at(bx - 1);
                    uint16_t right_bit = bit_mask_at(bx + 1);
                    uint16_t obs_curr = static_obs[by] | box_bit;
                    uint16_t obs_up = static_obs[by - 1];
                    uint16_t obs_down = static_obs[by + 1];
                    if ((obs_curr & left_bit) && (obs_up & box_bit) && (obs_up & left_bit)) continue;
                    if ((obs_curr & right_bit) && (obs_up & box_bit) && (obs_up & right_bit)) continue;
                    if ((obs_curr & left_bit) && (obs_down & box_bit) && (obs_down & left_bit)) continue;
                    if ((obs_curr & right_bit) && (obs_down & box_bit) && (obs_down & right_bit)) continue;
                }
            } else {
                Position slide_player = new_player;
                while (is_slide_tunnel_cell(&route_map, slide_player, dir)) {
                    Position next_slide = {
                        (uint8_t)(slide_player.x + DIRECTIONS[dir].dx),
                        (uint8_t)(slide_player.y + DIRECTIONS[dir].dy)
                    };
                    if (pos_equal(next_slide, current_box) ||
                        is_blocked_from_layer(static_obs, next_slide)) break;
                    slide_player = next_slide;
                    slide_steps++;
                }
                new_player = slide_player;
            }

            uint8_t current_dir = (uint8_t)(current.dir_and_steps & ASTAR_DIR_MASK);
            uint16_t turn_penalty =
                (current_dir != 4u && current_dir != dir) ? ASTAR_TURN_PENALTY : 0u;
            uint16_t new_g = (uint16_t)(
                current.g_cost + ASTAR_STEP_COST + turn_penalty +
                slide_steps * ASTAR_STEP_COST
            );
            if ((uint32_t)new_g >= (uint32_t)g_astar_max_steps * ASTAR_STEP_COST) continue;

            uint16_t new_player_idx = POS_TO_IDX(new_player);
            uint16_t new_box_idx = POS_TO_IDX(new_box);
            bool unseen = g_fast_cost[new_player_idx][new_box_idx] == UINT16_MAX;
            if (unseen && admitted_states >= state_limit) {
                astar_batch_mark_incomplete(out);
                return ASTAR_BATCH_INCOMPLETE;
            }
            if (!is_better_path_and_add(new_player_idx, new_box_idx, new_g)) continue;
            if (unseen) admitted_states++;

            uint16_t h_cost = astar_batch_target_at(target_by_tile, new_box) >= 0
                ? 0u : g_astar_single_target_dist[new_box.y][new_box.x];
            if (h_cost == UINT16_MAX) continue;
            if (g_astar_max_steps != UINT16_MAX &&
                (uint32_t)new_g + (uint32_t)h_cost * ASTAR_STEP_COST >=
                    (uint32_t)g_astar_max_steps * ASTAR_STEP_COST) {
                continue;
            }

            AStarNode new_node;
            memset(&new_node, 0, sizeof(new_node));
            new_node.player_idx = (uint8_t)new_player_idx;
            new_node.box_idx = (uint8_t)new_box_idx;
            new_node.g_cost = new_g;
            new_node.f_cost = (uint16_t)(new_g + h_cost * ASTAR_HEURISTIC_WEIGHT);
            new_node.parent_index = (uint16_t)current_index;
            new_node.dir_and_steps =
                (uint8_t)(dir | ((uint8_t)(1u + slide_steps) << ASTAR_STEPS_SHIFT));
            if (!astar_batch_push(&heap, &new_node)) {
                astar_batch_mark_incomplete(out);
                return ASTAR_BATCH_INCOMPLETE;
            }
        }
    }

    out->status = ASTAR_BATCH_COMPLETE;
    for (uint8_t i = 0; i < num_targets; i++) {
        if (out->candidates[i].status != ASTAR_BATCH_TARGET_FOUND) {
            out->candidates[i].status = ASTAR_BATCH_TARGET_PROVED_NO_PATH;
        }
    }
    return ASTAR_BATCH_COMPLETE;
}

FAST_RAM_FUNC bool astar_solve_with_mask(
    AStarNode* heap_buffer,
    ClosedNode* closed_buffer,
    const BitboardMap* bmap,
    Position start_player,
    Position start_box,
    Position* target_positions,
    int num_targets,
    int* reached_target_idx,
    uint8_t collision_mask,
    Direction* out_path,
    uint16_t* out_len,
    int macro_depth,
    AStarRouteType route_type
) {
    
    MinHeap heap;
    uint16_t static_obs[MAP_ROWS];
    heap_init(&heap, heap_buffer, MAX_HEAP_SIZE);
    bake_static_obstacles(static_obs, bmap, collision_mask);

    bool edge_target_L = false, edge_target_R = false, edge_target_U = false, edge_target_D = false;
    if (route_type == ROUTE_BOX_NORMAL || route_type == ROUTE_SUPER_EVAC) {
        for (int y = 1; y < MAP_ROWS - 1; y++) {
            if ((bmap->targets[y] & bit_mask_at(1)) != 0) edge_target_L = true;
            if ((bmap->targets[y] & bit_mask_at(MAP_COLS - 2)) != 0) edge_target_R = true;
        }
        for (int x = 1; x < MAP_COLS - 1; x++) {
            if ((bmap->targets[1] & bit_mask_at(x)) != 0) edge_target_U = true;
            if ((bmap->targets[MAP_ROWS - 2] & bit_mask_at(x)) != 0) edge_target_D = true;
        }
    }

    uint16_t (*bomb_attack_dist)[MAP_COLS] = g_astar_bomb_attack_dist;
    bool use_bomb_attack_dist = false;
    uint16_t bomb_attack_start_manhattan = 0;
    if (route_type == ROUTE_BOMB_ATTACK && num_targets == 1) {
        bomb_attack_start_manhattan = manhattan_distance(start_box, target_positions[0]);
    }

    uint16_t (*single_target_dist)[MAP_COLS] = g_astar_single_target_dist;
    bool use_single_target_dist = false;
    if (route_type == ROUTE_BOX_NORMAL && num_targets == 1) {
        memset(single_target_dist, 0xFF, sizeof(g_astar_single_target_dist));
        Position* q = g_astar_dist_queue;
        int head = 0, tail = 0;
        Position target = target_positions[0];
        if (target.x > 0 && target.x < MAP_COLS - 1 && target.y > 0 && target.y < MAP_ROWS - 1 &&
            (static_obs[target.y] & bit_mask_at(target.x)) == 0) {
            single_target_dist[target.y][target.x] = 0;
            q[tail++] = target;
            use_single_target_dist = true;
        }
        while (head < tail) {
            Position curr = q[head++];
            uint16_t next_dist = (uint16_t)(single_target_dist[curr.y][curr.x] + 1);
            for (int d = 0; d < 4; d++) {
                int nx = curr.x + DIRECTIONS[d].dx;
                int ny = curr.y + DIRECTIONS[d].dy;
                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                if (single_target_dist[ny][nx] != 0xFFFF) continue;
                if ((static_obs[ny] & bit_mask_at(nx)) != 0) continue;
                single_target_dist[ny][nx] = next_dist;
                q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
            }
        }
    }
    if (route_type == ROUTE_BOMB_ATTACK && num_targets == 1 && g_astar_max_steps < ASTAR_BOMB_ATTACK_DIST_STEP_LIMIT && bomb_attack_start_manhattan >= ASTAR_BOMB_ATTACK_DIST_MIN_RANGE) {
        memset(bomb_attack_dist, 0xFF, sizeof(g_astar_bomb_attack_dist));
        Position* q = g_astar_dist_queue;
        int head = 0, tail = 0;
        Position target = target_positions[0];
        if (target.x > 0 && target.x < MAP_COLS - 1 && target.y > 0 && target.y < MAP_ROWS - 1) {
            bomb_attack_dist[target.y][target.x] = 0;
            q[tail++] = target;
            use_bomb_attack_dist = true;
        }
        while (head < tail) {
            Position curr = q[head++];
            uint16_t next_dist = (uint16_t)(bomb_attack_dist[curr.y][curr.x] + 1);
            for (int d = 0; d < 4; d++) {
                int nx = curr.x + DIRECTIONS[d].dx;
                int ny = curr.y + DIRECTIONS[d].dy;
                if (nx <= 0 || nx >= MAP_COLS - 1 || ny <= 0 || ny >= MAP_ROWS - 1) continue;
                if (bomb_attack_dist[ny][nx] != 0xFFFF) continue;
                if ((static_obs[ny] & bit_mask_at(nx)) != 0) continue;
                bomb_attack_dist[ny][nx] = next_dist;
                q[tail++] = (Position){(uint8_t)nx, (uint8_t)ny};
            }
        }
    }

    int closed_count = 0;
    int current_index = ASTAR_NO_PARENT;

    AStarNode start_node;
    start_node.player_idx = POS_TO_IDX(start_player);
    start_node.box_idx = POS_TO_IDX(start_box);
    start_node.g_cost = 0;

    uint16_t start_h_cost;
    if (route_type == ROUTE_BOMB_ATTACK) {
        start_h_cost = use_bomb_attack_dist ? bomb_attack_dist[start_box.y][start_box.x]
                                            : manhattan_distance(start_box, target_positions[0]);
        if (start_h_cost == 0xFFFF) return false;
    } else if (use_single_target_dist) {
        start_h_cost = single_target_dist[start_box.y][start_box.x];
        if (start_h_cost == 0xFFFF) return false;
    } else if (macro_depth == ASTAR_NO_MACRO_DEPTH) {
        start_h_cost = min_target_manhattan(start_box, target_positions, num_targets);
    } else {
        if (macro_depth < 0 || macro_depth >= MAX_MACRO_DEPTH) return false;
        start_h_cost = g_macro_dist_field[macro_depth][start_box.y][start_box.x];
        if (start_h_cost == 0xFFFF) return false;
    }
    start_node.f_cost = start_h_cost * ASTAR_HEURISTIC_WEIGHT;
    start_node.parent_index = ASTAR_NO_PARENT;
    start_node.dir_and_steps = (uint8_t)(4 | (1 << ASTAR_STEPS_SHIFT));

    // 起始节点加入状态表
    is_better_path_and_add(start_node.player_idx, start_node.box_idx, 0);
    heap_push(&heap, &start_node);

    while (heap.heap_size > 0) {
        AStarNode current;
        heap_pop(&heap, &current);

        // 弹栈防御：直接使用绝对状态表索引。
        if (is_stale_path(current.player_idx, current.box_idx, current.g_cost)) {
            continue;
        }

        // 防止闭表溢出
        if (closed_count >= MAX_CLOSED_SIZE) break;

        // 将数据写入独立的 4 字节结构体数组
        closed_buffer->parent_index[closed_count] = current.parent_index;
        closed_buffer->dir_and_steps[closed_count] = current.dir_and_steps;
        current_index = closed_count;
        closed_count++;
        

        Position current_player = idx_to_pos_fast(current.player_idx);
        Position current_box = idx_to_pos_fast(current.box_idx);

        // 单目标直接 O(1) 判定，多目标保持线性命中检测
        if (num_targets == 1) {
            if (pos_equal(current_box, target_positions[0])) {
                if (reached_target_idx) *reached_target_idx = 0;
                goto found_path;
            }
        } else {
            for (int i = 0; i < num_targets; i++) {
                if (pos_equal(current_box, target_positions[i])) {
                    if (reached_target_idx) *reached_target_idx = i;
                    goto found_path;
                }
            }
        }

        for (int dir = 0; dir < 4; dir++) {
            Position new_player = {current_player.x + DIRECTIONS[dir].dx, current_player.y + DIRECTIONS[dir].dy};
            Position new_box = current_box;
            uint16_t slide_steps = 0;

            if (is_blocked_from_layer(static_obs, new_player)) continue;

            if (pos_equal(new_player, current_box)) {
                new_box.x += DIRECTIONS[dir].dx;
                new_box.y += DIRECTIONS[dir].dy;

                bool box_blocked = is_blocked_from_layer(static_obs, new_box);

                // 仅在炸弹爆破模式下允许撞入指定目标墙。
                if (route_type == ROUTE_BOMB_ATTACK) {
                    for (int i = 0; i < num_targets; i++) {
                        if (pos_equal(new_box, target_positions[i])) {
                            box_blocked = false;
                            break;
                        }
                    }
                }

                if (box_blocked) continue;

                Position slide_box = new_box;
                Position slide_player = new_player;
                while (is_slide_tunnel_cell(bmap, slide_box, dir)) {
                    Position next_slide_box = {
                        (uint8_t)(slide_box.x + DIRECTIONS[dir].dx),
                        (uint8_t)(slide_box.y + DIRECTIONS[dir].dy)
                    };
                    if (is_blocked_from_layer(static_obs, next_slide_box)) break;

                    slide_box = next_slide_box;
                    slide_player.x = (uint8_t)(slide_player.x + DIRECTIONS[dir].dx);
                    slide_player.y = (uint8_t)(slide_player.y + DIRECTIONS[dir].dy);
                    slide_steps++;
                }

                new_box = slide_box;
                new_player = slide_player;

                if (route_type == ROUTE_BOX_NORMAL || route_type == ROUTE_SUPER_EVAC) {
                    bool is_on_target = (bmap->targets[new_box.y] & bit_mask_at(new_box.x)) != 0;

                    if (!is_on_target) {
                        if (route_type != ROUTE_SUPER_EVAC) {
                            if ((bmap->deadlocks[new_box.y] & bit_mask_at(new_box.x)) != 0) continue;
                        }

                        if ((new_box.x == 1 && !edge_target_L) ||
                            (new_box.x == MAP_COLS - 2 && !edge_target_R) ||
                            (new_box.y == 1 && !edge_target_U) ||
                            (new_box.y == MAP_ROWS - 2 && !edge_target_D)) {
                            continue;
                        }

                        int bx = new_box.x, by = new_box.y;
                        uint16_t box_bit = bit_mask_at(bx);
                        uint16_t left_bit = bit_mask_at(bx - 1);
                        uint16_t right_bit = bit_mask_at(bx + 1);

                        uint16_t obs_curr = static_obs[by] | box_bit;
                        uint16_t obs_up   = static_obs[by - 1];
                        uint16_t obs_dn   = static_obs[by + 1];

                        bool super_route = (route_type == ROUTE_SUPER_EVAC);
                        uint16_t dyn_curr = bmap->boxes[by];
                        uint16_t dyn_up   = bmap->boxes[by - 1];
                        uint16_t dyn_dn   = bmap->boxes[by + 1];

                        if ((obs_curr & left_bit) && (obs_up & box_bit) && (obs_up & left_bit)) {
                            if (!super_route || (dyn_curr & left_bit) || (dyn_up & box_bit) || (dyn_up & left_bit)) continue;
                        }
                        if ((obs_curr & right_bit) && (obs_up & box_bit) && (obs_up & right_bit)) {
                            if (!super_route || (dyn_curr & right_bit) || (dyn_up & box_bit) || (dyn_up & right_bit)) continue;
                        }
                        if ((obs_curr & left_bit) && (obs_dn & box_bit) && (obs_dn & left_bit)) {
                            if (!super_route || (dyn_curr & left_bit) || (dyn_dn & box_bit) || (dyn_dn & left_bit)) continue;
                        }
                        if ((obs_curr & right_bit) && (obs_dn & box_bit) && (obs_dn & right_bit)) {
                            if (!super_route || (dyn_curr & right_bit) || (dyn_dn & box_bit) || (dyn_dn & right_bit)) continue;
                        }
                    }
                }
            } else {
                Position slide_player = new_player;
                while (is_slide_tunnel_cell(bmap, slide_player, dir)) {
                    Position next_slide = {
                        (uint8_t)(slide_player.x + DIRECTIONS[dir].dx),
                        (uint8_t)(slide_player.y + DIRECTIONS[dir].dy)
                    };
                    if (pos_equal(next_slide, current_box)) break;
                    if (is_blocked_from_layer(static_obs, next_slide)) break;

                    slide_player = next_slide;
                    slide_steps++;
                }
                new_player = slide_player;
            }

            uint8_t current_dir = (uint8_t)(current.dir_and_steps & ASTAR_DIR_MASK);
            uint16_t step_cost = ASTAR_STEP_COST;
            uint16_t turn_penalty = 0;
            if (current_dir != 4 && dir != current_dir) {
                turn_penalty = ASTAR_TURN_PENALTY;
            }

            uint16_t new_g = current.g_cost + step_cost + turn_penalty;
            uint8_t node_steps = (uint8_t)(1 + slide_steps);
            new_g = (uint16_t)(new_g + (slide_steps * ASTAR_STEP_COST));

            // 超过步数预算时剪枝。
            if ((uint32_t)new_g >= ((uint32_t)g_astar_max_steps * ASTAR_STEP_COST)) {
                continue;
            }

            // 绝对状态表保留更低代价路径。
            uint16_t new_p_idx = POS_TO_IDX(new_player);
            uint16_t new_b_idx = POS_TO_IDX(new_box);
            if (!is_better_path_and_add(new_p_idx, new_b_idx, new_g)) {
                continue;
            }

            uint16_t h_cost;
            if (route_type == ROUTE_BOMB_ATTACK) {
                h_cost = use_bomb_attack_dist ? bomb_attack_dist[new_box.y][new_box.x]
                                              : manhattan_distance(new_box, target_positions[0]);
                if (h_cost == 0xFFFF) continue;
            } else if (use_single_target_dist) {
                h_cost = single_target_dist[new_box.y][new_box.x];
                if (h_cost == 0xFFFF) continue;
            } else if (macro_depth == ASTAR_NO_MACRO_DEPTH) {
                h_cost = min_target_manhattan(new_box, target_positions, num_targets);
            } else {
                h_cost = g_macro_dist_field[macro_depth][new_box.y][new_box.x];
                if (h_cost == 0xFFFF) continue;
            }

            if (g_astar_max_steps != 0xFFFFu &&
                (uint32_t)new_g + ((uint32_t)h_cost * ASTAR_STEP_COST) >= ((uint32_t)g_astar_max_steps * ASTAR_STEP_COST)) {
                continue;
            }

            AStarNode new_node;
            new_node.player_idx = new_p_idx;
            new_node.box_idx = new_b_idx;
            new_node.g_cost = new_g;

            // A* 评分：f = g + h * 权重。
            new_node.f_cost = new_g + (h_cost * ASTAR_HEURISTIC_WEIGHT);
            new_node.parent_index = current_index;
            new_node.dir_and_steps = (uint8_t)(dir | (node_steps << ASTAR_STEPS_SHIFT));

// 在堆容量范围内安全入堆
            if (heap.heap_size < MAX_HEAP_SIZE) {
                heap_push(&heap, &new_node);
            }
        }
    }
    
    return false;

found_path:
    {
        return astar_reconstruct_path(closed_buffer, current_index, out_path, out_len);
    }
}
