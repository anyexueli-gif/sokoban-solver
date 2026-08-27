#ifndef ASTAR_H
#define ASTAR_H

#include "sokoban_types.h"

#define MAX_HEAP_SIZE 2048     // 堆大小减半
#define MAX_CLOSED_SIZE 5000   // 新增闭表独立容量
#define MAX_SINGLE_PATH 128    // 单次路径最大长度
#define ASTAR_TOUCHED_CAPACITY 7168u // A* fast touched table capacity
#define ASTAR_MAX_BLOCKING MAX_BOXES
#define ASTAR_NO_PARENT 8191
#define MAX_MACRO_DEPTH (MAX_BOMBS + 1)
#define ASTAR_NO_MACRO_DEPTH (-1)

#define ASTAR_STATE_AXIS  192  // 16x12 紧凑坐标轴：地图行列乘积为 192

#if (MAP_ROWS * MAP_COLS) != ASTAR_STATE_AXIS
#error "ASTAR_STATE_AXIS must match MAP_ROWS * MAP_COLS for compact A* indices."
#endif

#if ASTAR_STATE_AXIS > 255
#error "A* compact indices must fit in uint8_t."
#endif

typedef enum {
    ROUTE_NAV_ONLY = 0, // 纯玩家导航
    ROUTE_BOX_NORMAL,   // 常规推箱子
    ROUTE_BOMB_ATTACK,  // 炸弹爆破路线
    ROUTE_SUPER_EVAC
} AStarRouteType;

typedef struct {
    uint16_t f_cost;
    uint16_t g_cost;
    uint16_t parent_index;
    uint8_t dir_and_steps;
    uint8_t player_idx;
    uint8_t box_idx;
    uint8_t reserved8;
    uint16_t reserved16;
} __attribute__((aligned(4))) AStarNode;

_Static_assert(sizeof(AStarNode) == 12, "AStarNode explicit layout must stay 12 bytes");

// 闭表回溯数据拆分存放，减少结构填充占用。
typedef struct {
    uint16_t* parent_index;
    uint8_t* dir_and_steps;
} ClosedNode;

typedef enum {
    ASTAR_BATCH_COMPLETE = 0,
    ASTAR_BATCH_INCOMPLETE = 1,
    ASTAR_BATCH_FATAL = 2
} AStarBatchStatus;

typedef enum {
    ASTAR_BATCH_TARGET_FOUND = 0,
    ASTAR_BATCH_TARGET_PROVED_NO_PATH = 1,
    ASTAR_BATCH_TARGET_INCOMPLETE = 2
} AStarBatchTargetStatus;

typedef struct {
    uint8_t target_slot;
    uint8_t status;
    uint16_t path_len;
    Direction path[MAX_SINGLE_PATH];
} AStarBatchCandidate;

typedef struct {
    uint8_t status;
    uint8_t candidate_count;
    uint8_t found_count;
    uint8_t reserved;
    AStarBatchCandidate candidates[MAX_TARGETS];
} AStarBatchResult;

_Static_assert(sizeof(AStarBatchCandidate) == 260, "AStarBatchCandidate layout changed");
_Static_assert(sizeof(AStarBatchResult) == 4u + sizeof(AStarBatchCandidate) * MAX_TARGETS,
               "AStarBatchResult layout changed");

typedef void (*AStarPathEmitFn)(uint8_t slot, const Direction* path, uint16_t len, void* ctx);

// 辅助宏：2D 坐标与 1D 紧凑索引转换。
#define POS_TO_IDX(pos)     ((uint8_t)((((uint16_t)(pos).y) * MAP_COLS) + (uint16_t)(pos).x))
#define IDX_TO_POS(idx)     ((Position){(uint8_t)((idx) % MAP_COLS), (uint8_t)((idx) / MAP_COLS)})

extern const Direction DIRECTIONS[4];
extern uint16_t g_macro_dist_field[MAX_MACRO_DEPTH][MAP_ROWS][MAP_COLS];


/**
 * 清空 A* 状态表（内部使用）
 */
void hash_table_clear(void);

/**
 * 物理清空所有持久化 A* 表和桶队列状态。
 * 在独立地图批次之间释放求解器时调用。
 */
void astar_reset_tables(void);

/**
 * 最小堆结构（内部使用）
 */
typedef struct {
    AStarNode* heap;      // 堆数组
    uint16_t heap_size;   // 当前大小
    uint16_t max_size;    // 最大容量
} MinHeap;

/**
 * 初始化最小堆（内部使用）
 * 参数 h：堆指针
 * 参数 buffer：堆缓冲区
 * 参数 max_size：最大容量
 */
void heap_init(MinHeap* h, AStarNode* buffer, uint16_t max_size);

/**
 * 向堆中添加节点（内部使用）
 * 参数 h：堆指针
 * 参数 node：节点指针
 */
void heap_push(MinHeap* h, const AStarNode* node);

/**
 * 从堆中弹出最小节点（内部使用）
 * 参数 h：堆指针
 * 参数 out：输出节点
 * 返回：true表示成功，false表示堆为空
 */
bool heap_pop(MinHeap* h, AStarNode* out);

struct SokobanSolver;  // 前置声明，避免循环依赖

/**
 * 通用A*算法（内部核心实现 - 位掩码版本）
 * 支持三种模式：单箱子单目标、单箱子多目标、纯导航
 * 参数 heap_buffer：堆缓冲区（需要最大堆容量）
 * 参数 map：位掩码地图
 * 参数 start_player：玩家起始位置
 * 参数 start_box：箱子起始位置（导航模式可忽略）
 * 参数 target_positions：目标位置数组
 * 参数 num_targets：目标数量
 * 参数 reached_target_idx：输出参数，到达的目标索引（多目标时有效）
 * 参数 collision_mask：碰撞掩码（例如墙和炸弹）
 * 参数 out_path：输出路径数组
 * 参数 out_len：输出路径长度
 * 参数 macro_depth：当前炸墙宏观深度；无宏观深度标记表示禁用距离场
 * 参数 route_type：路由上下文，决定启发式和死锁拦截策略
 * 返回：true表示找到路径，false表示无路径
 */
bool astar_solve_with_mask(
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
);

AStarBatchStatus astar_solve_single_box_targets_mask(
    const BitboardMap* bmap,
    Position start_player,
    Position start_box,
    const Position* target_positions,
    const uint8_t* target_slots,
    uint8_t num_targets,
    uint8_t collision_mask,
    uint32_t capacity_limit,
    AStarBatchResult* out
);

int astar_bomb_reach_all(
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
);

int astar_bomb_reach_all_emit(
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
);

/**
 * 单箱子单目标A*求解（位掩码版本）
 */
static inline bool astar_solve_single_box_mask(
    AStarNode* heap_buffer,
    ClosedNode* closed_buffer,
    const BitboardMap* bmap,
    Position start_player,
    Position start_box,
    Position target_pos,
    uint8_t collision_mask,
    Direction* out_path,
    uint16_t* out_len,
    int macro_depth,
    AStarRouteType route_type
) {
    int dummy_idx;
    return astar_solve_with_mask(heap_buffer, closed_buffer, bmap, start_player, start_box, &target_pos, 1, 
                                &dummy_idx, collision_mask, out_path, out_len, macro_depth, route_type);
}

/**
 * 单箱子多目标A*求解（位掩码版本）
 */
static inline bool astar_solve_multi_target_mask(
    AStarNode* heap_buffer,
    ClosedNode* closed_buffer,
    const BitboardMap* bmap,
    Position start_player,
    Position start_box,
    Position* available_targets,
    int num_available_targets,
    int* reached_target_idx,
    uint8_t collision_mask,
    Direction* out_path,
    uint16_t* out_len,
    int macro_depth
) {
    return astar_solve_with_mask(heap_buffer, closed_buffer, bmap, start_player, start_box, available_targets, 
                                num_available_targets, reached_target_idx, collision_mask, 
                                out_path, out_len, macro_depth, ROUTE_BOX_NORMAL);
}

/**
 * 纯导航A*求解（位掩码版本）
 */
bool astar_navigate_mask(
    AStarNode* heap_buffer,
    ClosedNode* closed_buffer,
    const BitboardMap* bmap,
    Position start,
    Position target,
    uint8_t collision_mask,
    Direction* out_path,
    uint16_t* out_len
);
#endif
