#ifndef SOKOBAN_SOLVER_H
#define SOKOBAN_SOLVER_H

#include "astar.h"
#include "sokoban_types.h"
/* 移植时可按本地目录调整头文件路径。
#include "zf_common_headfile.h"
*/

// 

#define MAX_BOMB_DELAY_EVENTS MAX_BOMBS
/* The beam exposes at most two verified tails per bomb layer. */
#define SOKOBAN_SCAN_BOMB_MAIN_ROUTE_CANDIDATE_LIMIT (MAX_BOMBS * 2u)


typedef struct {
    uint16_t path_index;
    Position player_pos;
    Position blast_center;
    Position next_pos;
} BombDelayEvent;

typedef struct SokobanSolver {
    // 位棋盘地图
    BitboardMap bmap;

    // 被毁墙壁的极速掩码
    uint32_t destroyed_walls_mask;
    
    // 统一的实体数组
    Entity boxes[MAX_BOXES];
    Entity targets[MAX_TARGETS];
    Entity bombs[MAX_BOMBS];
    
    Position start_player;
    /* 完整地图加载时的原始车位；扫描和求解推进 start_player 后仍保留。 */
    Position map_start_player;
    bool map_start_player_valid;
    uint8_t num_boxes;
    uint8_t num_targets;
    uint8_t num_bombs;
    
    // 扫描模式相关状态
    bool is_scanning;
    int scan_waypoint_count;
    int scan_current_index;

    Position scan_player_pause_positions[SOKOBAN_SCAN_MAX_WAYPOINTS];
    Entity scan_waypoints[SOKOBAN_SCAN_MAX_WAYPOINTS];

    uint8_t bomb_delay_count;
    BombDelayEvent bomb_delay_events[MAX_BOMB_DELAY_EVENTS];

    // 只保留轻量级的状态标志位
    bool backup_needed_box;
    bool backup_needed_target;
    bool backup_activated;
    
    // 外部内存挂载
    AStarNode* heap;
    ClosedNode* closed_list;  // 闭表挂载点
    
    // 已找到的路径
    Direction* best_path;
    uint16_t best_path_len;
    uint16_t best_steps;
    
    // 严格目标模式
    bool strict_target_mode;

    // 当前正式求解是否来自扫描识别；识别求解的正式路径不得写入 Flash。
    bool identified_solve_mode;
    
} SokobanSolver;

/**
 * 检查指定位置是否为墙壁
 * 参数 solver：求解器实例
 * 参数 x：X坐标
 * 参数 y：Y坐标
 * 返回：true表示是墙壁或越界，false表示可通行
 */
static inline bool is_wall(const SokobanSolver* solver, int x, int y) {
    if (x < 0 || x >= MAP_COLS || y < 0 || y >= MAP_ROWS) return true;
    return (solver->bmap.walls[y] & (1 << x)) != 0;
}

/**
 * 判断被跟踪箱子是否可以被目标格吸收。
 */
static inline bool solver_should_absorb_box(const SokobanSolver* solver, int b_idx, Position target_pos) {
    /* Recognition maps obey ID absorption for the entire scan session, even
       while scan routing temporarily uses the non-strict solve mode. */
    if (!solver->strict_target_mode && !solver->is_scanning) return true;
    if (b_idx < 0 || b_idx >= solver->num_boxes) return true;

    for (int t = 0; t < solver->num_targets; t++) {
        if (pos_equal(solver->targets[t].pos, target_pos)) {
            int b_id = solver->boxes[b_idx].id;
            int t_id = solver->targets[t].id;
            return b_id == -1 || t_id == -1 || b_id == t_id;
        }
    }
    return false;
}

/**
 * 在模拟地图上推动被跟踪箱子，并应用求解器的目标吸收规则。
 */
static inline void solver_apply_tracked_box_push(
    const SokobanSolver* solver,
    BitboardMap* bmap,
    Position* box_positions,
    Position from,
    Position to
) {
    int b_idx = tracked_position_index(box_positions, solver->num_boxes, from);
    bool absorb = solver_should_absorb_box(solver, b_idx, to);
    apply_tracked_box_push_bits(bmap, box_positions, b_idx, from, to, absorb);
}

/**
 * 创建求解器实例
 * 返回：求解器指针，失败返回空指针
 */
SokobanSolver* solver_create(void);
void solver_warmup(void);

/**
 * 销毁求解器实例并释放内存
 * 参数 solver：求解器指针
 */
void solver_destroy(SokobanSolver* solver);
/**
 * Flash scan cache switch.
 * 1 = solver_generate_scan_path() may read/store flash cache through flash_* functions.
 * 0 = force pure solving path; no flash read/store/clear is performed.
 * Default is 0.
 */
extern uint8_t g_sokoban_flash_cache_enabled;

bool solver_clear_scan_cache_flash(void);

/**
 * 从字符串加载地图数据到求解器
 * 参数 solver：求解器指针
 * 参数 map_string：地图字符串，格式：row1|row2|...|row16
 *                   支持的字符：
 *                   '#' = 墙壁
 *                   '@' = 玩家初始位置
 *                   '0'-'9' = 带ID的箱子
 *                   '$' = 无ID箱子（自动分配）
 *                   'a'-'j' = 带ID的目标点（ID 0-9，最多加载 MAX_TARGETS 个）
 *                   '.' = 无ID目标点（自动分配）
 *                   'B' = 炸弹
 * 返回：true表示加载成功，false表示失败
 */
bool solver_load_map_from_string(SokobanSolver* solver, const char* map_string);

/* Load a complete residual observation with strict syntax but relaxed counts. */
bool solver_load_residual_map_from_string(SokobanSolver* solver, const char* map_string);

/**
 * 求解推箱子问题
 * 使用深度优先搜索、A*算法、死锁检测、炸弹策略
 * 参数 solver：求解器指针
 * 返回：true表示找到解决方案，false表示无解
 */
bool solver_solve(SokobanSolver* solver);

/**
 * 带事后验证和黑名单剪枝的鲁棒求解入口
 * 参数 solver：求解器指针
 * 返回：true表示找到通过验证的解决方案，false表示无解
 */
bool solver_solve_robust(SokobanSolver* solver);
uint8_t solver_get_scan_bomb_main_route_candidate_count(void);
bool solver_copy_scan_bomb_main_route_candidate(
    uint8_t index, Direction* out_path, uint16_t* out_len
);

/**
 * 获取求解路径（Direction数组）
 * 参数 solver：求解器指针
 * 参数 length：输出参数，返回路径长度
 * 返回：方向数组指针，包含每一步的移动方向
 */
Direction* solver_get_solution(SokobanSolver* solver, uint16_t* length);

/**
 * 设置严格目标模式
 * 参数 solver：求解器指针
 * 参数 strict_mode：true=严格模式（箱子必须推到指定ID的目标点）
 *                    false=普通模式（箱子可推到任意目标点）
 */
void solver_set_strict_target_mode(SokobanSolver* solver, bool strict_mode);
void solver_set_identified_solve_mode(SokobanSolver* solver, bool identified);

/**
 * 生成扫描路径（接口封装）
 * 例如：地图有2个箱子和2个目标点，生成4个停靠点，依次扫描
 * 参数 solver：求解器指针
 * 返回：true表示生成成功，false表示失败
 */
bool solver_generate_scan_path(SokobanSolver* solver);
bool solver_build_scan_cache_key(const SokobanSolver* solver, SokobanScanCacheKey* out_key);
bool solver_try_apply_scan_cache(SokobanSolver* solver, const SokobanScanCachePayload* payload);
bool solver_export_scan_cache(const SokobanSolver* solver, SokobanScanCachePayload* out_payload);

/**
 * 扫描流程接口：将指定ID写入当前扫描点
 * 参数 solver：求解器指针
 * 参数 id：大于等于0表示分配ID，-1表示跳过分配ID(无ID模式)
 * 返回：0=无ID模式完成，1=继续扫描，2=扫描全部完成，-1=错误
 */
int solver_assign_next_scan_id(SokobanSolver* solver, int id);

uint8_t solver_get_bomb_delay_count(const SokobanSolver* solver);
bool solver_get_bomb_delay_event(const SokobanSolver* solver, int index, BombDelayEvent* out_event);
bool solver_get_bomb_delay_at_path_index(const SokobanSolver* solver, uint16_t path_index, BombDelayEvent* out_event);


/**
 * Driver 内部辅助函数，由求解模块和扫描模块共享。
 */
void solver_optimize_post_path(SokobanSolver* solver);
void solver_optimize_bomb_mixed_path(
    SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
);
void solver_sim_clear_explosion_walls(BitboardMap* bmap, Position center);
void solver_build_bomb_delay_events_from_state(
    SokobanSolver* solver,
    const BitboardMap* initial_map,
    Position initial_player,
    const Entity* initial_boxes,
    int initial_num_boxes,
    const Entity* initial_bombs,
    int initial_num_bombs
);

/**
 * 外部修改地图后刷新静态死锁层和通道层。
 */
void solver_refresh_deadlocks(SokobanSolver* solver);


#endif
