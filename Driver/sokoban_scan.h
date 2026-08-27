#ifndef SOKOBAN_SCAN_H
#define SOKOBAN_SCAN_H

#include <stdbool.h>
#include "sokoban_types.h"

#define SCAN_WAYPOINT_BOX_TAG_BASE (-16)
#define SCAN_WAYPOINT_TARGET_TAG_BASE (-32)


// 前向声明。
struct SokobanSolver;

/**
 * 生成初始扫描路径
 * 扫描完成后停在最后一个扫描点，坐标保存到 solver->scan_waypoints
 */
bool sokoban_generate_scan_path(struct SokobanSolver* solver);
bool sokoban_plan_rescan_route(
    const struct SokobanSolver* solver,
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
);

/**
 * 按需扩展扫描路径
 * 需要补扫时在数组尾部追加路径
 */
bool sokoban_extend_scan_path(struct SokobanSolver* solver, bool need_box, bool need_target, int current_idx);

/**
 * 扫描录入后自动补全或协调剩余 ID；发现 ID 冲突时返回 false。
 */
bool sokoban_auto_assign_remaining_ids(struct SokobanSolver* solver);

/**
 * 清空扫描阶段保留的扫描模块快照。
 */
void sokoban_scan_reset_state(void);


#endif
