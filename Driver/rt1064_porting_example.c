#include "sokoban_solver.h"
#include "sokoban_scan.h"
#include "sokoban_recovery.h"
#include <stdio.h>
#include <stdlib.h>

/*
 * map1：初始完整地图。
 * map2：当前完整残局地图；每次请求观测时都使用最新内容。
 */
extern char map1[];
extern char map2[];

int main(void) {


    // 此例显式开启缓存；全局默认值为 0，需由上层按需启用。
    // 若不需要使用缓存，可将上面的开关设为 0。
    g_sokoban_flash_cache_enabled = 1u;
    // g_sokoban_flash_cache_enabled = 0u;

    // 清除 flash cache 时直接调用：
    // solver_clear_scan_cache_flash();

    SokobanSolver* solver = solver_create();
    SokobanRecovery* recovery = sokoban_recovery_create();
    if (!solver || !recovery) {
        if (solver) solver_destroy(solver);
        return -1;
    }

    if (!solver_load_map_from_string(solver, map1)) {
        sokoban_recovery_reset(recovery);
        solver_destroy(solver);
        return -1;
    }

    // 生成扫描路径。flash cache 读写已在 solver_generate_scan_path() 内部自动处理。
    if (!solver_generate_scan_path(solver)) {
        sokoban_recovery_reset(recovery);
        solver_destroy(solver);
        return -1;
    }

    // 恢复会话必须在扫描和正式求解路径执行前创建。
    // 本示例后续扫描流程按 ID 求解，因此使用 IDENTIFIED；若改为无 ID
    // 直接求解，应同时改用 SOKOBAN_RECOVERY_DIRECT，并让 detected_id 使用 -1。
    const SokobanRecoveryMode recovery_mode = SOKOBAN_RECOVERY_IDENTIFIED;
    if (!sokoban_recovery_begin(recovery, solver, recovery_mode)) {
        sokoban_recovery_reset(recovery);
        solver_destroy(solver);
        return -1;
    }

    // 恢复过程只通过统一 step 接口推进。
    SokobanRecoveryResult recovery_result;

    uint16_t scan_len;
    Direction* scan_path = solver_get_solution(solver, &scan_len);

    // 执行扫描。
    int i = 0;
    while (i < scan_len) {
        Direction d = scan_path[i];
        
        // 遇到 {0,0} 代表到达预定的扫码暂停点
        if (d.dx == 0 && d.dy == 0) {
            
            // 控制云台转向目标。
            int current_idx = solver->scan_current_index;
            Position target_pos = solver->scan_waypoints[current_idx].pos;        // 目标实体的绝对坐标
            Position player_pos = solver->scan_player_pause_positions[current_idx]; // 小车当前的绝对坐标
            int waypoint_tag = (int)solver->scan_waypoints[current_idx].id;
            bool is_box = false;
            bool is_target = false;

            if (waypoint_tag <= SCAN_WAYPOINT_BOX_TAG_BASE && waypoint_tag > SCAN_WAYPOINT_BOX_TAG_BASE - MAX_BOXES) {
                int box_idx = SCAN_WAYPOINT_BOX_TAG_BASE - waypoint_tag;
                is_box = (box_idx >= 0 && box_idx < solver->num_boxes);
            } else if (waypoint_tag <= SCAN_WAYPOINT_TARGET_TAG_BASE && waypoint_tag > SCAN_WAYPOINT_TARGET_TAG_BASE - MAX_TARGETS) {
                int target_idx = SCAN_WAYPOINT_TARGET_TAG_BASE - waypoint_tag;
                is_target = (target_idx >= 0 && target_idx < solver->num_targets);
            }

            if (!is_box && !is_target) {
                is_box = (solver->bmap.boxes[target_pos.y] & (1 << target_pos.x)) != 0;
                is_target = (solver->bmap.targets[target_pos.y] & (1 << target_pos.x)) != 0;
            }
            
            int look_dx = target_pos.x - player_pos.x;
            int look_dy = target_pos.y - player_pos.y;
            
            if (look_dx == 1) {
                // 待接入：摄像头向右转 motor_camera_right();
            } else if (look_dx == -1) {
                // 待接入：摄像头向左转 motor_camera_left();
            } else if (look_dy == 1) {
                // 待接入：摄像头向下转 motor_camera_down();
            } else if (look_dy == -1) {
                // 待接入：摄像头向上转 motor_camera_up();
            }

            // 视觉识别。
            // 待接入：调用图像识别算法，获取 ID（0~9）。
            // 未识别时返回 -2，用于触发动态补扫。
            int detected_id = -2;
            if (is_box) {
                // detected_id = recognize_box_id();
            } else if (is_target) {
                // detected_id = recognize_target_id();
            }
            
            // 记录分配前的停靠点总数
            int old_waypoint_count = solver->scan_waypoint_count;
            
            // 更新扫描状态。
            int status = solver_assign_next_scan_id(solver, detected_id);
            
            // 返回值说明：0=无ID/中止, 1=继续扫描, 2=全部扫描完成
            if (status < 0) {
                sokoban_recovery_reset(recovery);
                solver_destroy(solver);
                return -1;
            }
            if (status == 0) {
                break;
            }

            // 停靠点增加时，重新获取扩展后的路径。
            if (solver->scan_waypoint_count > old_waypoint_count) {
                // 必须重新获取路径指针和总长度，否则小车走不到新加的目标点
                scan_path = solver_get_solution(solver, &scan_len);
            }
        } 
        else {
            // 常规移动指令。
            // 根据 dx 和 dy 判断车盘如何移动
            if (d.dx == -1) {
                // 待接入：motor_move_left();
            } else if (d.dx == 1) {
                // 待接入：motor_move_right();
            } else if (d.dy == -1) {
                // 待接入：motor_move_up();
            } else if (d.dy == 1) {
                // 待接入：motor_move_down();
            }
        }
        
        i++; // 手动推进执行索引
    }

    // 最终求解和执行。
    
    // 刷新死锁图层。
    // 扫描过程中箱子可能移动，需要刷新静态死锁。
    solver_refresh_deadlocks(solver);

    if (solver_solve_robust(solver)) {
        uint16_t solve_len = 0;
        Direction* final_path = solver_get_solution(solver, &solve_len);
        
        // 遍历并执行最终路径
        for (uint16_t j = 0; j < solve_len; j++) {
            Direction move = final_path[j];
            
            // 物理执行逻辑映射
            if (move.dx == 1) {
                // 待接入：motor_move_right();
            } else if (move.dx == -1) {
                // 待接入：motor_move_left();
            } else if (move.dy == 1) {
                // 待接入：motor_move_down();
            } else if (move.dy == -1) {
                // 待接入：motor_move_up();
            }
        }

    } else {
       // 无解状态处理
       // 待接入：触发报警 buzzer_alarm();
    }

    /*
     * 原求解路径执行完后进入逐轮恢复循环。控制层只做三类外部动作：执行一
     * 个完整路径段、提供最新 map2、识别一个 ID；当前该交给哪类输入由 Driver
     * 内部阶段决定。不设两张地图上限，也不需要根据 next_status 选择
     * submit_observation() 或 submit_id()。
     */
    SokobanRecoveryInput recovery_input = {0};
    bool recovery_finished = false;
    bool recovery_failed = false;
    while (!recovery_finished) {
        recovery_result = sokoban_recovery_step(recovery, solver, &recovery_input);
        recovery_input = (SokobanRecoveryInput){0};

        switch (recovery_result.status) {
            case SOKOBAN_RECOVERY_PATH_READY: {
                bool path_executed = false;

                /* 底层完整执行该路径；只有成功返回后，下一轮空输入才确认完成。 */
                // path_executed = motor_execute_path(
                //     recovery_result.path, recovery_result.path_len);
                if (!path_executed) {
                    // 待接入：停止小车并报警，由外层按真实现场重建恢复会话。
                    recovery_failed = true;
                    recovery_finished = true;
                }
                break;
            }

            case SOKOBAN_RECOVERY_NEED_OBSERVATION: {
                bool fresh_map_ready = false;

                /*
                 * 底层重新拍摄完整残局并刷新 map2；控制层提交原始地图，不自行
                 * 判断箱子/目的地数量，也不改写地图内容。
                 */
                // fresh_map_ready = camera_wait_and_update_map2();
                if (!fresh_map_ready) {
                    recovery_failed = true;
                    recovery_finished = true;
                } else {
                    recovery_input.map_string = map2;
                    recovery_input.has_observation = true;
                }
                break;
            }

            case SOKOBAN_RECOVERY_RETRY_OBSERVATION: {
                bool fresh_map_ready = false;

                /*
                 * 上一次观测未通过校验。底层必须等待另一张新图，不能立即重复
                 * 提交同一份 map2；Driver 会继续保留当前恢复会话。
                 */
                // fresh_map_ready = camera_wait_and_update_map2();
                if (!fresh_map_ready) {
                    recovery_failed = true;
                    recovery_finished = true;
                } else {
                    recovery_input.map_string = map2;
                    recovery_input.has_observation = true;
                }
                break;
            }

            case SOKOBAN_RECOVERY_NEED_ID: {
                bool recognition_finished = false;
                int recovery_id = -2;

                /*
                 * 底层完成一次真实识别后再提交：0 到 9 是可信 ID，识别失败为
                 * -2，Driver 会自行规划其它观察方向。
                 */
                /*
                 * Driver 已预先算好观察方向；控制层只分发云台命令，不再做坐标差计算。
                 */
                if (recovery_result.view_direction.dx == 1 &&
                    recovery_result.view_direction.dy == 0) {
                    // 待接入：摄像头向右转 motor_camera_right();
                } else if (recovery_result.view_direction.dx == -1 &&
                           recovery_result.view_direction.dy == 0) {
                    // 待接入：摄像头向左转 motor_camera_left();
                } else if (recovery_result.view_direction.dx == 0 &&
                           recovery_result.view_direction.dy == 1) {
                    // 待接入：摄像头向下转 motor_camera_down();
                } else if (recovery_result.view_direction.dx == 0 &&
                           recovery_result.view_direction.dy == -1) {
                    // 待接入：摄像头向上转 motor_camera_up();
                } else {
                    recovery_failed = true;
                    recovery_finished = true;
                    break;
                }

                if (recovery_result.observation_kind == SOKOBAN_RECOVERY_ENTITY_BOX) {
                    // recovery_id = recognize_box_id(); recognition_finished = true;
                } else if (recovery_result.observation_kind == SOKOBAN_RECOVERY_ENTITY_TARGET) {
                    // recovery_id = recognize_target_id(); recognition_finished = true;
                }
                if (!recognition_finished) {
                    recovery_failed = true;
                    recovery_finished = true;
                } else {
                    recovery_input.id = recovery_id;
                    recovery_input.has_id = true;
                }
                break;
            }

            case SOKOBAN_RECOVERY_COMPLETE:
                // 当前残局已全部清空，不需要恢复返航路径。
                recovery_finished = true;
                break;

            case SOKOBAN_RECOVERY_PARTIAL_RETURNED:
                // 最终返航段已执行完成；该状态不保证此前一定完成过安全投递。
                recovery_finished = true;
                break;

            case SOKOBAN_RECOVERY_ERROR:
            default:
                // 待接入：错误处理或报警。
                recovery_failed = true;
                recovery_finished = true;
                break;
        }
    }

    // 恢复模块没有单独的 destroy 接口，结束后 reset，再释放 solver。
    sokoban_recovery_reset(recovery);
    solver_destroy(solver);
    return recovery_failed ? -1 : 0;
}
