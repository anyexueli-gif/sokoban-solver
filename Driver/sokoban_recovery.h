#ifndef SOKOBAN_RECOVERY_H
#define SOKOBAN_RECOVERY_H

#include "sokoban_solver.h"

typedef enum {
    SOKOBAN_RECOVERY_DIRECT = 0,
    SOKOBAN_RECOVERY_IDENTIFIED = 1
} SokobanRecoveryMode;

typedef enum {
    SOKOBAN_RECOVERY_ERROR = 0,
    SOKOBAN_RECOVERY_PATH_READY,
    SOKOBAN_RECOVERY_NEED_OBSERVATION,
    SOKOBAN_RECOVERY_NEED_ID,
    SOKOBAN_RECOVERY_RETRY_OBSERVATION,
    SOKOBAN_RECOVERY_COMPLETE,
    SOKOBAN_RECOVERY_PARTIAL_RETURNED
} SokobanRecoveryStatus;

typedef enum {
    SOKOBAN_RECOVERY_ENTITY_NONE = 0,
    SOKOBAN_RECOVERY_ENTITY_TARGET = 1,
    SOKOBAN_RECOVERY_ENTITY_BOX = 2
} SokobanRecoveryEntityKind;

/*
 * 当前恢复会话的返航标志：0 表示残局无目的地或已经完全恢复，不计算返航路径；
 * 1 表示恢复不完全，Driver 已经计算了返航路径，其他控制模块可以直接读取。
 */
extern uint8_t g_sokoban_recovery_need_return_path;

typedef struct {
    SokobanRecoveryStatus status;
    SokobanRecoveryStatus next_status;
    const Direction* path;
    uint16_t path_len;
    Position observation_pos;
    uint8_t observation_kind;
    Position entity_pos;
    Direction view_direction;
} SokobanRecoveryResult;

/*
 * Unified external input. map_string is consumed only when has_observation
 * is true. has_id distinguishes an unavailable ID from a submitted failed
 * recognition: -2 is a valid submitted failure and asks Driver for another view.
 */
typedef struct {
    const char* map_string;
    bool has_observation;
    int id;
    bool has_id;
} SokobanRecoveryInput;

typedef struct SokobanRecovery SokobanRecovery;

SokobanRecovery* sokoban_recovery_create(void);
void sokoban_recovery_reset(SokobanRecovery* recovery);

/* begin() selects fixed return tile {5,1}, or {6,1} when {5,1} is a wall in
   the original map. Later scan-path changes to start_player do not affect it. */
bool sokoban_recovery_begin(
    SokobanRecovery* recovery,
    const SokobanSolver* solver,
    SokobanRecoveryMode mode
);

SokobanRecoveryResult sokoban_recovery_get_result(const SokobanRecovery* recovery);

SokobanRecoveryResult sokoban_recovery_submit_observation(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const char* map_string
);

/* Only 0..9 are trusted IDs. Other values request another reachable view;
   the Driver skips the entity only after all view directions are exhausted. */
SokobanRecoveryResult sokoban_recovery_submit_id(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    int id
);

/*
 * Advance the session through one unified entry point. Driver chooses whether
 * to consume map_string or id from its internal phase. Every PATH_READY
 * segment is executed as a whole, then the caller calls again; a residual
 * segment always returns to observation/ID input until a fresh empty map or a
 * final return segment is observed. A valid observation has exactly
 * MAP_ROWS x MAP_COLS cells, one '@', and only the residual-map characters
 * accepted by solver_load_residual_map_from_string. Null/empty, malformed,
 * over-capacity, duplicate/missing-player, and explicitly retryable count
 * contradictions return RETRY_OBSERVATION without consuming the consecutive
 * same-physical-layer streak. Player position is ignored by that comparison;
 * walls, boxes, targets, and bombs are not.
 */
SokobanRecoveryResult sokoban_recovery_step(
    SokobanRecovery* recovery,
    SokobanSolver* solver,
    const SokobanRecoveryInput* input
);

#endif
