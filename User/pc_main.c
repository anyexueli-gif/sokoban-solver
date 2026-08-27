#include "../Driver/sokoban_solver.h"
#include "../Driver/sokoban_scan.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

typedef enum {
    RESP_OK = 0,
    RESP_ERROR = 1,
    RESP_SCAN_STEP = 2,
    RESP_SCAN_COMPLETE = 4,
    RESP_SOLUTION = 5,
    RESP_NO_SOLUTION = 6
} ResponseType;

typedef enum {
    PC_PROTOCOL_EMPTY = 0,
    PC_PROTOCOL_MAP_READY,
    PC_PROTOCOL_SCANNING,
    PC_PROTOCOL_SCAN_DIRECT_READY,
    PC_PROTOCOL_SCAN_IDS_PENDING,
    PC_PROTOCOL_IDS_READY,
    PC_PROTOCOL_SOLVE_DONE
} PcProtocolPhase;

static SokobanSolver* g_solver = NULL;
static PcProtocolPhase g_protocol_phase = PC_PROTOCOL_EMPTY;
static char g_scan_path[2048];
static Position g_scan_positions[MAX_BOXES + MAX_TARGETS];
static int8_t g_scan_tags[MAX_BOXES + MAX_TARGETS];
static int g_scan_position_count = 0;
static int g_last_waypoint_count = 0;
static uint16_t g_last_scan_path_len = 0;

static char direction_to_char(Direction dir) {
    if (dir.dx == -1 && dir.dy == 0) return 'L';
    if (dir.dx == 1 && dir.dy == 0) return 'R';
    if (dir.dx == 0 && dir.dy == -1) return 'U';
    if (dir.dx == 0 && dir.dy == 1) return 'D';
    if (dir.dx == 0 && dir.dy == 0) return '?';
    return '?';
}

static bool generate_scan_with_cache(SokobanSolver* solver) {
    return solver_generate_scan_path(solver);
}

static void send_response(ResponseType type, const char* data) {
    printf("RESP:%d:%s\n", type, data ? data : "");
    fflush(stdout);
}

#define PC_PROTOCOL_PHASE_MASK(phase) (1u << (unsigned)(phase))

static const char* protocol_phase_name(PcProtocolPhase phase) {
    switch (phase) {
        case PC_PROTOCOL_EMPTY: return "empty";
        case PC_PROTOCOL_MAP_READY: return "map_ready";
        case PC_PROTOCOL_SCANNING: return "scanning";
        case PC_PROTOCOL_SCAN_DIRECT_READY: return "scan_direct_ready";
        case PC_PROTOCOL_SCAN_IDS_PENDING: return "scan_ids_pending";
        case PC_PROTOCOL_IDS_READY: return "ids_ready";
        case PC_PROTOCOL_SOLVE_DONE: return "solve_done";
        default: return "invalid";
    }
}

static bool protocol_phase_require(const char* command, uint32_t allowed_phases) {
    uint32_t current = PC_PROTOCOL_PHASE_MASK(g_protocol_phase);
    if ((allowed_phases & current) != 0u) return true;

    char message[160];
    snprintf(message, sizeof(message),
             "Protocol phase violation: %s current=%s",
             command ? command : "unknown",
             protocol_phase_name(g_protocol_phase));
    send_response(RESP_ERROR, message);
    return false;
}

static void append_scan_waypoint(char* response, size_t response_size, const Entity* waypoint) {
    if (!response || !waypoint || response_size == 0) return;

    char coord[40];
    snprintf(coord, sizeof(coord), "|%d,%d,%d",
             waypoint->pos.x, waypoint->pos.y, waypoint->id);
    size_t used = strlen(response);
    if (used + 1 >= response_size) return;
    strncat(response, coord, response_size - used - 1);
}

static char* receive_command(void) {
    static char buffer[1024];
    if (fgets(buffer, sizeof(buffer), stdin)) {
        size_t len = strlen(buffer);
        if (len > 0 && buffer[len - 1] == '\n') buffer[len - 1] = '\0';
        return buffer;
    }
    return NULL;
}

static void handle_load_map(const char* data) {
    if (!data) {
        send_response(RESP_ERROR, "No map data");
        return;
    }
    if (!g_solver) {
        g_solver = solver_create();
        if (!g_solver) {
            send_response(RESP_ERROR, "Failed to create solver");
            return;
        }
    }
    if (!solver_load_map_from_string(g_solver, data)) {
        g_protocol_phase = PC_PROTOCOL_EMPTY;
        send_response(RESP_ERROR, "Failed to load map");
        return;
    }
    g_protocol_phase = PC_PROTOCOL_MAP_READY;
    send_response(RESP_OK, "Map loaded");
}

static bool handle_start_scan(void) {
    if (!g_solver) return false;
    if (!generate_scan_with_cache(g_solver)) return false;

    g_last_waypoint_count = g_solver->scan_waypoint_count;
    g_scan_position_count = g_solver->scan_waypoint_count;
    for (int i = 0; i < g_solver->scan_waypoint_count; i++) {
        g_scan_positions[i] = g_solver->scan_waypoints[i].pos;
        g_scan_tags[i] = g_solver->scan_waypoints[i].id;
    }

    uint16_t path_len;
    Direction* path = solver_get_solution(g_solver, &path_len);
    g_last_scan_path_len = path_len;
    for (int i = 0; i < path_len && i < (int)sizeof(g_scan_path) - 1; i++) {
        g_scan_path[i] = direction_to_char(path[i]);
    }
    g_scan_path[path_len] = '\0';
    return true;
}

static int handle_scan_id_input(const char* data) {
    if (!g_solver || !g_solver->is_scanning || !data) return -1;

    int parsed_id = -3;
    if (strcmp(data, "no") == 0 || strncmp(data, "no:", 3) == 0) parsed_id = -1;
    else if (data[0] == '?') parsed_id = -2;
    else if (data[0] >= '0' && data[0] <= '9') parsed_id = data[0] - '0';

    if (parsed_id == -3) return -1;
    return solver_assign_next_scan_id(g_solver, parsed_id);
}

static bool handle_set_id_at(const char* data) {
    if (!g_solver || !data) return false;

    int x = -1, y = -1, id = -1;
    if (sscanf(data, "%d,%d,%d", &x, &y, &id) != 3) return false;
    if (x < 0 || x >= MAP_COLS || y < 0 || y >= MAP_ROWS || id < 0 || id > 9) return false;

    Position pos = {(uint8_t)x, (uint8_t)y};
    for (int i = 0; i < g_solver->num_boxes; i++) {
        if (pos_equal(g_solver->boxes[i].pos, pos)) {
            g_solver->boxes[i].id = id;
            g_solver->strict_target_mode = true;
            return true;
        }
    }
    for (int i = 0; i < g_solver->num_targets; i++) {
        if (pos_equal(g_solver->targets[i].pos, pos)) {
            g_solver->targets[i].id = id;
            g_solver->strict_target_mode = true;
            return true;
        }
    }
    return false;
}

static bool handle_start_solve(void) {
    if (!g_solver) return false;
    solver_refresh_deadlocks(g_solver);
    return solver_solve_robust(g_solver);
}

static bool find_scan_continuation_start(const Direction* path,
                                         uint16_t path_len,
                                         int completed_pause_count,
                                         uint16_t* out_start_idx) {
    if (!path || !out_start_idx || completed_pause_count <= 0) return false;

    int pause_count = 0;
    for (uint16_t i = 0; i < path_len; i++) {
        if (path[i].dx != 0 || path[i].dy != 0) continue;
        pause_count++;
        if (pause_count == completed_pause_count) {
            *out_start_idx = (uint16_t)(i + 1u);
            return true;
        }
    }
    return false;
}

static void handle_reset(void) {
    if (g_solver) {
        solver_destroy(g_solver);
        g_solver = NULL;
    }
    g_protocol_phase = PC_PROTOCOL_EMPTY;
    send_response(RESP_OK, "Reset complete");
}

int main(void) {
    while (1) {
        char* cmd = receive_command();
        if (!cmd) continue;
        char* colon = strchr(cmd, ':');
        char* data = NULL;
        if (colon) {
            *colon = '\0';
            data = colon + 1;
        }

        if (strcmp(cmd, "WARMUP") == 0) {
            solver_warmup();
            send_response(RESP_OK, "Warmup complete");
        } else if (strcmp(cmd, "FLASH_CLEAR") == 0) {
            send_response(solver_clear_scan_cache_flash() ? RESP_OK : RESP_ERROR, "Flash cache cleared");
        } else if (strcmp(cmd, "LOAD_MAP") == 0) {
            handle_load_map(data);
        } else if (strcmp(cmd, "START_SCAN") == 0) {
            if (!protocol_phase_require(
                    cmd,
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_MAP_READY) |
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_SOLVE_DONE))) {
                continue;
            }
            if (handle_start_scan()) {
                char response[4096];
                strcpy(response, g_scan_path);
                for (int i = 0; i < g_scan_position_count; i++) {
                    Entity waypoint = {
                        g_scan_positions[i],
                        g_scan_tags[i],
                        true
                    };
                    append_scan_waypoint(response, sizeof(response), &waypoint);
                }
                g_protocol_phase = g_scan_position_count > 0
                    ? PC_PROTOCOL_SCANNING
                    : PC_PROTOCOL_SCAN_IDS_PENDING;
                send_response(RESP_SCAN_STEP, response);
            } else {
                send_response(RESP_ERROR, "Failed to generate scan path");
            }
        } else if (strcmp(cmd, "SCAN_ID") == 0) {
            if (!protocol_phase_require(
                    cmd, PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_SCANNING))) {
                continue;
            }
            int result = handle_scan_id_input(data);
            if (result == 1) {
                uint16_t path_len;
                Direction* path = solver_get_solution(g_solver, &path_len);
                if (path_len > g_last_scan_path_len || g_solver->scan_waypoint_count > g_last_waypoint_count) {
                    char rem_path[2048] = {0};
                    int p_idx = 0;
                    uint16_t start_idx = 0;
                    if (!find_scan_continuation_start(path,
                                                      path_len,
                                                      g_solver->scan_current_index,
                                                      &start_idx)) {
                        send_response(RESP_ERROR, "Failed to locate scan continuation");
                        continue;
                    }
                    for (uint16_t i = start_idx; i < path_len && p_idx < 2047; i++) {
                        rem_path[p_idx++] = direction_to_char(path[i]);
                    }
                    rem_path[p_idx] = '\0';

                    char response[4096];
                    strcpy(response, rem_path);
                    for (int i = g_solver->scan_current_index; i < g_solver->scan_waypoint_count; i++) {
                        append_scan_waypoint(response, sizeof(response), &g_solver->scan_waypoints[i]);
                    }
                    g_last_scan_path_len = path_len;
                    g_last_waypoint_count = g_solver->scan_waypoint_count;
                    send_response(RESP_SCAN_STEP, response);
                } else {
                    send_response(RESP_OK, "continue");
                }
            } else if (result == 0) {
                g_protocol_phase = PC_PROTOCOL_SCAN_DIRECT_READY;
                send_response(RESP_SCAN_COMPLETE, "no_id");
            } else if (result == 2) {
                g_protocol_phase = PC_PROTOCOL_SCAN_IDS_PENDING;
                send_response(RESP_SCAN_COMPLETE, "with_id");
            } else {
                send_response(RESP_ERROR, "Failed to record ID");
            }
        } else if (strcmp(cmd, "SET_ID_AT") == 0) {
            if (!protocol_phase_require(
                    cmd,
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_MAP_READY) |
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_IDS_READY) |
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_SOLVE_DONE))) {
                continue;
            }
            if (handle_set_id_at(data)) {
                if (g_protocol_phase == PC_PROTOCOL_SOLVE_DONE) {
                    g_protocol_phase = PC_PROTOCOL_MAP_READY;
                }
                send_response(RESP_OK, "ID set");
            } else {
                send_response(RESP_ERROR, "Failed to set ID");
            }
        } else if (strcmp(cmd, "FINALIZE_SCAN_IDS") == 0) {
            if (!protocol_phase_require(
                    cmd, PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_SCAN_IDS_PENDING))) {
                continue;
            }
            if (!sokoban_auto_assign_remaining_ids(g_solver)) {
                send_response(RESP_ERROR, "Invalid scan IDs");
            } else {
                g_solver->strict_target_mode = true;
                solver_set_identified_solve_mode(g_solver, true);
                g_protocol_phase = PC_PROTOCOL_IDS_READY;
                send_response(RESP_OK, "IDs finalized");
            }
        } else if (strcmp(cmd, "START_SOLVE") == 0) {
            if (!protocol_phase_require(
                    cmd,
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_MAP_READY) |
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_SCAN_DIRECT_READY) |
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_IDS_READY) |
                    PC_PROTOCOL_PHASE_MASK(PC_PROTOCOL_SOLVE_DONE))) {
                continue;
            }
            if (handle_start_solve()) {
                uint16_t path_len;
                Direction* path = solver_get_solution(g_solver, &path_len);
                char solution[MAX_PATH_LENGTH + 1];
                for (int i = 0; i < path_len && i < MAX_PATH_LENGTH; i++) {
                    solution[i] = direction_to_char(path[i]);
                }
                solution[path_len] = '\0';
                g_protocol_phase = PC_PROTOCOL_SOLVE_DONE;
                send_response(RESP_SOLUTION, solution);
            } else {
                send_response(RESP_NO_SOLUTION, "No solution found");
            }
        } else if (strcmp(cmd, "RESET") == 0) {
            handle_reset();
        } else if (strcmp(cmd, "EXIT") == 0) {
            break;
        } else {
            send_response(RESP_ERROR, "Unknown command");
        }
    }
    if (g_solver) solver_destroy(g_solver);
    return 0;
}
