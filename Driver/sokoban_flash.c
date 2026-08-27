#include "sokoban_flash.h"

#include <stdint.h>
#include <string.h>

#ifndef ALLOC_IN_SDRAM_CACHE
#define ALLOC_IN_SDRAM_CACHE SOKOBAN_BSS_SECTION("SDRAM_CACHE")
#endif
#ifndef ALLOC_IN_SDRAM
#define ALLOC_IN_SDRAM ALLOC_IN_SDRAM_CACHE
#endif

#define SCAN_CACHE_FLASH_MAGIC 0x31434253u
#define SCAN_CACHE_FLASH_VERSION 5u
#define SCAN_CACHE_FLASH_SECTION 127u
#define SCAN_CACHE_FLASH_PAGE_COUNT 8u
#define SCAN_CACHE_FLASH_SLOT_COUNT 64u
#define SCAN_CACHE_FLASH_SLOT_SIZE 512u
#define SCAN_CACHE_FLASH_FILE_SIZE (SCAN_CACHE_FLASH_SLOT_COUNT * SCAN_CACHE_FLASH_SLOT_SIZE)
#define SCAN_CACHE_FLASH_MAX_PACKED_PATH 128u
#define SCAN_CACHE_FLASH_PAGE_SIZE 0x00001000u
#define SCAN_CACHE_FLASH_PAGE_WORDS (SCAN_CACHE_FLASH_PAGE_SIZE / sizeof(uint32_t))
#define SCAN_CACHE_FLASH_SLOTS_PER_PAGE \
    (SCAN_CACHE_FLASH_PAGE_SIZE / SCAN_CACHE_FLASH_SLOT_SIZE)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    uint32_t write_counter;
    SokobanScanCacheKey key;
    uint16_t path_len;
    uint16_t packed_path_len;
    uint8_t waypoint_count;
    uint8_t reserved;
    Position end_player;
    uint16_t after_walls[MAP_ROWS];
    Entity waypoints[SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS];
    Position pause_positions[SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS];
    uint8_t packed_path[SCAN_CACHE_FLASH_MAX_PACKED_PATH];
    uint32_t data_crc;
} ScanCacheFlashRecord;

_Static_assert(sizeof(ScanCacheFlashRecord) <= SCAN_CACHE_FLASH_SLOT_SIZE,
               "scan cache flash record must fit one slot");
_Static_assert(SCAN_CACHE_FLASH_PAGE_SIZE % SCAN_CACHE_FLASH_SLOT_SIZE == 0u,
               "scan cache slots must not cross flash pages");
_Static_assert(SCAN_CACHE_FLASH_PAGE_COUNT * SCAN_CACHE_FLASH_SLOTS_PER_PAGE ==
                   SCAN_CACHE_FLASH_SLOT_COUNT,
               "scan cache page and slot counts must cover the same image");

static uint32_t g_scan_cache_flash_image[SCAN_CACHE_FLASH_FILE_SIZE / sizeof(uint32_t)] ALLOC_IN_SDRAM;
static uint8_t g_scan_cache_flash_slot[SCAN_CACHE_FLASH_SLOT_SIZE] ALLOC_IN_SDRAM;
static ScanCacheFlashRecord g_scan_cache_flash_record ALLOC_IN_SDRAM;
static ScanCacheFlashRecord g_scan_cache_flash_crc_record ALLOC_IN_SDRAM;
static SokobanScanCachePayload g_scan_cache_flash_payload ALLOC_IN_SDRAM;

SokobanScanCachePayload* scan_cache_flash_payload(void) {
    return &g_scan_cache_flash_payload;
}

static uint8_t* scan_cache_flash_image_bytes(void) {
    return (uint8_t*)g_scan_cache_flash_image;
}

static bool scan_cache_flash_ready(void) {
    static bool ready = false;
    if (ready) return true;
    if (flash_init() != 0u) return false;
    ready = true;
    return true;
}

static uint32_t scan_cache_flash_crc32(const void* data, uint32_t len) {
    const uint8_t* p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static uint32_t scan_cache_flash_record_crc(const ScanCacheFlashRecord* record) {
    if (!record) return 0;
    memcpy(&g_scan_cache_flash_crc_record, record, sizeof(g_scan_cache_flash_crc_record));
    g_scan_cache_flash_crc_record.data_crc = 0;
    return scan_cache_flash_crc32(&g_scan_cache_flash_crc_record,
                                  (uint32_t)sizeof(g_scan_cache_flash_crc_record));
}

static uint32_t scan_cache_flash_key_crc(const SokobanScanCacheKey* key);

static const uint8_t g_scan_cache_flash_pages[SCAN_CACHE_FLASH_PAGE_COUNT] = {
    0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u
};

static bool scan_cache_flash_load_image(void) {
    if (!scan_cache_flash_ready()) return false;
    for (uint32_t page = 0; page < SCAN_CACHE_FLASH_PAGE_COUNT; page++) {
        flash_read_page(SCAN_CACHE_FLASH_SECTION, g_scan_cache_flash_pages[page],
                        &g_scan_cache_flash_image[page * SCAN_CACHE_FLASH_PAGE_WORDS],
                        (uint16_t)SCAN_CACHE_FLASH_PAGE_WORDS);
    }
    return true;
}

static bool scan_cache_flash_save_page(uint32_t page) {
    if (page >= SCAN_CACHE_FLASH_PAGE_COUNT || !scan_cache_flash_ready()) return false;
    return flash_write_page(SCAN_CACHE_FLASH_SECTION, g_scan_cache_flash_pages[page],
                            &g_scan_cache_flash_image[page * SCAN_CACHE_FLASH_PAGE_WORDS],
                            (uint16_t)SCAN_CACHE_FLASH_PAGE_WORDS) == 0u;
}

bool scan_cache_flash_clear(void) {
    if (!scan_cache_flash_ready()) return false;
    for (uint32_t page = 0; page < SCAN_CACHE_FLASH_PAGE_COUNT; page++) {
        if (flash_erase_page(SCAN_CACHE_FLASH_SECTION, g_scan_cache_flash_pages[page]) != 0u) {
            return false;
        }
    }
    memset(g_scan_cache_flash_image, 0xFF, sizeof(g_scan_cache_flash_image));
    memset(g_scan_cache_flash_slot, 0xFF, sizeof(g_scan_cache_flash_slot));
    memset(&g_scan_cache_flash_record, 0, sizeof(g_scan_cache_flash_record));
    memset(&g_scan_cache_flash_crc_record, 0, sizeof(g_scan_cache_flash_crc_record));
    memset(&g_scan_cache_flash_payload, 0, sizeof(g_scan_cache_flash_payload));
    return true;
}

static uint8_t scan_cache_flash_dir_code(Direction d) {
    if (d.dx == 0 && d.dy == -1) return 0;
    if (d.dx == 0 && d.dy == 1) return 1;
    if (d.dx == -1 && d.dy == 0) return 2;
    if (d.dx == 1 && d.dy == 0) return 3;
    if (d.dx == 0 && d.dy == 0) return 4;
    return 0xFFu;
}

static bool scan_cache_flash_dir_from_code(uint8_t code, Direction* out) {
    if (!out) return false;
    switch (code) {
        case 0: out->dx = 0; out->dy = -1; return true;
        case 1: out->dx = 0; out->dy = 1; return true;
        case 2: out->dx = -1; out->dy = 0; return true;
        case 3: out->dx = 1; out->dy = 0; return true;
        case 4: out->dx = 0; out->dy = 0; return true;
        default: return false;
    }
}

static bool scan_cache_flash_pack_path(const Direction* path, uint16_t len,
                                       uint8_t* out, uint16_t* out_len) {
    if (!path || !out || !out_len) return false;
    uint32_t bit_count = (uint32_t)len * 3u;
    uint16_t byte_count = (uint16_t)((bit_count + 7u) / 8u);
    if (byte_count > SCAN_CACHE_FLASH_MAX_PACKED_PATH) return false;

    memset(out, 0, SCAN_CACHE_FLASH_MAX_PACKED_PATH);
    for (uint16_t i = 0; i < len; i++) {
        uint8_t code = scan_cache_flash_dir_code(path[i]);
        if (code == 0xFFu) return false;
        uint32_t bit_pos = (uint32_t)i * 3u;
        for (uint8_t bit = 0; bit < 3; bit++) {
            if ((code & (1u << bit)) != 0) {
                uint32_t dst_bit = bit_pos + bit;
                out[dst_bit / 8u] |= (uint8_t)(1u << (dst_bit % 8u));
            }
        }
    }
    *out_len = byte_count;
    return true;
}

static bool scan_cache_flash_unpack_path(const uint8_t* packed, uint16_t path_len,
                                         uint16_t packed_len, Direction* out) {
    if (!packed || !out) return false;
    uint32_t need_bytes = (((uint32_t)path_len * 3u) + 7u) / 8u;
    if (packed_len > SCAN_CACHE_FLASH_MAX_PACKED_PATH || need_bytes > packed_len) return false;

    for (uint16_t i = 0; i < path_len; i++) {
        uint8_t code = 0;
        uint32_t bit_pos = (uint32_t)i * 3u;
        for (uint8_t bit = 0; bit < 3; bit++) {
            uint32_t src_bit = bit_pos + bit;
            if ((packed[src_bit / 8u] & (1u << (src_bit % 8u))) != 0) {
                code |= (uint8_t)(1u << bit);
            }
        }
        if (!scan_cache_flash_dir_from_code(code, &out[i])) return false;
    }
    return true;
}

static bool scan_cache_flash_key_equal(const SokobanScanCacheKey* a,
                                       const SokobanScanCacheKey* b) {
    return a && b && memcmp(a, b, sizeof(*a)) == 0;
}

static bool scan_cache_flash_record_valid(const ScanCacheFlashRecord* record) {
    if (!record) return false;
    if (record->magic != SCAN_CACHE_FLASH_MAGIC) return false;
    if (record->version != SCAN_CACHE_FLASH_VERSION) return false;
    if (record->record_size != sizeof(ScanCacheFlashRecord)) return false;
    if (record->key.policy_version != SOKOBAN_SCAN_CACHE_POLICY_VERSION) return false;
    if (record->key.cache_kind != SOKOBAN_FLASH_CACHE_KIND_SCAN &&
        record->key.cache_kind != SOKOBAN_FLASH_CACHE_KIND_DIRECT) return false;
    if (record->key.cache_kind == SOKOBAN_FLASH_CACHE_KIND_SCAN &&
        (record->path_len == 0 || record->path_len >= MAX_PATH_LENGTH)) return false;
    if (record->key.cache_kind == SOKOBAN_FLASH_CACHE_KIND_DIRECT &&
        (record->path_len == 0 || record->path_len >= MAX_PATH_LENGTH)) return false;
    if (record->packed_path_len > SCAN_CACHE_FLASH_MAX_PACKED_PATH) return false;
    if (record->waypoint_count > SOKOBAN_SCAN_CACHE_MAX_WAYPOINTS) {
        return false;
    }
    if (record->key.cache_kind == SOKOBAN_FLASH_CACHE_KIND_SCAN &&
        record->waypoint_count == 0) return false;
    if (record->key.key_crc != scan_cache_flash_key_crc(&record->key)) return false;
    return scan_cache_flash_record_crc(record) == record->data_crc;
}

bool scan_cache_flash_find(const SokobanScanCacheKey* key,
                           SokobanScanCachePayload* out) {
    if (!key || !out) return false;
    if (!scan_cache_flash_load_image()) return false;

    uint8_t* image = scan_cache_flash_image_bytes();
    for (uint32_t slot = 0; slot < SCAN_CACHE_FLASH_SLOT_COUNT; slot++) {
        const ScanCacheFlashRecord* record =
            (const ScanCacheFlashRecord*)&image[slot * SCAN_CACHE_FLASH_SLOT_SIZE];
        if (!scan_cache_flash_record_valid(record)) continue;
        if (!scan_cache_flash_key_equal(&record->key, key)) continue;

        memset(out, 0, sizeof(*out));
        out->path_len = record->path_len;
        out->waypoint_count = record->waypoint_count;
        out->end_player = record->end_player;
        memcpy(out->after_walls, record->after_walls, sizeof(out->after_walls));
        memcpy(out->waypoints, record->waypoints,
               record->waypoint_count * sizeof(Entity));
        memcpy(out->pause_positions, record->pause_positions,
               record->waypoint_count * sizeof(Position));
        return scan_cache_flash_unpack_path(record->packed_path, record->path_len,
                                            record->packed_path_len, out->path);
    }
    return false;
}

bool scan_cache_flash_store(const SokobanScanCacheKey* key,
                            const SokobanScanCachePayload* payload) {
    if (!key || !payload) return false;
    if (!scan_cache_flash_load_image()) return false;

    uint8_t* image = scan_cache_flash_image_bytes();
    int target_slot = -1;
    int empty_slot = -1;
    int oldest_slot = 0;
    uint32_t oldest_write = 0xFFFFFFFFu;
    uint32_t max_write = 0;

    for (uint32_t slot = 0; slot < SCAN_CACHE_FLASH_SLOT_COUNT; slot++) {
        ScanCacheFlashRecord* record =
            (ScanCacheFlashRecord*)&image[slot * SCAN_CACHE_FLASH_SLOT_SIZE];
        bool valid = scan_cache_flash_record_valid(record);
        if (valid) {
            if (record->write_counter > max_write) max_write = record->write_counter;
            if (record->write_counter < oldest_write) {
                oldest_write = record->write_counter;
                oldest_slot = (int)slot;
            }
            if (scan_cache_flash_key_equal(&record->key, key)) {
                target_slot = (int)slot;
            }
        } else if (empty_slot < 0) {
            empty_slot = (int)slot;
        }
    }

    if (target_slot < 0) {
        target_slot = (empty_slot >= 0) ? empty_slot : oldest_slot;
    }

    memset(&g_scan_cache_flash_record, 0, sizeof(g_scan_cache_flash_record));
    g_scan_cache_flash_record.magic = SCAN_CACHE_FLASH_MAGIC;
    g_scan_cache_flash_record.version = SCAN_CACHE_FLASH_VERSION;
    g_scan_cache_flash_record.record_size = sizeof(g_scan_cache_flash_record);
    g_scan_cache_flash_record.write_counter = max_write + 1u;
    g_scan_cache_flash_record.key = *key;
    g_scan_cache_flash_record.path_len = payload->path_len;
    g_scan_cache_flash_record.waypoint_count = payload->waypoint_count;
    g_scan_cache_flash_record.end_player = payload->end_player;
    memcpy(g_scan_cache_flash_record.after_walls, payload->after_walls,
           sizeof(g_scan_cache_flash_record.after_walls));
    memcpy(g_scan_cache_flash_record.waypoints, payload->waypoints,
           payload->waypoint_count * sizeof(Entity));
    memcpy(g_scan_cache_flash_record.pause_positions, payload->pause_positions,
           payload->waypoint_count * sizeof(Position));
    if (!scan_cache_flash_pack_path(payload->path, payload->path_len,
                                    g_scan_cache_flash_record.packed_path,
                                    &g_scan_cache_flash_record.packed_path_len)) {
        return false;
    }
    g_scan_cache_flash_record.data_crc =
        scan_cache_flash_record_crc(&g_scan_cache_flash_record);

    memset(g_scan_cache_flash_slot, 0xFF, sizeof(g_scan_cache_flash_slot));
    memcpy(g_scan_cache_flash_slot, &g_scan_cache_flash_record,
           sizeof(g_scan_cache_flash_record));
    memcpy(&image[(uint32_t)target_slot * SCAN_CACHE_FLASH_SLOT_SIZE],
           g_scan_cache_flash_slot, sizeof(g_scan_cache_flash_slot));
    return scan_cache_flash_save_page(
        (uint32_t)target_slot / SCAN_CACHE_FLASH_SLOTS_PER_PAGE);
}

static uint32_t scan_cache_flash_key_crc(const SokobanScanCacheKey* key) {
    SokobanScanCacheKey temp;
    const uint8_t* p;
    uint32_t crc = 0xFFFFFFFFu;
    if (!key) return 0u;
    temp = *key;
    temp.key_crc = 0u;
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

static bool scan_cache_flash_key_kind(const SokobanScanCacheKey* key,
                                      SokobanFlashCacheKind kind) {
    return key && key->cache_kind == (uint8_t)kind;
}

bool scan_cache_flash_find_direct(const SokobanScanCacheKey* key,
                                  SokobanScanCachePayload* out) {
    if (!scan_cache_flash_key_kind(key, SOKOBAN_FLASH_CACHE_KIND_DIRECT)) return false;
    return scan_cache_flash_find(key, out);
}

bool scan_cache_flash_store_direct(const SokobanScanCacheKey* key,
                                   const SokobanScanCachePayload* payload) {
    if (!scan_cache_flash_key_kind(key, SOKOBAN_FLASH_CACHE_KIND_DIRECT)) return false;
    if (!payload || payload->waypoint_count != 0 ||
        payload->path_len == 0 || payload->path_len >= MAX_PATH_LENGTH) return false;
    return scan_cache_flash_store(key, payload);
}
