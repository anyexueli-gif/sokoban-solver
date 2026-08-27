#include "pc_flash.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define PC_FLASH_MKDIR(path) _mkdir(path)
#else
#define PC_FLASH_MKDIR(path) mkdir((path), 0777)
#endif

#define PC_FLASH_PAGE_COUNT 8u
#define PC_FLASH_FILE_SIZE (FLASH_PAGE_SIZE * PC_FLASH_PAGE_COUNT)
#define PC_FLASH_WORDS_PER_PAGE (FLASH_PAGE_SIZE / sizeof(uint32_t))
#define PC_FLASH_SCAN_CACHE_SECTION 127u

static int pc_flash_path_exists(const char* path) {
    struct stat st;
    return path && stat(path, &st) == 0;
}

static const char* pc_flash_dir(void) {
    if (pc_flash_path_exists("Driver")) return "FLASH";
    if (pc_flash_path_exists("../Driver")) return "../FLASH";
    return "FLASH";
}

static const char* pc_flash_path(void) {
    if (pc_flash_path_exists("Driver")) return "FLASH/scan_cache.bin";
    if (pc_flash_path_exists("../Driver")) return "../FLASH/scan_cache.bin";
    return "FLASH/scan_cache.bin";
}

static void pc_flash_ensure_dir(void) {
    const char* dir = pc_flash_dir();
    if (!pc_flash_path_exists(dir)) {
        (void)PC_FLASH_MKDIR(dir);
    }
}

static uint32_t pc_flash_page_offset(flash_page_enum page_num) {
    if ((uint32_t)page_num >= PC_FLASH_PAGE_COUNT) return PC_FLASH_FILE_SIZE;
    return (uint32_t)page_num * FLASH_PAGE_SIZE;
}

static uint8_t pc_flash_load(uint8_t image[PC_FLASH_FILE_SIZE]) {
    pc_flash_ensure_dir();
    memset(image, 0xFF, PC_FLASH_FILE_SIZE);

    FILE* f = fopen(pc_flash_path(), "rb");
    if (!f) {
        f = fopen(pc_flash_path(), "wb");
        if (!f) return 1u;
        size_t written = fwrite(image, 1, PC_FLASH_FILE_SIZE, f);
        fclose(f);
        return (written == PC_FLASH_FILE_SIZE) ? 0u : 1u;
    }

    size_t read = fread(image, 1, PC_FLASH_FILE_SIZE, f);
    fclose(f);
    if (read < PC_FLASH_FILE_SIZE) {
        FILE* out = fopen(pc_flash_path(), "wb");
        if (!out) return 1u;
        size_t written = fwrite(image, 1, PC_FLASH_FILE_SIZE, out);
        fclose(out);
        return (written == PC_FLASH_FILE_SIZE) ? 0u : 1u;
    }

    return 0u;
}

static uint8_t pc_flash_save(const uint8_t image[PC_FLASH_FILE_SIZE]) {
    pc_flash_ensure_dir();
    FILE* f = fopen(pc_flash_path(), "wb");
    if (!f) return 1u;
    size_t written = fwrite(image, 1, PC_FLASH_FILE_SIZE, f);
    fclose(f);
    return (written == PC_FLASH_FILE_SIZE) ? 0u : 1u;
}

uint8_t flash_init(void) {
    uint8_t image[PC_FLASH_FILE_SIZE];
    return pc_flash_load(image);
}

void flash_read_page(uint32_t sector_num, flash_page_enum page_num, uint32_t* buf, uint16_t len) {
    if (!buf) return;

    uint32_t word_count = len;
    if (word_count > PC_FLASH_WORDS_PER_PAGE) word_count = PC_FLASH_WORDS_PER_PAGE;
    memset(buf, 0xFF, (size_t)word_count * sizeof(uint32_t));

    if (sector_num != PC_FLASH_SCAN_CACHE_SECTION) return;

    uint8_t image[PC_FLASH_FILE_SIZE];
    if (pc_flash_load(image) != 0u) return;

    uint32_t offset = pc_flash_page_offset(page_num);
    if (offset >= PC_FLASH_FILE_SIZE) return;
    memcpy(buf, &image[offset], (size_t)word_count * sizeof(uint32_t));
}

uint8_t flash_write_page(uint32_t sector_num, flash_page_enum page_num, const uint32_t* buf, uint16_t len) {
    if (!buf || sector_num != PC_FLASH_SCAN_CACHE_SECTION) return 1u;

    uint32_t word_count = len;
    if (word_count > PC_FLASH_WORDS_PER_PAGE) return 1u;

    uint8_t image[PC_FLASH_FILE_SIZE];
    if (pc_flash_load(image) != 0u) return 1u;

    uint32_t offset = pc_flash_page_offset(page_num);
    if (offset >= PC_FLASH_FILE_SIZE) return 1u;

    memset(&image[offset], 0xFF, FLASH_PAGE_SIZE);
    memcpy(&image[offset], buf, (size_t)word_count * sizeof(uint32_t));
    return pc_flash_save(image);
}

uint8_t flash_erase_page(uint32_t sector_num, flash_page_enum page_num) {
    if (sector_num != PC_FLASH_SCAN_CACHE_SECTION) return 1u;

    uint8_t image[PC_FLASH_FILE_SIZE];
    if (pc_flash_load(image) != 0u) return 1u;

    uint32_t offset = pc_flash_page_offset(page_num);
    if (offset >= PC_FLASH_FILE_SIZE) return 1u;

    memset(&image[offset], 0xFF, FLASH_PAGE_SIZE);
    return pc_flash_save(image);
}
