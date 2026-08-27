#ifndef PC_FLASH_H
#define PC_FLASH_H

#include <stdint.h>

#define FLASH_PAGE_SIZE 0x00001000u

typedef enum {
    FLASH_PAGE_0,
    FLASH_PAGE_1,
    FLASH_PAGE_2,
    FLASH_PAGE_3,
    FLASH_PAGE_4,
    FLASH_PAGE_5,
    FLASH_PAGE_6,
    FLASH_PAGE_7,
} flash_page_enum;

uint8_t flash_init(void);
void flash_read_page(uint32_t sector_num, flash_page_enum page_num, uint32_t* buf, uint16_t len);
uint8_t flash_write_page(uint32_t sector_num, flash_page_enum page_num, const uint32_t* buf, uint16_t len);
uint8_t flash_erase_page(uint32_t sector_num, flash_page_enum page_num);

#endif
