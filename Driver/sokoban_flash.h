#ifndef SOKOBAN_FLASH_H
#define SOKOBAN_FLASH_H

#include "sokoban_types.h"

SokobanScanCachePayload* scan_cache_flash_payload(void);
bool scan_cache_flash_find(const SokobanScanCacheKey* key, SokobanScanCachePayload* out);
bool scan_cache_flash_store(const SokobanScanCacheKey* key, const SokobanScanCachePayload* payload);
bool scan_cache_flash_find_direct(const SokobanScanCacheKey* key, SokobanScanCachePayload* out);
bool scan_cache_flash_store_direct(const SokobanScanCacheKey* key, const SokobanScanCachePayload* payload);
bool scan_cache_flash_clear(void);

#endif
