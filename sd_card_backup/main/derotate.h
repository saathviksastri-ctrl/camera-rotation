#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Source: camera frame, 800x800, ISP "YUV420" packed layout (O_UYY_E_VYY):
 *   even rows: U Y Y U Y Y ...    odd rows: V Y Y V Y Y ...   (1.5 bytes/pixel)
 * Output: same layout, 640x480. */
#define DEROTATE_SRC_W     800
#define DEROTATE_SRC_H     800
#define DEROTATE_DST_W     640
#define DEROTATE_DST_H     480
#define DEROTATE_DST_SIZE  (DEROTATE_DST_W * DEROTATE_DST_H * 3 / 2)   /* 460800 bytes */

/* Call once at startup (builds lookup tables, starts the core-1 worker). */
esp_err_t derotate_init(void);

/* Allocates one output buffer (PSRAM, 128-byte aligned, zeroed). NULL on failure. */
uint8_t *derotate_alloc_output(void);

/* Rotates the 800x800 frame by angle_rad (about its centre) and writes the central
 * 640x480 window into dst. Returns dst, or NULL if the angle is invalid
 * (NaN/inf) or a sample would fall outside the source (frame left untouched).
 * Not re-entrant: call from one task only. Blocks until both cores are done. */
uint8_t *derotate_800x800_to_640x480(const uint8_t *src, uint8_t *dst, float angle_rad);

#ifdef __cplusplus
}
#endif
