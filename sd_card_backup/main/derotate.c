#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "derotate.h"

static const char *TAG = "derotate";

/* Set to 1 if the CPU reads stale data from the camera buffer (garbled/old pixels
 * in the scan test). Needs the source buffer to be cache-line aligned. */
#ifndef DEROTATE_INVALIDATE_SRC
#define DEROTATE_INVALIDATE_SRC 0
#endif

/* Log the average time per frame every N frames (0 = off). */
#ifndef DEROTATE_LOG_EVERY
#define DEROTATE_LOG_EVERY 60
#endif

#define SRC_W       DEROTATE_SRC_W
#define SRC_H       DEROTATE_SRC_H
#define SRC_STRIDE  (SRC_W * 3 / 2)          /* 1200 bytes per row */
#define DST_W       DEROTATE_DST_W
#define DST_H       DEROTATE_DST_H
#define DST_STRIDE  (DST_W * 3 / 2)          /* 960 bytes per row  */

/* Q16 coordinates of the source centre, +0.5 so that >>16 rounds to nearest */
#define Q16_BASE    ((int32_t)(399.5 * 65536.0) + (1 << 15))

static uint32_t s_row_off[SRC_H];            /* y * SRC_STRIDE                  */
static uint16_t s_col_y[SRC_W];              /* byte offset of Y for column x   */
static uint16_t s_col_c[SRC_W];              /* byte offset of U/V for column x */

typedef struct {
    const uint8_t *src;
    uint8_t *dst;
    int32_t c, s;                            /* cos, sin in Q16 */
    int pair_begin, pair_end;                /* range of output row PAIRS */
} derot_job_t;

static derot_job_t s_job;
static SemaphoreHandle_t s_go, s_done;
static bool s_ready;

#define Y_AT(X, Y)  src[s_row_off[(Y) >> 16] + s_col_y[(X) >> 16]]

static void derot_rows(const derot_job_t *job)
{
    const uint8_t *restrict src = job->src;
    uint8_t *restrict dst = job->dst;
    const int32_t c = job->c, s = job->s;

    /* Q16 offsets of the other pixels of a 2x2 output block, relative to its top-left */
    const int32_t d10x = c,  d10y = s;                    /* right        */
    const int32_t d01x = -s, d01y = c;                    /* below        */
    const int32_t d11x = c - s, d11y = s + c;             /* below-right  */
    const int32_t dchx = (c - s) >> 1, dchy = (s + c) >> 1;   /* block centre */
    const int32_t c2 = 2 * c, s2 = 2 * s;                 /* step of 2 output pixels */

    for (int j = job->pair_begin; j < job->pair_end; j++) {
        /* Source position of output pixel (0, 2j); output centre is (319.5, 239.5):
         *   sx = cx + dx*cos - dy*sin ;  sy = cy + dx*sin + dy*cos */
        int32_t k  = 4 * j - 479;                         /* 2 * dy */
        int32_t ax = Q16_BASE - ((639 * c) >> 1) - ((k * s) >> 1);
        int32_t ay = Q16_BASE - ((639 * s) >> 1) + ((k * c) >> 1);

        uint8_t *d0 = dst + (size_t)(2 * j) * DST_STRIDE;  /* U Y Y row */
        uint8_t *d1 = d0 + DST_STRIDE;                     /* V Y Y row */

        for (int i = 0; i < DST_W / 2; i++, d0 += 3, d1 += 3, ax += c2, ay += s2) {
            d0[1] = Y_AT(ax,        ay);
            d0[2] = Y_AT(ax + d10x, ay + d10y);
            d1[1] = Y_AT(ax + d01x, ay + d01y);
            d1[2] = Y_AT(ax + d11x, ay + d11y);

            int32_t cx = (ax + dchx) >> 16;
            int32_t cy = (ay + dchy) >> 16;
            uint32_t col = s_col_c[cx];
            d0[0] = src[s_row_off[cy & ~1] + col];         /* U */
            d1[0] = src[s_row_off[cy |  1] + col];         /* V */
        }
    }

    /* make this core's writes visible to the encoder's DMA */
    esp_err_t err = esp_cache_msync(dst + (size_t)job->pair_begin * 2 * DST_STRIDE,
                                    (size_t)(job->pair_end - job->pair_begin) * 2 * DST_STRIDE,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cache msync failed: %d", (int)err);
    }
}

static void derot_worker(void *arg)
{
    for (;;) {
        xSemaphoreTake(s_go, portMAX_DELAY);
        derot_rows(&s_job);
        xSemaphoreGive(s_done);
    }
}

/* The mapping is affine, so checking the four output corners proves that every
 * sample (including the chroma block centres) stays inside the source. */
static bool derot_in_bounds(int32_t c, int32_t s)
{
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            int32_t dx2 = i ? 639 : -639;                  /* 2*dx at left/right columns */
            int32_t dy2 = j ? 479 : -479;                  /* 2*dy at top/bottom rows    */
            int32_t sx = Q16_BASE + ((dx2 * c - dy2 * s) >> 1);
            int32_t sy = Q16_BASE + ((dx2 * s + dy2 * c) >> 1);
            if (sx < 0 || sy < 0 || (sx >> 16) >= SRC_W || (sy >> 16) >= SRC_H) {
                return false;
            }
        }
    }
    return true;
}

esp_err_t derotate_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }
    for (int y = 0; y < SRC_H; y++) {
        s_row_off[y] = (uint32_t)y * SRC_STRIDE;
    }
    for (int x = 0; x < SRC_W; x++) {
        s_col_c[x] = (x >> 1) * 3;
        s_col_y[x] = (x >> 1) * 3 + 1 + (x & 1);
    }
    s_go   = xSemaphoreCreateBinary();
    s_done = xSemaphoreCreateBinary();
    if (!s_go || !s_done) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(derot_worker, "derot", 4096, NULL, 5, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_ready = true;
    return ESP_OK;
}

uint8_t *derotate_alloc_output(void)
{
    return heap_caps_aligned_calloc(128, 1, DEROTATE_DST_SIZE, MALLOC_CAP_SPIRAM);
}

uint8_t *derotate_800x800_to_640x480(const uint8_t *src, uint8_t *dst, float angle_rad)
{
    if (!s_ready || !src || !dst || !isfinite(angle_rad)) {
        return NULL;
    }

    int32_t c = lrintf(cosf(angle_rad) * 65536.0f);
    int32_t s = lrintf(sinf(angle_rad) * 65536.0f);
    if (!derot_in_bounds(c, s)) {
        ESP_LOGE(TAG, "sample out of range, frame skipped");
        return NULL;
    }

#if DEROTATE_INVALIDATE_SRC
    esp_cache_msync((void *)src, SRC_H * SRC_STRIDE, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
#endif

#if DEROTATE_LOG_EVERY
    int64_t t0 = esp_timer_get_time();
#endif

    const int pairs = DST_H / 2;                           /* 240 row pairs */
    s_job = (derot_job_t){ src, dst, c, s, pairs / 2, pairs };
    xSemaphoreGive(s_go);                                  /* second half on core 1 */

    derot_job_t mine = { src, dst, c, s, 0, pairs / 2 };
    derot_rows(&mine);                                     /* first half on this core */

    xSemaphoreTake(s_done, portMAX_DELAY);

#if DEROTATE_LOG_EVERY
    static int64_t total_us;
    static int frames;
    total_us += esp_timer_get_time() - t0;
    if (++frames >= DEROTATE_LOG_EVERY) {
        ESP_LOGI(TAG, "avg %lld us/frame over %d frames", (long long)(total_us / frames), frames);
        total_us = 0;
        frames = 0;
    }
#endif
    return dst;
}
