/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "example_video_common.h"

#include "esp_muxer.h"
#include "mp4_muxer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "derotate.h"
static uint8_t *s_rot_buf;

#define CHUNK_SECONDS 60
#define TARGET_FPS    30

#if CONFIG_EXAMPLE_FORMAT_MJPEG
#define ENCODE_DEV_PATH        ESP_VIDEO_JPEG_DEVICE_NAME
#define ENCODE_OUTPUT_FORMAT   V4L2_PIX_FMT_JPEG
#elif CONFIG_EXAMPLE_FORMAT_H264
#if CONFIG_EXAMPLE_H264_MAX_QP <= CONFIG_EXAMPLE_H264_MIN_QP
#error "CONFIG_EXAMPLE_H264_MAX_QP should larger than CONFIG_EXAMPLE_H264_MIN_QP"
#endif

#define ENCODE_DEV_PATH        ESP_VIDEO_H264_DEVICE_NAME
#define ENCODE_OUTPUT_FORMAT   V4L2_PIX_FMT_H264
#define CAPTURE_SECONDS 3
#endif

#define VIDEO_BUFFER_COUNT         4
#define VIDEO_ENCODER_BUFFER_COUNT 4
#define SKIP_STARTUP_FRAME_COUNT   2
#define EXAMPLE_MAX_CHAR_SIZE      64
#define MOUNT_POINT                CONFIG_EXAMPLE_SDMMC_MOUNT_POINT

typedef enum {
    SD_IMG_FORMAT_JPEG,
    SD_IMG_FORMAT_H264,
} sd_image_format_t;

typedef struct sd_card_fb {
    uint8_t *buf;
    size_t buf_bytesused;
    sd_image_format_t fmt;
    size_t width;
    size_t height;
    struct timeval timestamp;
    uint32_t m2m_index;
} sd_card_fb_t;

typedef struct image_sd_card {
    int cap_fd;
    uint32_t format;
    uint8_t *cap_buffer[VIDEO_BUFFER_COUNT];

    int m2m_fd;
    uint8_t *m2m_cap_buffer[VIDEO_ENCODER_BUFFER_COUNT];

    sd_card_fb_t sd_fb;

    example_storage_handle_t fatfs;
} image_sd_card_t;

typedef struct {
    uint8_t *data;
    size_t len;
    uint32_t pts;
    bool key_frame;
    uint32_t m2m_index;
} video_queue_item_t;

static const char *TAG = "example";
static QueueHandle_t s_video_queue = NULL;

static void print_video_device_info(const struct v4l2_capability *capability)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability->version >> 16),
             (uint8_t)(capability->version >> 8),
             (uint8_t)capability->version);
    ESP_LOGI(TAG, "driver:  %s", capability->driver);
    ESP_LOGI(TAG, "card:    %s", capability->card);
    ESP_LOGI(TAG, "bus:     %s", capability->bus_info);
}

static esp_err_t init_capture_video(image_sd_card_t *sd)
{
    int fd = open(EXAMPLE_CAM_DEV_PATH, O_RDONLY);
    assert(fd >= 0);

    struct v4l2_capability capability;
    ESP_ERROR_CHECK(ioctl(fd, VIDIOC_QUERYCAP, &capability));
    print_video_device_info(&capability);

    sd->cap_fd = fd;
    return ESP_OK;
}

static esp_err_t set_codec_control(int fd, uint32_t ctrl_class, uint32_t id, int32_t value)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    controls.ctrl_class = ctrl_class;
    controls.count = 1;
    controls.controls = control;
    control[0].id = id;
    control[0].value = value;

    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set control: %" PRIu32, id);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t init_codec_video(image_sd_card_t *sd)
{
    int fd = open(ENCODE_DEV_PATH, O_RDONLY);
    assert(fd >= 0);

    struct v4l2_capability capability;
    ESP_ERROR_CHECK(ioctl(fd, VIDIOC_QUERYCAP, &capability));

#if CONFIG_EXAMPLE_FORMAT_MJPEG
    set_codec_control(fd, V4L2_CID_JPEG_CLASS, V4L2_CID_JPEG_COMPRESSION_QUALITY, CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY);
#elif CONFIG_EXAMPLE_FORMAT_H264
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, CONFIG_EXAMPLE_H264_I_PERIOD);
    // Set 1.5 Mbps bitrate target to ensure smooth writing to SD card
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_BITRATE, 1500000);
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_H264_MIN_QP, CONFIG_EXAMPLE_H264_MIN_QP);
    set_codec_control(fd, V4L2_CID_CODEC_CLASS, V4L2_CID_MPEG_VIDEO_H264_MAX_QP, CONFIG_EXAMPLE_H264_MAX_QP);
#endif

    sd->format = ENCODE_OUTPUT_FORMAT;
    sd->m2m_fd = fd;

    return ESP_OK;
}

static esp_err_t example_video_start(image_sd_card_t *sd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_buffer buf;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    struct v4l2_format init_format;
    uint32_t capture_fmt = 0;
    uint32_t width, height;

    memset(&init_format, 0, sizeof(struct v4l2_format));
    init_format.type = type;
    if (ioctl(sd->cap_fd, VIDIOC_G_FMT, &init_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        return ESP_FAIL;
    }
    width = init_format.fmt.pix.width;
    height = init_format.fmt.pix.height;

    /* derotation needs exactly this camera frame size */
    if (width != DEROTATE_SRC_W || height != DEROTATE_SRC_H) {
        ESP_LOGE(TAG, "camera must output %dx%d for derotation, got %ux%u",
                 DEROTATE_SRC_W, DEROTATE_SRC_H, (unsigned)width, (unsigned)height);
        return ESP_FAIL;
    }

    if (sd->format == V4L2_PIX_FMT_JPEG) {
        capture_fmt = V4L2_PIX_FMT_RGB565;
    } else if (sd->format == V4L2_PIX_FMT_H264) {
        capture_fmt = V4L2_PIX_FMT_YUV420;
    }

    /* 1. Camera Capture Config */
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = VIDEO_BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_REQBUFS, &req));

    for (int i = 0; i < VIDEO_BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_QUERYBUF, &buf));

        sd->cap_buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                            MAP_SHARED, sd->cap_fd, buf.m.offset);
        assert(sd->cap_buffer[i]);
        ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_QBUF, &buf));
    }

    /* 2. Codec Input Config (m2m OUTPUT) */
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = DEROTATE_DST_W;
    format.fmt.pix.height = DEROTATE_DST_H;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = VIDEO_ENCODER_BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_USERPTR;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_REQBUFS, &req));

    // Replace block 3 of example_video_start() with this:
    /* 3. Codec Encoded Output Config (m2m CAPTURE) */
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = DEROTATE_DST_W;
    format.fmt.pix.height = DEROTATE_DST_H;
    format.fmt.pix.pixelformat = sd->format;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = VIDEO_ENCODER_BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_REQBUFS, &req));

    for (int i = 0; i < VIDEO_ENCODER_BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_QUERYBUF, &buf));

        sd->m2m_cap_buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                                MAP_SHARED, sd->m2m_fd, buf.m.offset);
        assert(sd->m2m_cap_buffer[i]);
        // NOTE: DO NOT call ioctl(sd->m2m_fd, VIDIOC_QBUF, &buf) here!
    }

    // Queue ONLY initial buffer 0 to prime the pipeline:
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = 0;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_QBUF, &buf));

    /* 4. Enable Streams */
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_STREAMON, &type));
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_STREAMON, &type));

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_STREAMON, &type));

    for (int i = 0; i < SKIP_STARTUP_FRAME_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_DQBUF, &buf));
        ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_QBUF, &buf));
    }

    sd->sd_fb.width  = DEROTATE_DST_W;
    sd->sd_fb.height = DEROTATE_DST_H;
    sd->sd_fb.fmt    = sd->format == V4L2_PIX_FMT_JPEG ? SD_IMG_FORMAT_JPEG : SD_IMG_FORMAT_H264;

    return ESP_OK;
}

static void example_video_stop(image_sd_card_t *sd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(sd->cap_fd, VIDIOC_STREAMOFF, &type);

    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(sd->m2m_fd, VIDIOC_STREAMOFF, &type);
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(sd->m2m_fd, VIDIOC_STREAMOFF, &type);
}

float rotAngle = 0.0f;

static sd_card_fb_t *example_video_fb_get(image_sd_card_t *sd)
{
    struct v4l2_buffer cap_buf;
    struct v4l2_buffer m2m_out_buf;
    struct v4l2_buffer m2m_cap_buf;
    int64_t us;

    memset(&cap_buf, 0, sizeof(cap_buf));
    cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_DQBUF, &cap_buf));

    /* --- derotation: camera frame (800x800) -> s_rot_buf (640x480) --- */
    float angle = 0.0f;   /* TODO: angle from the IMU for this frame */
    uint8_t *rot = derotate_800x800_to_640x480(sd->cap_buffer[cap_buf.index], s_rot_buf, rotAngle);
    if (!rot) {
        rot = s_rot_buf;  /* bad angle: re-send the previous rotated frame */
    }

    /* camera buffer is no longer needed: give it back right away */
    ESP_ERROR_CHECK(ioctl(sd->cap_fd, VIDIOC_QBUF, &cap_buf));

    memset(&m2m_out_buf, 0, sizeof(m2m_out_buf));
    m2m_out_buf.index     = 0;
    m2m_out_buf.type      = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    m2m_out_buf.memory    = V4L2_MEMORY_USERPTR;
    m2m_out_buf.m.userptr = (unsigned long)rot;
    m2m_out_buf.length    = DEROTATE_DST_SIZE;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_QBUF, &m2m_out_buf));

    memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
    m2m_cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_DQBUF, &m2m_cap_buf));

    ESP_ERROR_CHECK(ioctl(sd->m2m_fd, VIDIOC_DQBUF, &m2m_out_buf));

    sd->sd_fb.buf           = sd->m2m_cap_buffer[0];
    sd->sd_fb.buf_bytesused = m2m_cap_buf.bytesused;
    
    us = esp_timer_get_time();
    sd->sd_fb.timestamp.tv_sec  = us / 1000000UL;
    sd->sd_fb.timestamp.tv_usec = us % 1000000UL;

    rotAngle += 0.17f;

    return &sd->sd_fb;
}

static int mp4_url_pattern(char *name, int len, int slice_idx)
{
    snprintf(name, len, "%s/vid_%04d.mp4", MOUNT_POINT, slice_idx);
    return 0;
}

static esp_muxer_handle_t muxer_begin(const image_sd_card_t *sd, int *vstream)
{
    mp4_muxer_config_t cfg = {0};
    cfg.base_config.muxer_type     = ESP_MUXER_TYPE_MP4;
    cfg.base_config.url_pattern    = mp4_url_pattern;
    cfg.base_config.slice_duration = CHUNK_SECONDS * 1000;
    cfg.base_config.ram_cache_size = 256 * 1024;

    esp_muxer_handle_t m = esp_muxer_open(&cfg.base_config, sizeof(cfg));
    if (!m) return NULL;

    esp_muxer_video_stream_info_t vi = {
        .codec  = ESP_MUXER_VDEC_H264,
        .width  = sd->sd_fb.width,
        .height = sd->sd_fb.height,
        .fps    = TARGET_FPS,
    };

    if (esp_muxer_add_video_stream(m, &vi, vstream) != ESP_MUXER_ERR_OK) {
        esp_muxer_close(m);
        return NULL;
    }
    return m;
}

static bool h264_is_keyframe(const uint8_t *d, size_t n)
{
    size_t search_len = n < 128 ? n : 128;
    for (size_t i = 0; i + 4 < search_len; i++) {
        if (d[i] == 0 && d[i + 1] == 0 &&
            (d[i + 2] == 1 || (d[i + 2] == 0 && d[i + 3] == 1))) {
            size_t h = i + (d[i + 2] == 1 ? 3 : 4);
            if ((d[h] & 0x1F) == 5) {
                return true;
            }
        }
    }
    return false;
}

static void sd_write_task(void *pvParameters)
{
    image_sd_card_t *sd = (image_sd_card_t *)pvParameters;
    int vstream = 0;

    esp_muxer_handle_t muxer = muxer_begin(sd, &vstream);
    if (!muxer) {
        ESP_LOGE(TAG, "Muxer init failed");
        vTaskDelete(NULL);
        return;
    }

    video_queue_item_t item;
    while (1) {
        if (xQueueReceive(s_video_queue, &item, portMAX_DELAY) == pdTRUE) {
            esp_muxer_video_packet_t pkt = {
                .data      = item.data,
                .len       = item.len,
                .pts       = item.pts,
                .dts       = item.pts,
                .key_frame = item.key_frame,
            };

            if (esp_muxer_add_video_packet(muxer, vstream, &pkt) != ESP_MUXER_ERR_OK) {
                ESP_LOGW(TAG, "Failed to mux frame");
            }

            free(item.data);
        }
    }

    esp_muxer_close(muxer);
    vTaskDelete(NULL);
}

static esp_err_t store_video_to_sd_card(image_sd_card_t *sd)
{
    ESP_ERROR_CHECK(derotate_init());
    s_rot_buf = derotate_alloc_output();
    assert(s_rot_buf);
    if (mp4_muxer_register() != ESP_MUXER_ERR_OK) {
        ESP_LOGE(TAG, "mp4 muxer register failed");
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(example_video_start(sd));

    s_video_queue = xQueueCreate(20, sizeof(video_queue_item_t));
    if (!s_video_queue) {
        ESP_LOGE(TAG, "Failed to create video queue");
        return ESP_FAIL;
    }

    xTaskCreatePinnedToCore(sd_write_task, "sd_write_task", 8192, sd, 5, NULL, 1);

    int64_t t0 = esp_timer_get_time();
    while (1) {
        sd_card_fb_t *fb = example_video_fb_get(sd);
        uint32_t pts_ms = (esp_timer_get_time() - t0) / 1000;

        uint8_t *frame_copy = malloc(fb->buf_bytesused);
        if (frame_copy) {
            memcpy(frame_copy, fb->buf, fb->buf_bytesused);

            video_queue_item_t item = {
                .data      = frame_copy,
                .len       = fb->buf_bytesused,
                .pts       = pts_ms,
                .key_frame = h264_is_keyframe(fb->buf, fb->buf_bytesused),
            };

            if (xQueueSend(s_video_queue, &item, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Queue full, frame dropped!");
                free(frame_copy);
            }
        }

        // Return buffer 0 back to V4L2 driver to unlock the encoder for the next frame
        struct v4l2_buffer m2m_cap_buf;
        memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
        m2m_cap_buf.index  = 0;
        m2m_cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
        ioctl(sd->m2m_fd, VIDIOC_QBUF, &m2m_cap_buf);
    }

    example_video_stop(sd);
    return ESP_OK;
}

static esp_err_t init_sd_card(image_sd_card_t *sd)
{
    return example_mount_fatfs_to_mmc(&sd->fatfs);
}

static esp_err_t deinit_sd_card(image_sd_card_t *sd)
{
    return example_unmount_fatfs_in_mmc(sd->fatfs);
}

void app_main(void)
{
    image_sd_card_t *sdc = calloc(1, sizeof(image_sd_card_t));
    assert(sdc);

    ESP_ERROR_CHECK(example_video_init());
    ESP_ERROR_CHECK(init_capture_video(sdc));
    ESP_ERROR_CHECK(init_codec_video(sdc));
    ESP_ERROR_CHECK(init_sd_card(sdc));
    ESP_ERROR_CHECK(store_video_to_sd_card(sdc));
    ESP_ERROR_CHECK(deinit_sd_card(sdc));
    free(sdc);
}