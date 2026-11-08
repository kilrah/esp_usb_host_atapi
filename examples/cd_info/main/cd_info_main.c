/*
 * usb_host_atapi example: print drive info, TOC and CD-TEXT of the inserted disc
 *
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/atapi_host.h"

static const char *TAG = "cd_info";

typedef struct {
    bool connected;
    uint8_t address;                   /* connected */
    atapi_host_device_handle_t handle; /* disconnected */
} app_event_t;

static QueueHandle_t s_events;

/* Called from the driver's event task: don't send commands here, hand over to the main task */
static void atapi_event_cb(const atapi_host_event_t *event, void *arg)
{
    app_event_t e = {.connected = event->event == ATAPI_DEVICE_CONNECTED};
    if (e.connected) {
        e.address = event->device.address;
    } else {
        e.handle = event->device.handle;
    }
    xQueueSend(s_events, &e, portMAX_DELAY);
}

static void usb_lib_task(void *arg)
{
    const usb_host_config_t host_config = {.intr_flags = ESP_INTR_FLAG_LOWMED};
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    const atapi_host_driver_config_t atapi_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = atapi_event_cb,
    };
    ESP_ERROR_CHECK(atapi_host_install(&atapi_config));
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static const char *cdtext(const atapi_cd_text_t *t, atapi_cd_text_field_t field, int index)
{
    const char *s = t ? t->text[field][index] : NULL;
    return s ? s : "";
}

static void print_disc(atapi_host_device_handle_t dev)
{
    static atapi_toc_t toc;
    esp_err_t err = atapi_host_read_toc(dev, &toc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "READ TOC failed: %s", esp_err_to_name(err));
        return;
    }
    static atapi_cd_text_t text; /* ~3.3 KB: not on the stack */
    bool has_text = atapi_host_read_cd_text(dev, &text) == ESP_OK;
    const atapi_cd_text_t *t = has_text ? &text : NULL;

    ESP_LOGI(TAG, "TOC: tracks %d-%d, lead-out at LBA %"PRIu32"%s", toc.first_track, toc.last_track,
             toc.leadout_lba, has_text ? "" : ", no CD-TEXT");
    if (has_text) {
        ESP_LOGI(TAG, "Album: %s / %s", cdtext(t, ATAPI_CDTEXT_TITLE, 0), cdtext(t, ATAPI_CDTEXT_PERFORMER, 0));
    }
    int first_audio = -1;
    for (int i = 0; i < toc.num_tracks; i++) {
        const atapi_toc_entry_t *e = &toc.tracks[i];
        bool data = atapi_toc_entry_is_data(e);
        uint32_t secs = atapi_toc_track_length(&toc, i) / ATAPI_CD_FRAMES_PER_SECOND;
        ESP_LOGI(TAG, "  %2d %s LBA %6"PRIu32"  %02"PRIu32":%02"PRIu32"  %s", e->number, data ? "data " : "audio",
                 e->lba, secs / 60, secs % 60, cdtext(t, ATAPI_CDTEXT_TITLE, e->number));
        if (!data && first_audio < 0) {
            first_audio = i;
        }
    }
    if (has_text) {
        atapi_cd_text_free(&text);
    }

    if (first_audio >= 0) {
        uint8_t *buf = malloc(ATAPI_CD_AUDIO_SECTOR_SIZE);
        if (buf && atapi_host_read_cd_audio(dev, toc.tracks[first_audio].lba, 1, buf) == ESP_OK) {
            ESP_LOG_BUFFER_HEX(TAG, buf, 16); /* 16-bit little-endian stereo samples */
        }
        free(buf);
    }
}

void app_main(void)
{
    s_events = xQueueCreate(4, sizeof(app_event_t));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "Waiting for an optical drive...");

    atapi_host_device_handle_t dev = NULL;
    bool disc_shown = false;
    while (true) {
        app_event_t e;
        if (xQueueReceive(s_events, &e, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (e.connected && !dev) {
                if (atapi_host_install_device(e.address, &dev) != ESP_OK) {
                    ESP_LOGW(TAG, "Device %d is not a usable optical drive", e.address);
                    dev = NULL;
                    continue;
                }
                atapi_host_device_info_t info;
                atapi_host_get_device_info(dev, &info);
                ESP_LOGI(TAG, "Drive: %s %s (rev %s), bridge VID 0x%04x PID 0x%04x", info.vendor, info.product,
                         info.revision, info.vid, info.pid);
                disc_shown = false;
            } else if (!e.connected && dev && e.handle == dev) {
                ESP_LOGI(TAG, "Drive disconnected");
                atapi_host_uninstall_device(dev);
                dev = NULL;
            }
            continue;
        }
        if (!dev) {
            continue;
        }
        /* Poll the drive: show the disc once it's ready, forget it when it's removed */
        esp_err_t err = atapi_host_wait_ready(dev, 100);
        if (err == ESP_OK && !disc_shown) {
            print_disc(dev);
            disc_shown = true;
        } else if (err == ESP_ERR_ATAPI_NO_MEDIUM && disc_shown) {
            ESP_LOGI(TAG, "Disc removed");
            disc_shown = false;
        }
    }
}
