/*
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sys/queue.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/atapi_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mass storage interface subclasses that may carry MMC (ATAPI) commands */
#define MSC_SUBCLASS_SFF8020I   0x02 /* ATAPI / MMC-x (CD/DVD) */
#define MSC_SUBCLASS_SFF8070I   0x05
#define MSC_SUBCLASS_SCSI       0x06 /* SCSI transparent - used by most SATA bridges */
#define MSC_PROTOCOL_BOT        0x50

/* Default timeouts */
#define ATAPI_TIMEOUT_DEFAULT_MS   10000
#define ATAPI_TIMEOUT_CBW_MS        5000
#define ATAPI_TIMEOUT_LONG_MS      30000 /* spin-up, eject, ... */

/* Maximum payload of a single BOT data phase (split larger reads) */
#define ATAPI_MAX_XFER_SIZE        (16 * ATAPI_CD_AUDIO_SECTOR_SIZE) /* 37632 B, multiple of 64 */

typedef struct atapi_host_device {
    STAILQ_ENTRY(atapi_host_device) tailq_entry;
    SemaphoreHandle_t lock;          /* Serializes commands */
    SemaphoreHandle_t transfer_done; /* Signalled by transfer callbacks */
    usb_device_handle_t dev_hdl;
    usb_transfer_t *bulk_xfer;       /* Bulk transfers (CBW, data, CSW) */
    usb_transfer_t *ctrl_xfer;       /* Control transfers (reset recovery, clear halt) */
    bool xfer_lost;                  /* A timed-out transfer could not be cancelled: the device is unusable */
    uint8_t usb_address;
    uint8_t iface_num;
    uint8_t subclass;
    uint8_t alt_setting;
    uint8_t ep_in;
    uint8_t ep_out;
    uint16_t mps_in;
    uint32_t tag;
    atapi_sense_t last_sense;
    atapi_host_device_info_t info;
} atapi_device_t;

/**
 * @brief Execute a command without taking the device lock and without auto sense.
 *
 * @return ESP_OK, ESP_ERR_ATAPI_CHECK_CONDITION (CSW status 1) or a transport error
 */
esp_err_t atapi_bot_command(atapi_device_t *dev, const uint8_t *cdb, size_t cdb_len,
                            void *data, size_t data_len, bool data_in,
                            uint32_t timeout_ms, size_t *actual_len);

#ifdef __cplusplus
}
#endif
