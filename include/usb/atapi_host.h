/*
 * USB Host ATAPI / MMC driver for optical drives
 *
 * Talks to CD/DVD drives that are exposed over USB Mass Storage Bulk-Only
 * Transport (typically a SATA/PATA optical drive behind a USB bridge).
 * Commands are sent as SCSI MMC packets (the same packets ATAPI carries),
 * which gives access to raw CD-DA audio sectors via READ CD.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Constants                                                                */
/* ------------------------------------------------------------------------ */

#define ATAPI_CD_AUDIO_SECTOR_SIZE   2352 /*!< Raw CD-DA sector: 588 stereo 16-bit LE samples */
#define ATAPI_CD_DATA_SECTOR_SIZE    2048 /*!< Mode 1 / Mode 2 Form 1 user data */
#define ATAPI_CD_FRAMES_PER_SECOND   75   /*!< CD sectors (frames) per second of audio */
#define ATAPI_CD_MSF_OFFSET          150  /*!< LBA 0 == MSF 00:02:00 */
#define ATAPI_MAX_TRACKS             99

#define ATAPI_PERIPHERAL_TYPE_CDROM  0x05 /*!< INQUIRY peripheral device type for CD/DVD */

#define ATAPI_SPEED_MAX              0xFFFF /*!< Use with atapi_host_set_speed() for the drive's maximum */
#define ATAPI_SPEED_1X_KBPS          176    /*!< 1x CD-DA speed in kB/s as used by SET CD SPEED */

/* Error codes (in addition to standard esp_err_t values) */
#define ESP_ERR_ATAPI_BASE           0x7A000
#define ESP_ERR_ATAPI_CHECK_CONDITION (ESP_ERR_ATAPI_BASE + 1) /*!< Command failed, sense data available */
#define ESP_ERR_ATAPI_NO_MEDIUM      (ESP_ERR_ATAPI_BASE + 2) /*!< No disc in drive (sense 02/3A) */
#define ESP_ERR_ATAPI_NOT_READY      (ESP_ERR_ATAPI_BASE + 3) /*!< Drive did not become ready in time */
#define ESP_ERR_ATAPI_TRANSPORT      (ESP_ERR_ATAPI_BASE + 4) /*!< USB / Bulk-Only Transport failure */
#define ESP_ERR_ATAPI_STALL          (ESP_ERR_ATAPI_BASE + 5) /*!< Endpoint stalled (internal) */

/* Common sense keys */
#define ATAPI_SENSE_NO_SENSE         0x00
#define ATAPI_SENSE_RECOVERED_ERROR  0x01
#define ATAPI_SENSE_NOT_READY        0x02
#define ATAPI_SENSE_MEDIUM_ERROR     0x03
#define ATAPI_SENSE_HARDWARE_ERROR   0x04
#define ATAPI_SENSE_ILLEGAL_REQUEST  0x05
#define ATAPI_SENSE_UNIT_ATTENTION   0x06

/* ------------------------------------------------------------------------ */
/* Types                                                                    */
/* ------------------------------------------------------------------------ */

typedef struct atapi_host_device *atapi_host_device_handle_t;

typedef enum {
    ATAPI_DEVICE_CONNECTED,    /*!< A USB mass storage device that may be an optical drive was connected */
    ATAPI_DEVICE_DISCONNECTED, /*!< An installed device was disconnected; call atapi_host_uninstall_device() */
} atapi_host_event_id_t;

typedef struct {
    atapi_host_event_id_t event;
    union {
        uint8_t address;                   /*!< USB address, for ATAPI_DEVICE_CONNECTED */
        atapi_host_device_handle_t handle; /*!< Device handle, for ATAPI_DEVICE_DISCONNECTED */
    } device;
} atapi_host_event_t;

typedef void (*atapi_host_event_cb_t)(const atapi_host_event_t *event, void *arg);

typedef struct {
    bool create_background_task; /*!< Create a task that handles USB client events */
    unsigned task_priority;      /*!< Priority of the background task */
    size_t stack_size;           /*!< Stack size of the background task */
    BaseType_t core_id;          /*!< Core of the background task (tskNO_AFFINITY allowed) */
    atapi_host_event_cb_t callback; /*!< Connection/disconnection callback (required) */
    void *callback_arg;
} atapi_host_driver_config_t;

typedef struct {
    uint8_t key;  /*!< Sense key */
    uint8_t asc;  /*!< Additional sense code */
    uint8_t ascq; /*!< Additional sense code qualifier */
} atapi_sense_t;

typedef struct {
    uint16_t vid;
    uint16_t pid;
    uint8_t usb_address;
    uint8_t interface_subclass; /*!< 0x02 = SFF-8020i/MMC (ATAPI), 0x05 = SFF-8070i, 0x06 = SCSI transparent */
    uint8_t peripheral_type;    /*!< 0x05 for CD/DVD drives */
    bool removable;
    char vendor[9];
    char product[17];
    char revision[5];
} atapi_host_device_info_t;

typedef struct {
    uint8_t number;  /*!< Track number (1..99) */
    uint8_t adr;     /*!< Sub-channel Q ADR field */
    uint8_t control; /*!< Sub-channel Q control field (see helpers below) */
    uint32_t lba;    /*!< Start address (LBA) */
} atapi_toc_entry_t;

typedef struct {
    uint8_t first_track;
    uint8_t last_track;
    uint8_t num_tracks;
    uint32_t leadout_lba; /*!< First LBA after the last track */
    atapi_toc_entry_t tracks[ATAPI_MAX_TRACKS];
} atapi_toc_t;

typedef enum {
    ATAPI_MEDIA_EVENT_NONE = 0,
    ATAPI_MEDIA_EVENT_EJECT_REQUEST = 1, /*!< User pressed the eject button */
    ATAPI_MEDIA_EVENT_NEW_MEDIA = 2,
    ATAPI_MEDIA_EVENT_MEDIA_REMOVAL = 3,
    ATAPI_MEDIA_EVENT_MEDIA_CHANGED = 4,
} atapi_media_event_t;

typedef struct {
    atapi_media_event_t event;
    bool tray_open;
    bool media_present;
} atapi_media_status_t;

typedef enum {
    ATAPI_CDTEXT_TITLE = 0,
    ATAPI_CDTEXT_PERFORMER,
    ATAPI_CDTEXT_SONGWRITER,
    ATAPI_CDTEXT_COMPOSER,
    ATAPI_CDTEXT_ARRANGER,
    ATAPI_CDTEXT_MESSAGE,
    ATAPI_CDTEXT_DISC_ID,
    ATAPI_CDTEXT_UPC_ISRC, /*!< Index 0: UPC/EAN of the disc, tracks: ISRC */
    ATAPI_CDTEXT_FIELD_COUNT
} atapi_cd_text_field_t;

/**
 * @brief CD-TEXT (block 0 / first language only)
 *
 * text[field][0] is the album (disc) value, text[field][n] the value for track n.
 * Missing entries are NULL. Strings are UTF-8 (converted from ISO 8859-1).
 */
typedef struct {
    uint8_t charset; /*!< 0x00 ISO 8859-1, 0x01 ASCII, 0x80 MS-JIS */
    const char *text[ATAPI_CDTEXT_FIELD_COUNT][ATAPI_MAX_TRACKS + 1];
    char *pool;      /*!< Internal string storage, released by atapi_cd_text_free() */
} atapi_cd_text_t;

/** Track is a data track (otherwise audio) */
static inline bool atapi_toc_entry_is_data(const atapi_toc_entry_t *t)
{
    return (t->control & 0x04) != 0;
}

/** Audio track was mastered with pre-emphasis (50/15 us) */
static inline bool atapi_toc_entry_has_preemphasis(const atapi_toc_entry_t *t)
{
    return (t->control & 0x05) == 0x01;
}

/** Number of sectors in track index @p idx of @p toc */
static inline uint32_t atapi_toc_track_length(const atapi_toc_t *toc, int idx)
{
    uint32_t end = (idx + 1 < toc->num_tracks) ? toc->tracks[idx + 1].lba : toc->leadout_lba;
    return end - toc->tracks[idx].lba;
}

/* ------------------------------------------------------------------------ */
/* Driver                                                                   */
/* ------------------------------------------------------------------------ */

/**
 * @brief Install the ATAPI driver (registers a USB Host client).
 *
 * The USB Host Library must already be installed.
 */
esp_err_t atapi_host_install(const atapi_host_driver_config_t *config);

/**
 * @brief Uninstall the driver. All devices must be uninstalled first.
 */
esp_err_t atapi_host_uninstall(void);

/**
 * @brief Handle USB client events. Only needed if create_background_task was false.
 */
esp_err_t atapi_host_handle_events(TickType_t timeout);

/**
 * @brief Open and initialize an optical drive found at @p usb_address.
 *
 * Claims the Bulk-Only interface and issues INQUIRY. Does not require a disc.
 *
 * @return ESP_ERR_NOT_SUPPORTED if the device is not a CD/DVD drive
 */
esp_err_t atapi_host_install_device(uint8_t usb_address, atapi_host_device_handle_t *handle);

esp_err_t atapi_host_uninstall_device(atapi_host_device_handle_t handle);

esp_err_t atapi_host_get_device_info(atapi_host_device_handle_t handle, atapi_host_device_info_t *info);

esp_err_t atapi_host_print_descriptors(atapi_host_device_handle_t handle);

/* ------------------------------------------------------------------------ */
/* Commands                                                                 */
/*                                                                          */
/* All command functions are thread safe (serialized per device). When a    */
/* command returns ESP_ERR_ATAPI_CHECK_CONDITION, the sense data has been   */
/* fetched automatically and is available via atapi_host_get_last_sense().  */
/* ------------------------------------------------------------------------ */

/**
 * @brief Execute an arbitrary MMC/SCSI command.
 *
 * @param cdb        Command descriptor block (6..16 bytes)
 * @param data       Data buffer, or NULL for no data phase
 * @param data_len   Length of data buffer
 * @param data_in    true: device-to-host, false: host-to-device
 * @param timeout_ms Timeout for the data and status phases
 * @param[out] actual_len  Bytes actually transferred (optional)
 */
esp_err_t atapi_host_execute_cdb(atapi_host_device_handle_t handle,
                                 const uint8_t *cdb, size_t cdb_len,
                                 void *data, size_t data_len, bool data_in,
                                 uint32_t timeout_ms, size_t *actual_len);

/** Sense data of the last command that failed with CHECK CONDITION */
esp_err_t atapi_host_get_last_sense(atapi_host_device_handle_t handle, atapi_sense_t *sense);

/** Human readable description of common CD sense codes */
const char *atapi_sense_to_str(const atapi_sense_t *sense);

/** TEST UNIT READY. ESP_OK when a disc is loaded and ready. */
esp_err_t atapi_host_test_unit_ready(atapi_host_device_handle_t handle);

/**
 * @brief Poll TEST UNIT READY until the drive is ready.
 *
 * Swallows UNIT ATTENTION and "becoming ready" conditions.
 *
 * @return ESP_OK, ESP_ERR_ATAPI_NO_MEDIUM, ESP_ERR_ATAPI_NOT_READY (timeout) or other error
 */
esp_err_t atapi_host_wait_ready(atapi_host_device_handle_t handle, uint32_t timeout_ms);

/** GET EVENT STATUS NOTIFICATION (media class, polled). Clears the reported event. */
esp_err_t atapi_host_get_media_status(atapi_host_device_handle_t handle, atapi_media_status_t *status);

/** READ TOC/PMA/ATIP format 0000b in LBA form */
esp_err_t atapi_host_read_toc(atapi_host_device_handle_t handle, atapi_toc_t *toc);

/**
 * @brief Read CD-TEXT from the lead-in (READ TOC format 0101b)
 *
 * On success, release with atapi_cd_text_free(). The struct is ~3.3 KB, avoid the stack.
 *
 * @return ESP_OK, ESP_ERR_NOT_FOUND if the disc has no CD-TEXT, or another error
 */
esp_err_t atapi_host_read_cd_text(atapi_host_device_handle_t handle, atapi_cd_text_t *cdtext);

void atapi_cd_text_free(atapi_cd_text_t *cdtext);

/**
 * @brief Read raw CD-DA sectors (READ CD, expected sector type CD-DA).
 *
 * Output is 16-bit little-endian interleaved stereo (L, R), 44.1 kHz.
 * Large requests are split into multiple USB commands internally.
 *
 * @param buf  Buffer of at least count * ATAPI_CD_AUDIO_SECTOR_SIZE bytes
 */
esp_err_t atapi_host_read_cd_audio(atapi_host_device_handle_t handle, uint32_t lba, uint32_t count, void *buf);

/**
 * @brief Read 2048-byte data sectors (READ(10)).
 */
esp_err_t atapi_host_read_data(atapi_host_device_handle_t handle, uint32_t lba, uint32_t count, void *buf);

/**
 * @brief SET CD SPEED.
 * @param read_kbps Read speed in kB/s (176 = 1x), or ATAPI_SPEED_MAX
 */
esp_err_t atapi_host_set_speed(atapi_host_device_handle_t handle, uint16_t read_kbps);

/**
 * @brief START STOP UNIT
 * @param start  true: spin up, false: spin down
 * @param load_eject  true: load (start=true) or eject (start=false) the tray
 */
esp_err_t atapi_host_start_stop(atapi_host_device_handle_t handle, bool start, bool load_eject);

static inline esp_err_t atapi_host_eject(atapi_host_device_handle_t handle)
{
    return atapi_host_start_stop(handle, false, true);
}

static inline esp_err_t atapi_host_load(atapi_host_device_handle_t handle)
{
    return atapi_host_start_stop(handle, true, true);
}

/** PREVENT ALLOW MEDIUM REMOVAL (locks the eject button) */
esp_err_t atapi_host_prevent_removal(atapi_host_device_handle_t handle, bool prevent);

#ifdef __cplusplus
}
#endif
