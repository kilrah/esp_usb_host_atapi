/*
 * USB Host ATAPI driver - MMC command set
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "atapi_private.h"

static const char *TAG = "atapi_mmc";

/* MMC opcodes */
#define MMC_TEST_UNIT_READY     0x00
#define MMC_REQUEST_SENSE       0x03
#define MMC_START_STOP_UNIT     0x1B
#define MMC_PREVENT_ALLOW       0x1E
#define MMC_READ_10             0x28
#define MMC_READ_TOC            0x43
#define MMC_GET_EVENT_STATUS    0x4A
#define MMC_MODE_SENSE_10       0x5A
#define MMC_SET_STREAMING       0xB6
#define MMC_SET_CD_SPEED        0xBB
#define MMC_READ_CD             0xBE

static inline void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = v >> 8;
    p[1] = v;
}

static inline void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static inline uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* ------------------------------------------------------------------------ */
/* Command execution with auto sense                                        */
/* ------------------------------------------------------------------------ */

static esp_err_t request_sense(atapi_device_t *dev, atapi_sense_t *sense)
{
    uint8_t cdb[6] = {MMC_REQUEST_SENSE, 0, 0, 0, 18, 0};
    uint8_t resp[18] = {0};
    size_t len = 0;
    ESP_RETURN_ON_ERROR(atapi_bot_command(dev, cdb, sizeof(cdb), resp, sizeof(resp), true,
                                          ATAPI_TIMEOUT_DEFAULT_MS, &len), TAG, "REQUEST SENSE failed");
    sense->key = len > 2 ? (resp[2] & 0x0F) : 0;
    sense->asc = len > 12 ? resp[12] : 0;
    sense->ascq = len > 13 ? resp[13] : 0;
    return ESP_OK;
}

/* Execute a command with the device lock held. Fetches sense data on CHECK CONDITION. */
static esp_err_t run_cmd(atapi_device_t *dev, const uint8_t *cdb, size_t cdb_len,
                         void *data, size_t data_len, bool data_in, uint32_t timeout_ms, size_t *actual)
{
    xSemaphoreTake(dev->lock, portMAX_DELAY);
    esp_err_t err = atapi_bot_command(dev, cdb, cdb_len, data, data_len, data_in, timeout_ms, actual);
    if (err == ESP_ERR_ATAPI_CHECK_CONDITION) {
        atapi_sense_t sense = {0};
        if (request_sense(dev, &sense) == ESP_OK) {
            dev->last_sense = sense;
            ESP_LOGD(TAG, "CMD 0x%02x: sense %X/%02X/%02X (%s)", cdb[0], sense.key, sense.asc, sense.ascq,
                     atapi_sense_to_str(&sense));
            if (sense.key == ATAPI_SENSE_NOT_READY && sense.asc == 0x3A) {
                err = ESP_ERR_ATAPI_NO_MEDIUM;
            }
        }
    }
    xSemaphoreGive(dev->lock);
    return err;
}

esp_err_t atapi_host_execute_cdb(atapi_host_device_handle_t handle,
                                 const uint8_t *cdb, size_t cdb_len,
                                 void *data, size_t data_len, bool data_in,
                                 uint32_t timeout_ms, size_t *actual_len)
{
    ESP_RETURN_ON_FALSE(handle && cdb, ESP_ERR_INVALID_ARG, TAG, "");
    ESP_RETURN_ON_FALSE(data_len <= ATAPI_MAX_XFER_SIZE, ESP_ERR_INVALID_SIZE, TAG, "transfer too large");
    return run_cmd(handle, cdb, cdb_len, data, data_len, data_in,
                   timeout_ms ? timeout_ms : ATAPI_TIMEOUT_DEFAULT_MS, actual_len);
}

esp_err_t atapi_host_get_last_sense(atapi_host_device_handle_t handle, atapi_sense_t *sense)
{
    ESP_RETURN_ON_FALSE(handle && sense, ESP_ERR_INVALID_ARG, TAG, "");
    *sense = handle->last_sense;
    return ESP_OK;
}

const char *atapi_sense_to_str(const atapi_sense_t *s)
{
    switch (s->asc) {
    case 0x00: return s->key == 0 ? "No sense" : "No additional sense information";
    case 0x04:
        switch (s->ascq) {
        case 0x01: return "Becoming ready";
        case 0x02: return "Initializing command required";
        case 0x07: return "Operation in progress";
        default:   return "Not ready";
        }
    case 0x11: return "Unrecovered read error";
    case 0x15: return "Positioning error";
    case 0x20: return "Invalid command opcode";
    case 0x21: return "Logical block address out of range";
    case 0x24: return "Invalid field in CDB";
    case 0x28: return "Medium may have changed";
    case 0x29: return "Power on / reset";
    case 0x30: return "Incompatible medium";
    case 0x3A:
        switch (s->ascq) {
        case 0x01: return "Medium not present - tray closed";
        case 0x02: return "Medium not present - tray open";
        default:   return "Medium not present";
        }
    case 0x53: return "Media load or eject failed";
    case 0x57: return "Unable to recover TOC";
    case 0x64: return "Illegal mode for this track";
    default:   return "Unknown";
    }
}

/* ------------------------------------------------------------------------ */
/* Unit state                                                               */
/* ------------------------------------------------------------------------ */

esp_err_t atapi_host_test_unit_ready(atapi_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[6] = {MMC_TEST_UNIT_READY};
    return run_cmd(handle, cdb, sizeof(cdb), NULL, 0, false, ATAPI_TIMEOUT_DEFAULT_MS, NULL);
}

esp_err_t atapi_host_wait_ready(atapi_host_device_handle_t handle, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    TickType_t start = xTaskGetTickCount();
    esp_err_t err;

    do {
        err = atapi_host_test_unit_ready(handle);
        if (err == ESP_OK || err == ESP_ERR_ATAPI_NO_MEDIUM) {
            return err;
        }
        if (err != ESP_ERR_ATAPI_CHECK_CONDITION) {
            return err; /* transport error */
        }
        const atapi_sense_t *s = &handle->last_sense;
        if (s->key == ATAPI_SENSE_UNIT_ATTENTION || s->key == ATAPI_SENSE_NO_SENSE) {
            continue; /* media changed / reset: retry immediately */
        }
        if (s->key != ATAPI_SENSE_NOT_READY) {
            ESP_LOGW(TAG, "Unexpected sense while waiting: %X/%02X/%02X (%s)", s->key, s->asc, s->ascq,
                     atapi_sense_to_str(s));
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(250)); /* becoming ready */
    } while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms));

    return ESP_ERR_ATAPI_NOT_READY;
}

esp_err_t atapi_host_get_media_status(atapi_host_device_handle_t handle, atapi_media_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle && status, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[10] = {MMC_GET_EVENT_STATUS, 0x01 /* polled */, 0, 0, 0x10 /* media class */};
    uint8_t resp[8] = {0};
    size_t len = 0;
    put_be16(&cdb[7], sizeof(resp));

    ESP_RETURN_ON_ERROR(run_cmd(handle, cdb, sizeof(cdb), resp, sizeof(resp), true,
                                ATAPI_TIMEOUT_DEFAULT_MS, &len), TAG, "GESN failed");

    memset(status, 0, sizeof(*status));
    /* NEA bit set or wrong class: no media event data */
    if (len < 6 || (resp[2] & 0x80) || (resp[2] & 0x07) != 4) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    status->event = (atapi_media_event_t)(resp[4] & 0x0F);
    status->tray_open = (resp[5] & 0x01) != 0;
    status->media_present = (resp[5] & 0x02) != 0;
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* TOC                                                                      */
/* ------------------------------------------------------------------------ */

/* Parse a format 0 TOC response and check that it is consistent */
static esp_err_t parse_toc(const uint8_t *resp, size_t len, atapi_toc_t *toc)
{
    memset(toc, 0, sizeof(*toc));
    size_t data_len = MIN(len, (size_t)(((resp[0] << 8) | resp[1]) + 2));
    if (data_len < 4 + 8) {
        ESP_LOGW(TAG, "TOC too short (%u bytes)", (unsigned)data_len);
        return ESP_ERR_INVALID_RESPONSE;
    }
    toc->first_track = resp[2];
    toc->last_track = resp[3];

    bool have_leadout = false;
    for (size_t off = 4; off + 8 <= data_len; off += 8) {
        const uint8_t *d = &resp[off];
        uint8_t track = d[2];
        uint32_t lba = get_be32(&d[4]);
        if (track == 0xAA) {
            toc->leadout_lba = lba;
            have_leadout = true;
        } else if (track >= 1 && track <= ATAPI_MAX_TRACKS && toc->num_tracks < ATAPI_MAX_TRACKS) {
            atapi_toc_entry_t *e = &toc->tracks[toc->num_tracks++];
            e->number = track;
            e->adr = d[1] >> 4;
            e->control = d[1] & 0x0F;
            e->lba = lba;
        }
    }

    /* Consistency: guards against corrupted transfers (seen once with a JMicron USB-IDE bridge) */
    const char *problem = NULL;
    if (toc->num_tracks == 0 || !have_leadout) {
        problem = "no tracks or lead-out";
    } else if (toc->first_track < 1 || toc->last_track < toc->first_track ||
               toc->num_tracks != toc->last_track - toc->first_track + 1) {
        problem = "track count doesn't match the header";
    } else {
        for (int i = 0; i < toc->num_tracks && !problem; i++) {
            uint32_t next = i + 1 < toc->num_tracks ? toc->tracks[i + 1].lba : toc->leadout_lba;
            if (toc->tracks[i].number != toc->first_track + i) {
                problem = "track numbers not consecutive";
            } else if (toc->tracks[i].lba >= next) {
                problem = "track addresses not increasing";
            }
        }
    }
    if (problem) {
        ESP_LOGW(TAG, "Invalid TOC (%s), %u bytes:", problem, (unsigned)data_len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, resp, data_len, ESP_LOG_WARN);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t read_toc_once(atapi_device_t *dev, uint8_t *resp, size_t alloc, atapi_toc_t *toc)
{
    /* Two steps, like Linux sr: the header first (plus the first descriptor), then exactly the TOC length.
     * A USB-IDE bridge (JMicron 152d:2338) hung (bulk IN timed out) when the allocation length exceeded
     * the data the drive returned, instead of ending the transfer with a short packet. */
    uint8_t cdb[10] = {MMC_READ_TOC, 0x00 /* LBA format */, 0x00 /* format 0: TOC */, 0, 0, 0, 1 /* from track 1 */};
    const size_t hdr_len = 12;
    put_be16(&cdb[7], hdr_len);
    size_t len = 0;
    ESP_RETURN_ON_ERROR(run_cmd(dev, cdb, sizeof(cdb), resp, hdr_len, true, ATAPI_TIMEOUT_LONG_MS, &len),
                        TAG, "READ TOC header failed");
    size_t want = len >= 2 ? ((resp[0] << 8) | resp[1]) + 2 : 0;
    if (want < 4 + 8 || want > alloc) {
        ESP_LOGW(TAG, "TOC header reports %u bytes, reading the maximum", (unsigned)want);
        want = alloc;
    }
    if (want > len) {
        memset(resp, 0, alloc);
        put_be16(&cdb[7], want);
        ESP_RETURN_ON_ERROR(run_cmd(dev, cdb, sizeof(cdb), resp, want, true, ATAPI_TIMEOUT_LONG_MS, &len),
                            TAG, "READ TOC failed");
    }
    return parse_toc(resp, len, toc);
}

esp_err_t atapi_host_read_toc(atapi_host_device_handle_t handle, atapi_toc_t *toc)
{
    ESP_RETURN_ON_FALSE(handle && toc, ESP_ERR_INVALID_ARG, TAG, "");

    const size_t alloc = 4 + (ATAPI_MAX_TRACKS + 1) * 8; /* header + 99 tracks + lead-out */
    uint8_t *resp = calloc(1, alloc);
    ESP_RETURN_ON_FALSE(resp, ESP_ERR_NO_MEM, TAG, "");

    esp_err_t err = ESP_ERR_INVALID_RESPONSE;
    for (int attempt = 0; attempt < 3 && err == ESP_ERR_INVALID_RESPONSE; attempt++) {
        err = read_toc_once(handle, resp, alloc, toc);
    }
    free(resp);
    return err;
}

/* ------------------------------------------------------------------------ */
/* Reading                                                                  */
/* ------------------------------------------------------------------------ */

static esp_err_t read_sectors(atapi_device_t *dev, uint32_t lba, uint32_t count, uint8_t *buf,
                              size_t sector_size, bool audio)
{
    const uint32_t max_per_cmd = ATAPI_MAX_XFER_SIZE / sector_size;

    while (count) {
        uint32_t n = MIN(count, max_per_cmd);
        size_t bytes = n * sector_size;
        size_t actual = 0;
        uint8_t cdb[12] = {0};
        size_t cdb_len;

        if (audio) {
            cdb[0] = MMC_READ_CD;
            cdb[1] = 0x01 << 2; /* expected sector type: CD-DA */
            put_be32(&cdb[2], lba);
            cdb[6] = n >> 16;
            cdb[7] = n >> 8;
            cdb[8] = n;
            cdb[9] = 0x10;      /* user data (2352 bytes for CD-DA) */
            cdb[10] = 0x00;     /* no sub-channel */
            cdb_len = 12;
        } else {
            cdb[0] = MMC_READ_10;
            put_be32(&cdb[2], lba);
            put_be16(&cdb[7], n);
            cdb_len = 10;
        }

        esp_err_t err = run_cmd(dev, cdb, cdb_len, buf, bytes, true, ATAPI_TIMEOUT_LONG_MS, &actual);
        if (err != ESP_OK) {
            if (err == ESP_ERR_ATAPI_CHECK_CONDITION) {
                const atapi_sense_t *s = &dev->last_sense;
                ESP_LOGW(TAG, "Read LBA %"PRIu32"+%"PRIu32" failed: %X/%02X/%02X (%s)", lba, n,
                         s->key, s->asc, s->ascq, atapi_sense_to_str(s));
            }
            return err;
        }
        if (actual != bytes) {
            ESP_LOGW(TAG, "Short read at LBA %"PRIu32": %u of %u bytes", lba, (unsigned)actual, (unsigned)bytes);
            return ESP_ERR_INVALID_SIZE;
        }
        lba += n;
        count -= n;
        buf += bytes;
    }
    return ESP_OK;
}

esp_err_t atapi_host_read_cd_audio(atapi_host_device_handle_t handle, uint32_t lba, uint32_t count, void *buf)
{
    ESP_RETURN_ON_FALSE(handle && buf, ESP_ERR_INVALID_ARG, TAG, "");
    return read_sectors(handle, lba, count, buf, ATAPI_CD_AUDIO_SECTOR_SIZE, true);
}

esp_err_t atapi_host_read_data(atapi_host_device_handle_t handle, uint32_t lba, uint32_t count, void *buf)
{
    ESP_RETURN_ON_FALSE(handle && buf, ESP_ERR_INVALID_ARG, TAG, "");
    return read_sectors(handle, lba, count, buf, ATAPI_CD_DATA_SECTOR_SIZE, false);
}

/* ------------------------------------------------------------------------ */
/* Drive control                                                            */
/* ------------------------------------------------------------------------ */

esp_err_t atapi_host_set_speed(atapi_host_device_handle_t handle, uint16_t read_kbps)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[12] = {MMC_SET_CD_SPEED};
    put_be16(&cdb[2], read_kbps);
    put_be16(&cdb[4], 0xFFFF); /* write speed: don't care */
    return run_cmd(handle, cdb, sizeof(cdb), NULL, 0, false, ATAPI_TIMEOUT_DEFAULT_MS, NULL);
}

esp_err_t atapi_host_set_streaming(atapi_host_device_handle_t handle, uint32_t end_lba, uint16_t read_kbps)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    /* Performance descriptor (MMC-5 6.39): read_kbps kB per 1000 ms over LBA 0..end_lba. Write values are
     * required to be non-zero by some drives, so they get the same numbers. */
    uint8_t desc[28] = {0};
    desc[0] = 0x00;             /* WRC 0, RDD 0 (don't restore defaults), Exact 0, RA 0 */
    put_be32(&desc[4], 0);      /* start LBA */
    put_be32(&desc[8], end_lba);
    put_be32(&desc[12], read_kbps); /* read size, kB */
    put_be32(&desc[16], 1000);      /* read time, ms */
    put_be32(&desc[20], read_kbps); /* write size, kB */
    put_be32(&desc[24], 1000);      /* write time, ms */
    uint8_t cdb[12] = {MMC_SET_STREAMING};
    cdb[8] = 0x00; /* type: performance descriptor */
    put_be16(&cdb[9], sizeof(desc));
    return run_cmd(handle, cdb, sizeof(cdb), desc, sizeof(desc), false, ATAPI_TIMEOUT_DEFAULT_MS, NULL);
}

esp_err_t atapi_host_get_speed(atapi_host_device_handle_t handle, uint16_t *cur_kbps, uint16_t *max_kbps)
{
    ESP_RETURN_ON_FALSE(handle && cur_kbps && max_kbps, ESP_ERR_INVALID_ARG, TAG, "");
    /* MODE SENSE(10), page 2Ah. Header first, then the exact length (some bridges stall on a larger
     * allocation length than the reply, see READ TOC). */
    uint8_t cdb[10] = {MMC_MODE_SENSE_10, 0x08 /* DBD */, 0x2A};
    uint8_t hdr[8] = {0};
    size_t len = 0;
    put_be16(&cdb[7], sizeof(hdr));
    ESP_RETURN_ON_ERROR(run_cmd(handle, cdb, sizeof(cdb), hdr, sizeof(hdr), true, ATAPI_TIMEOUT_DEFAULT_MS, &len),
                        TAG, "MODE SENSE header");
    size_t total = ((hdr[0] << 8) | hdr[1]) + 2;
    uint8_t buf[64] = {0};
    total = MIN(total, sizeof(buf));
    ESP_RETURN_ON_FALSE(total >= 8 + 16, ESP_ERR_INVALID_RESPONSE, TAG, "page 2Ah too short");
    put_be16(&cdb[7], total);
    ESP_RETURN_ON_ERROR(run_cmd(handle, cdb, sizeof(cdb), buf, total, true, ATAPI_TIMEOUT_DEFAULT_MS, &len),
                        TAG, "MODE SENSE");
    const uint8_t *page = &buf[8 + ((buf[6] << 8) | buf[7])]; /* skip block descriptors, if any */
    ESP_RETURN_ON_FALSE(page + 16 <= buf + len && (page[0] & 0x3F) == 0x2A, ESP_ERR_INVALID_RESPONSE, TAG,
                        "bad page 2Ah");
    *max_kbps = (page[8] << 8) | page[9];
    *cur_kbps = (page[14] << 8) | page[15];
    return ESP_OK;
}

esp_err_t atapi_host_start_stop(atapi_host_device_handle_t handle, bool start, bool load_eject)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[6] = {MMC_START_STOP_UNIT, 0, 0, 0, (load_eject ? 0x02 : 0) | (start ? 0x01 : 0), 0};
    return run_cmd(handle, cdb, sizeof(cdb), NULL, 0, false, ATAPI_TIMEOUT_LONG_MS, NULL);
}

esp_err_t atapi_host_prevent_removal(atapi_host_device_handle_t handle, bool prevent)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[6] = {MMC_PREVENT_ALLOW, 0, 0, 0, prevent ? 1 : 0, 0};
    return run_cmd(handle, cdb, sizeof(cdb), NULL, 0, false, ATAPI_TIMEOUT_DEFAULT_MS, NULL);
}
