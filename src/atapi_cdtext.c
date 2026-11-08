/*
 * USB Host ATAPI driver - CD-TEXT (READ TOC format 0101b)
 *
 * CD-TEXT is stored in the lead-in as 18-byte packs:
 *   [0] pack type (0x80 title, 0x81 performer, ... 0x8F size info)
 *   [1] track number (bit 7: extension flag)
 *   [2] sequence number
 *   [3] bit 7: double byte chars, bits 6-4: block number, bits 3-0: char position
 *   [4..15] 12 bytes of text
 *   [16..17] CRC
 * The text of all packs of one type is a stream of NUL-terminated strings, the
 * first belonging to the track number of the first pack and each following one
 * to the next track. A lone TAB means "same as previous track".
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_check.h"
#include "atapi_private.h"

static const char *TAG = "atapi_cdtext";

#define PACK_SIZE        18
#define PACK_TEXT_LEN    12
#define PACK_TYPE_FIRST  0x80
#define PACK_TYPE_DISCID 0x86
#define PACK_TYPE_UPC    0x8E
#define PACK_TYPE_SIZE   0x8F

#define CHARSET_ISO8859_1 0x00
#define CHARSET_ASCII     0x01

static int pack_type_to_field(uint8_t type)
{
    if (type >= PACK_TYPE_FIRST && type <= PACK_TYPE_DISCID) {
        return type - PACK_TYPE_FIRST; /* title .. disc id */
    }
    if (type == PACK_TYPE_UPC) {
        return ATAPI_CDTEXT_UPC_ISRC;
    }
    return -1;
}

/*
 * Bytes 0x80..0x9F are C1 control characters in ISO 8859-1, which never belong in a title. Burning software
 * (e.g. AnyBurn) writes Windows-1252 there instead: ’ “ ” – — … € ™ Š Œ ... So decode them as Windows-1252.
 * 0 = undefined in Windows-1252.
 */
static const uint16_t CP1252_80_9F[32] = {
    0x20AC, 0, 0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021, 0x02C6, 0x2030, 0x0160, 0x2039, 0x0152, 0, 0x017D, 0,
    0, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014, 0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0, 0x017E, 0x0178,
};

/* Max. UTF-8 bytes per CD-TEXT byte (Windows-1252 0x80..0x9F maps up to U+20AC) */
#define UTF8_MAX_PER_BYTE 3

/* Append @p len bytes to the pool, converting to UTF-8. Returns the string start. */
static char *pool_add(char **wr, const uint8_t *src, size_t len, uint8_t charset)
{
    char *start = *wr;
    char *p = start;
    for (size_t i = 0; i < len; i++) {
        uint8_t c = src[i];
        if (charset == CHARSET_ISO8859_1 && c >= 0x80 && c <= 0x9F) {
            uint16_t u = CP1252_80_9F[c - 0x80];
            if (!u) {
                *p++ = '?';
            } else if (u < 0x800) {
                *p++ = 0xC0 | (u >> 6);
                *p++ = 0x80 | (u & 0x3F);
            } else {
                *p++ = 0xE0 | (u >> 12);
                *p++ = 0x80 | ((u >> 6) & 0x3F);
                *p++ = 0x80 | (u & 0x3F);
            }
        } else if (charset == CHARSET_ISO8859_1 && c >= 0x80) {
            *p++ = 0xC0 | (c >> 6);
            *p++ = 0x80 | (c & 0x3F);
        } else if (charset == CHARSET_ASCII && c >= 0x80) {
            *p++ = '?';
        } else {
            *p++ = c; /* ASCII, or raw for MS-JIS / unknown */
        }
    }
    *p++ = '\0';
    *wr = p;
    return start;
}

esp_err_t atapi_host_read_cd_text(atapi_host_device_handle_t handle, atapi_cd_text_t *cdtext)
{
    ESP_RETURN_ON_FALSE(handle && cdtext, ESP_ERR_INVALID_ARG, TAG, "");
    memset(cdtext, 0, sizeof(*cdtext));

    /* 1. Header only, to learn the size */
    uint8_t cdb[10] = {0x43, 0x00, 0x05 /* CD-TEXT */};
    uint8_t hdr[4] = {0};
    size_t len = 0;
    cdb[8] = sizeof(hdr);
    esp_err_t err = atapi_host_execute_cdb(handle, cdb, sizeof(cdb), hdr, sizeof(hdr), true, 0, &len);
    if (err == ESP_ERR_ATAPI_CHECK_CONDITION) {
        return ESP_ERR_NOT_FOUND; /* typically ILLEGAL REQUEST: no CD-TEXT on this disc */
    }
    ESP_RETURN_ON_ERROR(err, TAG, "CD-TEXT header read failed");
    size_t total = (size_t)((hdr[0] << 8) | hdr[1]) + 2;
    if (len < 4 || total < 4 + PACK_SIZE) {
        return ESP_ERR_NOT_FOUND;
    }
    total = MIN(total, (size_t)ATAPI_MAX_XFER_SIZE);

    /* 2. Full read */
    uint8_t *buf = malloc(total);
    ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "");
    cdb[7] = total >> 8;
    cdb[8] = total;
    err = atapi_host_execute_cdb(handle, cdb, sizeof(cdb), buf, total, true, 0, &len);
    if (err != ESP_OK) {
        free(buf);
        return err == ESP_ERR_ATAPI_CHECK_CONDITION ? ESP_ERR_NOT_FOUND : err;
    }
    size_t num_packs = (MIN(len, total) - 4) / PACK_SIZE;
    const uint8_t *packs = buf + 4;

    /* Character set of block 0 from the first size-info pack */
    cdtext->charset = CHARSET_ISO8859_1;
    for (size_t i = 0; i < num_packs; i++) {
        const uint8_t *pk = packs + i * PACK_SIZE;
        if (pk[0] == PACK_TYPE_SIZE && ((pk[3] >> 4) & 0x07) == 0 && pk[2] != 0xFF) {
            cdtext->charset = pk[4];
            break;
        }
    }

    /* Worst case each byte becomes UTF8_MAX_PER_BYTE UTF-8 bytes, plus terminators */
    cdtext->pool = malloc(num_packs * PACK_TEXT_LEN * UTF8_MAX_PER_BYTE + num_packs + 16);
    if (!cdtext->pool) {
        free(buf);
        return ESP_ERR_NO_MEM;
    }
    char *wr = cdtext->pool;

    /* Per field: accumulate the current string across packs */
    uint8_t acc[160];
    for (int field = 0; field < ATAPI_CDTEXT_FIELD_COUNT; field++) {
        size_t acc_len = 0;
        int track = -1;
        for (size_t i = 0; i < num_packs; i++) {
            const uint8_t *pk = packs + i * PACK_SIZE;
            if (pack_type_to_field(pk[0]) != field || ((pk[3] >> 4) & 0x07) != 0) {
                continue; /* other field, or not block 0 */
            }
            if (track < 0) {
                track = pk[1] & 0x7F;
            }
            for (int j = 0; j < PACK_TEXT_LEN; j++) {
                uint8_t c = pk[4 + j];
                if (c != '\0') {
                    if (acc_len < sizeof(acc)) {
                        acc[acc_len++] = c;
                    }
                    continue;
                }
                /* End of string for 'track' */
                if (track <= ATAPI_MAX_TRACKS && acc_len > 0) {
                    if (acc_len == 1 && acc[0] == '\t' && track > 0) {
                        cdtext->text[field][track] = cdtext->text[field][track - 1];
                    } else {
                        cdtext->text[field][track] = pool_add(&wr, acc, acc_len, cdtext->charset);
                    }
                }
                acc_len = 0;
                track++;
            }
        }
    }

    free(buf);

    bool any = false;
    for (int f = 0; f < ATAPI_CDTEXT_FIELD_COUNT && !any; f++) {
        for (int t = 0; t <= ATAPI_MAX_TRACKS && !any; t++) {
            any = cdtext->text[f][t] != NULL;
        }
    }
    if (!any) {
        atapi_cd_text_free(cdtext);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

void atapi_cd_text_free(atapi_cd_text_t *cdtext)
{
    if (cdtext) {
        free(cdtext->pool);
        memset(cdtext, 0, sizeof(*cdtext));
    }
}
