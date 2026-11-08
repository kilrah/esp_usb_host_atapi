/*
 * USB Host ATAPI driver - USB client handling and Bulk-Only Transport
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/queue.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"
#include "usb/usb_types_ch9.h"
#include "atapi_private.h"

static const char *TAG = "atapi_host";

/* ------------------------------------------------------------------------ */
/* Bulk-Only Transport definitions                                          */
/* ------------------------------------------------------------------------ */

#define CBW_SIGNATURE   0x43425355 /* "USBC" */
#define CSW_SIGNATURE   0x53425355 /* "USBS" */
#define CBW_SIZE        31
#define CSW_SIZE        13
#define CBW_FLAG_IN     0x80

#define CSW_STATUS_PASSED       0x00
#define CSW_STATUS_FAILED       0x01
#define CSW_STATUS_PHASE_ERROR  0x02

#define BOT_REQ_RESET       0xFF
#define CTRL_XFER_SIZE      (USB_SETUP_PACKET_SIZE + 64)

typedef struct __attribute__((packed)) {
    uint32_t signature;
    uint32_t tag;
    uint32_t data_length;
    uint8_t flags;
    uint8_t lun;
    uint8_t cb_length;
    uint8_t cb[16];
} bot_cbw_t;

typedef struct __attribute__((packed)) {
    uint32_t signature;
    uint32_t tag;
    uint32_t residue;
    uint8_t status;
} bot_csw_t;

_Static_assert(sizeof(bot_cbw_t) == CBW_SIZE, "CBW size");
_Static_assert(sizeof(bot_csw_t) == CSW_SIZE, "CSW size");

/* ------------------------------------------------------------------------ */
/* Driver state                                                             */
/* ------------------------------------------------------------------------ */

typedef struct {
    usb_host_client_handle_t client_hdl;
    atapi_host_event_cb_t user_cb;
    void *user_arg;
    SemaphoreHandle_t all_events_handled;
    volatile bool end_client_event_handling;
    bool event_handling_started;
    STAILQ_HEAD(devices, atapi_host_device) devices;
} atapi_driver_t;

static atapi_driver_t *s_driver;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------------ */
/* Descriptor parsing                                                       */
/* ------------------------------------------------------------------------ */

static bool is_supported_subclass(uint8_t subclass)
{
    return subclass == MSC_SUBCLASS_SFF8020I ||
           subclass == MSC_SUBCLASS_SFF8070I ||
           subclass == MSC_SUBCLASS_SCSI;
}

/**
 * Find a Bulk-Only mass storage interface (any alternate setting, so UAS-capable
 * bridges that expose BOT on alt 0 and UAS on alt 1 are handled).
 */
static const usb_intf_desc_t *find_bot_interface(const usb_config_desc_t *cfg, int *intf_offset)
{
    int offset = 0;
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *)cfg;

    while ((desc = usb_parse_next_descriptor_of_type(desc, cfg->wTotalLength,
                                                     USB_B_DESCRIPTOR_TYPE_INTERFACE, &offset)) != NULL) {
        const usb_intf_desc_t *intf = (const usb_intf_desc_t *)desc;
        if (intf->bInterfaceClass == USB_CLASS_MASS_STORAGE &&
                is_supported_subclass(intf->bInterfaceSubClass) &&
                intf->bInterfaceProtocol == MSC_PROTOCOL_BOT) {
            if (intf_offset) {
                *intf_offset = offset;
            }
            return intf;
        }
    }
    return NULL;
}

static esp_err_t parse_endpoints(atapi_device_t *dev, const usb_config_desc_t *cfg,
                                 const usb_intf_desc_t *intf, int intf_offset)
{
    dev->ep_in = 0;
    dev->ep_out = 0;
    for (int i = 0; i < intf->bNumEndpoints; i++) {
        int offset = intf_offset;
        const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, i, cfg->wTotalLength, &offset);
        if (!ep || USB_EP_DESC_GET_XFERTYPE(ep) != USB_TRANSFER_TYPE_BULK) {
            continue;
        }
        if (USB_EP_DESC_GET_EP_DIR(ep)) {
            dev->ep_in = ep->bEndpointAddress;
            dev->mps_in = USB_EP_DESC_GET_MPS(ep);
        } else {
            dev->ep_out = ep->bEndpointAddress;
        }
    }
    ESP_RETURN_ON_FALSE(dev->ep_in && dev->ep_out && dev->mps_in, ESP_ERR_NOT_SUPPORTED, TAG,
                        "Bulk endpoints not found");
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Low level transfers                                                      */
/* ------------------------------------------------------------------------ */

static void transfer_cb(usb_transfer_t *xfer)
{
    atapi_device_t *dev = (atapi_device_t *)xfer->context;
    xSemaphoreGive(dev->transfer_done);
}

#define CANCEL_WAIT_MS 2000

static usb_transfer_status_t submit_and_wait(atapi_device_t *dev, usb_transfer_t *xfer, bool is_ctrl)
{
    if (dev->xfer_lost) {
        return USB_TRANSFER_STATUS_ERROR; /* a transfer is still owned by the stack: fail fast */
    }
    xfer->device_handle = dev->dev_hdl;
    xfer->callback = transfer_cb;
    xfer->context = dev;

    esp_err_t err = is_ctrl ? usb_host_transfer_submit_control(s_driver->client_hdl, xfer)
                    : usb_host_transfer_submit(xfer);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Transfer submit failed: %s", esp_err_to_name(err));
        return USB_TRANSFER_STATUS_ERROR;
    }

    if (xSemaphoreTake(dev->transfer_done, pdMS_TO_TICKS(xfer->timeout_ms)) != pdTRUE) {
        /* Timed out: cancel the transfer by halting and flushing the endpoint */
        ESP_LOGW(TAG, "Transfer on EP 0x%02x timed out", xfer->bEndpointAddress);
        esp_err_t h = usb_host_endpoint_halt(dev->dev_hdl, xfer->bEndpointAddress);
        esp_err_t f = usb_host_endpoint_flush(dev->dev_hdl, xfer->bEndpointAddress);
        esp_err_t c = usb_host_endpoint_clear(dev->dev_hdl, xfer->bEndpointAddress);
        /* Flushed, so it normally completes immediately. If not (seen with a wedged bridge, likely on EP0),
         * waiting forever would block the caller silently: give up on the device instead. */
        if (xSemaphoreTake(dev->transfer_done, pdMS_TO_TICKS(CANCEL_WAIT_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "Transfer on EP 0x%02x could not be cancelled (halt %s, flush %s, clear %s); "
                     "device unusable until reconnected", xfer->bEndpointAddress, esp_err_to_name(h),
                     esp_err_to_name(f), esp_err_to_name(c));
            dev->xfer_lost = true;
            return USB_TRANSFER_STATUS_ERROR;
        }
        return USB_TRANSFER_STATUS_TIMED_OUT;
    }
    return xfer->status;
}

static esp_err_t ensure_bulk_buffer(atapi_device_t *dev, size_t size)
{
    if (dev->bulk_xfer && dev->bulk_xfer->data_buffer_size >= size) {
        return ESP_OK;
    }
    if (dev->bulk_xfer) {
        usb_host_transfer_free(dev->bulk_xfer);
        dev->bulk_xfer = NULL;
    }
    return usb_host_transfer_alloc(size, 0, &dev->bulk_xfer);
}

static esp_err_t status_to_err(usb_transfer_status_t status)
{
    switch (status) {
    case USB_TRANSFER_STATUS_COMPLETED: return ESP_OK;
    case USB_TRANSFER_STATUS_STALL:     return ESP_ERR_ATAPI_STALL;
    case USB_TRANSFER_STATUS_TIMED_OUT: return ESP_ERR_TIMEOUT;
    case USB_TRANSFER_STATUS_NO_DEVICE: return ESP_ERR_INVALID_STATE;
    default:                            return ESP_ERR_ATAPI_TRANSPORT;
    }
}

/** Bulk transfer. For IN, @p len is the expected size; @p actual receives the received size. */
static esp_err_t bulk_xfer(atapi_device_t *dev, bool in, void *data, size_t len,
                           uint32_t timeout_ms, size_t *actual)
{
    size_t xfer_len = in ? usb_round_up_to_mps(len, dev->mps_in) : len;
    ESP_RETURN_ON_ERROR(ensure_bulk_buffer(dev, MAX(xfer_len, 64)), TAG, "No memory for transfer");

    usb_transfer_t *xfer = dev->bulk_xfer;
    xfer->bEndpointAddress = in ? dev->ep_in : dev->ep_out;
    xfer->num_bytes = xfer_len;
    xfer->timeout_ms = timeout_ms;
    if (!in) {
        memcpy(xfer->data_buffer, data, len);
    }

    esp_err_t err = status_to_err(submit_and_wait(dev, xfer, false));
    if (err == ESP_OK && in) {
        size_t n = MIN((size_t)xfer->actual_num_bytes, len);
        memcpy(data, xfer->data_buffer, n);
        if (actual) {
            *actual = n;
        }
    } else if (err == ESP_OK && actual) {
        *actual = xfer->actual_num_bytes;
    }
    return err;
}

static esp_err_t control_xfer(atapi_device_t *dev, const usb_setup_packet_t *setup)
{
    usb_transfer_t *xfer = dev->ctrl_xfer;
    memcpy(xfer->data_buffer, setup, USB_SETUP_PACKET_SIZE);
    xfer->bEndpointAddress = 0;
    xfer->num_bytes = USB_SETUP_PACKET_SIZE + setup->wLength;
    xfer->timeout_ms = ATAPI_TIMEOUT_CBW_MS;
    return status_to_err(submit_and_wait(dev, xfer, true));
}

static esp_err_t clear_feature_halt(atapi_device_t *dev, uint8_t ep)
{
    usb_setup_packet_t setup = {
        .bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD |
        USB_BM_REQUEST_TYPE_RECIP_ENDPOINT,
        .bRequest = USB_B_REQUEST_CLEAR_FEATURE,
        .wValue = 0, /* ENDPOINT_HALT */
        .wIndex = ep,
        .wLength = 0,
    };
    return control_xfer(dev, &setup);
}

/**
 * Clear halt on both bulk endpoints and resynchronize data toggles.
 *
 * CLEAR_FEATURE(ENDPOINT_HALT) resets the device's data toggle to DATA0, but the
 * ESP USB host keeps the channel's toggle (it is only reset when a pipe is
 * allocated). A mismatch makes the host silently drop every IN packet, so all
 * following CSWs time out. Re-claiming the interface re-allocates the pipes with
 * DATA0. Since that resets both host pipes, both device endpoints are cleared too.
 */
static esp_err_t clear_halt(atapi_device_t *dev)
{
    const uint8_t eps[2] = {dev->ep_in, dev->ep_out};
    for (int i = 0; i < 2; i++) {
        usb_host_endpoint_halt(dev->dev_hdl, eps[i]);
        usb_host_endpoint_flush(dev->dev_hdl, eps[i]);
        usb_host_endpoint_clear(dev->dev_hdl, eps[i]);
    }
    esp_err_t err = clear_feature_halt(dev, dev->ep_in);
    esp_err_t err2 = clear_feature_halt(dev, dev->ep_out);

    esp_err_t claim_err = usb_host_interface_release(s_driver->client_hdl, dev->dev_hdl, dev->iface_num);
    if (claim_err == ESP_OK) {
        claim_err = usb_host_interface_claim(s_driver->client_hdl, dev->dev_hdl, dev->iface_num, dev->alt_setting);
    }
    if (claim_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-claim interface: %s", esp_err_to_name(claim_err));
        return claim_err;
    }
    return err != ESP_OK ? err : err2;
}

/** BOT 5.3.4 Reset Recovery */
static esp_err_t reset_recovery(atapi_device_t *dev)
{
    ESP_LOGW(TAG, "Performing BOT reset recovery");
    usb_setup_packet_t setup = {
        .bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS |
        USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        .bRequest = BOT_REQ_RESET,
        .wValue = 0,
        .wIndex = dev->iface_num,
        .wLength = 0,
    };
    esp_err_t err = control_xfer(dev, &setup);
    clear_halt(dev);
    return err;
}

/* ------------------------------------------------------------------------ */
/* Bulk-Only Transport command                                              */
/* ------------------------------------------------------------------------ */

esp_err_t atapi_bot_command(atapi_device_t *dev, const uint8_t *cdb, size_t cdb_len,
                            void *data, size_t data_len, bool data_in,
                            uint32_t timeout_ms, size_t *actual_len)
{
    ESP_RETURN_ON_FALSE(cdb_len >= 1 && cdb_len <= 16, ESP_ERR_INVALID_ARG, TAG, "bad CDB length");
    if (data == NULL) {
        data_len = 0;
    }
    if (actual_len) {
        *actual_len = 0;
    }

    /* ATAPI (SFF-8020i / SFF-8070i) bridges expect 12-byte packets */
    size_t cb_len = cdb_len;
    if ((dev->subclass == MSC_SUBCLASS_SFF8020I || dev->subclass == MSC_SUBCLASS_SFF8070I) && cb_len < 12) {
        cb_len = 12;
    }

    bot_cbw_t cbw = {
        .signature = CBW_SIGNATURE,
        .tag = ++dev->tag,
        .data_length = data_len,
        .flags = (data_len && data_in) ? CBW_FLAG_IN : 0,
        .lun = 0,
        .cb_length = cb_len,
    };
    memcpy(cbw.cb, cdb, cdb_len);

    /* 1. Command transport */
    esp_err_t err = bulk_xfer(dev, false, &cbw, CBW_SIZE, ATAPI_TIMEOUT_CBW_MS, NULL);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "CBW failed: %s", esp_err_to_name(err));
        if (err != ESP_ERR_INVALID_STATE) {
            reset_recovery(dev);
        }
        return err == ESP_ERR_INVALID_STATE ? err : ESP_ERR_ATAPI_TRANSPORT;
    }

    /* 2. Data transport. A STALL here is legal: clear it and go on to the status phase. */
    size_t transferred = 0;
    if (data_len) {
        err = bulk_xfer(dev, data_in, data, data_len, timeout_ms, &transferred);
        if (err == ESP_ERR_ATAPI_STALL) {
            ESP_LOGD(TAG, "Data phase stalled");
            clear_halt(dev);
        } else if (err != ESP_OK) {
            ESP_LOGD(TAG, "Data phase failed: %s", esp_err_to_name(err));
            if (err != ESP_ERR_INVALID_STATE) {
                reset_recovery(dev);
            }
            return err == ESP_ERR_INVALID_STATE ? err : ESP_ERR_ATAPI_TRANSPORT;
        }
    }

    /* 3. Status transport (retry once after clearing a stall) */
    bot_csw_t csw = {0};
    size_t csw_len = 0;
    err = bulk_xfer(dev, true, &csw, CSW_SIZE, timeout_ms, &csw_len);
    if (err == ESP_ERR_ATAPI_STALL) {
        clear_halt(dev);
        err = bulk_xfer(dev, true, &csw, CSW_SIZE, timeout_ms, &csw_len);
    }
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "CSW failed: %s", esp_err_to_name(err));
        if (err != ESP_ERR_INVALID_STATE) {
            reset_recovery(dev);
        }
        return err == ESP_ERR_INVALID_STATE ? err : ESP_ERR_ATAPI_TRANSPORT;
    }

    if (csw_len != CSW_SIZE || csw.signature != CSW_SIGNATURE || csw.tag != cbw.tag) {
        ESP_LOGW(TAG, "Invalid CSW (len %u sig 0x%08"PRIx32" tag %"PRIu32"/%"PRIu32")",
                 (unsigned)csw_len, csw.signature, csw.tag, cbw.tag);
        reset_recovery(dev);
        return ESP_ERR_ATAPI_TRANSPORT;
    }

    if (data_len && data_in && transferred != data_len - MIN(csw.residue, data_len)) {
        ESP_LOGW(TAG, "CMD 0x%02x: received %u bytes, CSW says %u (requested %u, residue %"PRIu32")", cdb[0],
                 (unsigned)transferred, (unsigned)(data_len - MIN(csw.residue, data_len)), (unsigned)data_len,
                 csw.residue);
    }
    if (actual_len) {
        /* Prefer the device-reported residue; fall back to what the bus delivered */
        size_t by_residue = csw.residue <= data_len ? data_len - csw.residue : 0;
        *actual_len = MIN(by_residue, transferred);
    }

    switch (csw.status) {
    case CSW_STATUS_PASSED:
        return ESP_OK;
    case CSW_STATUS_FAILED:
        return ESP_ERR_ATAPI_CHECK_CONDITION;
    default:
        ESP_LOGW(TAG, "CSW phase error");
        reset_recovery(dev);
        return ESP_ERR_ATAPI_TRANSPORT;
    }
}

/* ------------------------------------------------------------------------ */
/* USB Host client                                                          */
/* ------------------------------------------------------------------------ */

static bool device_has_bot_interface(uint8_t address)
{
    bool found = false;
    usb_device_handle_t dev_hdl;
    const usb_config_desc_t *cfg;
    const usb_device_desc_t *dev_desc;

    if (usb_host_device_open(s_driver->client_hdl, address, &dev_hdl) == ESP_OK) {
        if (usb_host_get_device_descriptor(dev_hdl, &dev_desc) == ESP_OK &&
                usb_host_get_active_config_descriptor(dev_hdl, &cfg) == ESP_OK) {
            found = find_bot_interface(cfg, NULL) != NULL;
            ESP_LOGI(TAG, "USB device %d: VID 0x%04x PID 0x%04x%s", address, dev_desc->idVendor,
                     dev_desc->idProduct, found ? "" : " - no Bulk-Only mass storage interface");
            if (!found) {
                int offset = 0;
                const usb_standard_desc_t *d = (const usb_standard_desc_t *)cfg;
                while ((d = usb_parse_next_descriptor_of_type(d, cfg->wTotalLength,
                                                              USB_B_DESCRIPTOR_TYPE_INTERFACE, &offset)) != NULL) {
                    const usb_intf_desc_t *i = (const usb_intf_desc_t *)d;
                    ESP_LOGI(TAG, "  interface %d alt %d: class 0x%02x subclass 0x%02x protocol 0x%02x",
                             i->bInterfaceNumber, i->bAlternateSetting, i->bInterfaceClass,
                             i->bInterfaceSubClass, i->bInterfaceProtocol);
                }
            }
        }
        usb_host_device_close(s_driver->client_hdl, dev_hdl);
    }
    return found;
}

static atapi_device_t *find_device(usb_device_handle_t dev_hdl)
{
    atapi_device_t *it, *found = NULL;
    portENTER_CRITICAL(&s_lock);
    STAILQ_FOREACH(it, &s_driver->devices, tailq_entry) {
        if (it->dev_hdl == dev_hdl) {
            found = it;
            break;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    return found;
}

static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        if (device_has_bot_interface(msg->new_dev.address)) {
            atapi_host_event_t ev = {
                .event = ATAPI_DEVICE_CONNECTED,
                .device.address = msg->new_dev.address,
            };
            s_driver->user_cb(&ev, s_driver->user_arg);
        } else {
            ESP_LOGD(TAG, "Device %d has no Bulk-Only mass storage interface", msg->new_dev.address);
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
        atapi_device_t *dev = find_device(msg->dev_gone.dev_hdl);
        if (dev) {
            atapi_host_event_t ev = {
                .event = ATAPI_DEVICE_DISCONNECTED,
                .device.handle = dev,
            };
            s_driver->user_cb(&ev, s_driver->user_arg);
        }
        break;
    }
    default:
        break;
    }
}

esp_err_t atapi_host_handle_events(TickType_t timeout)
{
    ESP_RETURN_ON_FALSE(s_driver, ESP_ERR_INVALID_STATE, TAG, "not installed");
    s_driver->event_handling_started = true;
    esp_err_t ret = usb_host_client_handle_events(s_driver->client_hdl, timeout);
    if (s_driver->end_client_event_handling) {
        xSemaphoreGive(s_driver->all_events_handled);
        return ESP_FAIL;
    }
    return ret;
}

static void event_task(void *arg)
{
    while (atapi_host_handle_events(portMAX_DELAY) == ESP_OK) {
    }
    vTaskDelete(NULL);
}

/* Before esp_restart(): let a running command finish and block new ones, so the ESP doesn't reset in the middle
 * of a BOT command. Some bridges (JMicron 152d:2338 with a TSSTcorp drive) stay wedged after that. */
static void shutdown_handler(void)
{
    if (!s_driver) {
        return;
    }
    atapi_device_t *dev;
    STAILQ_FOREACH(dev, &s_driver->devices, tailq_entry) {
        if (dev->lock && xSemaphoreTake(dev->lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
            ESP_LOGW(TAG, "Restart while a command is running on device %d", dev->usb_address);
        }
    }
}

esp_err_t atapi_host_install(const atapi_host_driver_config_t *config)
{
    ESP_RETURN_ON_FALSE(config && config->callback, ESP_ERR_INVALID_ARG, TAG, "callback required");
    ESP_RETURN_ON_FALSE(!config->create_background_task || (config->stack_size && config->task_priority),
                        ESP_ERR_INVALID_ARG, TAG, "task config");
    ESP_RETURN_ON_FALSE(!s_driver, ESP_ERR_INVALID_STATE, TAG, "already installed");

    esp_err_t ret = ESP_OK;
    atapi_driver_t *drv = calloc(1, sizeof(atapi_driver_t));
    ESP_RETURN_ON_FALSE(drv, ESP_ERR_NO_MEM, TAG, "");
    drv->user_cb = config->callback;
    drv->user_arg = config->callback_arg;
    STAILQ_INIT(&drv->devices);
    drv->all_events_handled = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(drv->all_events_handled, ESP_ERR_NO_MEM, fail, TAG, "");

    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 10,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = NULL,
        },
    };
    ESP_GOTO_ON_ERROR(usb_host_client_register(&client_config, &drv->client_hdl), fail, TAG, "client register");
    s_driver = drv;
    esp_register_shutdown_handler(shutdown_handler); /* ESP_ERR_INVALID_STATE if already registered: fine */

    if (config->create_background_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(event_task, "atapi_usb", config->stack_size, NULL,
                                                config->task_priority, NULL, config->core_id);
        ESP_GOTO_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, fail, TAG, "task create");
    }
    return ESP_OK;

fail:
    s_driver = NULL;
    if (drv->client_hdl) {
        usb_host_client_deregister(drv->client_hdl);
    }
    if (drv->all_events_handled) {
        vSemaphoreDelete(drv->all_events_handled);
    }
    free(drv);
    return ret;
}

esp_err_t atapi_host_uninstall(void)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_driver || s_driver->end_client_event_handling || !STAILQ_EMPTY(&s_driver->devices)) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_driver->end_client_event_handling = true;
    portEXIT_CRITICAL(&s_lock);

    if (s_driver->event_handling_started) {
        usb_host_client_unblock(s_driver->client_hdl);
        xSemaphoreTake(s_driver->all_events_handled, portMAX_DELAY);
    }
    vSemaphoreDelete(s_driver->all_events_handled);
    ESP_RETURN_ON_ERROR(usb_host_client_deregister(s_driver->client_hdl), TAG, "deregister");
    free(s_driver);
    s_driver = NULL;
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Device                                                                   */
/* ------------------------------------------------------------------------ */

static void copy_trimmed(char *dst, const uint8_t *src, size_t len)
{
    memcpy(dst, src, len);
    dst[len] = '\0';
    for (int i = len - 1; i >= 0 && (dst[i] == ' ' || dst[i] == '\0'); i--) {
        dst[i] = '\0';
    }
}

static esp_err_t do_inquiry(atapi_device_t *dev)
{
    uint8_t cdb[6] = {0x12, 0, 0, 0, 36, 0};
    uint8_t resp[36] = {0};
    size_t len = 0;
    esp_err_t err = ESP_FAIL;

    /* The bridge may not answer while the drive is still busy (e.g. initializing a disc after
     * the ESP was reset with the drive powered). Use short timeouts and keep trying for ~60 s. */
    for (int attempt = 0; attempt < 20; attempt++) {
        err = atapi_bot_command(dev, cdb, sizeof(cdb), resp, sizeof(resp), true, 3000, &len);
        if (err == ESP_OK && len >= 32) {
            break;
        }
        if (err == ESP_ERR_INVALID_STATE || dev->xfer_lost) {
            break; /* device gone, or a transfer couldn't be cancelled (wedged bridge) */
        }
        ESP_LOGI(TAG, "Drive not responding yet (%s), retrying...", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_RETURN_ON_ERROR(err, TAG, "INQUIRY failed");
    ESP_RETURN_ON_FALSE(len >= 32, ESP_ERR_INVALID_RESPONSE, TAG, "INQUIRY too short (%u)", (unsigned)len);

    dev->info.peripheral_type = resp[0] & 0x1F;
    dev->info.removable = (resp[1] & 0x80) != 0;
    copy_trimmed(dev->info.vendor, &resp[8], 8);
    copy_trimmed(dev->info.product, &resp[16], 16);
    copy_trimmed(dev->info.revision, &resp[32], MIN(4, len - 32));
    return ESP_OK;
}

static void free_device(atapi_device_t *dev, bool claimed)
{
    portENTER_CRITICAL(&s_lock);
    STAILQ_REMOVE(&s_driver->devices, dev, atapi_host_device, tailq_entry);
    portEXIT_CRITICAL(&s_lock);

    if (claimed) {
        usb_host_interface_release(s_driver->client_hdl, dev->dev_hdl, dev->iface_num);
    }
    if (dev->xfer_lost && xSemaphoreTake(dev->transfer_done, pdMS_TO_TICKS(CANCEL_WAIT_MS)) == pdTRUE) {
        dev->xfer_lost = false; /* completed after all (typically with NO_DEVICE once unplugged) */
    }
    if (dev->dev_hdl) {
        usb_host_device_close(s_driver->client_hdl, dev->dev_hdl);
    }
    if (dev->xfer_lost) {
        /* The stack still owns a transfer and will call transfer_cb(dev) when the device goes away: keep the
         * device struct, its transfers and semaphores alive (a few hundred bytes, leaked until reboot). */
        ESP_LOGW(TAG, "Device %d freed with a transfer still pending; leaking its context", dev->usb_address);
        return;
    }
    if (dev->bulk_xfer) {
        usb_host_transfer_free(dev->bulk_xfer);
    }
    if (dev->ctrl_xfer) {
        usb_host_transfer_free(dev->ctrl_xfer);
    }
    if (dev->transfer_done) {
        vSemaphoreDelete(dev->transfer_done);
    }
    if (dev->lock) {
        vSemaphoreDelete(dev->lock);
    }
    free(dev);
}

esp_err_t atapi_host_install_device(uint8_t usb_address, atapi_host_device_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    ESP_RETURN_ON_FALSE(s_driver, ESP_ERR_INVALID_STATE, TAG, "not installed");

    esp_err_t ret = ESP_OK;
    bool claimed = false;
    const usb_config_desc_t *cfg;
    const usb_device_desc_t *dev_desc;

    atapi_device_t *dev = calloc(1, sizeof(atapi_device_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "");
    portENTER_CRITICAL(&s_lock);
    STAILQ_INSERT_TAIL(&s_driver->devices, dev, tailq_entry);
    portEXIT_CRITICAL(&s_lock);

    dev->usb_address = usb_address;
    dev->lock = xSemaphoreCreateMutex();
    dev->transfer_done = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(dev->lock && dev->transfer_done, ESP_ERR_NO_MEM, fail, TAG, "");

    ESP_GOTO_ON_ERROR(usb_host_device_open(s_driver->client_hdl, usb_address, &dev->dev_hdl), fail, TAG, "open");
    ESP_GOTO_ON_ERROR(usb_host_get_device_descriptor(dev->dev_hdl, &dev_desc), fail, TAG, "dev desc");
    ESP_GOTO_ON_ERROR(usb_host_get_active_config_descriptor(dev->dev_hdl, &cfg), fail, TAG, "cfg desc");

    int intf_offset = 0;
    const usb_intf_desc_t *intf = find_bot_interface(cfg, &intf_offset);
    ESP_GOTO_ON_FALSE(intf, ESP_ERR_NOT_SUPPORTED, fail, TAG, "no BOT interface");
    dev->iface_num = intf->bInterfaceNumber;
    dev->subclass = intf->bInterfaceSubClass;
    ESP_GOTO_ON_ERROR(parse_endpoints(dev, cfg, intf, intf_offset), fail, TAG, "endpoints");

    ESP_GOTO_ON_ERROR(usb_host_transfer_alloc(CTRL_XFER_SIZE, 0, &dev->ctrl_xfer), fail, TAG, "ctrl xfer");
    ESP_GOTO_ON_ERROR(ensure_bulk_buffer(dev, 512), fail, TAG, "bulk xfer");
    ESP_GOTO_ON_ERROR(usb_host_interface_claim(s_driver->client_hdl, dev->dev_hdl, dev->iface_num,
                                               intf->bAlternateSetting), fail, TAG, "claim");
    dev->alt_setting = intf->bAlternateSetting;
    claimed = true;

    dev->info.vid = dev_desc->idVendor;
    dev->info.pid = dev_desc->idProduct;
    dev->info.usb_address = usb_address;
    dev->info.interface_subclass = dev->subclass;

    ESP_LOGI(TAG, "Bulk-Only interface %d (subclass 0x%02x), EP IN 0x%02x (MPS %d), EP OUT 0x%02x",
             dev->iface_num, dev->subclass, dev->ep_in, dev->mps_in, dev->ep_out);

    ESP_GOTO_ON_ERROR(do_inquiry(dev), fail, TAG, "inquiry");
    ESP_LOGI(TAG, "INQUIRY: '%s' '%s' rev '%s', type 0x%02x%s", dev->info.vendor, dev->info.product,
             dev->info.revision, dev->info.peripheral_type, dev->info.removable ? ", removable" : "");
    ESP_GOTO_ON_FALSE(dev->info.peripheral_type == ATAPI_PERIPHERAL_TYPE_CDROM, ESP_ERR_NOT_SUPPORTED, fail, TAG,
                      "Not an optical drive (peripheral type 0x%02x)", dev->info.peripheral_type);

    *handle = dev;
    return ESP_OK;

fail:
    free_device(dev, claimed);
    return ret;
}

esp_err_t atapi_host_uninstall_device(atapi_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle && s_driver, ESP_ERR_INVALID_ARG, TAG, "");
    atapi_device_t *dev = handle;
    /* Wait for any running command to complete */
    xSemaphoreTake(dev->lock, portMAX_DELAY);
    xSemaphoreGive(dev->lock);
    free_device(dev, true);
    return ESP_OK;
}

esp_err_t atapi_host_get_device_info(atapi_host_device_handle_t handle, atapi_host_device_info_t *info)
{
    ESP_RETURN_ON_FALSE(handle && info, ESP_ERR_INVALID_ARG, TAG, "");
    *info = handle->info;
    return ESP_OK;
}

esp_err_t atapi_host_print_descriptors(atapi_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    const usb_device_desc_t *dev_desc;
    const usb_config_desc_t *cfg;
    ESP_RETURN_ON_ERROR(usb_host_get_device_descriptor(handle->dev_hdl, &dev_desc), TAG, "");
    ESP_RETURN_ON_ERROR(usb_host_get_active_config_descriptor(handle->dev_hdl, &cfg), TAG, "");
    usb_print_device_descriptor(dev_desc);
    usb_print_config_descriptor(cfg, NULL);
    return ESP_OK;
}
