# esp_usb_host_atapi

ESP-IDF USB Host class driver for **optical drives** (CD/DVD/BD) behind USB mass storage bridges.
It sends SCSI MMC commands (the packet commands ATAPI carries) over **Bulk-Only Transport**. This
gives access to things the generic MSC driver can't do: raw CD-DA audio sectors, TOC, CD-TEXT,
tray control.

It requires ESP-IDF >= 6.0 and the `espressif/usb` component. It has been tested on ESP32-S3 with several drives
behind USB-IDE/SATA bridges, in a CD player that streams CD audio to I2S. It should work on the ESP32-S2
(same full-speed USB OTG peripheral; builds, but untested).

## Installation

```
idf.py add-dependency "kilrah/esp_usb_host_atapi^0.1.0"
```

The [cd_info example](examples/cd_info) prints the drive's identification, the TOC and the CD-TEXT of the
inserted disc. To create a project from it: `idf.py create-project-from-example "kilrah/esp_usb_host_atapi^0.1.0:cd_info"`.

**Power:** optical drives draw more current than a dev board can supply on VBUS, especially at spin-up. Power the
drive and its bridge from their own supply. A drive that enumerates and then drops off the bus is usually a power
problem. Avoid power banks: many short the USB data lines to signal fast charging.

## Files

| File | Content |
|---|---|
| [include/usb/atapi_host.h](include/usb/atapi_host.h) | Public API and types |
| [private_include/atapi_private.h](private_include/atapi_private.h) | Device struct, timeouts, max transfer size |
| [src/atapi_host.c](src/atapi_host.c) | USB Host client, device install, descriptor parsing, Bulk-Only Transport, error recovery |
| [src/atapi_mmc.c](src/atapi_mmc.c) | MMC commands with auto REQUEST SENSE: TUR, wait ready, GESN, READ TOC, READ CD, READ(10), SET CD SPEED, SET STREAMING, MODE SENSE page 2Ah (speed), START STOP UNIT, PREVENT ALLOW |
| [src/atapi_cdtext.c](src/atapi_cdtext.c) | CD-TEXT (READ TOC format 0101b) pack parser |

## Usage

```c
// After usb_host_install():
const atapi_host_driver_config_t cfg = {
    .create_background_task = true, .task_priority = 5, .stack_size = 4096,
    .core_id = tskNO_AFFINITY, .callback = my_event_cb,
};
atapi_host_install(&cfg);

// In my_event_cb: ATAPI_DEVICE_CONNECTED gives event->device.address -> forward to a task:
atapi_host_device_handle_t dev;
atapi_host_install_device(address, &dev);         // claims interface, INQUIRY, checks type 0x05

if (atapi_host_wait_ready(dev, 30000) == ESP_OK) {
    static atapi_toc_t toc;
    atapi_host_read_toc(dev, &toc);

    static atapi_cd_text_t text;                   // ~3.3 KB
    if (atapi_host_read_cd_text(dev, &text) == ESP_OK) {
        printf("%s\n", text.text[ATAPI_CDTEXT_TITLE][0] ?: "");   // [0] = album, [n] = track n
        atapi_cd_text_free(&text);
    }

    uint8_t *buf = malloc(16 * ATAPI_CD_AUDIO_SECTOR_SIZE);
    atapi_host_read_cd_audio(dev, toc.tracks[0].lba, 16, buf); // 16-bit LE stereo, 44.1 kHz
}

// On ATAPI_DEVICE_DISCONNECTED: atapi_host_uninstall_device(event->device.handle)
```

Every command function is thread-safe: calls are serialized per device with a mutex.

### Errors

| Code | Meaning |
|---|---|
| `ESP_ERR_ATAPI_CHECK_CONDITION` | The drive rejected the command. The sense data was fetched automatically: `atapi_host_get_last_sense()`, `atapi_sense_to_str()` |
| `ESP_ERR_ATAPI_NO_MEDIUM` | Sense 02/3A. ASCQ 01 = tray closed, 02 = tray open |
| `ESP_ERR_ATAPI_NOT_READY` | `atapi_host_wait_ready()` timed out (disc still spinning up) |
| `ESP_ERR_ATAPI_TRANSPORT` | USB / BOT failure. Recovery has already been attempted |
| `ESP_ERR_INVALID_STATE` | Device gone |
| `ESP_ERR_NOT_FOUND` | (`read_cd_text`) The disc has no CD-TEXT |

## Design notes

- **Interface matching:** mass storage class, subclass 0x02 (SFF-8020i/ATAPI), 0x05 (SFF-8070i) or
  0x06 (SCSI transparent), protocol 0x50 (BOT), any alternate setting. UAS-capable bridges expose BOT on alt 0.
  The peripheral type must be 0x05 (CD/DVD), otherwise `install_device` returns `ESP_ERR_NOT_SUPPORTED`.
- **12-byte CDB padding** for subclass 0x02/0x05 bridges. Linux does the same (`USB_SC_8020`).
- **Transfer size:** each BOT data phase is limited to `ATAPI_MAX_XFER_SIZE` = 16 × 2352 = 37632 bytes
  (a multiple of 64). Larger reads are split internally. Data is copied out of the USB DMA buffer,
  so caller buffers can live in PSRAM.
- **BOT error handling:** a STALL in the data phase is cleared, then the CSW is read. A STALL on the CSW
  triggers one retry. An invalid CSW, a phase error or a timeout triggers Reset Recovery (class request 0xFF + clear
  both endpoints). On CHECK CONDITION the driver issues REQUEST SENSE automatically.
- **Exact allocation lengths for variable-size replies:** READ TOC first reads 12 bytes (header + first
  descriptor), then exactly the length the header reports, like Linux `sr`. CD-TEXT does the same. A JMicron
  USB-IDE bridge (VID 0x152d PID 0x2338, JLMS XJ-HD165H drive) did not end the data phase with a short packet
  when the allocation length was larger than the reply: the bulk IN timed out after 30 s on every TOC read.
  Keep this in mind for new commands with variable-length replies.
- **TOC validation:** the entry count must match the header's first/last track, track numbers must be consecutive,
  and addresses must increase up to the lead-out. Otherwise the TOC is re-read (3 attempts) and dumped at warning
  level. Once, the JMicron bridge returned a TOC with CSW bytes (`USBS`) in the middle of the data.
  If the bus delivers a different byte count than the CSW residue implies, `atapi_host` logs a warning.
- **Timeouts:** CBW 5 s, normal commands 10 s, TOC/reads/eject 30 s. INQUIRY at install retries every
  3 s for up to ~60 s, because the bridge may not answer while the drive is busy.

### Data toggle workaround (important)

`CLEAR_FEATURE(ENDPOINT_HALT)` resets the **device's** data toggle to DATA0. The ESP USB Host
library (`espressif/usb` 1.6.0, `hcd_dwc.c`) does **not** reset the host channel's toggle on
`usb_host_endpoint_clear()`. The DWC channel PID is only set to DATA0 when a pipe is allocated.
After a clear, host and device were out of sync about half the time. The host then silently dropped
every IN packet as a duplicate, so every following CSW timed out and Reset Recovery could never fix it.

The fix is in `clear_halt()` in `atapi_host.c`. It always clears **both** bulk endpoints on the device,
then **releases and re-claims the interface**, which re-allocates the host pipes at DATA0.
To test it, issue several READ CD commands beyond the lead-out in a row (each one stalls the data phase).
Normal reads afterwards must still succeed.

The stock `usb_host_msc` driver has the same latent bug. It may be worth reporting upstream (esp-usb).

## Tested hardware

| Bridge | Drive | Notes |
|---|---|---|
| JMicron 152d:2338, USB-IDE | JLMS XJ-HD165H (DVD-ROM) | Works well; honours SET CD SPEED. Needed the exact allocation lengths (above) |
| JMicron 152d:2338, USB-IDE | TSSTcorp SH-S182M (DVD writer) | Works, but reads audio at full speed whatever the speed commands say, and wedges after a USB re-enumeration without power cycle (host reset); see below |
| (built in) | HP USB laptop drive | Works; can't close its own tray |

If a timed-out transfer can't be cancelled (the ESP USB Host library can't halt or flush EP0, so a hung control
transfer never completes), the device is marked unusable: further commands fail at once with
`ESP_ERR_ATAPI_TRANSPORT` and `install_device` returns an error instead of blocking. Only a power cycle of the
drive (or of VBUS, if the board can switch it) helps then.

## Limitations / TODO

- Single LUN (LUN 0) only.
- CD-TEXT: only block 0 (first language) is parsed. Charset ISO 8859-1 is converted to UTF-8, with bytes 0x80-0x9F
  decoded as Windows-1252 (burning software such as AnyBurn writes ’ “ ” – — … € ™ there; they are C1 control
  characters in ISO 8859-1). MS-JIS / double-byte text is passed through raw. CRC is not checked.
- READ CD does not request C2 error pointers or sub-channel data.
- No support for suspend/resume events.
- Full-speed only so far. On high-speed hosts (ESP32-P4) `ATAPI_MAX_XFER_SIZE` would have to become a multiple
  of 512.

## License

Apache-2.0, see [LICENSE](LICENSE).

This driver was written with AI assistance (Claude by Anthropic), directed, reviewed and tested on hardware by the
author.
