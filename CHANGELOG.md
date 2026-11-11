# Changelog

## 0.1.3

- ESP32-S31 added to the supported targets (preview target in ESP-IDF 6.1). Tested on an
  ESP32-S31-Function-CoreBoard-1, USB 2.0 OTG high-speed port, with an HP BD ROM BC-5501H behind an Initio bridge:
  TOC, a 41-track CD-TEXT and continuous CD-DA playback work unchanged.

## 0.1.2

- ESP32-P4 added to the supported targets. Tested on its high-speed USB-OTG port (512-byte bulk packets) with an
  HP BD ROM BC-5501H behind an Initio bridge: TOC, CD-TEXT and continuous CD-DA playback work unchanged, since
  transfer buffers and IN transfers were already rounded up to the endpoint's max packet size.
- README: the "full-speed only" limitation is removed and the transfer size note explains the rounding.

## 0.1.1

- Fix: audio reads failed with "No memory for transfer" after reading a large CD-TEXT (e.g. a 41-track
  compilation), although enough internal RAM was free. The bulk transfer buffer grew on demand, and the heap
  fragmented around it. It is now allocated once at full size when the device is installed.

## 0.1.0

First release.

- Bulk-Only Transport with Reset Recovery and automatic REQUEST SENSE; works around a data toggle desync
  after CLEAR_FEATURE(ENDPOINT_HALT) in the ESP USB Host library.
- MMC commands: INQUIRY, TEST UNIT READY / wait ready, GET EVENT STATUS NOTIFICATION, READ TOC (exact
  allocation lengths, validated), CD-TEXT, READ CD (CD-DA), READ(10), SET CD SPEED, SET STREAMING,
  MODE SENSE page 2Ah, START STOP UNIT, PREVENT ALLOW MEDIUM REMOVAL, raw CDB pass-through.
- A transfer that can't be cancelled (wedged bridge) no longer blocks forever: the device is marked unusable.
- Shutdown handler: lets a running command finish before `esp_restart()`.
