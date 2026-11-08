# Changelog

## 0.1.0

First release.

- Bulk-Only Transport with Reset Recovery and automatic REQUEST SENSE; works around a data toggle desync
  after CLEAR_FEATURE(ENDPOINT_HALT) in the ESP USB Host library.
- MMC commands: INQUIRY, TEST UNIT READY / wait ready, GET EVENT STATUS NOTIFICATION, READ TOC (exact
  allocation lengths, validated), CD-TEXT, READ CD (CD-DA), READ(10), SET CD SPEED, SET STREAMING,
  MODE SENSE page 2Ah, START STOP UNIT, PREVENT ALLOW MEDIUM REMOVAL, raw CDB pass-through.
- A transfer that can't be cancelled (wedged bridge) no longer blocks forever: the device is marked unusable.
- Shutdown handler: lets a running command finish before `esp_restart()`.
