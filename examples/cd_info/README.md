# cd_info example

Waits for an optical drive on the USB Host port and prints its identification, the TOC and the CD-TEXT
of the inserted disc, then the first bytes of audio. Insert another disc or reconnect the drive to run again.

Connect the drive (usually an IDE/SATA drive with a USB bridge) to the USB OTG port. Optical drives need
more current than a dev board can supply on VBUS: power the drive and bridge from their own supply.

```
idf.py set-target esp32s3
idf.py -p PORT flash monitor
```

The log shows the drive's INQUIRY strings and the bridge's USB IDs, the track list with start LBA, length
and CD-TEXT titles, and the first 16 bytes of audio of the first audio track.
