# UF2 bootloader hex files

Drop an Adafruit nRF52 UF2 bootloader hex into this directory to have the
build produce a `zephyr_full.hex` containing the bootloader, SoftDevice (if
`WITH_SOFTDEVICE`), and application in a single image. That hex is what you
flash to a fresh chip via SWD/J-Link/`nrfjprog`. The plain `zephyr.hex` is
application-only and assumes the bootloader is already programmed.

## File naming

The build looks for, in order:

1. The path passed via `-DUF2_BOOTLOADER_HEX=<path>` to cmake.
2. The `UF2_BOOTLOADER_HEX` environment variable.
3. `bootloader/<board>.hex` — e.g. `bootloader/mochi_uf2.hex`,
   `bootloader/promicro_uf2.hex`.
4. `bootloader/<soc>.hex` — e.g. `bootloader/nrf52833.hex`,
   `bootloader/nrf52840.hex`.

Per-board files take precedence so different boards on the same SoC can pin
different bootloader builds.

## Where to get one

Adafruit publishes bootloader hex builds at
<https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases>. Pick the
`*_bootloader-*_nosd.hex` matching your board, or build your own from that
repo. The hex must contain:

- the MBR at `0x00000`,
- the bootloader code at the SoC's bootloader region
  (`0x74000`–`0x80000` on nRF52833, `0xF4000`–`0x100000` on nRF52840), and
- `UICR.NRFFW[0]` pointing at the bootloader start.

A bootloader hex without those UICR bits will boot the application but USB
DFU (the drag-and-drop UF2 path) will not work.

## .gitignore

`bootloader/*.hex` is gitignored — the bootloader is a separate project with
its own license, so we don't vendor it here.
