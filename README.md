# CHDK Rewired Fork With New Features and Fixes

A fork of [CHDK_Rewired](https://github.com/lr-m/CHDK_Rewired) that fixes an issue with the a540 causing it to crash after 1-2 flash photos are taken, and adds new glitch effects and quality-of-life features.

## What's new

### Flash crash fix
- Holding the shutter used to make CHDK repaint the whole screen every 10ms, which starved the camera of the time it needed to charge the flash and take the shot. The screen is now cleared once when the shutter is pressed.
- Bent photos are no longer tagged while the camera is still writing the JPEG, which avoids two writers on the same file.
- `CHDK/SCRIPTS/flashdbg.lua` logs flash shots to the card, for tracking down any remaining problems.

### New experimental effects
Find these in the bend panel's **X** tab or **Circuit Bending → Experimental**.
- **Melt:** bright highlights smear into streaky, glitchy trails. The **Colour** setting picks how the streaks are tinted:
  - **Classic:** fixed colours.
  - **Image:** the complementary colour of the photo's main colour.
  - **Local:** the complementary colour of each part of the photo.
- **Rain:** Melt that only affects sky and clouds, leaving the subject untouched. You can also set Melt's **Protect** setting to **Sky only** for the same result.
- **Aerochrome:** imitates Kodak Aerochrome infrared film, where foliage turns red and pink. The **Colour** setting picks the filter: Yellow 12, Orange 21 or Red 25.

### Live preview
Shows the bend matrix and experimental effects on the live view before you shoot. Turn it on with **Live preview** in the bend panel's X tab or in the Circuit Bending menu. The preview is low resolution and limited to the screen's colours, so treat it as a guide; the photo itself is full quality.

### Remembered image size
The FUNC menu's **SIZE** and **QUALITY** settings are now remembered across restarts and mode changes.

## Supported cameras

| Camera | Firmware | Bend matrix | Persistent OSD | Record UI | Custom sounds | Boot screen |
|---|---|:---:|:---:|:---:|:---:|:---:|
| PowerShot A540 | 100b | ✅ | ✅ | ✅ | ✅ | ❌ |

## Installing

1. Install the normal CHDK_Rewired firmware onto your a540 by [following their tutorial](https://github.com/lr-m/CHDK_Rewired/#installing). 
2. Download this repo, unzip it, then click and drag it onto your a540's SD card. Make sure to copy and replace over the existing files on the card.
3. Use your Canon a540 to take circuit-bent photos with flash without crashes!

## Firmware

**No Canon firmware is distributed here.** ROM dumps and `.FIR` update files are Canon's copyrighted code and are not in this repository.

## Licence

GPL, as upstream; see [`COPYING`](COPYING). Upstream CHDK is the work of the CHDK community; this fork adds to it and takes none of the credit for it.
