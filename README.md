# CoolStar Audio for AMD ACP 3.0

Open Source alternative for the AMD ACP 3.0 driver (may also work on newer AMD chipsets using ACP)

Currently Implemented:

* 16-bit 48 Khz Audio Streams
* BT and I2S TDM streams (Speaker/DMIC is on BT, 3.5mm jack on I2S)
* Runtime Power Management
* Play / Pause / Stop support for streams
* WDM Position Counter

Not Yet Implemented:
* Notifications using IRQ with watermark register

Tested on HP Chromebook 14b (Ryzen 3 3250C)

Based off SimpleAudioSample (commit 9e1a643093cac60cd333b6d69abc1e4118a12d63) and TabletAudioSample from Microsoft's WDK 10 samples

Based off Linux commit 6f6f28bf5d8e070c1e4a10d62d2a1af264683042 (relevant to sound/soc/amd/raven folder)
