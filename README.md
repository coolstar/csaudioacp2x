# CoolStar Audio for AMD ACP 2.2

Open Source AMD ACP 2.2 driver

Currently Implemented:

* 16-bit 48 Khz Audio Streams
* BT and I2S TDM streams (Speaker/DMIC is on BT, 3.5mm jack on I2S)
* Runtime Power Management
* Play / Pause / Stop support for streams
* WDM Position Counter

Tested on HP Chromebook 14a (AMD A4-9120C)

Based off csaudioacp3x driver and Linux kernel 6.4 (soc/dwc and soc/amd)