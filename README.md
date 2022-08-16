# ESP32-oled-spectrum

![Picture](/images/closeup.jpeg)

⚠️ **Disclaimer!** ⚠️ This project depends on fixes introduced in the latest (as of writing) ESP32 v2.0.4 core. Upgrade the core through the Arduino boards manager.

People generally cannot perceive frequencies beyond 20Hz and 20kHz, and the frequencies within that range are heard logarithmically. [Many recorded pieces of music have a dynamic range that doesn't exceed 23 dB](https://hub.yamaha.com/audio/music/what-is-dynamic-range-and-why-does-it-matter) (coincidentally the range of a VU meter), and we hear that variation in amplitude logarithmically too. [Rhythms beyond 600 BPM (10 Hz) approach the limits of human perception](https://www.youtube.com/watch?v=h3kqBX1j7f8).

ESP32-oled-spectrum is a project that tries to visually represent all music within these constraints--faithfully--as a high-performance, high-resolution audio spectrum visualizer. It leverages what the ESP32 microcontroller and the SSD1306 OLED module uniquely offer. Namely, it's 32 bars of logarithmically-spaced frequencies moving at ~107 fps, the typical maximum refresh rate of the SSD1306. To accomplish this, it involves the following:

* the I2S and I2C/SPI peripherals, not to mention both cores of the ESP32
* a Constant Q transform ([cq_kernel](https://github.com/colonelwatch/cq_kernel)) (used to emulate a bank of bandpass filters with Q of ~4.5 and equal group delay) computed from a 6144-point FFT ([kissfft](https://github.com/mborgerding/kissfft))
* post-processing and filters, including a 2x interpolation routine

All that said, it's an Arduino sketch, so please give it an upload and see for yourself! The promised performance is on an SPI SSD1306, but I also have a routine for the ubiquitous I2C one that runs at 89 fps. Below is an okay amplification circuit into pin 36/VP to get started with.

![Amplification circuit](/images/amplification.png)

It presents an impedance of at least 5k, which should be okay for line-level and definitely good with a phone. Alternatively, you can plug in any signal that is 3V peak-to-peak and DC-biased at 1.65V. You may also add an LED to pin 15 to get a clipping indicator.