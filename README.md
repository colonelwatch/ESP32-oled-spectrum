# ESP32-oled-spectrum

![Picture](/images/closeup.jpeg)

⚠️ **Disclaimer!** ⚠️ This project depends on fixes introduced in the latest (as of writing) ESP32 v2.0.4 core. Upgrade the core through the Arduino boards manager.

ESP32-oled-spectrum is a high-performance, high-resolution audio spectrum visualizer, leveraging what the ESP32 microcontroller and the SSD1306 OLED module uniquely offer. Namely, it's 64 bars of logarithmically-spaced frequencies moving at *426* frames per second, produced from the following:

* the I2S and I2C/SPI peripherals, not to mention both cores of the ESP32
* a Constant Q transform ([cq_kernel](https://github.com/colonelwatch/cq_kernel)) computed from a 6144-point FFT ([kissfft](https://github.com/mborgerding/kissfft)), 142 times a second
  * the CQT essentially emulates sampling a bank of filters
* post-processing, including a 3x interpolation routine

So, please give this Arduino sketch an upload yourself. Its a project I started in 2019 then continued to improve over the years, and now, cameras literally cannot capture how nice this looks! The promised performance is on a SPI SSD1306 OLED module. I also have a routine for the ubiquitous I2C one, still at a respectable 89 frames per second. Below is an okay amplification circuit into pin 36/VP to get started with.

![Amplification circuit](/images/amplification.png)

It presents an impedance of at least 5k, which should be okay for line-level and definitely good with a phone. Alternatively, you can feed a line-level signal biased at half of 3.3 volts. You may also add an LED to pin 15 to get a clipping indicator.