# ESP32-oled-spectrum

[![Picture](/images/closeup.jpeg)](https://youtu.be/_1pETVk4ngk)
*Click through image to view demonstration on Youtube*

This is a quick adaptation of an old project, [ESP32-dotmatrix-spectrum](https://github.com/colonelwatch/ESP32-dotmatrix-spectrum), into the more accessible OLED. 
I borrowed from another project [attiny85-spectrum](https://github.com/colonelwatch/attiny85-spectrum), but the result was so appealing that I decided to share it!

⚠️ **Disclaimer!** ⚠️ This project will crash if it is compiled with the latest ESP32 core. Downgrade to version 1.0.4 through the Arduino boards manager before compiling.

There are also a number of improvements and notes here that will be carried over to the original project eventually.

That includes:
* My own latest [cq_kernels](https://github.com/colonelwatch/cq_kernel) library, which integrates kiss_fft into a fast constant Q transform. This gives true logarithmic frequency scaling.
  * Amplitudes are now in decibels, which is unambiguous though there is no obvious reference
* Switching to the kiss_fft library, which is faster and has a permissive license
* Other optimizations

If you are adjusting the settings, banding in the output means the FFT size is too small, however keep in mind that performance (refresh rate) and resolution (FFT size) are in a balance.

Here is a schematic for a decent amplification circuit into this project. It presents an impedance of at least 5k, which should be okay for line-level and definitely good with a phone. You may also add an LED to pin 19 to get a clipping indicator.

![Amplification circuit](/images/amplification.png)
