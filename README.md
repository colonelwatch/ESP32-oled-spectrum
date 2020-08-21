# ESP32-oled-spectrum

[![Picture](/images/closeup.jpeg)](https://www.youtube.com/watch?v=TEzQY4_xW6o)

*Click through for youtube video of demonstration*

This is a quick adaptation of an old project, [ESP32-dotmatrix-spectrum](https://github.com/colonelwatch/ESP32-dotmatrix-spectrum), into the more accessible OLED. 
I borrowed from another project [attiny85-spectrum](https://github.com/colonelwatch/attiny85-spectrum), but the result was so appealing that I decided to share it!

There are also a number of improvements and notes here that will be carried over to the original project eventually.

That includes:
* True logarithmic scaling (frequency- *and* ampltitude-wise!), credit to and permission received from the [Rainmeter](https://github.com/rainmeter/rainmeter) project
regarding their frequency scaling code.
  * Apologies to their dev team for taking my sweet time with the project!
  * Gating of the FFT output to stop artifacting
* Reverting to the original fix_fft library, which was 16-bit integer FFT, but now with a maximum size of 2048
* Oversampling to prevent aliasing
* Replaced floating-point sqrt() with recursive integer-based square root and other optimizations
* Improved semantics

If you are adjusting the settings, banding in the output means the FFT size is too small, however keep in mind that 
performance (refresh rate) and resolution (FFT size) are in a balance.

Here are some goals toward expanding room for both:
* I2S sampling, to eliminate sampling overhead
* Newton's square root, if it helps

Finally, a note on the attached schematic: this was NOT designed for line-level audio. Rather, it is
meant to amplify listening level signals for headphones or earbuds. However, this was tested with a not very
sensitive pair, so use a smaller resistor for (or short) R7 if you need more gain. Alternatively, raise the
SENSITIVITY setting in the code.

![Amplification circuit](/images/amplification.png)
