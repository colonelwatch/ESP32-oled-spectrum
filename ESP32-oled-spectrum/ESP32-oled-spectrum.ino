// Copyright 2020 colonelwatch

// Note to self: the latest code was compiled with O2 optimization. That
//  should make this code run faster if you configure this too.
// To do this, go to the platforms.txt files for the ESP32 in your AppData
//  Arduino folder, under the "tools" folder and replace any "-Os" with "-O2".
#include "fix_fft16.c"
// Used for watchdog reset
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

/* User-configurable settings */
// Sampling settings
#define OVERSAMPLE 1                  // Number of readings collected per sample, results
                                      //  in less aliasing. Only necessary when sampling
                                      //  at under 44.1 kHz, and raises overhead.
// FFT settings
#define SAMPLES 2048                  // Must be a power of 2. Raise for higher resolution
                                      //  (less banding) and lower for faster performance.
                                      //  Currently cannot greater than 2048.
#define SAMPLING_FREQUENCY 44100      // Hz, raise for greater frequency range, decrease to
                                      //  reduce banding
#define MAX_FREQUENCY 20000           // Hz, must be 1/2 of sampling frequency or less
#define MIN_FREQUENCY 40              // Hz, cannot be 0, decreasing causes banding
// Post-processing settings
#define TIME_FACTOR 4                 // Configures rise smoothing function (raise
                                      //  for smoother output, lower for dynamic
                                      //  output)
#define TIME_FACTOR2 4                // Configures fall smoothing function (raise
                                      //  for smoother output, lower for dynamic
                                      //  output.
#define SENSITIVITY 1.00              // FFT output multiplier before post-processing.
#define CAP 100                       // Use to map post-processed FFT output to
                                      //  display (raise for longer bars, lower
                                      //  to shorter bars)
#define FLOOR 2                       // Use to cut off the buttom of the post-processed
                                      //  output.
// Device settings
#define DEBOUNCE 500                  // Debounce time in milliseconds for BOOT button
#define COLUMNS 32                    // Number of columns to display, fewer columns
                                      //  will cause less banding
#define COLUMN_SIZE 1                 // Size of columns in pixels


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 1000000UL);
// Microphone input is 39
// 3.5mm input is 36

/* Global constants, calculated from user-defined values */
const float coeff = 1./TIME_FACTOR;                 // Coefficients for rise smoothing
const float anti_coeff = (TIME_FACTOR-1.)/TIME_FACTOR;
const float coeff2 = 1./TIME_FACTOR2;               // Coefficients for fall smoothing
const float anti_coeff2 = (TIME_FACTOR2-1.)/TIME_FACTOR2;
const int sample_period = 1000000/SAMPLING_FREQUENCY/OVERSAMPLE;

/* Global variables */
// Benchmarking variables
int frames;
volatile int refresh;

volatile float lin_fn[SAMPLES/2], log_fn[COLUMNS];  // Stores frequencies of FFT bins and output bands
volatile int analogBuffer[SAMPLES] = {0};         // Circular buffer for storing analogReads
volatile int analogBuffer_index = 0;          // Write index for analogBuffer, also used for reads
volatile bool analogBuffer_availible = true;  // Memory busy flag for analogBuffer
// Note to self, alter buffer sizes for OLED
uint8_t displayBuffer[64] = {0};       // First buffer for storing LED matrix output
uint8_t doubleBuffer[64]  = {0};       // Second buffer for storing LED matrix output
uint8_t *readBuffer = doubleBuffer;     // Pointer that specifies which buffer is to be read from
uint8_t *writeBuffer = displayBuffer;   // Pointer that specifies which buffer is to be written to
volatile bool displayBuffer_availible = true;    // Memory busy flag for display buffer
// End note
volatile int inputPin = 36;      // ADC pin, either 36(aux) or 39(microphone)

/* Function prototypes, for some reason the complier wants them */
void analogBuffer_store(int val);
void watchdogReset();
//*Flash display function prototype removed
int8_t int_sqrt(int16_t val);

/* Core 0 Interrupt thread */
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer(){
  // Basic function is to record new values for circular analogBuffer, but in cases where
  // the interrupt is triggered while the buffer is being read from, for the sake of
  // accuracy the values must be stored in a contingency buffer. Then, when the interrupt
  // is triggered againt and the reading is over, the values are transferred from the
  // contingency buffer to analogBuffer.
  // Moreover, by sampling at SAMPLING_FREQUENCY*OVERSAMPLE and taking the average of
  // OVERSAMPLE samples, the original sampling frequency is preserved, and and digital
  // filter with a Nyquist corner frequency
  static int tempBuffer[OVERSAMPLE];
  static int tempBuffer_index = 0;
  tempBuffer[tempBuffer_index++] = analogRead(inputPin)-2048;
  tempBuffer_index %= OVERSAMPLE;

  if(tempBuffer_index == 0){
    int virtual_reading = 0;
    for(int i = 0; i < OVERSAMPLE; i++) virtual_reading += tempBuffer[i];
    
    static int contigBuffer[SAMPLES];
    static int contigBuffer_index = 0;
    if(!analogBuffer_availible){
      contigBuffer[contigBuffer_index] = virtual_reading;
      contigBuffer_index++;
    }
    else{
      for(int i = 0; i < contigBuffer_index; i++)
        analogBuffer_store(contigBuffer[i]);
      contigBuffer_index = 0;
      analogBuffer_store(virtual_reading);
    }
  }
}

/* Core 0 thread */
TaskHandle_t Task1;
void Task1code( void * pvParameters ){
  for(int i = 0; i < SAMPLES/2; i++) lin_fn[i] = SAMPLING_FREQUENCY * (i + 0.5) / SAMPLES;
  float f_step = (log((float)MAX_FREQUENCY/MIN_FREQUENCY)/log(2.)) / COLUMNS;
  for(int i = 0; i < COLUMNS; i++){
    if(i == 0) log_fn[i] = MIN_FREQUENCY * pow(2.,f_step/2.);
    else log_fn[i] = (float)log_fn[i-1] * pow(2.,f_step);
  }

  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, sample_period, true);
  timerAlarmEnable(timer);
  
  while(true){
    // Debouncing code that checks if the BOOT button has been pressed and, if
    // so, changes the audio source between the microphone and the 3.5mm jack
    
    static unsigned long milliseconds = millis();
    // Update so that this does not overflow after enough time
    if(digitalRead(0) == LOW && millis() > milliseconds + DEBOUNCE){
      milliseconds = millis();
      if(inputPin == 36) inputPin = 39;
      else inputPin = 36;
    }
    
    int sum = 0;
    int16_t vReal[SAMPLES] = {0};
    int16_t vImag[SAMPLES] = {0};
    /*SAMPLING*/
    // Reads entire analog buffer for FFT calculations. This means a LOT of
    // redundant data but its necessary to because using new data every time
    // requires very long delays in total when recording at low frequencies.
    while(!analogBuffer_availible) watchdogReset();
    analogBuffer_availible = false;   // Closes off buffer to prevent corruption
    // Reads entire circular buffer, starting from analogBuffer_index
    for(int i = 0; i < SAMPLES; i++){
      // Samples are doubled to maximize precision at later stages
      vReal[i] = analogBuffer[(i+analogBuffer_index)%SAMPLES]*2;
      sum += vReal[i];
    }
    analogBuffer_availible = true;    // Restores access to buffer
  
    int avg = sum/SAMPLES;
    for(int i = 0; i < SAMPLES; i++) vReal[i] -= avg;
   
    // Finds size of samples in a power of 2 and does the FFT
    int power = 1;
    while(SAMPLES != 1 << power) power++;
    fft_windowing_Hamming(vReal,power);     
                                    // Hamming windowing function based on a
                                    //  Hann windowing function I found online
                                    //  for 8-bit numbers and adapted for the
                                    //  original 16-bit library. Reduces FFT
                                    //  amplitudes by around 2 to 4.
    fix_fft(vReal,vImag,power,0);   // Faster, real version is not used because
                                    //  of frequency response issues.
                                    //  TO-DO: Fix the problems and implement?
    
    for(int i = 0; i < SAMPLES/2; i++)
      vReal[i] = int_sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);

    // Adapted from open source Rainmeter's audio visualization code with
    //  permission from the development team.
    //  Relevant Github file: https://github.com/rainmeter/rainmeter/blob/master/Plugins/PluginAudioLevel/PluginAudioLevel.cpp
    int output[COLUMNS] = {0};
    int iBand = 0;
    int iBin = 0;
    int binCount = 0;
    int f0 = 0;
    while(iBand < COLUMNS && iBin <= SAMPLES/2){
      binCount++;
      if(lin_fn[iBin] <= log_fn[iBand]){
        if(vReal[iBin] > 2) output[iBand] += vReal[iBin];
        iBin++;
      }
      else{
        if(vReal[iBin] > 2) output[iBand] += vReal[iBin];
        binCount = 0;
        iBand++;
      }
    }

    // Performs multiple operations: flattening, post-processing, and smoothing
    uint16_t postprocess[64];
    static float buff[64];
    
    for(int iCol = 0; iCol < 64; iCol++){
      // Combining imaginary and real data into a unified array
      postprocess[iCol] = SENSITIVITY*(output[iCol]);
      // Logarithmic scaling to create more visible output display height of 16
      // log(0.5) term is added to compensate for buffer pulling at sampling stage
      // log(SENSITIVITY) term is to emulate the above line (which MUST be used for
      //  the below if statement for some reason) but with greater precision
      //  TO-DO: Find out how to eleiminate dependence on above statement
      if(postprocess[iCol] > OVERSAMPLE)
        postprocess[iCol] = 40.*(log((float)(output[iCol]))+log(0.5/OVERSAMPLE)+log(SENSITIVITY));
      else postprocess[iCol] = 0;   // Cuts off negative values before they are calculated
      //
      
      if(postprocess[iCol] > buff[iCol]){
        // Smoothing by factoring in past data
        postprocess[iCol] = buff[iCol] * anti_coeff + (float)postprocess[iCol] * coeff;
        buff[iCol] = postprocess[iCol];       // Storing new output as next frame's past data
      }
      else{
        postprocess[iCol] = buff[iCol] * anti_coeff2 + (float)postprocess[iCol] * coeff2;
        buff[iCol] = postprocess[iCol];
      }
    }

    // Cutting off the smaller numbers representing signals too weak to be relevant
    for(int i = 0; i < COLUMNS; i++){
      if(postprocess[i] < FLOOR) postprocess[i] = 0;
      else postprocess[i] -= FLOOR;
    }
  
    // Translating output data into column heights, which is entered into the buffer
    displayBuffer_availible = false;  // REDUNDANT, Old code to block access to the buffer
                                      //  TODO: Remove?
    for(int i = 0; i < COLUMNS; i++) writeBuffer[i] = postprocess[i]*64/CAP;
    displayBuffer_availible = true;   // Signals to other core that write is complete, and
                                      //  that it can switch the buffers
    
    // Outputs benchmark data
    frames++;
    watchdogReset();
    static bool benchmark_posted = false;
    if(millis() > 5000 && !benchmark_posted){
      Serial.print(frames/5);
      Serial.print(' ');
      Serial.println(refresh/5);
      benchmark_posted = true;
    }
  }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(15, INPUT);

    //*LED display output pins setup removed
    
    pinMode(39, INPUT);
    pinMode(36, INPUT);
    pinMode(0, INPUT_PULLUP);

    displayBuffer_availible = false;
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();
    
    xTaskCreatePinnedToCore(
              Task1code,   /* Task function. */
              "Task1",     /* name of task. */
              100000,       /* Stack size of task */
              NULL,        /* parameter of the task */
              0,           /* priority of the task */
              &Task1,      /* Task handle to keep track of created task */
              0);          /* pin task to core 0 */ 
    delay(500);
}

void loop() {
    // Double-buffered output with page flipping to display
    if(displayBuffer_availible){
      uint8_t *tempPointer = readBuffer;
      readBuffer = writeBuffer;
      writeBuffer = tempPointer;
      displayBuffer_availible = false;
    }

    display.clearDisplay();
    display.drawFastHLine(0, 63, 128, WHITE);
    const int pixels_per_column = 128/COLUMNS;
    for(int i = 0; i < COLUMNS; i++){
      display.drawRect(i*pixels_per_column-COLUMN_SIZE, 64-readBuffer[i], COLUMN_SIZE, readBuffer[i], WHITE);
      display.fillRect(i*pixels_per_column-COLUMN_SIZE, 64-readBuffer[i], COLUMN_SIZE, readBuffer[i], WHITE);
      //display.drawFastVLine(i*pixels_per_column, 64-readBuffer[i], readBuffer[i], WHITE);
    }
    display.display();
    
    refresh++;
}

// Stores the passed value into a SAMPLES-sized circular buffer called analogBuffer,
//  which is declared as a volatile int array. Uses global volatile int
//  analogBuffer_index as the write index and reads analogBuffer_availible as
//  memory flag.
void analogBuffer_store(int val){
  analogBuffer[analogBuffer_index] = val;

  // Increments index then uses modulo to limit range
  analogBuffer_index++;
  analogBuffer_index %= SAMPLES;
}
// Function note: Currently, analogBuffer_index is also used as the reference
// in order to read the entire array in chronological order.

void watchdogReset(){
  // Credit goes to akshar001 for this watchdog reset three-liner
  // Github thread: https://github.com/espressif/arduino-esp32/issues/595
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed=1;
  TIMERG0.wdt_wprotect=0;
}

//* Flash display function definition removed

// Recursively computes square root and replaces the Arduino-provided sqrt
// function, which used floating point math. Outputs reasonably for any
// number less than int32_t max but works fastest on smaller numbers like
// in this case.
int16_t int_sqrt(int32_t val){
  // Initial values if n = 0
  int32_t Nsquared = 0;
  int32_t twoNplus1 = 1;
  while(Nsquared <= val){
    Nsquared += twoNplus1;
    twoNplus1 += 2;
  }
  return twoNplus1/2 - 1;
}
