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
// FFT settings
#define SAMPLES 2048                  // Must be a power of 2. Raise for higher resolution
                                      //  (less banding) and lower for faster performance.
                                      //  Currently cannot greater than 2048.
#define SAMPLING_FREQUENCY 44100      // Hz, changing not recommended
#define MAX_FREQUENCY 14000           // Hz, must be 1/2 of sampling frequency or less
#define MIN_FREQUENCY 40              // Hz, cannot be 0, decreasing causes banding
// Post-processing settings
#define TIME_FACTOR 4.0               // Configures rise smoothing (factor of exponential
                                      //  moving average)
#define TIME_FACTOR2 4.0              // Configures fall smoothing (same as above)
#define SENSITIVITY 0.125             // FFT output multiplier before post-processing.
                                      //  (increasing trades more output artifacting
                                      //  for sensitivity)
#define THRESHOLD 32                  // Minimum threshold for log-scale frequency mapping,
                                      //  necessary to reduce noise artifacts (changing not
                                      //  recommended)
#define CAP 100                       // Use to map post-processed FFT output to
                                      //  display (changing not recommended)
// Device settings
#define COLUMNS 32                    // Number of columns to display (fewer columns will
                                      //  cause less banding)
#define COLUMN_SIZE 2                 // Size of columns in pixels

#define INPUT_PIN 36 // Labeled VP
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 1000000UL);

/* Global constants, calculated from user-defined values */
const float coeff = 1./TIME_FACTOR;                 // Coefficients for rise smoothing
const float anti_coeff = (TIME_FACTOR-1.)/TIME_FACTOR;
const float coeff2 = 1./TIME_FACTOR2;               // Coefficients for fall smoothing
const float anti_coeff2 = (TIME_FACTOR2-1.)/TIME_FACTOR2;
const int sample_period = 1000000/SAMPLING_FREQUENCY;

/* Global variables */
// Benchmarking variables
int frames;
volatile int refresh;

template <typename TYPE> class doubleBuffer{
  public:
    volatile TYPE *readBuffer, *writeBuffer;
    volatile bool swap_ready = false;
    void swap(){
      volatile TYPE *temp = readBuffer;
      readBuffer = writeBuffer;
      writeBuffer = temp;
    }
    doubleBuffer(int size){
      readBuffer = (TYPE*)calloc(size, sizeof(TYPE));
      writeBuffer = (TYPE*)calloc(size, sizeof(TYPE));
    }
};
doubleBuffer<uint8_t> screenBuffer(COLUMNS);

template <typename TYPE> class circularBuffer{
  public:
    int buffer_size;
    volatile TYPE *buffer;
    volatile int write_index;
    volatile bool available = true;
    void insert(TYPE val){
      buffer[write_index] = val;
      write_index++;
      write_index %= buffer_size;
    }
    circularBuffer(int size){
      buffer_size = size;
      buffer = (TYPE*)calloc(size, sizeof(TYPE));
    }
};
circularBuffer<int16_t> analogBuffer(SAMPLES);

void watchdogReset();
//*Flash display function prototype removed
int8_t int_sqrt(int16_t val);

/* Core 0 Interrupt thread */
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer(){
  // Basic function is to insert new values for circular analogBuffer, but in cases where
  // the interrupt is triggered while the buffer is being read from, for the sake of
  // accuracy the values must be stored in a contingency buffer. Then, when the interrupt
  // is triggered again and the reading is over, the values are transferred from the
  // contingency buffer to analogBuffer.
  
  static int contigBuffer[SAMPLES];
  static int contigBuffer_index = 0;
  if(!analogBuffer.available){
    contigBuffer[contigBuffer_index] = analogRead(INPUT_PIN) - 2048;
    contigBuffer_index++;
  }
  else{
    for(int i = 0; i < contigBuffer_index; i++)
      analogBuffer.insert(contigBuffer[i]);
    contigBuffer_index = 0;
    analogBuffer.insert(analogRead(INPUT_PIN) - 2048);
  }
}

/* Core 0 thread */
TaskHandle_t Task1;
void Task1code( void * pvParameters ){
  // Preps frequency information for scaling algorithm (credit to Rainmeter project below)
  float lin_fn[SAMPLES/2], log_fn[COLUMNS];
  for(int i = 0; i < SAMPLES/2; i++) lin_fn[i] = SAMPLING_FREQUENCY * (i + 0.5) / SAMPLES;
  float f_step = (log((float)MAX_FREQUENCY/MIN_FREQUENCY)/log(2.)) / COLUMNS;
  for(int i = 0; i < COLUMNS; i++){
    if(i == 0) log_fn[i] = MIN_FREQUENCY * pow(2.,f_step/2.);
    else log_fn[i] = (float)log_fn[i-1] * pow(2.,f_step);
  }

  // Initializes sampling interrupt
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, sample_period, true);
  timerAlarmEnable(timer);
  
  while(true){
    int sum = 0;
    int16_t vReal[SAMPLES] = {0};
    int16_t vImag[SAMPLES] = {0};
    // Reads ENTIRE circular analog buffer starting from analogBuffer.write_index, this
    //  means a lot of overlapping data, but it decouples FFT calculations from sampling
    analogBuffer.available = false;   // Closes off buffer to prevent corruption
    // Reads entire circular buffer, starting from analogBuffer_index
    for(int i = 0; i < SAMPLES; i++){
      // Samples are upscaled to maximize precision at later stages
      vReal[i] = analogBuffer.buffer[(i+analogBuffer.write_index)%SAMPLES]*16;
      sum += vReal[i];
    }
    analogBuffer.available = true;    // Restores access to buffer
  
    // Removes leftover DC bias in signal
    int avg = sum/SAMPLES;
    for(int i = 0; i < SAMPLES; i++) vReal[i] -= avg;
   
    // Finds size of samples in a power of 2 and does the FFT
    int power = 1;
    while(SAMPLES != 1 << power) power++;
    fft_windowing_Hamming(vReal, power);  // Hamming windowing function based on a Hann 
                                          //  windowing function and adapted for the 
                                          //  original 16-bit library. Reduces FFT amplitudes
                                          //  by around 2 to 4.
    fix_fft(vReal, vImag, power, 0);
    
    for(int i = 0; i < SAMPLES/2; i++)
      vReal[i] = int_sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);

    // Adapted from open source Rainmeter's audio visualization code with permission from the development team.
    //  https://github.com/rainmeter/rainmeter/blob/master/Plugins/PluginAudioLevel/PluginAudioLevel.cpp
    int output[COLUMNS] = {0};
    int iBand = 0;
    int iBin = 0;
    while(iBand < COLUMNS && iBin <= SAMPLES/2){
      if(lin_fn[iBin] <= log_fn[iBand]){
        if(vReal[iBin] > THRESHOLD) output[iBand] += vReal[iBin];
        iBin++;
      }
      else{
        if(vReal[iBin] > THRESHOLD) output[iBand] += vReal[iBin];
        iBand++;
      }
    }

    float postprocess[64];
    static float buff[64];
    for(int iCol = 0; iCol < 64; iCol++){
      // Applying log scale, log(0.0625) term compensates for 16x scaling before FFT and
      //  log(SENSITIVITY) term applies sensitivity
      if(output[iCol] > 16/SENSITIVITY)
        postprocess[iCol] = 40*(log((float)(output[iCol]))+log(0.0625)+log(SENSITIVITY));
      else postprocess[iCol] = 0;   // Cuts off negative values before they are calculated
      
      // Exponential moving average smoothing
      if(postprocess[iCol] > buff[iCol])
        postprocess[iCol] = buff[iCol] * anti_coeff + postprocess[iCol] * coeff;
      else
        postprocess[iCol] = buff[iCol] * anti_coeff2 + postprocess[iCol] * coeff2;
      buff[iCol] = postprocess[iCol];
    }
  
    // Translating output data into column heights, which is entered into the buffer
    for(int i = 0; i < COLUMNS; i++) screenBuffer.writeBuffer[i] = 64*postprocess[i]/CAP;
    screenBuffer.swap_ready = true;   // Raises flag to indicate buffer is ready to push
    
    watchdogReset();

    // Outputs benchmark data
    frames++;
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
    
    pinMode(INPUT_PIN, INPUT);
    pinMode(0, INPUT_PULLUP);

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
    if(screenBuffer.swap_ready){
      screenBuffer.swap();
      screenBuffer.swap_ready = false;
    }

    display.clearDisplay();
    display.drawFastHLine(0, 63, 128, WHITE);
    const int col_px = 128/COLUMNS;
    for(int i = 0; i < COLUMNS; i++){
      int length = screenBuffer.readBuffer[i];
      display.drawRect(i*col_px-COLUMN_SIZE, 64-length, COLUMN_SIZE, length, WHITE);
      display.fillRect(i*col_px-COLUMN_SIZE, 64-length, COLUMN_SIZE, length, WHITE);
      //display.drawFastVLine(i*pixels_per_column, 64-readBuffer[i], readBuffer[i], WHITE);
    }
    display.display();
    
    refresh++;
}

void watchdogReset(){
  // Credit goes to akshar001 for this watchdog reset three-liner
  // Github thread: https://github.com/espressif/arduino-esp32/issues/595
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed=1;
  TIMERG0.wdt_wprotect=0;
}

//* Flash display function definition removed

// Recursively computes square root and replaces the Arduino-provided sqrt function, 
// which used floating point math. Outputs reasonably for any number less than int32_t 
// max but works fastest on smaller numbers.
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
