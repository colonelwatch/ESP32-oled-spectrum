// Copyright 2020 colonelwatch

// Note: This NEEDS the attached platform.local.txt file to compile correctly. It sends a
//  preprocessor flag to the kiss_fft library properly. To use it, copy it into:
//  C:\Users\%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4
#include "kiss_fftr.h"
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
#define SAMPLING_FREQUENCY 44100      // Hz, changing not recommended
#define MAX_FREQUENCY 14000           // Hz, must be 1/2 of sampling frequency or less
#define MIN_FREQUENCY 40              // Hz, cannot be 0, decreasing causes banding
// Post-processing settings
#define TIME_FACTOR 6.0               // Configures rise smoothing (factor of exponential
                                      //  moving average)
#define TIME_FACTOR2 6.0              // Configures fall smoothing (same as above)
#define THRESHOLD -40                 // dB, minimum display value
#define CAP -20                       // dB, maximum display value
// Device settings
#define COLUMNS 32                    // Number of columns to display (fewer columns will
                                      //  cause less banding)
#define COLUMN_SIZE 2                 // Size of columns in pixels

#define CLIP_PIN 19  // Connect LED to this pin to get a clipping indicator
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
    volatile int read_index = 0;
    volatile int write_index = 0;
    volatile bool available = true;
    void insert(TYPE val){
      buffer[write_index] = val;
      write_index++;
      write_index %= buffer_size;
    }
    TYPE pop(){
      TYPE val = buffer[read_index];
      read_index++;
      read_index %= buffer_size;
      return val;
    }
    bool empty(){
      return read_index == write_index;
    }
    circularBuffer(int size){
      buffer_size = size;
      buffer = (TYPE*)calloc(size, sizeof(TYPE));
    }
};
circularBuffer<int> analogBuffer(SAMPLES);

void watchdogReset();
int16_t int_sqrt(int32_t val);
void calculate_Hann(int16_t window[], int N);
void apply_window(int16_t in[], int16_t window[], int N);

/* Core 0 Interrupt thread */
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer(){
  // Basic function is to insert new values for circular analogBuffer, but in cases where
  // the interrupt is triggered while the buffer is being read from, for the sake of
  // accuracy the values must be stored in a contingency buffer. Then, when the interrupt
  // is triggered again and the reading is over, the values are transferred from the
  // contingency buffer to analogBuffer.

  static circularBuffer<int> contigBuffer(SAMPLES);

  int val = analogRead(INPUT_PIN);
  if(val > 4000) digitalWrite(CLIP_PIN, HIGH);
  else digitalWrite(CLIP_PIN, LOW);

  if(!analogBuffer.available) contigBuffer.insert(val);
  else{
    while(!contigBuffer.empty()) analogBuffer.insert(contigBuffer.pop());
    analogBuffer.insert(val);
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

  // Initializes kiss_fftr and window
  int16_t window[SAMPLES];
  calculate_Hann(window, SAMPLES);
  kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);

  // Initalize benchmark
  float currentMillis = millis();
  
  while(true){
    int32_t sum = 0;
    int16_t in[SAMPLES];
    // Reads ENTIRE circular analog buffer starting from analogBuffer.write_index, this
    //  means a lot of overlapping data, but it decouples FFT calculations from sampling
    analogBuffer.available = false;   // Closes off buffer to prevent corruption
    // Reads entire circular buffer, starting from analogBuffer_index
    for(int i = 0; i < SAMPLES; i++){
      int val = analogBuffer.buffer[(i+analogBuffer.write_index)%SAMPLES];
      in[i] = 16*(val - 2048); // TODO: possible clipping when subtracting the average?
      sum += in[i];
    }
    analogBuffer.available = true;    // Restores access to buffer
    
    // Removes leftover DC bias in signal
    int16_t avg = sum/SAMPLES;
    for(int i = 0; i < SAMPLES; i++) in[i] -= avg;
   
    // Applies windowing and FFT
    kiss_fft_cpx out[SAMPLES] = {0};
    apply_window(in, window, SAMPLES);
    kiss_fftr(cfg, in, out);
    
    // Finds magnitudes of complex "out" (normalized) and places them back in contiguous "in"
    for(int i = 0; i < SAMPLES/2; i++)
      in[i] = 2*int_sqrt(out[i].r*out[i].r + out[i].i*out[i].i); // TODO: Switch with int version

    // Adapted from open source Rainmeter's audio visualization code with permission from the development team.
    //  https://github.com/rainmeter/rainmeter/blob/master/Plugins/PluginAudioLevel/PluginAudioLevel.cpp
    int16_t out_bands[COLUMNS] = {0};
    int iBand = 0;
    int iBin = 0;
    while(iBand < COLUMNS && iBin <= SAMPLES/2){
      if(lin_fn[iBin] <= log_fn[iBand]){
        out_bands[iBand] += in[iBin];
        iBin++;
      }
      else{
        out_bands[iBand] += in[iBin];
        iBand++;
      }
    }

    float out_columns[COLUMNS];
    static float past_columns[COLUMNS];
    for(int i = 0; i < COLUMNS; i++){
      out_columns[i] = 20*(log10(out_bands[i])-log10(1<<15)); // Finds decibel value

      // Converting decibel values into positive values and blocking anything under THRESHOLD
      if(out_columns[i] < THRESHOLD) out_columns[i] = 0;
      else out_columns[i] -= THRESHOLD;
      
      // Exponential moving average smoothing
      if(out_columns[i] > past_columns[i])
        out_columns[i] = past_columns[i] * anti_coeff + out_columns[i] * coeff;
      else
        out_columns[i] = past_columns[i] * anti_coeff2 + out_columns[i] * coeff2;
      past_columns[i] = out_columns[i];
  
      // Translating output data into column heights, which is entered into the buffer
      screenBuffer.writeBuffer[i] = 64*out_columns[i]/(CAP-THRESHOLD);
    }
    screenBuffer.swap_ready = true;   // Raises flag to indicate buffer is ready to push
    
    watchdogReset();

    // Outputs benchmark data
    frames++;
    static bool benchmark_posted = false;
    if(millis()-currentMillis > 5000 && !benchmark_posted){
      Serial.print(frames/5);
      Serial.print(' ');
      Serial.println(refresh/5);
      benchmark_posted = true;
    }
  }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(CLIP_PIN, OUTPUT);
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

// Calculates Hann window in Q15 form
void calculate_Hann(int16_t window[], int N){
  for(int i = 0; i < N; i++){
    float a0 = 0.5;
    window[i] = (1<<15)*(a0-(1-a0)*cos(2*PI*i/N));
  }
}

// Applies a window in Q15 form
void apply_window(int16_t in[], int16_t window[], int N){
  for(int i = 0; i < N; i++) in[i] = (in[i]*window[i])/(1<<15);
}

// Computes square root and replaces the Arduino-provided sqrt function, which used floating point math.
// Outputs reasonably fast for any number less than int32_t max but works fastest on smaller numbers.
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