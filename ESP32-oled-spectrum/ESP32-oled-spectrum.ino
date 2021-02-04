// Copyright 2020 colonelwatch

// Note: This NEEDS the attached platform.local.txt file to compile correctly. It sends a
//  preprocessor flag to the kiss_fft library properly. To use it, copy it into:
//  C:\Users\%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4
#include "kiss_fftr.h"
#include "cq_kernel.h"
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
#define SAMPLES 8192                  // Must be a power of 2. Raise for higher resolution
                                      //  (less banding) and lower for faster performance.
#define SAMPLING_FREQUENCY 44100      // Hz, changing not recommended
#define MAX_FREQUENCY 14000           // Hz, must be 1/2 of sampling frequency or less
#define MIN_FREQUENCY 40              // Hz, cannot be 0, decreasing causes banding
// Post-processing settings
#define CUTOFF 20                     // Helps determine where to cut low-value noise in
                                      //  FFT output.
#define MINVAL 6500                   // Recommended range (3000-7500). Cutoff in Constant
                                      //  Q kernels. Raise for filter selectivity, lower
                                      //  for more bleed. Lower impacts performance slightly.
#define TIME_FACTOR 3.0               // Configures output smoothing (factor of exponential
                                      //  moving average)
#define THRESHOLD -70                 // dB, minimum display value
#define CAP -30                       // dB, maximum display value
// Device settings
#define COLUMNS 64                    // Number of columns to display (fewer columns will
                                      //  cause less banding)
#define COLUMN_SIZE 1                 // Size of columns in pixels

#define CLIP_PIN 19  // Connect LED to this pin to get a clipping indicator
#define INPUT_PIN 36 // Labeled VP
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 1000000UL);

/* Global constants, calculated from user-defined values */
const float coeff = 1./TIME_FACTOR;                 // Coefficients for output smoothing
const float anti_coeff = (TIME_FACTOR-1.)/TIME_FACTOR;
const int sample_period = 1000000/SAMPLING_FREQUENCY;
struct cq_kernel_cfg cq_cfg = { // Holds config for all cq_kernels functions to access
  .samples = SAMPLES,
  .bands = COLUMNS,
  .fmin = MIN_FREQUENCY,
  .fmax = MAX_FREQUENCY,
  .fs = SAMPLING_FREQUENCY,
  .min_val = MINVAL
};

/* Global variables */
cq_kernels_t kernels; // Will point to kernels allocated in dynamic memory
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

void calculate_Hamming(int16_t window[], int N);
void apply_window(int16_t in[], int16_t window[], int N);

/* Sampling interrupt */
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
  // Initializes kiss_fftr and window
  int16_t window[SAMPLES];
  calculate_Hamming(window, SAMPLES);
  kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);

  // Initalize benchmark
  float currentMillis = millis();
  refresh = 0;
  frames = 0;
  
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
    
    // Cutting off garbage values with a threshold inversely proportional to SAMPLES
    for(int i = 0; i < SAMPLES; i++){
      if(out[i].r < 2048*CUTOFF/SAMPLES) out[i].r = 0;
      if(out[i].i < 2048*CUTOFF/SAMPLES) out[i].i = 0;
    }

    // Convert FFT output to Constant Q output using kernels generated by cq_kernels
    kiss_fft_cpx bands_cpx[COLUMNS] = {0};
    apply_kernels(out, bands_cpx, kernels, cq_cfg);

    float out_columns[COLUMNS];
    static float past_columns[COLUMNS];
    for(int i = 0; i < COLUMNS; i++){
      // Finds decibel value of complex magnitude (relative to 1<<14, apparent maximum)
      int32_t mag_squared = bands_cpx[i].r*bands_cpx[i].r + bands_cpx[i].i*bands_cpx[i].i;
      out_columns[i] = 20*(log10(mag_squared)/2-log10(1<<14));

      // Converting decibel values into positive values and blocking anything under THRESHOLD
      if(out_columns[i] < THRESHOLD) out_columns[i] = 0;
      else out_columns[i] -= THRESHOLD;
      
      // Exponential moving average smoothing
      out_columns[i] = past_columns[i] * anti_coeff + out_columns[i] * coeff;
      past_columns[i] = out_columns[i];
  
      // Translating output data into column heights, which is entered into the buffer
      screenBuffer.writeBuffer[i] = 64*out_columns[i]/(CAP-THRESHOLD);
    }
    screenBuffer.swap_ready = true;   // Raises flag to indicate buffer is ready to push

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
    
    // Generate kernels (memory-intensive task) before allocating core 0 stack
    kernels = generate_kernels(cq_cfg);
    kernels = reallocate_kernels(kernels, cq_cfg);

    // Initializes sampling interrupt
    timer = timerBegin(1, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, sample_period, true);
    timerAlarmEnable(timer);
    
    // Launch Task1code on core 0 with no parameters (access global variables)
    xTaskCreatePinnedToCore(Task1code, "Task1", 100000, NULL, 0, &Task1, 0);
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
    }
    display.display();
    
    refresh++;
}

// Calculates Hamming window in Q15 form
void calculate_Hamming(int16_t window[], int N){
  float a0 = 0.54;
  for(int i = 0; i < N; i++) window[i] = (1<<15)*(a0-(1-a0)*cos(2*PI*i/N));
}

// Applies a window in Q15 form
void apply_window(int16_t in[], int16_t window[], int N){
  for(int i = 0; i < N; i++) in[i] = (in[i]*window[i])/(1<<15);
}