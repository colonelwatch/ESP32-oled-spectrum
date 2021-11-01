// Copyright 2020 colonelwatch

// Note: This WILL crash if it is compiled with the latest ESP32 core. Downgrade
//  to version 1.0.4 through the Arduino boards manager before compiling.
// Note: This NEEDS the attached platform.local.txt file to compile correctly. 
//  It sends a preprocessor flag to the kiss_fft library properly. To use it, 
//  copy it into:
//  C:\Users\%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "kiss_fftr.h"
#include "cq_kernel.h"

/* User-configurable global constants */
// FFT settings
#define SAMPLES 8192        // Must be a power of 2. Raise for less banding and 
                            //  lower for faster performance.
#define MAX_FREQUENCY 14000 // Hz, must be 1/2 of sampling frequency or less
#define MIN_FREQUENCY 40    // Hz, cannot be 0, decreasing causes banding
// Post-processing settings
#define CUTOFF 20           // Helps determine where to cut low-value noise in
                            //  FFT output. Set as low as the noise will allow.
#define TIME_FACTOR 3.0     // Output smoothing factor (exponential moving 
                            //  average alpha), 1.0 for no smoothing
#define THRESHOLD -70       // dB, minimum display value
#define CAP -30             // dB, maximum display value
// Device settings
#define COLUMNS 64          // Number of columns to display (fewer columns will
                            //  cause less banding)
#define COLUMN_SIZE 1       // pixels, size of columns
#define CLIP_PIN 19         // Connect LED to this pin to get a clipping indicator
#define INPUT_PIN 36        // Connect DC-biased line-level audio signal to this

/* Other global constants, changing not recommended */ 
#define SCREEN_WIDTH 128              // pixels, OLED display width
#define SCREEN_HEIGHT 64              // pixels, OLED display height
#define SAMPLING_FREQUENCY 44100      // Hz
#define MINVAL 6500                   // used in cq_kernel

/* Other global constants, calculated from #define'd values */
const float coeff = 1./TIME_FACTOR;
const float anti_coeff = (TIME_FACTOR-1.)/TIME_FACTOR;
const int16_t minimum_mag = 2048*CUTOFF/SAMPLES; // in FFT value
const int32_t minimum_mag_squared = minimum_mag*minimum_mag;
const int sample_period = 1000000/SAMPLING_FREQUENCY;
struct cq_kernel_cfg cq_cfg = { // config for cq_kernel
    .samples = SAMPLES,
    .bands = COLUMNS,
    .fmin = MIN_FREQUENCY,
    .fmax = MAX_FREQUENCY,
    .fs = SAMPLING_FREQUENCY,
    .min_val = MINVAL
};

/* Global variables */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 1000000UL);
cq_kernels_t kernels; // Will point to kernels allocated in dynamic memory
int frames; volatile int refresh; // Benchmarking variables

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
circularBuffer<int> analogBuffer(SAMPLES), contigBuffer(512);

/* Sampling interrupt */
hw_timer_t *timer = NULL;
void IRAM_ATTR onTimer(){
    // Basic function is to insert new values for circular analogBuffer. However, 
    // if the interrupt is triggered as analogBuffer is being read, the new value 
    // is stashed in contigBuffer and inserted later.

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
void Task1code(void *pvParameters){
    // Allocate some large arrays
    int16_t *in = (int16_t*)malloc(SAMPLES*sizeof(int16_t));
    kiss_fft_cpx *out = (kiss_fft_cpx*)malloc(SAMPLES*sizeof(kiss_fft_cpx));
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);
    kiss_fft_cpx *bands_cpx = (kiss_fft_cpx*)malloc(COLUMNS*sizeof(kiss_fft_cpx));
    float *past_dB_level = (float*)calloc(COLUMNS, sizeof(float));

    // Initalize benchmark
    float currentMillis = millis();
    refresh = 0;
    frames = 0;
    
    while(true){
        // Reads ENTIRE analogBuffer starting from analogBuffer.write_index, 
        //  this means a lot of overlap, but it decouples FFT calculations from 
        //  sampling
        analogBuffer.available = false;   // Closes off buffer to prevent corruption
        int32_t sum = 0;
        for(int i = 0; i < SAMPLES; i++){
            int val = analogBuffer.buffer[(i+analogBuffer.write_index)%SAMPLES];
            in[i] = 16*(val-2048);
            sum += in[i];
        }
        analogBuffer.available = true;    // Restores access to buffer
        
        int16_t avg = sum/SAMPLES;
        for(int i = 0; i < SAMPLES; i++){
            in[i] -= avg;
            out[i] = (kiss_fft_cpx){0, 0}; // necessary before calling kiss_fftr
        }
      
        kiss_fftr(cfg, in, out);
        
        // Cutting off noise with a threshold inversely proportional to SAMPLES
        for(int i = 0; i < SAMPLES; i++)
            if(out[i].r*out[i].r+out[i].i*out[i].i < minimum_mag_squared)
                out[i] = (kiss_fft_cpx){0, 0};

        // Convert FFT output to Constant Q output using cq_kernel
        for(int i = 0; i < COLUMNS; i++) bands_cpx[i] = (kiss_fft_cpx){0, 0};
        apply_kernels(out, bands_cpx, kernels, cq_cfg);

        for(int i = 0; i < COLUMNS; i++){
            // Finds decibel value of complex magnitude (relative to 1<<14, apparent maximum)
            int32_t mag_squared = bands_cpx[i].r*bands_cpx[i].r+bands_cpx[i].i*bands_cpx[i].i;
            float dB_level = 20*(log10(mag_squared)/2-log10(1<<14));

            // Makes decibel values into positive values and blocking anything under THRESHOLD
            if(dB_level < THRESHOLD) dB_level = 0;
            else dB_level -= THRESHOLD;
            
            // Exponential moving average smoothing
            dB_level = past_dB_level[i]*anti_coeff+dB_level*coeff;
            past_dB_level[i] = dB_level;
        
            // Translates output data into column heights, which is entered into the buffer
            screenBuffer.writeBuffer[i] = 64*dB_level/(CAP-THRESHOLD);
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

/* Core 1 thread - run once */
void setup() {
    Serial.begin(115200);
    
    pinMode(CLIP_PIN, OUTPUT);
    pinMode(INPUT_PIN, INPUT);

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
    
    // Launches Task1code on core 0 with no parameters (accesses global variables)
    xTaskCreatePinnedToCore(Task1code, "Task1", 2500, NULL, 0, &Task1, 0);
}

/* Core 1 thread - run forever */
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