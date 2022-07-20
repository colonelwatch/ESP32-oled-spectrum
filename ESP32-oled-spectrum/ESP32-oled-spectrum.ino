// Copyright 2020 colonelwatch

#include <driver/i2s.h>
#include <Wire.h>
// #include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "buffer.h"
#include "kiss_fftr.h"
#include "cq_kernel.h"

/* User-configurable global constants */
// FFT settings
#define SAMPLES 6144        // Prime factorization should contain as many 2s as 
                            //  possible. Raise for less banding and lower for 
                            //  faster performance.
#define MAX_FREQUENCY 14000 // Hz, must be 1/2 of sampling frequency or less
#define MIN_FREQUENCY 40    // Hz, cannot be 0, decreasing causes banding
// Post-processing settings
#define CUTOFF 15.0f        // Helps determine where to cut low-value noise in
                            //  FFT output. Set as low as the noise will allow.
#define TIME_FACTOR 2.25     // Output smoothing factor (exponential moving 
                            //  average alpha), 1.0 for no smoothing
#define THRESHOLD 5         // dB, minimum display value
#define CAP 45              // dB, maximum display value
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
#define MINVAL 0.225f                 // used in cq_kernel

/* Other global constants, calculated from #define'd values */
const float coeff = 1./TIME_FACTOR;
const float anti_coeff = (TIME_FACTOR-1.)/TIME_FACTOR;
const int16_t minimum_mag = 2048*CUTOFF/SAMPLES; // in FFT value
const int32_t minimum_mag_squared = minimum_mag*minimum_mag;
struct cq_kernel_cfg cq_cfg = { // config for cq_kernel
    .samples = SAMPLES,
    .bands = COLUMNS,
    .fmin = MIN_FREQUENCY,
    .fmax = MAX_FREQUENCY,
    .fs = SAMPLING_FREQUENCY,
    .min_val = MINVAL
};
const i2s_config_t i2s_cfg = {
    .mode = (i2s_mode_t)( I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN ),
    .sample_rate = SAMPLING_FREQUENCY,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
};

/* Global variables */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 1000000UL);
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 16, 17, 5, 10000000UL);
cq_kernels_t kernels; // Will point to kernels allocated in dynamic memory
int frames; volatile int refresh; // Benchmarking variables

volatile bool screenBuffer_swap_ready = false;
doubleBuffer<uint8_t, COLUMNS> screenBuffer;
fftBuffer<float, SAMPLES> analogBuffer;

void screen_Task_routine(void *pvParameters){
    screenBuffer.alloc();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();

    delay(1000); // give time for the other tasks to allocate memory

    while(true){
        delay(1); // give time for the IDLE task (including watchdog)

        if(screenBuffer_swap_ready){
            screenBuffer.swap();
            screenBuffer_swap_ready = false;
        }

        display.clearDisplay();
        display.drawFastHLine(0, 63, 128, WHITE);
        const int col_px = 128/COLUMNS;
        for(int i = 0; i < COLUMNS; i++){
            int length = screenBuffer.readBuffer[i];
            display.fillRect(i*col_px-COLUMN_SIZE, 64-length, COLUMN_SIZE, length, WHITE);
        }
        display.display();
        
        refresh++;
    }
}

void comp_Task_routine(void *pvParameters){
    // Allocate some large arrays
    float *in = (float*)malloc(SAMPLES*sizeof(float));
    kiss_fft_cpx *out = (kiss_fft_cpx*)malloc(SAMPLES*sizeof(kiss_fft_cpx));
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);
    kiss_fft_cpx *bands_cpx = (kiss_fft_cpx*)malloc(COLUMNS*sizeof(kiss_fft_cpx));

    // Initialize I2S sampling
    analogBuffer.alloc();
    int16_t *samples_raw = (int16_t*)malloc(sizeof(int16_t)*512);
    float *samples = (float*)malloc(sizeof(float)*512);
    i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
    i2s_adc_enable(I2S_NUM_0);

    float *y_1 = (float*)calloc(COLUMNS, sizeof(float)),
          *x_1 = (float*)calloc(COLUMNS, sizeof(float)),
          *a_1 = (float*)calloc(COLUMNS, sizeof(float));

    delay(1000); // give time for the other tasks to allocate memory

    // Initalize benchmark
    unsigned long currentMillis;
    unsigned long timestamps[5] = {0}; // 0 means not recorded yet
    bool benchmark_started = false;
    bool benchmark_posted = false;

    while(true){
        // begin benchmarking over 5 seconds
        if(!benchmark_posted && !benchmark_started){
            refresh = 0;
            frames = 0;
            currentMillis = millis();
            benchmark_started = true;
        }
        
        if(!timestamps[0]) timestamps[0] = micros(); // run-once timestamping

        size_t bytes_read = 0;
        i2s_read(I2S_NUM_0, samples_raw, sizeof(int16_t)*308, &bytes_read, portMAX_DELAY); // blocking call down to ~142Hz
        int samples_read = bytes_read/sizeof(int16_t);
        for(int i = 0; i < samples_read; i++) samples[i] = (float)(samples_raw[i]-2048)*(1.0f/2048.0f);
        for(int i = 0; i < samples_read; i += 2){ // even and odd samples are switched for some reason
            float temp = samples[i];
            samples[i] = samples[i+1];
            samples[i+1] = temp;
        }
        analogBuffer.write(samples, samples_read);
        
        // Reads ENTIRE analogBuffer starting from analogBuffer.write_index, 
        //  this means a lot of overlap, but it decouples FFT calculations from 
        //  sampling
        float sum = 0;
        analogBuffer.read(in);
        for(int i = 0; i < SAMPLES; i++) sum += in[i];
        
        float avg = sum*(1.0f/SAMPLES);
        for(int i = 0; i < SAMPLES; i++){
            in[i] -= avg;
            out[i] = (kiss_fft_cpx){0, 0}; // necessary before calling kiss_fftr
        }

        if(!timestamps[1]) timestamps[1] = micros();
      
        kiss_fftr(cfg, in, out);

        if(!timestamps[2]) timestamps[2] = micros();
        
        // Cutting off noise with a threshold inversely proportional to SAMPLES
        for(int i = 0; i < SAMPLES; i++)
            if(out[i].r*out[i].r+out[i].i*out[i].i < minimum_mag_squared)
                out[i] = (kiss_fft_cpx){0, 0};

        // Convert FFT output to Constant Q output using cq_kernel
        for(int i = 0; i < COLUMNS; i++) bands_cpx[i] = (kiss_fft_cpx){0, 0};
        apply_kernels(out, bands_cpx, kernels, cq_cfg);

        if(!timestamps[3]) timestamps[3] = micros();

        for(int i = 0; i < COLUMNS; i++){
            // Finds decibel value of complex magnitude (relative to 1<<14, apparent maximum)
            float mag_squared = bands_cpx[i].r*bands_cpx[i].r+bands_cpx[i].i*bands_cpx[i].i;
            float x = 10.0f*log10(mag_squared); // decibel level, reference is arbitrary

            // Makes decibel values into positive values and blocking anything under THRESHOLD
            if(x < THRESHOLD) x = 0;
            else x -= THRESHOLD;
            
            float a = a_1[i]*anti_coeff+x*(coeff*0.5f)+x_1[i]*(coeff*0.5f);
            float y = y_1[i]*anti_coeff+a*(coeff*0.5f)+a_1[i]*(coeff*0.5f);
            y_1[i] = y;
            x_1[i] = x;
            a_1[i] = a;
        
            // Translates output data into column heights, which is entered into the buffer
            screenBuffer.writeBuffer[i] = 64*y*(1.0f/(CAP-THRESHOLD));
        }
        screenBuffer_swap_ready = true;   // Raises flag to indicate buffer is ready to push

        if(!timestamps[4]) timestamps[4] = micros();

        // Outputs benchmark data
        frames++;
        if(millis()-currentMillis > 5000 && benchmark_started && !benchmark_posted){
            for(int i = 1; i < 5; i++){
                Serial.print(timestamps[i]-timestamps[i-1]);
                Serial.print(' ');
            }
            Serial.println();

            Serial.print(frames/5);
            Serial.print(' ');
            Serial.println(refresh/5);

            benchmark_started = false;
            benchmark_posted = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(CLIP_PIN, OUTPUT);
    pinMode(INPUT_PIN, INPUT);
    
    // Generate kernels (memory-intensive!) before starting any other tasks
    kernels = generate_kernels(cq_cfg);
    kernels = reallocate_kernels(kernels, cq_cfg);
    
    // Launches comp_Task_routine on core 0 with no parameters (accesses global variables)
    xTaskCreatePinnedToCore(screen_Task_routine, "screen", 2500, NULL, configMAX_PRIORITIES-1, new TaskHandle_t, 0);
    xTaskCreatePinnedToCore(comp_Task_routine, "comp", 2500, NULL, configMAX_PRIORITIES-1, new TaskHandle_t, 1);
}

void loop() {
    vTaskSuspend(NULL); // suspend the arduino loop
}