// Copyright 2020 colonelwatch

#include <driver/i2s.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // includes references to both Wire.h and Spi.h

#include "buffer.h"
#include "kiss_fftr.h"
#include "cq_kernel.h"

// End-user constants, adjust depending on your electrical configuration
const int dB_min = 5; // dB, minimum value to display
const int dB_max = 45; // dB, maximum value to display
const int clip_pin = 19; // Connect LED to this pin to get a clipping indicator (TODO: reimplement)
const adc1_channel_t adc_channel = ADC1_CHANNEL_0; // Connect DC-biased line signal to this, see IDF docs for pin nums
const float fft_mag_cutoff = 15.0; // factor used for cutting off noise in raw spectrum, raise if noise is in the output
// #define SPI_SSD1306 // Uncomment if using a SPI SSD1306 OLED, also injects an interp routine for 3x the "frame rate"

// Critical constants, not intended for end-user modification
const int N_samples = 6144; // FFT length, prime factorication should contain as many 2s as possible, affects performance
const int sampling_frequency = 44100; // Hz, I2S sampling frequency
const int max_freq = 14000; // Hz, last CQT center freq to display, ensure CQT kernels aren't degenerated when changing
const int min_freq = 40; // Hz, first CQT center freq to display, ensure CQT kernels aren't degenerated when changing
const float min_val = 0.225; // see Brown CQT paper for explanation
const int N_columns = 64; // number of columns to display
const int col_width = 1; // px, width of each column
const int screen_width = 128; // px, width of screen
const int screen_height = 64; // px, height of screen

// global variables, accessed during execution
struct cq_kernel_cfg cq_cfg = { // accessed before all other tasks are started, so its global
    .samples = N_samples,
    .bands = N_columns,
    .fmin = min_freq,
    .fmax = max_freq,
    .fs = sampling_frequency,
    .min_val = min_val
};
cq_kernels_t kernels; // will point to kernels allocated in dynamic memory
int frames; volatile int refresh; // benchmarking variables
fftBuffer<float, N_samples> analogBuffer; // for samples
volatile bool colBuffer_swap_ready = false; doubleBuffer<float, N_columns> colBuffer; // for CQT out before post-processing
#ifdef SPI_SSD1306
Adafruit_SSD1306 display(screen_width, screen_height, &SPI, 16, 17, 5, 10000000UL);
#else
Adafruit_SSD1306 display(screen_width, screen_height, &Wire, -1, 1000000UL);
#endif

void screen_Task_routine(void *pvParameters){
    colBuffer.alloc();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();
    
    unsigned long currentMicros = micros();
    float   *y = (float*)calloc(N_columns, sizeof(float)),
            *y_1 = (float*)calloc(N_columns, sizeof(float)),
            *y_2 = (float*)calloc(N_columns, sizeof(float)),
            *x_1 = (float*)calloc(N_columns, sizeof(float)),
            *x_2 = (float*)calloc(N_columns, sizeof(float));
    #ifdef SPI_SSD1306
    bool impulse = false;
    #endif

    delay(1000); // give time for the other tasks to allocate memory

    while(true){
        #ifdef SPI_SSD1306
        while(micros()-currentMicros < 1000000/(142*3)); // precise spin-waiting
        currentMicros = micros();
        if(colBuffer_swap_ready){
            colBuffer.swap();
            colBuffer_swap_ready = false;
            impulse = true;
        }
        #else
        while(micros()-currentMicros < 1000000/89); // precise, slower spin-waiting
        currentMicros = micros();
        if(colBuffer_swap_ready){
            colBuffer.swap();
            colBuffer_swap_ready = false;
        }
        #endif

        for(int i = 0; i < N_columns; i++){
            float x = colBuffer.readBuffer[i]-dB_min;
            if(x < 0) x = 0;
            #ifdef SPI_SSD1306
            // upsampling by inserting zeros then filtering, rather than holding x for extended time 
            //  (called zero-order hold) then filtering
            if(impulse) x *= 3;
            else x = 0;
            // 2nd-order Butterworth IIR with cutoff at 20Hz (426Hz "sampling") as an interpolator
            y[i] = 0.004917646918866*x+0.009835293837732*x_1[i]+0.004917646918866*x_2[i] \
                +1.792062605350460*y_1[i]-0.811733193025923*y_2[i];
            #else
            // 2nd-order Butterworth IIR with cutoff at 10Hz (89Hz "sampling") as a filter
            //  Lower cutoff because filter failed to kill 60Hz cycles otherwise (appears as ghosting)
            y[i] = 0.081926471866054*x+0.163852943732109*x_1[i]+0.081926471866054*x_2[i] \
                +1.043326781704508*y_1[i]-0.371032669168726*y_2[i];
            #endif

            x_2[i] = x_1[i];
            x_1[i] = x;
            y_2[i] = y_1[i];
            y_1[i] = y[i];
        }

        #ifdef SPI_SSD1306
        impulse = false;
        #endif

        display.clearDisplay();
        display.drawFastHLine(0, 63, 128, WHITE);
        const int col_px = 128/N_columns;
        for(int i = 0; i < N_columns; i++){
            int length = y[i]*(64.0f/(dB_max-dB_min));
            display.fillRect(i*col_px-col_width, 64-length, col_width, length, WHITE);
        }
        display.display();
        
        refresh++;
    }
}

void comp_Task_routine(void *pvParameters){
    // Allocate some large arrays
    float *in = (float*)malloc(N_samples*sizeof(float));
    kiss_fft_cpx *out = (kiss_fft_cpx*)malloc(N_samples*sizeof(kiss_fft_cpx));
    kiss_fftr_cfg cfg = kiss_fftr_alloc(N_samples, 0, NULL, NULL);
    kiss_fft_cpx *bands_cpx = (kiss_fft_cpx*)malloc(N_columns*sizeof(kiss_fft_cpx));

    // Initialize I2S sampling
    const i2s_config_t i2s_cfg = {
        .mode = (i2s_mode_t)( I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN ),
        .sample_rate = sampling_frequency,
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
    analogBuffer.alloc();
    int16_t *samples_raw = (int16_t*)malloc(sizeof(int16_t)*512);
    float *samples = (float*)malloc(sizeof(float)*512);
    i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
    i2s_adc_enable(I2S_NUM_0);

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
        analogBuffer.write(samples, samples_read); // write only 308 samples to the buffer...
        
        float sum = 0, avg;
        analogBuffer.read(in); // ...and read the past N_samples out!
        for(int i = 0; i < N_samples; i++) sum += in[i];
        avg = sum*(1.0f/N_samples);
        for(int i = 0; i < N_samples; i++){
            in[i] -= avg;
            out[i] = (kiss_fft_cpx){0, 0}; // necessary before calling kiss_fftr
        }

        if(!timestamps[1]) timestamps[1] = micros();
      
        kiss_fftr(cfg, in, out);

        if(!timestamps[2]) timestamps[2] = micros();
        
        // Cutting off noise with a threshold inversely proportional to N_samples
        const float minimum_mag = 2048*fft_mag_cutoff/N_samples;
        const float minimum_mag_squared = minimum_mag*minimum_mag;
        for(int i = 0; i < N_samples; i++)
            if(out[i].r*out[i].r+out[i].i*out[i].i < minimum_mag_squared)
                out[i] = (kiss_fft_cpx){0, 0};

        // Convert FFT output to Constant Q output using cq_kernel
        for(int i = 0; i < N_columns; i++) bands_cpx[i] = (kiss_fft_cpx){0, 0};
        apply_kernels(out, bands_cpx, kernels, cq_cfg);

        if(!timestamps[3]) timestamps[3] = micros();

        for(int i = 0; i < N_columns; i++){
            // Finds decibel value of complex magnitude (relative to 1<<14, apparent maximum)
            float mag_squared = bands_cpx[i].r*bands_cpx[i].r+bands_cpx[i].i*bands_cpx[i].i;
            float x = 10.0f*log10(mag_squared); // dB, (squared in ==> 10*log10, not 20*log10), reference level is arbitrary
            colBuffer.writeBuffer[i] = x;
        }
        colBuffer_swap_ready = true;   // Raises flag to indicate buffer is ready to push

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
    
    pinMode(clip_pin, OUTPUT);
    
    // Generate kernels (memory-intensive!) before starting any other tasks
    kernels = generate_kernels(cq_cfg);
    kernels = reallocate_kernels(kernels, cq_cfg);

    disableCore0WDT(); // disable the watchdog in order to let screen_Task_routine spin-wait
    
    xTaskCreatePinnedToCore(screen_Task_routine, "screen", 2500, NULL, configMAX_PRIORITIES-1, new TaskHandle_t, 0);
    xTaskCreatePinnedToCore(comp_Task_routine, "comp", 2500, NULL, configMAX_PRIORITIES-1, new TaskHandle_t, 1);
}

void loop() {
    vTaskSuspend(NULL); // suspend the arduino loop
}