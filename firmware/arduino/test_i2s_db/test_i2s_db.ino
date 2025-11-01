#include <math.h>

#include <I2S.h>

// Pin definitions
#define LED_PIN LED_BUILTIN
#define I2S_DATA_PIN  20
#define I2S_BCLK_PIN  18  // Note that LRCLK is set to BCLK + 1

// Audio ettings
#define SERIAL_BAUD_RATE  115200
#define SAMPLE_RATE       16000
#define BITS_PER_SAMPLE   32
#define BUFFER_SIZE       256
#define NUM_BUFFERS       6

// dB calculation settings
#define UPDATE_INTERVAL_MS  500  // Calculate dB every 0.5 seconds
#define BUFFERS_PER_UPDATE  31   // ~500ms worth of buffers (500ms / 16ms per buffer)

// ICS-43434 sensitivity: -26 dBFS at 94 dB SPL
#define MIC_SENSITIVITY_DBFS  -26.0
#define MIC_REF_DB_SPL        94.0
#define MIC_OFFSET_DB         (MIC_REF_DB_SPL - MIC_SENSITIVITY_DBFS)  // = 120 dB

// Full scale reference for 32-bit samples
// ICS-43434 outputs 24-bit left-aligned in 32-bit = max value around 2^31
#define FULL_SCALE_32BIT  2147483648.0  // 2^31

// I2S input object
I2S i2s(INPUT);

// Buffer for processing
volatile bool bufferReady = false;
int32_t processingBuffer[BUFFER_SIZE * 2];  // *2 for stereo (L+R)

// RMS accumulation
uint64_t sumOfSquares = 0;
uint32_t samplesAccumulated = 0;
uint32_t buffersAccumulated = 0;
unsigned long lastUpdateTime = 0;

// ISR: runs when buffer is ready
void onI2SReceive() {
  bufferReady = true;
}

void setup() {
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);

  // Initialize serial and wait for connection
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  // Initialize I2S
  i2s.setDATA(I2S_DATA_PIN);
  i2s.setBCLK(I2S_BCLK_PIN);
  i2s.setBitsPerSample(BITS_PER_SAMPLE);
  i2s.setFrequency(SAMPLE_RATE);
  
  // Set up buffer configuration
  // setBuffers(num_buffers, buffer_size_in_words, silence_value)
  i2s.setBuffers(NUM_BUFFERS, BUFFER_SIZE, 0);
  
  // Register the callback BEFORE starting I2S
  i2s.onReceive(onI2SReceive);
  
  // Start I2S
  if (!i2s.begin()) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }
  
  Serial.println("I2S initialized successfully");
  Serial.printf("Sample rate: %d Hz\n", SAMPLE_RATE);
  Serial.printf("Buffer size: %d samples (%d ms)\n", BUFFER_SIZE, (BUFFER_SIZE * 1000) / SAMPLE_RATE);
  Serial.printf("Update interval: %d ms\n", UPDATE_INTERVAL_MS);
  Serial.printf("Buffers per update: %d\n", BUFFERS_PER_UPDATE);
  Serial.println("\nListening... (dB SPL readings every 0.5 seconds)");
  Serial.println("Time(ms), dBFS, dB SPL");
  
  lastUpdateTime = millis();
}

void loop() {
  // Check if a buffer is ready
  if (bufferReady) {
    bufferReady = false;
    
    // Read the buffer data
    size_t bytesRead = i2s.read((uint8_t*)processingBuffer, BUFFER_SIZE * 2 * sizeof(int32_t));
    
    // Accumulate sum of squares from left channel only
    for (int i = 0; i < BUFFER_SIZE * 2; i += 2) {  
      int64_t sample = processingBuffer[i];
      // Normalize to avoid overflow: divide by 256 before squaring
      int64_t normalized = sample >> 8;  // Divide by 256
      sumOfSquares += (normalized * normalized);
    }
    
    samplesAccumulated += BUFFER_SIZE;
    buffersAccumulated++;
    
    // Check if it's time to calculate and report dB
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
      
      if (samplesAccumulated > 0) {
        double meanSquare = (double)sumOfSquares / (double)samplesAccumulated;
        double rms = sqrt(meanSquare);
        
        // Account for the normalization (we divided by 256, so multiply back)
        rms = rms * 256.0;
        
        // Convert to dBFS
        double dbFS;
        if (rms > 0) {
          dbFS = 20.0 * log10(rms / FULL_SCALE_32BIT);
        } else {
          dbFS = -INFINITY;
        }
        
        // Convert to dB SPL using microphone sensitivity
        double dbSPL = dbFS + MIC_OFFSET_DB;
        
        // Print results
        Serial.printf("%lu, %.1f, %.1f\n", currentTime, dbFS, dbSPL);
        
        // Reset accumulators for next measurement period
        sumOfSquares = 0;
        samplesAccumulated = 0;
        buffersAccumulated = 0;
      }
      
      lastUpdateTime = currentTime;
    }
  }
}
