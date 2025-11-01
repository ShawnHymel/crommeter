#include <I2S.h>

// Pin definitions
#define LED_PIN LED_BUILTIN
#define I2S_DATA_PIN  20
#define I2S_BCLK_PIN  18  // Note that LRCLK is set to BCLK + 1

// Settings
#define SERIAL_BAUD_RATE  115200
#define SAMPLE_RATE       16000
#define BITS_PER_SAMPLE   32
#define BUFFER_SIZE       256
#define NUM_BUFFERS       6

// I2S input object
I2S i2s(INPUT);

// Buffer for processing
volatile bool bufferReady = false;
int32_t processingBuffer[BUFFER_SIZE * 2];  // *2 for stereo (L+R)
volatile int buffersReceived = 0;

// ISR: runs when buffer is ready
void onI2SReceive() {
  bufferReady = true;
  buffersReceived++;
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
  
  Serial.println("I2S initialized with interrupt callback");
  Serial.printf("Buffer size: %d samples\n", BUFFER_SIZE);
  Serial.printf("Sample rate: %d Hz\n", SAMPLE_RATE);
  Serial.printf("Buffer duration: %.2f ms\n", (BUFFER_SIZE * 1000.0) / SAMPLE_RATE);
  Serial.println("\nListening for audio...");
}

void loop() {
  // Check if a buffer is ready (set by interrupt)
  if (bufferReady) {
    bufferReady = false;
    
    // Read the buffer data
    // Note: read() returns number of bytes read / 4
    size_t samplesRead = i2s.read((uint8_t*)processingBuffer, BUFFER_SIZE * 2 * sizeof(int32_t));
    
    // Process the buffer here
    // For now, just print some info every 10 buffers
    if (buffersReceived % 10 == 0) {
      // Calculate some basic stats from left channel only
      int32_t minVal = processingBuffer[0];
      int32_t maxVal = processingBuffer[0];
      int64_t sum = 0;
      
      for (int i = 0; i < BUFFER_SIZE * 2; i += 2) {  // Step by 2 to get only left channel
        int32_t sample = processingBuffer[i];
        if (sample < minVal) minVal = sample;
        if (sample > maxVal) maxVal = sample;
        sum += abs(sample);
      }
      
      int32_t avgAbs = sum / BUFFER_SIZE;
      
      Serial.printf("Buffer #%d - Min: %d, Max: %d, Avg(abs): %d\n", 
                    buffersReceived, minVal, maxVal, avgAbs);
    }
  }
  
  // Add a small delay to prevent tight looping
  delay(1);
}
