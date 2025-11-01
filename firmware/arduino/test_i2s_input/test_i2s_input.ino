#include <I2S.h>

// Pin definitions
#define LED_PIN LED_BUILTIN
#define I2S_DATA_PIN  20
#define I2S_BCLK_PIN  18  // Note that LRCLK is set to BCLK + 1

// Audio configuration
#define SAMPLE_RATE   16000
#define BITS_PER_SAMPLE 32
#define CAPTURE_SECONDS 2

// Calculate buffer size (samples per channel)
#define TOTAL_SAMPLES (SAMPLE_RATE * CAPTURE_SECONDS)

// I2S input object
I2S i2s(INPUT);

void setup() {

  // Buffer to store captured audio
  int32_t audioBuffer[TOTAL_SAMPLES];
  int bufferIndex = 0;
  bool captureComplete = false;

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);

  // Initialize serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for serial connection
  }

  // Initialize I2S
  i2s.setDATA(I2S_DATA_PIN);
  i2s.setBCLK(I2S_BCLK_PIN);
  i2s.setBitsPerSample(BITS_PER_SAMPLE);
  i2s.setFrequency(SAMPLE_RATE);

  // Start I2S
  if (!i2s.begin()) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  Serial.println("I2S initialized successfully");
  Serial.println("Starting audio capture in 1 second...");
  delay(1000);

  // Show that we're starting to collect data
  Serial.println("Capturing audio...");
  unsigned long startTime = millis();
  digitalWrite(LED_PIN, HIGH);
  
  // Capture audio data
  while (bufferIndex < TOTAL_SAMPLES) {
    int32_t left, right;
    
    // Read stereo sample (even though mic is mono on left channel)
    if (i2s.read32(&left, &right)) {
      // Store only the left channel (where our mono mic is connected)
      audioBuffer[bufferIndex] = left;
      bufferIndex++;
    }
  }

  // Done collection
  unsigned long endTime = millis();
  captureComplete = true;
  digitalWrite(LED_PIN, LOW);
  Serial.printf("Capture complete! Took %lu ms\n", endTime - startTime);
  Serial.println();
  
  // Print captured data
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    Serial.printf("%d", audioBuffer[i]);
    if (i < TOTAL_SAMPLES - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();
}

void loop() {
  // Do nothing
}
