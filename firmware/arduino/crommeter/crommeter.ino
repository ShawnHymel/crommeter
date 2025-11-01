#include <math.h>

#include <hardware/timer.h>
#include <hardware/irq.h>
#include <I2S.h>

/******************************************************************************
 * Pins
 */

// Toggle debug
#define DEBUG_ENABLE  1

// LED and button
#define LED_PIN       29
#define BTN_PIN       24

// I2S pins
#define I2S_DATA_PIN  20
#define I2S_BCLK_PIN  18  // Note that LRCLK is set to BCLK + 1

// 7-segment display pins
#define SEG_A         3
#define SEG_B         4
#define SEG_C         7
#define SEG_D         8
#define SEG_E         5
#define SEG_F         2
#define SEG_G         6
#define SEG_DP        9
#define DIGIT_1       10
#define DIGIT_2       12
#define DIGIT_3       11

// Eye LED pins
#define EYE_L_PIN     17
#define EYE_R_PIN     16

/******************************************************************************
 * Settings
 */

// Audio settings
#define SERIAL_BAUD_RATE  115200
#define SAMPLE_RATE       16000
#define BITS_PER_SAMPLE   32
#define BUFFER_SIZE       256
#define NUM_BUFFERS       6

// dB calculation settings
#define UPDATE_INTERVAL_MS  200  // Calculate dB every 0.5 seconds
#define BUFFERS_PER_UPDATE  13   // ~500ms worth of buffers (500ms / 16ms per buffer)

// dB range for LED brightness calculation
#define MIN_DB 40.0
#define MAX_DB 120.0

// ICS-43434 sensitivity: -26 dBFS at 94 dB SPL
#define MIC_SENSITIVITY_DBFS  -26.0
#define MIC_REF_DB_SPL        94.0
#define MIC_OFFSET_DB         (MIC_REF_DB_SPL - MIC_SENSITIVITY_DBFS)  // = 120 dB

// Full scale reference for 32-bit samples
// ICS-43434 outputs 24-bit left-aligned in 32-bit = max value around 2^31
#define FULL_SCALE_32BIT  2147483648.0  // 2^31

// Timer configuration
#define DISPLAY_REFRESH_RATE  60   // Hz - full display refresh rate
#define DIGIT_UPDATE_RATE     (DISPLAY_REFRESH_RATE * 3)  // 60 Hz per digit

// Peak recording mode
#define DEBOUNCE_DELAY_MS     50
#define PEAK_DISPLAY_HOLD_MS  2000  // How long (ms) to hold display with peak value

/******************************************************************************
 * Macros
 */

#if DEBUG_ENABLE
  #define DEBUG_BEGIN(baud)     Serial.begin(baud)
  #define DEBUG(...)            Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(baud)     // Do nothing
  #define DEBUG(...)            // Do nothing
#endif

/******************************************************************************
 * Globals
 */

// 7-segment digit encoding (common cathode, adjust for common anode)
// Segments: ABCDEFG (bit 7=DP, bits 6-0 = GFEDCBA)
const uint8_t DIGIT_PATTERNS[11] = {
  0b00111111,  // 0
  0b00000110,  // 1
  0b01011011,  // 2
  0b01001111,  // 3
  0b01100110,  // 4
  0b01101101,  // 5
  0b01111101,  // 6
  0b00000111,  // 7
  0b01111111,  // 8
  0b01101111,  // 9
  0b01000000   // -
};

// Double buffer for display data
volatile uint8_t displayBuffer[2][3];  // [buffer][digit]
volatile uint8_t activeBuffer = 0;     // Which buffer ISR is reading from
volatile uint8_t writeBuffer = 1;      // Which buffer main code writes to

// Hardware timer for display refresh
struct repeating_timer displayTimer;

// ISR state
volatile uint8_t currentDigit = 0;     // Which digit (0-2) is currently lit

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

// Peak recording mode
double peakDbSPL = 0.0;

// Debounce
bool buttonState;
bool lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;

// Display time
unsigned long displayTime = 0;

// State
unsigned int mode = 0;

/******************************************************************************
 * Interrupt service routines
 */

// Notify that I2S buffer is ready
void onI2SReceive() {
  bufferReady = true;
}

// 7-segment display driver
bool __isr __time_critical_func(displayTimerCallback)(struct repeating_timer *t) {
  // Turn off all digits first to prevent ghosting
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(DIGIT_3, LOW);
  
  // Get the pattern for current digit from active buffer
  uint8_t pattern = displayBuffer[activeBuffer][currentDigit];
  
  // Output the segment pattern
  digitalWrite(SEG_A, (pattern & 0b00000001) ? HIGH : LOW);
  digitalWrite(SEG_B, (pattern & 0b00000010) ? HIGH : LOW);
  digitalWrite(SEG_C, (pattern & 0b00000100) ? HIGH : LOW);
  digitalWrite(SEG_D, (pattern & 0b00001000) ? HIGH : LOW);
  digitalWrite(SEG_E, (pattern & 0b00010000) ? HIGH : LOW);
  digitalWrite(SEG_F, (pattern & 0b00100000) ? HIGH : LOW);
  digitalWrite(SEG_G, (pattern & 0b01000000) ? HIGH : LOW);
  // digitalWrite(SEG_DP, (pattern & 0b10000000) ? HIGH : LOW);  // Optional
  
  // Turn on the current digit
  switch(currentDigit) {
    case 0: digitalWrite(DIGIT_1, HIGH); break;
    case 1: digitalWrite(DIGIT_2, HIGH); break;
    case 2: digitalWrite(DIGIT_3, HIGH); break;
  }
  
  // Move to next digit
  currentDigit++;
  if (currentDigit >= 3) {
    currentDigit = 0;
  }
  
  return true;  // Keep the timer running
}

/******************************************************************************
 * Functions
 */

void setupDisplay() {
  // Configure segment pins as outputs
  pinMode(SEG_A, OUTPUT);
  pinMode(SEG_B, OUTPUT);
  pinMode(SEG_C, OUTPUT);
  pinMode(SEG_D, OUTPUT);
  pinMode(SEG_E, OUTPUT);
  pinMode(SEG_F, OUTPUT);
  pinMode(SEG_G, OUTPUT);
  pinMode(SEG_DP, OUTPUT);
  
  // Configure digit select pins as outputs
  pinMode(DIGIT_1, OUTPUT);
  pinMode(DIGIT_2, OUTPUT);
  pinMode(DIGIT_3, OUTPUT);
  
  // Turn off all digits initially
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(DIGIT_3, LOW);
  
  // Initialize buffers to show "000"
  for (int buf = 0; buf < 2; buf++) {
    for (int dig = 0; dig < 3; dig++) {
      displayBuffer[buf][dig] = DIGIT_PATTERNS[0];
    }
  }
  
  // Start the timer - fires every 1000000/180 = ~5555 microseconds
  int64_t intervalUs = 1000000 / DIGIT_UPDATE_RATE;
  add_repeating_timer_us(-intervalUs, displayTimerCallback, NULL, &displayTimer);
}

void swapBuffers() {
  // Swap buffers atomically
  noInterrupts();
  uint8_t temp = activeBuffer;
  activeBuffer = writeBuffer;
  writeBuffer = temp;
  interrupts();
}

void updateDisplay(int value) {
  // Clamp value to 0-999
  if (value < 0) value = 0;
  if (value > 999) value = 999;
  
  // Extract individual digits
  uint8_t digit0 = (value / 100) % 10;      // Hundreds
  uint8_t digit1 = (value / 10) % 10;       // Tens
  uint8_t digit2 = value % 10;              // Ones
  
  // Write to the inactive buffer
  displayBuffer[writeBuffer][0] = DIGIT_PATTERNS[digit0];
  displayBuffer[writeBuffer][1] = DIGIT_PATTERNS[digit1];
  displayBuffer[writeBuffer][2] = DIGIT_PATTERNS[digit2];
  
  swapBuffers();
}

void displayDashes() {
  // Write dashes to the inactive buffer
  displayBuffer[writeBuffer][0] = DIGIT_PATTERNS[10];
  displayBuffer[writeBuffer][1] = DIGIT_PATTERNS[10];
  displayBuffer[writeBuffer][2] = DIGIT_PATTERNS[10];
  
  swapBuffers();
}

uint8_t dbToPWM(double dbSPL) {  
  // Clamp input
  if (dbSPL < MIN_DB) dbSPL = MIN_DB;
  if (dbSPL > MAX_DB) dbSPL = MAX_DB;
  
  // Linear mapping in dB space
  double normalized = (dbSPL - MIN_DB) / (MAX_DB - MIN_DB);
  
  // Exponential brightness curve: y = e^(x*k) - 1 / (e^k - 1)
  // This gives more dynamic range at lower sound levels
  const double k = 3.0;  // Adjust for more/less emphasis on low levels
  double corrected = (exp(normalized * k) - 1.0) / (exp(k) - 1.0);
  
  // Scale to PWM range
  uint8_t pwmValue = (uint8_t)(corrected * 255.0);
  
  return pwmValue;
}

/******************************************************************************
 * Main
 */

void setup() {
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);

  // Initialize eyes
  pinMode(EYE_L_PIN, OUTPUT);
  pinMode(EYE_R_PIN, OUTPUT);

  // Setup button with pullup and interrupt
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Initialize serial and wait for connection
  DEBUG_BEGIN(115200);

  // Initialize 7-segment display
  setupDisplay();
  
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
    DEBUG("Failed to initialize I2S!");
    while (1);
  }
  
  DEBUG("I2S initialized successfully");
  DEBUG("Sample rate: %d Hz\n", SAMPLE_RATE);
  DEBUG("Buffer size: %d samples (%d ms)\n", BUFFER_SIZE, (BUFFER_SIZE * 1000) / SAMPLE_RATE);
  DEBUG("Update interval: %d ms\n", UPDATE_INTERVAL_MS);
  DEBUG("Buffers per update: %d\n", BUFFERS_PER_UPDATE);
  DEBUG("\nListening... (dB SPL readings every 0.5 seconds)");
  DEBUG("Time(ms), dBFS, dB SPL");
 
  lastUpdateTime = millis();
}

void loop() {
  bool buttonReading = digitalRead(BTN_PIN);
  
  // Check if button state changed
  if (buttonReading != lastButtonReading) {
    lastDebounceTime = millis();
  }
  
  // If enough time has passed (debounce)
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (buttonReading != buttonState) {
      buttonState = buttonReading;
      if (buttonState == LOW) {
        DEBUG("pressed\r\n");
        mode = 1;
      } else {
        DEBUG("released\r\n");
        displayTime = millis();
        mode = 2;
      }
    }
  }
  
  // Store previous button reading
  lastButtonReading = buttonReading;

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

        // Track peak in recording mode
        if (mode == 1) {
          if (dbSPL > peakDbSPL) {
            peakDbSPL = dbSPL;
          }
        }

        // Round to nearest dB
        int disp_dbSPL = (int)round(dbSPL);

        // Convert SPL to LED brightness level
        uint8_t brightness = dbToPWM(dbSPL);

        // Display based on mode
        switch (mode) {
          case 0:
            updateDisplay(disp_dbSPL);
            analogWrite(EYE_L_PIN, brightness);
            analogWrite(EYE_R_PIN, brightness);
            break;
          case 1:
            displayDashes();
            analogWrite(EYE_L_PIN, brightness);
            analogWrite(EYE_R_PIN, brightness);
            break;
          case 2:
            if ((millis() - displayTime) > PEAK_DISPLAY_HOLD_MS) {
              DEBUG("Max SPL: %.1f\r\n", peakDbSPL);
              peakDbSPL = 0;
              mode = 0;
            }
            updateDisplay(peakDbSPL);
            analogWrite(EYE_L_PIN, 0);
            analogWrite(EYE_R_PIN, 0);
            break;
          default:
            break;
        }

        // Print results
        DEBUG("Mode: %u, PWM: %u, dB SPL: %.1f\r\n", mode, brightness, dbSPL);
        
        // Reset accumulators for next measurement period
        sumOfSquares = 0;
        samplesAccumulated = 0;
        buffersAccumulated = 0;
      }
      
      lastUpdateTime = currentTime;
    }
  }
}
