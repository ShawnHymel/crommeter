/*
   Timer-based 7-segment display multiplexing
   Updates display 60 times per second using hardware timer ISR
   Double buffer prevents display artifacts during updates
*/

#include <hardware/timer.h>
#include <hardware/irq.h>

// 7-segment display pin definitions
// Adjust these to match your actual wiring
#define SEG_A    3
#define SEG_B    4
#define SEG_C    7
#define SEG_D    8
#define SEG_E    5
#define SEG_F    2
#define SEG_G    6
#define SEG_DP   9  // Decimal point (optional)

// Digit select pins (common cathode or anode)
#define DIGIT_1  10
#define DIGIT_2  12
#define DIGIT_3  11

// Timer configuration
#define DISPLAY_REFRESH_RATE  60   // Hz - full display refresh rate
#define DIGIT_UPDATE_RATE     (DISPLAY_REFRESH_RATE * 3)  // 60 Hz per digit

// 7-segment digit encoding (common cathode, adjust for common anode)
// Segments: ABCDEFG (bit 7=DP, bits 6-0 = GFEDCBA)
const uint8_t DIGIT_PATTERNS[10] = {
  0b00111111,  // 0
  0b00000110,  // 1
  0b01011011,  // 2
  0b01001111,  // 3
  0b01100110,  // 4
  0b01101101,  // 5
  0b01111101,  // 6
  0b00000111,  // 7
  0b01111111,  // 8
  0b01101111   // 9
};

// Double buffer for display data
volatile uint8_t displayBuffer[2][3];  // [buffer][digit]
volatile uint8_t activeBuffer = 0;     // Which buffer ISR is reading from
volatile uint8_t writeBuffer = 1;      // Which buffer main code writes to

// ISR state
volatile uint8_t currentDigit = 0;     // Which digit (0-2) is currently lit

// Timer alarm handler
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

// Hardware timer for display refresh
struct repeating_timer displayTimer;

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

void updateDisplay(int value) {
  // Clamp value to 0-999
  if (value < 0) value = 0;
  if (value > 999) value = 999;
  
  // Extract individual digits
  uint8_t digit0 = (value / 100) % 10;      // Hundreds
  uint8_t digit1 = (value / 10) % 10;       // Tens
  uint8_t digit2 = value % 10;              // Ones

  Serial.printf("%d: %d, %d, %d\r\n", value, digit0, digit1, digit2);
  
  // Write to the inactive buffer
  displayBuffer[writeBuffer][0] = DIGIT_PATTERNS[digit0];
  displayBuffer[writeBuffer][1] = DIGIT_PATTERNS[digit1];
  displayBuffer[writeBuffer][2] = DIGIT_PATTERNS[digit2];
  
  // Swap buffers atomically
  noInterrupts();
  uint8_t temp = activeBuffer;
  activeBuffer = writeBuffer;
  writeBuffer = temp;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  Serial.println("7-Segment Display with Timer ISR");
  
  setupDisplay();
  
  Serial.println("Display initialized");
}

void loop() {
  // Example: count from 0 to 999
  static int counter = 0;
  
  updateDisplay(counter);
  
  counter++;
  if (counter > 999) {
    counter = 0;
  }
  
  delay(100);  // Update display value 10 times per second
}