#define BTN_PIN         24
#define DEBOUNCE_DELAY  50
#define PRINT_DELAY     1000

bool buttonState;
bool lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;
unsigned int count = 0;
unsigned long lastPrintTime = 0;

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  bool buttonReading = digitalRead(BTN_PIN);
  
  // Check if button state changed
  if (buttonReading != lastButtonReading) {
    lastDebounceTime = millis();
  }
  
  // If enough time has passed (debounce)
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (buttonReading != buttonState) {
      buttonState = buttonReading;
      if (buttonState == LOW) {
        count++;
        Serial.println(count);
      } else {
        Serial.println("released");
      }
    }
  }
  
  lastButtonReading = buttonReading;

  if ((millis() - lastPrintTime) > PRINT_DELAY) {
    lastPrintTime = millis();
    Serial.println("test");
  }
}