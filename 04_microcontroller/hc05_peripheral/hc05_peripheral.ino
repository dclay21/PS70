#include <Arduino.h>

// HC-05 on Serial1 (data-mode)
HardwareSerial btSerialPeripheral(1);
#define RX_S 3    // HC-05 TX → ESP RX
#define TX_S 2    // HC-05 RX ← ESP TX

// I/O
const int buttonPin = 8;
const int ledPin    = 9;

// Debounce
int lastButtonReading    = HIGH;
int debouncedState       = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(9600);
  // while (!Serial) {}

  // Start HC-05 in data mode at its default 9600 baud
  btSerialPeripheral.begin(9600, SERIAL_8N1, RX_S, TX_S);

  pinMode(buttonPin, INPUT_PULLUP);  // button → GND
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.println("Peripheral ready");
}

void loop() {
  // 1) Check for incoming "ON" command
  if (btSerialPeripheral.available()) {
    String cmd = btSerialPeripheral.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("ON-C")) {
      digitalWrite(ledPin, HIGH);
      Serial.println("Received ON, turning LED on...");
    } else if (cmd.equalsIgnoreCase("OFF-C")) {
      digitalWrite(ledPin, LOW);
      Serial.println("Turning LED off");
    }
  }

  // 2) Read & debounce button
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > debounceDelay) {
    if (reading != debouncedState) {
      debouncedState = reading;
      // button is pressed when pulled LOW
      if (debouncedState == LOW) {
        Serial.println("Button pressed → sending ON");
        btSerialPeripheral.println("ON-P");
      } else {
        Serial.println("Not pressed, sending OFF");
        btSerialPeripheral.println("OFF-P");
      }
    }
  }
  lastButtonReading = reading;
    // 2) Raw button read
//   int state = digitalRead(buttonPin);
//   Serial.print("Button state: ");
//   Serial.println(state);  // should print 1 when released, 0 when pressed

//   if (state == LOW) {
//     Serial.println("!! Button pressed → sending ON");
//     btSerialPeripheral.println("ON-P");
//     delay(200);  // simple bounce‐filter and rate‐limiter
//   }

//   delay(50);
}
