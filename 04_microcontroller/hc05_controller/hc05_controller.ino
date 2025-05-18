#include <Arduino.h>

//=== HC-05 on Serial1 configuration ===
HardwareSerial btSerialController(1);
#define RX_M 3    // HC-05 TX → ESP RX
#define TX_M 2    // HC-05 RX ← ESP TX

//=== I/O pins ===
const int buttonPin = 8;
const int ledPin    = 9;

//=== Debounce state ===
int     lastButtonReading = HIGH;
int     debouncedState    = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // ms

void configureHC05() {
  delay(500);
  Serial.println("Entering AT mode...");
  btSerialController.print("AT\r\n");
  delay(500); readResponse();

  Serial.println("Set role = Master");
  btSerialController.print("AT+ROLE=1\r\n");
  delay(500); readResponse();

  Serial.println("Set connection mode = fixed address");
  btSerialController.print("AT+CMODE=0\r\n");
  delay(500); readResponse();

  Serial.println("Bind to target address");
  btSerialController.print("AT+BIND=0025,00,0011DF\r\n");
  delay(500); readResponse();

  Serial.println("Initialize SPP");
  btSerialController.print("AT+INIT\r\n");
  delay(500); readResponse();

  Serial.println("Link to target");
  btSerialController.print("AT+LINK=0025,00,0011DF\r\n");
  delay(500); readResponse();

  Serial.println("HC-05 configured.");
}

void readResponse() {
  while (btSerialController.available()) {
    Serial.write(btSerialController.read());
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  // while (!Serial) { }

  // start HC-05 at AT-mode baud
  btSerialController.begin(9600, SERIAL_8N1, RX_M, TX_M);

  // I/O
  pinMode(buttonPin, INPUT_PULLUP);  // wiring: button → GND
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // configure HC-05 once
  // configureHC05();
}

void loop() {
  // 1) Check for incoming BT command
  if (btSerialController.available()) {
    String cmd = btSerialController.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("ON-P")) {
      digitalWrite(ledPin, HIGH);
      Serial.println("turning controller LED on");
    } else if (cmd.equalsIgnoreCase("OFF-P")) {
      digitalWrite(ledPin, LOW);
      Serial.println("Turning LED off");
    }
  }

  // 2) Read button with debounce
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > debounceDelay) {
    if (reading != debouncedState) {
      debouncedState = reading;
      // button pulled LOW when pressed
      if (debouncedState == LOW) {
        Serial.println("Button pressed → sending ON");
        btSerialController.println("ON-C");
      } else{
        Serial.println("Not pressed, sending OFF");
        btSerialController.println("OFF-C");
      }
    }
  }
  lastButtonReading = reading;
  // int state = digitalRead(buttonPin);
  // Serial.print("Button state: ");
  // Serial.println(state);  // should print 1 when released, 0 when pressed

  // if (state == LOW) {
  //   Serial.println("!! Button pressed → sending ON");
  //   btSerialController.println("ON-C");
  //   delay(200);  // simple bounce‐filter and rate‐limiter
  // }

  // delay(50);
}
