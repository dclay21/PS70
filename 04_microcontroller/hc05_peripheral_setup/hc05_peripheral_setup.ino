#include <Arduino.h>

#define RX_S 3
#define TX_S 2

HardwareSerial btSerialPeripheral(1); 

void setup() {
  Serial.begin(9600);
  btSerialPeripheral.begin(38400, SERIAL_8N1, RX_S, TX_S);

}

void loop() {
  delay(2000);
  btSerialPeripheral.print("AT+ADDR?\r\n");
  delay(1000);
  while (btSerialPeripheral.available()) {
    Serial.write(btSerialPeripheral.read());
  }
  Serial.println();
}

// +ADDR:0025:00:0011DF

// #include <Arduino.h>

// #define RX_S 5
// #define TX_S 4

// HardwareSerial btSerialSlave(1); 
// void setup() {
//   Serial.begin(9600);
//   btSerialSlave.begin(9600, SERIAL_8N1, RX_S, TX_S);

// }

// void loop() {
//   delay(10000);
//   Serial.println("Reading");
//   while (btSerialSlave.available()) {
//     Serial.write(btSerialSlave.read());
//   }
// }







