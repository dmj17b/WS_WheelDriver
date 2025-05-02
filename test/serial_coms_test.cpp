/* Unit testing of serial communication methods between
Teensy 4.1 and computer through USB port
*/
#include <Arduino.h>

void setup() {
    Serial.begin(115200); // Start serial communication at 115200 baud rate
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Serial communication test started.");
}

void loop() {
    // Test serial communication by sending a message every second
    while(Serial.available()) {
        int val = Serial.parseInt(); // Read an integer from the serial port
        float val2 = Serial.parseFloat(); // Read a float from the serial port
        Serial.print("Received integer: ");
        Serial.print(val); // Print the received integer
        Serial.print("  Received float: ");
        Serial.println(val2); // Print the received float
    }
}

