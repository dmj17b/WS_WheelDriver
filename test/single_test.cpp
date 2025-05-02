/**
 * LS7466 Decoder Library Usage Example
 * 
 * This example demonstrates how to use the updated LS7466 library,
 * following the approach used in the datasheet example.
 */

 #include <Arduino.h>
 #include <SPI.h>
 #include "LS7466.h"
 
 // Define the chip select pin
 #define DECODER1_CS_PIN 2  // Updated to match your example
 
 // Create an instance of the LS7466 class
 LS7466 decoder1(DECODER1_CS_PIN);
 
 void setup() {
  Serial.begin(9600);
  decoder1.begin();
  
  // Reset registers
  decoder1.loadResetReg(RST_MCR0xy);
  decoder1.loadResetReg(RST_MCR1xy);
  decoder1.loadResetReg(RST_CNTRxy);
  
  // Set 3-byte mode, enable counting
  decoder1.configMCR0(LS7466::AXIS_X, BYTE_2 | EN_CNTR);
  decoder1.configMCR0(LS7466::AXIS_Y, BYTE_2 | EN_CNTR);
  
  
  // Reset counter to zero
  decoder1.resetCounter(LS7466::AXIS_X);
  
  delay(100); // Give time for everything to settle
}
 
 void loop() {
   // Read counter values from both axes using the direct method
   long xCount = decoder1.readCounter(LS7466::AXIS_X);
   long yCount = decoder1.readCounter(LS7466::AXIS_Y);
   
   // Also read using the exact same method as the example code
   decoder1.singleByteRD(RDC_LDSy);
   
   // Print the counter values
   Serial.print("X axis count: ");
   Serial.print(xCount);
   Serial.print("\tY axis count: ");
   Serial.println(yCount);
   
   // Wait before reading again
   delay(500);
 }