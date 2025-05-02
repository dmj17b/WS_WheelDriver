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
 #define DECODER1_CS_PIN 2  
 #define DECODER2_CS_PIN 3  
 #define DECODER3_CS_PIN 5  
 #define DECODER4_CS_PIN 4  
 
 
 // Create an instance of the LS7466 class
 LS7466 decoder1(DECODER1_CS_PIN);
 LS7466 decoder2(DECODER2_CS_PIN);
 LS7466 decoder3(DECODER3_CS_PIN);
 LS7466 decoder4(DECODER4_CS_PIN);
 
 void setup() {
 Serial.begin(9600);
 decoder1.begin();
 
 // Reset registers
 decoder1.loadResetReg(RST_MCR0xy);
 decoder1.loadResetReg(RST_MCR1xy);
 decoder1.loadResetReg(RST_CNTRxy);
 decoder2.loadResetReg(RST_MCR0xy);
 decoder2.loadResetReg(RST_MCR1xy);
 decoder2.loadResetReg(RST_CNTRxy);
 decoder3.loadResetReg(RST_MCR0xy);
 decoder3.loadResetReg(RST_MCR1xy);
 decoder3.loadResetReg(RST_CNTRxy);
 decoder4.loadResetReg(RST_MCR0xy);
 decoder4.loadResetReg(RST_MCR1xy);
 decoder4.loadResetReg(RST_CNTRxy);
 
 // Set 3-byte mode, enable counting
 decoder1.configMCR0(LS7466::AXIS_X, BYTE_2 | EN_CNTR);
 decoder1.configMCR0(LS7466::AXIS_Y, BYTE_2 | EN_CNTR);
 decoder2.configMCR0(LS7466::AXIS_X, BYTE_2 | EN_CNTR);
 decoder2.configMCR0(LS7466::AXIS_Y, BYTE_2 | EN_CNTR);
 decoder3.configMCR0(LS7466::AXIS_X, BYTE_2 | EN_CNTR);
 decoder3.configMCR0(LS7466::AXIS_Y, BYTE_2 | EN_CNTR);
 decoder4.configMCR0(LS7466::AXIS_X, BYTE_2 | EN_CNTR);
 decoder4.configMCR0(LS7466::AXIS_Y, BYTE_2 | EN_CNTR);
 
 
 // Reset counter to zero
 decoder1.resetCounter(LS7466::AXIS_X);
 decoder1.resetCounter(LS7466::AXIS_Y);
 decoder2.resetCounter(LS7466::AXIS_X);
 decoder2.resetCounter(LS7466::AXIS_Y);
 decoder3.resetCounter(LS7466::AXIS_X);
 decoder3.resetCounter(LS7466::AXIS_Y);
 decoder4.resetCounter(LS7466::AXIS_X);
 decoder4.resetCounter(LS7466::AXIS_Y);
 
 
 delay(100); // Give time for everything to settle
 }
 
 void loop() {
   // Read counter values from both axes using the direct method
 
   long m1 = decoder1.readCounter(LS7466::AXIS_X);
   long m2 = decoder1.readCounter(LS7466::AXIS_Y);
   long m3 = decoder2.readCounter(LS7466::AXIS_X);
   long m4 = decoder2.readCounter(LS7466::AXIS_Y);
   long m5 = decoder3.readCounter(LS7466::AXIS_X);
   long m6 = decoder3.readCounter(LS7466::AXIS_Y);
   long m7 = decoder4.readCounter(LS7466::AXIS_X);
   long m8 = decoder4.readCounter(LS7466::AXIS_Y);
   // Print the counter values
   Serial.print("M1: ");
   Serial.print(m1);
   Serial.print("\tM2: ");
   Serial.print(m2);
   Serial.print("\tM3: ");
   Serial.print(m3);
   Serial.print("\tM4: ");
   Serial.print(m4);
   Serial.print("\tM5: ");
   Serial.print(m5);
   Serial.print("\tM6: ");
   Serial.print(m6);
   Serial.print("\tM7: ");
   Serial.print(m7);
   Serial.print("\tM8: ");
   Serial.println(m8);
 
   // Wait before reading again
   delay(500);
 }