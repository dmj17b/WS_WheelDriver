/**
 * LS7466 Decoder Library Usage Example
 * 
 * This example demonstrates how to use the updated LS7466 library,
 * following the approach used in the datasheet example.
 */

 #include <Arduino.h>
 #include <SPI.h>
 #include "WheelDriver.h"

 
 
 
 // Define the chip select pin for each decoder (for SPI communication)
 #define DECODER1_CS_PIN 2  
 #define DECODER2_CS_PIN 3  
 #define DECODER3_CS_PIN 5  
 #define DECODER4_CS_PIN 4  
 
 
 // Define the motor control pins
 #define MOTOR1_EN_PIN 23
 #define MOTOR1_DIR_A_PIN 22
 #define MOTOR1_DIR_B_PIN 21
 
 #define MOTOR2_EN_PIN 6
 #define MOTOR2_DIR_A_PIN 7
 #define MOTOR2_DIR_B_PIN 8
 
 #define MOTOR3_EN_PIN 20
 #define MOTOR3_DIR_A_PIN 19
 #define MOTOR3_DIR_B_PIN 18
 
 #define MOTOR4_EN_PIN 24
 #define MOTOR4_DIR_A_PIN 25
 #define MOTOR4_DIR_B_PIN 26
 
 #define MOTOR5_EN_PIN 17
 #define MOTOR5_DIR_A_PIN 16
 #define MOTOR5_DIR_B_PIN 38
 
 #define MOTOR6_EN_PIN 27
 #define MOTOR6_DIR_A_PIN 28
 #define MOTOR6_DIR_B_PIN 29
 
 #define MOTOR7_EN_PIN 41
 #define MOTOR7_DIR_A_PIN 40
 #define MOTOR7_DIR_B_PIN 31
 
 #define MOTOR8_EN_PIN 30
 #define MOTOR8_DIR_A_PIN 31
 #define MOTOR8_DIR_B_PIN 32
 
 
 // Initialize motor objects
 Motor FL1 = Motor(DECODER1_CS_PIN, 0, MOTOR1_EN_PIN, MOTOR1_DIR_A_PIN, MOTOR1_DIR_B_PIN);
 Motor FL2 = Motor(DECODER1_CS_PIN, 1, MOTOR2_EN_PIN, MOTOR2_DIR_A_PIN, MOTOR2_DIR_B_PIN);
 
 Motor FR1 = Motor(DECODER2_CS_PIN, 0, MOTOR3_EN_PIN, MOTOR3_DIR_A_PIN, MOTOR3_DIR_B_PIN);
 Motor FR2 = Motor(DECODER2_CS_PIN, 1, MOTOR4_EN_PIN, MOTOR4_DIR_A_PIN, MOTOR4_DIR_B_PIN);
 
 Motor BL1 = Motor(DECODER3_CS_PIN, 0, MOTOR5_EN_PIN, MOTOR5_DIR_A_PIN, MOTOR5_DIR_B_PIN);
 Motor BL2 = Motor(DECODER3_CS_PIN, 1, MOTOR6_EN_PIN, MOTOR6_DIR_A_PIN, MOTOR6_DIR_B_PIN);
 
 Motor BR1 = Motor(DECODER4_CS_PIN, 0, MOTOR7_EN_PIN, MOTOR7_DIR_A_PIN, MOTOR7_DIR_B_PIN);
 Motor BR2 = Motor(DECODER4_CS_PIN, 1, MOTOR8_EN_PIN, MOTOR8_DIR_A_PIN, MOTOR8_DIR_B_PIN);
 
 Motor* motors[] = {&FL1, &FL2, &FR1, &FR2, &BL1, &BL2, &BR1, &BR2};
 
 // Define motor gains for all motors:
 float Kp = 0.1; // Proportional gain
 float Ki = 0.01; // Integral gain
 float Kd = 0.01; // Derivative gain
 

 // Set up serial communication
  Comms com = Comms();
  MotorCommand cmd_msg;


 /***** SETUP ******/
 void setup() {
   // Initialize serial communication
    com.init();
 
   // Assign universal motor parameters
   for(int i = 0; i < 8; i++){
     motors[i]->setGains(Kp, Ki, Kd); // Set gains for each motor
   }
   
 }
 
 
 
 /***** LOOP ******/
 void loop() {

  cmd_msg = com.getCMD();

  // Check if a command was received
  if(cmd_msg.cmd != "none"){
    // Check if the command is for a specific motor
    if(cmd_msg.motorID < 8){
      Serial.print("Motor ");
      Serial.print(cmd_msg.motorID);
      Serial.print(" target velocity set to ");
      Serial.println(cmd_msg.val);
    }
    else{
      Serial.println("Invalid motor ID");
    }
  }

 }