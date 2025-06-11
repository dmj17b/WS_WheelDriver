// This is the main file for the WheelDriver project, which controls a set of motors using LS7466 encoders.

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
 
 #define MOTOR3_EN_PIN 20 // Not a pwm pin :(
 #define MOTOR3_DIR_A_PIN 19
 #define MOTOR3_DIR_B_PIN 18
 
 #define MOTOR4_EN_PIN 24
 #define MOTOR4_DIR_A_PIN 25
 #define MOTOR4_DIR_B_PIN 26
 
 #define MOTOR5_EN_PIN 17 // Not a pwm pin :(
 #define MOTOR5_DIR_A_PIN 16
 #define MOTOR5_DIR_B_PIN 38
 
 #define MOTOR6_EN_PIN 27 // Not a pwm pin :(
 #define MOTOR6_DIR_A_PIN 28
 #define MOTOR6_DIR_B_PIN 29
 
 #define MOTOR7_EN_PIN 41 // Not a pwm pin :(
 #define MOTOR7_DIR_A_PIN 40
 #define MOTOR7_DIR_B_PIN 31
 
 #define MOTOR8_EN_PIN 30 // Not a pwm pin :(
 #define MOTOR8_DIR_A_PIN 31
 #define MOTOR8_DIR_B_PIN 32

  // Create instances of the LS7466 class
 LS7466 decoder1(DECODER1_CS_PIN);
 LS7466 decoder2(DECODER2_CS_PIN);
 LS7466 decoder3(DECODER3_CS_PIN);
 LS7466 decoder4(DECODER4_CS_PIN);
 
 // Initialize motor objects
 Motor M0 = Motor(&decoder1, 1, MOTOR1_EN_PIN, MOTOR1_DIR_A_PIN, MOTOR1_DIR_B_PIN);
 Motor M1 = Motor(&decoder1, 0, MOTOR2_EN_PIN, MOTOR2_DIR_A_PIN, MOTOR2_DIR_B_PIN);
 
 Motor M2 = Motor(&decoder2, 0, MOTOR3_EN_PIN, MOTOR3_DIR_A_PIN, MOTOR3_DIR_B_PIN);
 Motor M3 = Motor(&decoder2, 1, MOTOR4_EN_PIN, MOTOR4_DIR_A_PIN, MOTOR4_DIR_B_PIN);
 
 Motor M4 = Motor(&decoder3, 0, MOTOR5_EN_PIN, MOTOR5_DIR_A_PIN, MOTOR5_DIR_B_PIN);
 Motor M5 = Motor(&decoder3, 1, MOTOR6_EN_PIN, MOTOR6_DIR_A_PIN, MOTOR6_DIR_B_PIN);
 
 Motor M6 = Motor(&decoder4, 0, MOTOR7_EN_PIN, MOTOR7_DIR_A_PIN, MOTOR7_DIR_B_PIN);
 Motor M7 = Motor(&decoder4, 1, MOTOR8_EN_PIN, MOTOR8_DIR_A_PIN, MOTOR8_DIR_B_PIN);

 // Create an array of motor pointers for easy access
 Motor* motors[] = {&M0, &M1, &M2, &M3, &M4, &M5, &M6, &M7};
 
 // Define motor gains for all motors:
 float Kp = 1.0; // Proportional gain
 float Ki = 0.01; // Integral gain
 float Kd = 0.0; // Derivative gain
 

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
   Serial.println("Program started");
   
 }
 
 
 
 /***** LOOP ******/
 void loop() {
    // Check for incoming commands. If no command is received, cmd_msg will be set to "none"
    cmd_msg = com.getCMD();

    // If a valid command is received, update the corresponding motor
    if (cmd_msg.motorID < 8 && cmd_msg.cmd != "none") { 
      motors[cmd_msg.motorID]->update(cmd_msg); 
    }
    
    // Run control loop for each motor
    for(int i = 0; i < 8; i++){
      motors[i]->controlLoop();
    }
    
    delay(10); // Small delay to prevent overwhelming the loop
 }
