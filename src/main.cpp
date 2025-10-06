// This is the main file for the WheelDriver project, which controls a set of motors using LS7466 encoders.

 #include <Arduino.h>
 #include <SPI.h>
 #include "WheelDriver.h"

 
 
 //Whoopee Light:
 #define LED_PIN 30

// Define the chip select pin for each decoder (for SPI communication)
#define DECODER0_CS_PIN 2  
#define DECODER1_CS_PIN 3  
#define DECODER2_CS_PIN 5  
#define DECODER3_CS_PIN 4  

// Define the motor control pins
#define MOTOR0_EN_PIN 23
#define MOTOR0_DIR_A_PIN 22
#define MOTOR0_DIR_B_PIN 21

#define MOTOR1_EN_PIN 6
#define MOTOR1_DIR_A_PIN 7
#define MOTOR1_DIR_B_PIN 8

#define MOTOR2_EN_PIN 9 // Not a pwm pin :(
#define MOTOR2_DIR_A_PIN 19
#define MOTOR2_DIR_B_PIN 18

#define MOTOR3_EN_PIN 24
#define MOTOR3_DIR_A_PIN 25
#define MOTOR3_DIR_B_PIN 26

#define MOTOR4_EN_PIN 36
#define MOTOR4_DIR_A_PIN 16
#define MOTOR4_DIR_B_PIN 38

#define MOTOR5_EN_PIN 37
#define MOTOR5_DIR_A_PIN 28
#define MOTOR5_DIR_B_PIN 29

#define MOTOR6_EN_PIN 15 
#define MOTOR6_DIR_A_PIN 40
#define MOTOR6_DIR_B_PIN 39

#define MOTOR7_EN_PIN 14 
#define MOTOR7_DIR_A_PIN 31
#define MOTOR7_DIR_B_PIN 32

// Create instances of the LS7466 class
LS7466 decoder0(DECODER0_CS_PIN);
LS7466 decoder1(DECODER1_CS_PIN);
LS7466 decoder2(DECODER2_CS_PIN);
LS7466 decoder3(DECODER3_CS_PIN);

// Initialize motor objects
// Format: Motor(decpder_object, decoder_axis, enablePin, dirAPin, dirBPin)
Motor M0 = Motor(&decoder0, 0, MOTOR0_EN_PIN, MOTOR0_DIR_A_PIN, MOTOR0_DIR_B_PIN);
Motor M1 = Motor(&decoder0, 1, MOTOR1_EN_PIN, MOTOR1_DIR_A_PIN, MOTOR1_DIR_B_PIN);

Motor M2 = Motor(&decoder1, 1, MOTOR2_EN_PIN, MOTOR2_DIR_A_PIN, MOTOR2_DIR_B_PIN);
Motor M3 = Motor(&decoder1, 0, MOTOR3_EN_PIN, MOTOR3_DIR_A_PIN, MOTOR3_DIR_B_PIN);

Motor M4 = Motor(&decoder2, 1, MOTOR4_EN_PIN, MOTOR4_DIR_A_PIN, MOTOR4_DIR_B_PIN);
Motor M5 = Motor(&decoder2, 0, MOTOR5_EN_PIN, MOTOR5_DIR_A_PIN, MOTOR5_DIR_B_PIN);

Motor M6 = Motor(&decoder3, 0, MOTOR6_EN_PIN, MOTOR6_DIR_A_PIN, MOTOR6_DIR_B_PIN);
Motor M7 = Motor(&decoder3, 1, MOTOR7_EN_PIN, MOTOR7_DIR_A_PIN, MOTOR7_DIR_B_PIN);

// Create an array of motor pointers for easy access
Motor* motors[] = {&M0, &M1, &M2, &M3, &M4, &M5, &M6, &M7};

// Define motor gains for all motors:
float Kp = 10.0; // Proportional gain
float Ki = 0.00; // Integral gain
float Kd = 0.0; // Derivative gain


// Set up serial communication
Comms com = Comms();
MotorCommand cmd_msg;


/***** SETUP ******/
void setup() {
   // Initialize serial communication
    com.init();
    pinMode(LED_PIN, OUTPUT); // Set LED pin as output
 
   // Assign universal motor parameters
   for(int i = 0; i < 8; i++){
     motors[i]->setGains(Kp, Ki, Kd); // Set gains for each motor
     motors[i]->_controlMode = 2; // Set initial control mode
   }
   Serial.println("Program started");
   digitalWrite(LED_PIN, HIGH); // Turn on LED to indicate setup completion
   
}
 
 float test_vel = 10; // Test velocity for motor
 float test_duty = 100; // Test duty cycle for motors

 /***** LOOP ******/
 void loop() {
    // Check for incoming commands. If no command is received, cmd_msg will be set to "none"
    cmd_msg = com.getCMD();
    if (cmd_msg.cmd != "none") {
        //If valid command received, update the motor ID and command
        if (cmd_msg.motorID < 8) {
          // Update motors based on received command
          motors[cmd_msg.motorID]->update(cmd_msg);
        }
        // If motorID is 8, send the command to all motors
        else if (cmd_msg.motorID == 8){
          // Send same command to all motors
          for(int i = 0; i < 8; i++){
            motors[i]->update(cmd_msg);
          }

        }
    }


    // Perform main control loop and read back velocity for all motors
    for(int i = 0; i<8; i++){
      motors[i]->controlLoop();
      Serial.print(i);
      Serial.print(",");
      Serial.println(motors[i]->_currentVel);
    }

    delay(1);
 }
