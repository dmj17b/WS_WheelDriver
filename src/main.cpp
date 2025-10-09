#include <Arduino.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "wheel_motor_messages.pb.h"
#include "MotorLink.h"
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

#define VELOCITY ControlMode_VELOCITY
#define BRAKE ControlMode_BREAK

// Use the single USB Serial port for communication with the Orin
MotorLink motor_link;

// Timing for sending state back to the Orin
const long STATE_INTERVAL_MS = 1; // 1000ms / 500Hz
unsigned long last_state_time = 0;

// Debug control - set to false to disable debug output
const bool DEBUG_ENABLED = true;
uint8_t current_mode = 0;

void setup() {
    // Start the USB serial for communication with the Orin
    motor_link.begin(4000000); // 4 Mbps
    motor_link.enableDebug(DEBUG_ENABLED); 

    for(int i = 0; i < 8; ++i) {
        motors[i]->set_controlMode(0); // Set all motors to idle mode initially
        motors[i]->set_targetVel(0.0); // Set target velocity to 0
        motors[i]->set_Kp(10.0); // Set a default Kp value
    }
    
    if (DEBUG_ENABLED) {
        motor_link.debugPrint("Teensy ready to receive commands via USB Serial");
    }
}

void loop() {
    // 1. Always call update() to process any incoming bytes from the Orin.
    motor_link.update();

    // 2. Check if a new, valid command has been received and parsed.
    WheelMotorCommand command;
    if (motor_link.readCommand(command)) {
        // --- YOUR MOTOR CONTROL LOGIC GOES HERE ---
        // Act on the new command immediately.
        if (command.control_mode == VELOCITY && command.velocity_setpoint_count == 8) {
            current_mode = VELOCITY;
            // Turn on LED to indicate active command
            digitalWrite(LED_PIN, HIGH);
            // Change control mode, velocity setpoints, and gains for each motor
            for (int i = 0; i < 8; i++) {
                motors[i]->set_Kp(command.kp[i]);
                motors[i]->set_controlMode(2); // Set to velocity control mode
                motors[i]->set_targetVel(command.velocity_setpoint[i]);
            }            
            if (DEBUG_ENABLED) {
                motor_link.debugPrint("Velocity command received for 8 motors");
            }
        } else if (command.control_mode == BRAKE) {
            // Example: apply brakes
            for (int i = 0; i < 8; i++) {
                motors[i]->set_controlMode(4); // Set to brake mode
            }
            if (DEBUG_ENABLED) {
                motor_link.debugPrint("Break command received");
            }
        } else if (command.control_mode == ControlMode_IDLE) {
            // Example: set motors to idle
            for (int i = 0; i < 8; i++) {
                motors[i]->set_controlMode(0); // Set to idle (estop) mode
            }
            digitalWrite(LED_PIN, LOW); // Turn off LED to indicate idle
            if (DEBUG_ENABLED) {
                motor_link.debugPrint("Idle command received");
            }
        } else {
            if (DEBUG_ENABLED) {
                motor_link.debugPrint("Unknown command or invalid data");
            }
        }
        
        // Optional debug output (only if debugging is enabled)
        if (DEBUG_ENABLED) {
            String debug_msg = "New command received! Mode: " + String(command.control_mode);
            motor_link.debugPrint(debug_msg);
        }
    }

    // 3. Execute control loops for all motors
    for (int i = 0; i < 8; i++) {
        motors[i]->controlLoop();
    }

    // 4. At a fixed 500 Hz rate, read motor state and send it back to the Orin.
    if (millis() - last_state_time >= STATE_INTERVAL_MS) {
        last_state_time = millis();

        WheelMotorState current_state = WheelMotorState_init_zero;
        current_state.control_mode = VELOCITY; // Report current mode
        current_state.velocity_count = 8;
        for (int i = 0; i < 8; ++i) {
            // --- READ YOUR ACTUAL MOTOR ENCODER/SENSOR VALUES HERE ---
            current_state.velocity[i] = motors[i]->getVel(); // rad/s           
        }

        motor_link.sendState(current_state);
    }
}
