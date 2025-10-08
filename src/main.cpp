#include <Arduino.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "wheel_motor_messages.pb.h"
#include "MotorLink.h"

#define VELOCITY ControlMode_VELOCITY
#define BREAK ControlMode_BREAK

// Use the single USB Serial port for communication with the Orin
MotorLink motor_link;

// Timing for sending state back to the Orin at 500 Hz
const long STATE_INTERVAL_MS = 1; // 1000ms / 500Hz
unsigned long last_state_time = 0;

// Debug control - set to false to disable debug output
const bool DEBUG_ENABLED = true;

void setup() {
    // Start the USB serial for communication with the Orin
    motor_link.begin(4000000); // 4 Mbps
    motor_link.enableDebug(DEBUG_ENABLED);
    
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
            // Example: Set motor velocities based on the received command.
            // for (int i = 0; i < 8; ++i) {
            //   my_motors[i].setVelocity(command.velocity_setpoint[i], command.kp[i]);
            // }
            
            if (DEBUG_ENABLED) {
                motor_link.debugPrint("Velocity command received for 8 motors");
            }
        } else if (command.control_mode == BREAK) {
            // Example: apply brakes
            if (DEBUG_ENABLED) {
                motor_link.debugPrint("Break command received");
            }
        } else if (command.control_mode == ControlMode_IDLE) {
            // Example: set motors to idle
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

    // 3. At a fixed 500 Hz rate, read motor state and send it back to the Orin.
    if (millis() - last_state_time >= STATE_INTERVAL_MS) {
        last_state_time = millis();

        WheelMotorState current_state = WheelMotorState_init_zero;
        current_state.control_mode = VELOCITY; // Report current mode
        current_state.velocity_count = 8;
        for (int i = 0; i < 8; ++i) {
            // --- READ YOUR ACTUAL MOTOR ENCODER/SENSOR VALUES HERE ---
            // current_state.velocity[i] = my_motors[i].getVelocity();
            current_state.velocity[i] = 4.95; // Placeholder value for testing
        }

        motor_link.sendState(current_state);
    }
}
