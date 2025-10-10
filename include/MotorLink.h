#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "wheel_motor_messages.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

class MotorLink {
public:
    MotorLink();  // No longer takes a serial port parameter
    void begin(long baud_rate);
    void update();
    bool readCommand(WheelMotorCommand& command);
    bool sendState(const WheelMotorState& state);
    
    // Connection status checking
    bool isConnected() const { return Serial && Serial.availableForWrite(); }
    
    // Debug output control
    void enableDebug(bool enable) { debug_enabled_ = enable; }
    void debugPrint(const char* message);
    void debugPrint(const String& message);

private:
    enum class ParseState {
        WAITING_FOR_SOF,
        READING_LENGTH,
        READING_PAYLOAD
    };

    ParseState current_state_ = ParseState::WAITING_FOR_SOF;

    uint8_t rx_buffer_[WheelMotorCommand_size];
    uint8_t payload_len_ = 0;
    uint8_t bytes_received_ = 0;

    WheelMotorCommand last_command_;
    bool new_command_available_ = false;
    bool debug_enabled_ = false;
};