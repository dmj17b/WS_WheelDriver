/*
    * WheelDriver.h
    * Abstraction class for controlling brushed motors using LS7466 decoders
*/

#include "LS7466.h"
#include <Arduino.h>

class Motor{
    public:
        // Constructor
        // Takes in encoder chip select pin, encoder number, motor enable pin, and direction pins
        Motor(uint8_t encoderCS, uint8_t encAxis, uint8_t EN, uint8_t DIR_A, uint8_t DIR_B);


        // Motor init pins
        uint8_t _motorEnablePin; // Motor enable pin
        uint8_t _motorDirAPin; // Motor direction A pin
        uint8_t _motorDirBPin; // Motor direction B pin

        // Encoder init variables
        LS7466* _encoder; // Pointer to LS7466 encoder object
        uint8_t encAxis; // Encoder number (0 or 1)
        long _encoderCount; // Motor encoder count
        int _encoderCPR = 12; // Encoder counts per revolution


        // Motor control variables
        bool reverse = false; // Reverse motor direction
        bool safety_on = true;  // Safety switch for motor control
        int _motorDutyCycle; // Motor duty cycle
        bool _motorDirection; // Motor direction
        float _gearRatio = 1.0; // Gear ratio for motor
        float _targetPos = 0.0; // Target position for PID control
        float _targetVel = 0.0; // Target velocity for PID control
        float _currentPos = 0.0; // Current position
        float _currentVel = 0.0; // Current velocity
        float _lastPos = 0.0; // Last position for velocity calculation
        float _lastTime = 0.0; // Last time for velocity calculation
        float Kp = 0.0; // Proportional gain for PID control
        float Ki = 0.0; // Integral gain for PID control
        float Kd = 0.0; // Derivative gain for PID control




        // Basic functions
        void fwd_drive(int dutyCycle);
        float getPos();
        float getVel();

        

};

Motor :: Motor(uint8_t encoderCS, uint8_t encAxis, uint8_t EN, uint8_t DIR_A, uint8_t DIR_B){
    // Initialize the LS7466 encoder
    _encoder = new LS7466(encoderCS);
    _encoder->begin();
    _encoder->configMCR0(encAxis, BYTE_2 | EN_CNTR);
    _encoder->resetCounter(encAxis);

    encAxis = encAxis; // Set encoder axis


    // Set up motor control pins
    _motorEnablePin = EN;
    _motorDirAPin = DIR_A;
    _motorDirBPin = DIR_B;

    pinMode(_motorEnablePin, OUTPUT);
    pinMode(_motorDirAPin, OUTPUT);
    pinMode(_motorDirBPin, OUTPUT);
};

float Motor::getPos(){
    // Read the encoder count
    _encoderCount = _encoder->readCounter(encAxis);
    // Convert to position in degrees
    float pos = (_encoderCount / (float)_encoderCPR) * 360.0;
    return pos;
}