/*
    * WheelDriver.h
    * Abstraction class for controlling brushed motors using LS7466 decoders
*/

#include "LS7466.h"
#include "SerComs.h"
#include <Arduino.h>

class Motor{
    public:
        // Constructor
        // Takes in encoder chip select pin, encoder number, motor enable pin, and direction pins
        Motor(uint8_t encoderCS, uint8_t encAxis, uint8_t EN, uint8_t DIR_A, uint8_t DIR_B);

        enum controlMode{
            POS_CONTROL = 0,
            VEL_CONTROL = 1,
            COAST = 2,
            BRAKE = 3,
            E_STOP = 4,
        };

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
        float _gearRatio = 1.0; // Gear ratio for motor
        float _targetPos = 0.0; // Target position for PID control
        float _targetVel = 0.0; // Target velocity for PID control
        float _currentPos = 0.0; // Current position
        float _currentVel = 0.0; // Current velocity
        float _lastPos = 0.0; // Last position for velocity calculation
        float _lastTime = 0.0; // Last time for velocity calculation
        float _Kp = 0.0; // Proportional gain for PID control
        float _Ki = 0.0; // Integral gain for PID control
        float _Kd = 0.0; // Derivative gain for PID control

        // Main control parsing function:
        void parseCommand(MotorCommand cmd_msg);


        // Motor Control functions
        void controlLoop();     // Main control loop function
        void fwd_drive(int dutyCycle);  //FWD drive function
        void rev_drive(int dutyCycle);  //REV drive function
        void positionControl(float targetPos);
        void velocityControl(float targetVel);
        void brake();
        void coast();
        void estop();
        void setGains(float Kp, float Ki, float Kd);

        // Data retrieval functions
        float getPos();
        float getVel();

};

// Constructor for Motor class
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


// Function to retrieve current position
float Motor::getPos(){
    // Read the encoder count
    _encoderCount = _encoder->readCounter(encAxis);
    // Convert to position in degrees
    float pos = (_encoderCount / ((float)_encoderCPR*_gearRatio)) * 360.0;
    return pos;
}

void Motor::setGains(float Kp, float Ki, float Kd){
    // Set PID gains
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
};



// Function to forward drive the motor
void Motor::fwd_drive(int dutyCycle){
    // Set motor direction and enable
    if (dutyCycle > 0){
        digitalWrite(_motorDirAPin, HIGH);
        digitalWrite(_motorDirBPin, LOW);
    }
    else{
        digitalWrite(_motorDirAPin, LOW);
        digitalWrite(_motorDirBPin, HIGH);
    }
    analogWrite(_motorEnablePin, abs(dutyCycle));
};

// Function to reverse drive the motor
void Motor::rev_drive(int dutyCycle){
    // Set motor direction and enable
    if (dutyCycle > 0){
        digitalWrite(_motorDirAPin, LOW);
        digitalWrite(_motorDirBPin, HIGH);
    }
    else{
        digitalWrite(_motorDirAPin, HIGH);
        digitalWrite(_motorDirBPin, LOW);
    }
    analogWrite(_motorEnablePin, abs(dutyCycle));
};


// Brake and coast functions (one needs to change when we test)
void Motor::brake(){
    // Set motor direction and enable
    digitalWrite(_motorDirAPin, LOW);
    digitalWrite(_motorDirBPin, LOW);
    analogWrite(_motorEnablePin, 0);
};

void Motor::coast(){
    // Set motor direction and enable
    digitalWrite(_motorDirAPin, LOW);
    digitalWrite(_motorDirBPin, LOW);
    analogWrite(_motorEnablePin, 0);
};

// MAIN CONTROL LOOP
void Motor::controlLoop(){
    // Calculate the current position and velocity
    _currentPos = getPos();
    _currentVel = (_currentPos - _lastPos) / (millis() - _lastTime);
    _lastPos = _currentPos;
    _lastTime = millis();

    // Calculate the error
    float posError = _targetPos - _currentPos;
    float velError = _targetVel - _currentVel;

    // Calculate the PID output
    float output = (_Kp * posError) + (_Ki * velError) + (_Kd * (velError - posError));

    // Set the motor duty cycle
    if (output > 0){
        fwd_drive(output);
    }
    else{
        rev_drive(-output);
    }
}