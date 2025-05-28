/*
    * WheelDriver.h
    * Abstraction class for controlling brushed motors using LS7466 decoders
*/

#include "LS7466.h"
#include "SerComs.h"
#include <Arduino.h>

#define IDLE 0
#define POSITION 1
#define VELOCITY 2
#define DUTY 3
#define BRAKE 4


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
        float _gearRatio = 1.0; // Gear ratio for motor
        float _targetPos = 0.0; // Target position for PID control
        float _targetVel = 0.0; // Target velocity for PID control
        float _currentPos = 0.0; // Current position
        float _currentVel = 0.0; // Current velocity
        float _lastPos = 0.0; // Last position for velocity calculation
        float _lastTime = 0.0; // Last time for velocity calculation
        float _pos_error = 0.0; // Position error for PID control
        float _vel_error = 0.0; // Velocity error for PID control
        float _Kp = 0.0; // Proportional gain for PID control
        float _Ki = 0.0; // Integral gain for PID control
        float _Kd = 0.0; // Derivative gain for PID control

        // Main control parsing function:
        MotorCommand cmd_msg; // Motor command message


        // Motor Control functions
        uint8_t _motorID; // Motor ID
        uint8_t _controlMode;
        void update(MotorCommand cmd_msg); // Update function to handle commands
        void controlLoop();     // Main control loop function
        void fwd_drive(int dutyCycle);  //FWD drive function
        void rev_drive(int dutyCycle);  //REV drive function
        void positionControl(float targetPos);
        void velocityControl(float targetVel);
        void brake();
        void coast();
        void estop();
        void setGains(float Kp, float Ki, float Kd);
        

        // Data retrieval functions (for returning to host computer)
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
    return _currentPos;
};
// Function to retrieve current velocity
float Motor::getVel(){
    return _currentVel;
};

// Function to set control gains
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

// UPDATE FUNCTION
void Motor::update(MotorCommand cmd_msg){
    // Update the command message
    this->cmd_msg = cmd_msg;
    if(cmd_msg.cmd == "setMode"){
        _controlMode = cmd_msg.val;
    }
    else if (cmd_msg.cmd == "setPos"){
        _targetPos = cmd_msg.val;
    }
    else if (cmd_msg.cmd == "setVel"){
        _targetVel = cmd_msg.val;
    }
    else if (cmd_msg.cmd == "setDuty"){
        _motorDutyCycle = cmd_msg.val;
    }
    else if (cmd_msg.cmd == "brake"){
        brake();
    }
    else if (cmd_msg.cmd == "coast"){
        coast();
    }
    else if (cmd_msg.cmd == "estop"){
        estop();
    }
    else{
        Serial.println("Invalid command");
    }
};

// MAIN CONTROL LOOP
void Motor::controlLoop(){
    // Calculate the current position (angle) and velocity
    _encoderCount = _encoder->readCounter(encAxis);
    _currentPos = (_encoderCount / ((float)_encoderCPR*_gearRatio))*2*PI;    // Convert encoder count to position in radians
    _currentVel = (_currentPos - _lastPos) / (pow(10,6)*(micros() - _lastTime));    // Calculate velocity in radians per second

    // Update the last position and time
    _lastPos = _currentPos;
    _lastTime = micros();


    // Switch control loop based on control mode:
    switch(_controlMode){
        case 0: // Position control
            positionControl(_targetPos);
            break;
        case 1: // Velocity control
            velocityControl(_targetVel);
            break;
        case 2: // Duty cycle control
            if (_motorDutyCycle > 0){
                fwd_drive(_motorDutyCycle);
            }
            else{
                rev_drive(_motorDutyCycle);
            }
            break;
        default:
            Serial.println("Invalid control mode");
    }

}

void Motor::positionControl(float targetPos){
    // Calculate the error
    float pos_error = targetPos - _currentPos;

    // Calculate the control output using PID control
    float controlOutput = _Kp * pos_error ; // Proportional control

    // Set the motor duty cycle based on the control output
    if (controlOutput > 0){
        fwd_drive(controlOutput);
    }
    else{
        rev_drive(-controlOutput);
    }
}