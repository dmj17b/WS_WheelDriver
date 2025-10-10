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
        Motor(LS7466 *encoder, uint8_t _encAxis, uint8_t EN, uint8_t DIR_A, uint8_t DIR_B);
        // Motor init pins
        uint8_t _motorEnablePin; // Motor enable pin
        uint8_t _motorDirAPin; // Motor direction A pin
        uint8_t _motorDirBPin; // Motor direction B pin

        // Encoder init variables
        LS7466* _encoder; // Pointer to LS7466 encoder object
        uint8_t _encAxis; // Encoder number (0 or 1)
        long _encoderCount; // Motor encoder count
        int _encoderCPR = 14; // Encoder counts per revolution

        // Motor control variables
        bool reverse = false; // Reverse motor direction
        bool safety_on = true;  // Safety switch for motor control
        int _motorDutyCycle; // Motor duty cycle
        float _gearRatio = 51.0; // Gear ratio for motor
        float _targetPos = 0.0; // Target position for PID control
        float _targetVel = 0.0; // Target velocity for PID control
        float _currentPos = 0.0; // Current position
        float _currentVel = 0.0; // Current velocity
        float _currentAcc = 0.0; // Current acceleration
        float _vel_integral = 0.0; // Integral term for velocity control
        float _lastPos = 0.0; // Last position for velocity calculation
        float _lastVel = 0.0; // Last velocity for acceleration calculation
        float _lastTime = 0.0; // Last time for velocity calculation
        float _pos_error = 0.0; // Position error for PID control
        float _vel_error = 0.0; // Velocity error for PID control
        float _Kp = 0.0; // Proportional gain for PID control
        float _Ki = 0.0; // Integral gain for PID control
        float _Kd = 0.0; // Derivative gain for PID control
        float controlOutput = 0.0; // Control output for PID control

        // EWMA filtering
        float _velFiltered = 0.0;     // Filtered velocity value
        float _alpha = 0.99;           // Smoothing factor (tweakable)
        bool _velFilterInitialized = false;


        // Main control parsing function:
        MotorCommand cmd_msg; // Motor command message


        // Motor Control functions
        uint8_t _motorID; // Motor ID
        uint8_t _controlMode = 0;   // Control mode 
        void update(MotorCommand cmd_msg); // Update function to handle commands
        void controlLoop();     // Main control loop function
        void fwd_drive(int dutyCycle);  //FWD drive function
        void rev_drive(int dutyCycle);  //REV drive function
        void positionControl(float targetPos);
        void velocityControl(float targetVel);
        void brake();
        void coast();
        void estop();
        void set_Kp(float Kp);
        void set_targetVel(float targetVel);
        void set_controlMode(uint8_t mode);


        // Data retrieval functions (for returning to host computer)
        float getPos();
        float getVel();

};

// Constructor for Motor class
Motor :: Motor(LS7466 *encoder, uint8_t encAxis, uint8_t EN, uint8_t DIR_A, uint8_t DIR_B){
    // Initialize the LS7466 encoder
    _encAxis = encAxis; // Set encoder axis
    _encoder = encoder; // Assign the encoder object
    _encoder->begin();
    _encoder->configMCR0(_encAxis, BYTE_2 | EN_CNTR);
    _encoder->resetCounter(_encAxis);

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

void Motor::set_controlMode(uint8_t mode){
    _controlMode = mode;
};

void Motor::set_Kp(float Kp){
    _Kp = Kp;
};

void Motor::set_targetVel(float targetVel){
    _targetVel = targetVel;
};


// Function to forward drive the motor
void Motor::fwd_drive(int dutyCycle){
    // Set motor direction and enable
    if (reverse == true){
        digitalWrite(_motorDirAPin, LOW);
        digitalWrite(_motorDirBPin, HIGH);
    }
    else{
        digitalWrite(_motorDirAPin, HIGH);
        digitalWrite(_motorDirBPin, LOW);
    }
    analogWrite(_motorEnablePin, abs(dutyCycle));
};

// Function to reverse drive the motor
void Motor::rev_drive(int dutyCycle){
    // Set motor direction and enable
    if (reverse == true){
        digitalWrite(_motorDirAPin, HIGH);
        digitalWrite(_motorDirBPin, LOW);
    }
    else{
        digitalWrite(_motorDirAPin, LOW);
        digitalWrite(_motorDirBPin, HIGH);
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
    digitalWrite(_motorDirAPin, HIGH);
    digitalWrite(_motorDirBPin, HIGH);
    analogWrite(_motorEnablePin, 0);
};



// MAIN CONTROL LOOP
void Motor::controlLoop(){
    unsigned long currentTime = micros();
    
    // Initialize _lastTime on first run
    if (_lastTime == 0.0) {
        _lastTime = currentTime;
        _encoderCount = _encoder->readCounter(_encAxis);
        _currentPos = (_encoderCount / ((float)_encoderCPR * _gearRatio)) * 2 * PI;
        _lastPos = _currentPos;
        _currentVel = 0.0;
        _velFiltered = 0.0;
        _velFilterInitialized = true;
        return; // Skip first iteration
    }
    
    // Calculate time delta with overflow protection
    unsigned long timeDelta = currentTime - (unsigned long)_lastTime;
    
    // Skip calculation if time delta is too small (< 100 microseconds) or too large (overflow)
    if (timeDelta < 100 || timeDelta > 1000000) {
        _lastTime = currentTime;
        return;
    }
    
    // Calculate the current position
    long newEncoderCount = _encoder->readCounter(_encAxis);
    
    // Check for encoder overflow/underflow
    long encoderDelta = newEncoderCount - _encoderCount;
    if (abs(encoderDelta) > 1000000) { // Probably an overflow
        _encoderCount = newEncoderCount;
        _lastTime = currentTime;
        return;
    }
    
    _encoderCount = newEncoderCount;
    _currentPos = (_encoderCount / ((float)_encoderCPR * _gearRatio)) * 2 * PI;
    
    // Calculate velocity with safety checks
    float posDelta = _currentPos - _lastPos;
    float timeDeltaSeconds = timeDelta * 1e-6;
    
    // Safety check for reasonable values
    if (timeDeltaSeconds > 0.0001 && abs(posDelta) < 100.0) { // Reasonable position change
        float rawVel = posDelta / timeDeltaSeconds;
        
        // Limit velocity to reasonable range (e.g., ±1000 rad/s)
        if (abs(rawVel) < 1000.0) {
            // Apply EWMA filter to velocity
            if (!_velFilterInitialized) {
                _velFiltered = rawVel;
                _velFilterInitialized = true;
            } else {
                _velFiltered = _alpha * rawVel + (1.0 - _alpha) * _velFiltered;
            }
            _currentVel = _velFiltered;
        }
        // If velocity is unreasonable, keep the old value
    }
    
    // Update historical values
    _lastPos = _currentPos;
    _lastTime = currentTime;

    // Switch control loop based on control mode:
    switch(this->_controlMode){
        case 0: // ESTOP
            estop();
            break;
        case 1: // Position control
            positionControl(_targetPos);
            break;
        case 2: // Velocity control
            velocityControl(_targetVel);
            break;
        case 3: // Duty cycle control
            if (_motorDutyCycle > 0){
                fwd_drive(_motorDutyCycle);
            }
            else{
                rev_drive(_motorDutyCycle);
            }
            break;
        case 4: // Brake
            brake();
            break;
        case 5: // Coast
            coast();
            break;
        default:
            estop(); // Default to safe state instead of printing
    }

}

void Motor::estop(){
    // Emergency stop function
    // Serial.println("Emergency stop activated");
    brake(); // Brake the motor
    _controlMode = 0; // Set control mode to ESTOP
    _targetPos = 0.0; // Reset target position
    _targetVel = 0.0; // Reset target velocity
    _motorDutyCycle = 0; // Reset motor duty cycle
};

void Motor::positionControl(float targetPos){
    // Calculate the error
    _pos_error = targetPos - _currentPos;
    _vel_error = _targetVel - _currentVel;

    // Calculate the control output using PID control
    controlOutput = _Kp * _pos_error + _Kd * _vel_error; // Proportional control

    // Set the motor duty cycle based on the control output
    if (controlOutput > 0){
        fwd_drive(controlOutput);
    }
    else{
        rev_drive(controlOutput);
    }
}

void Motor::velocityControl(float targetVel){
    // Calculate the error
    _vel_error = targetVel - _currentVel;
    _vel_integral += _vel_error;
    if(_vel_error < 0.5 && _vel_error > -0.5){
        _vel_integral = 0; // Reset integral if error is small
    }

    // Calculate the control output using PID control
    float controlOutput = _Kp * _vel_error; // Proportional control

    _motorDutyCycle = controlOutput; // Update motor duty cycle

    // Set the motor duty cycle based on the control output
    if (controlOutput > 0){
        fwd_drive(controlOutput);
    }
    else{
        rev_drive(controlOutput);
    }
}

