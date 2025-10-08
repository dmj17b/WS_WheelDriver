#include <Arduino.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"


// Motor Command Structure
// MotorID
// Command (e.g., setVel)
// Value (e.g., 0.5 for 50% speed)
struct MotorCommand{
	uint8_t motorID;
	String cmd;
	float val;
};

// Data structure for sending back to the host
struct MotorData{
	uint8_t motorID;
	String param;
	float val;
};



// Class to handle serial communication
// and parse commands for motor control
class Comms{
    public:
        Comms();
        void init();
        MotorCommand handleLine(const String& l);
				// Define motor cmd and getCMD function
				MotorCommand cmd_msg;
				MotorCommand getCMD();

		private:
				// Private members if needed
};

Comms::Comms(){
    // Constructor
}

void Comms::init(){
    // Initialization code if needed
    Serial.begin(115200); // Start serial communication at 115200 baud rate
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB devices
    }
}

/* General function used for receiving serial commands*/
MotorCommand Comms::getCMD(){
    // Read a whole line (ASCII example)
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length()>0){
					return handleLine(line);
				} 
    }
    else{
        cmd_msg.motorID = 0;
        cmd_msg.cmd = "none";
        cmd_msg.val = 0;
    }
    
    return cmd_msg;
}

/* Function used to parse a serial command*/
MotorCommand Comms::handleLine(const String& l) {
    // parse CSV: motorID,cmd,value
    uint8_t motorID = l.toInt();
    int p1 = l.indexOf(',');
    int p2 = l.indexOf(',', p1+1);
    String cmd = l.substring(p1+1, p2);
    float   val = l.substring(p2+1).toFloat();
		cmd_msg.motorID = motorID;
		cmd_msg.cmd = cmd;
		cmd_msg.val = val;
  
    return cmd_msg;
    // … add more commands …

  }