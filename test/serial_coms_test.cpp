// Teensy sketch

#include <Arduino.h>


enum Cmd : uint8_t { CMD_SET_VEL=1, CMD_GET_VEL=2, /*…*/ };

void handleLine(const String& l);

void setup() {
  Serial.begin(115200);
}

void loop() {
  // 1) Read a whole line (ASCII example)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()>0) handleLine(line);
  }

  // 2) ... do your motor control in background ...
}

void handleLine(const String& l) {
  // parse CSV: motorID,cmd,value
  int parts = 0;
  uint8_t motorID = l.toInt();            // before first comma
  int p1 = l.indexOf(',');
  int p2 = l.indexOf(',', p1+1);
  String cmd = l.substring(p1+1, p2);
  float   val = l.substring(p2+1).toFloat();
  float v = 2.5;

  if (cmd == "setVel") {
    Serial.printf("ACK,%u,setVel\n", motorID);
  }
  else if (cmd == "getVel") {
    Serial.printf("RPT,%u,vel,%.3f\n", motorID, v);
  }
  // … add more commands …
}