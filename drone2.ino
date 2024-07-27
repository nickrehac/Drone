#include <ESP8266WiFi.h>
#include "sensor.h"
#include "control.h"


FlightData flightData;
FlightControl flightControl;

unsigned long lastTime = 0;

WiFiServer server(333);
WiFiClient client;


void handle_request() {
  
  int numToRead = client.available();
  if(!numToRead) return;
  //Serial.println(numToRead);
  byte firstByte = client.read();
  //Serial.println(firstByte);
  float buf[4];
  
  if(firstByte == 0b11111111) {
    
    client.write(0b10101010);
    
  }
  else if(firstByte & 0b10000000) {//if is telemetry request
    
    firstByte &= 0b01111111;
    
    switch(firstByte) {
      
      case 0: {
        Quaternion attitude = flightData.getAttitude();
        client.write((char *) &attitude, sizeof(Quaternion));
        break;
      }

      case 1: {
        float motors[4];
        client.write((char *) motors, sizeof(motors));
        break;
      }

      case 2: {
        float battery = 1.0f;
        client.write((char *) &battery, sizeof(float));
        break;
      }

      default:
      break;
      
    }

  }
  else {//if is flight command
    
    switch(firstByte) {
      
      case 0:
        client.read((char*)buf, 5*sizeof(float));
        flightControl.setTarget(*(Quaternion *)buf);
        flightControl.throttle = buf[3];
        break;
        
      case 1:
        client.read((char*)buf, sizeof(float));
        flightControl.throttle = *buf;
        break;

      case 2:
        client.read((char*)buf, sizeof(float));
        flightControl.pitchPID.target = *buf;
        break;
      
      default:
      break;
      /*
      case 2:
      client.read(buf, 4);
      //control_pitch = getUnalignedFloat(buf);
      memcpy(&control_pitch, buf, 4);
      pitch_pid.setTarget(control_pitch);
      break;
      case 3:
      client.read(buf, 4);
      //control_roll = getUnalignedFloat(buf);
      memcpy(&control_roll, buf, 4);
      roll_pid.setTarget(control_roll);
      break;
      case 4:
      client.read(buf, 4);
      control_yaw = getUnalignedFloat(buf);
      break;
      //TODO: trim
      default:
      break;
      */
    }
  }
}


void setup() {

  pinMode(0, OUTPUT);
  
  Serial.begin(9600);
  
  initSensor();

  delay(5000);
  
  Serial.print("whoami returns: ");
  Serial.println(readReg(0x75));
  Serial.print("Sleep register: ");
  Serial.println(readReg(PWR_ADDR));

  flightData.calibrate();
  Serial.println("CALIBRATED");

  bool apUP = WiFi.softAP("ESPBT");
  bool apCON = WiFi.softAPConfig(IPAddress(192,168,0,1),IPAddress(192,168,0,1),IPAddress(255,255,255,0));
  server.begin();
  server.setNoDelay(true);

  lastTime = micros();
  
}


void loop() {

  unsigned long curTime = micros();
  float dtime = (curTime - lastTime) / 1000000.0f;
  lastTime = curTime;

  if(client.connected()) {
    if(client.available()) {
      handle_request();
      if(client.available()) digitalWrite(0, HIGH);
    }
    else {
      digitalWrite(0, LOW);
    }
  }
  else {
    client = server.accept();
  }
  
  flightData.update(dtime);
  flightControl.update(flightData.getAttitude(), dtime);

  flightData.getAttitude().print();


  
    
  //flightData.isolatedAccel().print();

  
  Serial.println();
  
}
