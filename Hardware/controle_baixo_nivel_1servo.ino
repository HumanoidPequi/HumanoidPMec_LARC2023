#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <HardwareSerial.h>
#define DXL_SERIAL Serial2
#define RXD0 3
#define TXD0 1 
const int DXL_DIR_PIN = 13; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

ros::NodeHandle  nh;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  dxl.setGoalPosition(DXL_ID, cmd_msg.data);  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  dxl.begin(57600);
  //dxl.begin(57600, SERIAL_8N1, RXD0, TXD0);
  nh.initNode();
  nh.subscribe(sub);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);  
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
