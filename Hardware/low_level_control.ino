#define USE_USBCON
#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#define DXL_SERIAL Serial2
const int DXL_DIR_PIN = 13; // DYNAMIXEL Shield DIR PIN
int LED_BUILTIN = 2; 
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

ros::NodeHandle  nh;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  dxl.setGoalPosition(DXL_ID, cmd_msg.data);  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.write("hello");
  // put your setup code here, to run once:
  pinMode (LED_BUILTIN, OUTPUT); 
  digitalWrite (LED_BUILTIN, HIGH);
  nh.getHardware()->setBaud(115200);
  dxl.begin(9600);
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
  delay(2000);
}

void loop() {
  digitalWrite (LED_BUILTIN, LOW);
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
