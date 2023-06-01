#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <ArduinoHardware.h>
#define DXL_SERIAL Serial2
const int DXL_DIR_PIN = 13; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

ros::NodeHandle nh;

void servo_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  if (cmd_msg.data_length != DXL_ID_CNT) {
    return;
  }

  for (int i = 0; i < DXL_ID_CNT; i++) {
    dxl.setGoalPosition(DXL_ID_LIST[i], cmd_msg.data[i], UNIT_DEGREE);
    delay(0.1);
  }
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

void setup() {
  Serial.begin(57600);
  dxl.begin(1000000);
  nh.initNode();
  nh.subscribe(sub);

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < DXL_ID_CNT; i++) {
    dxl.ping(DXL_ID_LIST[i]);
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
    dxl.torqueOn(DXL_ID_LIST[i]);
  }
}

void loop() {
  nh.spinOnce();
  delay(1);
}
