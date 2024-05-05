#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// Please modify it to suit your hardware.
#define DEBUG_SERIAL Serial
#define DXL_SERIAL1 Serial1
#define DXL_SERIAL2 Serial2
#define DXL_SERIAL3 Serial3
#define DXL_SERIAL5 Serial5

#define blinkPin 13

const int DXL_DIR_PIN1 = 2; // pino de controle Serial1
const int DXL_DIR_PIN2 = 9; // pino de controle Serial2
const int DXL_DIR_PIN3 = 23; // pino de controle Serial3
const int DXL_DIR_PIN5 = 22; // pino de controle Serial5

ros::NodeHandle nh;

sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;
ros::Publisher pub("micro/IMU", &inertial);
char in[] = "/imu";

Dynamixel2Arduino dxl1(DXL_SERIAL1, DXL_DIR_PIN1); //Perna direita
Dynamixel2Arduino dxl2(DXL_SERIAL2, DXL_DIR_PIN2); //Perna esquerda
Dynamixel2Arduino dxl3(DXL_SERIAL3, DXL_DIR_PIN3); //Perna esquerda
Dynamixel2Arduino dxl5(DXL_SERIAL5, DXL_DIR_PIN5); //Perna direita

BNO080 myIMU;

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 1.0;
const uint8_t DXL_ID_CNT = 6;
const uint8_t DXL_ID_CNT_12 = 12;
//const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {15, 18}; //braços
//const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6}; //perna direita
const uint8_t DXL_ID_LIST[DXL_ID_CNT_12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t DXL_ID_LIST_L[DXL_ID_CNT] = {6, 8, 9, 10, 11, 12}; //perna esquerda
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 38;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 2;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 30;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 2;
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

/*
typedef struct sw_data1{
  int32_t goal_position1;
} __attribute__((packed)) sw_data_t1;
*/

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data1[DXL_ID_CNT];
sw_data_t sw_data2[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos1;
DYNAMIXEL::InfoSyncWriteInst_t sw_infos2;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw1[DXL_ID_CNT];
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw2[DXL_ID_CNT];

//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

int32_t goal_position[DXL_ID_CNT_12] = {        856, 1820, 2048, 2048, 1024, 2048,     2048, 2048, 2048, 2048, 2048, 1593};
int32_t goal_position_initial[DXL_ID_CNT_12] = {856, 1820, 2048, 2048, 1024, 2048,     2048, 2048, 2048, 2048, 2048, 1593};
int32_t goal_position1[DXL_ID_CNT_12] = {856, 1820, 2048, 2048, 1024, 2048,     2048, 2048, 2048, 2048, 2048, 1593};

void servo_cb(const std_msgs::Int16MultiArray& cmd_msg){
  for(int i; i < DXL_ID_CNT; i++){
  goal_position[i] = goal_position_initial[i] + cmd_msg.data[i];
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  bno_setup();
  
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(1000000);
  dxl1.begin(1000000);
  dxl2.begin(1000000);
  
  dxl1.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
  dxl2.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

// Prepare the SyncRead structure
  for(i = 0; i < DXL_ID_CNT; i++){
    dxl1.torqueOff(DXL_ID_LIST[i]);
    dxl1.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  for(i = 6; i < DXL_ID_CNT_12; i++){
    dxl2.torqueOff(DXL_ID_LIST[i]);
    dxl2.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl1.torqueOn(BROADCAST_ID);
  dxl2.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos1.packet.p_buf = nullptr;
  sw_infos1.packet.is_completed = false;
  sw_infos1.addr = SW_START_ADDR;
  sw_infos1.addr_length = SW_ADDR_LEN;
  sw_infos1.p_xels = info_xels_sw1;
  sw_infos1.xel_count = 0;

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sw1[i].id = DXL_ID_LIST[i];
    info_xels_sw1[i].p_data = (uint8_t*)&sw_data1[i].goal_position;
    sw_infos1.xel_count++;
  }

  sw_infos1.is_info_changed = true;

 
  sw_infos2.packet.p_buf = nullptr;
  sw_infos2.packet.is_completed = false;
  sw_infos2.addr = SW_START_ADDR;
  sw_infos2.addr_length = SW_ADDR_LEN;
  sw_infos2.p_xels = info_xels_sw2;
  sw_infos2.xel_count = 0;

  
  for(i = 6; i < DXL_ID_CNT_12; i++){
    info_xels_sw2[i].id = DXL_ID_LIST[i];
    info_xels_sw2[i].p_data = (uint8_t*)&sw_data2[i].goal_position;
    sw_infos2.xel_count++;
  }
  
  sw_infos2.is_info_changed = true;

}

void loop() {
  nh.spinOnce();
  // put your main code here, to run repeatedly:
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;
  
  // Insert a new Goal Position to the SyncWrite Packet
  for(i = 0; i < DXL_ID_CNT; i++){
    nh.spinOnce();
    sw_data1[i].goal_position = goal_position[i];
  }
  sw_infos1.is_info_changed = true;
  dxl1.syncWrite(&sw_infos1);

  delay(1);

  for(i = 6; i < DXL_ID_CNT_12; i++){
    nh.spinOnce();
    sw_data2[i].goal_position = goal_position_initial[i];
  }
  
  // Update the SyncWrite packet status
  sw_infos2.is_info_changed = true;
  dxl2.syncWrite(&sw_infos2); 
 
 
  /*DEBUG_SERIAL.print("Serial 1:");
  DEBUG_SERIAL.println(dxl1.syncWrite(&sw_infos1));
  //delayMicroseconds(250);
  DEBUG_SERIAL.print("Serial 2:");
  DEBUG_SERIAL.println(dxl2.syncWrite(&sw_infos2));*/
  
  bno_loop();
  
  delayMicroseconds(250);
//  for(i = 0; i < 50001; i++){
//    ;
//  }
}

void bno_setup(){
  pinMode(blinkPin, OUTPUT);
    Wire.begin();
    digitalWrite(blinkPin, LOW);
    if (myIMU.begin() == false)
    {
        digitalWrite(blinkPin, HIGH);
        delay(1000);
        digitalWrite(blinkPin, LOW);
        delay(1000);
        digitalWrite(blinkPin, HIGH);
        while (1)
            ;
    }
    delay(10);
    Wire.setClock(400000); // Increase I2C data rate to 400kHz
    delay(10);
    myIMU.enableLinearAccelerometer(8); // m/s^2 no gravity
    myIMU.enableRotationVector(4);      // quat
    myIMU.enableGyro(12);               // rad/s

    nh.initNode();
    nh.advertise(pub);

    digitalWrite(blinkPin, LOW);
}

void bno_loop(){
  inertial.header.frame_id = in;
    inertial.header.stamp = nh.now();
    if (myIMU.dataAvailable() == true)
    {
        digitalWrite(blinkPin, HIGH);
        // internal copies of the IMU data
        byte linAccuracy = 0;
        byte gyroAccuracy = 0;
        float quatRadianAccuracy = 0;
        byte quatAccuracy = 0;

        // get IMU data in one go for each sensor type
        myIMU.getLinAccel(lin.x, lin.y, lin.z, linAccuracy);
        myIMU.getGyro(ang.x, ang.y, ang.z, gyroAccuracy);
        myIMU.getQuat(inertial.orientation.x, inertial.orientation.y, inertial.orientation.z, inertial.orientation.w, quatRadianAccuracy, quatAccuracy);
        inertial.angular_velocity = ang;
        inertial.linear_acceleration = lin;
        pub.publish(&inertial);
        digitalWrite(blinkPin, LOW);
    }
    pub.publish(&inertial);
    delay(1);
}
  // Build a SyncWrite Packet and transmit to DYNAMIXEL  
  /*if(dxl.syncWrite(&sw_infos) == true){
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for(i = 0; i<sw_infos.xel_count; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.println(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Position: ");DEBUG_SERIAL.println(sw_data[i].goal_position);
    }
    if(goal_position_index == 0)
      goal_position_index = 1;
    else
      goal_position_index = 0;
  } else {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(250);


  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  recv_cnt = dxl.syncRead(&sr_infos);
  if(recv_cnt > 0) {
    DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for(i = 0; i<recv_cnt; i++){
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", Error: ");
      DEBUG_SERIAL.println(sr_infos.p_xels[i].error);
      DEBUG_SERIAL.print("\t Present Position: ");
      DEBUG_SERIAL.println(sr_data[i].present_position);
    }
  }else{
    DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(750);
}*/
