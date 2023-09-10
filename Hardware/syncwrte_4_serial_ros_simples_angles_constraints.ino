#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#define DXL_SERIAL1 Serial1
#define DXL_SERIAL2 Serial2
#define DXL_SERIAL3 Serial3
#define DXL_SERIAL5 Serial5

#define DEBUG_SERIAL Serial

const int DXL_DIR_PIN1 = 2; // pino de controle Serial1
const int DXL_DIR_PIN2 = 9; // pino de controle Serial2
const int DXL_DIR_PIN3 = 23; // pino de controle Serial3
const int DXL_DIR_PIN5 = 22; // pino de controle Serial5

ros::NodeHandle nh;

//IMU ros config
sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;


ros::Publisher pub("micro/IMU", &inertial);
char in[] = "/imu";


/*
  std_msgs::String msg;
  ros::Publisher pub("arm/position", &msg);
*/
  #define blinkPin 13

  Dynamixel2Arduino dxl1(DXL_SERIAL1, DXL_DIR_PIN1); //Perna direita
  Dynamixel2Arduino dxl2(DXL_SERIAL2, DXL_DIR_PIN2); //Perna esquerda
  Dynamixel2Arduino dxl3(DXL_SERIAL3, DXL_DIR_PIN3); //Braço direito
  Dynamixel2Arduino dxl5(DXL_SERIAL5, DXL_DIR_PIN5); //Braço esquerdo

  BNO080 myIMU;

  /* syncRead
  Structures containing the necessary information to process the 'syncRead' packet.

  typedef struct XELInfoSyncRead{
    uint8_t *p_recv_buf;
    uint8_t id;
    uint8_t error;
  } __attribute__((packed)) XELInfoSyncRead_t;

  typedef struct InfoSyncReadInst{
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncRead_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoSyncReadInst_t;
*/

/* syncWrite
  Structures containing the necessary information to process the 'syncWrite' packet.

  typedef struct XELInfoSyncWrite{
    uint8_t* p_data;
    uint8_t id;
  } __attribute__((packed)) XELInfoSyncWrite_t;

  typedef struct InfoSyncWriteInst{
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncWrite_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoSyncWriteInst_t;
*/

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 1.0;

//LEGS
const uint8_t DXL_ID_CNT = 6;
const uint8_t QTD = 6;
const uint8_t DXL_ID_LIST_R[QTD] = {6, 5, 4, 3, 2, 1}; //Serial1 direita
const uint8_t DXL_ID_LIST_L[QTD] = {12, 11, 10, 9, 8, 7};//Serial2 esquerda

//ARMS
const uint8_t DXL_ID_CNT_ARMS = 1;
const uint8_t QTD_ARMS = 1;
const uint8_t MARTA_QTD = 14;
const uint8_t DXL_ID_LIST_ARM_R[QTD] = {15};//Serial5 braço esquerdo
const uint8_t DXL_ID_LIST_ARM_L[QTD] = {18};//Serial3 braço direito


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
typedef struct sr_data {
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data {
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data1[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos1;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw1[DXL_ID_CNT];

sw_data_t sw_data2[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos2;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw2[DXL_ID_CNT];

sw_data_t sw_data3[DXL_ID_CNT_ARMS];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos3;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw3[DXL_ID_CNT_ARMS];

sw_data_t sw_data5[DXL_ID_CNT_ARMS];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos5;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw5[DXL_ID_CNT_ARMS];


//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

bool joint_reversed[MARTA_QTD] = {false, false, false, false, false, false, false, false, false, false, false, false, false};

//TO-DO --> ARRUMAR MINIMOS E MÁXIMOS DOS SERVOS CONSIDERANDO O ANGULO INICIAL DE CADA UM DELES
//TOTAL
int32_t marta_joints[MARTA_QTD] = {853, 1820, 2048, 2048, 1024, 2048, 2048, 2048, 2048, 2048, 2048, 1593, 2048, 3076}; //last two are id 15 and 18 respectivelly
int32_t marta_joints_initial[MARTA_QTD] = {853, 1820, 2048, 2048, 1024, 2048, 2048, 2048, 2048, 2048, 2048, 1593, 2048, 3076}; //last two are id 15 and 18 respectivelly
unsigned int minPos[MARTA_QTD] = {1707, 1593, 1024, 1593, 1900, 1707, 1707, 1593, 1024, 1593, 1900, 1707, 1000, 0}; // angulos minimos (12Bits)
unsigned int maxPos[MARTA_QTD] = {2389, 2503, 3076, 3076, 2503, 2389, 2389, 2503, 3076, 3076, 2503, 2389, 2500, 4095}; // angulos máximos (12Bits)


//LEGS
int32_t goal_position_R[QTD] = {853, 1820, 2048, 2048, 1024, 2048};
int32_t goal_position_L[QTD] = {2048, 2048, 2048, 2048, 2048, 1593};

//ARMS
int32_t goal_position_arm_R[QTD_ARMS] = {2048};
int32_t goal_position_arm_L[QTD_ARMS] = {3076};

int convert(int ang_postiton /*, int32_t marta_joints_initial, bool joint_reversed*/) {
  //Converte os valores de 16 bits recebidos pelo rosserial para os valores de 12 bits
  int relative_position = map(ang_postiton, -1800, 1800, -2048, 2048);

  // Faz o offset com os angulos iniciais
  /*  if (joint_reversed) {
      return base_position - relative_position;
    }
    return relative_position + base_position;*/
  return relative_position;
}

unsigned int constrained(int32_t val, unsigned int out_min, unsigned int out_max ) {
  if (val < out_min) return out_min;
  if (val > out_max) return out_max;
  return val;
}

void servo_cb(const std_msgs::Int16MultiArray& cmd_msg) {
  int i;
  for (i = 0; i < MARTA_QTD; i++) {
    marta_joints[i] = constrained(marta_joints_initial[i] + convert(cmd_msg.data[i]), minPos[i], maxPos[i]);
  }

  for (i = 0; i < QTD; i++) {
    goal_position_R[i] = marta_joints[i];
    goal_position_L[i] = marta_joints[i + 6];
  }

  for (i = 0; i < QTD_ARMS; i++) {
    goal_position_arm_R[i] = marta_joints[i + 12];
    goal_position_arm_L[i] = marta_joints[i + 13];
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);

  dynamixel_setup();

  bno_setup();

  sync_write_setup();

  delay(100);
}

void loop() {
  dxl1.writeControlTableItem(MOVING_SPEED, BROADCAST_ID, 0);
  dxl2.writeControlTableItem(MOVING_SPEED, BROADCAST_ID, 0);
  dxl3.writeControlTableItem(MOVING_SPEED, BROADCAST_ID, 0);
  dxl5.writeControlTableItem(MOVING_SPEED, BROADCAST_ID, 0);

  bno_loop();

  nh.spinOnce();
  // put your main code here, to run repeatedly:
  static uint32_t try_count = 0;
  uint8_t i = 0, recv_cnt;
  // Insert a new Goal Position to the SyncWrite Packet
  for (i = 0; i < QTD; i++) {
    sw_data1[i].goal_position = goal_position_R[i];
    sw_data2[i].goal_position = goal_position_L[i];
  }

  // Update the SyncWrite packet status
  sw_infos1.is_info_changed = true;
  sw_infos2.is_info_changed = true;


  for (i = 0; i < QTD_ARMS; i++) {
    sw_data3[i].goal_position = goal_position_arm_R[i];
    sw_data5[i].goal_position = goal_position_arm_L[i];
  }

  // Update the SyncWrite packet status
  sw_infos3.is_info_changed = true;
  sw_infos5.is_info_changed = true;

  //sending sync write packet
  dxl1.syncWrite(&sw_infos1);
  dxl2.syncWrite(&sw_infos2);
  dxl3.syncWrite(&sw_infos3);
  dxl5.syncWrite(&sw_infos5);

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  delayMicroseconds(250);
}

////////////////////////////////////////////////////////////////////////////////
//FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void sync_write_setup() {
  int i;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos1.packet.p_buf = nullptr;
  sw_infos1.packet.is_completed = false;
  sw_infos1.addr = SW_START_ADDR;
  sw_infos1.addr_length = SW_ADDR_LEN;
  sw_infos1.p_xels = info_xels_sw1;
  sw_infos1.xel_count = 0;

  for (i = 0; i < QTD; i++) {
    info_xels_sw1[i].id = DXL_ID_LIST_R[i];
    info_xels_sw1[i].p_data = (uint8_t*)&sw_data1[i].goal_position;
    sw_infos1.xel_count++;
  }
  sw_infos1.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos2.packet.p_buf = nullptr;
  sw_infos2.packet.is_completed = false;
  sw_infos2.addr = SW_START_ADDR;
  sw_infos2.addr_length = SW_ADDR_LEN;
  sw_infos2.p_xels = info_xels_sw2;
  sw_infos2.xel_count = 0;

  for (i = 0; i < QTD; i++) {
    info_xels_sw2[i].id = DXL_ID_LIST_L[i];
    info_xels_sw2[i].p_data = (uint8_t*)&sw_data2[i].goal_position;
    sw_infos2.xel_count++;
  }
  sw_infos2.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos3.packet.p_buf = nullptr;
  sw_infos3.packet.is_completed = false;
  sw_infos3.addr = SW_START_ADDR;
  sw_infos3.addr_length = SW_ADDR_LEN;
  sw_infos3.p_xels = info_xels_sw3;
  sw_infos3.xel_count = 0;

  for (i = 0; i < QTD_ARMS; i++) {
    info_xels_sw3[i].id = DXL_ID_LIST_ARM_R[i];
    info_xels_sw3[i].p_data = (uint8_t*)&sw_data3[i].goal_position;
    sw_infos3.xel_count++;
  }
  sw_infos3.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos5.packet.p_buf = nullptr;
  sw_infos5.packet.is_completed = false;
  sw_infos5.addr = SW_START_ADDR;
  sw_infos5.addr_length = SW_ADDR_LEN;
  sw_infos5.p_xels = info_xels_sw5;
  sw_infos5.xel_count = 0;

  for (i = 0; i < QTD_ARMS; i++) {
    info_xels_sw5[i].id = DXL_ID_LIST_ARM_L[i];
    info_xels_sw5[i].p_data = (uint8_t*)&sw_data5[i].goal_position;
    sw_infos5.xel_count++;
  }
  sw_infos5.is_info_changed = true;
}

void dynamixel_setup() {
  int i;
  DEBUG_SERIAL.begin(1000000);
  dxl1.begin(1000000);
  dxl1.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  dxl2.begin(1000000);
  dxl2.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  dxl3.begin(1000000);
  dxl3.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);


  dxl5.begin(1000000);
  dxl5.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  // Prepare the SyncRead structure
  for (i = 0; i < QTD; i++) {
    dxl1.torqueOff(DXL_ID_LIST_R[i]);
    dxl1.setOperatingMode(DXL_ID_LIST_R[i], OP_POSITION);
  }
  dxl1.torqueOn(BROADCAST_ID);

  // Prepare the SyncRead structure
  for (i = 0; i < QTD; i++) {
    dxl2.torqueOff(DXL_ID_LIST_L[i]);
    dxl2.setOperatingMode(DXL_ID_LIST_L[i], OP_POSITION);
  }
  dxl2.torqueOn(BROADCAST_ID);


  // Prepare the SyncRead structure
  for (i = 0; i < QTD_ARMS; i++) {
    dxl3.torqueOff(DXL_ID_LIST_ARM_R[i]);
    dxl3.setOperatingMode(DXL_ID_LIST_ARM_R[i], OP_POSITION);
  }
  dxl3.torqueOn(BROADCAST_ID);


  // Prepare the SyncRead structure
  for (i = 0; i < QTD_ARMS; i++) {
    dxl5.torqueOff(DXL_ID_LIST_ARM_L[i]);
    dxl5.setOperatingMode(DXL_ID_LIST_ARM_L[i], OP_POSITION);
  }
  dxl5.torqueOn(BROADCAST_ID);
}
void bno_setup() {
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

  nh.advertise(pub);

  digitalWrite(blinkPin, LOW);
}

void bno_loop() {
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
    //        digitalWrite(blinkPin, LOW);
  }
  pub.publish(&inertial);
  delay(1);
}
