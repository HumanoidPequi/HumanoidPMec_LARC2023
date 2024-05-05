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

//define id dos motores

//perna direita
const int32_t rHipYaw  =    2048; //1
const int32_t rHipRoll  =   2048; //2
const int32_t rHipPitch  =  2048; //3
const int32_t rKneePitch =  2048; //4
const int32_t rAnklePitch = 2048; //5
const int32_t rAnkleRoll =  2048;  //6

//perna esquerda
const int32_t lHipYaw   =   2048; //7
const int32_t lHipRoll  =   2048; //8
const int32_t lHipPitch =   2048; //9
const int32_t lKneePitch =  2048; //10
const int32_t lAnklePitch = 2048; //11
const int32_t lAnkleRoll  = 2048; //12

//braço direito
const int32_t rSholderPitch = 3299;  //16

//const int32_t rSholderRoll =  2204;  //17 BRAÇO RETO
const int32_t rSholderRoll =  2204 + 900;  //17 BRACO ESTICADO

const int32_t rElbowPitch  =  2048;  //18

//braço esquerdo
const int32_t lSholderPitch = 2048;   //13

//const int32_t lSholderRoll =  1991;   //14 BRAÇO RETO
const int32_t lSholderRoll = 1991 - 950; // 14 BRACO ESTICADO

const int32_t lElbowPitch  =  2048;   //15

//cabeça
const int32_t Neckyaw = 2048;         //19
const int32_t Neckpitch = 2048;       //20

const int DXL_DIR_PIN1 = 2; // pino de controle Serial1
const int DXL_DIR_PIN2 = 9; // pino de controle Serial2
const int DXL_DIR_PIN3 = 22; // pino de controle Serial3
const int DXL_DIR_PIN5 = 23; // pino de controle Serial5

bool BRACO_RETO = false;

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


const uint8_t MARTA_QTD = 12;

//LEGS
const uint8_t DXL_ID_CNT = 6;
const uint8_t QTD = 6;
//const uint8_t DXL_ID_LIST_R[QTD] = {6, 5, 4, 3, 2, 1}; //Serial1 direita
//const uint8_t DXL_ID_LIST_L[QTD] = {12, 11, 10, 9, 8, 7};//Serial2 esquerda
const uint8_t DXL_ID_LIST_R[QTD] = {1, 2, 3, 4, 5, 6}; //Serial1 direita
const uint8_t DXL_ID_LIST_L[QTD] = {7, 8, 9, 10, 11, 12};//Serial2 esquerda


//ARMS
const uint8_t DXL_ID_CNT_ARMS = 6;
const uint8_t QTD_ARMS = 6;
//const uint8_t DXL_ID_LIST_ARM[QTD_ARMS] = {15, 14, 13, 18, 17, 16};//Braços serial 3
const uint8_t DXL_ID_LIST_ARM[QTD_ARMS] = {13, 14, 15, 16, 17, 18};//Braços serial 3

//HEAD
const uint8_t DXL_ID_CNT_HEAD = 2;
const uint8_t QTD_HEAD = 2;
const uint8_t DXL_ID_LIST_HEAD[QTD_HEAD] = {20, 19};


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

sw_data_t sw_data5[DXL_ID_CNT_HEAD];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos5;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw5[DXL_ID_CNT_HEAD];


//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

//antigo
/*
bool joint_reversed[MARTA_QTD] = {false, false, true, true, true, true, 
                                  false, true, false, false, true, true, 
                                  false, true, false, false, false, true};
*/

bool joint_reversed[MARTA_QTD] = {true, true, true, true, false, false, 
                                  true, true , false, false, true, false};

// ID list: {15, 14, 13, 18, 17, 16}
//false, true, false, false, false, true --> JOINT REVERSED DOS BRAÇOS ANTIGO

// ID list novo: {13, 14, 15, 16, 17, 18}
bool joint_reversed_arms[QTD_ARMS] = {false, true, false, true, false, false};
/*

//perna direita
const int32_t rHipYaw  =    2048; //1
const int32_t rHipRoll  =   2048; //2
const int32_t rHipPitch  =  2048; //3
const int32_t rKneePitch =  2048; //4
const int32_t rAnklePitch = 2048; //5
const int32_t rAnkleRoll =  2048;  //6

//perna esquerda
const int32_t lHipYaw   =   2048; //7
const int32_t lHipRoll  =   2048; //8
const int32_t lHipPitch =   2048; //9
const int32_t lKneePitch =  2048; //10
const int32_t lAnklePitch = 2048; //11
const int32_t lAnkleRoll  = 2048; //12

//braço direito
const int32_t rSholderPitch = 1820;  //16
const int32_t rSholderRoll =  2104;  //17
const int32_t rElbowPitch  =  2048;  //18

//braço esquerdo
const int32_t lSholderPitch = 2048;   //13
const int32_t lSholderRoll =  1991;   //14
const int32_t lElbowPitch  =  2048; //15

//cabeça
const int32_t Neckyaw = 2048;         //19
const int32_t Neckpitch = 2048;       //20

bool joint_reversed[MARTA_QTD] = {false, false, true, true, true, true, 
                                  false, true, false, false, true, true, 
                                  false, false, false, false, false, false};

*/

//TOTAL
/*
int32_t marta_joints[MARTA_QTD] = {rAnkleRoll, rAnklePitch, rKneePitch, rHipPitch, rHipRoll, rHipYaw, 
                                   lAnkleRoll, lAnklePitch, lKneePitch, lHipPitch, lHipRoll, lHipYaw, 
                                   lElbowPitch, lSholderRoll, lSholderPitch, rElbowPitch, rSholderRoll, rSholderPitch}; 
*/

int32_t marta_joints[MARTA_QTD] = {rHipYaw, rHipRoll, rHipPitch, rKneePitch, rAnklePitch, rAnkleRoll, //
                                   lHipYaw, lHipRoll, lHipPitch, lKneePitch, lAnklePitch, lAnkleRoll};   

//lElbowPitch, lSholderRoll, lSholderPitch, rElbowPitch, rSholderRoll, rSholderPitch --> VETOR DE POSIÇÕES DOS BRAÇOS(UTILIZAR NO MARTA JOINTS E NO INITIAL)
/*
int32_t marta_joints_initial[MARTA_QTD] = {rAnkleRoll, rAnklePitch, rKneePitch, rHipPitch, rHipRoll, rHipYaw, 
                                           lAnkleRoll, lAnklePitch, lKneePitch, lHipPitch, lHipRoll, lHipYaw,  
                                           lElbowPitch, lSholderRoll, lSholderPitch, rElbowPitch, rSholderRoll, rSholderPitch};                                                                    
*/             
                      
int32_t marta_joints_initial[MARTA_QTD] = {rHipYaw, rHipRoll, rHipPitch, rKneePitch, rAnklePitch, rAnkleRoll, 
                                           lHipYaw, lHipRoll, lHipPitch, lKneePitch, lAnklePitch, lAnkleRoll};

//ARMS
// ID list: {13, 14, 15, 16, 17, 18}
int32_t arm_joints[QTD_ARMS] = {lSholderPitch, lSholderRoll, lElbowPitch, rSholderPitch, rSholderRoll, rElbowPitch};                                                                 
int32_t arm_joints_initial[QTD_ARMS] = {lSholderPitch, lSholderRoll, lElbowPitch, rSholderPitch, rSholderRoll, rElbowPitch};


//lElbowPitch, lSholderRoll - 1934, lSholderPitch - 1934, rElbowPitch, rSholderRoll - 1934, rSholderPitch - 1934 --> MINIMO DOS BRAÇOS                                  
//lElbowPitch, lSholderRoll + 0, lSholderPitch + 1934, rElbowPitch, rSholderRoll + 0, rSholderPitch + 1934 --> MÁXIMO DOS BRAÇOS

unsigned int minPos_arm[QTD_ARMS] = {lSholderPitch-1934, lSholderRoll-1934, lElbowPitch, rSholderPitch-1934, rSholderRoll-1934, rElbowPitch};
unsigned int maxPos_arm[QTD_ARMS] = {lSholderPitch + 1934, lSholderRoll + 0, lElbowPitch ,rSholderPitch + 1934, rSholderRoll + 0, rElbowPitch}; // angulos máximos (12Bits)/*/



//HEAD
int32_t ball_tracking[QTD_HEAD] = {Neckpitch, Neckyaw}; // 20 and 19

int32_t ball_tracking_initial[QTD_HEAD] = {Neckpitch, Neckyaw}; // 20 and 19

unsigned int minPos_head[QTD_HEAD] = {Neckpitch - 1138, Neckyaw - 1024};
unsigned int maxPos_head[QTD_HEAD] = {Neckpitch + 228, Neckyaw + 1024};

/*
unsigned int minPos[MARTA_QTD] = {rAnkleRoll-341, rAnklePitch-455, rKneePitch-1024, rHipPitch-455, rHipRoll-148, rHipYaw-341, 
                                  lAnkleRoll-341, lAnklePitch-455, lKneePitch-1024, lHipPitch-1024, lHipRoll-455, lHipYaw-341, 
                                  lElbowPitch, lSholderRoll - 1934, lSholderPitch - 1934, rElbowPitch, rSholderRoll - 1934, rSholderPitch - 1934}; // angulos minimos (12Bits)
                                  
*/
unsigned int minPos[MARTA_QTD] = {rHipYaw-341, rHipRoll-0, rHipPitch-455,rKneePitch-1024, rAnklePitch-455, rAnkleRoll-341,
                                   lHipYaw-341, lHipRoll-455, lHipPitch-1024, lKneePitch-1024, lAnklePitch-455, lAnkleRoll-341}; // angulos minimos (12Bits)

//lElbowPitch, lSholderRoll - 1934, lSholderPitch - 1934, rElbowPitch, rSholderRoll - 1934, rSholderPitch - 1934 --> MINIMO DOS BRAÇOS                                  
//lElbowPitch, lSholderRoll + 0, lSholderPitch + 1934, rElbowPitch, rSholderRoll + 0, rSholderPitch + 1934 --> MÁXIMO DOS BRAÇOS
/*                                  
unsigned int maxPos[MARTA_QTD] = {rAnkleRoll+341, rAnklePitch+455, rKneePitch+1024, rHipPitch+1024, rHipRoll+455, rHipYaw+341, 
                                  lAnkleRoll+341, lAnklePitch+455, lKneePitch+1024, lHipPitch+455, lHipRoll+148, lHipYaw+341, 
                                  lElbowPitch, lSholderRoll + 0, lSholderPitch + 1934, rElbowPitch, rSholderRoll + 0, rSholderPitch + 1934}; // angulos máximos (12Bits)/*/

unsigned int maxPos[MARTA_QTD] = {rHipYaw+341, rHipRoll+455, rHipPitch+1024,rKneePitch+1024, rAnklePitch+455, rAnkleRoll+341, 
                                  lHipYaw+341, lHipRoll+0, lHipPitch+455,lKneePitch+1024, lAnklePitch+455, lAnkleRoll+341}; // angulos máximos (12Bits)

//unsigned int minPos[MARTA_QTD] = {1707, 1593, 1024, 1593, 1900, 1707, 1707, 1593, 1024, 1593, 1900, 1707, 1000, 0}; // angulos minimos (12Bits)
//unsigned int maxPos[MARTA_QTD] = {2389, 2503, 3076, 3076, 2503, 2389, 2389, 2503, 3076, 3076, 2503, 2389, 2500, 4095}; // angulos máximos (12Bits)


//LEGS
int32_t goal_position_R[QTD] = {2048, 2048, 2048, 2048, 2048, 2048};
int32_t goal_position_L[QTD] = {2048, 2048, 2048, 2048, 2048, 2048};

//ARMS
// ID list: {13, 14, 15, 16, 17, 18}

int32_t goal_position_arm[QTD_ARMS] = {lSholderPitch, lSholderRoll, lElbowPitch, rSholderPitch, rSholderRoll, rElbowPitch};
//int32_t goal_position_arm_R[QTD_ARMS] = {2048};
//int32_t goal_position_arm_L[QTD_ARMS] = {3076};

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

int8_t reversed(bool reversed_joints){
   if (reversed_joints == true){
     return -1;
   }
   else{
    return 1;
   }
  } 

void servo_cb(const std_msgs::Int16MultiArray& cmd_msg) {
  int i;
  int mult;
  for (i = 0; i < MARTA_QTD; i++) {
    mult = reversed(joint_reversed[i]);
    marta_joints[i] = constrained(marta_joints_initial[i] + mult*convert(cmd_msg.data[i]), minPos[i], maxPos[i]);
  }

  for (i = 0; i < QTD; i++) {
    goal_position_R[i] = marta_joints[i];
    goal_position_L[i] = marta_joints[i + 6];
  }

   //for (i = 0; i < QTD_ARMS; i++) {
   // goal_position_arm[i] = marta_joints[i];
  //}
}

void head_cb(const std_msgs::Int16MultiArray& cmd_msg){
  int i;
  for(i = 0; i < QTD_HEAD; i++){
    ball_tracking[i] = constrained(ball_tracking_initial[i] + convert(cmd_msg.data[i]), minPos_head[i], maxPos_head[i]);
  }
}

void arms_cb(const std_msgs::Int16MultiArray& cmd_msg){
  int i;
  int mult_arm = reversed(joint_reversed[i]);
  for(i = 0; i < QTD_ARMS; i++){
    goal_position_arm[i] = constrained(arm_joints_initial[i] + mult_arm*convert(cmd_msg.data[i]), minPos_arm[i], maxPos_arm[i]);
  }
}



ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);
ros::Subscriber<std_msgs::Int16MultiArray> ball("ball_tracking", head_cb);
ros::Subscriber<std_msgs::Int16MultiArray> arm("arms", arms_cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(ball);
  nh.subscribe(arm);
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);

  dynamixel_setup();

  bno_setup();

  sync_write_setup();

  delay(100);
}

void loop() {
  Serial.println(lSholderRoll);
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
    sw_data3[i].goal_position = goal_position_arm[i];
    //sw_data5[i].goal_position = goal_position_arm_L[i];
  }


  // Update the SyncWrite packet status
  sw_infos3.is_info_changed = true;

  //TODO: Fazer for da HEAD
  for(i = 0; i < QTD_HEAD; i++){
    sw_data5[i].goal_position = ball_tracking[i];
  }
  // Update the SyncWrite packet status
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
    info_xels_sw3[i].id = DXL_ID_LIST_ARM[i];
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

  for (i = 0; i < QTD_HEAD; i++) {
    info_xels_sw5[i].id = DXL_ID_LIST_HEAD[i];
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
    dxl3.torqueOff(DXL_ID_LIST_ARM[i]);
    dxl3.setOperatingMode(DXL_ID_LIST_ARM[i], OP_POSITION);
  }
  dxl3.torqueOn(BROADCAST_ID);


 // Prepare the SyncRead structure
  for (i = 0; i < QTD_HEAD; i++) {
    dxl5.torqueOff(DXL_ID_LIST_HEAD[i]);
    dxl5.setOperatingMode(DXL_ID_LIST_HEAD[i], OP_POSITION);
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

/*void braco_reto(bool reto){ 
  int32_t valor1 = 2204;
  int32_t valor2 = 1991;
  if (reto == true){
      rSholderRoll = valor1; 
      lSholderRoll = valor2;
  }
  else {
      rSholderRoll = valor1+900; 
      lSholderRoll = valor2-950;
  }
}*/
