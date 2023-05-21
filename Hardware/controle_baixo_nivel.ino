#include <Dynamixel2Arduino.h>
#include <actuator.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <ArduinoHardware.h>;
#define USE_USBCON
#define DXL_SERIAL Serial
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

//const uint8_t DXL_ID = 1;
const uint8_t BROADCAST_ID = 254;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
//const uint16_t user_pkt_buf_cap = 128;
//uint8_t user_pkt_buf[user_pkt_buf_cap];

/*// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;*/
typedef struct sw_data{
  uint8_t* goal_position;
} __attribute__((packed)) sw_data_t;


/*sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];*/

sw_data sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

ros::NodeHandle  nh;

uint8_t** goal_position;
uint8_t goal_position_index = 0;
uint8_t i;

/*void servo_cb( const std_msgs::UInt16& cmd_msg){
  dxl.setGoalPosition(DXL_ID_LIST, cmd_msg.data);  
}*/ 

  void servo_cb( const std_msgs::UInt16& cmd_msg){
  // Fill the members of structure to syncWrite using internal packet buffer
  //sw_infos.packet.p_buf = nullptr;
  //sw_infos.packet.is_completed = false;
  //sw_infos.addr = SW_START_ADDR;
  //sw_infos.addr_length = SW_ADDR_LEN;
  //sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;
  uint16_t conv_data = cmd_msg.data;
  //goal_position = new uint8_t*[DXL_ID_CNT];
   for (i = 0; i < DXL_ID_CNT; i++) {
    goal_position[i] = new uint8_t[sizeof(uint16_t)];
    memcpy(goal_position[i], &cmd_msg.data, sizeof(uint16_t));
  }

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&conv_data;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;

  static uint32_t try_count = 0;
  uint8_t recv_cnt;
  
  // Insert a new Goal Position to the SyncWrite Packet
  for(i = 0; i < DXL_ID_CNT; i++){
    sw_data[i].goal_position = (uint8_t*)&goal_position[goal_position_index];
  }

  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;

  DXL_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
  DXL_SERIAL.println(try_count++);
  
  // Build a SyncWrite Packet and transmit to DYNAMIXEL  
  if(dxl.syncWrite(&sw_infos) == true){
    DXL_SERIAL.println("[SyncWrite] Success");
    for(i = 0; i<sw_infos.xel_count; i++){
      //DXL_SERIAL.print("  ID: ");DXL_SERIAL.println(sw_infos.p_xels[i].id);
      //DXL_SERIAL.print("\t Goal Position: ");DXL_SERIAL.println(String(sw_data[i].goal_position));
      DXL_SERIAL.print("Congratulations!");
    }
    if(goal_position_index == 0)
      goal_position_index = 1;
    else
      goal_position_index = 0;
  } else {
    DXL_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DXL_SERIAL.print(dxl.getLastLibErrCode());
  }
  DXL_SERIAL.println();

  delay(250);
  for (int i = 0; i < DXL_ID_CNT; i++) {
    delete[] goal_position[i];
  }
  delete[] goal_position;
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup() {
  // put your setup code here, to run once:
  dxl.begin(57600);
  goal_position = new uint8_t*[DXL_ID_CNT];
    for (int i = 0; i < DXL_ID_CNT; i++) {
    goal_position[i] = new uint8_t[sizeof(uint16_t)];
  }
  nh.initNode();
  nh.subscribe(sub);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_LIST[i]);
  for(i = 0; i < DXL_ID_CNT; i++){ 
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);
}
  
void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
