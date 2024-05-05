#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Header.h>
#include "ESP32SerialPortHandler.cpp" 

#define BROADCAST_ID 254

Dynamixel2Arduino dxl; //Perna direita
Dynamixel2Arduino dxl2; //Perna esquerda

ESP32SerialPortHandler esp_dxl_port(Serial1, 26, 27, 12);
//ESP32SerialPortHandler esp_dxl_port(Serial1, 3, 1, 12);
ESP32SerialPortHandler esp_dxl2_port(Serial2, 16, 17, 13);

#define LEG_ID 6 //this variable was created only to be used in for.
#define RIGHT_ID 6 //this variable was created only to be used in for.
#define LEFT_ID 12 //this variable was created only to be used in for.
#define DXL_ID_CNT 12
uint8_t LEGS_ID[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
uint8_t RIGHT_LEG_ID[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6};
uint8_t LEFT_LEG_ID[DXL_ID_CNT] = {7, 8, 9, 10, 11, 12};
uint8_t right_leg[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72};
uint8_t left_leg[DXL_ID_CNT] = {180, 180, 180, 180, 180, 180};
uint8_t legs[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72, 180, 180, 180, 180, 180, 180};
uint8_t legs_initial[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72, 180, 180, 180, 180, 180, 180};

using namespace ControlTableItem;

ros::NodeHandle nh;

void servo_cb(const std_msgs::Int16MultiArray& cmd_msg){
  for(int i = 0; i < DXL_ID_CNT; i++){
    legs[i] = legs_initial[i] + cmd_msg.data[i];
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);

// Position PID Gains
// Adjust these gains to tune the behavior of DYNAMIXEL
uint16_t position_p_gain = 40;
uint16_t position_i_gain = 10;
uint16_t position_d_gain = 1;

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  //Serial.begin(57600);

  dxl.setPort(esp_dxl_port);
  dxl2.setPort(esp_dxl2_port);
    
  dxl.begin(1000000);
  dxl2.begin(1000000);

  dxl.setPortProtocolVersion(1.0);
  dxl2.setPortProtocolVersion(1.0);

  // Set Position PID Gains(Serial 1)
  dxl.writeControlTableItem(POSITION_P_GAIN, BROADCAST_ID, position_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, BROADCAST_ID, position_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, BROADCAST_ID, position_d_gain);

  // Set Position PID Gains(Serial 1)
  dxl2.writeControlTableItem(POSITION_P_GAIN, BROADCAST_ID, position_p_gain);
  dxl2.writeControlTableItem(POSITION_I_GAIN, BROADCAST_ID, position_i_gain);
  dxl2.writeControlTableItem(POSITION_D_GAIN, BROADCAST_ID, position_d_gain);
  
  for(int i = 0; i < RIGHT_ID; i++){
    dxl.setGoalPosition(LEGS_ID[i], legs_initial[i], UNIT_DEGREE);
    delay(0.001);
  }
  
  for(int i = 6; i < DXL_ID_CNT; i++){
    dxl2.setGoalPosition(LEGS_ID[i], legs_initial[i], UNIT_DEGREE);
    delay(0.001);
  }

  xTaskCreatePinnedToCore(
    tarefa1,   /* função que implementa a tarefa */
    "tarefa1", /* nome da tarefa */
    10000,     /* número de palavras a serem alocadas para uso com a pilha da tarefa */
    NULL,      /* parâmetro de entrada para a tarefa (pode ser NULL) */
    1,         /* prioridade da tarefa (0 a N) */
    NULL,      /* referência para a tarefa (pode ser NULL) */
    0);        /* Núcleo que executará a tarefa */

  //delay(500);  //tempo para a tarefa iniciar

  xTaskCreatePinnedToCore(
    tarefa2,   /* função que implementa a tarefa */
    "tarefa2", /* nome da tarefa */
    10000,     /* número de palavras a serem alocadas para uso com a pilha da tarefa */
    NULL,      /* parâmetro de entrada para a tarefa (pode ser NULL) */
    1,         /* prioridade da tarefa (0 a N) */
    NULL,      /* referência para a tarefa (pode ser NULL) */
    1);        /* Núcleo que executará a tarefa */

  delay(500);  //tempo para a tarefa iniciar

}

void loop() {

}

void tarefa1(void* pvParameters) {
  while(true){
    nh.spinOnce();
    for(int i = 0; i < RIGHT_ID; i++){
      //dxl.setGoalPosition(LEGS_ID[i], legs[i], UNIT_DEGREE);
      dxl.setGoalPosition(1, 20, UNIT_DEGREE);
      delay(0.001);
    }
  }
}

void tarefa2(void* pvParameters) {
  while(true){
    nh.spinOnce();
    for(int i = 6; i < DXL_ID_CNT; i++){
      //dxl2.setGoalPosition(LEGS_ID[i], legs[i], UNIT_DEGREE);
      dxl2.setGoalPosition(2, 30, UNIT_DEGREE);
      delay(0.001);
    }
  }
}
