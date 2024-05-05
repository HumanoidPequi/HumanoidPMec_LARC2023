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

#define BROADCAST_ID 254
#define DXL_SERIAL1 Serial1
#define DXL_SERIAL2 Serial2
#define DXL_SERIAL3 Serial3
#define DXL_SERIAL5 Serial5
#define blinkPin 13

const int DXL_DIR_PIN1 = 2; // pino de controle Serial1
const int DXL_DIR_PIN2 = 9; // pino de controle Serial2
const int DXL_DIR_PIN3 = 23; // pino de controle Serial3
const int DXL_DIR_PIN5 = 22; // pino de controle Serial5

Dynamixel2Arduino dxl1(DXL_SERIAL1, DXL_DIR_PIN1); //Perna direita
Dynamixel2Arduino dxl2(DXL_SERIAL2, DXL_DIR_PIN2); //Perna esquerda
Dynamixel2Arduino dxl3(DXL_SERIAL3, DXL_DIR_PIN3); //Perna esquerda
Dynamixel2Arduino dxl5(DXL_SERIAL5, DXL_DIR_PIN5); //Perna direita

BNO080 myIMU;

#define LEG_ID 6 //this variable was created only to be used in for.
#define RIGHT_ID 6 //this variable was created only to be used in for.
#define RIGHT_ID_TEMPORARIO 6 
#define LEFT_ID 12 //this variable was created only to be used in for.
#define DXL_ID_CNT 12
#define DXL_ID_CNT_TEMPORARIO 12
uint8_t LEGS_ID[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
uint8_t LEGS_ID_TEMPORARIO[DXL_ID_CNT] = {6, 5, 4, 3, 2, 1, 12, 11, 10, 9, 8, 7};
uint8_t RIGHT_LEG_ID[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6};
uint8_t LEFT_LEG_ID[DXL_ID_CNT] = {7, 8, 9, 10, 11, 12};
uint8_t right_leg[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72};
uint8_t left_leg[DXL_ID_CNT] = {180, 180, 180, 180, 180, 180};
uint8_t legs[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72, 180, 180, 180, 180, 180, 180};
uint8_t legs_temporario[DXL_ID_CNT] =         {75, 160, 180, 180, 90, 180,    180, 180, 180, 180, 180, 140};
uint8_t legs_initial_temporario[DXL_ID_CNT] = {75, 160, 180, 180, 90, 180,    180, 180, 180, 180, 180, 140};
uint8_t legs_initial[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72, 180, 180, 180, 180, 180, 180};

using namespace ControlTableItem;

ros::NodeHandle nh;

//IMU ros config
sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;
ros::Publisher pub("micro/IMU", &inertial);
char in[] = "/imu";

void servo_cb(const std_msgs::Int16MultiArray& cmd_msg){
  for(int i = 0; i < DXL_ID_CNT; i++){
    legs_temporario[i] = legs_initial_temporario[i] + cmd_msg.data[i];
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

  bno_setup();//BNO setup

  Serial.begin(1000000);
    
  dxl1.begin(1000000);
  dxl2.begin(1000000);
  dxl3.begin(1000000);
  dxl5.begin(1000000);

  dxl1.setPortProtocolVersion(1.0);
  dxl2.setPortProtocolVersion(1.0);
  dxl3.setPortProtocolVersion(1.0);
  dxl5.setPortProtocolVersion(1.0);

  // Set Position PID Gains(Serial 1)
  dxl1.writeControlTableItem(POSITION_P_GAIN, BROADCAST_ID, position_p_gain);
  dxl1.writeControlTableItem(POSITION_I_GAIN, BROADCAST_ID, position_i_gain);
  dxl1.writeControlTableItem(POSITION_D_GAIN, BROADCAST_ID, position_d_gain);
  

  // Set Position PID Gains(Serial 2)
  dxl2.writeControlTableItem(POSITION_P_GAIN, BROADCAST_ID, position_p_gain);
  dxl2.writeControlTableItem(POSITION_I_GAIN, BROADCAST_ID, position_i_gain);
  dxl2.writeControlTableItem(POSITION_D_GAIN, BROADCAST_ID, position_d_gain);
  dxl2.writeControlTableItem(RETURN_DELAY_TIME, 12, 0);
  dxl2.writeControlTableItem(STATUS_RETURN_LEVEL, 12, 1);
  

  // Set Position PID Gains(Serial 3)
  dxl3.writeControlTableItem(POSITION_P_GAIN, BROADCAST_ID, position_p_gain);
  dxl3.writeControlTableItem(POSITION_I_GAIN, BROADCAST_ID, position_i_gain);
  dxl3.writeControlTableItem(POSITION_D_GAIN, BROADCAST_ID, position_d_gain);
  

  // Set Position PID Gains(Serial 5)
  dxl5.writeControlTableItem(POSITION_P_GAIN, BROADCAST_ID, position_p_gain);
  dxl5.writeControlTableItem(POSITION_I_GAIN, BROADCAST_ID, position_i_gain);
  dxl5.writeControlTableItem(POSITION_D_GAIN, BROADCAST_ID, position_d_gain);
  

  for(int i = 0; i < 3; i++){
        dxl1.writeControlTableItem(STATUS_RETURN_LEVEL, LEGS_ID_TEMPORARIO[i], 1);
        dxl1.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 80);
        dxl1.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_initial_temporario[i], UNIT_DEGREE);
//        dxl1.setGoalPosition(1, 20, UNIT_DEGREE);
        delay(1);
//      dxl1.setGoalPosition(1, 30, UNIT_DEGREE);
    }

    for(int i = 3; i < 6; i++){
      dxl3.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 80);
      dxl3.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_initial_temporario[i], UNIT_DEGREE);
//      dxl.setGoalPosition(12, 180, UNIT_DEGREE);
      delay(1);
//    dxl2.setGoalPosition(1, 20, UNIT_DEGREE);
//    delay(100);
//    dxl2.setGoalPosition(1, 30, UNIT_DEGREE);
    }

    for(int i = 6; i < 9; i++){
      //dxl2.writeControlTableItem(RETURN_DELAY_TIME, LEGS_ID_TEMPORARIO[i], 5);
    //  dxl2.writeControlTableItem(STATUS_RETURN_LEVEL, LEGS_ID_TEMPORARIO[i], 1);
      dxl2.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 80);
      dxl2.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_initial_temporario[i], UNIT_DEGREE);
//      dxl.setGoalPosition(12, 180, UNIT_DEGREE);
      delay(1); 
      }

    for(int i = 9; i < 12; i++){
      dxl5.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 80);
      dxl5.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_initial_temporario[i], UNIT_DEGREE);
//      dxl.setGoalPosition(12, 180, UNIT_DEGREE);
      delay(1);
    }
    delay(6000);
}

void loop() {
  nh.spinOnce();
  bno_loop(); //bno loop
  for(int i = 0; i < 3; i++){
        dxl1.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 0);
        dxl1.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_temporario[i], UNIT_DEGREE);
        
//        dxl1.setGoalPosition(1, 20, UNIT_DEGREE);
       // delay(1);
//      dxl1.setGoalPosition(1, 30, UNIT_DEGREE);
    }

    for(int i = 3; i < 6; i++){
      dxl3.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 0);
      dxl3.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_temporario[i], UNIT_DEGREE);
      
//      dxl.setGoalPosition(12, 180, UNIT_DEGREE);
    //  delay(1);
//    dxl2.setGoalPosition(1, 20, UNIT_DEGREE);
//    delay(100);
//    dxl2.setGoalPosition(1, 30, UNIT_DEGREE);
    }

    for(int i = 6; i < 9; i++){
      dxl2.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 0);
      dxl2.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_temporario[i], UNIT_DEGREE);
      
//      dxl.setGoalPosition(12, 180, UNIT_DEGREE);
     // delay(1); 
      }

    for(int i = 9; i < 12; i++){
      dxl5.writeControlTableItem(MOVING_SPEED, LEGS_ID_TEMPORARIO[i], 0);
      dxl5.setGoalPosition(LEGS_ID_TEMPORARIO[i], legs_temporario[i], UNIT_DEGREE);
      
//      dxl.setGoalPosition(12, 180, UNIT_DEGREE);
    //  delay(1);
    }
    digitalWrite(blinkPin, LOW);
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
//        digitalWrite(blinkPin, LOW);
    }
    pub.publish(&inertial);
    delay(1);
}
