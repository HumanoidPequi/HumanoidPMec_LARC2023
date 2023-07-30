#include <ros.h>

//Dynamixel library
#include <Dynamixel2Arduino.h>

//Ros stuff(dynamixel, mpu e bno)
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

//Hardware specifications
#include <HardwareSerial.h>
//HardwareSerial Serial1(1);
//HardwareSerial Serial2(2);
#include <ArduinoHardware.h>
#define DXL_SERIAL Serial2
#define JOINTS_TOTAL 20
#define BROADCAST_ID 254
const int DXL_DIR_PIN = 13; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_ID_CNT = 12;
//const uint8_t DXL_ID_CNT1 = 14;
const uint8_t DXL_ID_CNT_LEGS = 6;
//const uint8_t DXL_ID_LIST[JOINTS_TOTAL] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 19 ,20};
//const uint8_t ball_tracking[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6,7, 8, 9, 10, 11, 12,19,20};
//const uint8_t ID_HEAD[DXL_ID_CNT] = {19, 20};
const uint8_t ID_RIGHT_LEG[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6}; 
//const uint8_t ID_LEFT_LEG[DXL_ID_CNT] = {7, 8, 9, 10, 11, 12};
//const uint8_t ID_RIGHT_ARM[DXL_ID_CNT] = {13, 14, 15};
//const uint8_t ID_RIGHT_ARM[DXL_ID_CNT] = {16, 17, 18};
const float DXL_PROTOCOL_VERSION = 1.0;

//Imu(BNO080) stuff
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;
//Imu(mpu)
//#include "FastIMU.h"
//#include "Madgwick.h"

//#define IMU_ADDRESS 0x68    //Change to the address of the IMU
//#define PERFORM_CALIBRATION //Comment to disable startup calibration
//MPU6050 IMU;               //Change to the name of any supported IMU!

//calData calib = { 0 };  //Calibration data
//AccelData IMUAccel;    //Sensor data
//GyroData IMUGyro;
//MagData IMUMag;
//Madgwick filter;

//Define led
#define blinkPin 2

//uint8_t head1[DXL_ID_CNT1] = {180, 88, 180, 200, 202, 72, 150, 180, 180, 180, 170, 180 ,180, 180};
//uint8_t head[DXL_ID_CNT] = {180, 180};
//uint8_t head_initial[DXL_ID_CNT] = {180, 180};
  uint8_t right_leg[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72, 180, 180, 180, 180, 180, 180};
  uint8_t right_leg_initial[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72, 180, 180, 180, 180, 180, 180};
//const uint8_t right_leg[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72};
//const uint8_t left_leg[DXL_ID_CNT] = {180, 180, 180, 180, 180, 180};
//const uint8_t right_arm[DXL_ID_CNT] = {180, 180, 180};
//const uint8_t left_arm[DXL_ID_CNT] = {180, 180, 180};
//uint8_t tracking_position[DXL_ID_CNT] = {180, 88, 180, 200, 202, 72,180, 180, 180, 180, 180, 180 ,180, 180};
//const uint8_t marta[DXL_ID_CNT] = {180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180};
//const uint8_t max_pos[DXL_ID_CNT] = {360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360};
//const uint8_t min_pos[DXL_ID_CNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//dxl1 = perna esquerda e pino 13; dxl2 = perna direita e pino 14
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

ros::NodeHandle nh;

//Imu ros configs
sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;

//Imu(bno) ros configs
ros::Publisher bno("imu/BNO", &inertial);
char imu_bno[] = "/bno";

//Imu(mpu) ros configs
//ros::Publisher mpu("imu/MPU", &inertial);
//char imu_mpu[] = "/mpu";

void servo_cb(const std_msgs::Int16MultiArray& cmd_msg) {
  if (cmd_msg.data_length != DXL_ID_CNT) {
    return;
  }

  for (int i = 0; i < DXL_ID_CNT; i++) {
    right_leg[i] = right_leg_initial[i] + cmd_msg.data[i];
    //dxl.setGoalPosition(ball_tracking[i], tracking_position[i] + cmd_msg.data[i], UNIT_DEGREE);
    //delay(0,01);
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);

// Position PID Gains
// Adjust these gains to tune the behavior of DYNAMIXEL
uint16_t position_p_gain = 40;
uint16_t position_i_gain = 10;
uint16_t position_d_gain = 1;

void setup() {
  //Dynamixel setup
  Serial.begin(57600);
  dxl.begin(1000000);
  nh.initNode();
  nh.subscribe(sub);

  setup_dynamixel();
  //Imu(bno) setup
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
    while (1);
  }
  delay(10);
  Wire.setClock(400000); // Increase I2C data rate to 400kHz
  delay(10);
  myIMU.enableLinearAccelerometer(8); // m/s^2 no gravity
  myIMU.enableRotationVector(4);      // quat
  myIMU.enableGyro(12);               // rad/s
  nh.advertise(bno);

  digitalWrite(blinkPin, LOW);

  //Imu(mpu) setup
  /*delay(1);
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  Serial.begin(57600);
  while (!Serial) {
    ;
  }

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU Calibrated Quaternion example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(1000);
  }
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);

  filter.begin(0.2f);
#endif
  nh.advertise(mpu);*/
} 

void loop() {
  nh.spinOnce();
   
  //Imu(bno) loop
  inertial.header.frame_id = imu_bno;
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
    bno.publish(&inertial);
    digitalWrite(blinkPin, LOW);
  }
  bno.publish(&inertial);
  //delay(1);

  //Imu(mpu) loop
  /*inertial.header.frame_id = imu_mpu;
  inertial.header.stamp = nh.now();

IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);
  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
  }
  else {
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }
  Serial.print("QW: ");
  Serial.print(filter.getQuatW());
  Serial.print("\tQX: ");
  Serial.print(filter.getQuatX());
  Serial.print("\tQY: ");
  Serial.print(filter.getQuatY());
  Serial.print("\tQZ: ");
  Serial.print(filter.getQuatZ());

  IMU.getAccel(&IMUAccel);
  Serial.print("\taccX: ");
  Serial.print(IMUAccel.accelX);
  Serial.print("\taccY");
  Serial.print(IMUAccel.accelY);
  Serial.print("\taccZ");
  Serial.print(IMUAccel.accelZ);
  
  IMU.getGyro(&IMUGyro);
  Serial.print("\tgyroX");
  Serial.print(IMUGyro.gyroX);
  Serial.print("\tgyroY");
  Serial.print(IMUGyro.gyroY);
  Serial.print("\tgyroZ");
  Serial.println(IMUGyro.gyroZ);
  
    ang.x = IMUAccel.accelX;
    ang.y = IMUAccel.accelY;
    ang.z = IMUAccel.accelZ;

    lin.x = IMUGyro.gyroX;
    lin.y = IMUGyro.gyroY;
    lin.z = IMUGyro.gyroZ;

    inertial.angular_velocity = ang;
    inertial.linear_acceleration = lin;
    
    inertial.orientation.x = filter.getQuatX();
    inertial.orientation.y = filter.getQuatY();
    inertial.orientation.z = filter.getQuatZ();
    inertial.orientation.w = filter.getQuatW();

    mpu.publish(&inertial);
  mpu.publish(&inertial);
  delay(1);
  
  //keep sending the latest angle to dynamixel*/
  set_position();
}

void set_position(){
  for (int i = 0; i < DXL_ID_CNT_LEGS; i++) {
    dxl.setGoalPosition(ID_RIGHT_LEG[i], right_leg[i], UNIT_DEGREE);
    delay(0.001);
  }
}

//initialize dynamixels function
void setup_dynamixel(){
  
  //Serial 1
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(BROADCAST_ID);
  dxl.torqueOff(BROADCAST_ID);
  dxl.setOperatingMode(BROADCAST_ID, OP_POSITION);
  dxl.torqueOn(BROADCAST_ID);

  for (int i = 0; i < DXL_ID_CNT_LEGS; i++) {
    dxl.setGoalPosition(ID_RIGHT_LEG[i], right_leg[i], UNIT_DEGREE);
    delay(0.0001);
  }

  // Set Position PID Gains
  dxl.writeControlTableItem(POSITION_P_GAIN, BROADCAST_ID, position_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, BROADCAST_ID, position_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, BROADCAST_ID, position_d_gain);
  dxl.writeControlTableItem(PROFILE_VELOCITY,BROADCAST_ID, 0);  
}
