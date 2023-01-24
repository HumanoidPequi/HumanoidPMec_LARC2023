#include "Wire.h"       
#include "I2Cdev.h"     
//#include "MPU6050.h"    
#include "MPU6050_6Axis_MotionApps20.h"

//cod ROS
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
uint8_t fifoBuffer[64]; // FIFO storage buffer
ros::NodeHandle  nh;

sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;
ros::Publisher pub("micro/IMU_head", &inertial);
char in[] = "/imu_head";
Quaternion q;           // [w, x, y, z]         quaternion container

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  //pinMode(LED_BUILTIN, OUTPUT);

  //ros
  nh.initNode(); // iniciando nó
  nh.advertise(pub);// associando publisher ao nó

}

void loop()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  data.X = map(ax, -17000, 17000, 0, 255 ); // X axis data
  data.Y = map(ay, -17000, 17000, 0, 255); 
  data.Z = map(az, -17000, 17000, 0, 255);  // Y axis data
 // delay(500);
  Serial.print("Axis X = ");
  Serial.print(data.X);
  Serial.print("  ");
  Serial.print("Axis Y = ");
  Serial.print(data.Y);
  Serial.print("  ");
  Serial.print("Axis Z  = ");
  Serial.println(data.Z);

  // ROS

  nh.spinOnce();
  inertial.header.frame_id = in;
  inertial.header.stamp = nh.now();
 
  digitalWrite(LED_BUILTIN, HIGH);
  // internal copies of the IMU data
  byte linAccuracy = 0;
  byte gyroAccuracy = 0;
  float quatRadianAccuracy = 0;
  byte quatAccuracy = 0;

  // get IMU data in one go for each sensor type

  ang.x = ax;
  ang.y = ay;
  ang.z = az;
  

  lin.x = gx;
  lin.y = gy;
  lin.z = gz;

  inertial.orientation.x = q.x;
  inertial.orientation.y = q.y;
  inertial.orientation.z = q.z;
  inertial.orientation.w = q.w;
  

  inertial.angular_velocity = ang;
  inertial.linear_acceleration = lin;
  pub.publish(&inertial);
  digitalWrite(LED_BUILTIN, LOW);
  
//  pub.publish(&inertial);
  delay(1);
}


