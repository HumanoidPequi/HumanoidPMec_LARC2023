// Código quase completamente baseado no do Luis Fernando Ferreira
// Pequi - Humanoide - Rui Jr

// ROSS STUFF
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// IMU STUFF
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

// LED
#define blinkPin 2

// ROSS CONFIGS
ros::NodeHandle nh;
sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;
ros::Publisher pub("micro/IMU", &inertial);
char in[] = "/imu";

void setup()
{
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

void loop()
{
    nh.spinOnce();
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
