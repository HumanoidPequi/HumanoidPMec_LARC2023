// PequiMec. - Humanoide - Rui Jr
// ROSS STUFF
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// IMU STUFF
#include <Wire.h>
#include "FastIMU.h"
#include "Madgwick.h"

#define IMU_ADDRESS 0x68    // Change to the address of the IMU
#define PERFORM_CALIBRATION // Comment to disable startup calibration
MPU6050 IMU;                // Change to the name of any supported IMU!

calData calib = {0}; // Calibration data
AccelData IMUAccel;  // Sensor data
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;

ros::NodeHandle nh;

sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;
ros::Publisher pub("micro/IMU", &inertial);

char in[] = "/imu";

void setup()
{

    //
    delay(1000);
    Wire.begin();
    Wire.setClock(400000); // 400khz clock

    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0)
    {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        while (true)
        {
            ;
        }
    }

    if (err != 0)
    {
        Serial.print("Error Setting range: ");
        Serial.println(err);
        while (true)
        {
            ;
        }
    }

#ifdef PERFORM_CALIBRATION
    Serial.println("FastIMU Calibrated Quaternion example");
    if (IMU.hasMagnetometer())
    {
        delay(1000);
        Serial.println("Move IMU in figure 8 pattern until done.");
        delay(3000);
        IMU.calibrateMag(&calib);
        Serial.println("Magnetic calibration done!");
    }
    else
    {
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
    if (IMU.hasMagnetometer())
    {
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

    //
    nh.initNode();
    nh.advertise(pub);
}

void loop()
{
    nh.spinOnce();
    inertial.header.frame_id = in;
    inertial.header.stamp = nh.now();

    //
    IMU.update();
    IMU.getAccel(&IMUAccel);
    IMU.getGyro(&IMUGyro);
    if (IMU.hasMagnetometer())
    {
        IMU.getMag(&IMUMag);
        filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
    }
    else
    {
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

    pub.publish(&inertial);
    pub.publish(&inertial);
    delay(1);

    delay(50);
    //
}
