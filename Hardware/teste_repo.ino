/*******************************************************************************
 * 
 * Derived by https://github.com/Maehem from ROBOTIS Dynamixel2Arduino demo code 
 * 
* Copyright 2016 ROBOTIS CO., LTD.
* 
* REQUIRES: Dynamixel2Arduino library in your Arduino IDE
* https://github.com/ROBOTIS-GIT/Dynamixel2Arduino
* 
* 
* Getting the Robotis Dynamixel to work on a ESP32 required some changes in
* the stock demo code.
* 
* -- A custom serial port handler was written and mostly based on Robitis own example code.
*     * begin() was overridden.
*     * Track whether port is open already to prevent esp32 lockup when changing baud rate.
*     * First time begin() calls port.begin() with pin settings and baud.
*     * Subsequent calls to begin only affect baud rate by calling esp32 updateBaudRate() method.
* 
* 
*DXL_BROADCAST_ID
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

// Debug messages will appear on USB/serial monitor connection.
// #define DEBUG_SERIAL Serial

// This is the other tab in the Arduino IDE
#include "ESP32SerialPortHandler.cpp"

#include <stdlib.h>

// #define DXL_SERIAL   Serial1
const uint8_t DXL_DIR_PIN = 12; //  DIR pin
const uint8_t DXL_RX_PIN = 26; //  RX PIN
const uint8_t DXL_TX_PIN = 27; //  TX PIN

// Port and pins specific to your ESP32 configuration.
#define DXL_SERIAL2   Serial2
const uint8_t DXL_DIR_PIN2 = 13; //  DIR pin
const uint8_t DXL_RX_PIN2 = 16; //  RX PIN
const uint8_t DXL_TX_PIN2 = 17; //  TX PIN

// #define MAX_BAUD  5
// const int32_t baud[MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};

// The old way of creating dxl
//Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// New way of creating dxl
Dynamixel2Arduino dxl;

HardwareSerial DXL_SERIAL(1);
// Our custom handler with RX and TX pins specified.
ESP32SerialPortHandler esp_dxl_port(DXL_SERIAL, DXL_RX_PIN, DXL_TX_PIN, DXL_DIR_PIN);

Dynamixel2Arduino dxl2;

// Our custom handler with RX and TX pins specified.
ESP32SerialPortHandler esp_dxl_port2(DXL_SERIAL2, DXL_RX_PIN2, DXL_TX_PIN2, DXL_DIR_PIN2);



void setup() {

  // Set custom port handler
  dxl.setPort(esp_dxl_port);
  delay(500);
  dxl2.setPort(esp_dxl_port2);

  dxl.setPortProtocolVersion(1.0);
  delay(500);
  dxl2.setPortProtocolVersion(1.0);

  dxl.begin(1000000);
  delay(500);
  dxl2.begin(1000000);
  // dxl.torqueOff(254);
  // dxl.setOperatingMode(254, OP_POSITION);
  // dxl.torqueOn(254);

  // dxl2.torqueOff(254);
  // dxl2.setOperatingMode(254, OP_POSITION);
  // dxl2.torqueOn(254);

}

void loop() {
  // put your main code here, to run repeatedly:
  dxl.setGoalPosition(254, 300, UNIT_DEGREE);
  dxl2.setGoalPosition(254, 260, UNIT_DEGREE);
  delay(500);

  dxl.setGoalPosition(254, 310, UNIT_DEGREE);
  dxl2.setGoalPosition(254, 190, UNIT_DEGREE);
  delay(500);
  
}
