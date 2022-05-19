#pragma once

#include <Arduino.h>

namespace SCRCAN
{
  //----------------- CAN -----------------//
  // static CAN_message_t frame; //FOR CAN Broadcasting if needed
  // ID: 0x01F0A000
  extern volatile int RPM;
  extern volatile unsigned char throttle;
  extern volatile signed char coolant_temp;

  // ID: 0x01F0A003
  extern volatile unsigned short int speed;
  extern volatile unsigned char gear;
  extern volatile float voltage;

  // ID: 0x01F0A004
  extern volatile float fuel_pressure;
  extern volatile float oil_pressure;
  extern volatile int VE;

  // ID: 0x01F0A005
  extern volatile bool launch_active; // Clutch Sw

  // ID: 0x01F0A007
  extern volatile int oil_temp; // offset -50C
  extern volatile bool logging; // logging active ? T/F

  // ID: 0x01F0A008
  extern volatile int launch_rpm; // 2StepTargetFuel [RPM]
  extern volatile int error;      // if any errors are a 1, then error = 1

  extern volatile int TC_FuelCut;  //% Fuel Cut
  extern volatile int TC_SparkCut; //% Spark Cut
  extern volatile int TC_Mode;     // TC Strength

  // ID: 0x01F0A012
  extern volatile int traction_control; // TC_SlipTarget; 16 bit unsigned 0.02 kph/bit 0-1310.7 kph
  extern volatile int TC_SlipMeas;

  // ID: 0x0000A0000
  extern volatile float gps_lat;
  extern volatile float gps_long;

  // ID: 0x0000A0001
  extern volatile int gps_speed;
  extern volatile int gps_altitude;

  // ID: 0x0000A0003
  extern volatile int x_acceleration;
  extern volatile int y_acceleration;
  extern volatile int z_acceleration;

  // ID: 0x0000A0004
  extern volatile int x_yaw;
  extern volatile int y_yaw;
  extern volatile int z_yaw;

  // ID: 0x7FE
  extern volatile double doubleTest;

  /**
   * @brief initialize mcp2561 + CAN transmission for AEM packets
   *
   * @param pin MCP2561 STBY pin if needed
   */
  void init(int pin = -1);

  /**
   * @brief loop to retrieve CAN data and store
   *
   */
  void recv();

  /**
   * @brief test send CAN data
   *
   * @param data a float data to send, some kinda current probs
   */
  void sendTest(double data);
}