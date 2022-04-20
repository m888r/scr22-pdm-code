#include <Arduino.h>
#include<ADC.h>
#include<ADC_util.h>

#define _TASK_MICRO_RES
#include <TaskScheduler.h> // docs https://github.com/arkhipenko/TaskScheduler/wiki/API-Documentation

#include <SCRCAN.hpp>

unsigned int RPM = 5000;
unsigned char speed = 0;
unsigned char throttle = 0;

//ID: 0x5F1

//ID: 0x5F2
unsigned char oil_temp = 105;

//ID: 0x5F3
unsigned char oil_pressure = 70;
unsigned char coolant_temp = 105;

//ID: 0x5F4
unsigned int gear;
unsigned int b_voltage;

///// ADC0 ////

ADC* adc = new ADC();

//  Function Prototypes
static void handleMessage_0 (const CANMessage & frame);
static void handleMessage_1 (const CANMessage & frame);
static void handleMessage_2 (const CANMessage & frame);
static void handleMessage_3 (const CANMessage & frame);
static void handleMessage_4 (const CANMessage & frame);
static void handleMessage_5 (const CANMessage & frame);
static void handleMessage_6 (const CANMessage & frame);
static void handleMessage_7 (const CANMessage & frame);
static void handleMessage_8 (const CANMessage & frame);
static void handleMessage_9 (const CANMessage & frame);
static void handleMessage_10 (const CANMessage & frame);
static void handleMessage_11 (const CANMessage & frame);

const uint8_t CURR_FUEL = A3;
const uint8_t CURR_H2O = A2;
const uint8_t CURR_FAN = A1;
const uint8_t CURR_12V = A0;
const auto FAN = 22;
const auto H2O = 21;
const auto FUEL = 9;
int CTRLarr[3] = {9, 21, 22};

uint8_t CTRL = 9; // 9 fuel, 21 h2o, 22 fan
int i = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Entered Setup");


  ACANSettings settings (500*1000); 
  const ACANPrimaryFilter primaryFilters [] = {
      ACANPrimaryFilter (kData, kExtended, 0x01F0A000, handleMessage_0),// 0x01F0A000 0xF88A000
      ACANPrimaryFilter (kData, kExtended, 0x01F0A003, handleMessage_1), 
      ACANPrimaryFilter (kData, kExtended, 0x01F0A004, handleMessage_2), //oil pressure
      ACANPrimaryFilter (kData, kExtended, 0x01F0A005, handleMessage_3), //launch active (laungh ramp time?)
      ACANPrimaryFilter (kData, kExtended, 0x01F0A007, handleMessage_4), //oil temp logging
      ACANPrimaryFilter (kData, kExtended, 0x01F0A008, handleMessage_5), //launch rpm fuel cut
      ACANPrimaryFilter (kData, kExtended, 0x01F0A010, handleMessage_6), //Sparkcut fuelcut
      ACANPrimaryFilter (kData, kExtended, 0x01F0A012, handleMessage_7), //tc_slip measured
      ACANPrimaryFilter (kData, kExtended, 0x0000A0000, handleMessage_8),
      ACANPrimaryFilter (kData, kExtended, 0x0000A0001, handleMessage_9),
      ACANPrimaryFilter (kData, kExtended, 0x0000A0003, handleMessage_10),
      ACANPrimaryFilter (kData, kExtended, 0x0000A0004, handleMessage_11)
  };
  Serial.println("ACANSettings done");

  const uint32_t errorCode = ACAN::can0.begin (settings, primaryFilters, 12) ;
  if (0 == errorCode) {
    Serial.println ("can0 ok") ;
  }
  else{
    Serial.print ("Error can0: ") ;
    Serial.println (errorCode) ;
  }

  Serial.println("End Setup");

  pinMode(A3, INPUT); // CURR_FUEL
  pinMode(A2, INPUT); // CURR_H2O
  pinMode(A1, INPUT); // CURR_FAN
  pinMode(A0, INPUT); // CURR_12V

  pinMode(FUEL, OUTPUT);
  pinMode(H2O, OUTPUT);
  pinMode(FAN, OUTPUT);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  adc->adc0->setAveraging(16);                                    // set number of averages
  adc->adc0->setResolution(16);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed

  // https://www.pjrc.com/teensy/td_pulse.html
  analogWriteResolution(12);
  analogWriteFrequency(FAN, 1000); // 4khz, pins 5, 6, 9, 10, 20, 21, 22, 23	all share a timer
}

int startTime = 0;
enum class slewCtrl {
  ACCEL,
  MAX,
  DECEL,
  STOP
};
slewCtrl fanState = slewCtrl::ACCEL;
slewCtrl lastFanState = slewCtrl::ACCEL;
int fanSpeed = 0;
int fanTargetSpeed = 4096;
const float slewRate = 4096.0 / 4000.0; // 4 seconds to reach max speed

int lastPrint = 0;
float lastFanCurrent = 0;
float lastFuelCurrent = 0;
float lastWaterCurrent = 0;
float lastMainCurrent = 0;

int lastTime = 0;

void changeFanState(slewCtrl state) {
  lastFanState = fanState;
  fanState = state;
}

void loop() {

  ACAN::can0.dispatchReceivedMessage ();
  int dT = millis() - lastTime;
  lastTime = lastTime + dT;

  // trapezoid speed control state machine
  switch (fanState) {
    case slewCtrl::ACCEL:
      if (fanSpeed >= fanTargetSpeed) {
        changeFanState(slewCtrl::MAX);
      }
      break;
    case slewCtrl::MAX:
      if (lastFanState != slewCtrl::MAX) {
        startTime = millis();
        changeFanState(slewCtrl::MAX);
      }
      if (millis() - startTime >= 2000) {
        changeFanState(slewCtrl::DECEL);
      }
      break;
    case slewCtrl::DECEL:
      if (fanSpeed <= 0) {
        changeFanState(slewCtrl::STOP);
      }
      break;
    case slewCtrl::STOP:
      if (lastFanState != slewCtrl::STOP) {
        startTime = millis();
        changeFanState(slewCtrl::STOP);
      }
      if (millis() - startTime >= 2000) {
        changeFanState(slewCtrl::ACCEL);
      }
      break;
  }

  // fan slew state machine
  switch (fanState) {
    case slewCtrl::ACCEL:
      fanSpeed += (slewRate * dT);
      break;
    case slewCtrl::MAX:
      fanSpeed = fanTargetSpeed;
      break;
    case slewCtrl::DECEL:
      fanSpeed -= (slewRate * dT);
      break;
    case slewCtrl::STOP:
      fanSpeed = 0;
      break;
  }

  analogWrite(FAN, fanSpeed);
  analogWrite(FUEL, 0);
  analogWrite(H2O, 0);

  float fuelCurrent = 0.1*(adc->adc0->analogRead(CURR_FUEL)/3750.0) + 0.9*lastFuelCurrent;//* 3.3 / 1024.0 / 50.0 / 5.0;
  float fanCurrent = 0.01*(adc->adc0->analogRead(CURR_FAN)/3750.0) + 0.99*lastFanCurrent;//* 3.3 / 1024.0 / 50.0 / 5.0;
  float mainCurrent = 0.1*(adc->adc0->analogRead(CURR_12V)/3750.0) + 0.9*lastMainCurrent;//*(3.3/4096.0)/50.0; //* 3.3 / 1024.0 / 50.0 / 5.0;
  float waterCurrent = 0.1*(adc->adc0->analogRead(CURR_H2O)/3750.0) + 0.9*lastWaterCurrent;//*(3.3/4096.0)/50.0; //* 3.3 / 1024.0 / 50.0 / 5.0;
  lastFuelCurrent = fuelCurrent;
  lastFanCurrent = fanCurrent;
  lastMainCurrent = mainCurrent;
  lastWaterCurrent = waterCurrent;

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("throttle:%d,fuel:%1.5f,fan:%1.5f,main:%1.5f,water:%1.5f\n", throttle, fuelCurrent, fanCurrent, mainCurrent, waterCurrent);
    // Serial.printf("fanpwm:%1.5f,fancurrent:%1.5f\n", fanSpeed * 5 / (double) fanTargetSpeed, fanCurrent);
    //Serial.printf("%1.5f, %1.5f\n", fanCurrent, mainCurrent);
  }

  delayMicroseconds(50);  
}

//----------------- CAN Handling -----------------//
static void handleMessage_0 (const CANMessage & frame) {
  RPM = 0.39063 * long((256*long(frame.data[0]) + frame.data[1]));
  throttle = 0.0015259 * long((256*long(frame.data[4]) + frame.data[5]));
  coolant_temp = frame.data[7];
}
static void handleMessage_1 (const CANMessage & frame) {
  // converted from kph to mph
  speed = 0.00390625 * long((256*long(frame.data[2]) + frame.data[3]));
  gear = frame.data[4];
  b_voltage = 0.0002455 * long((256*long(frame.data[6]) + frame.data[7]));
  
}
static void handleMessage_2 (const CANMessage & frame) {
  //fuel_pressure = 0.580151 * frame.data[3];
  oil_pressure = 0.580151 * frame.data[4];
  //VE = frame.data[2];
}
static void handleMessage_3 (const CANMessage & frame) {
  //launch_active = frame.data[8]; // Check, its bit 1 of byte 7 in the frame.
}
static void handleMessage_4 (const CANMessage & frame) {
  oil_temp = frame.data[4] - 50;
  //logging  = frame.data[8];  // Check, its bit 1 of byte 7 in the frame.
}
static void handleMessage_5 (const CANMessage & frame) {
  // launch_rpm = 0.39063 * long((256*long(frame.data[3]) + frame.data[4]));
      
  // if(frame.data[7] == 0){
  //     error = 0;
  // }
  // else{
  //     error = 1;
  // }
}
static void handleMessage_6 (const CANMessage & frame) {
  // TC_FuelCut = frame.data[0] * 0.392157; //% Fuel Cut
  // TC_SparkCut = frame.data[1] * 0.392157;//% Spark Cut
  // TC_Mode = frame.data[4]; //TC Strength
}
static void handleMessage_7 (const CANMessage & frame) {
  //converted from kph to mph
  // traction_control = 0.01242742 * long((256*long(frame.data[0]) + frame.data[1]));
  // TC_SlipMeas = 0.01242742 * long((256*long(frame.data[2]) + frame.data[3])); //0 - 1310.7 kph
}
static void handleMessage_8 (const CANMessage & frame) {
  // gps_lat = (frame.data[0]-2147483647.5)*4.19095159*pow(10,-8);
  // gps_long = (frame.data[4]-2147483647.5)*8.38190317*pow(10,-8); // assuming deg range uses all 32 bits
}
static void handleMessage_9 (const CANMessage & frame) {
  // gps_speed = 0.01 * long((256*long(frame.data[0]) + frame.data[1]));
  // gps_altitude = long((256*long(frame.data[2]) + frame.data[3])); 
  // this is signed, not sure how the library converts it; it is signed by magnitude, not 2's complement
}
static void handleMessage_10 (const CANMessage & frame) {
  // all 16 bit signed... 
  // x_acceleration = 0.000244141 * long((256*long(frame.data[0]) + frame.data[1])); 
  // y_acceleration = 0.000244141 * long((256*long(frame.data[2]) + frame.data[3])); 
  // z_acceleration = 0.000244141 * long((256*long(frame.data[4]) + frame.data[5])); 
}
static void handleMessage_11 (const CANMessage & frame) {
  // x_yaw = 0.015258789 * long((256*long(frame.data[0]) + frame.data[1])); 
  // y_yaw = 0.015258789 * long((256*long(frame.data[2]) + frame.data[3])); 
  // z_yaw = 0.015258789 * long((256*long(frame.data[4]) + frame.data[5])); 
}
