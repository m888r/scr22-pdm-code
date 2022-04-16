#include <Arduino.h>
#include<ADC.h>
#include<ADC_util.h>

#define _TASK_MICRO_RES
#include <TaskScheduler.h> // docs https://github.com/arkhipenko/TaskScheduler/wiki/API-Documentation

#include <SCRCAN.hpp>

///// ADC0 ////
ADC *adc = new ADC(); // adc object;

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

  SCRCAN::init();

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
    SCRCAN::loop();
    //Serial.printf("throttle:%d,fuel:%1.5f,fan:%1.5f,main:%1.5f,water:%1.5f\n", SCRCAN::throttle, fuelCurrent, fanCurrent, mainCurrent, waterCurrent);
    // Serial.printf("fanpwm:%1.5f,fancurrent:%1.5f\n", fanSpeed * 5 / (double) fanTargetSpeed, fanCurrent);
    //Serial.printf("%1.5f, %1.5f\n", fanCurrent, mainCurrent);
  }

  delayMicroseconds(50);  
}