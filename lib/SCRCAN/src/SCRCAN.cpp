#include "SCRCAN.hpp"

namespace SCRCAN
{
  //----------------- CAN -----------------//
  // static CAN_message_t frame; //FOR CAN Broadcasting if needed
  // ID: 0x01F0A000
  int RPM;
  unsigned char throttle = 0;
  signed char coolant_temp;

  // ID: 0x01F0A003
  unsigned short int speed;
  unsigned char gear;
  float voltage;

  // ID: 0x01F0A004
  float fuel_pressure;
  float oil_pressure;
  int VE;

  // ID: 0x01F0A005
  bool launch_active; // Clutch Sw

  // ID: 0x01F0A007
  int oil_temp; // offset -50C
  bool logging; // logging active ? T/F

  // ID: 0x01F0A008
  int launch_rpm; // 2StepTargetFuel [RPM]
  int error;      // if any errors are a 1, then error = 1

  int TC_FuelCut;  //% Fuel Cut
  int TC_SparkCut; //% Spark Cut
  int TC_Mode;     // TC Strength

  // ID: 0x01F0A012
  int traction_control; // TC_SlipTarget; 16 bit unsigned 0.02 kph/bit 0-1310.7 kph
  int TC_SlipMeas;

  // ID: 0x0000A0000
  float gps_lat;
  float gps_long;

  // ID: 0x0000A0001
  int gps_speed;
  int gps_altitude;

  // ID: 0x0000A0003
  int x_acceleration;
  int y_acceleration;
  int z_acceleration;

  // ID: 0x0000A0004
  int x_yaw;
  int y_yaw;
  int z_yaw;

  //----------------- CAN Handling -----------------//
  void AEM_handleMessage_0(const CANMessage &frame)
  {
    //Serial.println("Read throttle");
    RPM = 0.39063 * long((256 * long(frame.data[0]) + frame.data[1]));
    throttle = 0.0015259 * long((256 * long(frame.data[4]) + frame.data[5]));
    coolant_temp = frame.data[7];
  }
  void AEM_handleMessage_1(const CANMessage &frame)
  {
    //Serial.println("Read voltage");
    // converted from kph to mph
    speed = 0.00390625 * long((256 * long(frame.data[2]) + frame.data[3]));
    gear = frame.data[4];
    voltage = 0.0002455 * long((256 * long(frame.data[6]) + frame.data[7]));
    //Serial.print(voltage);
  }
  void AEM_handleMessage_2(const CANMessage &frame)
  {
    //Serial.println("Read fuel/oil");
    fuel_pressure = 0.580151 * frame.data[3];
    oil_pressure = 0.580151 * frame.data[4];
    VE = frame.data[2];
  }
  void AEM_handleMessage_3(const CANMessage &frame)
  {
    //Serial.println("Read launch ctrl");
    launch_active = frame.data[8]; // Check, its bit 1 of byte 7 in the frame.
  }
  void AEM_handleMessage_4(const CANMessage &frame)
  {
    //Serial.print("Read oil temp");
    oil_temp = frame.data[4] - 50;
    logging = frame.data[8]; // Check, its bit 1 of byte 7 in the frame.
  }
  void AEM_handleMessage_5(const CANMessage &frame)
  {
    //Serial.println("Read launch rpm");
    launch_rpm = 0.39063 * long((256 * long(frame.data[3]) + frame.data[4]));

    if (frame.data[7] == 0)
    {
      error = 0;
    }
    else
    {
      error = 1;
    }
  }
  void AEM_handleMessage_6(const CANMessage &frame)
  {
    //Serial.println("Fuel cut");
    TC_FuelCut = frame.data[0] * 0.392157;  //% Fuel Cut
    TC_SparkCut = frame.data[1] * 0.392157; //% Spark Cut
    TC_Mode = frame.data[4];                // TC Strength
  }
  void AEM_handleMessage_7(const CANMessage &frame)
  {
    // converted from kph to mph
    traction_control = 0.01242742 * long((256 * long(frame.data[0]) + frame.data[1]));
    TC_SlipMeas = 0.01242742 * long((256 * long(frame.data[2]) + frame.data[3])); // 0 - 1310.7 kph
  }
  void AEM_handleMessage_8(const CANMessage &frame)
  {
    gps_lat = (frame.data[0] - 2147483647.5) * 4.19095159 * pow(10, -8);
    gps_long = (frame.data[4] - 2147483647.5) * 8.38190317 * pow(10, -8); // assuming deg range uses all 32 bits
  }
  void AEM_handleMessage_9(const CANMessage &frame)
  {
    gps_speed = 0.01 * long((256 * long(frame.data[0]) + frame.data[1]));
    gps_altitude = long((256 * long(frame.data[2]) + frame.data[3]));
    // this is signed, not sure how the library converts it; it is signed by magnitude, not 2's complement
  }
  void AEM_handleMessage_10(const CANMessage &frame)
  {
    // all 16 bit signed...
    x_acceleration = 0.000244141 * long((256 * long(frame.data[0]) + frame.data[1]));
    y_acceleration = 0.000244141 * long((256 * long(frame.data[2]) + frame.data[3]));
    z_acceleration = 0.000244141 * long((256 * long(frame.data[4]) + frame.data[5]));
  }
  void AEM_handleMessage_11(const CANMessage &frame)
  {
    x_yaw = 0.015258789 * long((256 * long(frame.data[0]) + frame.data[1]));
    y_yaw = 0.015258789 * long((256 * long(frame.data[2]) + frame.data[3]));
    z_yaw = 0.015258789 * long((256 * long(frame.data[4]) + frame.data[5]));
  }

  void init(int pin)
  {
    //----------------- CAN Init -----------------//
    ACANSettings settings(500 * 1000); // 500 kbits/s
    //settings.mListenOnlyMode = true;

    const ACANPrimaryFilter primaryFilters[] = {
        ACANPrimaryFilter(kData, kExtended, 0x01F0A000, AEM_handleMessage_0), // 0x01F0A000 0xF88A000
        ACANPrimaryFilter(kData, kExtended, 0x01F0A003, AEM_handleMessage_1),
        ACANPrimaryFilter(kData, kExtended, 0x01F0A004, AEM_handleMessage_2), // oil pressure
        ACANPrimaryFilter(kData, kExtended, 0x01F0A005, AEM_handleMessage_3), // launch active (laungh ramp time?)
        ACANPrimaryFilter(kData, kExtended, 0x01F0A007, AEM_handleMessage_4), // oil temp logging
        ACANPrimaryFilter(kData, kExtended, 0x01F0A008, AEM_handleMessage_5), // launch rpm fuel cut
        ACANPrimaryFilter(kData, kExtended, 0x01F0A010, AEM_handleMessage_6), // Sparkcut fuelcut
        ACANPrimaryFilter(kData, kExtended, 0x01F0A012, AEM_handleMessage_7), // tc_slip measured
        ACANPrimaryFilter(kData, kExtended, 0x0000A0000, AEM_handleMessage_8),
        ACANPrimaryFilter(kData, kExtended, 0x0000A0001, AEM_handleMessage_9),
        ACANPrimaryFilter(kData, kExtended, 0x0000A0003, AEM_handleMessage_10),
        ACANPrimaryFilter(kData, kExtended, 0x0000A0004, AEM_handleMessage_11)

    };

    Serial.println("ACANSettings done");

    const uint32_t errorCode = ACAN::can0.begin(settings, primaryFilters, 12);
    if (0 == errorCode)
    {
      Serial.println("can0 ok");
    }
    else
    {
      Serial.print("Error can0: ");
      Serial.println(errorCode);
    }

    // STBY pin on MCP2561 transceiver they need to be set HIGH to turn it on
    // though we found that grounding STBY worked too so maybe it's LOW idk lol
    if (pin != -1)
    {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
    Serial.println("End Setup CAN");
  }

  void recv() {
    //if (ACAN::can0.available())
    ACAN::can0.dispatchReceivedMessage();
  }


  typedef union {
    double dataDouble;
    uint64_t dataInt64;
    uint32_t dataInt32[2];
  } conversionUnion;

  conversionUnion dataStorage; 
  void sendTest(double data) {
    // going to use can0, even though we're also receiving on it
    CANMessage frame;
    // we're sending data not requesting, so this is a data frame
    frame.rtr = false;
    // use standard frame IDs for our ones
    frame.ext = false;
    frame.id = 0x542; // arbitrary id
    // frame.id = millis () & 0x7FE; 

    dataStorage.dataDouble = data;
    // test, also endianness shouldnt matter since they're both ARM cortex?
    frame.data64 = dataStorage.dataInt64;
    frame.len = 8; // 8 bytes?

    const bool ok = ACAN::can0.tryToSend(frame);
    if (ok) {
      Serial.printf("Sent Can Frame\n");
    } else {
      Serial.printf("tried to send and failed\n");
    }
  }
}