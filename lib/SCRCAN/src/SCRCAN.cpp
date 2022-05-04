#include "SCRCAN.hpp"
#include <FlexCAN_T4.h>

namespace SCRCAN
{
  //----------------- CAN -----------------//
  // static CAN_message_t frame; //FOR CAN Broadcasting if needed
  // ID: 0x01F0A000
  volatile int RPM;
  volatile unsigned char throttle = 0;
  volatile signed char coolant_temp;

  // ID: 0x01F0A003
  volatile unsigned short int speed;
  volatile unsigned char gear;
  volatile float voltage;

  // ID: 0x01F0A004
  volatile float fuel_pressure;
  volatile float oil_pressure;
  volatile int VE;

  // ID: 0x01F0A005
  volatile bool launch_active; // Clutch Sw

  // ID: 0x01F0A007
  volatile int oil_temp; // offset -50C
  volatile bool logging; // logging active ? T/F

  // ID: 0x01F0A008
  volatile int launch_rpm; // 2StepTargetFuel [RPM]
  volatile int error;      // if any errors are a 1, then error = 1

  volatile int TC_FuelCut;  //% Fuel Cut
  volatile int TC_SparkCut; //% Spark Cut
  volatile int TC_Mode;     // TC Strength

  // ID: 0x01F0A012
  volatile int traction_control; // TC_SlipTarget; 16 bit unsigned 0.02 kph/bit 0-1310.7 kph
  volatile int TC_SlipMeas;

  // ID: 0x0000A0000
  volatile float gps_lat;
  volatile float gps_long;

  // ID: 0x0000A0001
  volatile int gps_speed;
  volatile int gps_altitude;

  // ID: 0x0000A0003
  volatile int x_acceleration;
  volatile int y_acceleration;
  volatile int z_acceleration;

  // ID: 0x0000A0004
  volatile int x_yaw;
  volatile int y_yaw;
  volatile int z_yaw;

  //----------------- CAN Handling -----------------//
  void AEM_handleMessage_0(const CAN_message_t &frame)
  {
    // Serial.println("Read throttle");
    RPM = 0.39063 * long((256 * long(frame.buf[0]) + frame.buf[1]));
    throttle = 0.0015259 * long((256 * long(frame.buf[4]) + frame.buf[5]));
    coolant_temp = frame.buf[7];
  }
  void AEM_handleMessage_1(const CAN_message_t &frame)
  {
    // Serial.println("Read voltage");
    //  converted from kph to mph
    speed = 0.00390625 * long((256 * long(frame.buf[2]) + frame.buf[3]));
    gear = frame.buf[4];
    voltage = 0.0002455 * long((256 * long(frame.buf[6]) + frame.buf[7]));
    // Serial.print(voltage);
  }
  void AEM_handleMessage_2(const CAN_message_t &frame)
  {
    // Serial.println("Read fuel/oil");
    fuel_pressure = 0.580151 * frame.buf[3];
    oil_pressure = 0.580151 * frame.buf[4];
    VE = frame.buf[2];
  }
  void AEM_handleMessage_3(const CAN_message_t &frame)
  {
    // Serial.println("Read launch ctrl");
    launch_active = frame.buf[8]; // Check, its bit 1 of byte 7 in the frame.
  }
  void AEM_handleMessage_4(const CAN_message_t &frame)
  {
    // Serial.print("Read oil temp");
    oil_temp = frame.buf[4] - 50;
    logging = frame.buf[8]; // Check, its bit 1 of byte 7 in the frame.
  }
  void AEM_handleMessage_5(const CAN_message_t &frame)
  {
    // Serial.println("Read launch rpm");
    launch_rpm = 0.39063 * long((256 * long(frame.buf[3]) + frame.buf[4]));

    if (frame.buf[7] == 0)
    {
      error = 0;
    }
    else
    {
      error = 1;
    }
  }
  void AEM_handleMessage_6(const CAN_message_t &frame)
  {
    // Serial.println("Fuel cut");
    TC_FuelCut = frame.buf[0] * 0.392157;  //% Fuel Cut
    TC_SparkCut = frame.buf[1] * 0.392157; //% Spark Cut
    TC_Mode = frame.buf[4];                // TC Strength
  }
  void AEM_handleMessage_7(const CAN_message_t &frame)
  {
    // converted from kph to mph
    traction_control = 0.01242742 * long((256 * long(frame.buf[0]) + frame.buf[1]));
    TC_SlipMeas = 0.01242742 * long((256 * long(frame.buf[2]) + frame.buf[3])); // 0 - 1310.7 kph
  }
  void AEM_handleMessage_8(const CAN_message_t &frame)
  {
    gps_lat = (frame.buf[0] - 2147483647.5) * 4.19095159 * pow(10, -8);
    gps_long = (frame.buf[4] - 2147483647.5) * 8.38190317 * pow(10, -8); // assuming deg range uses all 32 bits
  }
  void AEM_handleMessage_9(const CAN_message_t &frame)
  {
    gps_speed = 0.01 * long((256 * long(frame.buf[0]) + frame.buf[1]));
    gps_altitude = long((256 * long(frame.buf[2]) + frame.buf[3]));
    // this is signed, not sure how the library converts it; it is signed by magnitude, not 2's complement
  }
  void AEM_handleMessage_10(const CAN_message_t &frame)
  {
    // all 16 bit signed...
    x_acceleration = 0.000244141 * long((256 * long(frame.buf[0]) + frame.buf[1]));
    y_acceleration = 0.000244141 * long((256 * long(frame.buf[2]) + frame.buf[3]));
    z_acceleration = 0.000244141 * long((256 * long(frame.buf[4]) + frame.buf[5]));
  }
  void AEM_handleMessage_11(const CAN_message_t &frame)
  {
    x_yaw = 0.015258789 * long((256 * long(frame.buf[0]) + frame.buf[1]));
    y_yaw = 0.015258789 * long((256 * long(frame.buf[2]) + frame.buf[3]));
    z_yaw = 0.015258789 * long((256 * long(frame.buf[4]) + frame.buf[5]));
  }

  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can0;
  const int NUM_TX_MAILBOXES = 1;
  const int NUM_RX_MAILBOXES = 12;
  void init(int pin)
  {
    can0.begin();
    can0.setBaudRate(500000);
    can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    for (int i = 0; i < NUM_RX_MAILBOXES; i++)
    { // configure all the AEM's extended frames
      can0.setMB((FLEXCAN_MAILBOX)i, RX, EXT);
    }
    for (int i = NUM_RX_MAILBOXES - 1; i < NUM_TX_MAILBOXES + NUM_RX_MAILBOXES; i++)
    {
      can0.setMB((FLEXCAN_MAILBOX)i, TX, STD); // for loopback
    }
    can0.setMBFilter(REJECT_ALL);
    can0.enableMBInterrupts();
    // can0.enableFIFO();
    // can0.enableFIFOInterrupt();
    can0.onReceive(MB0, AEM_handleMessage_0);
    can0.onReceive(MB1, AEM_handleMessage_1);
    can0.onReceive(MB2, AEM_handleMessage_2);
    can0.onReceive(MB3, AEM_handleMessage_3);
    can0.onReceive(MB4, AEM_handleMessage_4);
    can0.onReceive(MB5, AEM_handleMessage_5);
    can0.onReceive(MB6, AEM_handleMessage_6);
    can0.onReceive(MB7, AEM_handleMessage_7);
    can0.onReceive(MB8, AEM_handleMessage_8);
    can0.onReceive(MB9, AEM_handleMessage_9);
    can0.onReceive(MB10, AEM_handleMessage_10);
    can0.onReceive(MB11, AEM_handleMessage_11);
    can0.setMBUserFilter(MB0, 0x01F0A000, 0xFF); // 0x01F0A000 0xF88A000
    can0.setMBUserFilter(MB1, 0x01F0A003, 0xFF);
    can0.setMBUserFilter(MB2, 0x01F0A004, 0xFF); // oil pressure
    can0.setMBUserFilter(MB3, 0x01F0A005, 0xFF); // launch active (laungh ramp time?)
    can0.setMBUserFilter(MB4, 0x01F0A007, 0xFF); // oil temp logging
    can0.setMBUserFilter(MB5, 0x01F0A008, 0xFF); // launch rpm fuel cut
    can0.setMBUserFilter(MB6, 0x01F0A010, 0xFF); // Sparkcut fuelcut
    can0.setMBUserFilter(MB7, 0x01F0A012, 0xFF); // tc_slip measured
    can0.setMBUserFilter(MB8, 0x0000A0000, 0xFF);
    can0.setMBUserFilter(MB9, 0x0000A0001, 0xFF);
    can0.setMBUserFilter(MB10, 0x0000A0003, 0xFF);
    can0.setMBUserFilter(MB11, 0x0000A0004, 0xFF);
    can0.mailboxStatus();

    // STBY pin on MCP2561 transceiver needs to be set LOW to turn it on
    if (pin != -1)
    {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
    Serial.println("End Setup CAN");
  }

  void recv()
  {
    can0.events();
  }

  typedef union
  {
    double dataDouble;
    uint64_t dataInt64;
    uint32_t dataInt32[2];
    uint8_t dataInt8[8];
  } conversionUnion;

  conversionUnion dataStorage;
  void sendTest(double data)
  {

    static auto lastSent = millis();
    if (millis() - lastSent > 100)
    {

      CAN_message_t frame;
      frame.id = 0x7FE;
      dataStorage.dataDouble = data;
      Serial.printf("Attempting to send: %d, %d, %d, %d etc. ", dataStorage.dataInt8[0], dataStorage.dataInt8[1], dataStorage.dataInt8[2], dataStorage.dataInt8[3]);

      for (int i = 0; i < 8; i++)
      {
        frame.buf[i] = dataStorage.dataInt8[i];
      }

      frame.mb = MB12;

      can0.write(frame);
    }

    // // going to use can0, even though we're also receiving on it
    // CANMessage frame;
    // // we're sending data not requesting, so this is a data frame
    // frame.rtr = false;
    // // use standard frame IDs for our ones
    // frame.ext = true;
    // frame.id = 0x000542; // arbitrary id
    // // frame.id = millis () & 0x7FE;

    // dataStorage.dataDouble = data;
    // // test, also endianness shouldnt matter since they're both ARM cortex?
    // frame.buf64 = dataStorage.dataInt64;
    // frame.len = 8; // 8 bytes?

    // const bool ok = ACAN::can0.tryToSend(frame);
    // if (ok) {
    //   Serial.printf("Sent Can Frame\n");
    // } else {
    //   Serial.printf("tried to send and failed\n");
    // }
  }
}