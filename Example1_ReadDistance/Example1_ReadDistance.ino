/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "ICM_20948.h"

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

#define DATA_ARRAY_SIZE ;


//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "b4735562-362d-4d17-b3e4-c4cd7256fa09"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

#define AD0_VAL 1
#define WIRE_PORT Wire


//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite,
                                                  MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT,
                                               BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING,
                                                  BLERead | BLENotify,
                                                  MAX_MSG_SIZE);


SFEVL53L1X distance_sensor_1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distance_sensor_2;
ICM_20948_I2C myICM;

enum command_types {
  PING,
  TOF,
  IMU,
};

RobotCommand robot_cmd(":|");
EString tx_estring_value;
void handle_command(ICM_20948_I2C *myICM) {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }

  switch (cmd_type) {
    case PING:
      tx_estring_value.clear();
      tx_estring_value.append("PONG");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());
      break;
    case TOF:
      Serial.print("TOF");
      break;
    case IMU:
      Serial.print("IMU");
      break;
    default:
      Serial.print("Invalid Cmd Type: ");
      Serial.println(cmd_type);
  }
}

void setup(void) {
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  // To use two sensors
  pinMode(SHUTDOWN_PIN, OUTPUT);
  // XSHUT Low on sensor 1
  digitalWrite(SHUTDOWN_PIN, LOW);
  distance_sensor_2.setI2CAddress(0x77);
  digitalWrite(SHUTDOWN_PIN, HIGH);

  if (distance_sensor_1.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  if (distance_sensor_2.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Both sensors online!");

  // Setup BLE
  BLE.begin();

  // // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  BLE.addService(testService);

  // // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

  bool initialized = false;
  myICM.begin(WIRE_PORT, AD0_VAL);
  while (!initialized) {
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again...");
      delay(500);
    } else {
      // Blink LED on startup
      pinMode(LED_BUILTIN, OUTPUT);
      for (int i = 0; i <= 2; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
      }
      initialized = true;
    }
  }
}

void loop(void) {
  BLEDevice central = BLE.central();

  if (myICM.dataReady()) {
    myICM.getAGMT();
  }

  if (central) {
    if (rx_characteristic_string.written()) {
      handle_command(&myICM);
    }
  }

  unsigned long start = millis();

  distance_sensor_1.startRanging();  //Write configuration bytes to initiate measurement
  distance_sensor_2.startRanging();

  distance_sensor_1.setDistanceModeShort();
  distance_sensor_2.setDistanceModeShort();


  while (!distance_sensor_1.checkForDataReady()) {
    delay(1);
  }
  unsigned long ready = millis() - start;
  Serial.print("Distance Sensor 1 data Ready in ");
  Serial.print(ready);
  while (!distance_sensor_2.checkForDataReady()) {
    delay(1);
  }
  ready = millis() - start;
  Serial.print("Distance Sensor 2 data Ready in ");
  Serial.print(ready);

  Serial.println();

  int distance = distance_sensor_1.getDistance();  //Get the result of the measurement from the sensor
  distance_sensor_1.clearInterrupt();
  distance_sensor_1.stopRanging();


  Serial.print("Distance From Sensor 1(mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  distance = distance_sensor_2.getDistance();

  distance_sensor_2.clearInterrupt();
  distance_sensor_2.stopRanging();


  Serial.print("Distance From Sensor 2(mm): ");
  Serial.print(distance);

  distanceInches = distance * 0.0393701;
  distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  ready = millis() - start;
  Serial.println("Loop is Done!");

  Serial.println();
}
