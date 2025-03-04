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

#define DATA_ARRAY_SIZE 1000;


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


int tof_in_mm[10000];
int tof2_in_mm[10000];
float imu_pitch[10000];
float imu_roll[10000];
unsigned long times[10000];
int data_sent = 0;
const unsigned long duration = 5000;

enum command_types {
  PING,
  TOF,
  TOF_AND_IMU,
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
    case TOF_AND_IMU:
      {
        unsigned long start = millis();
        while (millis() - start < duration) {
          distance_sensor_1.startRanging();
          distance_sensor_2.startRanging();
          while (!distance_sensor_1.checkForDataReady()) {
            delay(1);
          }
          while (!distance_sensor_2.checkForDataReady()) {
            delay(1);
          }
          int distance = distance_sensor_1.getDistance();
          int distance2 = distance_sensor_2.getDistance();
          distance_sensor_1.clearInterrupt();
          distance_sensor_1.stopRanging();
          distance_sensor_2.clearInterrupt();
          distance_sensor_2.stopRanging();

          if (myICM->dataReady()) {
            myICM->getAGMT();
            float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
            float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
            imu_pitch[data_sent] = pitch;
            imu_roll[data_sent] = roll;
          }


          tof_in_mm[data_sent] = distance;
          tof2_in_mm[data_sent] = distance2;
          times[data_sent] = millis();
          data_sent++;
        }

        for (int i = 0; i < data_sent; i++) {
          tx_estring_value.clear();
          tx_estring_value.append((float)times[i]);
          tx_estring_value.append(",");
          tx_estring_value.append(tof_in_mm[i]);
          tx_estring_value.append(",");
          tx_estring_value.append(tof2_in_mm[i]);

          tx_estring_value.append(",");
          tx_estring_value.append(imu_pitch[i]);
          tx_estring_value.append(",");
          tx_estring_value.append(imu_roll[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }

        // Start Index over for next batch;
        data_sent = 0;
        break;
      }

    case TOF:
      for (int i = 0; i < 32; i++) {
        unsigned long start = millis();
        distance_sensor_2.startRanging();
        distance_sensor_2.setDistanceModeShort();
        distance_sensor_2.setTimingBudgetInMs(33);
        while (!distance_sensor_2.checkForDataReady()) {
          delay(1);
        }
        int distance = distance_sensor_2.getDistance();
        distance_sensor_2.clearInterrupt();
        distance_sensor_2.stopRanging();
        unsigned long diff = millis() - start;
        Serial.print("Ranging took about: ");
        Serial.print(diff);
        Serial.println("ms");
        float distanceInMM = distance;

        tx_estring_value.clear();
        tx_estring_value.append(distanceInMM);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Timing Budget Is: ");
        int timing_budget = distance_sensor_2.getTimingBudgetInMs();
        Serial.println(timing_budget);
      }
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

    float pitch = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
    float roll = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" Roll: ");
    Serial.print(roll);
    Serial.print(" ");
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
  distance_sensor_1.setTimingBudgetInMs(33);
  distance_sensor_2.setDistanceModeShort();
  distance_sensor_2.setTimingBudgetInMs(33);


  while (!distance_sensor_1.checkForDataReady()) {
    delay(1);
  }

  while (!distance_sensor_2.checkForDataReady()) {
    delay(1);
  }

  int distance = distance_sensor_1.getDistance();  //Get the result of the measurement from the sensor
  int distance2 = distance_sensor_2.getDistance();
  distance_sensor_1.clearInterrupt();
  distance_sensor_1.stopRanging();
  distance_sensor_2.clearInterrupt();
  distance_sensor_2.stopRanging();

  Serial.print(" Stop Ranging at: ");
  unsigned long ready = millis() - start;
  Serial.print(ready);


  Serial.print("Distance From Sensor 1(mm): ");
  Serial.print(distance);
  // Serial.print(" ");

  // float distanceInches = distance * 0.0393701;
  // float distanceFeet = distanceInches / 12.0;

  // Serial.print("\tDistance(ft): ");
  // Serial.print(distanceFeet, 2);

  Serial.print("Distance From Sensor 2(mm): ");
  Serial.print(distance2);

  // distanceInches = distance2 * 0.0393701;
  // distanceFeet = distanceInches / 12.0;

  // Serial.print("\tDistance(ft): ");
  // Serial.print(distanceFeet, 2);

  ready = millis() - start;
  Serial.print("All done in: ");
  Serial.print(ready);

  Serial.println();
}
