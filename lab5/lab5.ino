#include "ICM_20948.h"
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include <Wire.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "b4735562-362d-4d17-b3e4-c4cd7256fa09"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite,
                                                  MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT,
                                               BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING,
                                                  BLERead | BLENotify,
                                                  MAX_MSG_SIZE);

enum command_types { PING };

EString tx_estring_value;
RobotCommand robot_cmd(":|");
void handle_command() {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  success = robot_cmd.get_command_type(cmd_type);
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
      // Add more commands here
    default:
      break;
  }
}

void setup() {
  Wire.begin();

  Serial.begin(115200);

  // Setup BLE
  BLE.begin();

  // // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  BLE.addService(testService);

  // // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();

  if (central) {
    if (rx_characteristic_string.written()) {
      handle_command();
    }
  }
}
