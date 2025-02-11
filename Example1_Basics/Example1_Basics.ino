/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "b4735562-362d-4d17-b3e4-c4cd7256fa09"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

#define ALPHA 0.38
#define THETA 0.90
#define DATA_ARRAY_SIZE 5000

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite,
                                                  MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT,
                                               BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING,
                                                  BLERead | BLENotify,
                                                  MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

enum CommandTypes {
  PING,
  SEND_DATA,
  FFT,
  FFT_LOW_PASS,
  GYR,
  COMP,
  SEND_IMU_DATA,
};

float pitch_a[DATA_ARRAY_SIZE] = { 0 };
float roll_a[DATA_ARRAY_SIZE] = { 0 };
float pitch_g[DATA_ARRAY_SIZE] = { 0 };
float roll_g[DATA_ARRAY_SIZE] = { 0 };
float yaw_g[DATA_ARRAY_SIZE] = { 0 };
float comp_pitch[DATA_ARRAY_SIZE] = { 0 };
float comp_roll[DATA_ARRAY_SIZE] = { 0 };
float times[DATA_ARRAY_SIZE];

const long duration = 5000;  // 10 seconds.

void handle_command(ICM_20948_I2C *myICM) {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
   * since it uses strtok internally (refer RobotCommand.h and
   * https://www.cplusplus.com/reference/cstring/strtok/)
   */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }
  // Handle the command type accordingly
  switch (cmd_type) {
    /*
   * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
   */
    case PING:
      tx_estring_value.clear();
      tx_estring_value.append("PONG");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;

    case SEND_DATA:
      // Expect 'P' for Pitch, 'R' for Roll
      char char_arr[1];
      success = robot_cmd.get_next_value(char_arr);
      if (!success)
        return;
      if (char_arr[0] == 'P') {
        for (int i = 0; i < 1000; i++) {
          if (myICM->dataReady()) {
            myICM->getAGMT();
            tx_estring_value.clear();
            tx_estring_value.append(atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
        }
      } else if (char_arr[0] == 'R') {
        for (int i = 0; i < 1000; i++) {
          if (myICM->dataReady()) {
            myICM->getAGMT();
            tx_estring_value.clear();
            tx_estring_value.append(atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
        }
      }

      break;

    case FFT:
      {
        unsigned long start = millis();
        while (millis() - start < duration) {
          if (myICM->dataReady()) {
            myICM->getAGMT();
            tx_estring_value.clear();
            float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
            float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
            tx_estring_value.append(pitch);
            tx_estring_value.append(",");
            tx_estring_value.append(roll);
            tx_estring_value.append(",");
            tx_estring_value.append((float)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(20);  // about 50Hz sampling rate accounting for time to send data
          }
        }
        break;
      }

    case FFT_LOW_PASS:
      {
        unsigned long start = millis();
        float pitch_lpf = 0, roll_lpf = 0;
        if (myICM->dataReady()) {
          myICM->getAGMT();
          float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
          float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
          float pitch_lpf = pitch;
          float roll_lpf = roll;
        }
        while (millis() - start < duration) {
          if (myICM->dataReady()) {
            myICM->getAGMT();
            tx_estring_value.clear();
            float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
            float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
            pitch_lpf = ALPHA * pitch + (1 - ALPHA) * pitch_lpf;
            roll_lpf = ALPHA * roll + (1 - ALPHA) * roll_lpf;

            tx_estring_value.append(pitch);
            tx_estring_value.append(",");
            tx_estring_value.append(roll);
            tx_estring_value.append(",");
            tx_estring_value.append(pitch_lpf);
            tx_estring_value.append(",");
            tx_estring_value.append(roll_lpf);
            tx_estring_value.append(",");
            tx_estring_value.append((float)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(20);  // about 50Hz sampling rate accounting for time to send data
          }
        }
        break;
      }

    case GYR:
      {
        unsigned long start = millis();
        float pitch_lpf = 0, roll_lpf = 0;
        float gy_pitch = 0, gy_roll = 0, gy_yaw = 0;
        if (myICM->dataReady()) {
          myICM->getAGMT();
          float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
          float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
          float pitch_lpf = pitch;
          float roll_lpf = roll;
          gy_pitch = gy_pitch + myICM->gyrY() * (millis() - start) / 1000;
          gy_roll = gy_roll + myICM->gyrX() * (millis() - start) / 1000;
          gy_yaw = gy_yaw + myICM->gyrZ() * (millis() - start) / 1000;
        }
        float last_time = millis();
        while (millis() - start < duration) {
          if (myICM->dataReady()) {
            myICM->getAGMT();
            float dt = (millis() - last_time) / 1000.0;
            tx_estring_value.clear();
            float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
            float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
            pitch_lpf = ALPHA * pitch + (1 - ALPHA) * pitch_lpf;
            roll_lpf = ALPHA * roll + (1 - ALPHA) * roll_lpf;

            gy_pitch = gy_pitch + myICM->gyrY() * dt;
            gy_roll = gy_roll + myICM->gyrX() * dt;
            gy_yaw = gy_yaw + myICM->gyrZ() * dt;

            tx_estring_value.clear();
            tx_estring_value.append(gy_pitch);
            tx_estring_value.append(",");
            tx_estring_value.append(gy_roll);
            tx_estring_value.append(",");
            tx_estring_value.append(gy_yaw);
            tx_estring_value.append(",");

            tx_estring_value.append(pitch);
            tx_estring_value.append(",");
            tx_estring_value.append(roll);
            tx_estring_value.append(",");
            tx_estring_value.append(pitch_lpf);
            tx_estring_value.append(",");
            tx_estring_value.append(roll_lpf);
            tx_estring_value.append(",");
            tx_estring_value.append((float)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(20);  // about 100Hz sampling rate accounting for time to send
                        // data
            last_time = millis();
          }
        }
        break;
      }

    case COMP:
      {
        unsigned long start = millis();
        float pitch_lpf = 0, roll_lpf = 0;
        float gy_pitch = 0, gy_roll = 0, comp_roll = 0, comp_pitch = 0;
        if (myICM->dataReady()) {
          myICM->getAGMT();
          float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
          float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
          float pitch_lpf = pitch;
          float roll_lpf = roll;
          gy_pitch = gy_pitch + myICM->gyrY() * ((millis() - start) / 1000);
          gy_roll = gy_roll + myICM->gyrX() * ((millis() - start) / 1000);
          comp_roll = ((comp_roll + gy_roll) * (1 - THETA)) + THETA * roll;
          comp_pitch = ((comp_pitch + gy_pitch) * (1 - THETA)) + THETA * pitch;
        }
        float last_time = millis();
        while (millis() - start < duration) {
          if (myICM->dataReady()) {
            myICM->getAGMT();
            float dt = (millis() - last_time) / 1000.0;
            tx_estring_value.clear();
            float pitch = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
            float roll = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
            pitch_lpf = ALPHA * pitch + (1 - ALPHA) * pitch_lpf;
            roll_lpf = ALPHA * roll + (1 - ALPHA) * roll_lpf;

            gy_pitch = gy_pitch + myICM->gyrY() * dt;
            gy_roll = gy_roll + myICM->gyrX() * dt;

            comp_roll = ((comp_roll + gy_roll) * (1 - THETA)) + THETA * roll_lpf;
            comp_pitch =
              ((comp_pitch + gy_pitch) * (1 - THETA)) + THETA * pitch_lpf;

            tx_estring_value.clear();
            tx_estring_value.append(gy_pitch);
            tx_estring_value.append(",");
            tx_estring_value.append(gy_roll);
            tx_estring_value.append(",");

            tx_estring_value.append(pitch);
            tx_estring_value.append(",");
            tx_estring_value.append(roll);
            tx_estring_value.append(",");
            tx_estring_value.append(pitch_lpf);
            tx_estring_value.append(",");
            tx_estring_value.append(roll_lpf);
            tx_estring_value.append(",");
            tx_estring_value.append(comp_pitch);
            tx_estring_value.append(",");
            tx_estring_value.append(comp_roll);
            tx_estring_value.append(",");
            tx_estring_value.append((float)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(20);  // about 100Hz sampling rate accounting for time to send
                        // data
            last_time = millis();
          }
        }
        break;
      }

    case SEND_IMU_DATA:
      {
        // Build the data arrays
        int i = 1;
        float dt = 0, start = millis(), last_time = start;
        times[0] = start;
        while ((millis()-start) < duration) {
          if (myICM->dataReady()) {
            myICM->getAGMT();
            dt = (millis() - last_time) / 1000.0;
            pitch_a[i] = atan2(myICM->accY(), myICM->accZ()) * 180 / M_PI;
            roll_a[i] = atan2(myICM->accX(), myICM->accZ()) * 180 / M_PI;
            pitch_g[i] = pitch_g[i - 1] + myICM->gyrY() * dt;
            roll_g[i] = roll_g[i - 1] + myICM->gyrX() * dt;
            yaw_g[i] = yaw_g[i - 1] + myICM->gyrZ() * dt;
            comp_pitch[i] = ((comp_pitch[i - 1] + pitch_g[i]) * (1 - THETA)) + THETA * pitch_g[i - 1];
            comp_roll[i] = ((comp_roll[i - 1] + roll_g[i]) * (1 - THETA)) + THETA * roll_g[i - 1];
            times[i] = millis();
            last_time = millis();
            i++;
          }
        }
        // Send the data back comma separated
        for (int k = 0; k < i; k++) {
          tx_estring_value.clear();
          tx_estring_value.append(pitch_a[k]);
          tx_estring_value.append(",");
          tx_estring_value.append(roll_a[k]);
          tx_estring_value.append(",");
          tx_estring_value.append(pitch_g[k]);
          tx_estring_value.append(",");
          tx_estring_value.append(roll_g[k]);
          tx_estring_value.append(",");
          tx_estring_value.append(yaw_g[k]);
          tx_estring_value.append(",");
          tx_estring_value.append(comp_pitch[k]);
          tx_estring_value.append(",");
          tx_estring_value.append(comp_roll[k]);
          tx_estring_value.append(",");
          tx_estring_value.append(times[k]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
        break;
      }
    /*
   * The default case may not capture all types of invalid commands.
   * It is safer to validate the command string on the central device (in
   * python) before writing to the characteristic.
   */
    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

// #define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT \
  SPI  // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN \
  2  // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT \
  Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper
// is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif

void setup() {

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // myICM.enableDebugging(); // Uncomment this line to enable helpful debug
  // messages on Serial
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  BLE.addService(testService);

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

  bool initialized = false;
  while (!initialized) {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
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

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

void read_data(ICM_20948_I2C *myICM) {
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written()) {
    handle_command(myICM);
  }
}

void write_data() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {

    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);

    if (tx_float_value > 10000) {
      tx_float_value = 0;
    }

    previousMillis = currentMillis;
  }
}

void loop() {
  BLEDevice central = BLE.central();

  float pitch_a = 0, roll_a = 0, pitch_g = 0, roll_g = 0, yaw_g = 0, dt = 0,
        pitch = 0, roll = 0, yaw = 0;
  if (myICM.dataReady()) {
    myICM.getAGMT();
    printScaledAGMT(&myICM);
  }

  // If a central is connected to the peripheral
  if (central) {
    // Send data
    write_data();

    // Read data
    read_data(&myICM);
  }
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val) {
  if (val > 0) {
    SERIAL_PORT.print(" ");
    if (val < 10000) {
      SERIAL_PORT.print("0");
    }
    if (val < 1000) {
      SERIAL_PORT.print("0");
    }
    if (val < 100) {
      SERIAL_PORT.print("0");
    }
    if (val < 10) {
      SERIAL_PORT.print("0");
    }
  } else {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000) {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000) {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100) {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10) {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt) {
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    SERIAL_PORT.print("-");
  } else {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      SERIAL_PORT.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    SERIAL_PORT.print(-val, decimals);
  } else {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor) {
#else
void printScaledAGMT(ICM_20948_I2C *sensor) {
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
