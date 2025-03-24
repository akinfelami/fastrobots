#include "ICM_20948.h"
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
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

///////////////// Data /////////////////
#define MAX_DATA_SIZE 10000

// Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

float times[MAX_DATA_SIZE];
int pwm_vals[MAX_DATA_SIZE];
int tof_vals[MAX_DATA_SIZE];
float vels[MAX_DATA_SIZE];

size_t curr_idx = 0;

// Set during start PID
bool is_step_response = false;
SFEVL53L1X distance_sensor_1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distance_sensor_2;

enum command_types { PING, START_STEP, STOP_STEP, COLLECT_DATA };

EString tx_estring_value;
RobotCommand robot_cmd(":|");

void drive_in_a_straight_line(int direction, int pwm, float calib) {
  if (direction) { // forward
    analogWrite(0, pwm * calib);
    analogWrite(1, 0);
    analogWrite(2, 0);
    analogWrite(3, pwm * calib);

  } else {
    // reverse
    analogWrite(0, 0);
    analogWrite(1, pwm * calib);
    analogWrite(2, pwm * calib);
    analogWrite(3, pwm * 0);
  }
}

void turn_right(int pwm) {
  analogWrite(0, pwm);
  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, pwm);
}

void turn_left(int pwm) {
  analogWrite(0, 0);
  analogWrite(1, pwm);
  analogWrite(2, pwm);
  analogWrite(3, 0);
}

void stop() {
  analogWrite(0, 0);
  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
}

int last_pos = 0;
unsigned long last_time = 0;

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
  case COLLECT_DATA: {
    size_t i;
    while (i < curr_idx && i < MAX_DATA_SIZE) {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(pwm_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(tof_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(vels[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      i++;
    }
    // reset the index
    curr_idx = 0;
    break;
  }
  case START_STEP: {
    is_step_response = true;
    break;
  }
  case STOP_STEP: {
    is_step_response = false;
    distance_sensor_1.stopRanging();
    distance_sensor_2.stopRanging();
    stop();
    curr_idx = 0;
    break;
  }

  default:
    break;
  }
}

void step_response(unsigned long test_duration, int step_pwm,
                   unsigned long start_time) {
  unsigned long current_time = start_time;

  while ((current_time - start_time) < test_duration &&
         curr_idx < MAX_DATA_SIZE) {
    // Apply step input
    drive_in_a_straight_line(0, step_pwm, 1.25);

    // Get sensor data
    distance_sensor_1.startRanging();
    if (distance_sensor_1.checkForDataReady()) {
      int current_distance = distance_sensor_1.getDistance();
      distance_sensor_1.clearInterrupt();

      // Get current timestamp
      current_time = millis();

      // Calculate velocity (change in position / change in time)
      float velocity = 0;
      if (curr_idx > 0) {
        float delta_time =
            (current_time - times[curr_idx - 1]) / 1000.0; // Convert to seconds
        if (delta_time > 0) {
          velocity = (current_distance - tof_vals[curr_idx - 1]) / delta_time;
        }
      }

      // Record data
      times[curr_idx] = current_time; // Record actual timestamp
      pwm_vals[curr_idx] = step_pwm;
      tof_vals[curr_idx] = current_distance;
      vels[curr_idx] = velocity;
      curr_idx++;
    }
    distance_sensor_1.stopRanging();
    current_time = millis();
  }
}

// void step_response() {
//   const unsigned long test_duration = 5000; // 5 seconds in milliseconds
//   const int step_pwm = 100;                 // PWM value for the step input

//   unsigned long start_time = millis();
//   unsigned long current_time = start_time;

//   // zero pwm for first 1 second

//   // Record initial state
//   if (curr_idx < MAX_DATA_SIZE) {
//     times[curr_idx] = current_time; // Record actual timestamp
//     pwm_vals[curr_idx] = 0;         // Initial PWM is 0
//     tof_vals[curr_idx] =
//         distance_sensor_1.getDistance(); // Get current distance
//     vels[curr_idx] = 0;                  // Initial velocity is 0
//     curr_idx++;
//   }

//   while ((current_time - start_time) < test_duration &&
//          curr_idx < MAX_DATA_SIZE) {
//     // Apply step input
//     drive_in_a_straight_line(0, step_pwm,
//                              1.25); // Forward direction with calibration 1.0

//     // Get sensor data
//     distance_sensor_1.startRanging();
//     if (distance_sensor_1.checkForDataReady()) {
//       int current_distance = distance_sensor_1.getDistance();
//       distance_sensor_1.clearInterrupt();

//       // Get current timestamp
//       current_time = millis();

//       // Calculate velocity (change in position / change in time)
//       float velocity = 0;
//       if (curr_idx > 0) {
//         float delta_time =
//             (current_time - times[curr_idx - 1]) / 1000.0; // Convert to
//             seconds
//         if (delta_time > 0) {
//           velocity = (current_distance - tof_vals[curr_idx - 1]) /
//           delta_time;
//         }
//       }

//       // Record data
//       times[curr_idx] = current_time; // Record actual timestamp
//       pwm_vals[curr_idx] = step_pwm;
//       tof_vals[curr_idx] = current_distance;
//       vels[curr_idx] = velocity;
//       curr_idx++;
//     }
//     distance_sensor_1.stopRanging();

//     current_time = millis();
//   }

//   // Stop motors after test

//   stop();
// }

void setup() {
  Wire.begin();

  Serial.begin(115200);

  // Left Wheels
  pinMode(0, OUTPUT); // xIN2
  pinMode(1, OUTPUT); // xIN1

  // Right Wheels
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  // To use two sensors
  pinMode(SHUTDOWN_PIN, OUTPUT);
  // XSHUT Low on sensor 1
  digitalWrite(SHUTDOWN_PIN, LOW);
  distance_sensor_2.setI2CAddress(0x77);
  digitalWrite(SHUTDOWN_PIN, HIGH);

  if (distance_sensor_1.begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println(
        "Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  if (distance_sensor_2.begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println(
        "Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  distance_sensor_1.setDistanceModeLong();
  distance_sensor_2.setDistanceModeLong();
  distance_sensor_1.setTimingBudgetInMs(20);
  distance_sensor_2.setTimingBudgetInMs(20);

  Serial.println("Both sensors online!");

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

const unsigned long step_time = 1500;

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();

  while (central.connected()) {
    if (rx_characteristic_string.written()) {
      handle_command();
    }
    distance_sensor_1.startRanging();
    if (is_step_response) {
      // Record initial state
      unsigned long start_time = millis();
      if (curr_idx < MAX_DATA_SIZE) {
        times[curr_idx] = start_time;
        pwm_vals[curr_idx] = 0; // Initial PWM is 0
        tof_vals[curr_idx] =
            distance_sensor_1.getDistance(); // Get current distance
        vels[curr_idx] = 0;                  // Initial velocity is 0
        curr_idx++;
      }
      step_response(100, 0, start_time);
      unsigned long second_phase = millis();

      step_response(3900, 100, second_phase);
      unsigned long third_phase = millis();

      step_response(1000, 0, third_phase);
      is_step_response = false;
    }

    // Stop Ranging
    distance_sensor_1.stopRanging();
    distance_sensor_2.stopRanging();
    stop();
  }
}
