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
size_t curr_idx = 0;
int pid_position_target = 304;

// Set during start PID
bool is_pid_running = false;
float kp = 0.1;
float ki = 0.0;
float kd = 0.0;

SFEVL53L1X distance_sensor_1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distance_sensor_2;

// Add these with other global variables at the top
float accumulated_error = 0.0;
unsigned long last_pid_time = 0;

enum command_types { PING, SEND_PID_DATA, START_PID, STOP_PID, SET_PID_GAINS };

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
  case SEND_PID_DATA: {
    size_t i;
    while (i < curr_idx && i < MAX_DATA_SIZE) {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(tof_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(pwm_vals[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      i++;
    }
    // reset the index
    curr_idx = 0;
    break;
  }
  case START_PID: {
    is_pid_running = true;
    accumulated_error = 0.0;  // Reset integral term
    last_pid_time = millis(); // Reset time
    break;
  }
  case STOP_PID: {
    is_pid_running = false;
    distance_sensor_1.stopRanging();
    distance_sensor_2.stopRanging();
    stop();
    curr_idx = 0;
    break;
  }
  case SET_PID_GAINS: {
    float new_kp;
    float new_ki;
    float new_kd;

    success = robot_cmd.get_next_value(new_kp);
    if (!success) {
      return;
    }
    success = robot_cmd.get_next_value(new_ki);
    if (!success) {
      return;
    }
    success = robot_cmd.get_next_value(new_kd);
    if (!success) {
      return;
    }
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;

    Serial.println("PID Gains set!");
    Serial.print(new_kp);
    Serial.print(", ");
    Serial.print(new_ki);
    Serial.print(", ");
    Serial.print(new_kd);
    break;
  }
  default:
    break;
  }
}

int pid_control(int curr_distance) {
  int error = pid_position_target - curr_distance;

  unsigned long current_time = millis();
  float dt =
      (float)(current_time - last_pid_time) / 1000.0; // Convert ms to seconds
  last_pid_time = current_time;

  if (dt > 0) {
    accumulated_error += error * dt;
  }

  // Calculate P and I terms
  float p_term = kp * error;
  float i_term = ki * accumulated_error;
  if (i_term > 200) {
    i_term = 200;
  } else if (i_term < -200) {
    i_term = -200;
  }
  float pwm = p_term + i_term;

  if (pwm > 0 && pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }

  return pwm;
}

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

int last_tof_reading = 0;
int second_last_tof_reading = 0;
unsigned long last_tof_time = 0;
unsigned long second_last_tof_time = 0;

int extrapolate_distance() {
  if (last_tof_time == 0 || second_last_tof_time == 0) {
    return last_tof_reading;
  }

  float time_diff = last_tof_time - second_last_tof_time;
  if (time_diff == 0)
    return last_tof_reading;

  float slope = (last_tof_reading - second_last_tof_reading) / time_diff;

  unsigned long current_time = millis();
  float time_since_last = current_time - last_tof_time;

  float estimated_distance = last_tof_reading + (slope * time_since_last);

  return (int)estimated_distance;
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();

  while (central.connected()) {
    if (rx_characteristic_string.written()) {
      handle_command();
    }

    if (is_pid_running) {

      // Add debug prints
      distance_sensor_1.startRanging();
      int pwm;
      if (distance_sensor_1.checkForDataReady()) {
        int dist = distance_sensor_1.getDistance();
        distance_sensor_1.clearInterrupt();
        distance_sensor_1.stopRanging();

        // Update history
        second_last_tof_reading = last_tof_reading;
        second_last_tof_time = last_tof_time;
        last_tof_reading = dist;
        last_tof_time = millis();

        pwm = pid_control(dist);

        // Log the data
        if (curr_idx < MAX_DATA_SIZE) {
          times[curr_idx] = (float)millis();
          pwm_vals[curr_idx] = pwm;
          tof_vals[curr_idx] = dist;
          curr_idx++;
        }
      } else {
        int estimated_dist = extrapolate_distance();
        pwm = pid_control(estimated_dist);
      }

      // Drive motors with scaled PWM and proper direction
      if (abs(pwm) > 0) {
        // Scale PWM to avoid deadband
        int scaled_pwm;
        if (pwm > 0) {
          scaled_pwm = map(pwm, 0, 255, 40, 255);
          drive_in_a_straight_line(1, scaled_pwm, 1.25); // forward
        } else {
          scaled_pwm = map(abs(pwm), 0, 255, 40, 255);
          drive_in_a_straight_line(0, scaled_pwm, 1.25); // backward
        }
      } else {
        stop(); // When PWM is exactly 0
      }
    }
  }

  // Stop Ranging
  distance_sensor_1.stopRanging();
  distance_sensor_2.stopRanging();
  stop();
}
