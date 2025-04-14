#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "ICM_20948.h"
#include "RobotCommand.h"
#include "SparkFun_VL53L1X.h"
#include <ArduinoBLE.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
using namespace BLA;

//////////////// BLE UUIDs //////////////////
#define BLE_UUID_TEST_SERVICE "b4735562-362d-4d17-b3e4-c4cd7256fa09"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite,
                                                  MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT,
                                               BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING,
                                                  BLERead | BLENotify,
                                                  MAX_MSG_SIZE);

//////////////// Data Logging //////////////////
#define MAX_DATA_SIZE 10000
float times[MAX_DATA_SIZE];
int pwm_vals[MAX_DATA_SIZE];
int tof_vals[MAX_DATA_SIZE];
float predicted_kalman_vals[MAX_DATA_SIZE];
size_t curr_idx = 0;

//////////////// Sensor Pins //////////////////
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

//////////////// Kalman Filter Variables //////////////////
float d = 0.000856;
float m = 0.00109;
Matrix<2, 2> A = {0, 1, 0, -d / m};
Matrix<2, 1> B = {0, 1 / m};
Matrix<1, 2> C = {1, 0};
float dt = 0.0980;
Matrix<2, 2> I = {1, 0, 0, 1};
Matrix<2, 2> Ad = I + (dt * A);
Matrix<2, 1> Bd = dt * B;
float sigma_1 = 31.94;
float sigma_3 = 20;
Matrix<2, 2> sig_u = {sigma_1 * sigma_1, 0, 0, sigma_1 *sigma_1};
Matrix<1, 1> sig_z = {sigma_3 * sigma_3};
Matrix<2, 2> sigma = {400, 0, 0, 100};
Matrix<2, 1> mu = {0.0, 0.0};

SFEVL53L1X distance_sensor_1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distance_sensor_2;

enum command_types { PING, SEND_PID_DATA, START_PID, STOP_PID, SET_PID_GAINS };

EString tx_estring_value;
RobotCommand robot_cmd(":|");

//////////////// Motor Functions //////////////////
void drive_in_a_straight_line(int direction, int pwm, float calib) {
  if (direction) {
    analogWrite(0, 0);
    analogWrite(1, pwm * calib);
    analogWrite(2, pwm * calib);
    analogWrite(3, 0);
  } else {
    analogWrite(0, pwm * calib);
    analogWrite(1, 0);
    analogWrite(2, 0);
    analogWrite(3, pwm * calib);
  }
}

void stop() {
  analogWrite(0, 0);
  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
}

//////////////// Flip Control Variables //////////////////
bool is_flip_running = false;
bool flip_flag = false;
int flip_forward_speed = 255;
int flip_reverse_speed = 255;
float flip_point = 1200.0;
;                                     // Set flip point to 1000mm (1 meter)
unsigned long forward_start_time = 0; // Time when forward motion started
const unsigned long FORWARD_DURATION =
    1400;                             // Forward motion duration (milliseconds)
unsigned long reverse_start_time = 0; // Time when reverse motion started
const unsigned long REVERSE_DURATION = 1400; // 3 seconds in millisecondsseconds
unsigned long start_time_flip = 0;

//////////////// Kalman Filter Update //////////////////
void kf_update(Matrix<2, 1> mu_p, Matrix<2, 2> sigma_p, Matrix<2, 1> &mu,
               Matrix<2, 2> &sigma, int distance) {
  Matrix<1, 1> sigma_m = (C * (sigma_p * (~C))) + sig_z;
  float sigma_m_inv = 1.0f / sigma_m(0, 0);
  Matrix<2, 1> k_gain = sigma_p * ((~C) * sigma_m_inv);
  Matrix<1, 1> y = {(float)distance};
  Matrix<1, 1> y_m = y - (C * mu_p);
  mu = mu_p + (k_gain * y_m);
  sigma = (I - (k_gain * C)) * sigma_p;
}

//////////////// Flip Routine //////////////////
void start_flip() {
  distance_sensor_1.startRanging();
  while (!distance_sensor_1.checkForDataReady()) {
    delay(1);
  }
  int current_distance = distance_sensor_1.getDistance();
  distance_sensor_1.clearInterrupt();
  distance_sensor_1.stopRanging();
  mu = {(float)current_distance, 0.0};
  is_flip_running = true;
  flip_flag = false;
  start_time_flip = millis();
  curr_idx = 0;
}

void flip() {
  distance_sensor_1.startRanging();
  float scaledSpeed =
      (flip_flag ? flip_reverse_speed : flip_forward_speed) / 100.0f;
  Matrix<2, 1> mu_p = (Ad * mu) + (Bd * scaledSpeed);
  Matrix<2, 2> sigma_p = (Ad * (sigma * (~Ad))) + sig_u;
  int raw_distance = -1;
  if (distance_sensor_1.checkForDataReady()) {
    raw_distance = distance_sensor_1.getDistance();
    distance_sensor_1.clearInterrupt();
    distance_sensor_1.stopRanging();
    distance_sensor_1.startRanging();
    kf_update(mu_p, sigma_p, mu, sigma, raw_distance);
  }
  int effective_distance = (raw_distance != -1) ? raw_distance : (int)mu(0, 0);

  if (!flip_flag) {
    // Check if the robot is at least 1 meter from the wall
    if (effective_distance > flip_point) {
      drive_in_a_straight_line(1, flip_forward_speed, 1.25);
      times[curr_idx] = (float)(millis() - start_time_flip);
      tof_vals[curr_idx] = effective_distance;
      pwm_vals[curr_idx] = flip_forward_speed;
      predicted_kalman_vals[curr_idx] = mu(0, 0);
      curr_idx++;
    } else {
      flip_flag = true;
      reverse_start_time = millis(); // Record when reverse motion starts
    }
  } else {
    drive_in_a_straight_line(0, flip_reverse_speed, 1.25);
    times[curr_idx] = (float)(millis() - start_time_flip);
    tof_vals[curr_idx] = effective_distance;
    pwm_vals[curr_idx] = -flip_reverse_speed;
    predicted_kalman_vals[curr_idx] = mu(0, 0);
    curr_idx++;
    // Check if reverse duration has elapsed
    if (millis() - reverse_start_time >= REVERSE_DURATION) {
      stop();
      tx_estring_value.clear();
      is_flip_running = false;
    }
  }
}

//////////////// Command Handling //////////////////
void handle_command() {
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());
  bool success;
  int cmd_type = -1;
  success = robot_cmd.get_command_type(cmd_type);
  if (!success)
    return;
  switch (cmd_type) {
  case PING:
    tx_estring_value.clear();
    tx_estring_value.append("PONG");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    break;
  case SEND_PID_DATA:
    for (size_t i = 0; i < curr_idx && i < MAX_DATA_SIZE; i++) {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(tof_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(pwm_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(predicted_kalman_vals[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    curr_idx = 0;
    break;
  case START_PID:
    start_flip();
    break;
  case STOP_PID:
    is_flip_running = false;
    distance_sensor_1.stopRanging();
    distance_sensor_2.stopRanging();
    stop();
    break;
  default:
    break;
  }
}

//////////////// Setup and Main Loop //////////////////
void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  distance_sensor_2.setI2CAddress(0x77);
  digitalWrite(SHUTDOWN_PIN, HIGH);
  if (distance_sensor_1.begin() != 0)
    while (1)
      ;
  if (distance_sensor_2.begin() != 0)
    while (1)
      ;
  distance_sensor_1.setDistanceModeLong();
  distance_sensor_2.setDistanceModeLong();
  distance_sensor_1.setTimingBudgetInMs(20);
  distance_sensor_2.setTimingBudgetInMs(20);
  BLE.begin();
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);
  BLE.addService(testService);
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  while (central.connected()) {
    if (rx_characteristic_string.written())
      handle_command();
    if (is_flip_running)
      flip();
  }
  stop();
  distance_sensor_1.stopRanging();
  distance_sensor_2.stopRanging();
}
