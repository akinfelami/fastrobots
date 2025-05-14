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
int imu_vals[MAX_DATA_SIZE];
float predicted_kalman_vals[MAX_DATA_SIZE];
size_t curr_idx = 0;
int pid_target = 180;

bool is_pid_running = false;
float kp = 2.5;
float ki = 0.02;
float kd = 0.7;

// Linear PID gains for distance control (separate from orientation PID)
float kp_linear = 0.09;
float ki_linear = 0.0005;
float kd_linear = 0.01;
float accumulated_error_linear = 0.0;
float prev_error_linear = 0.0;
unsigned long last_pid_time_linear = 0;

float accumulated_error = 0.0;
unsigned long last_pid_time = 0;

//////////////// Sensor Pins //////////////////
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3
#define WIRE_PORT Wire

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

void turn_around(int direction, int pwm, float calib) {
  if (direction) {
    analogWrite(0, 0);
    analogWrite(1, pwm * calib);
    analogWrite(2, 0);
    analogWrite(3, pwm * calib);
  } else {
    analogWrite(0, 0);
    analogWrite(1, pwm * calib);
    analogWrite(2, pwm * calib);
    analogWrite(3, 0);
  }
}

void stop() {
  analogWrite(0, 0);
  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
}

//////////////// Drift Control Variables //////////////////
bool is_drift_running =
    false; // Flag to indicate if the drift routine is active
bool drift_flag =
    false; // Flag to switch between forward drive and turning phase
int drive_forward_speed = 200; // PWM for forward movement
int drive_back_speed = 200;    // PWM for backward movement (after turn)
float drift_point =
    1500.0; // Distance threshold to initiate turn (mm), set to ~3ft
unsigned long reverse_start_time =
    0; // Time when reverse motion started (after turn)
const unsigned long REVERSE_DURATION =
    900; // Duration for reverse motion (milliseconds)
unsigned long start_time_drift = 0; // Timestamp when drift routine started
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

float yaw_gy = 0.0;
float gy_angle = 0.0;

float prev_error = 0.0;
//////////////// PID Controller //////////////////
// Calculates PWM output for turning based on current angle error
int pid_control(int curr_angle) {
  int error = pid_target - curr_angle;

  unsigned long current_time = millis();
  float time_delta =
      (float)(current_time - last_pid_time) / 1000.0; // Time step in seconds
  last_pid_time = current_time;

  // Integral term calculation (with anti-windup)
  if (time_delta > 0) {
    accumulated_error += error * time_delta;
    // Anti-windup (adjust limits as needed)
    accumulated_error =
        constrain(accumulated_error, -2000 / ki,
                  2000 / ki); // Example limits based on max PWM contribution
  }
  float i_term = ki * accumulated_error;

  // Derivative term calculation
  float error_rate = 0.0;
  if (time_delta > 0) {
    error_rate = (error - prev_error) / time_delta;
  }
  float d_term = kd * error_rate;
  prev_error = error;

  // Proportional term
  float p_term = kp * error;

  // Total PID output
  float pwm_float = p_term + i_term + d_term;

  // Clamp PWM output to valid range
  int pwm_int = constrain((int)pwm_float, -255, 255);

  return pwm_int;
}

//////////////// Linear PID Controller for Distance //////////////////
// Uses Kalman filter estimate for distance
int linear_pid_control(float current_distance, float target_distance) {
  int error = current_distance - target_distance; // match lab5 direction
  unsigned long current_time = millis();
  float dt = (float)(current_time - last_pid_time_linear) / 1000.0;
  last_pid_time_linear = current_time;

  if (dt > 0) {
    accumulated_error_linear += error * dt;
  }

  // Calculate P and I terms
  float p_term = kp_linear * error;
  float i_term = ki_linear * accumulated_error_linear;
  if (i_term > 200) {
    i_term = 200;
  } else if (i_term < -200) {
    i_term = -200;
  }
  float pwm = p_term + i_term;

  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < 0) {
    pwm = 0;
  }

  return (int)pwm;
}

int current_distance;

// NEW: Open-Loop Drift Constants and State
const unsigned long FORWARD_DURATION_OL = 900; // ms
const unsigned long STOP_DURATION_OL = 300;    // ms
const int TURN_PWM_OL = 150;                   // PWM for turning (NEEDS TUNING)
const unsigned long TURN_DURATION_OL =
    300; // ms for turning (NEEDS TUNING for ~90 deg)
const unsigned long REVERSE_DURATION_OL = 900; // ms

enum DriftPhase { INIT, FORWARD, STOP1, TURN, STOP2, REVERSE, DONE };
DriftPhase current_phase = INIT;
// END NEW

//////////////// Drift Routine //////////////////
void start_drift() {
  Serial.println("Starting drift routine...");
  distance_sensor_1.startRanging();
  // Wait for the first valid distance reading
  while (!distance_sensor_1.checkForDataReady()) {
    delay(1);
  }
  int current_distance = distance_sensor_1.getDistance();
  distance_sensor_1.clearInterrupt();
  distance_sensor_1.stopRanging(); // Stop continuous ranging briefly

  // Initialize Kalman filter state with the first reading
  mu = {(float)current_distance, 0.0}; // Assume initial velocity is 0
  sigma = {sigma_3 * sigma_3, 0, 0,
           100}; // Reset covariance (position variance based on sensor,
                 // velocity variance high)

  is_drift_running = true;
  drift_flag = false; // Start in the forward driving phase
  start_time_drift = millis();
  curr_idx = 0;             // Reset data logging index
  accumulated_error = 0.0;  // Reset PID integral term
  prev_error = 0.0;         // Reset PID derivative term
  last_pid_time = millis(); // Initialize PID timer

  accumulated_error_linear = 0.0;
  prev_error_linear = 0.0;
  last_pid_time_linear = millis();

  distance_sensor_1.startRanging(); // Restart continuous ranging
  record_dmp_data();
  pid_target = 90;
  current_phase = FORWARD; // Initialize phase for state machine
}

// filepath: /Users/akin/Desktop/Desktop/dev/fastrobots/lab8/lab8.ino

// Remove the old commented-out drift() function entirely

// NEW Open-Loop Drift Function
void drift() {
  unsigned long elapsed_time = millis() - start_time_drift;
  int current_pwm = 0; // Track PWM for logging

  // --- Sensor Readings ---
  record_dmp_data(); // Update IMU data (yaw_gy)

  int raw_distance = -1;
  if (distance_sensor_1.checkForDataReady()) {
    raw_distance = distance_sensor_1.getDistance();
    distance_sensor_1.clearInterrupt();
    // Keep ranging continuously
    current_distance = raw_distance; // Update global if needed
  }
  // --- End Sensor Readings ---

  // --- State Machine for Open-Loop Control ---
  switch (current_phase) {
  case FORWARD:
    if (elapsed_time < FORWARD_DURATION_OL) {
      drive_in_a_straight_line(1, drive_forward_speed, 1.25);
      current_pwm = drive_forward_speed;
    } else {
      stop();
      current_pwm = 0;
      current_phase = STOP1;
    }
    break;

  case STOP1:
    if (elapsed_time < FORWARD_DURATION_OL + STOP_DURATION_OL) {
      stop(); // Ensure stopped
      current_pwm = 0;
    } else {
      current_phase = TURN;
      current_pwm = 0; // Still stopped before turn starts
    }
    break;

  case TURN:
    // Turn right (direction 1). Adjust PWM/duration/direction as needed.
    if (elapsed_time <
        FORWARD_DURATION_OL + STOP_DURATION_OL + TURN_DURATION_OL) {
      turn_around(1, TURN_PWM_OL, 1.25);
      current_pwm = TURN_PWM_OL; // Log the turn PWM
    } else {
      stop();
      current_pwm = 0;
      current_phase = STOP2;
    }
    break;

  case STOP2:
    if (elapsed_time < FORWARD_DURATION_OL + STOP_DURATION_OL +
                           TURN_DURATION_OL + STOP_DURATION_OL) {
      stop(); // Ensure stopped
      current_pwm = 0;
    } else {
      current_phase = REVERSE;
      current_pwm = 0; // Still stopped before reverse starts
    }
    break;

  case REVERSE:
    if (elapsed_time < FORWARD_DURATION_OL + STOP_DURATION_OL +
                           TURN_DURATION_OL + STOP_DURATION_OL +
                           REVERSE_DURATION_OL) {
      drive_in_a_straight_line(1, drive_back_speed, 1.25);
      current_pwm = -drive_back_speed; // Log negative PWM for reverse
    } else {
      stop();
      current_pwm = 0;
      current_phase = DONE;
    }
    break;

  case DONE:
  default:  // Includes INIT state if somehow entered mid-run
    stop(); // Ensure stopped
    current_pwm = 0;
    if (is_drift_running) {     // Prevent multiple prints/stops
      is_drift_running = false; // Stop the drift routine
      distance_sensor_1.stopRanging();
      // distance_sensor_2.stopRanging(); // If sensor 2 was used
      Serial.println("Open-loop drift finished.");
    }
    break;
  }

  // --- Data Logging ---
  if (is_drift_running && curr_idx < MAX_DATA_SIZE) {
    times[curr_idx] = (float)elapsed_time;
    tof_vals[curr_idx] =
        raw_distance; // Log raw ToF (-1 if no new data read this cycle)
    pwm_vals[curr_idx] = current_pwm; // Log actual PWM applied in this phase
    imu_vals[curr_idx] = yaw_gy;      // Log current yaw
    predicted_kalman_vals[curr_idx] = -1; // Indicate no Kalman prediction
    curr_idx++;
  }
  // --- End Data Logging ---
}
// ... existing code ...

// void drift() {
//   int pwm;
//   distance_sensor_1.startRanging();
//   record_dmp_data();
//   float scaledSpeed = (drift_flag ? 0 : drive_forward_speed) / 100.0f;
//   Matrix<2, 1> mu_p = (Ad * mu) + (Bd * scaledSpeed);
//   Matrix<2, 2> sigma_p = (Ad * (sigma * (~Ad))) + sig_u;
//   int raw_distance = -1;
//   if (distance_sensor_1.checkForDataReady()) {
//     raw_distance = distance_sensor_1.getDistance();
//     distance_sensor_1.clearInterrupt();
//     distance_sensor_1.stopRanging();
//     distance_sensor_1.startRanging();
//     kf_update(mu_p, sigma_p, mu, sigma, raw_distance);
//     current_distance = raw_distance;
//   }

//   float estim_dist = raw_distance != -1 ? raw_distance : (float)mu(0, 0);
//   if (!drift_flag) {
//     if (millis() - start_time_drift < 900) {
//       if (curr_idx < MAX_DATA_SIZE) {
//         times[curr_idx] = (float)(millis() - start_time_drift);
//         tof_vals[curr_idx] = current_distance;
//         pwm_vals[curr_idx] = drive_forward_speed;
//         ;
//         imu_vals[curr_idx] = yaw_gy;
//         predicted_kalman_vals[curr_idx] = estim_dist;
//         curr_idx++;
//       }
//       drive_in_a_straight_line(1, drive_forward_speed, 1.25);
//     } else {
//       drift_flag = true;
//       stop();
//       delay(300);
//     }
//   } else {

//     // Drive and drive back
//     if (abs(pid_target - yaw_gy) > 5) { // if not close enough to target
//       pwm = pid_control(yaw_gy);
//       // Drive motors with scaled PWM and proper direction
//       if (abs(pwm) > 0) {
//         // Scale PWM to avoid deadband
//         int scaled_pwm;
//         if (pwm > 0) {
//           scaled_pwm = map(pwm, 1, 255, 70, 255); // Map from 1-255
//           turn_around(1, scaled_pwm, 1.25);
//         }
//       }
//     } else {
//       if (reverse_start_time ==
//           0) { // Check if timer hasn't been set for reverse yet
//         reverse_start_time = millis();
//       }
//       delay(100);
//       drive_in_a_straight_line(0, drive_back_speed, 1.25);
//       times[curr_idx] = (float)(millis() - start_time_drift);
//       tof_vals[curr_idx] = current_distance;
//       pwm_vals[curr_idx] = -drive_back_speed;
//       predicted_kalman_vals[curr_idx] = mu(0, 0);
//       imu_vals[curr_idx] = yaw_gy;
//       curr_idx++;
//       // Check duration ONLY after timer has started
//       if (reverse_start_time > 0 &&
//           (millis() - reverse_start_time > REVERSE_DURATION)) {
//         stop();
//         is_drift_running = false;
//         distance_sensor_1.stopRanging();
//         distance_sensor_2.stopRanging();
//         reverse_start_time = 0; // Reset timer flag for next run
//       }
//     }
//   }
// }

//////////////// Command Handling //////////////////
void handle_command() {
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());
  bool success;
  int cmd_type = -1;
  success = robot_cmd.get_command_type(cmd_type);
  if (!success) {
    Serial.println("Failed to parse command type.");
    return;
  }

  switch (cmd_type) {
  case PING:
    tx_estring_value.clear();
    tx_estring_value.append("PONG");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.println("Responded to PING.");
    break;

  case SEND_PID_DATA:
    Serial.print("Sending logged data points: ");
    Serial.println(curr_idx);
    for (size_t i = 0; i < curr_idx && i < MAX_DATA_SIZE; i++) {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(tof_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(pwm_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(predicted_kalman_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(imu_vals[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      delay(5); // Small delay to help BLE transmission buffer
    }
    curr_idx = 0; // Reset index after sending
    Serial.println("Finished sending data.");
    break;

  case START_PID: // Renamed from START_PID to START_DRIFT for clarity
    start_drift();
    break;

  case SET_PID_GAINS: {
    float new_kp, new_ki, new_kd;
    if (!robot_cmd.get_next_value(new_kp) ||
        !robot_cmd.get_next_value(new_ki) ||
        !robot_cmd.get_next_value(new_kd)) {
      Serial.println("Failed to parse PID gains.");
      return;
    }
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;
    Serial.print("PID Gains set: Kp=");
    Serial.print(kp);
    Serial.print(", Ki=");
    Serial.print(ki);
    Serial.print(", Kd=");
    Serial.println(kd);
    break;
  }

  case STOP_PID: // Renamed from STOP_PID to STOP_DRIFT for clarity
    Serial.println("Stopping drift routine via command.");
    is_drift_running = false; // Corrected variable name
    drift_flag = false;
    distance_sensor_1.stopRanging();
    distance_sensor_2.stopRanging(); // Stop both sensors
    stop();                          // Stop motors
    break;

  default:
    Serial.print("Unknown command received: ");
    Serial.println(cmd_type);
    break;
  }
}

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif
#define AD0_VAL 1
#define SPI_PORT SPI

//////////////// Setup and Main Loop //////////////////
void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(100);

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

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

  // Initialize ICM20948
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again..");
      delay(500);
    } else {
      initialized = true;
    }
  }

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP orientation sensor and set to 2000 dps
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) ==
              ICM_20948_Stat_Ok);

  // Configure DMP to output data at maximum ORD
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 1);

  // Check success
  if (!success) {
    Serial.println("Enabling DMP failed!");
    while (1) {
      // Freeze
    }
  }
  Serial.println("DMP configuration successful!");

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

double qw;
double qx;
double qy;
double qz;
double siny;
double cosy;
unsigned long time_since_last_reading = 0;

void record_dmp_data() {

  icm_20948_DMP_data_t dmp;
  myICM.readDMPdataFromFIFO(&dmp);

  // Check if the myICM has data
  if ((myICM.status != ICM_20948_Stat_Ok) &&
      (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail))
    return;

  // Check for Quat6 orientation data
  if ((dmp.header & DMP_header_bitmap_Quat6) <= 0) {
    Serial.println("No quaternion data available");
    return;
  }

  // Scale to +/- 1
  qx = ((double)dmp.Quat6.Data.Q1) / 1073741824.0;
  qy = ((double)dmp.Quat6.Data.Q2) / 1073741824.0;
  qz = ((double)dmp.Quat6.Data.Q3) / 1073741824.0;

  qw = sqrt(1.0 - min(((qx * qx) + (qy * qy) + (qz * qz)), 1.0));

  siny = 2.0 * (qw * qz + qx * qy);
  cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw_gy = (float)(atan2(siny, cosy) * 180.0 / PI);

  // Serial.print("YAW: ");
  // Serial.println(yaw_gy);
  // Only delay between readings if no more data is available
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
    delay(10);
  }
}

void loop() {
  BLEDevice central = BLE.central();
  while (central.connected()) {
    if (rx_characteristic_string.written())
      handle_command();
    if (is_drift_running)
      drift();
  }
  stop();
  distance_sensor_1.stopRanging();
  distance_sensor_2.stopRanging();
}
