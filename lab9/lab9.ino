#include "ICM_20948.h"
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include <Wire.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "ICM_20948.h"
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
#define WIRE_PORT Wire

float times[MAX_DATA_SIZE];
int pwm_vals[MAX_DATA_SIZE];
int imu_vals[MAX_DATA_SIZE];
int tof_vals[MAX_DATA_SIZE];
size_t curr_idx = 0;
int pid_target = 90;

// Set during start PID
bool is_pid_running = false;
float kp = 0.1;
float ki = 0.0;
float kd = 0.0;

float accumulated_error = 0.0;
unsigned long last_pid_time = 0;

enum command_types {
  PING,
  SEND_PID_DATA,
  START_PID,
  STOP_PID,
  SET_PID_GAINS,
  SET_SETPOINT
};

EString tx_estring_value;
RobotCommand robot_cmd(":|");

void drive_in_a_straight_line(int direction, int pwm, float calib) {
  if (direction) { // forward

    analogWrite(0, pwm * calib);
    analogWrite(1, 0);
    analogWrite(2, pwm * calib);
    analogWrite(3, 0);

  } else {
    // reverse
    analogWrite(0, 0);
    analogWrite(1, pwm * calib);
    analogWrite(2, 0);
    analogWrite(3, pwm * calib);
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
      tx_estring_value.append(imu_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(pwm_vals[i]);
      tx_estring_value.append("|");
      tx_estring_value.append(tof_vals[i]);
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
    curr_idx = 0;
    break;
  }
  case STOP_PID: {
    is_pid_running = false;
    stop();
    // curr_idx = 0;
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

  case SET_SETPOINT: {
    float new_setpoint;
    success = robot_cmd.get_next_value(new_setpoint);
    if (!success) {
      return;
    }
    pid_target = new_setpoint;
    Serial.println("Setpoint set!");
    Serial.print(new_setpoint);
    break;
  }
  default:
    break;
  }
}

float prev_error = 0.0;
int pid_control(int curr_angle) {
  int error = pid_target - curr_angle;

  unsigned long current_time = millis();
  float dt = (float)(current_time - last_pid_time) / 1000.0; // Convert ms to
  last_pid_time = current_time;

  if (dt > 0) {
    accumulated_error += error * dt;
  }

  // Calculate P and I terms
  float p_term = kp * error;
  float error_rate = 0.0;
  if (dt > 0) {
    error_rate = (error - prev_error) / dt;
  }
  float d_term = kd * error_rate;
  float i_term = ki * accumulated_error;
  if (i_term > 200) {
    i_term = 200;
  } else if (i_term < -200) {
    i_term = -200;
  }

  float pwm = p_term + d_term + i_term;
  prev_error = error;

  if (pwm > 0 && pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }

  return pwm;
}

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif
#define AD0_VAL 1
#define SPI_PORT SPI

SFEVL53L1X distance_sensor_1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distance_sensor_2;

void setup() {

  Serial.begin(115200);
  delay(100);

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // Left Wheels
  pinMode(0, OUTPUT); // xIN2
  pinMode(1, OUTPUT); // xIN1

  // Right Wheels
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

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

  // Check success
  if (!success) {
    Serial.println("Enabling DMP failed!");
    while (1) {
      // Freeze
    }
  }
  Serial.println("DMP configuration successful!");

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

  // Tof Sensors
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
  Serial.println("Both sensors online!");
}

float yaw_gy = 0.0;
float gy_angle = 0.0;

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
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();

  while (central.connected()) {
    if (rx_characteristic_string.written()) {
      handle_command();
    }

    if (is_pid_running) {
      int pwm;
      for (int i = 0; i < 360; i += 25) {
        pid_target = yaw_gy + i;
        unsigned long start = millis();
        while ((millis() - start) < 500) {
          unsigned long current_time = millis();
          record_dmp_data();
          pwm = pid_control(yaw_gy);

          // Drive motors with scaled PWM and proper direction
          if (abs(pwm) > 0) {
            // Scale PWM to avoid deadband
            int scaled_pwm;
            if (pwm > 0) {
              scaled_pwm = map(pwm, 0, 255, 70, 255);
              // Serial.print("Scaled_pwm: ");
              // Serial.println(scaled_pwm);
              drive_in_a_straight_line(1, scaled_pwm, 1.25); // forward
            } else {
              scaled_pwm = map(abs(pwm), 0, 255, 70, 255);
              // Serial.print("Scaled_pwm: ");
              // Serial.println(scaled_pwm);
              drive_in_a_straight_line(0, scaled_pwm, 1.25); // backward
            }
          } else {
            stop(); // When PWM is exactly 0
          }
        }
        // Stop and collect TOF Readings
        stop();
        for (int k = 0; k < 4; k++) {
          distance_sensor_1.startRanging();
          while (!distance_sensor_1.checkForDataReady()) {
            delay(1);
          }
          int distance = distance_sensor_1.getDistance();
          // Log the data
          // Log the data
          if (curr_idx < MAX_DATA_SIZE) {
            times[curr_idx] = (float)millis();
            pwm_vals[curr_idx] = pwm;
            imu_vals[curr_idx] = yaw_gy;
            tof_vals[curr_idx] = distance;
            curr_idx++;
          }
          distance_sensor_1.clearInterrupt();
          distance_sensor_1.stopRanging();
        }
      }
    }
  }

  stop();
}