/*
  Arduino Nano motor/encoder controller.

  Commands:
    ENCREAD <encoder>
    ENCRESET <encoder>
    IMUANGLE
    PING
    MOTCFG <motor> <pwm_pin> <dir_pin> <encoder>
           <motor_inverted> <encoder_reversed> <counts_per_rev>
           <kp> <ki> <kd> <static_pwm> <pwm_per_deg_per_sec>
    MOTPWR <motor> <-1.0..1.0>
    MOTVEL <motor> <deg_per_sec>
    MOTVELREAD <motor>

  Responses:
    OK
    ERR <message>
    VALUE <number>
*/

#include <Wire.h>

const uint8_t ENCODER_1_PIN_A = 2;
const uint8_t ENCODER_1_PIN_B = 4;
const uint8_t ENCODER_2_PIN_A = 3;
const uint8_t ENCODER_2_PIN_B = 5;
const uint8_t MOTOR_COUNT = 2;

const uint8_t BNO055_ADDRESS_PRIMARY = 0x28;
const uint8_t BNO055_ADDRESS_ALT = 0x29;
const uint8_t BNO055_CHIP_ID_REG = 0x00;
const uint8_t BNO055_CHIP_ID = 0xA0;
const uint8_t BNO055_PAGE_ID_REG = 0x07;
const uint8_t BNO055_EULER_HEADING_LSB_REG = 0x1A;
const uint8_t BNO055_UNIT_SEL_REG = 0x3B;
const uint8_t BNO055_OPR_MODE_REG = 0x3D;
const uint8_t BNO055_PWR_MODE_REG = 0x3E;
const uint8_t BNO055_SYS_TRIGGER_REG = 0x3F;
const uint8_t BNO055_MODE_CONFIG = 0x00;
const uint8_t BNO055_MODE_NDOF = 0x0C;
const uint8_t BNO055_POWER_NORMAL = 0x00;

const unsigned long CONTROL_INTERVAL_US = 50000;
const float VELOCITY_FILTER_ALPHA = 0.25;
const float INTEGRAL_ACTIVE_ERROR_DEG_PER_SEC = 75.0;
const float INTEGRAL_LIMIT = 150.0;
const float TARGET_ACCEL_LIMIT_DEG_PER_SEC2 = 1200.0;
const float PWM_SLEW_LIMIT_PER_SEC = 1000.0;

struct MotorController {
  bool configured;
  bool enabled;
  bool motorInverted;
  bool encoderReversed;
  bool filterReady;
  uint8_t pwmPin;
  uint8_t dirPin;
  uint8_t encoderIndex;
  float countsPerRev;
  float targetDegPerSec;
  float setpointDegPerSec;
  float measuredDegPerSec;
  float appliedPower;
  float staticPwm;
  float pwmPerDegPerSec;
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;
  float lastDtSec;
  long lastCount;
  unsigned long lastMicros;
};

volatile long encoderCounts[2] = {0, 0};
MotorController motors[MOTOR_COUNT];
bool imuReady = false;
uint8_t imuAddress = BNO055_ADDRESS_PRIMARY;

const int BUFFER_SIZE = 128;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

float absFloat(float value);
float clampFloat(float value, float low, float high);
float stepToward(float current, float target, float maxStep);
float zeroFloor(float value);
long readEncoderCount(uint8_t encoderIndex);
void resetEncoderCount(uint8_t encoderIndex);
bool readArgs(char *args[], uint8_t count);
bool validEncoder(uint8_t encoderIndex);
bool getEncoderArg(uint8_t &encoderIndex);
bool getMotorArg(uint8_t &motorIndex);
bool setupIMU();
bool setupIMUAtAddress(uint8_t address);
bool imuWrite8(uint8_t reg, uint8_t value);
bool imuRead8(uint8_t reg, uint8_t &value);
bool imuReadLen(uint8_t reg, uint8_t *values, uint8_t length);
bool readIMUAngle(float &angleDeg);
void resetControllerState(MotorController &motor);
void stopMotor(MotorController &motor);
void syncMotorsForEncoder(uint8_t encoderIndex);
void readSerial();
void handleCommand(char *line);
void handleEncoderRead();
void handleEncoderReset();
void handleIMUAngle();
void handlePing();
void handleMotorConfig();
void handleMotorPower();
void handleMotorVelocity();
void handleMotorVelocityRead();
float readMotorVelocity(MotorController &motor);
bool updateVelocityEstimate(MotorController &motor);
void updateControllers();
void updateController(MotorController &motor);
void applyMotorPower(MotorController &motor, float power);
void updateEncoder(uint8_t index, uint8_t pinA, uint8_t pinB);
void handleEncoder1Change();
void handleEncoder2Change();

float absFloat(float value) {
  return value < 0.0 ? -value : value;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float stepToward(float current, float target, float maxStep) {
  float delta = target - current;

  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

float zeroFloor(float value) {
  return value < 0.0 ? 0.0 : value;
}

long readEncoderCount(uint8_t encoderIndex) {
  noInterrupts();
  long value = encoderCounts[encoderIndex - 1];
  interrupts();
  return value;
}

void resetEncoderCount(uint8_t encoderIndex) {
  noInterrupts();
  encoderCounts[encoderIndex - 1] = 0;
  interrupts();
}

bool readArgs(char *args[], uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    args[i] = strtok(NULL, " ");

    if (args[i] == NULL) {
      Serial.println("ERR missing_argument");
      return false;
    }
  }

  return true;
}

bool validEncoder(uint8_t encoderIndex) {
  return encoderIndex == 1 || encoderIndex == 2;
}

bool getEncoderArg(uint8_t &encoderIndex) {
  char *arg = strtok(NULL, " ");

  if (arg == NULL) {
    Serial.println("ERR missing_encoder");
    return false;
  }

  encoderIndex = (uint8_t)atoi(arg);

  if (!validEncoder(encoderIndex)) {
    Serial.println("ERR invalid_encoder");
    return false;
  }

  return true;
}

bool getMotorArg(uint8_t &motorIndex) {
  char *arg = strtok(NULL, " ");

  if (arg == NULL) {
    Serial.println("ERR missing_motor");
    return false;
  }

  motorIndex = (uint8_t)atoi(arg);

  if (motorIndex < 1 || motorIndex > MOTOR_COUNT) {
    Serial.println("ERR invalid_motor");
    return false;
  }

  motorIndex--;
  return true;
}

bool setupIMU() {
  if (setupIMUAtAddress(BNO055_ADDRESS_PRIMARY)) {
    return true;
  }

  return setupIMUAtAddress(BNO055_ADDRESS_ALT);
}

bool setupIMUAtAddress(uint8_t address) {
  imuAddress = address;

  uint8_t chipId = 0;

  if (!imuRead8(BNO055_CHIP_ID_REG, chipId) || chipId != BNO055_CHIP_ID) {
    return false;
  }

  if (!imuWrite8(BNO055_OPR_MODE_REG, BNO055_MODE_CONFIG)) return false;
  delay(25);

  if (!imuWrite8(BNO055_PAGE_ID_REG, 0)) return false;
  if (!imuWrite8(BNO055_PWR_MODE_REG, BNO055_POWER_NORMAL)) return false;
  delay(10);

  if (!imuWrite8(BNO055_SYS_TRIGGER_REG, 0)) return false;
  delay(10);

  if (!imuWrite8(BNO055_UNIT_SEL_REG, 0)) return false;
  if (!imuWrite8(BNO055_OPR_MODE_REG, BNO055_MODE_NDOF)) return false;
  delay(20);

  return true;
}

bool imuWrite8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuRead8(uint8_t reg, uint8_t &value) {
  return imuReadLen(reg, &value, 1);
}

bool imuReadLen(uint8_t reg, uint8_t *values, uint8_t length) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesRead = Wire.requestFrom(imuAddress, length);

  if (bytesRead != length) {
    while (Wire.available()) {
      Wire.read();
    }

    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    values[i] = Wire.read();
  }

  return true;
}

bool readIMUAngle(float &angleDeg) {
  uint8_t values[2];

  if (!imuReadLen(BNO055_EULER_HEADING_LSB_REG, values, 2)) {
    return false;
  }

  int16_t rawHeading = (int16_t)(((uint16_t)values[1] << 8) | values[0]);
  angleDeg = rawHeading / 16.0;

  while (angleDeg < 0.0) {
    angleDeg += 360.0;
  }

  while (angleDeg >= 360.0) {
    angleDeg -= 360.0;
  }

  return true;
}

void resetControllerState(MotorController &motor) {
  motor.integral = 0.0;
  motor.previousError = 0.0;
  motor.lastDtSec = 0.0;
  motor.measuredDegPerSec = 0.0;
  motor.filterReady = false;
  motor.lastCount = readEncoderCount(motor.encoderIndex);
  motor.lastMicros = micros();
}

void stopMotor(MotorController &motor) {
  motor.enabled = false;
  motor.targetDegPerSec = 0.0;
  motor.setpointDegPerSec = 0.0;
  motor.integral = 0.0;
  motor.previousError = 0.0;
  motor.appliedPower = 0.0;
  analogWrite(motor.pwmPin, 0);
}

void syncMotorsForEncoder(uint8_t encoderIndex) {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if (motors[i].configured && motors[i].encoderIndex == encoderIndex) {
      resetControllerState(motors[i]);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  pinMode(ENCODER_1_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_1_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_A), handleEncoder1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_A), handleEncoder2Change, CHANGE);

  Serial.println("READY");
}

void loop() {
  updateControllers();
  readSerial();
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        buffer[bufferIndex] = '\0';
        handleCommand(buffer);
        bufferIndex = 0;
      }
      continue;
    }

    if (bufferIndex >= BUFFER_SIZE - 1) {
      bufferIndex = 0;
      Serial.println("ERR buffer_overflow");
      continue;
    }

    buffer[bufferIndex++] = c;
  }
}

void handleCommand(char *line) {
  char *command = strtok(line, " ");

  if (command == NULL) {
    Serial.println("ERR empty_command");
  } else if (strcmp(command, "ENCREAD") == 0) {
    handleEncoderRead();
  } else if (strcmp(command, "ENCRESET") == 0) {
    handleEncoderReset();
  } else if (strcmp(command, "IMUANGLE") == 0) {
    handleIMUAngle();
  } else if (strcmp(command, "PING") == 0) {
    handlePing();
  } else if (strcmp(command, "MOTCFG") == 0) {
    handleMotorConfig();
  } else if (strcmp(command, "MOTPWR") == 0) {
    handleMotorPower();
  } else if (strcmp(command, "MOTVEL") == 0) {
    handleMotorVelocity();
  } else if (strcmp(command, "MOTVELREAD") == 0) {
    handleMotorVelocityRead();
  } else {
    Serial.println("ERR unknown_command");
  }
}

void handleEncoderRead() {
  uint8_t encoderIndex;

  if (!getEncoderArg(encoderIndex)) return;

  Serial.print("VALUE ");
  Serial.println(readEncoderCount(encoderIndex));
}

void handleEncoderReset() {
  uint8_t encoderIndex;

  if (!getEncoderArg(encoderIndex)) return;

  resetEncoderCount(encoderIndex);
  syncMotorsForEncoder(encoderIndex);
  Serial.println("OK");
}

void handleIMUAngle() {
  if (!imuReady) {
    imuReady = setupIMU();
  }

  if (!imuReady) {
    Serial.println("ERR imu_not_ready");
    return;
  }

  float angleDeg = 0.0;

  if (!readIMUAngle(angleDeg)) {
    Serial.println("ERR imu_read_failed");
    return;
  }

  Serial.print("VALUE ");
  Serial.println(angleDeg, 4);
}

void handlePing() {
  Serial.println("OK");
}

void handleMotorConfig() {
  char *args[12];

  if (!readArgs(args, 12)) return;

  uint8_t motorIndex = (uint8_t)atoi(args[0]);
  uint8_t encoderIndex = (uint8_t)atoi(args[3]);
  float countsPerRev = atof(args[6]);

  if (motorIndex < 1 || motorIndex > MOTOR_COUNT) {
    Serial.println("ERR invalid_motor");
    return;
  }

  if (!validEncoder(encoderIndex)) {
    Serial.println("ERR invalid_encoder");
    return;
  }

  if (countsPerRev <= 0.0) {
    Serial.println("ERR invalid_counts_per_rev");
    return;
  }

  MotorController &motor = motors[motorIndex - 1];

  if (motor.configured) {
    stopMotor(motor);
  }

  motor.configured = true;
  motor.enabled = false;
  motor.pwmPin = (uint8_t)atoi(args[1]);
  motor.dirPin = (uint8_t)atoi(args[2]);
  motor.encoderIndex = encoderIndex;
  motor.motorInverted = atoi(args[4]) != 0;
  motor.encoderReversed = atoi(args[5]) != 0;
  motor.countsPerRev = countsPerRev;
  motor.kp = atof(args[7]);
  motor.ki = atof(args[8]);
  motor.kd = atof(args[9]);
  motor.staticPwm = clampFloat(atof(args[10]), 0.0, 255.0);
  motor.pwmPerDegPerSec = zeroFloor(atof(args[11]));
  motor.targetDegPerSec = 0.0;
  motor.setpointDegPerSec = 0.0;
  motor.appliedPower = 0.0;

  pinMode(motor.pwmPin, OUTPUT);
  pinMode(motor.dirPin, OUTPUT);
  resetControllerState(motor);
  analogWrite(motor.pwmPin, 0);

  Serial.println("OK");
}

void handleMotorPower() {
  uint8_t motorIndex;

  if (!getMotorArg(motorIndex)) return;

  MotorController &motor = motors[motorIndex];

  if (!motor.configured) {
    Serial.println("ERR motor_not_configured");
    return;
  }

  char *powerArg = strtok(NULL, " ");

  if (powerArg == NULL) {
    Serial.println("ERR missing_power");
    return;
  }

  motor.enabled = false;
  motor.targetDegPerSec = 0.0;
  motor.setpointDegPerSec = 0.0;
  motor.integral = 0.0;
  motor.previousError = 0.0;
  applyMotorPower(motor, clampFloat(atof(powerArg), -1.0, 1.0) * 255.0);
  Serial.println("OK");
}

void handleMotorVelocity() {
  uint8_t motorIndex;

  if (!getMotorArg(motorIndex)) return;

  MotorController &motor = motors[motorIndex];

  if (!motor.configured) {
    Serial.println("ERR motor_not_configured");
    return;
  }

  char *velocityArg = strtok(NULL, " ");

  if (velocityArg == NULL) {
    Serial.println("ERR missing_velocity");
    return;
  }

  float velocity = atof(velocityArg);

  if (velocity == 0.0) {
    stopMotor(motor);
  } else {
    bool wasEnabled = motor.enabled;
    motor.targetDegPerSec = velocity;
    motor.enabled = true;

    if (!wasEnabled) {
      motor.setpointDegPerSec = 0.0;
      resetControllerState(motor);
    }
  }

  Serial.println("OK");
}

void handleMotorVelocityRead() {
  uint8_t motorIndex;

  if (!getMotorArg(motorIndex)) return;

  MotorController &motor = motors[motorIndex];

  if (!motor.configured) {
    Serial.println("ERR motor_not_configured");
    return;
  }

  Serial.print("VALUE ");
  Serial.println(readMotorVelocity(motor), 6);
}

float readMotorVelocity(MotorController &motor) {
  if (!motor.enabled) {
    updateVelocityEstimate(motor);
  }

  return motor.measuredDegPerSec;
}

void updateControllers() {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    updateController(motors[i]);
  }
}

bool updateVelocityEstimate(MotorController &motor) {
  if (!motor.configured) return false;

  unsigned long now = micros();
  unsigned long elapsedUs = now - motor.lastMicros;

  if (elapsedUs < CONTROL_INTERVAL_US) return false;

  long count = readEncoderCount(motor.encoderIndex);
  long deltaCount = count - motor.lastCount;
  float dt = elapsedUs / 1000000.0;
  float rawDegPerSec = (deltaCount * 360.0) / (motor.countsPerRev * dt);

  motor.lastCount = count;
  motor.lastMicros = now;
  motor.lastDtSec = dt;

  if (motor.encoderReversed) {
    rawDegPerSec = -rawDegPerSec;
  }

  if (motor.filterReady) {
    motor.measuredDegPerSec += VELOCITY_FILTER_ALPHA * (
      rawDegPerSec - motor.measuredDegPerSec
    );
  } else {
    motor.measuredDegPerSec = rawDegPerSec;
    motor.filterReady = true;
  }

  return true;
}

void updateController(MotorController &motor) {
  if (!updateVelocityEstimate(motor) || !motor.enabled) return;

  float dt = motor.lastDtSec;
  motor.setpointDegPerSec = stepToward(
    motor.setpointDegPerSec,
    motor.targetDegPerSec,
    TARGET_ACCEL_LIMIT_DEG_PER_SEC2 * dt
  );

  float error = motor.setpointDegPerSec - motor.measuredDegPerSec;

  if (absFloat(error) <= INTEGRAL_ACTIVE_ERROR_DEG_PER_SEC) {
    motor.integral += error * dt;
    motor.integral = clampFloat(motor.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  } else {
    motor.integral = 0.0;
  }

  float derivative = (error - motor.previousError) / dt;
  motor.previousError = error;

  float targetMagnitude = absFloat(motor.setpointDegPerSec);
  float direction = motor.setpointDegPerSec > 0.0 ? 1.0 : -1.0;
  float feedForward = direction * (
    motor.staticPwm + motor.pwmPerDegPerSec * targetMagnitude
  );
  float output = feedForward;

  output += motor.kp * error;
  output += motor.ki * motor.integral;
  output += motor.kd * derivative;

  applyMotorPower(
    motor,
    stepToward(motor.appliedPower, output, PWM_SLEW_LIMIT_PER_SEC * dt)
  );
}

void applyMotorPower(MotorController &motor, float power) {
  power = clampFloat(power, -255.0, 255.0);
  motor.appliedPower = power;

  if (motor.motorInverted) {
    power = -power;
  }

  if (power == 0.0) {
    analogWrite(motor.pwmPin, 0);
    return;
  }

  int pwm = (int)absFloat(power);

  digitalWrite(motor.dirPin, power > 0.0 ? HIGH : LOW);
  analogWrite(motor.pwmPin, pwm);
}

void updateEncoder(uint8_t index, uint8_t pinA, uint8_t pinB) {
  if (digitalRead(pinA) == digitalRead(pinB)) {
    encoderCounts[index]++;
  } else {
    encoderCounts[index]--;
  }
}

void handleEncoder1Change() {
  updateEncoder(0, ENCODER_1_PIN_A, ENCODER_1_PIN_B);
}

void handleEncoder2Change() {
  updateEncoder(1, ENCODER_2_PIN_A, ENCODER_2_PIN_B);
}
