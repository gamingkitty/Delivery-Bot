/*
  Arduino Nano Serial GPIO/PWM Server

  Commands from Jetson:

  MODE <pin> <INPUT|OUTPUT|INPUT_PULLUP>
  WRITE <pin> <0|1>
  PWM <pin> <0-255>
  READ <pin>
  AREAD <pin>
  ENCREAD <1|2>
  ENCRESET <1|2>

  Example:
  MODE 5 OUTPUT
  PWM 5 128
  WRITE 7 1
  READ 8
  ENCREAD 1

  Responses:
  OK
  ERR <message>
  VALUE <number>
*/

#include <Servo.h>

Servo servos[14];
bool servoAttached[14] = {false};

const uint8_t ENCODER_1_PIN_A = 2;
const uint8_t ENCODER_1_PIN_B = 4;
const uint8_t ENCODER_2_PIN_A = 3;
const uint8_t ENCODER_2_PIN_B = 5;

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;


const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void handleEncoderRead();
void handleEncoderReset();
bool getEncoderIndex(int &encoderIndex);
long readEncoderCount(volatile long &count);
void resetEncoderCount(volatile long &count);
void handleEncoder1Rise();
void handleEncoder2Rise();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(ENCODER_1_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_1_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_A), handleEncoder1Rise, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_A), handleEncoder2Rise, CHANGE);
  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        buffer[bufferIndex] = '\0';
        handleCommand(buffer);
        bufferIndex = 0;
      }
    } else {
      if (bufferIndex < BUFFER_SIZE - 1) {
        buffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0;
        Serial.println("ERR buffer_overflow");
      }
    }
  }
}

void handleCommand(char *cmd) {
  char *command = strtok(cmd, " ");

  if (command == NULL) {
    Serial.println("ERR empty_command");
    return;
  }

  if (strcmp(command, "MODE") == 0) {
    handleMode();
  }
  else if (strcmp(command, "WRITE") == 0) {
    handleWrite();
  }
  else if (strcmp(command, "PWM") == 0) {
    handlePwm();
  }
  else if (strcmp(command, "READ") == 0) {
    handleRead();
  }
  else if (strcmp(command, "AREAD") == 0) {
    handleAnalogRead();
  }
  else if (strcmp(command, "ENCREAD") == 0) {
    handleEncoderRead();
  }
  else if (strcmp(command, "ENCRESET") == 0) {
    handleEncoderReset();
  }
  else if (strcmp(command, "SERVO") == 0) {
    handleServo();
  }
  else {
    Serial.println("ERR unknown_command");
  }
}

bool getPinAndValue(int &pin, int &value) {
  char *pinStr = strtok(NULL, " ");
  char *valueStr = strtok(NULL, " ");

  if (pinStr == NULL || valueStr == NULL) {
    Serial.println("ERR missing_argument");
    return false;
  }

  pin = atoi(pinStr);
  value = atoi(valueStr);
  return true;
}

bool getPin(int &pin) {
  char *pinStr = strtok(NULL, " ");

  if (pinStr == NULL) {
    Serial.println("ERR missing_pin");
    return false;
  }

  pin = atoi(pinStr);
  return true;
}

bool getEncoderIndex(int &encoderIndex) {
  char *encoderStr = strtok(NULL, " ");

  if (encoderStr == NULL) {
    Serial.println("ERR missing_encoder");
    return false;
  }

  encoderIndex = atoi(encoderStr);

  if (encoderIndex != 1 && encoderIndex != 2) {
    Serial.println("ERR invalid_encoder");
    return false;
  }

  return true;
}

long readEncoderCount(volatile long &count) {
  noInterrupts();
  long value = count;
  interrupts();
  return value;
}

void resetEncoderCount(volatile long &count) {
  noInterrupts();
  count = 0;
  interrupts();
}

void handleMode() {
  char *pinStr = strtok(NULL, " ");
  char *modeStr = strtok(NULL, " ");

  if (pinStr == NULL || modeStr == NULL) {
    Serial.println("ERR missing_argument");
    return;
  }

  int pin = atoi(pinStr);

  if (strcmp(modeStr, "INPUT") == 0) {
    pinMode(pin, INPUT);
  }
  else if (strcmp(modeStr, "OUTPUT") == 0) {
    pinMode(pin, OUTPUT);
  }
  else if (strcmp(modeStr, "INPUT_PULLUP") == 0) {
    pinMode(pin, INPUT_PULLUP);
  }
  else {
    Serial.println("ERR invalid_mode");
    return;
  }

  Serial.println("OK");
}

void handleWrite() {
  int pin, value;

  if (!getPinAndValue(pin, value)) {
    return;
  }

  digitalWrite(pin, value ? HIGH : LOW);
  Serial.println("OK");
}

void handlePwm() {
  int pin, value;

  if (!getPinAndValue(pin, value)) {
    return;
  }

  value = constrain(value, 0, 255);
  analogWrite(pin, value);
  Serial.println("OK");
}

void handleRead() {
  int pin;

  if (!getPin(pin)) {
    return;
  }

  int value = digitalRead(pin);
  Serial.print("VALUE ");
  Serial.println(value);
}

void handleAnalogRead() {
  int pin;

  if (!getPin(pin)) {
    return;
  }

  int value = analogRead(pin);
  Serial.print("VALUE ");
  Serial.println(value);
}

void handleEncoderRead() {
  int encoderIndex;

  if (!getEncoderIndex(encoderIndex)) {
    return;
  }

  long value = encoderIndex == 1 ? readEncoderCount(encoder1Count) : readEncoderCount(encoder2Count);
  Serial.print("VALUE ");
  Serial.println(value);
}

void handleEncoderReset() {
  int encoderIndex;

  if (!getEncoderIndex(encoderIndex)) {
    return;
  }

  if (encoderIndex == 1) {
    resetEncoderCount(encoder1Count);
  } else {
    resetEncoderCount(encoder2Count);
  }

  Serial.println("OK");
}

void handleServo() {
  int pin, angle;

  if (!getPinAndValue(pin, angle)) {
    return;
  }

  if (pin < 0 || pin > 13) {
    Serial.println("ERR invalid_servo_pin");
    return;
  }

  angle = constrain(angle, 0, 180);

  if (!servoAttached[pin]) {
    servos[pin].attach(pin);
    servoAttached[pin] = true;
  }

  servos[pin].write(angle);
  Serial.println("OK");
}

void handleEncoder1Rise() {
  bool aState = digitalRead(ENCODER_1_PIN_A);
  bool bState = digitalRead(ENCODER_1_PIN_B);

  if (aState == bState) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

void handleEncoder2Rise() {
  bool aState = digitalRead(ENCODER_2_PIN_A);
  bool bState = digitalRead(ENCODER_2_PIN_B);

  if (aState == bState) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}
