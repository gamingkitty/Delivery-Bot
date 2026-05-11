/*
  Arduino Nano Serial GPIO/PWM Server

  Commands from Jetson:

  MODE <pin> <INPUT|OUTPUT|INPUT_PULLUP>
  WRITE <pin> <0|1>
  PWM <pin> <0-255>
  READ <pin>
  AREAD <pin>

  Example:
  MODE 5 OUTPUT
  PWM 5 128
  WRITE 7 1
  READ 8

  Responses:
  OK
  ERR <message>
  VALUE <number>
*/

#include <Servo.h>

Servo servos[14];
bool servoAttached[14] = {false};


const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
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