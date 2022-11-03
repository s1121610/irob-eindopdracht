#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

const int MOTOR_PIN1 = 3;
const int MOTOR_PIN2 = 5;

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float integral = 0;
float derivative = 0;
float prev_error = 0;

void backwards(int runtime, int speed);
void forwards(int runtime, int speed);

void setup(){
  Serial.begin(9600);
  Serial.println("Initializing...");

  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);

  Wire.begin();
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  float KP = 1;
  float KI = 0;
  float KD = 2;

  float state = gx;
  float desired_state = 0;

  float error = state - desired_state;

  integral += error;
  derivative = error - prev_error;
  prev_error = error;

  float P = KP * error;
  float I = KI * integral;
  float D = KD * derivative;

  float pid = P + I + D;

  Serial.println(pid);

  if (pid < 0) {
    forwards(10, map(pid, 0, 3000, 130, 0));
  } else if (pid > 0) {
    backwards(10, map(pid, -3000, 0, 130, 0));
  }

  
  delay(111);
}

void forwards(int runtime = 1000, int speed = 10){
	digitalWrite(MOTOR_PIN2, HIGH);
  analogWrite(MOTOR_PIN1, speed);
  delay(runtime);
	digitalWrite(MOTOR_PIN2, LOW);
}

void backwards(int runtime = 1000, int speed = 10){
	digitalWrite(MOTOR_PIN1, HIGH);
  analogWrite(MOTOR_PIN2, speed);
  delay(runtime);
	digitalWrite(MOTOR_PIN1, LOW);
}
