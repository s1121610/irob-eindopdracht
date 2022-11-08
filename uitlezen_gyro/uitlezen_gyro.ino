#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

const int MOTOR_PIN1 = 5;
const int MOTOR_PIN2 = 3;
const int sampleTime {1};

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float integral = 0;
float derivative = 0;
float prev_error = 0;

void backwards(int runtime, int speed);
void forwards(int runtime, int speed);

void setup(){
  Serial.begin(38400);
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

  //Serial.print("a/g:\t");
  // Serial.print(ax); Serial.print("\t");
  // Serial.print(ay); Serial.print("\t");
  //Serial.print(az); Serial.print("\t");
  // Serial.print("X"); Serial.print(gx); Serial.print("\t");
  // Serial.print("Y"); Serial.print(gy); Serial.print("\t");
  // Serial.print("Z"); Serial.println(gz);

  float KP = 10;
  float KI = 30;
  float KD = 0.1;

  float state = ay;
  float desired_state = 16450;

  float error = state - desired_state;

  integral += error;
  derivative = error - prev_error;
  prev_error = error;

  float P = KP * error;
  float I = KI * integral * sampleTime;
  float D = KD * derivative / sampleTime;

  float pid = P + I - D;

  //Serial.println(pid);

  
  int mapping = constrain(pid, 0, 255);
  Serial.print("PID = "); Serial.println(pid);

  if (az < 0) {
    backwards(0, mapping);
  } else if (az > 0) {
    //long mappingf = map(pid, stable_point, 6000, 0, 130);
    // Serial.print("Map forwards = "); Serial.println(mappingf);
    forwards(0, mapping);

  } else{
    // OFF nnnnbvkhvhjvhj

  }

}

void forwards(int runtime = 0, int speed = 10){
  digitalWrite(MOTOR_PIN1, LOW);
	digitalWrite(MOTOR_PIN2, HIGH);
  analogWrite(MOTOR_PIN1, speed);
  delay(runtime);
	// digitalWrite(MOTOR_PIN2, LOW);
}

void backwards(int runtime = 0, int speed = 10){
  digitalWrite(MOTOR_PIN2, LOW);
	digitalWrite(MOTOR_PIN1, HIGH);
  analogWrite(MOTOR_PIN2, speed);
  delay(runtime);
	// digitalWrite(MOTOR_PIN1, LOW);
}

ISR(TIMER_COPA_vect) {

}