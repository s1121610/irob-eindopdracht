#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

const int MOTOR_PIN1 = 5;
const int MOTOR_PIN2 = 3;
#define sampleTime  0.005

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

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
  #define targetAngle 0

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


  // #####################################################3
  // calculate the angle of inclination
    accAngle = atan2(ay, az)*RAD_TO_DEG;
    gyroRate = map(gx, -32768, 32767, -250, 250);
    gyroAngle = (float)gyroRate*sampleTime;  
    currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
    
    error = currentAngle - targetAngle;
    errorSum = errorSum + error;  
    errorSum = constrain(errorSum, -300, 300);
    //calculate output from P, I and D values
    pid = KP*(error) + KI*(errorSum)*sampleTime - KD*(currentAngle-prevAngle)/sampleTime;
    prevAngle = currentAngle;
  // #####################################################3
  

  //Serial.println(pid);

  
  int mapping = constrain(pid, -255, 255);
  Serial.print("PID = "); Serial.println(mapping);

  if (mapping < -10) {
    backwards(0, mapping);
  } else if (mapping > 10) {
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
