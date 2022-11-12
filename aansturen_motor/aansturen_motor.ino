const int MOTOR_PIN1 {5};
const int MOTOR_PIN2 {3};
	
void setup() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
	Serial.begin(9600);
}

// Turn the wheels forwards for 1 second (by default)
void vooruit(int runtime = 1000, int speed = 10){
	digitalWrite(MOTOR_PIN2, HIGH);
  analogWrite(MOTOR_PIN1, speed);
  delay(runtime);
	digitalWrite(MOTOR_PIN2, LOW);
  analogWrite(MOTOR_PIN1, 0);
}

// Turn the wheels backwards for 1 second (by default)
void achteruit(int runtime = 1000, int speed = 10){
	digitalWrite(MOTOR_PIN1, HIGH);
  analogWrite(MOTOR_PIN2, speed);
  delay(runtime);
	digitalWrite(MOTOR_PIN1, LOW);
  analogWrite(MOTOR_PIN2, 0);
}

void loop() {
  // Every second, turn te motor 10 faster (slowest speed = 130, highest = 0)
  for(int i {0}; i < 130; i+=10){
    vooruit(1000, i);
    achteruit(1000, i);
	}
}
