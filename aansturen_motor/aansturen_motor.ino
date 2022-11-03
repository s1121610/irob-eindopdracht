const int MOTOR_PIN1 {3};
const int MOTOR_PIN2 {5};
	
void setup() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
	Serial.begin(9600);
}

void vooruit(int runtime = 1000, int speed = 10){
	digitalWrite(MOTOR_PIN2, HIGH);
  analogWrite(MOTOR_PIN1, speed);
  delay(runtime);
	digitalWrite(MOTOR_PIN2, LOW);
}

void achteruit(int runtime = 1000, int speed = 10){
	digitalWrite(MOTOR_PIN1, HIGH);
  analogWrite(MOTOR_PIN2, speed);
  delay(runtime);
	digitalWrite(MOTOR_PIN1, LOW);
}

void loop() {
  for(int i {0}; i < 130; i+=1){
    vooruit(1000, i);
    achteruit(1000, i);
	  //analogWrite(motor_pin1, i);
	}
}
