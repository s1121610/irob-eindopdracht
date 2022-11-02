int motor_pin1 {2}; //This pin 
int motor_pin2 {4};
	
void setup() {
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
	Serial.begin(9600);
}
	


void vooruit(int runtime = 1000, int speed = 10){
	digitalWrite(motor_pin2, HIGH);
  analogWrite(motor_pin1, speed);
  delay(runtime);
	digitalWrite(motor_pin2, LOW);
}

void achteruit(int runtime = 1000, int speed = 10){
	digitalWrite(motor_pin1, HIGH);
  analogWrite(motor_pin2, speed);
  delay(runtime);
	digitalWrite(motor_pin1, LOW);
}

void loop() {
  for(int i {0}; i < 130; i+=1){
    vooruit(1000, i);
    achteruit(1000, i);
	  //analogWrite(motor_pin1, i);
	}
}
