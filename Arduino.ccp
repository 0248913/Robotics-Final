const int int1 = 6;  
const int int2 = 5;  
const int int3 = 9;  
const int int4 = 10; 

const int enA = 4;
const int enB = 8;

const int motorSpeed = 90;

void setup() {
  
  pinMode(int1, OUTPUT);
  pinMode(int2, OUTPUT);
  pinMode(int3, OUTPUT);
  pinMode(int4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

 
  Serial.begin(9600);

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  
}

void loop() {
  if (Serial.available() > 0) {
    
    String direction = Serial.readStringUntil('\n');


    Serial.print("Direction: ");
    Serial.println(direction);

   
    if (direction == "Forward") {
      moveForward();
    } else if (direction == "Left Turn") {
      turnLeft();
    } else if (direction == "Right Turn") {
      turnRight();
    } else if (direction == "NO lines") {
      stopMotors();
    }
  }
}

void moveForward() {
  digitalWrite(int1, HIGH);
  digitalWrite(int2, LOW);
  digitalWrite(int3, HIGH);
  digitalWrite(int4, LOW);

  analogWrite(enA, motorSpeed);
  
}

void turnLeft() {
  digitalWrite(int1, LOW);
  digitalWrite(int2, HIGH);
  digitalWrite(int3, HIGH);
  digitalWrite(int4, LOW);

  analogWrite(enA, motorSpeed);
}

void turnRight() {
  digitalWrite(int1, HIGH);
  digitalWrite(int2, LOW);
  digitalWrite(int3, LOW);
  digitalWrite(int4, HIGH);

  analogWrite(enA, motorSpeed);
}

void stopMotors() {
  digitalWrite(int1, LOW);
  digitalWrite(int2, LOW);
  digitalWrite(int3, LOW);
  digitalWrite(int4, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
