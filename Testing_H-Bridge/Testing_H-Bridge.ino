// Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01


#define enA 11
#define in1 7
#define in2 8



void setup() {

  // Start serial
  Serial.begin(115200);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  
  int potValue = analogRead(A0); // Read potentiometer value
  int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
  Serial.print(potValue);
  Serial.print(" , ");
  Serial.println(pwmOutput);


  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  delay(100);
}