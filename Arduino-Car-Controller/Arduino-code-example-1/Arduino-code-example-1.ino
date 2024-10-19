#include <Servo.h>
/**
 *         Elextro-Tex
 * Arduino Car - Codigo de ejemplo
 *      Control Serial
 */
 
// Declaramos los pines a usar
int PIN_SERVO_A = 2;
int PIN_SERVO_B = 3;

//Pin para llanta de la izquierda
int PIN_IN1 = 7;
int PIN_IN2 = 6;

//Pin para llanta de la derecha
int PIN_IN3 = 8;
int PIN_IN4 = 9;

//Pin para el led/foco
int PIN_LED = 5;

//Pin HC-SR04
int PIN_ECHO = 10;
int PIN_TRIG = 11;

//Declaramos los servos
Servo servoA;
Servo servoB;

void setup() {
  
  // inicializar la comunicación serial a 9600 bits por segundo
  Serial.begin(9600);
  Serial.setTimeout(5);
  
  // Configuramos los pines como salida
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  
  // Configuramos los servos
  servoA.attach(PIN_SERVO_A);
  servoB.attach(PIN_SERVO_B);

  // Configuramos el led
  pinMode(PIN_LED, OUTPUT);

  // Configuramos sensor de distancia
  pinMode(PIN_TRIG, OUTPUT); // Sets the trigPin as an Output
  pinMode(PIN_ECHO, INPUT);
  
}

void loop() {
  
  char codigo;
  String valor = "";
  
  while (Serial.available() > 0) {    // Preguntamos si existe datos leidos por serial
    String linea = Serial.readString();
    for(int i = 0 ;i < linea.length(); i++){
      int caracter = linea[i];
      if (isDigit(caracter)) {        // Preguntamos si es digito
         valor += (char)caracter;     // Guardamos el valor leido
      }else if (caracter != '\n') {   // Preguntamos si es distinto a salto de linea
          codigo = caracter;          // Guardamos el codigo leido
      }else{
        break;
      }  
    }
  }

  switch(codigo){
    case 'A': avanzar();   break;
    case 'I': izquierda(); break;
    case 'R': retroceder();break;
    case 'D': derecha();   break;
    case 'P': parar();     break;
    case 'T': encenderLed();break;
    case 't': apagarLed(); break;
    case 'M':
      if(valor!=""){
        girarServoA(valor.toInt());
      }
      break;
    case 'N':
      if(valor!=""){
        girarServoB(valor.toInt());
      }
      break;
  }

  Serial.print("D");
  Serial.println(medirDistancia());
  
  delay(10);
}

long medirDistancia(){
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);  
 
  long duration = pulseIn(PIN_ECHO, HIGH);
 
  return microsecondsToCentimeters(duration);
}

long microsecondsToCentimeters (long microseconds) {
  return microseconds / 29 / 2;
}

void encenderLed(){
  digitalWrite(PIN_LED, HIGH);  
}
void apagarLed(){
  digitalWrite(PIN_LED, LOW);  
}
void girarServoA(int grados){
  servoA.write(grados);
}

void girarServoB(int grados){
  servoB.write(grados);
}
void avanzar(){
  //Avanza llantas de la izquierda
  digitalWrite (PIN_IN1, HIGH);
  digitalWrite (PIN_IN2, LOW);
  //Avanza llantas de la derecha
  digitalWrite (PIN_IN3, HIGH);
  digitalWrite (PIN_IN4, LOW);
}
void retroceder(){
  //Retrocede llantas de la izquierda
  digitalWrite (PIN_IN1, LOW);
  digitalWrite (PIN_IN2, HIGH);
  //Retrocede llantas de la derecha
  digitalWrite (PIN_IN3, LOW);
  digitalWrite (PIN_IN4, HIGH);
}

void parar(){
  //Para llanas de la izquierda
  digitalWrite (PIN_IN1, LOW);
  digitalWrite (PIN_IN2, LOW);
  //Para llanas de la derecha
  digitalWrite (PIN_IN3, LOW);
  digitalWrite (PIN_IN4, LOW);
}

void izquierda(){
  //Rertocede llantas de la izquierda
  digitalWrite (PIN_IN1, LOW);
  digitalWrite (PIN_IN2, HIGH);
  //Avanza llantas de la derecha
  digitalWrite (PIN_IN3, HIGH);
  digitalWrite (PIN_IN4, LOW);
}

void derecha(){
  //Retrocede llantas de la derecha
  digitalWrite (PIN_IN3, LOW);
  digitalWrite (PIN_IN4, HIGH);
  //Avanza llantas de la izquierda
  digitalWrite (PIN_IN1, HIGH);
  digitalWrite (PIN_IN2, LOW);
}
