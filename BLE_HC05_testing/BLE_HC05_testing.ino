#include <SoftwareSerial.h>

#define Rx 2
#define Tx 3
#define led 13

SoftwareSerial HC05(Rx,Tx);
String msg;

void setup(){
  pinMode(led, OUTPUT);
 	Serial.begin(9600);
 	HC05.begin(9600); 	
  delay(20);
  digitalWrite(led,LOW);		
}

void loop(){
 	readSerialPort();
 	
 	// Send answer to master
 	if(msg!=""){
 			Serial.print("Recieved msg: " );
 			Serial.println(msg);
 			HC05.print(msg);
 			msg=""; 
 	}
  if (msg == 'A'){
    digitalWrite(led,HIGH);
    delay(1000);
    digitalWrite(led,LOW);
  }
}

void readSerialPort(){
 while (HC05.available()) {
 		delay(10); 
 		if (HC05.available() >0) {
 				char c = HC05.read(); 	//gets one byte from serial buffer
 				msg += c; //makes the string readString
 		}
 }
 HC05.flush();
}

