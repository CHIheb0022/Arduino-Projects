//Don't forget to set HC05 enable pin high before supplying power to the module 
/* Connect the KEY pin high before applying power to the module. This will set the module into command mode at 38400 baud.
This is the default baud rate for the command mode and needed if you don’t know the baud rate the module is set to. You can use a serial monitor
to get the job done.
*/

//Then while configuration this en pin should be set high (wired to --> 5v or 3v)

#include <SoftwareSerial.h>
#define rxPin 2
#define txPin 3
#define baudrate 38400

String msg;

SoftwareSerial hc05(rxPin ,txPin);

void setup(){
 	pinMode(rxPin,INPUT);
 	pinMode(txPin,OUTPUT);
 	
 	Serial.begin(9600);
 	Serial.println("ENTER AT Commands:");
 	hc05.begin(baudrate);
}

void loop(){
 			readSerialPort();
 			if(msg!="") hc05.println(msg);
 			
 			if (hc05.available()>0){
 					Serial.write(hc05.read());
 			}
}

void readSerialPort(){
 	msg="";
 while (Serial.available()) {
 		delay(10); 	
 		if (Serial.available() >0) {
 				char c = Serial.read(); 	//gets one byte from serial buffer
 				msg += c; //makes the string readString
 		}
 }
}

