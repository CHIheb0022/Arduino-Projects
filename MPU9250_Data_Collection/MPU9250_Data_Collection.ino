/**
 * Magic Wand Data Collection (continuous)
 * 
 * Raw x, y, z acceleration data (in G) and x, y, z, rotational (gyroscope) 
 * data (in degrees/sec) are printed over the serial port. Note that Edge 
 * Impulse requires a timestamp (in milliseconds) column. There is a 1 second
 * break between each collection. Perform your gesture whenever you see the 
 * red LED light up.
 * 
 * Made for the Arduino Nano 33 BLE Sense, which has a built-in IMU (LSM9DS1).
 * 
 * For use with the serial-data-collect-csv.py script to save CSV files.
 * 
 * Author: Shawn Hymel (EdgeImpulse, Inc.)
 * Date: July 28, 2022
 * License: Apache-2.0 (apache.org/licenses/LICENSE-2.0)
 */

#include "MPU9250.h"

// Settings
#define LED_R_PIN           13        // Red LED pin
#define DELAY_COLLECTION    2000      // Delay (ms) between collections

// Constants
//#define CONVERT_G_TO_MS2    9.80665f  // Used to convert G to m/s^2 // This constant is used when interfacing with LSM6N
#define SAMPLING_FREQ_HZ    100       // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  100 / SAMPLING_FREQ_HZ   // Sampling period (ms) (in real world is ~= 17ms due to mpu delay). we could just define it to 1ms instead 
#define NUM_SAMPLES         200       // 200 samples at 100 Hz is 2 sec window

MPU9250 mpu; // You can also use MPU9255 as is

void setup() {

  // Enable LED pin (RGB LEDs are active low)
  pinMode(LED_R_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, LOW);

  // Start serial
  Serial.begin(115200);
  
  Wire.begin();
  delay(2000);

  mpu.setup(0x68);  // Change to your own address

  // Start accelerometer (part of IMU)
  if (!mpu.update()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  unsigned long timestamp;
  unsigned long start_timestamp;

  // Turn on LED to show we're recording (RGB LED is active low)
  pinMode(LED_R_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);

  
  //Check for updates from the mpu
  

  // Print header
  Serial.println("timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ");

  // Record samples in buffer
  start_timestamp = millis(); //This variable is used to compute timestamp parameter for our CSV file. 
    
  int i = 0;
  
  //while NUM_SAMPLES is no reach add a another sample. If exceded (=100) move out of this while loop to gather next example.. in another worlds new CSV file. This is ensured through the loop(). We stop when we pulg out our arduino board or when we press ctr+c in the when running python script.
  //This next example correspond to the same motion gesture it's like another image for a cat.. cat here represent the category or class).  
  while ((mpu.update()) && (i < NUM_SAMPLES)){ // to update MPU data we should call mpu.update()
  //mpu.update() is causing an approximate 7ms additional delay. Hence we are not going to receive lines with timestamp 
  //ranging from 0 to 2000 (with perfect 10ms padding)

    
    // Take timestamp so we can hit our target frequency
    timestamp = millis();
    
    // Read and convert accelerometer data to m/s^2
    acc_x = mpu.getAccX();
    acc_y = mpu.getAccY();
    acc_z = mpu.getAccZ();
    
    // Read gyroscope data (in degrees/sec)
    gyr_x = mpu.getGyroX();
    gyr_y = mpu.getGyroY();
    gyr_z = mpu.getGyroZ();
    
    //Print CSV data with timestamp
    Serial.print(timestamp - start_timestamp);
    Serial.print(",");
    Serial.print(acc_x);
    Serial.print(",");
    Serial.print(acc_y);
    Serial.print(",");
    Serial.print(acc_z);
    Serial.print(",");
    Serial.print(gyr_x);
    Serial.print(",");
    Serial.print(gyr_y);
    Serial.print(",");
    Serial.println(gyr_z);
    
    //Move to next sammple 
    i+=1;
    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }
  

  // Print empty line to transmit termination of recording
  Serial.println();

  // Turn off LED to show we're done
  digitalWrite(LED_R_PIN, LOW);

  // Wait before repeating the collection process
  delay(DELAY_COLLECTION);

}