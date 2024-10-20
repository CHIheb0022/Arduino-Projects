#include "MPU9250.h"

// SDA --> A4
// SCL --> A5 

MPU9250 mpu; // You can also use MPU9255 as is

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    mpu.setup(0x68);  // Change to your own address
}

void loop() {
    while (mpu.update()) {
        
        // Acc data
        Serial.print(mpu.getAccX()); Serial.print(", ");
        Serial.print(mpu.getAccY()); Serial.print(", ");
        Serial.print(mpu.getAccZ()); Serial.print(", ");

        //Gyro data 
        Serial.print(mpu.getGyroX()); Serial.print(", ");
        Serial.print(mpu.getGyroY()); Serial.print(", ");
        Serial.println(mpu.getGyroZ());

        // Get Pitch Yaw Roll 
        // Serial.print(mpu.getPitch()); Serial.print(", ");
        // Serial.print(mpu.getYaw()); Serial.print(", ");
        // Serial.print(mpu.getRoll()); Serial.print(", ");
         
    }
}
