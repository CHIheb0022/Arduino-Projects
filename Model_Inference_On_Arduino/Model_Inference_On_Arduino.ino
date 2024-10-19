



/**
 * Magic Wand Inference Demo
 * 
 * Attach a button to pin 2. Press button to start collection. Raw data will be
 * collected and fed to the impulse for inference. Inference results are printed
 * to the serial terminal.
 * 
 * Author: Shawn Hymel (EdgeImpulse, Inc.)
 * Date: October 10, 2022
 * License: Apache-2.0 (apache.org/licenses/LICENSE-2.0)
 */

#include <Chiheb_Hmida-project-1_inferencing.h>
#include "MPU9250.h"

// Settings
#define LED_R_PIN           13        // LED pin
#define THRESHOLD           0.6       // Threshold for performing action

// Constants
#define CONVERT_G_TO_MS2    9.80665f  // Used to convert G to m/s^2
#define SAMPLING_FREQ_HZ    100       // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_CHANNELS        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME // 6 channels
#define NUM_READINGS        EI_CLASSIFIER_RAW_SAMPLE_COUNT      // 200 readings
#define NUM_CLASSES         EI_CLASSIFIER_LABEL_COUNT           // 4 classes


MPU9250 mpu; // You can also use MPU9255 as is

void setup() {
  
  // Enable LED pin (RGB LEDs are active low on the Nano 33 BLE)
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
  
  float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
  unsigned long timestamp;
  ei_impulse_result_t result;
  int err;
  float input_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
  signal_t signal;

  // Wait for 2s
  delay(2000)
  
  //If the LED toggls perform the motion.Turn on LED to show we're recording
  digitalWrite(LED_R_PIN, HIGH);

  // Record samples in buffer
  for (int i = 0; i < NUM_READINGS; i++) {

    // Take timestamp so we can hit our target frequency
    timestamp = millis();

    // Get raw readings from the accelerometer and gyroscope
    acc_x = mpu.getAccX();
    acc_y = mpu.getAccY();
    acc_z = mpu.getAccZ();
    
    gyr_x = mpu.getGyroX();
    gyr_y = mpu.getGyroY();
    gyr_z = mpu.getGyroZ();

    // Convert accelerometer units from G to m/s^s
    acc_x *= CONVERT_G_TO_MS2;
    acc_y *= CONVERT_G_TO_MS2;
    acc_z *= CONVERT_G_TO_MS2;

    // Fill input_buf with the standardized readings. Recall tha the order
    // is [acc_x0, acc_y0, acc_z0, gyr_x0, gyr_y0, gyr_z0, acc_x1, ...]
    //input_buf[(NUM_CHANNELS * i) + 0] = acc_x;
    //input_buf[(NUM_CHANNELS * i) + 1] = acc_y;
    //input_buf[(NUM_CHANNELS * i) + 2] = acc_z;
    //input_buf[(NUM_CHANNELS * i) + 3] = gyr_x;
    //input_buf[(NUM_CHANNELS * i) + 4] = gyr_y;
    //input_buf[(NUM_CHANNELS * i) + 5] = gyr_z;

    ////This must be replaced by the funcion computing DSP operation to return our 78 generated features for model input nodes  ///
    //go to file > Example > Chiheb_Hmida-project-1_inferencing.h > accelometer then use the block of code to fill the raw input buffer

    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Turn off LED to show we're done recording
  digitalWrite(LED_R_PIN, LOW);

  // Turn the raw buffer into a signal for inference
  err = numpy::signal_from_buffer(input_buf, 
                                  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, 
                                  &signal);
  if (err != 0) {
    Serial.print("ERROR: Failed to create signal from buffer: ");
    Serial.println(err);
    return;
  }

  // Run the impulse
  err = run_classifier(&signal, &result, false);
  if (err != 0) {
    Serial.print("ERROR: Failed to run classifier: ");
    Serial.println(err);
    return;
  }

  // Print the results
  Serial.println("Predictions");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.print("  ");
    Serial.print(result.classification[i].label);
    Serial.print(": ");
    Serial.println(result.classification[i].value);
  }

  // Make sure the button has been released for a few milliseconds
  while (digitalRead(BTN_PIN) == 0);
  delay(100);
}