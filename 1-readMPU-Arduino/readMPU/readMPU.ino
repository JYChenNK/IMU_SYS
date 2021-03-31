#include "I2Cdev.h"
#include "MPU6050.h"
#include "ANO.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 IMU;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() 
{
  // Initialize Serial
  Serial.begin(115200);

  // Initialize I2C & IMU
  // By default, the measurement range of acc is 2g, and the gyro's range is 250 deg/s
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  IMU.initialize();

}

void loop() 
{
  // Get IMU Raw Data
  IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Send Data to PC
  ANO_DT_Send_Senser(ax,ay,az,gx,gy,gz,0,0,0);
  delay(10);
}
