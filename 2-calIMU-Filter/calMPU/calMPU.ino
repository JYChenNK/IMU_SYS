#include "I2Cdev.h"
#include "MsTimer2.h"
#include "MPU6050.h"
#include  "Filter.h"
#include "ANO.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 IMU;

unsigned long TimeCost;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accel_z, gyro, accel_angle, gyro_angle;
extern float Angle_Kalman, Bias_Kalman, Angle_FOCF;


void calIMU()
{
  sei();
  static unsigned long LastTime;
  LastTime = micros();
  
  // Get IMU Raw Data
  IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
  // Convert Data to right unit
  gyro = gx*250.0/32767,accel_z= az*2.0*9.8/32767;
  // Calculate angle directly by two methods
  accel_angle = atan2(ay,az)*180.0/PI, gyro_angle += (gyro+1)*dt;
  // One-Order Complementary Filter
  Fist_Older_Complementary_Filter(gyro,accel_angle);
  // Kalman Filter
  Kalman_Filter(gyro,accel_angle);
  // Send Data to PC
  ANO_DT_Send_Senser(gyro*100, accel_z*100, accel_angle*100, gyro_angle*100, Angle_FOCF*100, Angle_Kalman*100, Bias_Kalman*100,0,TimeCost);
  
  TimeCost = micros() - LastTime;
}

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

  MsTimer2::set(5,calIMU);
  MsTimer2::start();
  
}

void loop() 
{
  
}
