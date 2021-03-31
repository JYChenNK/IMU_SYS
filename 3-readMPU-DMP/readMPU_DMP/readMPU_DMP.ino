#include "MsTimer2.h"
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "ANO.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 IMU;

//#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  // set true if DMP init was successful
bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t acc[3];
int16_t gyro[3];
float euler[3];
Quaternion q;


int dataCounter = 0;
int dataFreq = 0;


void sendPC();
void IMU_Init();
void dmpDataReady();

void setup() 
{
  // Initialize Serial
  Serial.begin(115200);
  
  IMU_Init();
  
  MsTimer2::set(5,sendPC);
  MsTimer2::start();
}

void loop() 
{
  // Get IMU Raw Data
  //  IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Get IMU DMP Data
  IMU_DMP_read();
  
}

void sendPC()
{
  static int timeCounter = 0;
  
  timeCounter += 5;
  // Send Data to PC
  ANO_DT_Send_Senser(euler[1]*180/M_PI,euler[2]*180/M_PI, euler[0]*180/M_PI, dataFreq,0,0,0,0,0);
  
  if(timeCounter >= 1000)
  {
    dataFreq = dataCounter;
    dataCounter = 0;
    timeCounter = 0;
  }
}

void IMU_Init()
{
  // Initialize I2C & IMU
  // By default, the measurement range of acc is 2g, and the gyro's range is 250 deg/s
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  IMU.initialize();
  delay(200);
  
  // Initialize DMP
  devStatus = IMU.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  //  IMU.setXGyroOffset(220);
  //  IMU.setYGyroOffset(76);
  //  IMU.setZGyroOffset(-85);
  //  IMU.setZAccelOffset(1788); 

  if (devStatus == 0) 
  {
    // turn on the DMP
    IMU.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = IMU.getIntStatus();
    // set our DMP Ready flag
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = IMU.dmpGetFIFOPacketSize();
  }
  else
    Serial.println("IMU Init Failed!");
}

// INTERRUPT DETECTION ROUTINE
void dmpDataReady() 
{
    mpuInterrupt = true;
}

void IMU_DMP_read()
{
  // return if IMU is not initialized
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  if(!mpuInterrupt) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = IMU.getIntStatus();

  // get current FIFO count
  fifoCount = IMU.getFIFOCount();

  // check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
    // reset so we can continue cleanly
    IMU.resetFIFO();
    Serial.println("FIFO overflow!");
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) 
      fifoCount = IMU.getFIFOCount();

    // read a packet from FIFO
    IMU.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    IMU.dmpGetQuaternion(&q, fifoBuffer);
    IMU.dmpGetEuler(euler, &q);
    
    dataCounter++;

  }  
}
