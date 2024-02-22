#include "Wire.h"
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Kalman.h"
#include <DHT.h>

#define INTERRUPT_PIN 2

Kalman kalman1, kalman2, kalman3;
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Servo servo1;
Servo servo2;
Servo servo3;
float t;

uint32_t timer;

int capPin = 3;
int resPin = 2;

int buttonPin = 7;
int buttonState = 0;

unsigned long startTime = 0;
int count = 0;

float ypr[3];
Quaternion q; 
VectorFloat gravity; 

int r;
int preVa11;
int preVa12;

DHT dht(13, DHT11);

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialize MPU");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  dht.begin();
  pinMode(capPin, INPUT);
  pinMode(resPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  servo1.attach(5);
  servo2.attach(6);
  //servo3.attach(7);
  //servo1.write(0);
  timer = micros();
}

/*void readTemp(){
  t = dht.readTemperature();
  Serial.println((String)t);
  delay(50);
  t = dht.readTemperature();
  Serial.println((String)t);
}

int slider(){
  int sl;
  sl = analogRead(A0);
  Serial.println(sl, DEC);
  if (sl > 124){
    Serial.println("solid");
  }
  else{
    Serial.println("liquid");
  }
  return sl;
}

void tapTest(){
  unsigned long currentTime = millis();
  if (currentTime - startTime >= 10000){
    count = 0;
    startTime = currentTime;
  }
  count = piezoCapacitive(count);
  Serial.println(count);
}
int piezoCapacitive(int count){
  digitalWrite(resPin, HIGH);
  int d = analogRead(capPin);
  Serial.println((String)d);
  if (d > 0){
    count++;
  }
  else{
  }
  return count;
}*/

int va11;
int va12;
int val3;

void loop()
{
  //buttonState = digitalRead(buttonPin);
  //int sl = slider();
  /*if (buttonState == HIGH){
    startTime = millis();
    tapTest();
  }
  if (sl > 124){
    Serial.println("solid");
    Solid();
  }
  else{
    Serial.println("liquid");
    Liquid();
  }*/
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

  //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //va11 = map(ax, -17000, 17000, -90, 90);
  //readTemp();
  //va12 = map(ay, -17000, 17000, -90, 90);
  //val3 = map(az, -17000, 17000, -90, 90);
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);  
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw = ypr[0];   // Yaw
  float roll = ypr[1];  // Roll
  float pitch = ypr[2]; // Pitch

      // Update Kalman filter with yaw, roll, and pitch values
  float kalmanfi3 = kalman1.getAngle(yaw, 0.0f, dt);    // Assuming 0 angular rate for yaw
  float kalmanfi1 = kalman2.getAngle(roll, gx, dt);    // Using gyroscope data for roll
  float kalmanfi2 = kalman3.getAngle(pitch, gy, dt);
    
  if (kalmanfi1 > 0){
    servo1.write(kalmanfi1);
  }
  else{
    servo1.write(-kalmanfi1);
  }
  if (kalmanfi2 > 0){
    servo2.write(kalmanfi2);
  }
  else{
    servo2.write(-kalmanfi2);
  }
  if (kalmanfi3 > 0){
    Wire.beginTransmission(9);
    Wire.write((uint8_t)kalmanfi3);
    Wire.endTransmission();
    //servo3.write(-kalmanfi3);
  }
  else{
    Wire.beginTransmission(9);
    Wire.write((uint8_t)-kalmanfi3);
    Wire.endTransmission();
    //servo3.write(kalmanfi3);
  }
  }
}
