#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"
#include <DHT.h>


MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Servo servo1;
Servo servo2;
float t;

int capPin = 3;
int resPin = 2;

int buttonPin = 4;
int buttonState = 0;

unsigned long startTime = 0;
int count = 0;

int va11;
int va12;

int r;
int preVa11;
int preVa12;

DHT dht(13, DHT11);

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  dht.begin();
  pinMode(capPin, INPUT);
  pinMode(resPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  servo1.attach(5);
  servo2.attach(6);
  servo1.write(50);

}

void readTemp(){
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
}
void loop()
{
  buttonState = digitalRead(buttonPin);
  int sl = slider();
  if (buttonState == HIGH){
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
  }
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  va11 = map(ax, -17000, 17000, 0, 179);
  readTemp();
  if (va11 != preVa11)
  {
    servo1.write(va11);
    preVa11 = va11;
  }
  va12 = map(ay, -17000, 17000, 0, 179);
  if (va12 != preVa12);
  { 
    servo2.write(va12);
    preVa12 = va12;
  }
  Serial.print(va11);
  Serial.print(",");
  Serial.println(va12);
  delay(100);
}
