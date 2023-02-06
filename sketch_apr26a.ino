#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <MadgwickAHRS.h>
#include <BleKeyboard.h>

BleKeyboard bleKeyboard;
LSM9DS1 imu;
Madgwick filter;

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float increment = 0.0f;
int increment_cnt = 0;
int initial = 1;
float initialvalue;
int threshold = 3400;
int threshold2 = 3400; 
int choice = 0;



void setup() {
  Serial.begin(115200);
  Wire.begin(16,17);
  filter.begin(119);
  imu.begin();

  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);
  bleKeyboard.begin();

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 119;
  microsPrevious = micros();

  
}

void loop() {
  
  int sensorValue2 = analogRead(13);
  int sensorValue = analogRead(27);
  Serial.printf("threshold1 = %d \n", sensorValue);
  Serial.printf("threshold2 = %d \n", sensorValue2);
  digitalWrite(26, LOW);
  digitalWrite(25, LOW);
  digitalWrite(33, LOW);
  digitalWrite(32, LOW);
  if (sensorValue2 > threshold2 and sensorValue < threshold){
    digitalWrite(26, LOW);
      digitalWrite(25, LOW);
      digitalWrite(33, LOW);
      digitalWrite(32, LOW);
      
    if (choice == 1){
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.write(122);
      bleKeyboard.releaseAll();
      Serial.print("key 1 pressed");
      delay(500);
      
    }
    else if (choice == 2){
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.write(99);
      bleKeyboard.releaseAll();
      Serial.print("key 2 pressed");
      delay(500);
    }
    else if (choice == 3){
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.write(118);
      bleKeyboard.releaseAll();
      Serial.print("key 3 pressed");
      delay(500);
    }
    else if (choice == 4){
      bleKeyboard.press(KEY_LEFT_CTRL);
      bleKeyboard.write(115);
      bleKeyboard.releaseAll();
      Serial.print("key 4 pressed");
      delay(500);
    }
    delay(500);
  }
  if (sensorValue > threshold){
    int aix, aiy, aiz;
    int gix, giy, giz;
    int mix, miy, miz;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float roll, pitch, heading;
    unsigned long microsNow;
    microsNow = micros();
    if ((microsNow - microsPrevious) < microsPerReading) {
      return;
    }
    float samplingFreq = 1000000.0f / (microsNow - microsPrevious);
  
    microsPrevious = microsNow;
  
    if ( imu.gyroAvailable() )
    {
      imu.readGyro();
    }
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    if ( imu.magAvailable() )
    {
      imu.readMag();
    }
    
      // read raw data from CurieIMU
      aix= imu.ax;
      aiy = imu.ay;
      aiz = imu.az;
      gix = imu.gx;
      giy = imu.gy;
      giz = imu.gz;
      mix = imu.mx;
      miy = imu.my;
      miz = imu.mz;
      
      ax = imu.calcAccel(aix);
      ay = imu.calcAccel(aiy);
      az = imu.calcAccel(aiz);
      gx = imu.calcGyro(gix);
      gy = imu.calcGyro(giy);
      gz = imu.calcGyro(giz);
      mx = imu.calcMag(mix);
      my = imu.calcMag(miy);
      mz = imu.calcMag(miz);
      // convert from raw data to gravity and degrees/second units
      filter.updateIMU(gx, gy, -gz, ax, ay, -az);
  
      // print the heading, pitch and roll
      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw() - increment;
      increment = increment + 0.0030;
  
    if (initial){
      initialvalue = heading;
    }
      Serial.printf("\n starting heading = %f \n", initialvalue);
    if ((heading - initialvalue) > 15 && (heading - initialvalue) < 30 ){
      digitalWrite(26, HIGH);
      digitalWrite(25, LOW);
      digitalWrite(33, LOW);
      digitalWrite(32, LOW);
      choice = 1;
      Serial.print("1 picked \t"); 
    }
    if ((heading - initialvalue) > 30 && (heading - initialvalue) < 45 ){
      digitalWrite(26, LOW);
      digitalWrite(25, HIGH);
      digitalWrite(33, LOW);
      digitalWrite(32, LOW);
      choice = 2;
      Serial.print("2 picked \t");
    }
    if ((heading - initialvalue) > 45 && (heading - initialvalue) < 70 ){
      digitalWrite(26, LOW);
      digitalWrite(25, LOW);
      digitalWrite(33, LOW);
      digitalWrite(32, HIGH);
      choice = 3;
      Serial.print("3 picked \t");
    }
    if ((heading - initialvalue) > 70 && (heading - initialvalue) < 95){
      digitalWrite(26, LOW);
      digitalWrite(25, LOW);
      digitalWrite(33, HIGH);
      digitalWrite(32, LOW);
      choice = 4;
      Serial.print("4 picked \t");
    }
    initial = 0;
      Serial.print(increment);
      Serial.print('\t');
      Serial.print(roll);
      Serial.print('\t');
      Serial.print(pitch);
      Serial.print('\t');               
      Serial.print(heading);
      Serial.println('\t');
  }
  else {
    digitalWrite(26, LOW);
      digitalWrite(25, LOW);
      digitalWrite(33, LOW);
      digitalWrite(32, LOW);
  }
}
