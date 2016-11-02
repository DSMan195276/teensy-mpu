
#include <stdio.h>
#include <ctype.h>
#include "avr_emulation.h"
#include "WProgram.h"
#include "usb_desc.h"
#include "usb_dev.h"
#include "usb_serial.h"

#define serial_printf(fmt, ...) \
    do { \
        char __buffer[160]; \
        size_t __len; \
        __len = snprintf(__buffer, sizeof(__buffer), fmt, ## __VA_ARGS__); \
        Serial.write(__buffer, __len); \
    } while (0)

#if 0

// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(333);
}
#endif

#if 0
//mpu6050 sketch for eventually doing the old inverted pendulum thing.
//i2c library
//added this comment to look at a change in a git repository.
//dur dur turpa derp.
#include <Wire.h>

#include "mpu.h"
#include "calc.h"

int i = 0, data = 0;

//raw data variables
int16_t ax, ay, az, xangle, yangle;
int16_t gx, gy, gz;

//calculation variables
int angle, adjustment, calib;
float coeffa = .10, coeffb = -.1;   //.08, -.02

mpu6050 mpu;
calc    calculus;

void setup() {
  
  Serial.begin(115200);
  
  mpu.setup_i2c();
  mpu.verify_i2c();
  mpu.initialize_chip();
  //mpu.calibrate_gyro();
  mpu.calibrate_accel();

}

void loop()
{

  delay(10);
  
  mpu.get_gyro_rates();
  mpu.get_accel_values();
  mpu.get_accel_angles();
  gx = mpu.getx();
  gy = mpu.gety();        //I believe this is the axis you want
  gz = mpu.getz();
  ax = mpu.get_accelx();
  ay = mpu.get_accely();
  az = mpu.get_accelz();
  xangle = mpu.get_accel_xangle();
  yangle = mpu.get_accel_yangle();

  angle = xangle;

  adjustment = (int)(coeffa*(float)angle - coeffb*(float)calculus.derivative(angle));

  serial_printf("A: x:%d, y:%d, z:%d G: x:%d, y:%d, z:%d\n",
          ax, ay, az, gx, gy, gz);

#if 0
  if(adjustment > 0)
    motorcontrol.go(1,1,adjustment/2);
  if(adjustment < 0)
    motorcontrol.go(0,1,abs(adjustment)/2);
  
  i = ~i;
  if(i){
    digitalWrite(ledPin, LOW);
  }else{
    digitalWrite(ledPin, HIGH);
  }
#endif

}
#endif
