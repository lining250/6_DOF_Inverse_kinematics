#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int data[7];
int i;
int counter = 0;

int readStuff(){
  byte MSB = 0;  // to build  2 byte integer from serial in byte
  byte LSB = 0;  // to build  2 byte integer from serial in byte
  short   MSBLSB = 0;  //to build  2 byte integer from serial in byt
  MSB = Serial.read();
  LSB = Serial.read();
  MSBLSB=word(MSB, LSB);  
  return MSBLSB;
}


void setup(){
  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.begin(115200); 
}

void loop(){
  if (Serial.available() >= 2){  // wait for 2 bytes.
    data[counter]=readStuff(); 
    counter++;
  }
  if(counter == 7){
    counter = 0;
    for(i=0; i<7; i++){
      //Serial.println(data[i]);
      pwm.setPWM(i, 0, data[i]);
    }
  }
}


