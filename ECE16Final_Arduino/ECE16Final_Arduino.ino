/* 
 *  General Includes
 */
#include <AltSoftSerial.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* 
 *  Global Variables
 */

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

AltSoftSerial BTserial;

const int MPU_addr=0x68;  // I2C address of the MPU-6050, can be changed to 0x69
MPU6050 IMU(MPU_addr);
const int interruptPin = 2;
volatile bool ipinReady = false;
int16_t gx, gy, gz;

unsigned long samplePeriod = 40000; // 25 Hz for the BLE limitations
unsigned long startTime = 0;
unsigned long volatile elapsedTime = 0;
unsigned long volatile currentTime = 0;
unsigned long volatile lastTime = 0;
const int RED_PIN = 10;
const int GREEN_PIN = 11;
const int buttonPin = 4;
int num = 0;
int temp = 0;
bool newRead = false;
bool sending = true;
int buttonState;
int prevState = 0;
bool sleep = false;

/* 
 *  Function to check the interrupt pin to see if there is data available in the MPU's buffer
 */
void interruptPinISR() {
  ipinReady = true;
}

/* 
 *  Function to read a single sample of IMU data
 *  NOTE: ONLY READ WHAT YOU INTEND TO USE IN YOUR ALGORITHM!
 */
void readIMU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                    // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  
  //Gyroscope (3 Axis)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

/* 
 *  Function to poll to see if data is available to read
 *  'samplePeriod' defines how frequently we poll... default set to 25Hz
 */
void pollData() {

  if (ipinReady && !newRead) {
  
    currentTime = micros();
    if (currentTime - lastTime >= samplePeriod) {
      elapsedTime = (currentTime - startTime)/1e3;
      lastTime = currentTime;

      readIMU();
      newRead = true;
    }
  }  
}

/* 
 *  Function to send data to the Python processing
 *  NOTE: MODIFY ACCORDING TO YOUR ALGORITHM!
 */
void sendData() {
  // Displays the raw value and the time it was sampled
  BTserial.print(elapsedTime);
  BTserial.print(' ');
  BTserial.print(gx);
  BTserial.print(' ');
  BTserial.print(gy);
  BTserial.print(' ');
  BTserial.print(gz);
  BTserial.print(' ');

  if(num == 3){
    temp = analogRead(A2);
    num = 0;
  } else {
    num++; 
  }
  //BTserial.print(" GG ");
  BTserial.println(temp);
  //Serial.println(analogRead(A2));
  /*
  Serial.print(elapsedTime);
  Serial.print(' ');
  Serial.print(gx);
  Serial.print(' ');
  Serial.print(gy);
  Serial.print(' ');
  Serial.print(gz);
  Serial.print(' ');  
  if(num == 3){
    Serial.println(analogRead(A2));
    num = 0;
  } else {
    Serial.println();
    num++; 
  }*/
}

/* 
 *  Function to do the usual Arduino setup
 */
void setup(){

  // Intialize the IMU and the DMP ont he IMU
  IMU.initialize();
  IMU.dmpInitialize();
  IMU.setDMPEnabled(true);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(MPU_addr);   // PWR_MGMT_1 register
  Wire.write(0);          // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();
  
  Serial.begin(9600);
  BTserial.begin(9600); 

  // Create an interrupt for pin2, which is connected to the INT pin of the MPU6050
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptPinISR, RISING);

  // Start time of the Arduino (for elapsed time)
  startTime = micros();

  //set up led pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  //turn on red as default
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);

  //button stuff
  pinMode(buttonPin, INPUT);  
  buttonState = 0;
  sending = true;
}

/* 
 *  Function to loop continuously: poll ==> send ==> read
 *  NOTE: MODIFY TO SUIT YOUR ALGORITHM!
 */
void loop(){
  
  //get button state
  buttonState = digitalRead(buttonPin);

  if(buttonState == 0 && prevState == 1){   //this means button has been pressed
    //Serial.println("changed");
    if(sleep){
      sleep = false;
      sending = true;
    } else {
      sleep = true;
      sending = false;
    }
  }
  

  
  // no longer using an ISR for the sake of AltSoftSerial
  pollData();
  //sending = true;

  if (newRead && sending) {
    //Serial.println("sending");
    sendData();
    newRead = false;
  }
  if (BTserial.available() > 0) { 
    String dataFromPython =  BTserial.readStringUntil(',');

    if(dataFromPython[dataFromPython.length() - 1] == '!'){
      //turn LED green
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);
    } else {
      //turn LED red
      digitalWrite(RED_PIN,HIGH);
      digitalWrite(GREEN_PIN, LOW);
    }
    /*
    Serial.print("Received: ");
    Serial.println(dataFromPython);
    */
    display.clearDisplay();
    display.display();   
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println(dataFromPython);
    display.display();
  }

}
