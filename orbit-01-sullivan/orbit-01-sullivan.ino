#include <Average.h>
#include "Arduino.h"
#include <digitalWriteFast.h>
#include <TimerOne.h>
#include <Wire.h>
#include "RTClib.h"

// Settings
const bool bool_motorOn = true;
const bool bool_lightsOn = true;
const bool bool_deviationLightsOff = false;
const bool bool_speedIncrease = false
;

const bool bool_zeroMotor = true;
const bool bool_slowStart = true;
const bool bool_serialPrinter_on = false;
const bool bool_manualSpeed = false;
const bool bool_RTC_schedule_on = false;
byte motor1_fixedSpeedStart = 161; // if motor1=163, 2=164, they have exact same speed (won't fall in/out of sync)
byte motor1_fixedSpeedEnd = 163;
byte motor2_fixedSpeedStart = 161; // 162-165 for opening: almost smashed after 4.5 hours
byte motor2_fixedSpeedEnd = 163;
unsigned long motor1SpeedIncreaseDuration = 3600000; // 167/167 perfect after 4 hours, but smashed after 6 hours!
unsigned long motor2SpeedIncreaseDuration = 3600000; // 
byte minDim = 65; // not 70
byte maxDim = 110; // 110 max 128
const byte     photoGate_arraySize = 5;
byte           deviationAllowance = 500;

// 163 start = after 0.5h very close. after 1h15m getting a bit too weak.

// RTC setup
RTC_DS1307 RTC;
const byte AwakeTimeHour = 10;  // 0,1,2..23)
const byte AsleepTimeHour = 21;

// Time
unsigned long currentTime = 0;

// Dimmer setup
byte pin_dim1 = 8;               // Output to Opto Triac
byte pin_dim2 = 9;
byte interrupt_dim = 0;  // interrupt 0 = pin 2
volatile int dim_counter = 0;                           
volatile boolean zero_cross = 0;
int freqStep = 65; //65=128 steps, 32=256
byte dimStep = 128;
byte dim1 = dimStep;                 // Dimming level (0-128)  0 = on, 128 = 0ff
byte dim2 = dimStep;

// motor speed 1=164 / 2=162 // 62in circles

// Motor setup
byte pin_motorSTBY = 34;
// Motor 1 connected to A01 and A02
byte pin_motorPWMA = 44; //Speed control 
byte pin_motorAIN1 = 40; //Direction
byte pin_motorAIN2 = 42; //Direction
// Motor 2 connected to B01 and B02
byte pin_motorPWMB = 46; //Speed control
byte pin_motorBIN1 = 36; //Direction
byte pin_motorBIN2 = 38; //Direction

byte motor1 = 1;
byte motor2 = 2;
byte motorDirection1 = 0; // 1=CCW, 0=CW
byte motorDirection2 = 0; // 1=CCW, 0=CW
int motor1degrees = 0;
int motor2degrees = 0;

// Pot Setup
byte pin_motorSpeedPot1 = A8;
byte pin_motorSpeedPot2 = A9;    
byte val_motorSpeedPot1 = 0;  
byte val_motorSpeedPot2 = 0;

// photoGates Setup
byte interrupt_pin_photoGate1 = 5; // PIN 19
byte interrupt_pin_photoGate2 = 4; // PIN 18




//interrupt variables
volatile unsigned long  photoGate1_MS_now = 0;
volatile bool           bool_photoGate1 = false; // true=1,open; false=0,closed

unsigned long  photoGate1_MS_last = 0;
int            photoGate1_time_now = 0;
Average<int>   photoGate1_ave(photoGate_arraySize);
int            photoGate1_time_average = 0;
int            photoGate1_deviation = 0;

//interrupt variables
volatile unsigned long  photoGate2_MS_now = 0;
volatile bool           bool_photoGate2 = false; // true=1,open; false=0,closed

unsigned long  photoGate2_MS_last = 0;
int            photoGate2_time_now = 0;
Average<int>   photoGate2_ave(photoGate_arraySize);
int            photoGate2_time_average = 0;
int            photoGate2_deviation = 0;


int encoder1degrees = 0;
int encoder2degrees = 0;

void setup()
{
  Serial.begin(9600);

  // RTC
  Wire.begin();
  RTC.begin();
  
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
    
    if (now.unixtime() < compiled.unixtime()) {                   
      Serial.println("RTC is older than compile time! Updating");  
      // following line sets the RTC to the date & time this sketch was compiled
      RTC.adjust(DateTime(__DATE__, __TIME__));
    }
  
  // Dimmers
  pinMode(pin_dim1, OUTPUT);                                   // Set the Triac pin as output
  pinMode(pin_dim2, OUTPUT);  
  attachInterrupt(interrupt_dim, zero_cross_detect, RISING);   // Attach an Interupt to Pin 20 (interupt 3) for Zero Cross Detection
  Timer1.initialize(freqStep);                                 // Initialize TimerOne library for the freq we need 
  Timer1.attachInterrupt(dim_check, freqStep); 

  // Motors
  pinMode(pin_motorSTBY, OUTPUT);
  pinMode(pin_motorPWMA, OUTPUT);
  pinMode(pin_motorAIN1, OUTPUT);
  pinMode(pin_motorAIN2, OUTPUT);
  pinMode(pin_motorPWMB, OUTPUT);
  pinMode(pin_motorBIN1, OUTPUT);
  pinMode(pin_motorBIN2, OUTPUT);  
  digitalWrite(pin_motorSTBY, LOW); // Set motors to Standby
  
  // MotorPots
  pinMode(pin_motorSpeedPot1, INPUT);
  pinMode(pin_motorSpeedPot2, INPUT);

  // photoGates
  attachInterrupt(interrupt_pin_photoGate1, photoGate1Detect, RISING);
  attachInterrupt(interrupt_pin_photoGate2, photoGate2Detect, RISING);

  // Bring motor to 0
  if(!bool_RTC_schedule_on) {
      if(bool_motorOn) {
        if(bool_zeroMotor) {
          motorToZero();
          delay(5000);
        }
        if(bool_slowStart) {
          Serial.println("slow start");
          motorSlowStart();
        }
      }
  }  
}

void loop() { 
   
  if(bool_RTC_schedule_on) {
    byte our = 0;
    DateTime now = RTC.now();
    our = now.hour();
    
    if ((our >= AwakeTimeHour) && (our < AsleepTimeHour)) {        
      if(bool_motorOn) {
          if(bool_zeroMotor) {
            motorToZero();
            delay(3000);
          }
          if(bool_slowStart) {
            Serial.println("slow start");
            motorSlowStart();
          }
      } 
        while((our >= AwakeTimeHour) && (our < AsleepTimeHour)) {          
          update();
        }       
    } else {
      printTime(now);
      Serial.print("Asleep until ");
      Serial.print(AwakeTimeHour, DEC);
      Serial.print(" o'clock");
      Serial.println();
      delay(1000);
    }
  } else {
    update();
  } 
}

void update() {
  
  currentTime = millis();
  
  // RTC
  byte our = 0;
  DateTime now = RTC.now();
  our = now.hour();
          
  // MotorSpeed
  if(bool_motorOn) {
    
    if(bool_manualSpeed) {
          val_motorSpeedPot1 = map(analogRead(pin_motorSpeedPot1), 0, 1023, 120, 200); 
          val_motorSpeedPot2 = map(analogRead(pin_motorSpeedPot2), 0, 1023, 120, 200);
          
    } else if(bool_speedIncrease) {      
          if(currentTime < motor1SpeedIncreaseDuration) {
            val_motorSpeedPot1 = map(currentTime, 0, motor1SpeedIncreaseDuration, motor1_fixedSpeedStart, motor1_fixedSpeedEnd);
          } else {
            val_motorSpeedPot1 = motor1_fixedSpeedEnd;
          }
          if(currentTime < motor2SpeedIncreaseDuration) {
            val_motorSpeedPot2 = map(currentTime, 0, motor2SpeedIncreaseDuration, motor2_fixedSpeedStart, motor2_fixedSpeedEnd);
          } else {
            val_motorSpeedPot2 = motor2_fixedSpeedEnd;
          }
          
    } else {
      val_motorSpeedPot1 = motor1_fixedSpeedStart;
      val_motorSpeedPot2 = motor2_fixedSpeedStart;
    }
    
  } else {
          val_motorSpeedPot1 = 0;
          val_motorSpeedPot1 = 0;
  }
  
  // Motor control
  motorMove(motor1, val_motorSpeedPot1, motorDirection1); //motor 1, speed 0-255, left
  motorMove(motor2, val_motorSpeedPot2, motorDirection2); //motor 2, speed 0-255, left
  // motorStop(); //stop

  motor1degrees = map((currentTime - photoGate1_MS_now), 0, photoGate1_time_average, 0, 360);
  motor2degrees = map((currentTime - photoGate2_MS_now), 0, photoGate2_time_average, 0, 360);
 
  // Handle photoGate
  if(bool_photoGate1) {
    photoGate1_time_now = photoGate1_MS_now - photoGate1_MS_last;
    photoGate1_time_average = photoGate1_ave.rolling(photoGate1_time_now);
    photoGate1_deviation = photoGate1_ave.stddev();
    photoGate1_MS_last = photoGate1_MS_now;
    bool_photoGate1 = false;
    //Serial.println("photoGate1");
  }

  if(bool_photoGate2) {
    photoGate2_time_now = photoGate2_MS_now - photoGate2_MS_last;
    photoGate2_time_average = photoGate2_ave.rolling(photoGate2_time_now);
    photoGate2_deviation = photoGate2_ave.stddev();
    photoGate2_MS_last = photoGate2_MS_now;
    bool_photoGate2 = false;
    //Serial.print("\t");
    //Serial.println("photoGate2");
  }

  if(bool_lightsOn) {
      if(bool_deviationLightsOff) {
          if(photoGate1_deviation < deviationAllowance) {
            dim1 = map(dist(motor1degrees, motor2degrees), 0, 1000, minDim, maxDim);
          } else {
            dim1 = dimStep;
          }
          if(photoGate2_deviation < deviationAllowance) {
            dim2 = map(dist(motor1degrees, motor2degrees), 0, 1000, minDim, maxDim);
          } else {
            dim2 = dimStep;
          }
      }
      else {
        dim1 = map(dist(motor1degrees, motor2degrees), 0, 1000, minDim, maxDim);
        dim2 = map(dist(motor1degrees, motor2degrees), 0, 1000, minDim, maxDim);
      }
  } else {
    digitalWrite(pin_dim1, LOW);
    digitalWrite(pin_dim2, LOW);
  }
  
  if(bool_serialPrinter_on) {
    serialPrinter();
  }
  
}

// RTC
// ~~~ GetTime retrieves the current date/time from RTC module
void GetTime ()  {
  byte our = 0;
  DateTime now = RTC.now();
  printTime(now);
  our = now.hour();
  Serial.println(our, DEC);
}  
void printTime(DateTime now){
   Serial.print(now.year(), DEC);
   Serial.print('/');
   Serial.print(now.month(), DEC);
   Serial.print('/');
   Serial.print(now.day(), DEC);
   Serial.print(' ');
   Serial.print(now.hour(), DEC);
   Serial.print(':');
   Serial.print(now.minute(), DEC);
   Serial.print(':');
   Serial.print(now.second(), DEC);
   Serial.println();
}  

// photoGate Interrupt
void photoGate1Detect() {
  //Serial.println("gate1");
    bool_photoGate1 = true; // true=1,open; false=0,closed
    photoGate1_MS_now = currentTime;
}

void photoGate2Detect() {
  //Serial.println("gate2");
    bool_photoGate2 = true; // true=1,open; false=0,closed
    photoGate2_MS_now = currentTime;    
}

// Dimmer
void zero_cross_detect() {    
  zero_cross = true;               // set the boolean to true to tell our dimming function that a zero cross has occured
  dim_counter=0;
  digitalWrite(pin_dim1, LOW);       // turn off TRIAC (and AC)
  digitalWrite(pin_dim2, LOW); 
}  

// Turn on the TRIAC at the appropriate time
void dim_check() {                   
  if(zero_cross == true) {      
    if(dim_counter>=dim1) {                     
      digitalWrite(pin_dim1, HIGH); // turn on light
    } 
    if(dim_counter>=dim2) {                     
      digitalWrite(pin_dim2, HIGH); // turn on light
    }  
    dim_counter++; // increment time step counter                                                        
  }                                  
}


// Motor Control

void motorMove(int motor, int speed, int direction){
//motorMove specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(pin_motorSTBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(pin_motorAIN1, inPin1);
    digitalWrite(pin_motorAIN2, inPin2);
    analogWrite(pin_motorPWMA, speed);
  }else{
    digitalWrite(pin_motorBIN1, inPin1);
    digitalWrite(pin_motorBIN2, inPin2);
    analogWrite(pin_motorPWMB, speed);
  }
}

void motorStop(){
//enable standby  
  digitalWrite(pin_motorSTBY, LOW); 
}

// Distance calculation
float dist(float deg1, float deg2) {
   float rad1 = deg1 * PI/180;
   float rad2 = deg2 * PI/180;
   float r = 250;
   float offset = r*2;
   float x1 = r*cos(rad1);
   float y1 = r*sin(rad1);
   float x2 = r*cos(rad2)-offset;
   float y2 = r*sin(rad2);
   float dist = sqrt(pow((x1-x2),2) + pow(y1-y2,2));
   return dist;
}

void motorToZero() {
  Serial.println("Motor1 to Zero...");
  while(!bool_photoGate1) {
    motorMove(motor1, 120, motorDirection1);
  }
  motorMove(motor1, 0, 1);
  
  Serial.println("Motor2 to Zero...");
  while(!bool_photoGate2) {
    motorMove(motor2, 120, motorDirection2);
  }
  motorStop();
  Serial.println("Done.");
}

void motorSlowStart() {
  update();
  for(int i = 157; i < val_motorSpeedPot1; i++) {
    motorMove(motor1, i, motorDirection1);
    motorMove(motor2, i, motorDirection2);
    Serial.print("Motor speed: ");
    Serial.println(i);
    delay(10000);
  }
}

void serialPrinter() {
  Serial.print("Motor1: ");
  Serial.print(val_motorSpeedPot1);
  Serial.print("\t");
  Serial.print(photoGate1_time_average);
  Serial.print("\t");
  Serial.print(photoGate1_deviation);
  Serial.print("\t");
  Serial.print(motor1degrees);
  Serial.print("\t");

  Serial.print("Motor2: ");
  Serial.print(val_motorSpeedPot2);
  Serial.print("\t");
  Serial.print(photoGate2_time_average);
  Serial.print("\t");
  Serial.print(photoGate2_deviation);
  Serial.print("\t");
  Serial.print(motor2degrees);
  Serial.print("\t");
  
  Serial.print("Dim: ");
  Serial.print(dim1);
  Serial.print("/");
  Serial.print(dim2);
  
  Serial.print("\t");
  Serial.print(currentTime);
  Serial.print("\t");
  Serial.print(motor1SpeedIncreaseDuration);
  Serial.print("\t");
  Serial.print(motor2SpeedIncreaseDuration);
  
  Serial.println();
}
