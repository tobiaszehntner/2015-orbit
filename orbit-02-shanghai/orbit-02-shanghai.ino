// Maths
#include <Average.h>
#include "Arduino.h"
// Encoders
#include <digitalWriteFast.h>
// Dimmer
#include <TimerOne.h>
// RTC
#include <Wire.h>
#include "RTClib.h"
// Digit Displays
#include <SevSeg.h> // https://github.com/DeanIsMe/SevSeg

// Settings
const bool bool_motorOn = true;
const bool bool_lightsOn = true;

const bool bool_photoGatePrint = false;
const bool bool_deviationLightsOff = false;
const bool bool_zeroMotor = true;
const bool bool_slowStart = true;
const bool bool_serialPrinter_on = false;
const bool bool_manualSpeed = false;
const bool bool_RTC_schedule_on = true;

byte motor1_fixedSpeed = 144; // if motor1=163, 2=164, they have exact same speed (won't fall in/out of sync)
byte motor2_fixedSpeed = 141; // 162-165 for opening: almost smashed after 4.5 hours
byte motorSlowStartDifference = 5;
byte minDim = 50; //  70, 50, 30 flickers
byte maxDim = 110; // 110, 90
const byte photoGate_arraySize = 5;
byte deviationAllowance = 50;

// 163 start = after 0.5h very close. after 1h15m getting a bit too weak.

// RTC setup
RTC_DS1307 RTC;
const byte awake_hour = 10;
const byte awake_min = 00;
const byte asleep_hour = 18;
const byte asleep_min = 00;
bool bool_awake = true;
bool bool_runMotorSetup = true;

// Digit Displays
SevSeg display1;
SevSeg display2;

// Time
unsigned long currentTime = 0;

// Dimmer setup
byte pin_dim1 = 8;               // Output to Opto Triac
byte pin_dim2 = 9;
byte interrupt_dim = 0;  // interrupt 0 = pin 2
volatile int dim_counter = 0;                           
volatile boolean zero_cross = 0;
int freqStep = 78; //China = 78 // US = 65=128 steps, 32=256
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

  // Displays (adjust pins before enabling!)
//  byte display1_numDigits = 3;
//  byte display1_digitPins[] = {2, 3, 4, 5};
//  byte display1_segmentPins[] = {6, 7, 8, 9, 10, 11, 12, 13};
//  display1.begin(COMMON_ANODE, display1_numDigits, display1_digitPins, display1_segmentPins);
//  display1.setBrightness(90); // 0-100
//  
//  byte display2_numDigits = 3;
//  byte display2_digitPins[] = {2, 3, 4, 5};
//  byte display2_segmentPins[] = {6, 7, 8, 9, 10, 11, 12, 13};
//  display2.begin(COMMON_ANODE, display2_numDigits, display2_digitPins, display2_segmentPins);
//  display2.setBrightness(90); // 0-100
  
  // Bring motor to 0
  if(!bool_RTC_schedule_on) {
     motorSetup();
  }  
}

void loop() { 
  
  if(bool_RTC_schedule_on) {
    
    getTime();   
      
    if(bool_awake) {
      if(bool_runMotorSetup) {
        DateTime now = RTC.now();
        printTime(now);
        Serial.print("Awake until ");
        Serial.print(asleep_hour, DEC);
        Serial.print("h ");
        Serial.print(asleep_min, DEC);
        Serial.print("m");
        Serial.println();
        Serial.println();
        
        motorSetup();
        bool_runMotorSetup = false;
      }
      
      update();
        
    } else {
      
      DateTime now = RTC.now();
      printTime(now);
      Serial.print("Asleep until ");
      Serial.print(awake_hour, DEC);
      Serial.print("h ");
      Serial.print(awake_min, DEC);
      Serial.print("m");
      Serial.println();
      Serial.println();
      
      dim1 = dimStep;                 // Dimming level (0-128)  0 = on, 128 = 0ff
      dim2 = dimStep;
      motorStop();
      bool_runMotorSetup = true;
      
      delay(10000);
    }
  } else {
    update();
  } 
} // end loop

void update() {
  
  currentTime = millis();
  
  // MotorSpeed
  if(bool_motorOn) {
    
    if(bool_manualSpeed) {
          val_motorSpeedPot1 = map(analogRead(pin_motorSpeedPot1), 0, 1023, 120, 200); 
          val_motorSpeedPot2 = map(analogRead(pin_motorSpeedPot2), 0, 1023, 120, 200);
          
    } else {
      val_motorSpeedPot1 = motor1_fixedSpeed;
      val_motorSpeedPot2 = motor2_fixedSpeed;
    }
    
    motorMove(motor1, val_motorSpeedPot1, motorDirection1); //motor 1, speed 0-255, left
    motorMove(motor2, val_motorSpeedPot2, motorDirection2); //motor 2, speed 0-255, left
    
    display1.setNumber(val_motorSpeedPot1,0);
    display2.setNumber(val_motorSpeedPot2,0);
    display1.refreshDisplay();
    display2.refreshDisplay();
    
  } else {
      motorStop();
  }

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
//            Serial.print("Motor1 Deviation: ");
//            Serial.println(photoGate1_deviation);
          }
          if(photoGate2_deviation < deviationAllowance) {
            dim2 = map(dist(motor1degrees, motor2degrees), 0, 1000, minDim, maxDim);
          } else {
            dim2 = dimStep;
//            Serial.print("Motor2 Deviation: ");
//            Serial.println(photoGate2_deviation);
          }
      }
      else {
        dim1 = map(dist(motor1degrees, motor2degrees), 0, 1000, maxDim, minDim);
        dim2 = map(dist(motor1degrees, motor2degrees), 0, 1000, maxDim, minDim);
      }
  } else {
    dim1 = dimStep;
    dim2 = dimStep;
  }
  
  if(bool_serialPrinter_on) {
    serialPrinter();
  }
  
}


// RTC

void getTime() {
  DateTime now = RTC.now();
  byte RTC_hour = now.hour();
  byte RTC_min = now.minute();
  
  if(RTC_hour >= awake_hour && RTC_hour <= asleep_hour) {
    if(RTC_hour > awake_hour && RTC_hour < asleep_hour) {
      bool_awake = true;
    } else {
      if(RTC_hour == awake_hour && RTC_min >= awake_min) {
        if(RTC_hour == asleep_hour && RTC_min < asleep_min) {
          bool_awake = true;
        } else {
          bool_awake = false;
        }
      } else {
        bool_awake = false;
      }
    }
  } else {
      bool_awake = false;
  }
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
    bool_photoGate1 = true; // true=1,open; false=0,closed
    photoGate1_MS_now = currentTime;
    if(bool_photoGatePrint) {
      Serial.println("gate1");
    }
}

void photoGate2Detect() {
    bool_photoGate2 = true; // true=1,open; false=0,closed
    photoGate2_MS_now = currentTime;   
     if(bool_photoGatePrint) {
       Serial.print("\t");
       Serial.println("gate2");
      } 
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

void motorSetup() {
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
}

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
  int j = motor2_fixedSpeed - motorSlowStartDifference;
  for(int i = motor1_fixedSpeed - motorSlowStartDifference; i < motor1_fixedSpeed; i++) {
    motorMove(motor1, i, motorDirection1);
    motorMove(motor2, j, motorDirection2);
    Serial.print("Motor speed: ");
    Serial.print(i);
    Serial.print("/");
    Serial.println(j);
    j++;
    delay(15000);
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
  
  Serial.println();
}
