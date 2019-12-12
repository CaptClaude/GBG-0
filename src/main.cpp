#include <Arduino.h>

/*
Adapted from First Wheels Fluid Motion Device V 2.2 6/3/2015
Originally created by Blake Palmer of
    First Wheels Special Needs Mobility Ministry,
    A project of Thru the Roof Special Needs Ministry at Houston's First Baptist Church.
    http://www.firstwheelshouston.org
    http://www.houstonsfirst.org

Basics:
Reads digital input pins to determine button states.
To Do:
Read an analog input pin (A0) connected to a potentiometer, maps the result to a range from 0 to 255
and uses the result to set the max speed.
Second analog pin (A2) connected to second potentiometer to provide steering trim control to attempt to compensate for variations
in speed between the motors.

Updates by Jeff Mizener:
Adapted for the Platform IO environment
Modified to work with the Sabertooth motor controller 10/2019, incremental increase in functionality

Project sponsored by Workchops, Dallas TX October 2019
*/


 // The circuit:
 // * analog pin 0 - throttle/governor potentiometer signal (middle/white wire).
 //   Center pin of the potentiometer goes to the analog pin.
 //   side pins of the potentiometer go to +5V (red) and ground (black).

 // * analog pin 2 - steering trim potentiometer signal (middle/white wire).
 //   Center pin of the potentiometer goes to the analog pin.
 //   side pins of the potentiometer go to +5V (red) and ground (black).


//#include <Servo.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

//Servo Sabertooth; // We'll name the Sabertooth object Sabertooth.
// For how to configure the Sabertooth, see the DIP Switch Wizard for
// http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
// Be sure to select Simplified Serial Mode for use with this library.
// This sample uses a baud rate of 9600.
//
// Connections to make:
//   Arduino D11    ->  Sabertooth S1
//   Note: Serial output to Sabertooth is moved from TX01 (default) to D11 for easier debugging.
//   Arduino GND    ->  Sabertooth 0V
//   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//   Controls are switches connected between the digital pin and ground: Active Low
//   D2  ->  Emergency Stop Button
//   D3  ->  Forward Button
//   D4  ->  Reverse Button
//   D5  ->  Left Button
//   D6  ->  Right Button
// Connections for potentiometer (pot) to govern Maximum speed
//   0V  ->  Lead 1 - black (ground)
//   A0  ->  Lead 2 - white (signal)
//   5v  ->  Lead 3 - red (+)
// Connections for potentiometer (pot) to help match motor speeds for a given input
// Setting this is a calibration step
//   0V  ->  Lead 1 - black (ground)
//   A2  ->  Lead 2 - white (signal)
//   5v  ->  Lead 3 - red (+)


//***********************************************************
//*** Adjustable parameters:
const byte ndebug      = 1; //*** 0=run (debug messages off) 1=debug messages on
const byte accelFactor = 4; //*** accelerate increment
const byte decelFactor = 4; //*** decelerate increment
const byte trimSens    = 6;//*** trim sensitivity: 128 = highest sensitivity
//***********************************************************

// These constants shouldn't change.

const byte deadStop1 = 64;  // command value at center/stopped for motor 1
const byte deadStop2 = 192; // command value at center/stopped for motor 2

//***********************************************************
// Initialize work variables:

int  motorControl1 = deadStop1;
int  motorControl2 = deadStop2;
int  motorPWR_Right = 20;
int  motorPWR_Left = 20;

int  rawSpeed1 = 0;
int  rawSpeed2 = 0;
int  speedDifference = 0;
byte  direction1 = 1;    // '1' or '0' (Forward or Reverse) are valid values;
byte  direction2 = 1;


byte buttonState = 0;        // value read from the button
byte buttonPinNumber;        // input pin associated with this button
byte buttonPressedIndicator = 0; // Flag to indicate if any button pressed or not

float potValue   = 0;     // value read from the potentiometer
int   trimAdjustment = 0;

// These values are the max command values as determined by the throttle (potentiomemter) setting
byte maxFwdSpeed1 = 0;
byte maxFwdSpeed2 = 0;
byte maxRevSpeed1 = 0;
byte maxRevSpeed2 = 0;

//***********************************************************
#define BTN_ESTOP  2 // * Digital pin 2 - Emergency Stop button
#define BTN_FWD  3 // * Digital pin 3 - Forward button
#define BTN_REV  4 // * Digital pin 4 - Reverse button
#define BTN_LFT  5 // * Digital pin 5 - Left turn button
#define BTN_RGT  6 // * Digital pin 6 - Right turn Button
#define SoftSerialPin 11 // * Digital pin 11 - Serial output to Sabertooth
#define LED 13 // * Digital pin 13 - built-in LED
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

#define FORWARD 1
#define FORWARD_RIGHT 3
#define RIGHT 2
#define BACK_RIGHT 6
#define BACK 4
#define BACK_LEFT 12
#define LEFT 8
#define FORWARD_LEFT 9



// Transmit on pin 11, ignore what the Sabertooth sends back
SoftwareSerial SWSerial(NOT_A_PIN, SoftSerialPin); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

void setup()
{

 delay(200);  // Give the Sabertooth time to intialize.
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  SWSerial.begin(9600);
  ST.autobaud();

//  mySerial.begin(9600);

 //Note: Pin 11 is the Arduino serial output which connects to the Sabertooth S1 terminal.

 // Define the input pins (which correspond to button jacks)
  pinMode(BTN_ESTOP, INPUT_PULLUP); // Emergency Stop
  digitalWrite(BTN_ESTOP, HIGH);

  pinMode(BTN_FWD, INPUT_PULLUP); // Forward
  digitalWrite(BTN_FWD, HIGH);

  pinMode(BTN_REV, INPUT_PULLUP); // Reverse
  digitalWrite(BTN_REV, HIGH);

  pinMode(BTN_LFT, INPUT_PULLUP); // Left
  digitalWrite(BTN_LFT, HIGH);

  pinMode(BTN_RGT, INPUT_PULLUP); // Right
  digitalWrite(BTN_RGT, HIGH);

//  Sabertooth.attach(1);

};

void loop()
{
  int sw_F = digitalRead(BTN_FWD);
  int sw_R = digitalRead(BTN_RGT) * 2;
  int sw_B = digitalRead(BTN_REV) * 4;
  int sw_L = digitalRead(BTN_LFT) * 8;

//  Serial.print( "sw_F: " );
//  Serial.print( sw_F );
//  Serial.print( "  sw_R: ");
//  Serial.print( sw_R );
//  Serial.print( "  sw_B: ");
//  Serial.print( sw_B );
//  Serial.print( "  sw_L: ");
//  Serial.println( sw_L );

  int Direction = 15 - (sw_F + sw_R + sw_B + sw_L);
//  Serial.print("Direction: ");
//  Serial.println(Direction);
  switch (Direction) {
    case FORWARD:        //1
      Serial.println( "Forward" );
      ST.motor(MOTOR_LEFT, motorPWR_Left);
      ST.motor(MOTOR_RIGHT, motorPWR_Right);
      break;
    case FORWARD_RIGHT:  //3
      Serial.println( "Forward Right" );
      ST.motor(MOTOR_LEFT, motorPWR_Left/2);
      ST.motor(MOTOR_RIGHT, motorPWR_Right);
      break;
    case RIGHT:           //2
      Serial.println( "Right" );
      ST.motor(MOTOR_LEFT, 0);
      ST.motor(MOTOR_RIGHT, motorPWR_Right);
      break;
    case BACK_RIGHT:      //6
      Serial.println( "Back Right" );
      ST.motor(MOTOR_LEFT, -motorPWR_Left/2
      );
      ST.motor(MOTOR_RIGHT, -motorPWR_Right);
      break;
    case BACK:            //4
      Serial.println( "Back" );
      ST.motor(MOTOR_LEFT, -motorPWR_Left);
      ST.motor(MOTOR_RIGHT, -motorPWR_Right);
      break;
    case BACK_LEFT:       //12
      Serial.println( "Back Left" );
      ST.motor(MOTOR_LEFT, -motorPWR_Left);
      ST.motor(MOTOR_RIGHT, -motorPWR_Right/2);
      break;
    case LEFT:            //8
      Serial.println( "Left" );
      ST.motor(MOTOR_LEFT, motorPWR_Left);
      ST.motor(MOTOR_RIGHT, 0);
      break;
    case FORWARD_LEFT:   //9
      Serial.println( "Forward Left" );
      ST.motor(MOTOR_LEFT, motorPWR_Left);
      ST.motor(MOTOR_RIGHT, motorPWR_Right/2);
      break;
    case 0:
    ST.motor(MOTOR_LEFT, 0);
    ST.motor(MOTOR_RIGHT, 0);
      break;

 };


};
