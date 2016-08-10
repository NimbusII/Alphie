
#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#include "WProgram.h"
#include "QuadDecode_def.h"
//#include "DriveFunction/s.h"
#include <stdbool.h>

#define MODE_DEBUG 1
#define GUI_UPDATE_TIME   100000 //1s
#define STRAIGHTEN_SCALING_FACTOR 100

constexpr uint8_t DRIVE_LEFT_PIN_A = 5;
constexpr uint8_t DRIVE_LEFT_PIN_B = 6;
constexpr uint8_t DRIVE_RIGHT_PIN_A = 9;
constexpr uint8_t DRIVE_RIGHT_PIN_B = 10;

//constexpr uint8_t ENCODER_LEFT_PIN_A = 3
//constexpr uint8_t ENCODER_LEFT_PIN_B = 4
//constexpr uint8_t ENCODER_RIGHT_PIN_A = 25
//constexpr uint8_t ENCODER_RIGHT_PIN_B = 32

constexpr uint8_t FRONT_LEFT_IR_PIN = 7;
constexpr uint8_t FRONT_RIGHT_IR_PIN = 8;
constexpr uint8_t REAR_LEFT_IR_PIN = 11;
constexpr uint8_t REAR_RIGHT_IR_PIN = 12;

constexpr uint8_t START_PIN = 15;
constexpr uint8_t SHARP_IR = 17;

constexpr uint8_t LCD_RS_PIN = 18;
constexpr uint8_t LCD_ENABLE_PIN = 19;
constexpr uint8_t LCD_D4_PIN = 20;
constexpr uint8_t LCD_D5_PIN = 21;
constexpr uint8_t LCD_D6_PIN = 22;
constexpr uint8_t LCD_D7_PIN = 23;

// leftover pins: 0, 1 (TX, RX)
// 2
// 13-18 (13 built in led)
// 16, 17 will probably be secondary motors at some point

///////////////////ENCODERS////////////////////
volatile int32_t rtL = 0, rtR = 0;

volatile bool doOutput = false; // Run output routine

IntervalTimer serialTimer;

void timerInt(void);  // Main timing loop interrupt

QuadDecode<1> leftEncoder;  // Template using FTM1
QuadDecode<2> rightEncoder;  // Template using FTM2


//        rightEncoder.zeroFTM();

//////////////////////////////////////////////////////

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS_PIN, LCD_ENABLE_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

enum PROG {
  DRIVE_STRAIGHT,
  CAN_RUN,
  NUM_PROGRAMS
};

volatile bool isLeftFrontTriggered = false;
volatile bool isRightFrontTriggered = false;
volatile bool isLeftRearTriggered = false;
volatile bool isRightRearTriggered = false;

int leftSpeed = 0;
int rightSpeed = 0;
int rangeValue = 0;
int32_t prevLeftTicksAhead = 0;

///////////////FUNCTION DECLERAIONS////////////

void SetDrive(int left, int right);
void DriveStraight(int desiredSpeed);

/////////////////////////////////////////////

void initPins() {
  pinMode(DRIVE_LEFT_PIN_A, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_B, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_A, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_B, OUTPUT);

  pinMode(START_PIN, INPUT_PULLUP);
  
  pinMode(FRONT_LEFT_IR_PIN, INPUT);
  pinMode(FRONT_RIGHT_IR_PIN, INPUT);
  pinMode(REAR_LEFT_IR_PIN, INPUT);
  pinMode(REAR_RIGHT_IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_IR_PIN), ISR_IR_LF, RISING);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_IR_PIN), ISR_IR_RF, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_IR_PIN), ISR_IR_LR, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_IR_PIN), ISR_IR_RR, RISING);

  // initialize LCD and set up the number of columns and rows:
  lcd.begin(16, 2);
}

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  initPins();
  SetDrive(0, 0);

  leftEncoder.setup();      // Start Quad Decode position count
  rightEncoder.setup();      // Start Quad Decode position count

  serialTimer.begin(timerInt, GUI_UPDATE_TIME); // GUI Update time

  while (digitalRead(START_PIN))
  {
    delay(100);
  }
  
  isLeftFrontTriggered = false;
  isRightFrontTriggered = false;
  delay(500);
}

// the loop routine runs over and over again 5ever:
extern "C" int main(void)
{
  setup();

  while (true)
  {
	  rangeValue = analogRead(SHARP_IR);
	  Serial.println(rangeValue);
	  delay(100);
//    DriveLogic();

//    if (doOutput)
//    {
//      doOutput = false;
//
//      DriveStraight(80);
//
//      // Send out axis values
////      Serial.print("L_Encoder: ");
////      Serial.println(rtL);
////      Serial.print("R_Encoder: ");
////      Serial.println(rtR);
//
//      Serial.println("MSTeensy Loop");Serial.println("");
//    }


  }

}

void ISR_IR_LF()
{
  if (!isLeftFrontTriggered)
  {
    //    SetDrive(0, 0);
  }
  isLeftFrontTriggered = true;
}

void ISR_IR_RF()
{
  if (!isRightFrontTriggered) //hack debounce
  {
    //    SetDrive(0, 0);
  }
  isRightFrontTriggered = true;
}

void ISR_IR_LR()
{
  SetDrive(0, 0);
  isLeftRearTriggered = true;
}

void ISR_IR_RR()
{
  SetDrive(0, 0);
  isRightRearTriggered = true;
}

void timerInt(void)
{
    doOutput=true;
}

void SetDrive(int left, int right)
{
  if (left >= 0)
  {
    analogWrite(DRIVE_LEFT_PIN_A, abs(left));
    analogWrite(DRIVE_LEFT_PIN_B, 0);
  }
  else
  {
    analogWrite(DRIVE_LEFT_PIN_A, 0);
    analogWrite(DRIVE_LEFT_PIN_B, abs(left));
  }

  if (right >= 0)
  {
    analogWrite(DRIVE_RIGHT_PIN_A, abs(right));
    analogWrite(DRIVE_RIGHT_PIN_B, 0);
  }
  else
  {
    analogWrite(DRIVE_RIGHT_PIN_A, 0);
    analogWrite(DRIVE_RIGHT_PIN_B, abs(right));
  }
}

void DriveStraight(int desiredSpeed)
{
  rtL = leftEncoder.calcPosn();
  rtR = rightEncoder.calcPosn();

  Serial.print("initial L_Speed: ");
  Serial.println(leftSpeed);
  Serial.print("initial R_Speed: ");
  Serial.println(rightSpeed);

  Serial.print("L_Encoder: ");
  Serial.println(rtL);
  Serial.print("R_Encoder: ");
  Serial.println(rtR);

  int32_t leftTicksAhead = rtL - rtR;

  if((leftTicksAhead * prevLeftTicksAhead) < 0)
  {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  prevLeftTicksAhead = leftTicksAhead;
  Serial.print("leftTicksAhead: ");
  Serial.println(leftTicksAhead);

  if(leftSpeed == 0 && rightSpeed == 0)
  {
    leftSpeed = desiredSpeed;
    rightSpeed = desiredSpeed;
    return;
  }

  leftSpeed = leftSpeed - leftTicksAhead/STRAIGHTEN_SCALING_FACTOR;
  rightSpeed = rightSpeed + leftTicksAhead/STRAIGHTEN_SCALING_FACTOR;

  Serial.print("adjusted L_Speed: ");
  Serial.println(leftSpeed);
  Serial.print("adjusted R_Speed: ");
  Serial.println(rightSpeed);

  float readjustmentFactor = float(abs(leftSpeed) + abs(rightSpeed)) / float(abs(desiredSpeed) * 2.0F);
  Serial.print("readjustmentFactor: ");
  Serial.println(readjustmentFactor);
  int scaledleftSpeed = leftSpeed/readjustmentFactor;
  int scaledrightSpeed = rightSpeed/readjustmentFactor;
  
  Serial.print("scaled L_Speed: ");
  Serial.println(scaledleftSpeed);
  Serial.print("scaled R_Speed: ");
  Serial.println(scaledrightSpeed);
  
  SetDrive(scaledleftSpeed, scaledrightSpeed);
}

