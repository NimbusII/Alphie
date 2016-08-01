
#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

#define MODE_DEBUG 1

constexpr uint8_t DRIVE_LEFT_PIN_A = 3;
constexpr uint8_t DRIVE_LEFT_PIN_B = 4;
constexpr uint8_t DRIVE_RIGHT_PIN_A = 5;
constexpr uint8_t DRIVE_RIGHT_PIN_B = 6;

constexpr uint8_t ENCODER_LEFT_PIN_A = 11;
constexpr uint8_t ENCODER_LEFT_PIN_B = 12;
constexpr uint8_t ENCODER_RIGHT_PIN_A = 13;
constexpr uint8_t ENCODER_RIGHT_PIN_B = 14;

constexpr uint8_t FRONT_LEFT_IR_PIN = 7;
constexpr uint8_t FRONT_RIGHT_IR_PIN = 8;
constexpr uint8_t REAR_LEFT_IR_PIN = 9;
constexpr uint8_t REAR_RIGHT_IR_PIN = 10;

constexpr uint8_t START_PIN = 15;

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

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS_PIN, LCD_ENABLE_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

enum PROG{
  DRIVE_STRAIGHT,
  CAN_RUN,
  NUM_PROGRAMS
};


volatile double leftEncoderCount = 0;
volatile double rightEncoderCount = 0;

volatile bool isLeftFrontTriggered = false;
volatile bool isRightFrontTriggered = false;
volatile bool isLeftRearTriggered = false;
volatile bool isRightRearTriggered = false;

void initPins() {
  pinMode(DRIVE_LEFT_PIN_A, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_B, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_A, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_B, OUTPUT);

  pinMode(START_PIN, INPUT_PULLUP);

  // TODO: this is where I'm at. Don't trust anything past this point. Probably not much before this point either, but.....
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

void DisplayString(int currentProgramSelected)
{
  lcd.setCursor(0, 0);

  switch(currentProgramSelected)
  {
    case 0:
        lcd.print("Encoder Test");
        break;
    case 1:
        lcd.print("Main");
        break;
    default:
        lcd.print("Unknown Input");
        break;
  }
}

int ProgramSelect()
{
  int interval = 1024 / NUM_PROGRAMS; // truncate to whole number
  int currentSelected = 0;

  while(digitalRead(START_PIN))
  {
    currentSelected = analogRead(START_PIN) / interval;
    DisplayString(currentSelected);
  }

  return currentSelected;
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

void DriveLogic()
{
  //nah this is actually really bad, the interrupts are only going to stop it if they retrigger at a certain time, wtf am I even doing here?
  if(isLeftFrontTriggered)
  {
    delay(100);
    SetDrive(-200, -200);
    delay(700);
    SetDrive(200, -200);
    delay(400); //don't do this, this is awful;
    if(!digitalRead(FRONT_LEFT_IR_PIN))
    {
      isLeftFrontTriggered = false;
      isRightFrontTriggered = false;
    }
  }
  else if(isRightFrontTriggered)
  {
    delay(100);
    SetDrive(-200, -200);
    delay(700);
    SetDrive(-200, 200);
    delay(400); //don't do this, this is awful;
    if(!digitalRead(FRONT_RIGHT_IR_PIN))
    {
      isLeftFrontTriggered = false;
      isRightFrontTriggered = false;
    }
  }
  else
  {
    SetDrive(185, 185);
  }
}

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  initPins();
  SetDrive(0, 0);
    while (digitalRead(START_PIN))
    {
      delay(100);
    }
    isLeftFrontTriggered = false;
    isRightFrontTriggered = false;
    delay(500);

}

// the loop routine runs over and over again 5ever:
void loop() {

  DriveLogic();

}

void ISR_IR_LF()
{
  if(!isLeftFrontTriggered)
  {
//    SetDrive(0, 0);
  }
  isLeftFrontTriggered = true;
}

void ISR_IR_RF()
{
  if(!isRightFrontTriggered) //hack debounce
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



