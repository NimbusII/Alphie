
#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

#define MODE_DEBUG 1

constexpr uint8_t DRIVE_LEFT_PIN_A = 3;
constexpr uint8_t DRIVE_LEFT_PIN_B = 4;
constexpr uint8_t DRIVE_RIGHT_PIN_A = 5;
constexpr uint8_t DRIVE_RIGHT_PIN_B = 6;

constexpr uint8_t ENCODER_LEFT_PIN_A = 7;
constexpr uint8_t ENCODER_LEFT_PIN_B = 8;
constexpr uint8_t ENCODER_RIGHT_PIN_A = 9;
constexpr uint8_t ENCODER_RIGHT_PIN_B = 10;

constexpr uint8_t FRONT_LEFT_IR_PIN = 11;
constexpr uint8_t FRONT_RIGHT_IR_PIN = 12;
constexpr uint8_t REAR_LEFT_IR_PIN = 13;
constexpr uint8_t REAR_RIGHT_IR_PIN = 14;

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

volatile double leftEncoderCount = 0;
volatile double rightEncoderCount = 0;

void initPins() {
  pinMode(DRIVE_LEFT_PIN_A, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_B, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_A, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_B, OUTPUT);

  pinMode(START_PIN, INPUT_PULLUP);

  // TODO: this is where I'm at. Don't trust anything past this point. Probably not much before this point either, but.....
  pinMode(ENCODER_LEFT_PIN_A, INPUT);
  pinMode(ENCODER_LEFT_PIN_B, INPUT);
  pinMode(ENCODER_RIGHT_PIN_A, INPUT);
  pinMode(ENCODER_RIGHT_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_A), ISR_ENC_LA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_B), ISR_ENC_LB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_A), ISR_ENC_RA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_B), ISR_ENC_RB, RISING);

  // initialize LCD and set up the number of columns and rows:
  lcd.begin(16, 2);

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

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  initPins();
}

// the loop routine runs over and over again forever:
void loop() {
//  while (digitalRead(START_PIN))
//  {
//    delay(100);
//  }
//
//
//  SetDrive(0, 0);
//  delay(500);
//
//  SetDrive(200, 200);
//
//
//  delay(1000);

  cli();
  lcd.setCursor(0, 0);
  lcd.print("L: ");
  lcd.print(leftEncoderCount);
  lcd.setCursor(0, 1);
  lcd.print("R: ");
  lcd.print(rightEncoderCount);
  sei();

//  SetDrive(0, 0);
//  delay(500);
//
//  SetDrive(-200, -200);
//  delay(1000);
//  cli();
//  lcd.setCursor(0, 0);
//  lcd.print("L: ");
//  lcd.print(round(leftEncoderCount));
//  lcd.setCursor(0, 1);
//  lcd.print("R: ");
//  lcd.print(round(rightEncoderCount));
//  sei();
//
//  SetDrive(0, 0);
}

void ISR_ENC_LA()
{
//  if (digitalRead(ENCODER_LEFT_PIN_B))
//  {
//    if (leftEncoderCount < 0xFFFFFFFFFFFFFFFF)
      leftEncoderCount++;
//  }
//  else
//  {
//    if (leftEncoderCount > 0)
//      leftEncoderCount--;
//  }
}

void ISR_ENC_LB()
{
//  if (!digitalRead(ENCODER_LEFT_PIN_A))
//  {
//    if (leftEncoderCount < 0xFFFFFFFFFFFFFFFF)
      leftEncoderCount++;
//  }
//  else
//  {
//    if (leftEncoderCount > 0)
//      leftEncoderCount--;
//  }
}

void ISR_ENC_RA()
{
  if (!digitalRead(ENCODER_RIGHT_PIN_B))
  {
    if (rightEncoderCount < 0xFFFFFFFFFFFFFFFF)
      rightEncoderCount++;
  }
  else
  {
    if (rightEncoderCount > 0)
      rightEncoderCount--;
  }
}

void ISR_ENC_RB()
{
  if (digitalRead(ENCODER_RIGHT_PIN_A))
  {
    if (rightEncoderCount < 0xFFFFFFFFFFFFFFFF)
      rightEncoderCount++;
  }
  else
  {
    if (rightEncoderCount > 0)
      rightEncoderCount--;
  }
}

