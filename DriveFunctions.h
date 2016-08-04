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

  int32_t leftTicksAhead = rtL - rtR;

  leftSpeed = leftSpeed - leftTicksAhead/STRAIGHTEN_SCALING_FACTOR;
  rightSpeed = rightSpeed + leftTicksAhead/STRAIGHTEN_SCALING_FACTOR;

  float readjustmentFactor = (leftSpeed + rightSpeed) / desiredSpeed * 2.0F;
  leftSpeed = leftSpeed/readjustmentFactor;
  rightSpeed = rightSpeed/readjustmentFactor;

  SetDrive(int leftSpeed, int rightSpeed)
}

void DriveLogic()
{
  //nah this is actually really bad, the interrupts are only going to stop it if they retrigger at a certain time, wtf am I even doing here?
  if (isLeftFrontTriggered)
  {
    delay(100);
    SetDrive(-200, -200);
    delay(700);
    SetDrive(200, -200);
    delay(400); //don't do this, this is awful;
    if (!digitalRead(FRONT_LEFT_IR_PIN))
    {
      isLeftFrontTriggered = false;
      isRightFrontTriggered = false;
    }
  }
  else if (isRightFrontTriggered)
  {
    delay(100);
    SetDrive(-200, -200);
    delay(700);
    SetDrive(-200, 200);
    delay(400); //don't do this, this is awful;
    if (!digitalRead(FRONT_RIGHT_IR_PIN))
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
