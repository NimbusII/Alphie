void DisplayString(int currentProgramSelected)
{
  lcd.setCursor(0, 0);

  switch (currentProgramSelected)
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

  while (digitalRead(START_PIN))
  {
    currentSelected = analogRead(START_PIN) / interval;
    DisplayString(currentSelected);
  }

  return currentSelected;
}
