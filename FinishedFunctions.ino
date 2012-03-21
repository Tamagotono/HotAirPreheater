// *****************************************************************
//   ********************* FUNCTIONS *****************************
// *****************************************************************

// **************************** INTERRUPT ************************
// This is the Timer 1 CTC interrupt, it goes off once a second
SIGNAL(TIMER1_COMPA_vect) { 

  // time moves forward!
  if (menu == lastMenu) {
    seconds_time++;
    lastMenu = menu;
  }


  // save the last reading for our slope calculation
  previous_temperature = airTemp;

  // we will want to know the temperatures in the main loop()
  // instead of constantly reading them, we'll just use this interrupt
  // to track them and save them once a second to 'airTemp' and 'chipTemp'
  airTemp = airTC.readCelsius();
  chipTemp = chipTC.readCelsius();


  // Sum the error over time
  Summation += target_temperature - airTemp;

  if ( (airTemp < (target_temperature * (1.0 - WINDUPPERCENT))) ||
    (airTemp > (target_temperature * (1.0 + WINDUPPERCENT))) ) {
    // to avoid windup, we only integrate within 5%
    Summation = 0;
  }
  // print out a log so we can see whats up
  Serial.print(seconds_time);
  Serial.print("\t");
  Serial.print(buttonState);
  Serial.print("\t");
  Serial.print(menu);
  Serial.print("\t");
  Serial.print(buttonTime);
  Serial.print("\t");
  Serial.print(option);
  Serial.print("\t");
  Serial.print(airTemp);
  Serial.print("\t");
  Serial.print(target_temperature);
  Serial.print("\t");
  Serial.println(relay_state);
} 


// **************************** CHECK BUTTON STATE ************************
void check_button_state(){
  // check if the button is pressed
  buttonState = digitalRead(BUTTON);
  if ( lastButtonState != buttonState ) {
    if (buttonState == LOW) {
      //    Serial.println("Pressed");
      lastButtonState = LOW;
      buttonTime = millis();
    }
    else {
      //      Serial.println("Released");
    }  
  }
  lastButtonState = buttonState;
  if ( (buttonState == LOW) && (millis() - buttonTime >= RESET_TIME) ) {
    soft_reset(); // reset if encoder button is pressed for 5 seconds
  }
}


// **************************** WDT_INT (RESET) ************************
void wdt_init(void) // to disable the watchdog timer after a soft reset
{
  MCUSR = 0;
  wdt_disable();

  return;
}

void manual_mode() {//******************************  MANUAL MODE  ******************************
  if (menu != lastMenu) {
    Enc.write(DEFAULT_TEMP);
  }
  check_button_state();
  update_time();
  update_temp();

  newTarget = Enc.read();
  if (newTarget != target_temperature) {
    cli();
    if (newTarget > MAX_TEMP) {
      newTarget = 1;
      Enc.write(newTarget);
    }
    if (newTarget <= 0) {
      newTarget = MAX_TEMP;
      Enc.write(newTarget);
    }
    target_temperature = newTarget;
    update_temp();
    sei();
  }
}

// ***************************** update_temp ****************************
void update_temp(){
  // go to line #1
  lcd.setCursor(0,1);
  lcd.print(airTemp);
  lcd.setCursor(8,1);
  lcd.print(chipTemp);
#if ARDUINO >= 100
  lcd.write(0xDF); // print the degree symbol
#else
  lcd.print(0xDF, BYTE);
#endif
  lcd.print("C ");
}

