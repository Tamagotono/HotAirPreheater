// *****************************************************************
//   ********************* FUNCTIONS *****************************
// *****************************************************************

// **************************** INTERRUPT ************************
// This is the Timer 1 CTC interrupt, it goes off once a second
SIGNAL(TIMER1_COMPA_vect) { 

  // time moves forward!
  if (TM_Selection == lastMenu) {
    seconds_time++;
    lastMenu = TM_Selection;
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
  Serial.print(TM_Selection);
  Serial.print("\t");
  Serial.print(buttonTime);
  Serial.print("\t");
  Serial.print(CurrentlyDisplayedItem);
  Serial.print("\t");
  Serial.print(airTemp);
  Serial.print("\t");
  Serial.print(target_temperature);
  Serial.print("\t");
  Serial.println(relay_state);
} 


// *************** Top Menu Select *****************
void CheckForSelection( int* MenuLevelSelection, int MenuItem ){ 
// CheckForSelection( int *MenuLevelSelection, MenuItemNumber );
// must pass the address of the variable that stores the last selected menu item for that menu level
// if the button has been pressed long enough it will clear the display and update that variable with MenuItem.
//i.e.  CheckForSelection( &TM_Selection, 4 ) will update TM_Selection to the value of 4 if the button has been
// held down for greater than BUTTON_PRESS_TIME.
  if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_TIME) ) {
    lcd.clear();
    *MenuLevelSelection = MenuItem;
    Enc.write(lastEnc);
  }
}



void lcd_display_once( String DisplayText ){// ********* LCD DISPLAY ONCE ************
  if (CurrentlyDisplayedItem != LastDisplayedItem) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( DisplayText );
    LastDisplayedItem = CurrentlyDisplayedItem;
  }
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

// ***************************** DisplayTemp ****************************
void DisplayTemp(){
  //Display temperature setpoint
  lcd.setCursor(11,0);
  lcd.print(target_temperature);
  #if ARDUINO >= 100
  lcd.write(0xDF); // print the degree symbol
#else
  lcd.print(0xDF, BYTE);
#endif
  lcd.print("C ");

  lcd.print("  "); // get rid of any trailing digits 
  // go to line #1
//  lcd.setCursor(0,1);
  //Display current temp
//  lcd.print(airTemp); // Display temperature of the airstream
  lcd.setCursor(0,1);
  lcd.print(chipTemp); // Display temperature at the board
#if ARDUINO >= 100
  lcd.write(0xDF); // print the degree symbol
#else
  lcd.print(0xDF, BYTE);
#endif
  lcd.print("C ");
}


// ***************************** DisplayTime ****************************
void DisplayTime() {
  int minutes = (seconds_time / 60);
  int seconds = (seconds_time % 60);
  lcd.setCursor(0, 0);
  lcd.print("T:");
  if ( minutes <= 9) { // add a leading space to the minute clock if less than 10 seconds
    lcd.print(" ");
  }    
  lcd.print( minutes ); // minutes
  lcd.print(":");
  if ( seconds <= 9) { // add a leading zero to the seconds clock if less than 10 seconds
    lcd.print("0");
  }
  lcd.print( seconds ); // seconds
}

