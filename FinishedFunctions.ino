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
// **************************************************************************





// ************************** CheckForSelection *****************************
void CheckForSelection( int* MenuLevelSelection, int MenuItem ){ 
  // CheckForSelection( int *MenuLevelSelection, MenuItemNumber );
  // must pass the address of the variable that stores the last selected menu item for that menu level
  // if the button has been pressed long enough it will clear the display and update that variable with MenuItem.
  //i.e.  CheckForSelection( &TM_Selection, 4 ) will update TM_Selection to the value of 4 if the button has been
  // held down for greater than BUTTON_PRESS_TIME.
  if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_TIME) ) {
    //    lcd.clear();
    *MenuLevelSelection = MenuItem;
  }
}
// **************************************************************************




// ************************** lcd_display_once ******************************
void lcd_display_once( String DisplayText ){
  if (CurrentlyDisplayedItem != LastDisplayedItem) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( DisplayText );
    LastDisplayedItem = CurrentlyDisplayedItem;
  }
}
// **************************************************************************




// **************************** CHECK BUTTON STATE **************************
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
  /*
  lastButtonState = buttonState;
   if ( (buttonState == LOW) && (millis() - buttonTime >= BACK_MENU) ) {
   TM_Selection = 0; // goto the Top Menu if encoder button is pressed for 5 seconds
   }
   */
  if ( (buttonState == LOW) && (millis() - buttonTime >= RESET_TIME) ) {
    soft_reset(); // reset if encoder button is pressed for 5 seconds
  }
}
// **************************************************************************





// **************************** WDT_INT (RESET) *****************************
void wdt_init(void) // to disable the watchdog timer after a soft reset
{
  MCUSR = 0;
  wdt_disable();

  return;
}
// **************************************************************************




// ******************************* DisplayTemp ******************************
void DisplayTemp(){
  //Display temperature setpoint
  lcd.setCursor(11,0);
  if (target_temperature < 100) { //keep digits right justified
    lcd.print(" "); //clean any ghost digits
    if (target_temperature < 10) {
      lcd.print(" "); //clean any remaining ghost digits
    }
  }

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
// **************************************************************************




// ******************************* DisplayTime ******************************
void DisplayTime() {
  int minutes = (seconds_time / 60);
  int seconds = (seconds_time % 60);
  lcd.setCursor(0, 0);
  //  lcd.print("T:");
  if ( minutes <= 9) { // add a leading space to the minute clock if less than 10 seconds
    lcd.print("0");
  }    
  lcd.print( minutes ); // minutes
  lcd.print(":");
  if ( seconds <= 9) { // add a leading zero to the seconds clock if less than 10 seconds
    lcd.print("0");
  }
  lcd.print( seconds ); // seconds
}
// **************************************************************************


// ***************************** DisplayFanSpeed ****************************
void DisplayFanSpeed(){
  Serial.println( FanSpeed );
  lcd.setCursor(8,1);
  lcd.print("Fan:");
  if ( FanSpeed < 100 ) { // right justify fan speed
    lcd.print( " " );
    if ( FanSpeed < 10) {
      lcd.print( " " );
    }
  }
  lcd.print( FanSpeed );
  lcd.print("%");
}
// **************************************************************************


//******************************  MANUAL MODE  ******************************
void manual_mode() {
  /*
  if ( TM_Selection != lastMenu ) {
   Enc.write( target_temperature );
   }
   */
  //  check_button_state();
  //  CheckForSelection( &MM_Selection, 0 );
  Serial.println( MM_Selection );
  DisplayTime();
  DisplayTemp();
  DisplayFanSpeed();
  if ( MM_Selection == 1 ){
    if ( MM_Last_Selection != 1 ){ // if switching to temp set, set the encoder value to current temp setting
      Enc.write( target_temperature );
      MM_Last_Selection = MM_Selection;
      lcd.clear();
    }
    CheckForSelection( &MM_Selection, ( MM_Selection * -1 ) );

    newTarget = Enc.read();
    if ( newTarget != target_temperature ) {
      cli();
      if ( newTarget > MAX_TEMP ) {
        newTarget = 1;
        Enc.write( newTarget );
      }
      if ( newTarget < 0 ) {
        newTarget = MAX_TEMP;
        Enc.write( newTarget );
      }
      target_temperature = newTarget;
      DisplayTemp();
      sei();
    }
  }
  else {
    //    Serial.println(MM_Selection);
    if ( MM_Last_Selection == 1 ){ //if switching to fan speed, set the encoder value to the current set speed
      Enc.write( FanSpeed );
      MM_Last_Selection = MM_Selection;
      lcd.clear();
    }
    CheckForSelection( &MM_Selection, ( MM_Selection * -1 ) );
    NewFanSpeed = Enc.read();
    if ( NewFanSpeed != FanSpeed ) {
      cli();
      if ( NewFanSpeed > MAX_FAN_SPEED ) {
        NewFanSpeed = MAX_FAN_SPEED;
      }
      if ( NewFanSpeed <= MIN_FAN_SPEED ) {
        NewFanSpeed = MIN_FAN_SPEED;
      }
      Enc.write( NewFanSpeed );
      FanSpeed = NewFanSpeed;
    }
    DisplayFanSpeed();
    sei();

  }
}
// **************************************************************************



// ***************************** mode_select ****************************
void mode_select(){//*****************************  MENU  *****************************
  if (CurrentlyDisplayedItem != LastDisplayedItem) { //if the menu item displayed is different than last time...
    lastEnc = Enc.read();
    Enc.write(CurrentlyDisplayedItem);
  }
  check_button_state();
  switch (TM_Selection) {
  case 0:
    CurrentlyDisplayedItem = ( ( abs( Enc.read() ) / 4 ) %4 ); // abs function to deal with negitive encoder values
    switch (CurrentlyDisplayedItem) {

    case 0://                       MANUAL MODE
      lcd_display_once( "Mode: MANUAL" );
      CheckForSelection( &TM_Selection, 1 );
      break;

    case 1://                       PROGRAM MODE
      lcd_display_once( "Mode: PROGRAM" );
      CheckForSelection( &TM_Selection, 2 );
      break;

    case 2://                       REMOTE MODE
      lcd_display_once( "Mode: REMOTE" );
      CheckForSelection( &TM_Selection, 3 );
      break;

    case 3://                       SETTINGS MODE
      lcd_display_once( "Mode: SETTINGS" );
      CheckForSelection( &TM_Selection, 4 );
      break;
    }
    break;

  case 1:  //******************************  MANUAL MODE  ******************************
    manual_mode();
    break;

  case 2:  //*****************************  PROGRAM MODE  ******************************
    program_mode();
    break;

  case 3:  //******************************  REMOTE MODE  ******************************
    remote_mode();
    break;

  case 4:  //*****************************  SETTINGS MODE  *****************************
    settings_mode();
    break;

  }
}
// **************************************************************************



