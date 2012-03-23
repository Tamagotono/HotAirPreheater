


// *****************************************************************
//   ********************* FUNCTIONS *****************************
// *****************************************************************



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

//******************************  MANUAL MODE  ******************************
void manual_mode() {
  if ( TM_Selection != lastMenu ) {
    Enc.write( target_temperature );
  }
  //  check_button_state();
  CheckForSelection( &MM_Selection, ( MM_Selection * -1 ) );
  //  CheckForSelection( &MM_Selection, 0 );
  Serial.println( MM_Selection );
  DisplayTime();
  DisplayTemp();
  DisplayFanSpeed();
  if ( MM_Selection == 1 ){
    if ( MM_Last_Selection == 0 ){ // if switching to temp set, set the encoder value to current temp setting
      Enc.write( target_temperature );
      MM_Last_Selection = MM_Selection;
    }
//    Serial.println(MM_Selection);
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
    }
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
      DisplayFanSpeed();
      sei();
    }
  }
}
// **************************************************************************




//*****************************  PROGRAM MODE  ******************************
void program_mode() {
  if (TM_Selection != lastMenu) {
    lastMenu = TM_Selection;        
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PROGRAM mode    placeholder");
  }
  lcd.display();
  delay(750);
  lcd.noDisplay();
  delay(750);
  //insert code here!
}
// **************************************************************************




//******************************  REMOTE MODE  ******************************
void remote_mode() {
  if (TM_Selection != lastMenu) {
    lastMenu = TM_Selection;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("REMOTE mode     placeholder");
  }
  lcd.display();
  delay(750);
  lcd.noDisplay();
  delay(750);
  //insert code here!
}
// **************************************************************************




//*****************************  SETTINGS MODE  *****************************
void settings_mode() {
  if (TM_Selection != lastMenu) {
    lastMenu = TM_Selection;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SETTINGS mode     placeholder");
  }
  lcd.display();
}
// **************************************************************************





