


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


void manual_mode() {//******************************  MANUAL MODE  ******************************
  if (TM_Selection != lastMenu) {
    Enc.write(DEFAULT_TEMP);
  }
  check_button_state();
  DisplayTime();
  DisplayTemp();
  DisplayFanSpeed();

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
    DisplayTemp();
    sei();
  }
}


void program_mode() {//*****************************  PROGRAM MODE  ******************************
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

void remote_mode() {//******************************  REMOTE MODE  ******************************
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

void settings_mode() {//*****************************  SETTINGS MODE  *****************************
  if (TM_Selection != lastMenu) {
    lastMenu = TM_Selection;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SETTINGS mode     placeholder");
  }
  lcd.display();
}


void DisplayFanSpeed(){
  lcd.setCursor(8,1);
  lcd.print("Fan: ");
  lcd.print( FanSpeed );
  lcd.print("%");
}

