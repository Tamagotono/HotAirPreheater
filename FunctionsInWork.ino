


// *****************************************************************
//   ********************* FUNCTIONS *****************************
// *****************************************************************



// ***************************** mode_select ****************************
void mode_select(){//*****************************  MENU  *****************************
  if (option != lastOption) {
    lastEnc = Enc.read();
    Enc.write(option);
  }
  check_button_state();
  switch (menu) {
  case 0:
    option = ((Enc.read() / 4) % 4);
    switch (option) {

    case 0://                       MANUAL MODE
      lcd_display_once( "Mode: MANUAL" );
      TopMenuSelect( 1 );
      break;

    case 1://                       PROGRAM MODE
      lcd_display_once( "Mode: PROGRAM" );
      TopMenuSelect( 2 );
      break;

    case 2://                       REMOTE MODE
      lcd_display_once( "Mode: REMOTE" );
      TopMenuSelect( 3 );
      break;

    case 3://                       SETTINGS MODE
      lcd_display_once( "Mode: SETTINGS" );
      TopMenuSelect( 4 );
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

void TopMenuSelect( int MenuItem ){ // *************** Top Menu Select *****************
  if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_TIME) ) {
    lcd.clear();
    menu = MenuItem;
    Enc.write(lastEnc);
  }
}


void lcd_display_once( String DisplayText ){// ********* LCD DISPLAY ONCE ************
  if (option != lastOption) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( DisplayText );
    lastOption = option;
  }
}


void program_mode() {//*****************************  PROGRAM MODE  ******************************
  if (menu != lastMenu) {
    lastMenu = menu;        
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
  if (menu != lastMenu) {
    lastMenu = menu;
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
  if (menu != lastMenu) {
    lastMenu = menu;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SETTINGS mode     placeholder");
  }
  lcd.display();
}


// ***************************** update_time ****************************
void update_time() {
  int minutes = (seconds_time / 60);
  int seconds = (seconds_time % 60);
  lcd.setCursor(0, 0);
  lcd.print("Time:");
  if ( minutes <= 9) { // add a leading space to the minute clock if less than 10 seconds
    lcd.print(" ");
  }    
  lcd.print( minutes ); // minutes
  lcd.print(":");
  if ( seconds <= 9) { // add a leading zero to the seconds clock if less than 10 seconds
    lcd.print("0");
  }
  lcd.print( seconds ); // seconds
  lcd.setCursor(13,0);
  lcd.print(target_temperature);
  lcd.print("  "); // get rid of any trailing digits 
}

