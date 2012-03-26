// *****************************************************************
//   ********************* FUNCTIONS *****************************
// *****************************************************************


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



// ***************** SET/RESTORE INITIAL EEPROM VALUES **********************
void Restore_EEPROM_Defaults() { 
  EEPROM.write(PROFILE1_STARTINGTEMP_ADDRESS_h, 0);
  EEPROM.write(PROFILE1_STARTINGTEMP_ADDRESS_l, 25);
  EEPROM.write(PROFILE1_TSMIN_TIME_ADDRESS_h,   0);
  EEPROM.write(PROFILE1_TSMIN_TIME_ADDRESS_l,   120);
  EEPROM.write(PROFILE1_TSMIN_TEMP_ADDRESS_h,   0);
  EEPROM.write(PROFILE1_TSMIN_TEMP_ADDRESS_l,   140);
  EEPROM.write(PROFILE1_TSMAS_TIME_ADDRESS_h,   0);
  EEPROM.write(PROFILE1_TSMAS_TIME_ADDRESS_l,   120);
  EEPROM.write(PROFILE1_TSMAX_TEMP_ADDRESS_h,   0);
  EEPROM.write(PROFILE1_TSMAX_TEMP_ADDRESS_l,   180);
  EEPROM.write(PROFILE1_TL_ADDRESS_h,           0);
  EEPROM.write(PROFILE1_TL_ADDRESS_l,           183);
  EEPROM.write(PROFILE1_TPSTART_ADDRESS_h,      0);
  EEPROM.write(PROFILE1_TPSTART_ADDRESS_l,      30);
  EEPROM.write(PROFILE1_TPEND_ADDRESS_h,        0);
  EEPROM.write(PROFILE1_TPEND_ADDRESS_l,        30);
  EEPROM.write(PROFILE1_RDRATE_ADDRESS_h,       0);
  EEPROM.write(PROFILE1_RDRATE_ADDRESS_l,       30);
}

