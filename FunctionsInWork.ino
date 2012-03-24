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
  EEPROM.write(PROFILE1_STARTINGTEMP_ADDRESS, 25);
  EEPROM.write(PROFILE1_TSMIN_TIME_ADDRESS,30);


}

