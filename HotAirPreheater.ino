<<<<<<< HEAD
/********************************************************************************************************
 *                                                                                                       *
 *                                        Hot Air Preheater                                              *
 *                                                                                                       *
 *********************************************************************************************************
 *                                                                                                       *
 * ****************************************** SOFTWARE ************************************************* *
 *  Software is based off of the "Reflowduino" sketch found on adafruit's github                         *
 *  https://github.com/adafruit/Reflowduino written by PaintYourDragon.                                  *
 *                                                                                                       *
 *  Some portions were also taken from Scott Dixon's hotplate sketch                                     *
 *  http://dorkbotpdx.org/blog/scott_d/temperature_controller_board_final_design                         *
 *                                                                                                       *
 *  The Encoder library is written by Paul Stoffregen <paul@pjrc.com>                                    *
 *  http://www.pjrc.com/teensy/td_libs_Encoder.html                                                      *
 *  I chose this library because it allows use of 0,1 or 2 interupt pins for maximum flexability         *
 *                                                                                                       *
 *  The MAX6675 library is also taken from adafruit's github                                             *
 *  https://github.com/adafruit/MAX6675-library                                                          *
 *                                                                                                       *
 *  The RESET function was taken directly from the following website:                                    *
 *  http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1222941939/12                                        *
 *                                                                                                       *
 *  The rest are standard Arduino 1.0 libraries.                                                         *
 * ***************************************************************************************************** *
 *                                       **** HARDWARE ****                                              *
 * ***************************************************************************************************** *
 *  Rev 0.01                                                                                             *
 *  The hardware is currently a kludge of Scott Dixon's hotplate PCB with a second MAX6675 chip          *
 *  stacked on top of the original, with pins 3 & 6 bent out and wired separately.  Pin 3 is             *
 *  the input for the second TC and Pin 6 is the CS and wired to DP4 (originally used for IR             *
 *  thermometer).                                                                                        *
 *********************************************************************************************************/

#include <LiquidCrystal.h>
#include <max6675.h>
#include <Wire.h>
#include <Encoder.h>
<<<<<<< HEAD
#include <EEPROM.h>
#include <avr/wdt.h>           //watchdog timer needed for the RESET function

#define BUTTON 18              // Encoder's push button is on Digital Pin 18
#define BUTTON_PRESS_DETECT_TIME 150  // Have to press the button for a minimum of 150mS for it to detect it as pressed
#define RESET_TIME 2000        // If the button is pressed for 5seconds, it will trigger a reset

#define SSRPIN 13              // The pin we use to control the SSR
#define FANPIN 3               // The pin we use to control the fan speed

// The SPI pins we use for the TC sensors
#define SPI_CLK   17
#define SPI_DATA  16

// The SPI Chip Select (CS) pins
#define CHIP_CS   15                         // CS for the chipTC
#define AIR_CS    4                          // CS for the airTC

// PID pins
#define Kp        10                         // the Proportional control constant
#define Ki        0.5                        // the Integral control constant
#define Kd        100                        // the Derivative control constant 

#define WINDUPPERCENT 0.05                   // Windup error prevention, 5% by default

// Classic 16x2 LCD used
#define D4 7
#define D5 8
#define D6 9
#define D7 10
#define RW 11
#define E 6
#define RS 12
#define BL 5                                 // Backlight PWM control

LiquidCrystal lcd(RS, RW, E, D4, D5, D6, D7);

#define UPPERLEFT 0,0
#define BOTTOMLEFT 0,1
#define UPPERRIGHT 8,0
#define BOTTOMRIGHT 8,1


<<<<<<< HEAD
// defines for menu options
#define Top             0
#define Settings        1
#define Fan             2
#define Temp            3
#define TempDefault     30
#define TempMax         31
#define Program         4
#define ProgramProfile  40
#define SerialSpeed     5
#define MAX_FAN_SPEED   75
#define ManualMenu      6
#define ProfileMenu     400


// defines for EEPROM storage
#define TEMP_DEF_ADDRESS 0
#define TEMP_MAX_ADDRESS 1

LiquidCrystal lcd(RS, RW, E, D4, D5, D6, D7);
Encoder Enc(19,2);
#define BUTTON 18


//Setup the TCs
#define MAX_TEMP 999
#define DEFAULT_TEMP 150
MAX6675 airTC(SPI_CLK, AIR_CS, SPI_DATA);    //temp directly out of the heatgun
MAX6675 chipTC(SPI_CLK, CHIP_CS, SPI_DATA);  //temp measured at the chip/board




// volatile means it is going to be messed with inside an interrupt 
// otherwise the optimization code will ignore the interrupt
volatile long  seconds_time = 0;            // this will get incremented once a second
volatile float airTemp;                     // in celsius
volatile float chipTemp;                    // in celsius
volatile float previous_temperature;        // the last reading (1 second ago)

<<<<<<< HEAD
int target_temperature;                // the target temperature for the air
unsigned int set_temperature;          // the target temperature for the chip/board
int newTargetTemp = 0;

// we need this to be a global variable because we add error each second
float Summation;                            // The integral of error since time = 0

int relay_state;                            // whether the relay pin is high (on) or low (off)
int menu = 0;
int lastMenu = 1;
<<<<<<< HEAD
int topMenuOption = 0;
int lastTopMenuOption = 1;
int TM_Enc = 0;        // Top Menu      Encoder count storage
int MM_ENC = 0;        // Manual Mode   Encoder count storage
int PM_Enc = 0;        // Program Mode  Encoder count storage
int RM_Enc = 0;        // Remote Mode   Encoder count storage
int SM_Enc = 0;        // Settings Mode Encoder count storage
int SettingsMenu = 0;
int Settings_FanMenu = 0;
int Settings_TempMenu = 0;
int Temp_Default = 0;
int ProgramMenu = 0;
;
long Program_ProfileMenu = 10;
int SerialSpeedMenu = 0;
int SerialSpeed_BaudRate = 9600;

int newTargetFanSpeed = 0;;
int TargetFanSpeed = 0;
int FanSpeed = 0;


volatile  long buttonTime = 0;         // how long the encoder button has been pressed
int  lastButtonState = LOW;  // if the button is pressed or not
int  buttonState = HIGH;     // current button state
//int  menuSelection = 1;

// Soft reset code
#define soft_reset() do { wdt_enable(WDTO_15MS);  for(;;){} } while(0)  // code to enable the soft reset by calling the watchdog timer then having it time-out
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3"))); // needed to recover after calling the soft_reset


// *****************************************************************
//         *************** SETUP *****************************
// *****************************************************************

void setup() {  
  Serial.begin(SerialSpeed_BaudRate); 
  Serial.println("HotAir Preheater");

  // The data header (we have a bunch of data to track)
<<<<<<< HEAD
  Serial.print("Time (s)\buttonState\t topMenu\tbuttonTime\ttopMenuOption\tairTemp\ttargetTemp");
  /*
  // Now that we are mucking with stuff, we should track our variables
   Serial.print("\t\tKp = "); 
   Serial.print(Kp);
   Serial.print(" Ki = "); 
   Serial.print(Ki);
   Serial.print(" Kd = "); 
   Serial.println(Kd);
   */

  // the relay pin controls the heater
  pinMode(SSRPIN, OUTPUT);
  // ...and turn it off to start!
  pinMode(SSRPIN, LOW);

  // set the encoder button as input
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);

  // Set up 16x2 standard LCD  
  lcd.begin(16,2);

  // clear the screen and print out the current version
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("HotAir Preheater");
  lcd.setCursor(0,1);
  // compile date
  lcd.print(__DATE__);
  // pause for dramatic effect!
  delay(750);
  lcd.clear();


  // where we want to be
  target_temperature = DEFAULT_TEMP;  // degrees C default starting temp
  Enc.write(target_temperature); //set the encoder default value

<<<<<<< HEAD
    Summation = 0;                      // set the integral to 0

    // Setup 1 Hz timer to refresh display using 16 Timer 1
  TCCR1A = 0;                                     // CTC mode (interrupt after timer reaches OCR1A)
  TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS12);    // CTC & clock div 1024
  OCR1A  = 15609;                                 // 16mhz / 1024 / 15609 = 1 Hz
  TIMSK1 = _BV(OCIE1A);                          // turn on interrupt

}

// *****************************************************************
//     ********************* LOOP *****************************
// *****************************************************************

void loop() { 

  // we moved the LCD code into the interrupt so we don't have to worry about updating the LCD 
  // or reading from the airTC in the main loop

    float MV; // Manipulated Variable (ie. whether to turn on or off the relay!)
  float Error; // how off we are
  float Slope; // the change per second of the error
  Error = target_temperature - airTemp;
  Slope = previous_temperature - airTemp;
  // Summation is done in the interrupt
  // proportional-derivative controller only
  MV = Kp * Error + Ki * Summation + Kd * Slope;
  // Since we just have a relay, we'll decide 1.0 is 'relay on' and less than 1.0 is 'relay off'
  // this is an arbitrary number, we could pick 100 and just multiply the controller values
  if (MV >= 1.0) {
    relay_state = HIGH;
    digitalWrite(SSRPIN, HIGH);
  } 
  else {
    relay_state = LOW;
    digitalWrite(SSRPIN, LOW);
  }

  check_button_state();
<<<<<<< HEAD
  top_menu();
}

// *****************************************************************
//   ********************* FUNCTIONS *****************************
// *****************************************************************

// **************************** INTERRUPT ************************
// This is the Timer 1 CTC interrupt, it goes off once a second
SIGNAL(TIMER1_COMPA_vect) { 

  // time moves forward!
  if (topMenu == lastMenu) {
    seconds_time++;
    lastMenu = topMenu;
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
  Serial.print(topMenu);
  Serial.print("\t");
  Serial.print(buttonTime);
  Serial.print("\t");
  Serial.print(topMenuOption);
  Serial.print("\t");
  //  Serial.print();
  Serial.print(airTemp);
  Serial.print("\t");
  Serial.print(target_temperature);
  Serial.print("\t");
  Serial.println(relay_state);
} 


// **************************** CHECK BUTTON STATE ************************
void check_button_state(){
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


// ***************************** TOP MENU ****************************
void top_menu(){
  if (topMenuOption != lastTopMenuOption) {
    TM_Enc = Enc.read();
    Enc.write(topMenuOption);
  }
  check_button_state();
  switch (topMenu) {
  case 0: //------------------------------------------------DISPLAY THE LIST OF AVAILABLE MODES
    topMenuOption = ((Enc.read() / 4) % 4);
    switch (topMenuOption) {
    case 0://                       MANUAL MODE
      lcd_menu_display_once( "Mode: MANUAL" );
      menuSelection( Top, 1, TM_Enc );
      break;

    case 1://                       PROGRAM MODE
      lcd_menu_display_once( "Mode: PROGRAM" );
      menuSelection( Top, 2, PM_Enc );
      break;

    case 2://                       REMOTE MODE
      lcd_menu_display_once( "Mode: REMOTE" );
      menuSelection( Top, 3, RM_Enc );
      break;
    case 3://                       SETTINGS MODE
      lcd_menu_display_once( "Mode: SETTINGS" );
      menuSelection( Top, 4, SM_Enc );
      break;
    }
    break;

  case 1:  //******************************  MANUAL MODE  ******************************
    MM_ENC = Enc.read();
    manual_mode();
    break;

  case 2:  //*****************************  PROGRAM MODE  ******************************
    PM_Enc = Enc.read();
    program_mode();
    break;

  case 3:  //******************************  REMOTE MODE  ******************************
    RM_Enc = Enc.read();
    remote_mode();
    break;

  case 4:  //*****************************  SETTINGS MODE  *****************************
    SM_Enc = Enc.read();
    settings_mode();
    break;

  }
}

void menuSelection( int MenuLevel, int MenuNum, int EncVal ){ // **************** menu selection ******************
  // MenuNum = number of the switch/case selection you are choosi
  // EncVal  = Value you want to write to the encoder
  if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_DETECT_TIME) ) {
    lcd.clear();
    Enc.write(EncVal);
    switch (MenuLevel) {
    case Top:
      topMenu = MenuNum;
      break;

    case Settings:
      SettingsMenu = MenuNum;
      break;

    case Fan:
      Settings_FanMenu = MenuNum;
      break;

    case Temp:
      Settings_TempMenu = MenuNum;
      break;

    case TempDefault:
      Temp_Default = EncVal;
      EEPROM.write(TEMP_DEF_ADDRESS, EncVal);
      break;

    case TempMax:
      Temp_Default = EncVal;
      EEPROM.write(TEMP_MAX_ADDRESS, EncVal);
      break;

    case Program:
      ProgramMenu = MenuNum;
      break;

    case ProfileMenu:
       Program_ProfileMenu = MenuNum;
       break;

    case SerialSpeed:
      SerialSpeedMenu = MenuNum;
      break;

    case 9600:
      SerialSpeed_BaudRate = 9600;
      break;

    case 57600:
      SerialSpeed_BaudRate = 57600;
      break;

    case 115200:
      SerialSpeed_BaudRate = 115200;
      break;

    case ManualMenu:
      if (MenuNum = 1) {
        newTargetTemp = Enc.read();
        if (newTargetTemp != target_temperature) {
          cli(); //disable interrupts to prevent our target temp from being shown elsewhere on the screen
          if (newTargetTemp > MAX_TEMP) {
            newTargetTemp = 1;
            Enc.write(newTargetTemp);
          }
          if (newTargetTemp <= 0) {
            newTargetTemp = MAX_TEMP;
            Enc.write(newTargetTemp);
          }
          target_temperature = newTargetTemp;
          update_temp();
          sei(); //reenable interrupts
        }
      }
      else {
        newTargetFanSpeed = Enc.read();
        if (newTargetFanSpeed != TargetFanSpeed) {
          cli(); //disable interrupts to prevent our target temp from being shown elsewhere on the screen
          if (newTargetFanSpeed > MAX_FAN_SPEED) {
            newTargetFanSpeed = 1;
            Enc.write(newTargetFanSpeed);
          }
          if (newTargetFanSpeed <= 0) {
            newTargetFanSpeed = MAX_FAN_SPEED;
            Enc.write(newTargetFanSpeed);
          }
          TargetFanSpeed = newTargetFanSpeed;
          update_fanspeed();
          sei(); //reenable interrupts
        }
      }

    }

  }
}

void lcd_menu_display_once( String text ){ // ******************** lcd menu display once ************************
  if (topMenuOption != lastTopMenuOption) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print( text );
    lastTopMenuOption = topMenuOption;
  }
}


void manual_mode() {//******************************  MANUAL MODE  ******************************
  if (topMenu != lastMenu) {
    Enc.write(DEFAULT_TEMP);
  }
  check_button_state();
  update_time();
  update_temp();

  newTargetTemp = Enc.read();
  if (newTargetTemp != target_temperature) {
    cli(); //disable interrupts to prevent our target temp from being shown elsewhere on the screen
    if (newTargetTemp > MAX_TEMP) {
      newTargetTemp = 1;
      Enc.write(newTargetTemp);
    }
    if (newTargetTemp <= 0) {
      newTargetTemp = MAX_TEMP;
      Enc.write(newTargetTemp);
    }
    target_temperature = newTargetTemp;
    update_temp();
    sei(); //reenable interrupts
  }
}

void program_mode() {//*****************************  PROGRAM MODE  ******************************
  if (topMenu != lastMenu) {
    lastMenu = topMenu;        
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PROGRAM mode placeholder");
  }
  lcd.display();
  delay(750);
  lcd.noDisplay();
  delay(750);
  //insert code here!
}

void remote_mode() {//******************************  REMOTE MODE  ******************************
  if (topMenu != lastMenu) {
    lastMenu = topMenu;
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
  if (topMenu != lastMenu) {
    lastMenu = topMenu;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SETTINGS mode   placeholder");
  }
  lcd.display();
  delay(750);
  lcd.noDisplay();
  delay(750);
  //insert code here!
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


// ***************************** update_fanspeed ****************************
void update_fanspeed(){
  // go to line #1
  lcd.setCursor(0,1);
  lcd.print(airTemp);
  lcd.setCursor(8,1);
  lcd.print(FanSpeed);
  lcd.print("%");
}


// **************************** WDT_INT (RESET) ************************
void wdt_init(void) // to disable the watchdog timer after a soft reset
{
  MCUSR = 0;
  wdt_disable();

  return;
}











