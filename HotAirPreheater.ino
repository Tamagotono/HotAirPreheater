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
 
#include <LiquidCrystalFast.h>
#include <max6675.h>
#include <Wire.h>
#include <Encoder.h>
#include <avr/wdt.h> //watchdog timer needed for the RESET function

#define BUTTON 18            // Encoder's push button is on Digital Pin 18
#define BUTTON_PRESS_TIME 150
#define RESET_TIME 2000

#define SSRPIN 13            // The pin we use to control the SSR
#define FANPIN 3             // The pin we use to control the fan speed

// The SPI pins we use for the TC sensors
#define SPI_CLK 17
#define SPI_DATA 16
// The SPI Chip Select (CS) pins
#define CHIP_CS 15           // CS for the chipTC
#define AIR_CS 4             // CS for the airTC

// PID pins
#define Kp  10               // the Proportional control constant
#define Ki  0.5              // the Integral control constant
#define Kd  100              // the Derivative control constant 

#define WINDUPPERCENT 0.05   // Windup error prevention, 5% by default

// Classic 16x2 LCD used
#define D4 7
#define D5 8
#define D6 9
#define D7 10
#define RW 11
#define E 6
#define RS 12
#define BL 5                 // Backlight PWM control

LiquidCrystalFast lcd(RS, RW, E, D4, D5, D6, D7);
Encoder Enc(19,2);

//Setup the TCs
#define MAX_TEMP 999
#define DEFAULT_TEMP 150
MAX6675 airTC(SPI_CLK, AIR_CS, SPI_DATA);   //temp directly out of the heatgun
MAX6675 chipTC(SPI_CLK, CHIP_CS, SPI_DATA); //temp measured at the chip/board

// volatile means it is going to be messed with inside an interrupt 
// otherwise the optimization code will ignore the interrupt
volatile long  seconds_time = 0;       // this will get incremented once a second
volatile float airTemp;                // in celsius
volatile float chipTemp;               // in celsius
volatile float previous_temperature;   // the last reading (1 second ago)

int target_temperature;                // the target temperature for the air
unsigned int set_temperature;          // the target temperature for the chip/board
int newTarget = 0;

// we need this to be a global variable because we add error each second
float Summation;                       // The integral of error since time = 0

int relay_state;                       // whether the relay pin is high (on) or low (off)
int topMenu = 0;
int lastMenu = 1;
int topMenuOption = 0;
int lastTopMenuOption = 1;
int TM_Enc = 0;        // Top Menu      Encoder count storage
int MM_ENC = 0;        // Manual Mode   Encoder count storage
int PM_Enc = 0;        // Program Mode  Encoder count storage
int RM_Enc = 0;        // Remote Mode   Encoder count storage
int SM_Enc = 0;        // Settings Mode Encoder count storage

volatile  long buttonTime = 0;         // how long the encoder button has been pressed
          int  lastButtonState = LOW;  // if the button is pressed or not
          int  buttonState = HIGH;     // current button state
          int  menuSelection = 1;

// Soft reset code
#define soft_reset() do { wdt_enable(WDTO_15MS);  for(;;){} } while(0)          // code to enable the soft reset by calling the watchdog timer then having it time-out
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));  // needed to recover after calling the soft_reset


// *****************************************************************
//         *************** SETUP *****************************
// *****************************************************************

void setup() {  
  Serial.begin(9600); 
  Serial.println("HotAir Preheater");

  // The data header (we have a bunch of data to track)
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

  pinMode(SSRPIN, OUTPUT);          // the relay pin controls the heater
  pinMode(SSRPIN, LOW);             // ...and turn it off to start!

  pinMode(BUTTON, INPUT);           // set the encoder button as input
  digitalWrite(BUTTON, HIGH);       // enable the internal pullup resistor

  lcd.begin(16,2);                  // Set up 16x2 standard LCD  

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

  target_temperature = DEFAULT_TEMP;  // degrees C default starting temp
  Enc.write(target_temperature);      //set the encoder default value

  Summation = 0;                      // set the integral to 0

  // Setup 1 Hz timer to refresh display using 16 Timer 1
  TCCR1A = 0;                                     // CTC mode (interrupt after timer reaches OCR1A)
  TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS12);    // CTC & clock div 1024
  OCR1A = 15609;                                  // 16mhz / 1024 / 15609 = 1 Hz
  TIMSK1 = _BV(OCIE1A);                           // turn on interrupt

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
  //  top_topMenu();
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
      if (topMenuOption != lastTopMenuOption) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode: MANUAL");
        lastTopMenuOption = topMenuOption;
      }
      if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_TIME) ) {
        lcd.clear();
        topMenu = 1;
        Enc.write(TM_Enc);
      }
      break;


    case 1://                       PROGRAM MODE
      if (topMenuOption != lastTopMenuOption) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode: PROGRAM");
        lastTopMenuOption = topMenuOption;
      }
      if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_TIME) ) {
        lcd.clear();
        topMenu = 2;
        Enc.write(PM_Enc);
      }
      break;

    case 2://                       REMOTE MODE
      if (topMenuOption != lastTopMenuOption) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode: REMOTE");
        lastTopMenuOption = topMenuOption;
      }
      if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_TIME) ) {
        lcd.clear();
        topMenu = 3;
        Enc.write(RM_Enc);
      }
      break;
    case 3://                       SETTINGS MODE
      if (topMenuOption != lastTopMenuOption) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode: SETTINGS");
        lastTopMenuOption = topMenuOption;
      }
      if ( (buttonState == LOW) && ((millis() - buttonTime) > BUTTON_PRESS_TIME) ) {
        lcd.clear();
        topMenu = 3;
        Enc.write(SM_Enc);
      }
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

void manual_mode() {//******************************  MANUAL MODE  ******************************
  if (topMenu != lastMenu) {
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

void program_mode() {//*****************************  PROGRAM MODE  ******************************
  if (topMenu != lastMenu) {
    lastMenu = topMenu;        
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


// **************************** WDT_INT (RESET) ************************
void wdt_init(void) // to disable the watchdog timer after a soft reset
{
  MCUSR = 0;
  wdt_disable();

  return;
}







