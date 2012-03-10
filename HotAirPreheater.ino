/*
Hot Air Preheater

**** SOFTWARE ****
Software is based off of the "Reflowduino" sketch found on adafruit's github 
https://github.com/adafruit/Reflowduino written by PaintYourDragon.

Some portions were also taken from Scott Dixon's hotplate sketch 
http://dorkbotpdx.org/blog/scott_d/temperature_controller_board_final_design

The Encoder library is written by Paul Stoffregen <paul@pjrc.com>
http://www.pjrc.com/teensy/td_libs_Encoder.html
I chose this library because it allows use of 0,1 or 2 interupt pins for maximum flexability

The MAX6675 library is also taken from adafruit's github
https://github.com/adafruit/MAX6675-library

The rest are standard Arduino 1.0 librarys.
-----------------------------------------------------------------------------
**** HARDWARE ****
Rev 0.01
The hardware is currently a kludge of Scott Dixon's hotplate PCB with a second MAX6675 chip
stacked on top of the original, with pins 3 & 6 bent out and wired separately.  Pin 3 is
the input for the second TC and Pin 6 is the CS and wired to DP4.

*/
#include <LiquidCrystal.h>
#include <max6675.h>
#include <Wire.h>
#include <Encoder.h>

// The pin we use to control the relay
#define RELAYPIN 13

// The SPI pins we use for the thermocouple sensor
#define MAX_CLK 17
#define MAX_CS 15
#define MAX_DATA 16
#define MAX2_CS 4

// the Proportional control constant
#define Kp  10
// the Integral control constant
#define Ki  0.5
// the Derivative control constant 
#define Kd  100

// Windup error prevention, 5% by default
#define WINDUPPERCENT 0.05  

MAX6675 thermocouple(MAX_CLK, MAX2_CS, MAX_DATA);
MAX6675 thermocouple2(MAX_CLK, MAX_CS, MAX_DATA);

// Classic 16x2 LCD used
// LiquidCrystal display with:
// rs on pin 12
// rw is pin 11
// enable on pin 6
// backlight control on pin 5 (pwm 0=bright, 511=off)
// d4, d5, d6, d7 on pins 7, 8, 9, 10
// backlight pwm control is pin 5
#define D4 7
#define D5 8
#define D6 9
#define D7 10
#define RW 11
#define E 6
#define RS 12
#define BL 5

LiquidCrystal lcd(RS, RW, E, D4, D5, D6, D7);

Encoder myEnc(19,2);


// volatile means it is going to be messed with inside an interrupt 
// otherwise the optimization code will ignore the interrupt

volatile long seconds_time = 0;  // this will get incremented once a second
volatile float temp1;  // in celsius
volatile float temp2;  // in celsius
volatile float previous_temperature;  // the last reading (1 second ago)

// the current temperature
int target_temperature;

// we need this to be a global variable because we add error each second
float Summation;        // The integral of error since time = 0

int relay_state;       // whether the relay pin is high (on) or low (off)

void setup() {  
  Serial.begin(9600); 
  Serial.println("HotAir Preheater");
  
  // The data header (we have a bunch of data to track)
  Serial.print("Time (s)\tTemp (C)\tError\tSlope\tSummation\tPID Controller\tRelay");
 
   // Now that we are mucking with stuff, we should track our variables
  Serial.print("\t\tKp = "); Serial.print(Kp);
  Serial.print(" Ki = "); Serial.print(Ki);
  Serial.print(" Kd = "); Serial.println(Kd);
  
  // the relay pin controls the plate
  pinMode(RELAYPIN, OUTPUT);
  // ...and turn it off to start!
  pinMode(RELAYPIN, LOW);

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
  target_temperature = 100.0;  // 100 degrees C
  myEnc.write(target_temperature); //set the encoder default value
  
  // set the integral to 0
  Summation = 0;
  
  // Setup 1 Hz timer to refresh display using 16 Timer 1
  TCCR1A = 0;                           // CTC mode (interrupt after timer reaches OCR1A)
  TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS12);    // CTC & clock div 1024
  OCR1A = 15609;                                 // 16mhz / 1024 / 15609 = 1 Hz
  TIMSK1 = _BV(OCIE1A);                          // turn on interrupt
}

 
void loop() { 
  // we moved the LCD code into the interrupt so we don't have to worry about updating the LCD 
  // or reading from the thermocouple in the main loop

  float MV; // Manipulated Variable (ie. whether to turn on or off the relay!)
  float Error; // how off we are
  float Slope; // the change per second of the error
 
  
  Error = target_temperature - temp1;
  Slope = previous_temperature - temp1;
  // Summation is done in the interrupt
  
  // proportional-derivative controller only
  MV = Kp * Error + Ki * Summation + Kd * Slope;
  
  // Since we just have a relay, we'll decide 1.0 is 'relay on' and less than 1.0 is 'relay off'
  // this is an arbitrary number, we could pick 100 and just multiply the controller values
  
  if (MV >= 1.0) {
    relay_state = HIGH;
    digitalWrite(RELAYPIN, HIGH);
  } else {
    relay_state = LOW;
    digitalWrite(RELAYPIN, LOW);
  }


  int newTarget = myEnc.read();

  if (newTarget != target_temperature) {
    target_temperature = newTarget;
    lcd.setCursor(11,0);
    lcd.print(target_temperature);
  }
}


// This is the Timer 1 CTC interrupt, it goes off once a second
SIGNAL(TIMER1_COMPA_vect) { 
  
  // time moves forward!
  seconds_time++;

  // save the last reading for our slope calculation
  previous_temperature = temp1;

  // we will want to know the temperature in the main loop()
  // instead of constantly reading it, we'll just use this interrupt
  // to track it and save it once a second to 'temp1'
  temp1 = thermocouple.readCelsius();
  temp2 = thermocouple2.readCelsius();
//  temp1 = thermocouple.readFarenheit();
  
  // Sum the error over time
  Summation += target_temperature - temp1;
  
  if ( (temp1 < (target_temperature * (1.0 - WINDUPPERCENT))) ||
       (temp1 > (target_temperature * (1.0 + WINDUPPERCENT))) ) {
        // to avoid windup, we only integrate within 5%
         Summation = 0;
   }

  // display current time and temperature
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print(seconds_time);
  lcd.print(" s");
  lcd.setCursor(11,0);
  lcd.print(target_temperature);

  // go to line #1
  lcd.setCursor(0,1);
  lcd.print(temp1);
  lcd.setCursor(8,1);
  lcd.print(temp2);
#if ARDUINO >= 100
  lcd.write(0xDF);
#else
  lcd.print(0xDF, BYTE);
#endif
  lcd.print("C ");
  
  // print out a log so we can see whats up
  Serial.print(seconds_time);
  Serial.print("\t");
  Serial.print(temp1);
  Serial.print("\t");
  Serial.print(target_temperature);
  Serial.print("\t");
  Serial.print(target_temperature - temp1); // the Error!
  Serial.print("\t");
  Serial.print(previous_temperature - temp1); // the Slope of the Error
  Serial.print("\t");
  Serial.print(Summation); // the Integral of Error
  Serial.print("\t");
  Serial.print(Kp*(target_temperature - temp1) + Ki*Summation + Kd*(previous_temperature - temp1)); //  controller output
  Serial.print("\t");
  Serial.println(relay_state);
} 
