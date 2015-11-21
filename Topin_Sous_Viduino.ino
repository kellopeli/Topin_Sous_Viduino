//-------------------------------------------------------------------
//
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#include <Wire.h>

#include <Encoder.h>
Encoder myEnc(2, 3);

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#define LCD_BL 7

//#include <LiquidCrystal.h>
//enum ePins { LCD_RS=8, LCD_EN=9, LCD_D4=4, LCD_D5=5, LCD_D6=6, LCD_D7=7, LCD_BL=10 }; // define LCD pins
//LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // initialize the library with the numbers of the interface pins

//#include "DFRkeypad.h"

#define THRESHOLD 40
#define BUTTON_NONE 0
#define BUTTON_UP 0x08
#define BUTTON_DOWN 0x04
//#define BUTTON_LEFT 0x10
#define BUTTON_RIGHT 0x02
#define BUTTON_SELECT 0x01

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 8

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_GND 12
#define ONE_WIRE_BUS 11
#define ONE_WIRE_PWR 10

#define PUSHBUTTON 9

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define BUTTON_SHIFT BUTTON_SELECT

unsigned long lastInput = 0; // last button press

// Creat a set of new characters
const uint8_t charBitmap[][8] = {
   { 0xc, 0x12, 0x12, 0xc, 0, 0, 0, 0 },
   { 0x6, 0x9, 0x9, 0x6, 0, 0, 0, 0 },
   { 0x0, 0x6, 0x9, 0x9, 0x6, 0, 0, 0x0 },
   { 0x0, 0xc, 0x12, 0x12, 0xc, 0, 0, 0x0 },
   { 0x0, 0x0, 0xc, 0x12, 0x12, 0xc, 0, 0x0 },
   { 0x0, 0x0, 0x6, 0x9, 0x9, 0x6, 0, 0x0 },
   { 0x0, 0x0, 0x0, 0x6, 0x9, 0x9, 0x6, 0x0 },
   { 0x0, 0x0, 0x0, 0xc, 0x12, 0x12, 0xc, 0x0 }
   
};

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

long oldPosition = 0;


// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = RUN;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

void LoadParameters();
void StartAutoTune();
void FinishAutoTune();

uint8_t ReadButtons();
void SaveParameters();
void LoadParameters();
void EEPROM_writeDouble(int address, double value);
double EEPROM_readDouble(int address);

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);

   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   // Set up Ground & Power for the sensor from GPIO pins

   pinMode(ONE_WIRE_GND, OUTPUT);
   digitalWrite(ONE_WIRE_GND, LOW);

   pinMode(ONE_WIRE_PWR, OUTPUT);
   digitalWrite(ONE_WIRE_PWR, HIGH);

   // Initialize LCD DiSplay 
   pinMode(LCD_BL, OUTPUT);                        // pin LCD_BL is LCD backlight brightness (PWM)
   analogWrite(LCD_BL, 255);                       // set the PWM brightness to maximum

   lcd.begin(20, 4);
   
   int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));

   for ( int i = 0; i < charBitmapSize; i++ )
   {
      lcd.createChar ( i, (uint8_t *)charBitmap[i] );
   }
   
   lcd.home ();                   // go home
   lcd.print(F("    Adafruit"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Sous Vide!"));
   lcd.setCursor(0, 2);
   lcd.print(F("github/kellopeli/"));
   lcd.setCursor(0, 3);
   lcd.print(F("Topin_Sous_Viduino"));
   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 3);
      lcd.print(F("Sensor Error"));
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);

   delay(1000);  // Splash screen

   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;

  pinMode ( PUSHBUTTON, INPUT );
  digitalWrite ( PUSHBUTTON, HIGH );
  
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   // wait for button release before changing state
   while(ReadButtons() != 0) {}

   lcd.clear();

   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
   //analogWrite(LCD_BL, 150);                       // set the PWM brightness to maximum   }
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.home();
   lcd.print(F("    Adafruit"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Sous Vide! Off"));
   uint8_t buttons = 0;
   
   while(!(buttons & (BUTTON_RIGHT)))
   {
      buttons = ReadButtons();
   }
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading

   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// SHIFT for 10x tuning
// ************************************************
void Tune_Sp()
{
   lcd.print(F("Set Temperature:"));
   
   uint8_t buttons = 0;
   oldPosition = myEnc.read();

   while(true)
   {
      buttons = ReadButtons();

      float increment = 0.1;
      double steps = read_steps();

      Setpoint += increment * steps;

      if (buttons & BUTTON_RIGHT)
      {
         opState = TUNE_P;
         return;
      }
    
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
   lcd.print(F("Set Kp"));

   lcd.setCursor(0,3);
   lcd.print(F("Set Proportional"));
   lcd.setCursor(0,1);

   uint8_t buttons = 0;

   oldPosition = myEnc.read();

   while(true)
   {
      buttons = ReadButtons();
      long steps = read_steps();

      float increment = 1.0;

      Kp += increment * steps;

      if (buttons & BUTTON_RIGHT)
      {
         opState = TUNE_I;
         return;
      }

      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
   lcd.print(F("Set Ki"));
   lcd.setCursor(0,3);
   lcd.print(F("Set Integral"));
   lcd.setCursor(0,1);

   uint8_t buttons = 0;

   oldPosition = myEnc.read();
   
   while(true)
   {
      float increment = 0.01;
      long steps = read_steps();

      increment = 0.01 * steps;
      Ki += increment;
      
      buttons = ReadButtons();

      if (buttons & BUTTON_RIGHT)
      {
         opState = TUNE_D;
         return;
      }

      if ((millis() - lastInput) > 30000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}


// ************************************************
long read_steps()
{
  long newPosition = myEnc.read();
  long steps = newPosition - oldPosition;

  if (steps != 0) {
    delay(50);
    newPosition = myEnc.read();
    steps = newPosition - oldPosition;

    if (steps == 2) {
      steps = 1;
    }
    if (steps == -2) {
      steps = -1;
    }
    Serial.print(millis() - lastInput);
    Serial.print(" ");
    Serial.println(steps);

    lastInput = millis();
    oldPosition = newPosition;
  }
  
  return steps;
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
   lcd.print(F("Set Kd"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      float increment = 0.01;
      if (buttons & BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons & BUTTON_RIGHT)
      {
         opState = RUN;
         return;
      }
      if (buttons & BUTTON_UP)
      {
         Kd += increment;
         delay(200);
      }
      if (buttons & BUTTON_DOWN)
      {
         Kd -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// ************************************************
void Run()
{
   // set up the LCD's number of rows and columns: 
   lcd.print(F("Sp: "));
   lcd.print(Setpoint);
   lcd.write(1);
   lcd.print(F("C : "));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   uint8_t buttons = 0;
   while(true)
   {
      setBacklight();  // set backlight based on state

      buttons = ReadButtons();

      if ((buttons & BUTTON_SHIFT) 
         && (buttons & BUTTON_RIGHT) 
         && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
      {
         StartAutoTune();
      }
      else if (buttons & BUTTON_RIGHT)
      {
        opState = SETP;
        return;
      }
      
      DoControl();
      
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C : "));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
      lcd.setCursor(10,1);
      lcd.print(F("      "));
      lcd.setCursor(10,1);
      lcd.print(pct/10);
      //lcd.print(Output);
      lcd.print("%");

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        Serial.print(Input);
        Serial.print(",");
        Serial.println(Output);
        lastLogTime = millis();
      }

      delay(100);
   }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
   if (tuning)
   {
     analogWrite(LCD_BL, 255);                       // set the PWM brightness to maximum
   }
   else if (abs(Input - Setpoint) > 1.0)  
   {
     analogWrite(LCD_BL, 255);                       // set the PWM brightness to maximum
   }
   else if (abs(Input - Setpoint) > 0.2)  
   {
     analogWrite(LCD_BL, 255);                       // set the PWM brightness to maximum
   }
   else
   {
     analogWrite(LCD_BL, 255);                       // set the PWM brightness to maximum
   }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  int buttonState = digitalRead(PUSHBUTTON);
  uint8_t buttons = BUTTON_NONE;

  //Serial.print("Button: ");
  //Serial.println(buttonState);
  if (buttonState == LOW) {
    digitalWrite ( 13, HIGH );
    lastInput = millis();
    buttons = BUTTON_RIGHT;
  } else {
    digitalWrite ( 13, LOW );
  }

  return buttons;
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
