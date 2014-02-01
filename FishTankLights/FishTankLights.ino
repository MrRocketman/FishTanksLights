///////////////////////////////////////////////////////////////////
// Current Satellite LED+ Controller  V4.0                       //
//   Indychus...Dahammer...mistergreen @ plantedtank.net         //
//   This code is public domain.  Pass it on.                    //
//   Confirmed on Arduino UNO 1.0.5                              //
//   Req. Time, TimeAlarms, RTClib, IRremote                     //
///////////////////////////////////////////////////////////////////
//
// This version uses Ken Shirriff's IRremote library to Rx/Tx the IR codes
// http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
//
// This code does NOT use PIN 13 on the Uno, as do previous versions
// Instead PIN 3, which is a PWM pin, is used. So you'll need to connect
// your LED to PIN 3 instead of PIN 13 for it to work.
//
// You can test the IR commands via the Arduino software's serial monitor
// by sending in a value from 1 - 32. Values follow the remote control,
// left to right, top to bottom (e.g 1 = Orange, 2 = Blue, 21 = Moon1, etc)
//
// Install LCD per instructions at http://learn.adafruit.com/character-lcds/overview
//
#include <Wire.h>
#include <RTClib.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <IRremote.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>


// Pin definitions
#define LCD_RS 7
#define LCD_ENABLE 8
#define LCD_DB4 9
#define LCD_DB5 10
#define LCD_DB6 11
#define LCD_DB7 12

#define ENCODER_A 2
#define ENCODER_B 4
#define ENCODER_CLICK 5

#define BEEPER 6

// LCD Encoder knob information
#define ENCODER_B_MASK (1 << 1)
#define ENCODER_A_MASK (1 << 0)

#define encrot0 0
#define encrot1 2
#define encrot2 3
#define encrot3 1

#define ENCODER_PULSES_PER_STEP 2

// LCD information
#define LCD_COLUMNS 20      // Number of columns on the LCD (e.g. 16, 20, etc)
#define LCD_ROWS 4       // Number of rows on the LCD (e.g. 2, 4, etc)

// FLASH String storage info
#define MAX_MSG_LEN 13  // Maximum length of the lightingMessage messages
#define LIGHTING_OPTIONS 32

// Macro to print a string stored in flash memory
#define PGMSTR(x) (__FlashStringHelper*)(x)


// Main object variables
RTC_DS1307 RTC;
IRsend irsend;
LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

byte randAnalogPin = 0;   // This needs to be set to an unused Analog pin, Used by RandomStorm()

// Encoder variables
volatile int encoderDifference = 0;
int encoderPosition = 0;
volatile uint8_t encoderBits = 0;
volatile uint8_t previousEncoderBits = 0;
bool encoderClickStatus = false;

// Function definitions
void setAlarms();
time_t syncProvider();
void scheduleRandomStorm();
int serialReadInt();
void processComputerCommands(int cmd);
void sendIRCode(int cmd, byte numTimes);
void printNumberToLCDWithLeadingZeros(int numberToPrint);
int availableRAM();
void encoderClicked();
void encoderUnClicked();
void readEncoder();

// Current Satellite+ IR Codes (NEC Protocol)
unsigned long codeHeader = 0x20DF; // Always the same
                                   // Remote buttons listed left to right, top to bottom
PROGMEM unsigned int lightCodes[LIGHTING_OPTIONS] = {
    0x3AC5,  // 1 -  Orange
    0xBA45,  // 2 -  Blue
    0x827D,  // 3 -  Rose
    0x02FD,  // 4 -  Power On/Off
    0x1AE5,  // 5 -  White
    0x9A65,  // 6 -  FullSpec
    0xA25D,  // 7 -  Purple
    0x22DD,  // 8 -  Play/Pause
    0x2AD5,  // 9 -  Red Up
    0xAA55,  // 10 - Green Up
    0x926D,  // 11 - Blue Up
    0x12ED,  // 12 - White Up
    0x0AF5,  // 13 - Red Down
    0x8A75,  // 14 - Green Down
    0xB24D,  // 15 - Blue Down
    0x32CD,  // 16 - White Down
    0x38C7,  // 17 - M1 Custom
    0xB847,  // 18 - M2 Custom
    0x7887,  // 19 - M3 Custom
    0xF807,  // 20 - M4 Custom
    0x18E7,  // 21 - Moon 1
    0x9867,  // 22 - Moon 2
    0x58A7,  // 23 - Moon 3
    0xD827,  // 24 - Dawn/Dusk
    0x28D7,  // 25 - Cloud 1
    0xA857,  // 26 - Cloud 2
    0x6897,  // 27 - Cloud 3
    0xE817,  // 28 - Cloud 4
    0x08F7,  // 29 - Storm 1
    0x8877,  // 30 - Storm 2
    0x48B7,  // 31 - Storm 3
    0xC837}; // 32 - Storm 4

// These are the messages that print on the serial monitor & lcd when each IR code is sent
prog_char PROGMEM lightingMessage[][MAX_MSG_LEN + 1] = {
    "Orange",
    "Blue",
    "Rose",
    "Power On/Off",
    "White",
    "Full Spectrum",
    "Purple",
    "Play/Pause",
    "Red Up",
    "Green Up",
    "Blue Up",
    "White Up",
    "Red Down",
    "Green Down",
    "Blue Down",
    "White Down",
    "Custom Mix 1",
    "Custom Mix 2",
    "Custom Mix 3",
    "Custom Mix 4",
    "Moonlight 1",
    "Moonlight 2",
    "Moonlight 3",
    "Dawn/Dusk",
    "Cloud Cover 1",
    "Cloud Cover 2",
    "Cloud Cover 3",
    "Cloud Cover 4",
    "Storm 1",
    "Storm 2",
    "Storm 3",
    "Storm 4"
};

void setup()
{
    Wire.begin();
    RTC.begin();
    Serial.begin(9600);
    lcd.begin(LCD_COLUMNS, LCD_ROWS);
    
    if(!RTC.isrunning())
    {
        // If no RTC is installed, set time to compile time at each reset
        Serial.println(F("RTC is NOT running!\n"));  // Store this string in PROGMEM
        RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    
    // This keeps the RTC library and the Time library in sync
    setSyncProvider(syncProvider);     // reference our syncProvider function instead of RTC_DS1307::get()
    
    // Print the time
    Serial.print(F("Time: "));
    Serial.print(hour(), 2);
    Serial.print(minute(), 2);
    Serial.println(second(), 2);
    
    setAlarms();  // Set up above alarms
    
    // Print available SRAM for debugging, comment out if you want
    Serial.print(F("SRAM: "));
    Serial.print(availableRAM());
    Serial.println(" bytes");
    Serial.println(F("To test IR codes, send 1 - 32\n"));
    
    // Setup the click wheel encoder pins
    pinMode(ENCODER_A,INPUT);
    pinMode(ENCODER_B,INPUT);
    pinMode(ENCODER_CLICK,INPUT);
    digitalWrite(ENCODER_A,HIGH);
    digitalWrite(ENCODER_B,HIGH);
    digitalWrite(ENCODER_CLICK,HIGH);
    
    attachInterrupt(0, readEncoder, CHANGE);
}

void loop()
{
    // Check for computer commands
    if (Serial.available() > 0)
    {
        processComputerCommands(serialReadInt());  // Go grab IR code and send it
    }
    
    // Update the alarms
    Alarm.delay(0);
    
    // Print the time HH:MM:SS
    lcd.setCursor(0,0);
    printNumberToLCDWithLeadingZeros(hourFormat12());
    lcd.print(":");
    printNumberToLCDWithLeadingZeros(minute());
    lcd.print(":");
    printNumberToLCDWithLeadingZeros(second());
    if(isAM())
    {
        lcd.print(F(" AM"));
    }
    else
    {
        lcd.print(F(" PM"));
    }
    
    // See if the encoder has moved the required amount
    if(abs(encoderDifference) >= ENCODER_PULSES_PER_STEP)
    {
        encoderPosition += encoderDifference / ENCODER_PULSES_PER_STEP;
        encoderDifference = 0;
        
        // Wrap around the limit for the main screen
        if(encoderPosition >= LIGHTING_OPTIONS)
        {
            encoderPosition = 0;
        }
        else if(encoderPosition < 0)
        {
            encoderPosition = LIGHTING_OPTIONS - 1;
        }
        
        Serial.println(encoderPosition);
        
        // Print the encoder position to the lcd for debugging
        lcd.setCursor(0,3);
        lcd.print(F("                    ")); // Hackishly clear the line
        lcd.setCursor(0,3);
        lcd.print(encoderPosition);
        
        // Print the light type
        lcd.print(F(" "));
        lcd.print(PGMSTR(lightingMessage[encoderPosition]));
    }
    
    // Check the encoder click button
    if(digitalRead(ENCODER_CLICK) == 0)
    {
        // Button transitions from not clicked to clicked
        if(encoderClickStatus == false)
        {
            encoderClicked();
        }
        
        encoderClickStatus = true;
        //Serial.println(F("Click "));
    }
    else if(digitalRead(ENCODER_CLICK == 1))
    {
        if(encoderClickStatus == true)
        {
            encoderUnClicked();
        }
        
        encoderClickStatus = false;
    }
}

void setAlarms()
{
    // Set up your desired alarms here
    // The default value of dtNBR_ALARMS is 6 in TimeAlarms.h.
    // This code sets 6 alarms by default
    // Changes the times to suit yourself. Add as many alarms as you like, just stay within dtNBR_ALARMS
    Alarm.alarmRepeat(7,00,0, DawnDusk);
    Alarm.alarmRepeat(9,00,0, Cloud2);     // (HR,MIN,SEC,FUNCTION)
    Alarm.alarmRepeat(10,00,0, FullSpec);
    Alarm.alarmRepeat(16,00,0, Cloud2);
    Alarm.alarmRepeat(19,00,0, DawnDusk);
    Alarm.alarmRepeat(21,00,0, Moon2);
    
    int theHour = hour();
    if(theHour >= 7 && theHour < 9)
    {
        DawnDusk();
    }
    else if(theHour >= 9 && theHour < 10)
    {
        Cloud2();
    }
    else if(theHour >= 10 && theHour < 16)
    {
        FullSpec();
    }
    else if(theHour >= 16 && theHour < 19)
    {
        Cloud2();
    }
    else if(theHour >= 19 && theHour < 21)
    {
        DawnDusk();
    }
    else if((theHour >= 21 && theHour <= 23) || (theHour >= 0 && theHour < 7))
    {
        Moon2();
    }
    
    // Comment these out if you don't want the chance of a random storm each day
    Alarm.alarmRepeat(12,00,00, scheduleRandomStorm);
    scheduleRandomStorm();  // Sets up intial storm so we don't have wait until alarm time
}

time_t syncProvider()
{
    //this does the same thing as RTC_DS1307::get()
    return RTC.now().unixtime();
}

void scheduleRandomStorm()
{
    // Schedules a storm between 1 & 8 in the evening
    // It sets Storm2, followed by Cloud2 or DawnDusk or Moon2, depending on when the storm is over
    randomSeed(analogRead(randAnalogPin));  // Generate random seed on unused pin
    byte RH = random(23);                   // Randomizer for RandomStorm
    byte RM = random(59);
    byte RS = random(59);
    byte TSDurationH = random(2);
    byte TSDurationM = random(59);
    
    lcd.setCursor(0,1);
    if (RH >= 13 && RH <= 21)
    { // If 1:00PM - 8:00PM schedule a storm
        Alarm.alarmOnce(RH,RM,RS,Storm2);
        
        Serial.print(F("Shcedule Storm: "));
        Serial.print(RH);
        Serial.print(":");
        Serial.print(RM);
        Serial.print(":");
        Serial.print(RS);
        Serial.print(F(" Duration: "));
        Serial.print(TSDurationH);
        Serial.print(":");
        Serial.print(TSDurationM);
        
        lcd.print(F("Storm Time: "));
        printNumberToLCDWithLeadingZeros(hourFormat12(RH));
        lcd.print(":");
        printNumberToLCDWithLeadingZeros(RM);
        if(isAM())
        {
            lcd.print(F(" AM"));
        }
        else
        {
            lcd.print(F(" PM"));
        }
        
        if ((RH + TSDurationH) < 19)   // Switch to Cloud2 if storm ends between 1:00-6:59pm
        {
            Alarm.alarmOnce((RH + TSDurationH),(RM + TSDurationM),RS,Cloud2);
        }
        else if ((RH + TSDurationH) < 21)  // Switch to DawnDusk if storm ends between 7:00-8:59pm
        {
            Alarm.alarmOnce((RH + TSDurationH),(RM + TSDurationM),RS,DawnDusk);
        }
        else                               // Otherwise, switch to Night2
        {
            Alarm.alarmOnce((RH + TSDurationH),(RM + TSDurationM),RS,Moon2);
        }
    }
    else
    {
        // Don't really need this, but it can stay till we need the space
        Serial.println(F("No storm today"));
        lcd.print(F("No storm today"));
    }
}

int serialReadInt()
{
    // Reads first 2 bytes from serial monitor; anything more is tossed
    byte i;
    char inBytes[3];
    char * inBytesPtr = &inBytes[0];  // Pointer to first element
    
    for (i=0; i<2; i++)             // Only want first 2 bytes
        inBytes[i] = Serial.read();
    inBytes[i] =  '\0';             // Put NULL character at the end
    while (Serial.read() >= 0)      // If anything else is there, throw it away
        ; // do nothing
    return atoi(inBytesPtr);        // Convert to decimal
}

void processComputerCommands(int cmd)
{
    // Handles commands sent in from the serial monitor
    if (cmd >= 1 && cmd <= 32)
    {
        // cmd must be 1 - 32
        sendIRCode(cmd-1, 1);
    }
    else
    {
        Serial.println(F("Invalid Choice"));
    }
}

void sendIRCode(int cmd, byte numTimes)
{
    // cmd = the element of the arrCode[] array that holds the IR code to be sent
  // numTimes = number of times to emmit the command
  // Shift header 16 bits to left, fetch code from PROGMEM & add it
    unsigned long irCode = (codeHeader << 16) + pgm_read_word_near(lightCodes + cmd);
    for( byte i = 0; i < numTimes; i++)
    {
        irsend.sendNEC(irCode,32); // Send/emmit code
        delay(100);
    }
    // Print the string associated with the IR code & the time
    Serial.print(PGMSTR(lightingMessage[cmd]));
    Serial.print(": ");
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.println(second());
    
    lcd.setCursor(0,2);
    lcd.print(F("                    ")); // Hackishly clear the line
    lcd.setCursor(0,2);
    lcd.print(PGMSTR(lightingMessage[cmd]));
}

void printNumberToLCDWithLeadingZeros(int numberToPrint)
{
    // Utility function for digital clock display: prints leading 0's
    if(numberToPrint < 10)
        lcd.print("0");
    lcd.print(numberToPrint);
}

int availableRAM()
{
    // Returns available SRAM
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void encoderClicked()
{
    // Beep
    analogWrite(BEEPER, 128);
    delay(100);
    analogWrite(BEEPER, 0);
    //tone(BEEPER, 500, 10);
    
    // Send the appropriate ir code
    if(encoderPosition == 3 || encoderPosition == 7 || encoderPosition == 8 || encoderPosition == 9 || encoderPosition == 10 || encoderPosition == 11 || encoderPosition == 12 || encoderPosition == 13 || encoderPosition == 14 || encoderPosition == 15)
    {
        sendIRCode(encoderPosition, 1);
    }
    else
    {
        sendIRCode(encoderPosition, 2);
    }
}

void encoderUnClicked()
{
    // Do something here
}

void readEncoder()
{
    // Read the rotary encoder
    if(digitalRead(ENCODER_A) == 0)
    {
        encoderBits |= ENCODER_A_MASK;
    }
    if(digitalRead(ENCODER_B) == 0)
    {
        encoderBits |= ENCODER_B_MASK;
    }
    // Determine the encoder rotation change
    if(encoderBits != previousEncoderBits)
    {
        switch(encoderBits)
        {
            case encrot0:
                //Serial.println(F("rotation0"));
                if(previousEncoderBits == encrot3)
                    encoderDifference ++;
                else if(previousEncoderBits == encrot1)
                    encoderDifference --;
                break;
            case encrot1:
                //Serial.println(F("rotation1"));
                if(previousEncoderBits == encrot0)
                    encoderDifference ++;
                else if(previousEncoderBits == encrot2)
                    encoderDifference --;
                break;
            case encrot2:
                //Serial.println(F("rotation2"));
                if(previousEncoderBits == encrot1)
                    encoderDifference ++;
                else if(previousEncoderBits == encrot3)
                    encoderDifference --;
                break;
            case encrot3:
                //Serial.println(F("rotation3"));
                if(previousEncoderBits == encrot2)
                    encoderDifference ++;
                else if(previousEncoderBits == encrot0)
                    encoderDifference --;
                break;
        }
    }
    previousEncoderBits = encoderBits;
    encoderBits = 0;
}

// IR Code functions, called by alarms
void Orange() {sendIRCode(0,2);}
void Blue() {sendIRCode(1,2);}
void Rose() {sendIRCode(2,2);}
void PowerOnOff() {sendIRCode(3,1);}
void White() {sendIRCode(4,2);}
void FullSpec() {sendIRCode(5,2);}
void Purple() {sendIRCode(6,2);}
void Play() {sendIRCode(7,1);}
void RedUp() {sendIRCode(8,1);}
void GreenUp() {sendIRCode(9,1);}
void BlueUp() {sendIRCode(10,1);}
void WhiteUp() {sendIRCode(11,1);}
void RedDown() {sendIRCode(12,1);}
void GreenDown() {sendIRCode(13,1);}
void BlueDown() {sendIRCode(14,1);}
void WhiteDown() {sendIRCode(15,1);}
void M1Custom() {sendIRCode(16,2);}
void M2Custom() {sendIRCode(17,2);}
void M3Custom() {sendIRCode(18,2);}
void M4Custom() {sendIRCode(19,2);}
void Moon1() {sendIRCode(20,2);}
void Moon2() {sendIRCode(21,2);}
void Moon3() {sendIRCode(22,2);}
void DawnDusk() {sendIRCode(23,2);}
void Cloud1() {sendIRCode(24,2);}
void Cloud2() {sendIRCode(25,2);}
void Cloud3() {sendIRCode(26,2);}
void Cloud4() {sendIRCode(27,2);}
void Storm1() {sendIRCode(28,2);}
void Storm2() {sendIRCode(29,2);}
void Storm3() {sendIRCode(30,2);}
void Storm4() {sendIRCode(31,2);}