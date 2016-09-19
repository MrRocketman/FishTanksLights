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
// This uses pin 9 on a mega for the ir led, and pin 3 on an uno for the ir led
//
// You can test the IR commands via the Arduino software's serial monitor
// by sending in a value from 1 - 32. Values follow the remote control,
// left to right, top to bottom (e.g 1 = Orange, 2 = Blue, 21 = Moon1, etc)
//
// You'll need this type of LCD to make this work: http://www.hobbyking.com/hobbyking/store/__46899__3D_Printer_RepRap_Smart_Controller_Ramps_LCD_Control_.html
//

//**********************************************************************//
#pragma mark - Includes

#include <Wire.h>
#include <RTClib.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <IRremote.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

//**********************************************************************//
#pragma mark - Defines

/*
 * BOF preprocessor bug prevent
 * insert me on top of your arduino-code
 */
#define nop() __asm volatile ("nop")
#if 1
nop();
#endif
/*
 * EOF preprocessor bug prevent
 */

// 1 for degub mode
#define DEBUG 1

// Pin definitions
#define LCD_RS 7
#define LCD_ENABLE 8
#define LCD_DB4 3
#define LCD_DB5 10
#define LCD_DB6 11
#define LCD_DB7 12
#define LCD_MOSFET A12

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
#define LCD_BLANK_LINE ((LCD_COLUMNS == 20) ? "                    " : "                ")
#define LCD_ROWS 4       // Number of rows on the LCD (e.g. 2, 4, etc)

// Light Color Stuff
#define MAX_COLOR_STEPS 42

// FLASH String storage info
#define MAX_MSG_LEN 13  // Maximum length of the lightingMessage messages
#define LIGHTING_OPTIONS 32

// Macro to print a string stored in flash memory
#define PGMSTR(x) (__FlashStringHelper*)(x)

#define RESERVED_ALARMS 2

//**********************************************************************//
#pragma mark - Structs and Typedefs

// This is just to store the RGBW values for different colors
typedef struct {
  byte red;
  byte green;
  byte blue;
  byte white;
} LightColor;

// Alarm callback function typedef to make it easier to treat it as a return type
typedef void (*LightEffectFunction)();

// Alarm typedef to hold all the information we'll need for ramping and calling LightEffectFunctions
typedef struct {
  byte hour;
  byte minute;
  LightEffectFunction lightEffectFunction;
  LightColor lightColor;
  byte rampDuration;
  AlarmID_t alarmID;
} LCDAlarm;

//**********************************************************************//
#pragma mark - Function Declarations

// Alarm Functions
void initializeLCDAlarms();
void setCurrentLightingAccordingToSchedule();
LightEffectFunction getLightingForTime(byte hour, byte minute);
LCDAlarm getLCDAlarmForTime(byte hour, byte minute);
void addLCDAlarm(byte hour, byte minute, LightEffectFunction lightEffectFunction, byte rampDuration);
void setLCDAlarmAtIndex(byte index, byte hour, byte minute, LightEffectFunction lightEffectFunction, byte rampDuration);
LCDAlarm makeLCDAlarm(byte hour, byte minute, LightEffectFunction lightEffectFunction, byte rampDuration);
void alarmHandler();
// Color Ramp Functions
void rampTimerHandler();
void incrementNextRampLightColorIndex();
LightColor makeLightColor(byte red, byte green, byte blue, byte white);
// Misc Functions
time_t syncProvider();
void scheduleRandomStorm();
int availableRAM();
// Computer Interface Functions
int serialReadInt();
void processComputerCommands(int cmd);
// LCD Functions
void printNumberToLCDWithLeadingZeros(int numberToPrint);
void clearLCDLine(byte lineNumber);
void updateLCD();
void showStatusMenu();
void showLCDMainMenu();
void LCDPowerOn();
void LCDPowerOff();
// Encoder Functions
void checkEncoderStatus();
void encoderPositionChanged();
void encoderClicked();
void encoderUnClicked();
void readEncoder();
// IR Functions
void sendIRCode(byte cmd, byte numTimes);
//Light Effect Functions
void Orange();
void Blue();
void Rose();
void PowerOnOff();
void White();
void FullSpec();
void Purple();
void Play();
void RedUp();
void GreenUp();
void BlueUp();
void WhiteUp();
void RedDown();
void GreenDown();
void BlueDown();
void WhiteDown();
void M1Custom();
void M2Custom();
void M3Custom();
void M4Custom();
void Moon1();
void Moon2();
void Moon3();
void DawnDusk();
void Cloud1();
void Cloud2();
void Cloud3();
void Cloud4();
void Storm1();
void Storm2();
void Storm3();
void Storm4();

#if DEBUG
void SimpleRamp(LightColor lightColor, int tstep);
#endif

//**********************************************************************//
#pragma mark - Debug/Testing Variable Declarations

#if DEBUG
// define which test to run
#define TEST 3

// Test 1: every 5 seconds a new command is sent for testing
int loopTester = 0;
long interval = 5000;
long previousMillis = 0;

// Test 2: ramp testing
long simpleRampDuration = 10000;
#endif

//**********************************************************************//
#pragma mark - Variables

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
bool encoderClickStatus = true;

//Variables for ramping function
LightColor currentColor = {0, 0, 0, 0};
LightColor targetColor = {0, 0, 0, 0};
AlarmID_t rampTimerID = dtINVALID_ALARM_ID;
byte nextRampLightColorIndex = 0;

// Array to hold our current alarms
LCDAlarm lcdAlarms[dtNBR_ALARMS - RESERVED_ALARMS];
byte lcdAlarmsCount = 0;

// LCD Menu variables
byte lcdMenuLayer = 0;
byte lcdScrollPosition = 0;
byte lcdSelectorPosition = 0;

int previousLCDUpdateSecond = 0;

byte lastIRCodeSent = 19; //Really hackishly set this to 19 (ie Custom 4) to start with to assume that it's "off" when we start up

// Random Thunderstorm variables
byte nextStormHour = 0;
byte nextStormMinute = 0;
byte nextStormDurationHours = 0;
byte nextStormDurationMinutes = 0;

// Current Satellite+ IR Codes (NEC Protocol)
unsigned long codeHeader = 0x20DF; // Always the same
								   // Remote buttons listed left to right, top to bottom
const PROGMEM unsigned int lightCodes[LIGHTING_OPTIONS] = {
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
	0xC837 	 // 32 - Storm 4
};

// These are the messages that print on the serial monitor & lcd when each IR code is sent
const PROGMEM char lightingMessage[LIGHTING_OPTIONS][MAX_MSG_LEN + 1] = {
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

// Steps for each color preset. Used for ramping
LightColor lightColors[LIGHTING_OPTIONS];/* = {
	{MAX_COLOR_STEPS, MAX_COLOR_STEPS, 0, MAX_COLOR_STEPS},  								// 1 -  Orange
	{0, 0, MAX_COLOR_STEPS, MAX_COLOR_STEPS},  											// 2 -  Blue
	{MAX_COLOR_STEPS, 15, 0, MAX_COLOR_STEPS},  											// 3 -  Rose
	{0, 0, 0, 0},  																	// 4 -  Power On/Off
	{0, 0, 0, MAX_COLOR_STEPS},  											// 5 -  White
	{MAX_COLOR_STEPS, MAX_COLOR_STEPS, MAX_COLOR_STEPS, MAX_COLOR_STEPS},  	// 6 -  FullSpec
	{MAX_COLOR_STEPS, 0, MAX_COLOR_STEPS, 0},  								// 7 -  Purple
	{0, 0, 0, 0},  																	// 8 -  Play/Pause
	{0, 0, 0, 0},  																	// 9 -  Red Up
	{0, 0, 0, 0},  																	// 10 - Green Up
	{0, 0, 0, 0},  																	// 11 - Blue Up
	{0, 0, 0, 0},  																	// 12 - White Up
	{0, 0, 0, 0},  																	// 13 - Red Down
	{0, 0, 0, 0},  																	// 14 - Green Down
	{0, 0, 0, 0},  																	// 15 - Blue Down
	{0, 0, 0, 0},  																	// 16 - White Down
	{0, 0, 20, 0},  								// 17 - M1 Custom
	{28, 0, 28, 14},  															// 18 - M2 Custom
	{0, 28, 40, 30},  															// 19 - M3 Custom
	{0, 0, 0, 0},  															// 20 - M4 Custom
	{0, 0, 20, 0},  														// 21 - Moon 1
	{0, 0, 20, 0}, 															// 22 - Moon 2
	{0, 0, 10, 0}, 															// 23 - Moon 3
	{28, 0, 28, 14},														// 24 - Dawn/Dusk
	{0, 0, 0, MAX_COLOR_STEPS},  											// 25 - Cloud 1
	{0, 28, 40, 30},  											// 26 - Cloud 2
	{0, 0, 0, MAX_COLOR_STEPS},  											// 27 - Cloud 3
	{0, 0, 0, MAX_COLOR_STEPS},  											// 28 - Cloud 4
	{0, 0, 20, 0},  														// 29 - Storm 1
	{0, 0, 20, 0},  														// 30 - Storm 2
	{0, 0, 20, 0},  														// 31 - Storm 3
	{0, 0, 20, 0} 															// 32 - Storm 4
};*/

LightEffectFunction lightEffectFunctions[LIGHTING_OPTIONS];

//**********************************************************************//
#pragma mark - Setup and Loop

/*****************************
 // Setup everything; get LCD turned on and updated; set alarms
 *****************************/
void setup() {
	Wire.begin();
	RTC.begin();
	Serial.begin(9600);
	
	//Set the LCD mosfet pin as an output
	pinMode(LCD_MOSFET, OUTPUT);
	LCDPowerOn();
	
	if (!RTC.isrunning()) {
		// If no RTC is installed, set time to compile time at each reset
    #if DEBUG
		Serial.println(F("RTC is NOT running!\n"));  // Store this string in PROGMEM
    #endif
		RTC.adjust(DateTime(__DATE__, __TIME__));
	}
	
	// This keeps the RTC library and the Time library in sync
	setSyncProvider(syncProvider);     // reference our syncProvider function instead of RTC_DS1307::get()
	
  #if DEBUG
	// Print the time
	Serial.print(F("Time: "));
	Serial.print(hour());
	Serial.print(F(":"));
	Serial.print(minute());
	Serial.print(F(":"));
	Serial.println(second());
  #endif

  //Initialize all of our lists
  /*lightCodes[0] = 0x3AC5;  // 1 -  Orange
  lightCodes[1] = 0xBA45;  // 2 -  Blue
  lightCodes[2] = 0x827D;  // 3 -  Rose
  lightCodes[3] = 0x02FD;  // 4 -  Power On/Off
  lightCodes[4] = 0x1AE5;  // 5 -  White
  lightCodes[5] = 0x9A65;  // 6 -  FullSpec
  lightCodes[6] = 0xA25D;  // 7 -  Purple
  lightCodes[7] = 0x22DD;  // 8 -  Play/Pause
  lightCodes[8] = 0x2AD5;  // 9 -  Red Up
  lightCodes[9] = 0xAA55;  // 10 - Green Up
  lightCodes[10] = 0x926D;  // 11 - Blue Up
  lightCodes[11] = 0x12ED;  // 12 - White Up
  lightCodes[12] = 0x0AF5;  // 13 - Red Down
  lightCodes[13] = 0x8A75;  // 14 - Green Down
  lightCodes[14] = 0xB24D;  // 15 - Blue Down
  lightCodes[15] = 0x32CD;  // 16 - White Down
  lightCodes[16] = 0x38C7;  // 17 - M1 Custom
  lightCodes[17] = 0xB847;  // 18 - M2 Custom
  lightCodes[18] = 0x7887;  // 19 - M3 Custom
  lightCodes[19] = 0xF807;  // 20 - M4 Custom
  lightCodes[20] = 0x18E7;  // 21 - Moon 1
  lightCodes[21] = 0x9867;  // 22 - Moon 2
  lightCodes[22] = 0x58A7;  // 23 - Moon 3
  lightCodes[23] = 0xD827;  // 24 - Dawn/Dusk
  lightCodes[24] = 0x28D7;  // 25 - Cloud 1
  lightCodes[25] = 0xA857;  // 26 - Cloud 2
  lightCodes[26] = 0x6897;  // 27 - Cloud 3
  lightCodes[27] = 0xE817;  // 28 - Cloud 4
  lightCodes[28] = 0x08F7;  // 29 - Storm 1
  lightCodes[29] = 0x8877;  // 30 - Storm 2
  lightCodes[30] = 0x48B7;  // 31 - Storm 3
  lightCodes[31] = 0xC837;  // 32 - Storm 4

  lightingMessage[0] = "Orange";
  lightingMessage[1] = "Blue";
  lightingMessage[2] = "Rose";
  lightingMessage[3] = "Power On/Off";
  lightingMessage[4] = "White";
  lightingMessage[5] = "Full Spectrum";
  lightingMessage[6] = "Purple";
  lightingMessage[7] = "Play/Pause";
  lightingMessage[8] = "Red Up";
  lightingMessage[9] = "Green Up";
  lightingMessage[10] = "Blue Up";
  lightingMessage[11] = "White Up";
  lightingMessage[12] = "Red Down";
  lightingMessage[13] = "Green Down";
  lightingMessage[14] = "Blue Down";
  lightingMessage[15] = "White Down";
  lightingMessage[16] = "Custom Mix 1";
  lightingMessage[17] = "Custom Mix 2";
  lightingMessage[18] = "Custom Mix 3";
  lightingMessage[19] = "Custom Mix 4";
  lightingMessage[20] = "Moonlight 1";
  lightingMessage[21] = "Moonlight 2";
  lightingMessage[22] = "Moonlight 3";
  lightingMessage[23] = "Dawn/Dusk";
  lightingMessage[24] = "Cloud Cover 1";
  lightingMessage[25] = "Cloud Cover 2";
  lightingMessage[26] = "Cloud Cover 3";
  lightingMessage[27] = "Cloud Cover 4";
  lightingMessage[28] = "Storm 1";
  lightingMessage[29] = "Storm 2";
  lightingMessage[30] = "Storm 3";
  lightingMessage[31] = "Storm 4";*/

  lightColors[0] = {MAX_COLOR_STEPS, MAX_COLOR_STEPS, 0, MAX_COLOR_STEPS};                 // 1 -  Orange
  lightColors[1] = {0, 0, MAX_COLOR_STEPS, MAX_COLOR_STEPS};                       // 2 -  Blue
  lightColors[2] = {MAX_COLOR_STEPS, 15, 0, MAX_COLOR_STEPS};                        // 3 -  Rose
  lightColors[3] = {0, 0, 0, 0};                                   // 4 -  Power On/Off
  lightColors[4] = {0, 0, 0, MAX_COLOR_STEPS};                       // 5 -  White
  lightColors[5] = {MAX_COLOR_STEPS, MAX_COLOR_STEPS, MAX_COLOR_STEPS, MAX_COLOR_STEPS};   // 6 -  FullSpec
  lightColors[6] = {MAX_COLOR_STEPS, 0, MAX_COLOR_STEPS, 0};                 // 7 -  Purple
  lightColors[7] = {0, 0, 0, 0};                                   // 8 -  Play/Pause
  lightColors[8] = {0, 0, 0, 0};                                   // 9 -  Red Up
  lightColors[9] = {0, 0, 0, 0};                                   // 10 - Green Up
  lightColors[10] = {0, 0, 0, 0};                                   // 11 - Blue Up
  lightColors[11] = {0, 0, 0, 0};                                   // 12 - White Up
  lightColors[12] = {0, 0, 0, 0};                                   // 13 - Red Down
  lightColors[13] = {0, 0, 0, 0};                                   // 14 - Green Down
  lightColors[14] = {0, 0, 0, 0};                                   // 15 - Blue Down
  lightColors[15] = {0, 0, 0, 0};                                   // 16 - White Down
  lightColors[16] = {0, 0, 20, 0};                  // 17 - M1 Custom
  lightColors[17] = {28, 0, 28, 14};                                // 18 - M2 Custom
  lightColors[18] = {0, 28, 40, 30};                                // 19 - M3 Custom
  lightColors[19] = {0, 0, 0, 0};                               // 20 - M4 Custom
  lightColors[20] = {0, 0, 20, 0};                              // 21 - Moon 1
  lightColors[21] = {0, 0, 20, 0};                              // 22 - Moon 2
  lightColors[22] = {0, 0, 10, 0};                              // 23 - Moon 3
  lightColors[23] = {28, 0, 28, 14};                            // 24 - Dawn/Dusk
  lightColors[24] = {0, 0, 0, MAX_COLOR_STEPS};                       // 25 - Cloud 1
  lightColors[25] = {0, 28, 40, 30};                        // 26 - Cloud 2
  lightColors[26] = {0, 0, 0, MAX_COLOR_STEPS};                       // 27 - Cloud 3
  lightColors[27] = {0, 0, 0, MAX_COLOR_STEPS};                       // 28 - Cloud 4
  lightColors[28] = {0, 0, 20, 0};                              // 29 - Storm 1
  lightColors[29] = {0, 0, 20, 0};                              // 30 - Storm 2
  lightColors[30] = {0, 0, 20, 0};                              // 31 - Storm 3
  lightColors[31] = {0, 0, 20, 0};                               // 32 - Storm 4

  lightEffectFunctions[0] = Orange;
  lightEffectFunctions[1] = Blue;
  lightEffectFunctions[2] = Rose;
  lightEffectFunctions[3] = PowerOnOff;
  lightEffectFunctions[4] = White;
  lightEffectFunctions[5] = FullSpec;
  lightEffectFunctions[6] = Purple;
  lightEffectFunctions[7] = Play;
  lightEffectFunctions[8] = RedUp;
  lightEffectFunctions[9] = GreenUp;
  lightEffectFunctions[10] = BlueUp;
  lightEffectFunctions[11] = WhiteUp;
  lightEffectFunctions[12] = RedDown;
  lightEffectFunctions[13] = GreenDown;
  lightEffectFunctions[14] = BlueDown;
  lightEffectFunctions[15] = WhiteDown;
  lightEffectFunctions[16] = M1Custom;
  lightEffectFunctions[17] = M2Custom;
  lightEffectFunctions[18] = M3Custom;
  lightEffectFunctions[19] = M4Custom;
  lightEffectFunctions[20] = Moon1;
  lightEffectFunctions[21] = Moon2;
  lightEffectFunctions[22] = Moon3;
  lightEffectFunctions[23] = DawnDusk;
  lightEffectFunctions[24] = Cloud1;
  lightEffectFunctions[25] = Cloud2;
  lightEffectFunctions[26] = Cloud3;
  lightEffectFunctions[27] = Cloud4;
  lightEffectFunctions[28] = Storm1;
  lightEffectFunctions[29] = Storm2;
  lightEffectFunctions[30] = Storm3;
  lightEffectFunctions[31] = Storm4;
  
	initializeLCDAlarms();  // Set up above alarms
	
  #if DEBUG
  // Print available SRAM for debugging, comment out if you want
	Serial.print(F("SRAM: "));
	Serial.print(availableRAM());
	Serial.println(" bytes");
	Serial.println(F("To test IR codes, send 1 - 32\n"));
  #endif
	
	// Setup the click wheel encoder pins
	pinMode(ENCODER_A,INPUT);
	pinMode(ENCODER_B,INPUT);
	pinMode(ENCODER_CLICK,INPUT);
	digitalWrite(ENCODER_A,HIGH);
	digitalWrite(ENCODER_B,HIGH);
	digitalWrite(ENCODER_CLICK,HIGH);
	
	attachInterrupt(0, readEncoder, CHANGE);
}

/*****************************
 // Check for computer commands
 // If DEBUG, then loop through a rapid test schedule
 // Update the LCD
 *****************************/
void loop() {
	// Check for computer commands
#if DEBUG
	if (Serial.available() > 0) {
		processComputerCommands(serialReadInt());  // Go grab IR code and send it
	}
#endif
	
	// Update the alarms
	Alarm.delay(0);
	
#if DEBUG
#if TEST == 1
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis > interval) {
		previousMillis = currentMillis;
		
		if (loopTester == 0)
			Moon2();
		else if (loopTester == 1)
			DawnDusk();
		else if (loopTester == 2)
			Cloud2();
		else if (loopTester == 3)
			Blue();
		else if (loopTester == 4)
			DawnDusk();
		else if (loopTester == 5)
			Moon2();
		else if (loopTester == 6) {
			M4Custom();
			loopTester = -1;
		}
		
		loopTester++;
	}
#endif
#if TEST == 2
	//Simple ramp (using delays and for loops)
	M4Custom();
	Serial.println("ramp1");
	SimpleRamp(lightColors[21], simpleRampDuration / 15);
	Moon2();
	Play();
	Serial.println("ramp2");
	SimpleRamp(lightColors[23], simpleRampDuration / 40);
	DawnDusk();
	Play();
	Serial.println("ramp3");
	SimpleRamp(lightColors[21], simpleRampDuration / 40);
	Moon2();
	Play();
	Serial.println("ramp4");
	SimpleRamp(lightColors[19], simpleRampDuration / 15);
	M4Custom();
#endif
#endif
	
	// Check the encoder
	checkEncoderStatus();
	
	// Update the LCD if it's on
  if (digitalRead(LCD_MOSFET) == HIGH)
	  updateLCD();
}

//**********************************************************************//
#pragma mark - Alarms

/*****************************
 // For now scheudle is hardcoded. Eventually read it from EEPROM
 *****************************/
void initializeLCDAlarms() {
  #if DEBUG == 0
	// Soon this will read from EEPROM
	addLCDAlarm(10, 00, Moon2, 15);
  addLCDAlarm(10, 59, M1Custom, 0);
	addLCDAlarm(11, 00, DawnDusk, 15);
  addLCDAlarm(11, 59, M2Custom, 0);
	addLCDAlarm(12, 00, Cloud2, 15);
  addLCDAlarm(13, 59, M3Custom, 0);
	addLCDAlarm(14, 00, FullSpec, 5);
	addLCDAlarm(16, 00, Blue, 5);
	addLCDAlarm(18, 00, Cloud2, 5);
  addLCDAlarm(19, 59, M3Custom, 0);
	addLCDAlarm(20, 00, DawnDusk, 15);
  addLCDAlarm(20, 59, M2Custom, 0);
	addLCDAlarm(21, 00, Moon2, 15);
  addLCDAlarm(22, 59, M1Custom, 0);
	addLCDAlarm(23, 00, M4Custom, 15); // Off
  #endif
  #if DEBUG
  #if TEST == 3
  byte theHour = hour();
  byte theMinute = minute();
  addLCDAlarm(theHour, theMinute, Moon2, 1);
  /*addLCDAlarm(theHour, theMinute+1, M1Custom, 0);
  addLCDAlarm(theHour, theMinute+2, DawnDusk, 1);
  addLCDAlarm(theHour, theMinute+3, M2Custom, 0);
  addLCDAlarm(theHour, theMinute+4, Cloud2, 1);
  addLCDAlarm(theHour, theMinute+5, M3Custom, 0);
  addLCDAlarm(theHour, theMinute+6, FullSpec, 1);
  addLCDAlarm(theHour, theMinute+7, Blue, 1);
  addLCDAlarm(theHour, theMinute+8, Cloud2, 1);
  addLCDAlarm(theHour, theMinute+9, M3Custom, 0);
  addLCDAlarm(theHour, theMinute+10, DawnDusk, 1);
  addLCDAlarm(theHour, theMinute+11, M2Custom, 0);
  addLCDAlarm(theHour, theMinute+12, Moon2, 1);
  addLCDAlarm(theHour, theMinute+13, M1Custom, 0);*/
  addLCDAlarm(theHour, theMinute+1, M4Custom, 1);
  addLCDAlarm(theHour, theMinute+3, Moon2, 1);
  #endif
  #endif
	
	// Turn on the current lighting
	alarmHandler();
	
	// Comment these out if you don't want the chance of a random storm each day
	Alarm.alarmRepeat(00,01,00, scheduleRandomStorm);  // Schedule a new storm just after midnight each day.
	scheduleRandomStorm();  // Sets up intial storm so we don't have wait until alarm time
}

/*****************************
 // Figure out what the lights should be at right now.
 // Note: this does not do any ramping since we don't now what the lights were when we boot up.
 *****************************/
void setCurrentLightingAccordingToSchedule() {
	(*(getLightingForTime(hour(), minute())))(); // Haha, call the function for the current alarm
}

/*****************************
 // Figure out what light function is active right now.
 // Note: Does not include ramp time, this is just what light function should be active according to the alarm schedule.
 *****************************/
LightEffectFunction getLightingForTime(byte hour, byte minute) {
	return getLCDAlarmForTime(hour, minute).lightEffectFunction;
}

/*****************************
 // Figure out what lcd alarm is active at the given time. This assumes the LCDAlarms in lcdAlarms are ordered sequencially by time.
 *****************************/
LCDAlarm getLCDAlarmForTime(byte hour, byte minute) {
  int theHour = hour;
  int theMinute = minute;
  for (byte i = lcdAlarmsCount - 1; i >= 0; i --) {
    if ((theHour >= lcdAlarms[i].hour && theMinute >= lcdAlarms[i].minute)) {
      return lcdAlarms[i];
    }
  }
  
  if (theHour < lcdAlarms[0].hour && theMinute < lcdAlarms[0].minute) {
    return lcdAlarms[lcdAlarmsCount - 1];
  }
}

/*****************************
 // Add the alarm to the alarm array.
 // rampDuration is the length of time in minutes you want to take to get from where we were to where we're going.
 // The ramp up starts at the time from hour and minute and ends rampDuration minutes later, at which time the actual light function is called to make up for any missed transmissions
 // NOTE: Ramping only works if the previous light effect is NOT a dynamic effect. It must be a static color or custom color first.
 *****************************/
void addLCDAlarm(byte hour, byte minute, LightEffectFunction lightEffectFunction, byte rampDuration) {
  if (lcdAlarmsCount < dtNBR_ALARMS - RESERVED_ALARMS) {
  	lcdAlarms[lcdAlarmsCount] = makeLCDAlarm(hour, minute, lightEffectFunction, rampDuration);
  	
  	lcdAlarmsCount ++;
  	
  	// Actually create the alarm
  	lcdAlarms[lcdAlarmsCount].alarmID = Alarm.alarmRepeat(hour, minute, 0, alarmHandler);
  }
  #if DEBUG
  else {
    Serial.println(F("WARNING: Tried to add more Alarms than we can handle!"));
  }
  #endif
}

/*****************************
 // Set the alarm at index to the values given
 *****************************/
void setLCDAlarmAtIndex(byte index, byte hour, byte minute, LightEffectFunction lightEffectFunction, byte rampDuration) {
  if (index >= lcdAlarmsCount) {
    // Delete the alarm, then edit it
    Alarm.free(lcdAlarms[index].alarmID);
    
  	lcdAlarms[index] = makeLCDAlarm(hour, minute, lightEffectFunction, rampDuration);
  	
  	// Recreate the alarm
  	lcdAlarms[index].alarmID = Alarm.alarmRepeat(hour, minute, 0, alarmHandler);
  }
  #if DEBUG
  else {
    Serial.println(F("WARNING: Tried to add more Alarms than we can handle!"));
  }
  #endif
}

/*****************************
 // Make a new LCDAlarm with the values given
 *****************************/
LCDAlarm makeLCDAlarm(byte hour, byte minute, LightEffectFunction lightEffectFunction, byte rampDuration) {
  LCDAlarm lcdAlarm;
  lcdAlarm.hour = hour;
  lcdAlarm.minute = minute;
  lcdAlarm.lightEffectFunction = lightEffectFunction;
  //to set the lightColor, we need to figure out the index of the lightEffectFunction and use that
  for (int i = 0; i < LIGHTING_OPTIONS; i++) {
    if (lightEffectFunctions[i] == lightEffectFunction) {
      lcdAlarm.lightColor = lightColors[i];
      break;
    }
  }
  lcdAlarm.rampDuration = rampDuration;

  return lcdAlarm;
}

/*****************************
 // Handle each alarm when it comes in
 *****************************/
void alarmHandler() {
  //first thing, whenever there's an alarm check to see if there's a currently running timer and stop it.
  if (Alarm.readType(rampTimerID) == dtTimer) {
    Alarm.free(rampTimerID);
    rampTimerID == dtINVALID_ALARM_ID;
  }
  //get the current alarm and call the light effect function if there's no rampDuration. If there is, then set up the timer for the ramping
  LCDAlarm currentAlarm = getLCDAlarmForTime(hour(), minute());
  #if DEBUG
  Serial.print(F("rampDuration: "));
  Serial.println(currentAlarm.rampDuration);
  #endif
  if (currentAlarm.rampDuration == 0) {
    (*currentAlarm.lightEffectFunction)();
  }
  else {
    //first, set the target color we'll be aiming for
    targetColor = currentAlarm.lightColor;

    //then figure out how many steps will be involved in the ramping, and from that how often the timer needs to go off
    int redDifference = currentColor.red - targetColor.red;
    int greenDifference = currentColor.green - targetColor.green;
    int blueDifference = currentColor.blue - targetColor.blue;
    int whiteDifference = currentColor.white - targetColor.white;
    int totalSteps = abs(redDifference) + abs(greenDifference) + abs(blueDifference) + abs(whiteDifference);

    //now set the ramping timer
    byte timerDuration = currentAlarm.rampDuration * 60 / totalSteps;
    rampTimerID = Alarm.timerRepeat(0, 0, timerDuration >= 1 ? timerDuration : 1, rampTimerHandler);
    #if DEBUG
    Serial.print(F("rampTimerID: "));
    Serial.println(rampTimerID);
    Serial.print(F("Timer Duration: "));
    Serial.println(timerDuration >= 1 ? timerDuration : 1);
    #endif
  }
}

//**********************************************************************//
#pragma mark - Color Ramping

/*****************************
 // Each time this is called, we're going to increment 1 color value for our current color
 *****************************/
void rampTimerHandler() {
  #if DEBUG
    Serial.print(F("nextRampLightColorIndex: "));
    Serial.println(nextRampLightColorIndex);
    Serial.print(F("currentColor: "));
    Serial.print(currentColor.red);
    Serial.print(F(", "));
    Serial.print(currentColor.green);
    Serial.print(F(", "));
    Serial.print(currentColor.blue);
    Serial.print(F(", "));
    Serial.println(currentColor.white);
    Serial.print(F("targetColor: "));
    Serial.print(targetColor.red);
    Serial.print(F(", "));
    Serial.print(targetColor.green);
    Serial.print(F(", "));
    Serial.print(targetColor.blue);
    Serial.print(F(", "));
    Serial.println(targetColor.white);
  #endif
  // We need to check each color index to see if it can be changed. If it can, then change it, increment the nextRampLightColorIndex, and return. If it can't, then see if we're done incrementing.
  if (nextRampLightColorIndex == 0 && currentColor.red != targetColor.red) {
    if (currentColor.red < targetColor.red)
      RedUp();
    else
      RedDown();
  }
  if (nextRampLightColorIndex == 1 && currentColor.green != targetColor.green) {
    if (currentColor.green < targetColor.green)
      GreenUp();
    else
      GreenDown();
  }
  if (nextRampLightColorIndex == 2 && currentColor.blue != targetColor.blue) {
    if (currentColor.blue < targetColor.blue)
      BlueUp();
    else
      BlueDown();
  }
  if (nextRampLightColorIndex == 3 && currentColor.white != targetColor.white) {
    if (currentColor.white < targetColor.white)
      WhiteUp();
    else
      WhiteDown();
  }

  //now that we've incremented a color this round, figure out the next color that needs to be incremented next round
  incrementNextRampLightColorIndex();

  //if we get to this point, make sure all the colors match, then we want to destroy this timer, and call the final LightEffectFunction
  if (currentColor.red == targetColor.red && currentColor.green == targetColor.green && currentColor.blue == targetColor.blue && currentColor.white == targetColor.white) {
    (*(getLCDAlarmForTime(hour(), minute()).lightEffectFunction))();
    Alarm.free(rampTimerID);
    rampTimerID = dtINVALID_ALARM_ID;
  }
}

/*****************************
 // Each time this is called, we're going to increment the nextRampLightColorIndex to the next color that needs to be incremented
 *****************************/
void incrementNextRampLightColorIndex() {
  boolean nextRampLightColorIndexFound = false;
  byte startingIndex = nextRampLightColorIndex;
  do {
    //first, increment nextRampLightColor and make sure we're still in bounds
    nextRampLightColorIndex++;
    if (nextRampLightColorIndex >= 4)
      nextRampLightColorIndex = 0;

    //Then see if that color needs to be incremented. If it does, then we're done
    if ((nextRampLightColorIndex == 0 && currentColor.red != targetColor.red) ||
        (nextRampLightColorIndex == 1 && currentColor.green != targetColor.green) ||
        (nextRampLightColorIndex == 2 && currentColor.blue != targetColor.blue) ||
        (nextRampLightColorIndex == 3 && currentColor.white != targetColor.white))
      nextRampLightColorIndexFound = true;
  } while (!nextRampLightColorIndexFound && nextRampLightColorIndex != startingIndex);

  //if we didn't find a color that needs to be incremented, then reset nextRampLightColorIndex
  if (!nextRampLightColorIndexFound && nextRampLightColorIndex == startingIndex)
    nextRampLightColorIndex = 0;
}

/*****************************
 // Make a LightColor struct with the given values
 *****************************/
LightColor makeLightColor(byte red, byte green, byte blue, byte white) {
	LightColor lightColor = {red, green, blue, white};
  return lightColor;
}

//**********************************************************************//
#pragma mark - Misc

/*****************************
 // Sync RTC and arduino
 *****************************/
time_t syncProvider() {
	//this does the same thing as RTC_DS1307::get()
	return RTC.now().unixtime();
}

/*****************************
 // Schedules a storm between 1 & 9 in the evening
 // It sets Storm2, followed by the correct light function when the storm is over.
 // No ramping
 *****************************/
void scheduleRandomStorm() {
	randomSeed(analogRead(randAnalogPin));  // Generate random seed on unused pin
	nextStormHour = random(23);                   // Randomizer for RandomStorm
	nextStormMinute = random(59);
	byte nextStormSecond = random(59);
	nextStormDurationHours = random(2);
	nextStormDurationMinutes = random(59);
	
	// If 1:00PM - 9:00PM schedule a storm
	if (nextStormHour >= 13 && nextStormHour <= 21) {
		// Schedule the storm
		Alarm.alarmOnce(nextStormHour, nextStormMinute, nextStormSecond, Storm2);

    #if DEBUG
		// Show storm info to the console
		Serial.print(F("Scheduled Storm: "));
		Serial.print(nextStormHour);
		Serial.print(F(":"));
		Serial.print(nextStormMinute);
		Serial.print(F(":"));
		Serial.print(nextStormSecond);
		Serial.print(F(" Duration: "));
		Serial.print(nextStormDurationHours);
		Serial.print(F(":"));
		Serial.println(nextStormDurationMinutes);
    #endif
		
		// Schedule an alarm to change the weather back to the scheduled weather once the storm is over
		byte endHour = nextStormHour + nextStormDurationHours;
		byte endMinute = nextStormMinute + nextStormDurationMinutes;
		Alarm.alarmOnce(endHour, endMinute, nextStormSecond, getLightingForTime(endHour, endMinute));
	}
	else {
		// Set the duration to 0 so we know that there is no storm
		nextStormDurationHours = 0;
		nextStormDurationMinutes = 0;
		
    #if DEBUG
		// Don't really need this, but it can stay till we need the space
		Serial.println(F("No storm today"));
    #endif
	}
	
	// Don't "need" this because the lcd gets updated every loop anyways, but this is more verbose
	updateLCD();
}

/*****************************
 // Magic Code
 *****************************/
int availableRAM() {
	// Returns available SRAM
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//**********************************************************************//
#pragma mark - Computer Commands

/*****************************
 // For reading from the computer
 *****************************/
int serialReadInt() {
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

/*****************************
 // Read the function name
 *****************************/
void processComputerCommands(int cmd) {
	// Handles commands sent in from the serial monitor
	if (cmd >= 1 && cmd <= 32) {
		// cmd must be 1 - 32
		sendIRCode(cmd-1, 1);
	}
  #if DEBUG
	else {
		Serial.println(F("Invalid Choice"));
	}
  #endif
}

//**********************************************************************//
#pragma mark - LCD

/*****************************
 // I love long and explanatory function names
 *****************************/
void printNumberToLCDWithLeadingZeros(int numberToPrint) {
	// Utility function for digital clock display: prints leading 0's
	if (numberToPrint < 10)
		lcd.print(F("0"));
	lcd.print(numberToPrint);
}

/*****************************
 // Do I need to say it again?
 *****************************/
void clearLCDLine(byte lineNumber) {
	lcd.setCursor(0, lineNumber);
	lcd.print(LCD_BLANK_LINE);
}

/*****************************
 // Determine what menu layer we're in and update accordingly
 // TODO: Actually make it work
 *****************************/
void updateLCD() {
	//if(lcdMenuLayer == 0) {
	showStatusMenu();
	//}
	//else if(lcdMenuLayer == 1) {
	//    showLCDMainMenu();
	//}
}

/*****************************
 // This is the main layer. Shows the time, random storm status, current lighting, etc.
 *****************************/
void showStatusMenu() {
	// Update the LCD once a second - This needs to be improved to better handle changes
	if (second() != previousLCDUpdateSecond) {
		lcd.clear();
		
		// Print the time HH:MM:SS
		lcd.setCursor(0,0);
		printNumberToLCDWithLeadingZeros(hourFormat12());
		lcd.print(":");
		printNumberToLCDWithLeadingZeros(minute());
		lcd.print(":");
		printNumberToLCDWithLeadingZeros(second());
		if (isAM()) {
			lcd.print(F(" AM"));
		}
		else {
			lcd.print(F(" PM"));
		}
		previousLCDUpdateSecond = second();
		
		// Show storm info
		if (nextStormDurationHours == 0 && nextStormDurationMinutes == 0) {
			lcd.setCursor(0,1);
			lcd.print(F("No storm today"));
		}
		else {
			lcd.setCursor(0,1);
			lcd.print(F("Storm Time: "));
			printNumberToLCDWithLeadingZeros((nextStormHour <= 12) ? nextStormHour : (nextStormHour - 12));
			lcd.print(F(":"));
			printNumberToLCDWithLeadingZeros(nextStormMinute);
			if(nextStormHour < 12) {
				lcd.print(F(" AM"));
			}
			else {
				lcd.print(F(" PM"));
			}
		}
		
		// Show the lighting info
		lcd.setCursor(0,2);
		lcd.print(PGMSTR(lightingMessage[lastIRCodeSent]));
	}
}

/*****************************
 // TODO: Not written yet
 *****************************/
void showLCDMainMenu() {
	
}

/*****************************
 // Turn on the LCD. Must have LCD_MOSFET pin set
 *****************************/
void LCDPowerOn() {
 if (digitalRead(LCD_MOSFET) == LOW) {
	  digitalWrite(LCD_MOSFET, HIGH);
    lcd.begin(LCD_COLUMNS, LCD_ROWS);
	}
}

/*****************************
 // Turn off LCD for the night.
 *****************************/
void LCDPowerOff() {
  if (digitalRead(LCD_MOSFET) == HIGH) {
	  digitalWrite(LCD_MOSFET, LOW);
  }
}

//**********************************************************************//
#pragma mark - Encoder

/*****************************
 // Check encoder status (the clicky wheel thing)
 *****************************/
void checkEncoderStatus() {
	// See if the encoder has moved the required amount
	if (abs(encoderDifference) >= ENCODER_PULSES_PER_STEP) {
		encoderPosition += encoderDifference / ENCODER_PULSES_PER_STEP;
		encoderDifference = 0;
		
		encoderPositionChanged();
	}
	
	// Check the encoder click button
	if (digitalRead(ENCODER_CLICK) == 0) {
		// Button transitions from not clicked to clicked
		if(encoderClickStatus == false) {
			encoderClicked();
		}
		
		encoderClickStatus = true;
		//Serial.println(F("Click "));
	}
	else if (digitalRead(ENCODER_CLICK == 1)) {
		if(encoderClickStatus == true) {
			encoderUnClicked();
		}
		
		encoderClickStatus = false;
	}
}

/*****************************
 // Don't remember why this is here
 *****************************/
void encoderPositionChanged() {
	
}

/*****************************
 // Used to navigate through the menu
 *****************************/
void encoderClicked() {
	// Beep
	analogWrite(BEEPER, 128);
	delay(100);
	analogWrite(BEEPER, 0);
	//tone(BEEPER, 500, 10);
	
	if (lcdMenuLayer == 0) {
		lcdMenuLayer ++;
	}
	
	updateLCD();
}

/*****************************
 // ???
 *****************************/
void encoderUnClicked() {
	// Do something here
}

/*****************************
 //I assume it reads the encoder
 *****************************/
void readEncoder() {
	// Read the rotary encoder
	if (digitalRead(ENCODER_A) == 0) {
		encoderBits |= ENCODER_A_MASK;
	}
	if (digitalRead(ENCODER_B) == 0) {
		encoderBits |= ENCODER_B_MASK;
	}
	// Determine the encoder rotation change
	if (encoderBits != previousEncoderBits) {
		switch (encoderBits) {
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

//**********************************************************************//
#pragma mark - IR

/*****************************
 // Duh
 *****************************/
void sendIRCode(byte cmd, byte numTimes) {
	// cmd = the element of the arrCode[] array that holds the IR code to be sent
	// numTimes = number of times to emmit the command
	// Shift header 16 bits to left, fetch code from PROGMEM & add it
	unsigned long irCode = (codeHeader << 16) + pgm_read_word_near(lightCodes + cmd);
	for (byte i = 0; i < numTimes; i++) {
		irsend.sendNEC(irCode,32); // Send/emmit code
		delay(100);
	}
	
  #if DEBUG
	// Print the string associated with the IR code & the time
	Serial.print(PGMSTR(lightingMessage[cmd]));
  Serial.print(F(" "));
  Serial.print(irCode, HEX);
	Serial.print(": ");
	Serial.print(hour());
	Serial.print(":");
	Serial.print(minute());
	Serial.print(":");
	Serial.println(second());
  #endif
	
	lastIRCodeSent = cmd;
	
	// Don't "need" this because the lcd gets updated every loop anyways, but this is more verbose
	updateLCD();
}

// IR Code functions, called by alarms
void Orange() {
	sendIRCode(0,2);
	currentColor = lightColors[0];
	LCDPowerOn();
}
void Blue() {
	sendIRCode(1,2);
	currentColor = lightColors[1];
	LCDPowerOn();
}
void Rose() {
	sendIRCode(2,2);
	currentColor = lightColors[2];
	LCDPowerOn();
}
//We do not recommend calling this ever. Not good things could happen. Instead we recommend setting Custom M4 to "off" and using that.
void PowerOnOff() {
	sendIRCode(3,1);
  LCDPowerOff();
}
void White() {
	sendIRCode(4,2);
	currentColor = lightColors[4];
	LCDPowerOn();
}
void FullSpec() {
	sendIRCode(5,2);
	currentColor = lightColors[5];
	LCDPowerOn();
}
void Purple() {
	sendIRCode(6,2);
	currentColor = lightColors[6];
	LCDPowerOn();
}
void Play() {
	sendIRCode(7,1);
	LCDPowerOn();
}
void RedUp() {
	sendIRCode(8,1);
	currentColor.red++;
	LCDPowerOn();
}
void GreenUp() {
	sendIRCode(9,1);
	currentColor.green++;
	LCDPowerOn();
}
void BlueUp() {
	sendIRCode(10,1);
	currentColor.blue++;
	LCDPowerOn();
}
void WhiteUp() {
	sendIRCode(11,1);
	currentColor.white++;
	LCDPowerOn();
}
void RedDown() {
	sendIRCode(12,1);
	currentColor.red--;
	LCDPowerOn();
}
void GreenDown() {
	sendIRCode(13,1);
	currentColor.green--;
	LCDPowerOn();
}
void BlueDown() {
	sendIRCode(14,1);
	currentColor.blue--;
	LCDPowerOn();
}
void WhiteDown() {
	sendIRCode(15,1);
	currentColor.white--;
	LCDPowerOn();
}
void M1Custom() {
	sendIRCode(16,2);
	currentColor = lightColors[16];
	LCDPowerOn();
}
void M2Custom() {
	sendIRCode(17,2);
	currentColor = lightColors[17];
	LCDPowerOn();
}
void M3Custom() {
	sendIRCode(18,2);
	currentColor = lightColors[18];
	LCDPowerOn();
}
// We recommend setting Custom M4 to "off" and using this instead of PowerOnOff.
void M4Custom() {
	sendIRCode(19,2);
	currentColor = lightColors[19];
	LCDPowerOff();
}
void Moon1() {
	sendIRCode(20,2);
	currentColor = lightColors[20];
	LCDPowerOn();
}
void Moon2() {
	sendIRCode(21,2);
	currentColor = lightColors[21];
	LCDPowerOn();
}
void Moon3() {
	sendIRCode(22,2);
	currentColor = lightColors[22];
	LCDPowerOn();
}
void DawnDusk() {
	sendIRCode(23,2);
	currentColor = lightColors[23];
	LCDPowerOn();
}
void Cloud1() {
	sendIRCode(24,2);
	currentColor = lightColors[24];
	LCDPowerOn();
}
void Cloud2() {
	sendIRCode(25,2);
	currentColor = lightColors[25];
	LCDPowerOn();
}
void Cloud3() {
	sendIRCode(26,2);
	currentColor = lightColors[26];
	LCDPowerOn();
}
void Cloud4() {
	sendIRCode(27,2);
	currentColor = lightColors[27];
	LCDPowerOn();
}
void Storm1() {
	sendIRCode(28,2);
	currentColor = lightColors[28];
	LCDPowerOn();
}
void Storm2() {
	sendIRCode(29,2);
	currentColor = lightColors[29];
	LCDPowerOn();
}
void Storm3() {
	sendIRCode(30,2);
	currentColor = lightColors[30];
	LCDPowerOn();
}
void Storm4() {
	sendIRCode(31,2);
	currentColor = lightColors[31];
	LCDPowerOn();
}

# if TEST == 2
//**********************************************************************//
#pragma mark - Test Functions

/*****************************
 // Does a simple ramp using for loops and delays.
 // THE LCD WILL NOT WORK PROPERLY WHEN USING THIS FUNCTION!
 *****************************/
void SimpleRamp(LightColor lightColor, int tstep) {
	for (int i=1;i<43;i++) {
		if (currentColor.red < lightColor.red) {
			RedUp();
			Serial.print("red up");
			Serial.println(currentColor.red);
			delay(tstep);
		}
		if (currentColor.red > lightColor.red) {
			RedDown();
			Serial.print("red down");
			Serial.println(currentColor.red);
			delay(tstep); }
		if (currentColor.green < lightColor.green) {
			GreenUp();
			Serial.print("green up");
			Serial.println(currentColor.green);
			delay(tstep); }
		if (currentColor.green > lightColor.green) {
			GreenDown();
			Serial.print("green down");
			Serial.println(currentColor.green);
			delay(tstep);
		}
		if (currentColor.blue < lightColor.blue) {
			BlueUp();
			Serial.print("blue up");
			Serial.println(currentColor.blue);
			delay(tstep);
		}
		if (currentColor.blue > lightColor.blue) {
			BlueDown();
			Serial.print("blue down");
			Serial.println(currentColor.blue);
			delay(tstep);
    }
		if (currentColor.white < lightColor.white) {
			WhiteUp();
			Serial.print("white up");
			Serial.println(currentColor.white);
			delay(tstep);
		}
		if (currentColor.white > lightColor.white) {
			WhiteDown();
			Serial.print("white down");
			Serial.println(currentColor.white);
			delay(tstep);
		}
	}
}

#endif

