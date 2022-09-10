#include <IRremote.h>
#include <FastLED.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

/*
 * ATMETA328PB pin mapping.
 * 
 * |-------------+------+--------------|
 * | Description | Pin* | Arduino Pin  |
 * |-------------+------+--------------|
 * | XTAL1       |   7  |         N/A  |
 * | XTAL2       |   8  |         N/A  |
 * | IR-Data     |   1  |           3  |
 * | PWM-OUT1    |   9  |           5  |
 * | PWM-OUT2    |  10  |           6  |
 * | D_OUT       |  13  |           9  |
 * | MOSI        |  15  |          11  |
 * | MISO        |  16  |          12  |
 * | SCK         |  17  |          13  |
 * | SENSE1      |  24  |          A1  |
 * | SENSE2      |  23  |          A0  |
 * | SDA         |  27  |          A4  |
 * | SCL         |  28  |          A5  |
 * | Reset       |  29  |         N/A  |
 * | RX          |  30  |         N/A  |
 * | TX          |  31  |         N/A  |
 * |-------------+------+--------------|
 * 
 * *) Pin means physical pin number.
*/

/*
 * ASSIGN YOUR REQUIRED IR-CODES!
 */

struct TV_REMOTE_CTRL {
  uint32_t TV_FULL_OFF;
  uint32_t TV_STEP_OFF;
  uint32_t TV_STEP_ON;
  uint32_t TV_FULL_ON;
  bool IsValid;
};

#define TV_CODES_EEPROM_ADDR   0

struct TV_REMOTE_CTRL tv_remote_ctrl;
struct TV_REMOTE_CTRL tv_remote_new;

// button on the LG TV remote control to set PWM output to MIN
#define BTN_FADE_FULL_OFF 1888484100
// button on the LG TV remote control to decrease PWM output by CURRENT_VAL_PWM_OUT_STEP_DW
#define BTN_FADE_STEP_OFF 1336998660
// button on the LG TV remote control to increase PWM output by CURRENT_VAL_PWM_OUT_STEP_UP
#define BTN_FADE_STEP_ON  1169881860
// button on the LG TV remote control to set PWM output to MAX
#define BTN_FADE_FULL_ON  1905195780

// button on the ARDUINO remote control to set PWM output to MIN (Button CH-)
#define BTN_EXT_FADE_FULL_OFF 3125149440
// button on the ARDUINO remote control to decrease PWM output by CURRENT_VAL_PWM_OUT_STEP_DW (Button -)
#define BTN_EXT_FADE_STEP_OFF 4161273600
// button on the ARDUINO remote control to increase PWM output by CURRENT_VAL_PWM_OUT_STEP_UP (Button +)
#define BTN_EXT_FADE_STEP_ON  3927310080
// button on the ARDUINO remote control to set PWM output to MAX (Button CH+)
#define BTN_EXT_FADE_FULL_ON  3091726080

SoftwareSerial mySerial(0, 1); // RX, TX

// the IR receiver read data pin
#define RECV_PIN                     3

#define PROGRAMMING_PIN              8

// define two different outputs, so each output could be controlled seperatly (in this sketch both are controlled the same way)
#define PWM_OUT_1                    5
#define PWM_OUT_2                    6
#define SENSE_1                      A1 // CURRENT_VAL_PWM_OUT sense value of PWM out channel 1
#define SENSE_2                      A0 // CURRENT_VAL_PWM_OUT sense value of PWM out channel 2

#define LED_DATA_OUT                 9
#define NUM_LEDS                     8
#define LED_TYPE                     WS2811
#define COLOR_ORDER                  GRB
#define UPDATES_PER_SECOND           100
#define WS2812_BRIGHTNESS            7
CRGB WS2812_LED[NUM_LEDS];

// defines the parameters of this program (start value for the outputs, in- and decrease steps, ...)
#define PWM_START_VALUE              0    // AnalogWrite value when startup 
#define PWM_MAX                      250  // Maximum value for AnalogWrite (PWM to LED)
#define PWM_MIN                      0    // Minimum value for AnalogWrite (PWM to LED)
#define CURRENT_VAL_PWM_OUT_INC_UP   1    // Step-up value (PWM to LED)
#define CURRENT_VAL_PWM_OUT_INC_DW   1    // Step-down value (PWM to LED)
#define CURRENT_VAL_PWM_OUT_STEP_UP  25   // Increase value (new value) for fade-up if button pressed
#define CURRENT_VAL_PWM_OUT_STEP_DW  25   // Decrease value (new value) for fade-down if button pressed
#define DELAY_STEP_UP                25   // Delay between fade-up (from CURRENT_VAL_PWM_OUTent value to the new value, using steps given as CURRENT_VAL_PWM_OUT_STEP_UP)
#define DELAY_STEP_DW                25   // Delay between fade-down (from CURRENT_VAL_PWM_OUTent value to the new value, using steps given as CURRENT_VAL_PWM_OUT_STEP_DW)

// defines the delay length for each loop() run 
#define DELAY_LOOP_RUN 25

// if a process is already running this variable is set to true (e.g. while lightUpSlow is running)
// implemented, just in case a later implemented interrupt needs this status information
volatile bool runIt = false;
// the CURRENT_VAL_PWM_OUTent output value for the OUTs
volatile int CURRENT_VAL_PWM_OUT = 0;

int GetCurrentPWM(int Channel) // Channel not used for now, but could be in future 
{
  return CURRENT_VAL_PWM_OUT;
}

int CurrentSense_1 = 0;
int CurrentSense_2 = 0;
byte CurrentReadCounter = 0;
byte ProgrammingIndex = 0;
bool LED_STRIP_ON = false;
bool IsProgrammingMode = false;

struct CurrentValues {
    int Channel_One, Channel_Two;
};

void LightUpLED(unsigned int Index, bool AllOff = false);
void ShowAnimation(bool);
void InitADC(void);
void printSenseValues(void);
byte getNumbersOfLEDs(void);
void ShowPMWValue(void);
void FadeStepDown(int);
void FadeStepUp(int);
int sort_desc(const void *cmp1, const void *cmp2);
struct TV_REMOTE_CTRL GetTVCodesEEPROM(void);
void SetTVCodesEEPROM(struct TV_REMOTE_CTRL);
bool LoadTVCodesFromEEPROM(void);
struct CurrentValues readAnalogValues(void);
void checkCurrentConsumption(void);
void setDefaultCurrentValues(void);
bool processIRData(uint32_t);
void ProgramSet(uint32_t);
void ProcessSerialInput(void);


/*
 * Lights up only one specific LED of the LED strip.
 * The Index value indicates which LED should be turned on while others remains off.
 * If AllOff is set to true, the Index value is ignored and ALL LEDs are turned off.
 */
void LightUpLED(unsigned int Index, bool AllOff = false)
{
  CRGB Colors[8] = { CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Aqua, CRGB::Blue, CRGB::Purple, CRGB::Pink };

  for (int Idx = 0; Idx < NUM_LEDS; Idx++)
  {
    if (Idx == Index && !AllOff)
    {
        WS2812_LED[Idx] = Colors[Idx];
    } else
    {
      WS2812_LED[Idx] = CRGB::Black;
    }
  }
  FastLED.show();    
  delay(100);
}

/*
 * Just to make sure everything is working - Test function for WS2812 
 * 
 * Startup animation which can be changed as you wish. 
 * 
 * If OkState is set to true, a short "green" blink at the end of the animation indicates that every works as expected. Otherwise a short red blink is done.
 * 
 */
void ShowAnimation(bool OkState)
{
  CRGB Colors[8] = { CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Aqua, CRGB::Blue, CRGB::Purple, CRGB::Pink };
  
  for (int Current = 0; Current < NUM_LEDS; Current++)
  {
    for (int Idx = 0; Idx < NUM_LEDS; Idx++)
    {
      if (Idx == Current)
      {
          WS2812_LED[Idx] = Colors[Idx];
      } else
      {
        WS2812_LED[Idx] = CRGB::Black;
      }
    }
    FastLED.show();    
    delay(100);
  }

  for (int Current = NUM_LEDS - 1; Current >= 0; Current--)
  {
    for (int Idx = 0; Idx < NUM_LEDS; Idx++)
    {
      if (Idx == Current)
      {
          WS2812_LED[Idx] = Colors[Idx];
      } else
      {
        WS2812_LED[Idx] = CRGB::Black;
      }
    }
    FastLED.show();    
    delay(100);
  }    
  
  for (int Current = 0; Current < NUM_LEDS; Current++)
  {
    WS2812_LED[Current] = CRGB::Black;
    FastLED.show();    
  }  

  delay(100);

  if (!OkState)
  {
    WS2812_LED[0] = CRGB::Red;
    WS2812_LED[NUM_LEDS - 1] = CRGB::Red;
    FastLED.show();   
    return;
  }

  WS2812_LED[0] = CRGB::Green;
  WS2812_LED[NUM_LEDS - 1] = CRGB::Green;
  FastLED.show();   
  
  delay(100);
  
  WS2812_LED[0] = CRGB::Black;
  WS2812_LED[NUM_LEDS - 1] = CRGB::Black;
  FastLED.show(); 

  delay(100);
}

/*
 * Enables and setup the ADC (sets reference voltage and prescaler value).
 */
void InitADC(void)
{
  /*
   * REFS1  REFS0   Referenz
   * 0      0       externe Referenz
   * 0      1       interne Referenz: Avcc
   * 1      0       wird beim Mega8 nicht benutzt
   * 1      1       interne Referenz: 2.56 Volt 
  */
  // Select Vref=AVcc
  ADMUX |= (1<<REFS0)|(1<<REFS1);

  /*
   *   ADPS2   ADPS1   ADPS0   Vorteiler
   *   0       0       0         2
   *   0       0       1         2
   *   0       1       0         4
   *   0       1       1         8
   *   1       0       0        16
   *   1       0       1        32
   *   1       1       0        64
   *   1       1       1       128
  */
  //Set prescaller to 128 and enable ADC 
  ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0)|(1<<ADEN);    
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  // Serial used in case of any issues
  mySerial.begin(9600);
  mySerial.println(F("PROJECT:LIVING_WALL_LED_CONTROLLER|VERSION:002|CREATOR:bRanger82"));
  mySerial.println(F("FILE:" __FILE__ "|DATE:" __DATE__ "|IR_LIB_VERSION:" VERSION_IRREMOTE));
  mySerial.flush();
  
  FastLED.addLeds<LED_TYPE, LED_DATA_OUT, COLOR_ORDER>(WS2812_LED, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( WS2812_BRIGHTNESS );

  // activate IR receiver
  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);
  
  // set default PMW output value
  CURRENT_VAL_PWM_OUT = PWM_START_VALUE;

  pinMode(PROGRAMMING_PIN, INPUT);

  // set pinMode and values for PWM output
  digitalWrite(PWM_OUT_1, LOW);
  digitalWrite(PWM_OUT_2, LOW);
    
  analogWrite(PWM_OUT_1, CURRENT_VAL_PWM_OUT);
  analogWrite(PWM_OUT_2, CURRENT_VAL_PWM_OUT);
  
  pinMode(PWM_OUT_1, OUTPUT);
  pinMode(PWM_OUT_2, OUTPUT);

  InitADC();

  bool LoadState = LoadTVCodesFromEEPROM();

  if (!IsProgrammingMode)
  {
    // show awesome LED effect on startup
    ShowAnimation(true);    
  }
}

/*
 * Purpose: for debugging only. Showing the actual read current values on the RS232 interface.
 */
void printSenseValues(void)
{
  mySerial.print(F("PWM_1/2:"));
  mySerial.print(CURRENT_VAL_PWM_OUT);
  mySerial.print(F("|S_1:"));
  mySerial.print(analogRead(SENSE_1));
  mySerial.print(F("|S_2:"));
  mySerial.print(analogRead(SENSE_2));
  mySerial.println("");
  mySerial.flush();
}

/*
 * Based on the CURRENT_VAL_PWM_OUTent PWM output value, it returns the number of LEDs (WS2812) to light up
 * If CURRENT_VAL_PWM_OUTent PWM < 5 (so smaller than the minimum), 254 is return. As -1 does not work for type byte
*/
byte getNumbersOfLEDs(void)
{
  // as CURRENT_VAL_PWM_OUT stores a value between 0 and 255 --> calculate the number of 8 LEDs out of it
  
  if (CURRENT_VAL_PWM_OUT > 5 && CURRENT_VAL_PWM_OUT < 32)
  {
    return 1;  
  } else if (CURRENT_VAL_PWM_OUT >=32 && CURRENT_VAL_PWM_OUT < 63)
  {
    return 2;
  } else if (CURRENT_VAL_PWM_OUT >= 63 && CURRENT_VAL_PWM_OUT < 95)
  {
    return 3;
  } else if (CURRENT_VAL_PWM_OUT >= 95 && CURRENT_VAL_PWM_OUT < 127)
  {
    return 4;
  } else if (CURRENT_VAL_PWM_OUT >= 127 && CURRENT_VAL_PWM_OUT < 159)
  {
    return 5;
  } else if (CURRENT_VAL_PWM_OUT >= 159 && CURRENT_VAL_PWM_OUT < 191)
  {
    return 6;
  } else if (CURRENT_VAL_PWM_OUT >= 191 && CURRENT_VAL_PWM_OUT < 223)
  {
    return 7;
  } else if (CURRENT_VAL_PWM_OUT >= 223)
  {
    return 8;
  }
  // 255 must  be set as data type byte cannot return -1
  return 255;
}

/*
 * For this project, an 8 WS2812 LEDs stip was used. 
 * Based on the CURRENT_VAL_PWM_OUTent PWM output, the number of WS2812 are light up in different color. 
 * E.g. 25 percent -> 2 LEDs of the WS2812 lights up
 *      50 percent -> 4 LEDs of the WS2812 lights up
*/
void ShowPMWValue(void)
{
  byte numberOfLEDs = getNumbersOfLEDs() - 1; // returns number of LEDs - 1 ==> index
  mySerial.print(F("numberOfLEDs:"));
  mySerial.println(numberOfLEDs);
  
  for (byte i = 0; i < NUM_LEDS; i++) 
  {
    if (i <= numberOfLEDs && numberOfLEDs < NUM_LEDS)
    {
      switch(i)
      {
        case 0:
          WS2812_LED[i] = CRGB::Red;
          break;
        case 1: 
          WS2812_LED[i] = CRGB::Orange;
          break;
        case 2:
          WS2812_LED[i] = CRGB::Yellow;
          break;
        case 3:
          WS2812_LED[i] = CRGB::Green;
          break;
        case 4:
          WS2812_LED[i] = CRGB::Aqua;
          break;
        case 5:
          WS2812_LED[i] = CRGB::Blue;
          break;
        case 6:
          WS2812_LED[i] = CRGB::Purple;
          break;
        case 7:
          WS2812_LED[i] = CRGB::Pink;
          break;
        default:
          WS2812_LED[i] = CRGB::Black;
          break;
      }
    } else
    {
      WS2812_LED[i] = CRGB::Black;
    }
  }  
  FastLED.show();

  delay(50);
}

// FadeDown Function, increases light
// value: the value to fadeDown to (e.g. value = 25 --> outputs will be decreased from CURRENT_VAL_PWM_OUT -> 25)
void FadeStepDown(int value)
{
  runIt = true;
  
  //use new value, just in case it needs to be changed in future
  int ToValue = value;
  
  // check for limits
  if (ToValue < PWM_MIN) { ToValue = PWM_MIN; }
  if (ToValue > PWM_MAX) { ToValue = PWM_MAX; }

  while(CURRENT_VAL_PWM_OUT > ToValue)
  {
    // change the time for delay and/or for increasing the value, to change the time until the min. bBTN_FADE_FULL_ONness is reached
    CURRENT_VAL_PWM_OUT-=CURRENT_VAL_PWM_OUT_INC_DW;
    
    analogWrite(PWM_OUT_1, CURRENT_VAL_PWM_OUT);
    analogWrite(PWM_OUT_2, CURRENT_VAL_PWM_OUT);

    ShowPMWValue();
    FastLED.delay(DELAY_STEP_DW);
  }
  runIt = false;  
}

// FadeUp Function, increases light
// value: the value to fadeUp to (e.g. value = 250 --> outputs will be increased from CURRENT_VAL_PWM_OUT -> 250)
void FadeStepUp(int value)
{
  runIt = true;
  
  //use new value, just in case it needs to be changed in future
  int ToValue = value;
  
  // check for limits
  if (ToValue < PWM_MIN) { ToValue = PWM_MIN; }
  if (ToValue > PWM_MAX) { ToValue = PWM_MAX; }
  
  while(CURRENT_VAL_PWM_OUT < ToValue)
  {
    // change the time for delay and/or for increasing the value, to change the time until the max. bBTN_FADE_FULL_ONness is reached
    CURRENT_VAL_PWM_OUT+=CURRENT_VAL_PWM_OUT_INC_UP;
    
    analogWrite(PWM_OUT_1, CURRENT_VAL_PWM_OUT);
    analogWrite(PWM_OUT_2, CURRENT_VAL_PWM_OUT);

    ShowPMWValue();
    FastLED.delay(DELAY_STEP_UP);
  }
  runIt = false;
}

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

/*
 * 
 * Reads the Codes from the EEPROM of the microcontroller. 
 * 
 * If the codes are valid (have been saved before), the IsValid flag is set to true. 
 * So, each time, when this method is called, you should check the IsValid flag.
 * 
 */
struct TV_REMOTE_CTRL GetTVCodesEEPROM(void)
{
  struct TV_REMOTE_CTRL Codes; //Variable to store custom object read from EEPROM.
  
  EEPROM.get(TV_CODES_EEPROM_ADDR, Codes);
  
  if (Codes.TV_FULL_OFF == 0xFFFFFFFF || Codes.TV_STEP_OFF == 0xFFFFFFFF || Codes.TV_STEP_ON == 0xFFFFFFFF || Codes.TV_FULL_ON == 0xFFFFFFFF)
  {
    Codes.IsValid = false;
  } else if (Codes.TV_FULL_OFF == 0 || Codes.TV_STEP_OFF == 0 || Codes.TV_STEP_ON == 0 || Codes.TV_FULL_ON == 0)
  {
    Codes.IsValid = false;
  } else
  {
    Codes.IsValid = true;
  }
  
  return Codes;
}

/*
 * Saves the set Codes (parameter) into the EEPROM of the microcontroller
 */
void SetTVCodesEEPROM(struct TV_REMOTE_CTRL Codes)
{
  EEPROM.put(TV_CODES_EEPROM_ADDR, Codes);
  delay(100);
}

/*
 * Loads the saved TV codes from the EEPROM into the local tv_remote_ctrl structure.
 * If not programmed (IsValid flag is used), it enteres the programming mode.
 */
bool LoadTVCodesFromEEPROM(void)
{
  tv_remote_ctrl = GetTVCodesEEPROM();

  Serial.println(F("Read custom object from EEPROM: "));
  Serial.print(F("TV_FULL_OFF: "));
  Serial.println(tv_remote_ctrl.TV_FULL_OFF);
  Serial.print(F("TV_STEP_OFF: "));
  Serial.println(tv_remote_ctrl.TV_STEP_OFF);
  Serial.print(F("TV_STEP_ON: "));
  Serial.println(tv_remote_ctrl.TV_STEP_ON);
  Serial.print(F("TV_FULL_ON: "));
  Serial.println(tv_remote_ctrl.TV_FULL_ON);
  Serial.println(F("Done reading from EEPROM"));

  if (!tv_remote_ctrl.IsValid)
  {
    IsProgrammingMode = true;
    LightUpLED(0);
    return false;
  }  

  if (digitalRead(PROGRAMMING_PIN) == LOW && !IsProgrammingMode)
  {
    IsProgrammingMode = true;
    LightUpLED(0);
    return false;
  }

  return true;
}

/*
 * 
 * Based on the current running thru the shut resistors for the output channels, a voltage drop is created.
 * This voltage drop is proportial to the current. 
 * See schematic for more information (LM358 is used for the current sense reading).
 * 
 */
struct CurrentValues readAnalogValues(void)
{
  struct CurrentValues tmp;
  const int ArraySize = 5;
  int SENSE_1_VALUES[ArraySize] = {0};
  int SENSE_2_VALUES[ArraySize] = {0};
  for(int Counter = 0; Counter < ArraySize; Counter++)
  {
    SENSE_1_VALUES[Counter] = analogRead(SENSE_1);
    delay(2);
    SENSE_2_VALUES[Counter] = analogRead(SENSE_2);
    delay(2);
  }

  int lt_length = sizeof(SENSE_1_VALUES) / sizeof(SENSE_1_VALUES[0]);
  qsort(SENSE_1_VALUES, lt_length, sizeof(SENSE_1_VALUES[0]), sort_desc);
  qsort(SENSE_2_VALUES, lt_length, sizeof(SENSE_2_VALUES[0]), sort_desc);

  tmp.Channel_One = SENSE_1_VALUES[lt_length / 2];
  tmp.Channel_Two = SENSE_2_VALUES[lt_length / 2];
  
  mySerial.print("1: ");
  mySerial.print(tmp.Channel_One);
  mySerial.print(", 2: ");
  mySerial.println(tmp.Channel_Two);

  return tmp;
}

/*
 * If LED stip is turned ON (PMW > 0), then the current is read after the switch block for reading the IR codes.
 * After some time, the LEDs warm up and the current is increasing. 
 * To avoid higher current consumption (if PWM did not change), the PMW value for the affected channel has to be adapted.
 * 
 * E.g. if PWM is set to 50% and the current is measured -> default current
 * After some on-time, the current of the LED strip increases as it gets warmer
 * This code checks peroidically the actual current consumption 
 * If (actual current consumption > default current * 1.1) => Then decrease PWM duty cycle (hte multiplier 1.1 is just an example to give a little bit of a range)
 * If duty cycle is decreased, the actual current consumption is also decreased, it should get back to the default current consumption.
 * 
 */
void checkCurrentConsumption(void)
{
  // TODO
}

/*
 * 
 * Should be called after the final PWM values is set for the output channels.
 * It sets the default (startup) current consumption for the LED strips.
 * 
 */
void setDefaultCurrentValues(void)
{
  struct CurrentValues changedCurrentValues = readAnalogValues();
  CurrentSense_1 = changedCurrentValues.Channel_One;
  CurrentSense_2 = changedCurrentValues.Channel_Two;
  if (CURRENT_VAL_PWM_OUT == 0 && CURRENT_VAL_PWM_OUT == 0)
  {
    LED_STRIP_ON = false; 
    CurrentReadCounter = 0;
  } else 
  {
    LED_STRIP_ON = true;
  }
}

/*
 * Process when IR code is received. 
 * The main processíng happens here.
 * 
 * Return true, if the PWM value was changed. Otherwise false.
 * 
 */
bool processIRData(uint32_t received_code)
{
  // store CURRENT_VAL_PWM_OUTent value, might be needed in future
  int newValue = CURRENT_VAL_PWM_OUT;

  if (received_code == BTN_EXT_FADE_FULL_OFF || received_code == tv_remote_ctrl.TV_FULL_OFF)
  {
    FadeStepDown(PWM_MIN);
    return true;
  }

  if (received_code == BTN_EXT_FADE_STEP_OFF || received_code == tv_remote_ctrl.TV_STEP_OFF)
  {
    // reduce light by CURRENT_VAL_PWM_OUT_STEP_DW value
    newValue -= CURRENT_VAL_PWM_OUT_STEP_DW;
    FadeStepDown(newValue);
    return true;
  } 

  if (received_code == BTN_EXT_FADE_STEP_ON || received_code == tv_remote_ctrl.TV_STEP_ON)
  {
    // increase light by CURRENT_VAL_PWM_OUT_STEP_UP value
    newValue += CURRENT_VAL_PWM_OUT_STEP_UP;
    FadeStepUp(newValue);
    return true;
  }

  if (received_code == BTN_EXT_FADE_FULL_ON || received_code == tv_remote_ctrl.TV_FULL_ON)
  {
    FadeStepUp(PWM_MAX);
    return true;
  }

  printSenseValues();
  return false;
}

/*
 * Program the new values into the EEPROM.
 */
void ProgramSet(uint32_t RawData)
{
  switch(ProgrammingIndex)
  {
    case 0:
      tv_remote_new.TV_FULL_OFF = RawData;
      delay(2000);
      if (IrReceiver.decode()) 
        IrReceiver.resume();
      LightUpLED((NUM_LEDS / 2) - 1);
      ProgrammingIndex++;
      break;
    case 1:
      tv_remote_new.TV_STEP_OFF = RawData;
      delay(2000);
      if (IrReceiver.decode()) 
        IrReceiver.resume();
      LightUpLED((NUM_LEDS / 2) + 1);
      ProgrammingIndex++;
      break;
    case 2:
      tv_remote_new.TV_STEP_ON = RawData;
      delay(2000);
      if (IrReceiver.decode()) 
        IrReceiver.resume();
      LightUpLED(NUM_LEDS - 1);
      ProgrammingIndex++;
      break;
    case 3:
      tv_remote_new.TV_FULL_ON = RawData;
      delay(50);   
      IsProgrammingMode = false;
      ProgrammingIndex = 0;
      SetTVCodesEEPROM(tv_remote_new);
      delay(50);
      tv_remote_ctrl = GetTVCodesEEPROM();
      delay(50);
      ShowAnimation(tv_remote_new.IsValid);
      delay(50);
      if (IrReceiver.decode()) 
        IrReceiver.resume();
      delay(50);
      break;
    default:
      IsProgrammingMode = false;
      ProgrammingIndex = 0;
  }
}

/*
 * Process RS232 input
 */
void ProcessSerialInput(void)
{
  if (mySerial.available() > 0)
  {
    char c = mySerial.read();

    if (c == '1')
    {
      printSenseValues();
    }
    
    mySerial.flush();
  }
}

/*
 * The loop function runs over and over again, forever and ever ...
 */ 
void loop() 
{
  // need a break, need a kitk ... ahm delay :)
  delay(DELAY_LOOP_RUN);

  ProcessSerialInput();
  
  /*
   * Process IR data if received
   */
  if (IrReceiver.decode()) 
  {
    uint32_t RawData = IrReceiver.decodedIRData.decodedRawData;
    
    mySerial.print(F("REC'D:"));
    mySerial.println(RawData);
    mySerial.flush();

    if (IsProgrammingMode)
    {
      ProgramSet(RawData);
    } else
    {
      if (processIRData(RawData))
      {
        setDefaultCurrentValues();
      }
    }
    
    // ready for receiving the next value
    IrReceiver.resume(); // Receive the next value
  }
}
