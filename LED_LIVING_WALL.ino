#include <IRremote.h>
#include <FastLED.h>
#include <SoftwareSerial.h>

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
// button on the remote control to set PWM output to MIN
const unsigned long BTN_FADE_FULL_OFF = 551547150;
// button on the remote control to decrease PWM output by CURRENT_VAL_PWM_OUT_STEP_DW
const unsigned long BTN_FADE_STEP_OFF = 551489010;
// button on the remote control to increase PWM output by CURRENT_VAL_PWM_OUT_STEP_UP
const unsigned long BTN_FADE_STEP_ON = 551509410;
// button on the remote control to set PWM output to MAX
const unsigned long BTN_FADE_FULL_ON = 551514510;


SoftwareSerial mySerial(0, 1); // RX, TX

// the IR receiver read data pin
#define RECV_PIN                     3

// define two different outputs, so each output could be controlled seperatly (in this sketch both are controlled the same way)
#define PWM_OUT_1                    5
#define PWM_OUT_2                    6
#define SENSE_1                      A1 // CURRENT_VAL_PWM_OUTent sense value of PWM out channel 1
#define SENSE_2                      A0 // CURRENT_VAL_PWM_OUTent sense value of PWM out channel 2

#define LED_DATA_OUT                 9
#define NUM_LEDS                     8
#define LED_TYPE                     WS2811
#define COLOR_ORDER                  RGB
#define UPDATES_PER_SECOND           100
#define WS2812_BRIGHTNESS            7
#define COLOR_ORDER                  RGB
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

// using library for receiving and processing IR signals
IRrecv irrecv(RECV_PIN);
decode_results results;

// if a process is already running this variable is set to true (e.g. while lightUpSlow is running)
// implemented, just in case a later implemented interrupt needs this status information
volatile bool runIt = false;
// the CURRENT_VAL_PWM_OUTent output value for the OUTs
volatile int CURRENT_VAL_PWM_OUT = 0;

int CurrentSense_1 = 0;
int CurrentSense_2 = 0;
byte CurrentReadCounter = 0;
bool LED_STRIP_ON = false;


struct CurrentValues {
    int Channel_One, Channel_Two;
};

struct CurrentValues readAnalogValues(void);
byte getNumbersOfLEDs(void);
void ShowPMWValue(void);
void FadeStepDown(int value);
void FadeStepUp(int value);
void printSenseValues(void);
void sample_led(void);
void checkCurrentConsumption(void);
void setDefaultCurrentValues(void);
bool processIRData(unsigned long);

/*Just to make sure everything is working - Test function for WS2812 */
void sample_led(void)
{
  WS2812_LED[0] = CRGB::Red;
  WS2812_LED[1] = CRGB::Orange;
  WS2812_LED[2] = CRGB::Yellow;
  WS2812_LED[3] = CRGB::Green;
  WS2812_LED[4] = CRGB::Aqua;
  WS2812_LED[5] = CRGB::Blue;
  WS2812_LED[6] = CRGB::Purple;
  WS2812_LED[7] = CRGB::Pink;
  
  FastLED.show();
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  // Serial used in case of any issues
  mySerial.begin(9600);
  mySerial.println(F("PROJECT:LIVING_WALL_LED_CONTROLLER|VERSION:001|CREATOR:bRanger82"));
  mySerial.flush();
  
  FastLED.addLeds<LED_TYPE, LED_DATA_OUT, COLOR_ORDER>(WS2812_LED, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( WS2812_BRIGHTNESS );
  // show awesome LED effect on startup
  sample_led();
  
  // activate IR receiver
  irrecv.enableIRIn(); // Start the receiver
  
  // set default PMW output value
  CURRENT_VAL_PWM_OUT = PWM_START_VALUE;

  // set pinMode and values for PWM output
  pinMode(PWM_OUT_1, OUTPUT);
  pinMode(PWM_OUT_2, OUTPUT);
  analogWrite(PWM_OUT_1, CURRENT_VAL_PWM_OUT);
  analogWrite(PWM_OUT_2, CURRENT_VAL_PWM_OUT);
}

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
  const int readCounterMax = 5;
  tmp.Channel_One = 0;
  tmp.Channel_Two = 0;
  
  for (int readCounter = 0; readCounter <= readCounterMax; readCounter++)
  {
    tmp.Channel_One += analogRead(SENSE_1);
    tmp.Channel_Two += analogRead(SENSE_2);
  }
  tmp.Channel_One = tmp.Channel_One / readCounterMax;
  tmp.Channel_Two = tmp.Channel_Two / readCounterMax;

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
  if (++CurrentReadCounter > 250)
  {
    struct CurrentValues tmp = readAnalogValues();
    if (CurrentSense_1 > tmp.Channel_One * 1.1)
    {
      mySerial.println(F("CH_1_C"));
      // TBD adapt duty cycle
    }
    if (CurrentSense_2 > tmp.Channel_Two * 1.1)
    {
      mySerial.println(F("CH_2_C"));
      // TBD adapt duty cycle
    }
    CurrentReadCounter = 0;
  } 
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
bool processIRData(unsigned long received_code)
{
  // store CURRENT_VAL_PWM_OUTent value, might be needed in future
  int newValue = CURRENT_VAL_PWM_OUT;
  bool pmw_value_changed = true;
  switch(received_code)
  {
    // name your IR receiver code to your needs
    case BTN_FADE_FULL_OFF:
      // fade to minimum brightness
      FadeStepDown(PWM_MIN);
      break;
    case BTN_FADE_STEP_OFF:
      // reduce light by CURRENT_VAL_PWM_OUT_STEP_DW value
      newValue -= CURRENT_VAL_PWM_OUT_STEP_DW;
      FadeStepDown(newValue);
      break;
    case BTN_FADE_STEP_ON:
      // increase light by CURRENT_VAL_PWM_OUT_STEP_UP value
      newValue += CURRENT_VAL_PWM_OUT_STEP_UP;
      FadeStepUp(newValue);
      break;
    case BTN_FADE_FULL_ON:
      // fade to full brightness
      FadeStepUp(PWM_MAX);
      break;
    default:
      pmw_value_changed = false;
      printSenseValues();
      break;
  }  
  return pmw_value_changed;
}

// the loop function runs over and over again, forever and ever ...
void loop() 
{
  // need a break, need a kitk ... ahm delay :)
  delay(DELAY_LOOP_RUN);

  /*
   * If the LED strip is on (PWM duty cycle > 0%), check and adapt the current consumption by changing the PWM duty-cycle
  */
  if (LED_STRIP_ON)
  {
    checkCurrentConsumption();
  }

  /*
   * Process IR data if received
   */
  if (irrecv.decode(&results)) 
  {
    // For debugging only
    mySerial.print(F("REC'D:"));
    mySerial.println(results.value);
    mySerial.flush();
    
    /*
     * If the PWM value has changed, the default current values have to be read-out after the final PWM duty cycle was set.
     */
    if (processIRData(results.value))
    {
      setDefaultCurrentValues();
    }
    
    // ready for receiving the next value
    irrecv.resume(); // Receive the next value
  }
}
