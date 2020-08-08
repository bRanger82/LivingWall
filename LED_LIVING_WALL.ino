#include <IRremote.h>
#include <FastLED.h>
#include <SoftwareSerial.h>

/*
 * ATMETA328P pin mapping.
 * 
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
 * 
 * *) Pin means physical pin number.
*/


// the IR receiver read data pin
#define RECV_PIN         3

// define two different outputs, so each output could be controlled seperatly (in this sketch both are controlled the same way)
#define PWM_OUT_1          5
#define PWM_OUT_2          6
#define SENSE_1            A1
#define SENSE_2            A0

#define LED_DATA_OUT       9
#define NUM_LEDS           8
#define LED_TYPE           WS2811
#define COLOR_ORDER        RGB
#define UPDATES_PER_SECOND 100
#define BRIGHTNESS         7
CRGB WS2812_LED[NUM_LEDS];

// defines the parameters of this program (start value for the outputs, in- and decrease steps, ...)
#define CURR_START     0  // AnalogWrite value when startup 
#define PWM_MAX      250  // Maximum value for AnalogWrite (PWM to LED)
#define PWM_MIN        0  // Minimum value for AnalogWrite (PWM to LED)
#define CURR_INC_UP    1  // Step-up value (PWM to LED) 
#define CURR_INC_DW    1  // Step-down value (PWM to LED)
#define CURR_STEP_UP  25  // Increase value for Button fade-up 
#define CURR_STEP_DW  25  // Decrease value for Button fade-down
#define DELAY_STEP_UP 25  // Delay between fade-up (from current value to the new value, using steps given as CURR_STEP_UP)
#define DELAY_STEP_DW 25  // Delay between fade-down (from current value to the new value, using steps given as CURR_STEP_DW)


// defines the delay length for each loop() run 
#define DELAY_LOOP_RUN 50

// using library for receiving and processing IR signals
IRrecv irrecv(RECV_PIN);
decode_results results;

// fast-back button on the remote control
const unsigned long left = 551547150;
// play button on the remote control
const unsigned long leftMid = 551489010;
// pause button on the remote control
const unsigned long rightMid = 551509410;
// fast-forward button on the remote control
const unsigned long right = 551514510;

// if a process is already running this variable is set to true (e.g. while lightUpSlow is running)
// implemented, just in case a later implemented interrupt needs this status information
volatile bool runIt = false;
// the current output value for the OUTs
volatile int curr = 0;

#define COLOR_ORDER RGB

byte getNumbersOfLEDs(void);
void ShowPMWValue(void);
void FadeStepDown(int value);
void FadeStepUp(int value);
void printSenseValues(void);
void sample_led(byte);

/*Just to make sure everything is working - Test function for WS2812 */
void sample_led(byte show_id = 0)
{
  if (0 == show_id)
  {
    for (int i = 0; i < NUM_LEDS; i++) 
    {
      if (i % 3 == 0)
      {
        WS2812_LED[i] = CRGB::Green;
      } else if (i % 2 == 0)
      {
        WS2812_LED[i] = CRGB::Blue;
      } else
      {
        WS2812_LED[i] = CRGB::Red;
      }
    }  
  } else
  {
    for (int i = 0; i < NUM_LEDS; i++) 
    {
      if (i == (int)show_id)
      {
        WS2812_LED[i] = CRGB::Green;
      } else
      {
        WS2812_LED[i] = CRGB::Black;
      }
    }
  }
  
  FastLED.show();
  
}

SoftwareSerial mySerial(0, 1); // RX, TX

// the setup function runs once when you press reset or power the board
void setup() 
{
  mySerial.begin(9600);
  mySerial.println(F("PROJECT:LIVING_WALL_LED_CONTROLLER|VERSION:001|CREATOR:bRanger82"));
  mySerial.flush();
  FastLED.addLeds<LED_TYPE, LED_DATA_OUT, COLOR_ORDER>(WS2812_LED, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  sample_led(0);
  // activate IR receiver
  irrecv.enableIRIn(); // Start the receiver
  
  // when turned on the value is 0
  curr = CURR_START;
  // Define pins, currently only PWM_OUT_2 is used
  pinMode(PWM_OUT_1, OUTPUT);
  pinMode(PWM_OUT_2, OUTPUT);

  analogWrite(PWM_OUT_1, curr);
  analogWrite(PWM_OUT_2, curr);
}

// print PWM data and current-sensor resistor values via serial  
void printSenseValues(void)
{
  mySerial.print(F("PWM_OUT_1:"));
  mySerial.print(analogRead(PWM_OUT_1));
  mySerial.print(F("|PWM_OUT_2:"));
  mySerial.print(analogRead(PWM_OUT_2));
  mySerial.print(F("|SENSE_1:"));
  mySerial.print(analogRead(SENSE_1));
  mySerial.print(F("|SENSE_2:"));
  mySerial.print(analogRead(SENSE_2));
  mySerial.println("");
  mySerial.flush();
}

/*
 * Based on the current PWM output value, it returns the number of LEDs (WS2812) to light up
 * If current PWM < 5 (so smaller than the minimum), 254 is return. As -1 does not work for type byte
*/
byte getNumbersOfLEDs(void)
{
  if (curr > 5 && curr < 32)
  {
    return 1;  
  } else if (curr >=32 && curr < 63)
  {
    return 2;
  } else if (curr >= 63 && curr < 95)
  {
    return 3;
  } else if (curr >= 95 && curr < 127)
  {
    return 4;
  } else if (curr >= 127 && curr < 159)
  {
    return 5;
  } else if (curr >= 159 && curr < 191)
  {
    return 6;
  } else if (curr >= 191 && curr < 223)
  {
    return 7;
  } else if (curr >= 223)
  {
    return 8;
  }
  return 255;
}

/*
 * For this project, an 8 WS2812 LEDs stip was used. 
 * Based on the current PWM output, the number of WS2812 are light up in different color. 
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
      if (i % 3 == 0)
      {
        WS2812_LED[i] = CRGB::Green;
      } else if (i % 2 == 0)
      {
        WS2812_LED[i] = CRGB::Blue;
      } else
      {
        WS2812_LED[i] = CRGB::Red;
      }  
    } else
    {
      WS2812_LED[i] = CRGB::Black;
    }
  }  
  FastLED.show();
  
}
// FadeDown Function, increases light
// value: the value to fadeDown to (e.g. value = 25 --> outputs will be decreased from curr -> 25)
void FadeStepDown(int value)
{
  runIt = true;
  
  //use new value, just in case it needs to be changed in future
  int ToValue = value;
  
  // check for limits
  if (ToValue < PWM_MIN) { ToValue = PWM_MIN; }
  if (ToValue > PWM_MAX) { ToValue = PWM_MAX; }

  while(curr > ToValue)
  {
    // change the time for delay and/or for increasing the value, to change the time until the min. brightness is reached
    curr-=CURR_INC_DW;
    
    analogWrite(PWM_OUT_1, curr);
    analogWrite(PWM_OUT_2, curr);

    ShowPMWValue();
    FastLED.delay(DELAY_STEP_DW);
  }
  runIt = false;  
}

// FadeUp Function, increases light
// value: the value to fadeUp to (e.g. value = 250 --> outputs will be increased from curr -> 250)
void FadeStepUp(int value)
{
  runIt = true;
  
  //use new value, just in case it needs to be changed in future
  int ToValue = value;
  
  // check for limits
  if (ToValue < PWM_MIN) { ToValue = PWM_MIN; }
  if (ToValue > PWM_MAX) { ToValue = PWM_MAX; }
  
  while(curr < ToValue)
  {
    // change the time for delay and/or for increasing the value, to change the time until the max. brightness is reached
    curr+=CURR_INC_UP;
    
    analogWrite(PWM_OUT_1, curr);
    analogWrite(PWM_OUT_2, curr);

    ShowPMWValue();
    FastLED.delay(DELAY_STEP_UP);
  }
  runIt = false;
}

// the loop function runs over and over again forever
void loop() 
{
  // need a break, need a kitk ... ahm delay :)
  delay(DELAY_LOOP_RUN);
  
  if (irrecv.decode(&results)) 
  {
    mySerial.print(F("RECEIVED_DATA:"));
    mySerial.println(results.value);
    mySerial.flush();
    // store current value, might be needed in future
    int newValue = curr;
    switch(results.value)
    {
      // name your IR receiver code to your needs
      case left:
        // fade to minimum brightness
        FadeStepDown(PWM_MIN);
        break;
      case leftMid:
        // subtract the step down value to the current value 
        newValue -= CURR_STEP_DW;
        FadeStepDown(newValue);
        break;
      case rightMid:
        // add the step up value to the current value 
        newValue += CURR_STEP_UP;
        FadeStepUp(newValue);
        break;
      case right:
        // fade to full brightness
        FadeStepUp(PWM_MAX);
        break;
      default:
        printSenseValues();
        break;
    }
    
    // ready for receiving the next value
    irrecv.resume(); // Receive the next value
  }
}
