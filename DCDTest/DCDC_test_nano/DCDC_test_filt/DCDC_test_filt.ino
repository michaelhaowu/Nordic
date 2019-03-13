/*
IBT-2 Motor Control Board driven by Arduino.
 
Speed and direction controlled by a potentiometer attached to analog input 0.
One side pin of the potentiometer (either one) to ground; the other side pin to +5V
 
Connection to the IBT-2 board:
IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM)
IBT-2 pin 2 (LPWM) to Arduino pin 6(PWM)
IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
IBT-2 pin 8 (GND) to Arduino GND
IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


//Peltier Temperature Sensor Globals and Water Temperature Sensor Globals
#define ALL_PELT_TEMP_PIN 2
OneWire  oneWire(ALL_PELT_TEMP_PIN);  // on pin 2 (a 4.7K resistor is necessary)
DallasTemperature tempSensors(&oneWire);
float tempHotPeltier = 0;
float tempColdPeltier = 0;
float waterTemp = 0;

//Current Sensor Globals
#define CURRENT_PIN A7
int mVperAmp = 40; // Current Sensor Scale Value
int ACSoffset = 2500; // Current Sensor Offset
int RawValue= 0;
double Voltage = 0;
double Amps = 0;

#define VOLTAGE_PIN 11
float batteryVoltage = 0;

//Desired Temperature Set Potentiometer Globals
//#define POT_PIN    5
int POT_PIN = 0; // center pin of the potentiometer
float potValue;
float desiredTemp;
int RPWM_Output = 5;  
int LPWM_Output = 6; 
int RPWM_En = 11; //22; // Arduino PWM output pin 8; connect to IBT-2 pin 3 (R_EN)
int LPWM_En = 11; //23; // Arduino PWM output pin 9; connect to IBT-2 pin 4 (L_EN)

//Display Globals
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    7    // Used to be Pin 11 but Pin 11 is now used by LPWM_Output
#define OLED_CS    12
#define OLED_RESET 13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

unsigned long displayMillis = millis();

#define FAN_PIN 4
float fanOnTrigger = 25;

void setup()
{
  Serial.begin(9600);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  pinMode(RPWM_En, OUTPUT);
  pinMode(LPWM_En, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  setPwmFrequency(RPWM_Output, 8);
  setPwmFrequency(LPWM_Output, 8);

  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  tempSensors.begin();

}
 
void loop()
{
  
  float sensorValue = analogRead(POT_PIN);
  Serial.println("Potentiometer");
  Serial.println(sensorValue);
 
  // sensor value is in the range 0 to 1023
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (sensorValue < 512)
  {
    // reverse rotation
    unsigned int reversePWM = (500  - sensorValue)*(255.0 / 460.0); //(511 - sensorValue) / 2;  
    //int reversePWM = 255;
    digitalWrite(RPWM_En, HIGH);
    //digitalWrite(LPWM_En, LOW);
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, reversePWM);
    //digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Reverse PWM: ");
    Serial.println(reversePWM);
  }
  else
  {
    // forward rotation
    unsigned int forwardPWM = (sensorValue - 500)*(255.0 / 460.0);//(sensorValue - 512) / 2;
    //int forwardPWM = 255;
    //digitalWrite(RPWM_En, LOW);
    digitalWrite(LPWM_En, HIGH);
    analogWrite(LPWM_Output, forwardPWM);
    analogWrite(RPWM_Output, 0);
    //digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Forward PWM: ");
    Serial.println(forwardPWM);
  }
  //Serial.print(sensorValue, DEC);
  //Serial.print("\n");

  // Send the command to get temperatures
  tempSensors.requestTemperatures();
  tempHotPeltier = tempSensors.getTempCByIndex(0);
  tempColdPeltier = tempSensors.getTempCByIndex(1);
  waterTemp = tempSensors.getTempCByIndex(2);

  //read the potentiometer value and convert to a desired temperature
  //potValue = analogRead(POT_PIN);
  desiredTemp = sensorValue/1023*60 + 10;

  //read the current sensor value and convert to current 
  RawValue = analogRead(CURRENT_PIN);
  Voltage = (RawValue / 1023.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  if (Amps > 0)
    fanActuate(tempColdPeltier);
  else
    fanActuate(tempHotPeltier);
  
  //read the voltage sensor value and convert to current 
  batteryVoltage = digitalRead(VOLTAGE_PIN);

  if ((millis() - displayMillis) > 10000)
  {
    display.clearDisplay();
    displayText("Hot Temp: ", tempHotPeltier, 0);
    displayText("Cold Temp: ", tempColdPeltier, 10);
    displayText("Water Temp: ", waterTemp, 20);
    displayText("Desired: ", desiredTemp, 30);
    displayText("Current: ", Amps, 40);
    displayText("Voltage: ", batteryVoltage, 50); 
    displayMillis = millis();
  }
  
}

void displayText(char *descriptor, float value, int cursorY) {

  display.setTextSize(1.5); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, cursorY);
  
  display.print(descriptor);
  display.print(value);
  
  display.display();      // Show initial text
 
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void fanActuate(float celsius){
  if(celsius > fanOnTrigger)
  {
    digitalWrite(FAN_PIN, HIGH);
  }
  else
  {
    digitalWrite(FAN_PIN, LOW);
  }
}

