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
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//LED Globals
#define LED_PIN 3
#define NUMPIXELS 5
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

//Temperature Sensor Globals
OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)
float celsius = 0;
unsigned long convertMAX31920Time = millis();
bool convertTempFromMAX31820 = true;
bool waitConvertMAX31820Delay = false;
bool readScratchpadFromMAX31820 = false;

//Current Sensor Globals
#define CURRENT_PIN A7
int mVperAmp = 40; // Current Sensor Scale Value
int ACSoffset = 2500; // Current Sensor Offset
int RawValue= 0;
double Voltage = 0;
double Amps = 0;

//Desired Temperature Set Potentiometer Globals
//#define POT_PIN    5
int SENSOR_PIN = 0; // center pin of the potentiometer
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

  pixels.begin();
  
}
 
void loop()
{
  
  byte addr[8];
  float sensorValue = analogRead(SENSOR_PIN);
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

  //check if the temperature sensor is on the one wire line
  ds.search(addr);
  /*
  checkForAddresses(addr);
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
  */

  //read temperature data and convert to current temperature
  obtainTempData(addr);

  //read the potentiometer value and convert to a desired temperature
  //potValue = analogRead(POT_PIN);
  desiredTemp = sensorValue/1023*60 + 10;

  //read the current sensor value and convert to current 
  RawValue = analogRead(CURRENT_PIN);
  Voltage = (RawValue / 1023.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  if ((millis() - displayMillis) > 10000)
  {
    display.clearDisplay();
    displayText("Temp: ", celsius, 0);
    fanActuate(celsius);
    displayText("Desired: ", desiredTemp, 10);
    displayText("Current: ", Amps, 20);
    displayMillis = millis();
  }

  showLEDColour(celsius);
  
}

void displayText(char *descriptor, float value, int cursorY) {

  display.setTextSize(1.5); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, cursorY);
  
  display.print(descriptor);
  display.print(value);
  
  display.display();      // Show initial text
 
}

void checkForAddresses(byte *addr) {
    while ( !ds.search(addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
    }
    return;
}

void obtainTempData(byte *addr) {
  byte data[12];

  if (convertTempFromMAX31820 == true)
  {
      
      ds.reset();
      ds.select(addr);
      ds.write(0x44);        // start conversion, use ds.write(0x44,1) with parasite power on at the end

      convertTempFromMAX31820 = false;
      waitConvertMAX31820Delay = true;
      convertMAX31920Time = millis();
      Serial.println("ONE");
  }
  else if (waitConvertMAX31820Delay ==  true and (millis() - convertMAX31920Time) > 1000)
  {
      waitConvertMAX31820Delay = false;
      readScratchpadFromMAX31820 = true;  
      Serial.println("TWO");
  }
  else if (readScratchpadFromMAX31820 == true)
  {
      byte present = 0;
      present = ds.reset();
      ds.select(addr);    
      ds.write(0xBE);         // Read Scratchpad

      readScratchpadFromMAX31820 = false; 
      convertTempFromMAX31820 = true; 

      saveTempData(data);
      celsius = convertTempData_HumanReadable(data);

      Serial.println("THREE");
 
  }
  else{

  }
  
}

void saveTempData(byte *data) {
  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }  
}

float convertTempData_HumanReadable(byte *data) {
  byte type_s;

  
  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  float celsius = (float)raw / 16.0;
  return celsius;
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


void showLEDColour(float celsius){
  for(int i=0;i<NUMPIXELS;i++){
    pixels.setPixelColor(i, pixels.Color(0,150,0)); // Moderately bright green color.
    pixels.show();
  }
}

void fanActuate(float celsius){
  if(celsius > 25)
  {
    digitalWrite(4, HIGH);
  }
  else
  {
    digitalWrite(4, LOW);
  }
}

