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

OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)
float celsius = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

#define POT_PIN    5

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
 
int SENSOR_PIN = 0; // center pin of the potentiometer
 
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
int RPWM_En = 3;//22; // Arduino PWM output pin 8; connect to IBT-2 pin 3 (R_EN)
int LPWM_En = 4;//23; // Arduino PWM output pin 9; connect to IBT-2 pin 4 (L_EN)

void setup()
{
  Serial.begin(9600);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  pinMode(RPWM_En, OUTPUT);
  pinMode(LPWM_En, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
}
 
void loop()
{

  byte data[12];
  byte addr[8];
  float celsius;
  float potValue;
  float desiredTemp;
  
  int sensorValue = analogRead(SENSOR_PIN);
  Serial.println(sensorValue);
 
  // sensor value is in the range 0 to 1023
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (sensorValue < 512)
  {
    // reverse rotation
    unsigned int reversePWM = (511 - sensorValue) / 2;  
    //int reversePWM = 255;
    digitalWrite(RPWM_En, HIGH);
    digitalWrite(LPWM_En, LOW);
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, reversePWM);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    // forward rotation
    unsigned int forwardPWM = (sensorValue - 512) / 2;
    //int forwardPWM = 255;
    digitalWrite(RPWM_En, LOW);
    digitalWrite(LPWM_En, HIGH);
    analogWrite(LPWM_Output, forwardPWM);
    analogWrite(RPWM_Output, 0);
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.print(sensorValue, DEC);
  Serial.print("\n");

  checkForAddresses(addr);

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  // Serial.println();

  obtainTempData(addr);
  saveTempData(data);
  celsius = convertTempData_HumanReadable(data);

  potValue = analogRead(POT_PIN);
  desiredTemp = potValue/1023*60 + 10;
  

  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");

  //display.clearDisplay();

  //displayText("Temp: ", celsius, 0);
  //displayText("Desired: ", desiredTemp, 10);
  //delay(100);
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
      //delay(250);
    }
    return;
}

void obtainTempData(byte *addr) {
  byte present = 0;
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, use ds.write(0x44,1) with parasite power on at the end

  //delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
}

void saveTempData(byte *data) {
  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }  
}

float convertTempData_HumanReadable(byte *data) {
  byte type_s;

  
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
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

