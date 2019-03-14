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

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Peltier Temperature Sensor Globals and Water Temperature Sensor Globals
#define PELT_PIN A4
#define CUP_PIN 8
OneWire  oneWirePelt(PELT_PIN);  // on pin 2 (a 4.7K resistor is necessary)
OneWire  oneWireCup(CUP_PIN);  // on pin 8 (a 4.7K resistor is necessary)
DallasTemperature peltTemp(&oneWirePelt);
DallasTemperature cupTemp(&oneWireCup);
float peltTemperature = 0;
float cupTemperature = 0;

//Current Sensor Globals
#define CURRENT_PIN A2
float mVperAmp = 40.0; // Current Sensor Scale Value
int ACSoffset = 2500; // Current Sensor Offset
int RawValue = 0;
double Voltage = 0;
double Amps = 0;

#define VOLTAGE_PIN A1
float batteryVoltage = 0;

//Desired Temperature Set Potentiometer Globals
//#define POT_PIN    5
int POT_PIN = A0; // center pin of the potentiometer
float potValue;
float desiredTemp = 55;
int RPWM_Output = 5;  
int LPWM_Output = 6; 
int RLPWM_En = 11; //22; // Arduino PWM output pin 8; connect to IBT-2 pin 3 (R_EN)

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
  pinMode(RLPWM_En, OUTPUT);
  setPwmFrequency(RPWM_Output, 8);
  setPwmFrequency(LPWM_Output, 8);

  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  peltTemp.begin();
  cupTemp.begin();

}
 
void loop()
{
  
  int sensorValue = analogRead(POT_PIN);
  //Serial.print("Potentiometer:  ");
  //Serial.println(sensorValue);
 
  // sensor value is in the range 0 to 1023
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  /*
  if (sensorValue < 512)
  {
    // reverse rotation
    unsigned int reversePWM = (511 - sensorValue) / 2; 
    digitalWrite(RLPWM_En, HIGH);
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, reversePWM);
    //Serial.print("Reverse PWM: ");
    //Serial.println(reversePWM);
  }
  else
  {
    // forward rotation
    unsigned int forwardPWM = (sensorValue - 512) / 2;
    digitalWrite(RLPWM_En, HIGH);
    analogWrite(LPWM_Output, forwardPWM);
    analogWrite(RPWM_Output, 0);
    //Serial.print("Forward PWM: ");
    //Serial.println(forwardPWM);
  }
  //Serial.print(sensorValue, DEC);
  //Serial.print("\n");
  */

  //COOLING
  unsigned int reversePWM = (511 - sensorValue) / 2; 
  digitalWrite(RLPWM_En, HIGH);

  if(millis() < 1500)
  {
    reversePWM = 255*(millis()/1500)*(abs(cupTemperature - 55))/(55 - startTemperature);
    Serial.println("Ramp");
  }
  else
  
    reversePWM = 255*(abs(cupTemperature - 55))/(55 - startTemperature);;
    Serial.println("Ramp DONE");
  }

  if(abs(cupTemperature - 55) < 1) 
  {
  }
  else if((cupTemperature - 55) > 1) //cool down
  {
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, forwardPWM);
  }
  else{
    analogWrite(LPWM_Output, forwardPWM);
    analogWrite(RPWM_Output, 0);
  }

  //read the potentiometer value and convert to a desired temperature
  //potValue = analogRead(POT_PIN);
  desiredTemp = sensorValue/1023*60 + 10;

  //read the current sensor value and convert to current 
  RawValue = analogRead(CURRENT_PIN);
  Voltage = ((RawValue-511) /511.0) * 2500; // Gets you mV
  Amps = ((Voltage) / mVperAmp);

  //Serial.print("\t Amps = "); // shows the voltage measured 
  //Serial.println(RawValue); // the '3' after voltage allows you to display 3 digits after decimal point

  if (Amps > 0)
    fanActuate(30);
  else
    fanActuate(30);
  
  //read the voltage sensor value and convert to current 
  batteryVoltage = analogRead(VOLTAGE_PIN)/1023.0*20;
  Serial.print("Voltage = "); // shows the voltage measured 
  Serial.println(batteryVoltage); // the '3' after voltage allows you to display 3 digits after decimal point

  if ((millis() - displayMillis) > 10000)
  {
    // Send the command to get temperatures
    peltTemp.requestTemperatures();
    peltTemperature = peltTemp.getTempCByIndex(0);
    //Serial.print("Temp ");
    //Serial.print(peltTemperature, 4);
    
    cupTemp.requestTemperatures();
    cupTemperature = cupTemp.getTempCByIndex(0);
    
    //Serial.print("      Temp ");
    //Serial.println(cupTemperature, 4);
    
    display.clearDisplay();
    displayText("Peltier Temp: ", peltTemperature, 0);
    displayText("Water Temp: ", cupTemperature, 10);
    displayText("Desired: ", desiredTemp, 20);
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
