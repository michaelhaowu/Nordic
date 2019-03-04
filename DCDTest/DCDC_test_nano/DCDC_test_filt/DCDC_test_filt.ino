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

const int SENSOR_PIN = 0; // center pin of the potentiometer
const int analogIn = 1;
int mVperAmp = 40;
int RawValue= 0;
double Voltage = 0;
double Amps = 0;
int ACSoffset = 2500;
 
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
int RPWM_En = 22; // Arduino PWM output pin 8; connect to IBT-2 pin 3 (R_EN)
int LPWM_En = 23; // Arduino PWM output pin 9; connect to IBT-2 pin 4 (L_EN)
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
}
 
void loop()
{
  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1023.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp); //

  delay(10);

  Serial.print("\t Amps = "); // shows the voltage measured 
  Serial.println(Amps,3); // the '3' after voltage allows you to display 3 digits after decimal point
  //Serial.print("\t Raw = ");
  //Serial.println(RawValue,3);
  int sensorValue = analogRead(SENSOR_PIN);

  sensorValue = sensorValue < 40 ? 40 : sensorValue;
  sensorValue = sensorValue > 960 ? 960 : sensorValue;

  sensorValue = abs(sensorValue - 500) < 5 ? 500 : sensorValue;
 
  // sensor value is in the range 0 to 1023
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (sensorValue < (500))
  {
    // reverse rotation
    int reversePWM = (500  - sensorValue)*(255.0 / 460.0); 
    //Serial.print(reversePWM, DEC);
    //Serial.print("     ");
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
    unsigned int forwardPWM = (sensorValue - 500)*(255.0 / 460.0);
    //Serial.print(forwardPWM, DEC);
    //Serial.print("     ");
    //int forwardPWM = 255;
    digitalWrite(RPWM_En, LOW);
    digitalWrite(LPWM_En, HIGH);
    analogWrite(LPWM_Output, forwardPWM);
    analogWrite(RPWM_Output, 0);
    digitalWrite(LED_BUILTIN, LOW);
  }
  //Serial.print(sensorValue, DEC);
  //Serial.print("\n");
}
