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
 
int SENSOR_PIN = 0; // center pin of the potentiometer
 
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
int RPWM_En = 22; // Arduino PWM output pin 8; connect to IBT-2 pin 3 (R_EN)
int LPWM_En = 23; // Arduino PWM output pin 9; connect to IBT-2 pin 4 (L_EN)

void setup()
{
  Serial.begin(9600);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  pinMode(RPWM_En, OUTPUT);
  pinMode(LPWM_En, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}
 
void loop()
{
  int sensorValue = analogRead(SENSOR_PIN);
 
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
  // Serial.print(sensorValue, DEC);
  // Serial.print("\n");
}
