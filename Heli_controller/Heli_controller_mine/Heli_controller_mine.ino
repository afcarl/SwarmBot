/**
This sketch is straight from this website, but modified for my own helicopter:
http://www.kerrywong.com/2012/08/27/reverse-engineering-the-syma-s107g-ir-protocol/
It is designed to control the syma s107/s107g helicopter via three potentiometers
and an IR LED. It uses the TimerOne library:
http://www.arduino.cc/playground/code/timer1
*/

#include <TimerOne.h>

//comment this out to see the demodulated waveform
//it is useful for debugging purpose.
#define MODULATED 1 
 
const unsigned int IR_PIN = 3;//The IR pin
const unsigned int THROTTLE_PIN = A0;//The analog pin for the throttle pot
const unsigned int YAW_PIN = A1;//The analog pin for the yaw pot
const unsigned int PITCH_PIN = A2;//The analog pin for the pitch pin
const unsigned long DURATION = 180000l;//The period (microseconds) of the timer cycle (0.18 seconds)
const unsigned int HEADER_DURATION = 2000;//The duration (microseconds) of the header signal
const unsigned int HIGH_DURATION = 380;//The duration (microseconds) that a bit being sent is HIGH (the LOW duration determines the value of the bit)
const unsigned int ZERO_LOW_DURATION = 220;//The duration (microseconds) that a 0 bit being sent is LOW
const unsigned int ONE_LOW_DURATION = 600;//The duration (microseconds) that a 1 bit being sent is LOW
const byte YAW_STATIONARY = 63;//The value that the helicopter's yaw is when it is not turning
const byte PITCH_STATIONARY = 63;//The value that the helicopter's pitch is when it is not pitching
const byte CAL_BYTE = 65;//The calibration value to send as part of the overall package to send to the helicopter

int throttle, yaw, pitch; //The three pot readings

/**
Sends the header signal over IR (the header is sent
at the beginning of commands to let the heli know
a new command is coming).
*/
void sendHeader()
{
  #ifndef MODULATED
    digitalWrite(IR_PIN, HIGH);
  #else
    TCCR2A |= _BV(COM2B1);
  #endif
   
  delayMicroseconds(HEADER_DURATION);
   
  #ifndef MODULATED
    digitalWrite(IR_PIN, LOW);
  #else
    TCCR2A &= ~_BV(COM2B1);
  #endif
   
  delayMicroseconds(HEADER_DURATION);
   
  #ifndef MODULATED
    digitalWrite(IR_PIN, HIGH);
  #else
    TCCR2A |= _BV(COM2B1);
  #endif
   
  delayMicroseconds(HIGH_DURATION);
   
  #ifndef MODULATED
    digitalWrite(IR_PIN, LOW);
  #else
    TCCR2A &= ~_BV(COM2B1);
  #endif
}

/**
Sends a signal over IR that represents a zero bit
in the helicopter's protocol.
*/
void sendZero()
{
  Serial.print(0);
  delayMicroseconds(ZERO_LOW_DURATION);
 
  #ifndef MODULATED
    digitalWrite(IR_PIN, HIGH);
  #else  
    TCCR2A |= _BV(COM2B1);
  #endif
   
  delayMicroseconds(HIGH_DURATION);
   
  #ifndef MODULATED
    digitalWrite(IR_PIN, LOW);
  #else
    TCCR2A &= ~_BV(COM2B1);
  #endif
}

/**
Sends a signal over IR that represents a one bit
in the helicopter's protocol.
*/
void sendOne()
{
  Serial.print(1);
  delayMicroseconds(ONE_LOW_DURATION);
   
  #ifndef MODULATED
    digitalWrite(IR_PIN, HIGH);
  #else
    TCCR2A |= _BV(COM2B1);
  #endif
   
  delayMicroseconds(HIGH_DURATION);
   
  #ifndef MODULATED
    digitalWrite(IR_PIN, LOW);  
  #else
    TCCR2A &= ~_BV(COM2B1);
  #endif
}

/**
Sends the entire next command over IR according to the
helicopter's protocol:
Header - yaw - pitch - throttle - calibration
Each of the items sent other than the header is a single
byte of data, of which only the seven least significant
bits are used in each case. So each value can technically
be anything from 0 to 127. The yaw and pitch seem to be
centered around 63, while the throttle seems to start low -
perhaps at 0. The calibration byte is the same number
every time (in my case, 65).
*/
void sendCommand(int throttle, int yaw, int pitch)
{
  Serial.println("Sending Command");
  byte b;
  
  sendHeader();
  
  Serial.print("Y: ");
  for (int i = 7; i >=0; i--)
  {
    b = ((YAW_STATIONARY + yaw) & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
  
  Serial.print(", P: ");
  for (int i = 7; i >=0; i--)
  {
    b = ((PITCH_STATIONARY + pitch) & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  } 
   
  Serial.print(", T: ");
  for (int i = 7; i >=0; i--)
  {
    b = (throttle & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
  
  Serial.print(", C: ");
  for (int i = 7; i >=0; i--)
  {
    b = (CAL_BYTE & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
  
  Serial.println();
}
 
void setup()
{
  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, LOW);
  
  Serial.begin(9600);
 
  //setup interrupt interval: 180ms  
  Timer1.initialize(DURATION);//Sets up the period of the timer in microseconds
  Timer1.attachInterrupt(timerISR);//Attach the interrupt service routine
   
  //setup PWM: f=38Khz PWM=0.5
  byte v = 8000 / 38;
  TCCR2A = _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS20);
  OCR2A = v;
  OCR2B = v / 2;
}
 
void loop()
{  
    
}
 
void timerISR()
{
  //read control values from potentiometers
  throttle = analogRead(THROTTLE_PIN);
  yaw = analogRead(YAW_PIN);
  pitch = analogRead(PITCH_PIN);
   
  throttle = throttle / 4; //convert to 0 to 255 (analog read returns 0 to 1023)
  yaw = yaw / 8 - 64; //convert to -64 to 63
  pitch = pitch / 4 - 128; //convert to -128 to 127
   
  sendCommand(throttle, yaw, pitch);
}

