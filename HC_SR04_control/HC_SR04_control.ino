/**
This sketch is to fly the helicopter based on its elevation -
it reads from the hcsr04 distance sensor every 10th of a
second to get the distance. Meanwhile, it packages the data
and sends it out to the helicopter whenver it is not using the
distance sensor.

Timer1 is in use as a countdown, then interrupt.
Timer2 is in use as a pwm.


Check!
Debug version 0:
Check to see if the hcsr04 works well attached to the timer
interrupting every 100 milliseconds. No IR code.

Check!
Debug version 1:
Make sure the IR packets get sent based on the current
value in distance. IR is no longer attached to an interrupt -
just runs in the loop.

------
Debug version 2:
Final version - smooth out the noisy distance values by using
a filter on the last few data points brought in by the hcsr04.
*/

#include <TimerOne.h>


/**
Pin declarations
*/
const unsigned int TRIG_PIN = 5;
const unsigned int ECHO_PIN = 4;
const unsigned int IR_PIN = 3;//Must be a PWM pin
const unsigned int HEADER_DURATION = 2000;//The duration (microseconds) of the header signal
const unsigned int HIGH_DURATION = 380;//The duration (microseconds) of a high signal in the IR protocol (for zero or one)
const unsigned int ZERO_LOW_DURATION = 220;//The duration (microseconds) of a low signal that indicates a zero bit in the IR protocol
const unsigned int ONE_LOW_DURATION = 600;//The duration (microseconds) of a low signal that indicates a one bit in the IR protocol
const byte YAW_STATIONARY = 63;//The value that the helicopter's yaw is when it is not turning
const byte PITCH_STATIONARY = 63;//The value that the helicopter's pitch is when it is not pitching
const byte CAL_BYTE = 65;//The calibration value to send as part of the overall package to send to the helicopter

/**
hcsr04 stuff
*/
volatile long duration;//Duration measured by the hcsr04 in the ISR
volatile long distance = 0;//Distance (cm) measured by the hcsr04 in the ISR
const unsigned long TIMEOUT = 60000;//If pulseIn doesn't hear an echo back after 60 milliseconds, exit the ISR
volatile boolean ignore_next_pulse = false;//Flag for trying to guarantee the fidelity of our sensor data
const int MIN_DIST = 3;//cm
const long MAX_DIST = 80;//cm - hcsr04 caps out at about 80 before noise starts making it meaningless

/**
IR stuff
*/
unsigned int throttle = 0;//The value of the throttle to send to the heli

void setup()
{
  /**
  Pin modes
  */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, LOW);//ensure the signal is low to start out
  
  /**
  Set up the PWM on pin 3 and 11 (that is, timer2)
  frequency is 38KHz with a 50% duty cycle
  */
  byte freq = 8000 / 38;
  TCCR2A = _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS20);
  OCR2A = freq;
  OCR2B = freq / 2;
  
  /**
  Set up the timer - interval is 100 milliseconds
  */
  Timer1.initialize(100000l);//one hundred thousand long (mics) - one tenth of a second
  Timer1.attachInterrupt(measureDistanceISR);
  
  Serial.begin(9600);
}

void loop()
{
  packageAndSend();
  delay(180);//delay 180 ms to give the heli time to breathe
}


/**
ISR triggered every time the timer overflows - measures the
distance to the nearest object.

PulseIn should work - it doesn't use any interrupts that I can
see - it seems to just read port values, then increment a
counter every time it doesn't see a change in the port. Then
it returns the duration it waited by multiplying the number of
times it looped over the wait code by the factors needed to
turn it into microseconds (factors simply determined by clock
speed).
*/
void measureDistanceISR()
{
  if (ignore_next_pulse)
  {
    ignore_next_pulse = false;
    return;
  }
  
  /*
  According to the datasheet, the module sends a pulse when the trigger receives
  at least 10 microseconds of a high signal. So send a 2 us low, then 10 us high.
  */ 
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT);//Measures the amount of time the ECHO_PIN is HIGH - that time is the distance to the nearest object * 2
  if (duration == 0)
  {
    //The pulseIn timed out, now there is an eroneous echo out there somewhere - ignore the next pulse to be safe.
    ignore_next_pulse = true;
    return;
  }
  
  distance = (duration / 2) / 29.1;//speed of sound = 343 m/s, but the datasheet gives this as the formula to convert to cm
}

/**
Packages data to send to the heli based on the current
distance.
*/
void packageAndSend()
{
  long dist = distance;
 
  //throttle should be large if distance is small and small if distance is large
  //so map it
  dist = constrain(dist, MIN_DIST, MAX_DIST);
  Serial.print("Dist: "); Serial.print(dist); Serial.print(" ");
  throttle = map(dist, MIN_DIST, MAX_DIST, 126, 0);//126 is the maximum value held in the seven bits of data that are actually read by the heli
  Serial.print("Sending: "); Serial.println(throttle);
  sendCommand(throttle, 0, 0);
}

/**
Sends the header signal over IR (the header is sent
at the beginning of commands to let the heli know
a new command is coming).
*/
void sendHeader()
{
  TCCR2A |= _BV(COM2B1);
  delayMicroseconds(HEADER_DURATION);
  TCCR2A &= ~_BV(COM2B1);
  delayMicroseconds(HEADER_DURATION);
  TCCR2A |= _BV(COM2B1);
  delayMicroseconds(HIGH_DURATION);
  TCCR2A &= ~_BV(COM2B1);
}

/**
Sends a signal over IR that represents a zero bit
in the helicopter's protocol.
*/
void sendZero()
{
  delayMicroseconds(ZERO_LOW_DURATION);  
  TCCR2A |= _BV(COM2B1);
  delayMicroseconds(HIGH_DURATION); 
  TCCR2A &= ~_BV(COM2B1);
}

/**
Sends a signal over IR that represents a one bit
in the helicopter's protocol.
*/
void sendOne()
{
  delayMicroseconds(ONE_LOW_DURATION);
  TCCR2A |= _BV(COM2B1);
  delayMicroseconds(HIGH_DURATION);
  TCCR2A &= ~_BV(COM2B1);
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
  byte b;
  
  sendHeader();
  
  for (int i = 7; i >=0; i--)
  {
    b = ((YAW_STATIONARY + yaw) & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
  
  for (int i = 7; i >=0; i--)
  {
    b = ((PITCH_STATIONARY + pitch) & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  } 
   
  for (int i = 7; i >=0; i--)
  {
    b = (throttle & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
  
  for (int i = 7; i >=0; i--)
  {
    b = (CAL_BYTE & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
}
