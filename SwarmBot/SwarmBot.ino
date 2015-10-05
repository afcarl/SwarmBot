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

  Check!
  Debug version 1.1:
  Try sending data at a faster rate than every 180ms. Also,
  fine-tune the DELTA_THROTTLE to make sure that it provides
  a FAST, but SMOOTH change in throttle values.

  Check!
  Debug version 1.2:
  Get it to start on its own (must be zero for like 10 sends, then positive for 10ish).

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
//These values are not yet used - I put them here to remind myself that
//the pitch and the yaw must also have a maximum value 
//const byte MAX_PITCH = 126;//The maximum pitch that can be sent
//const byte MAX_YAW = 126;//The maximum yaw that can be sent
const byte MAX_THROTTLE = 126;//The maximum throttle value that can be sent
unsigned int throttle = 0;//The value of the throttle to send to the heli
unsigned int old_throttle = 0;//The previous value of the throttle sent. The new value should never be more than delta_throttle more or less than throttle
const unsigned int DELTA_THROTTLE = 20;//The maximum change in the throttle between two moments
const unsigned int HEADER_DURATION = 2000;//The duration (microseconds) of the header signal
const unsigned int HIGH_DURATION = 380;//The duration (microseconds) of a high signal in the IR protocol (for zero or one)
const unsigned int ZERO_LOW_DURATION = 220;//The duration (microseconds) of a low signal that indicates a zero bit in the IR protocol
const unsigned int ONE_LOW_DURATION = 600;//The duration (microseconds) of a low signal that indicates a one bit in the IR protocol
const byte YAW_STATIONARY = 63;//The value that the helicopter's yaw is when it is not turning
const byte PITCH_STATIONARY = 63;//The value that the helicopter's pitch is when it is not pitching
const byte CAL_BYTE = 65;//The calibration value to send as part of the overall package to send to the helicopter


//Flag for sending the lift_off signal
boolean has_not_lifted_off = true;

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
  if (has_not_lifted_off)
  {
    liftOff();
    has_not_lifted_off = false;
  }  
  
  packageAndSend();
  delay(60);//delay 60 ms to give the heli time to breathe
}

/**
Sends 30 low throttles, then 30 high throttles - the helicopter requires a few lows then
a few highs to start responding to the IR signal.
*/
void liftOff()
{
  Serial.println("Sending liftoff signal");
  
  Serial.println("Sending zeros");
  
  //send 30 lows, then 30 highs.
  for (int lows = 30; lows > 0; lows--)
  {
    sendCommand(0, 0, 0);//Send a zero throttle signal to the heli
    delay(20);//delay 20 ms
  }
  
  Serial.println("Sending highs");
  
  for (int highs = 30; highs > 0; highs--)
  {
    sendCommand(MAX_THROTTLE, 0, 0);//Send the maximum value for the throttle
    delay(20);//delay 20 ms
  }
  
  Serial.println("Liftoff signal sent");
}


/**
ISR triggered every time the timer overflows - measures the
distance to the nearest object.

Note to future self who might wonder about the pulseIn function inside
this function:

PulseIn should work - it doesn't use any interrupts that I can
see - it seems to just read port values, then increment a
counter every time it doesn't see a change in the port. Then
it returns the duration it waited by multiplying the number of
times it looped over the wait code by the factors needed to
turn it into microseconds (factors simply determined by clock
speed). So you are fine to use it in an ISR as long as you
give it a reasonably short timeout value.
*/
void measureDistanceISR()
{
  //Try to guarantee some fidelity
  if (ignore_next_pulse)
  {
    ignore_next_pulse = false;
    return;//This pulse is possibly a noisy rogue one; ignore it and wait for the next one
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
    //The pulseIn timed out, now there may be an eroneous echo out there somewhere - ignore the next pulse to be safe.
    ignore_next_pulse = true;
    return;
  }
  
  distance = (duration / 2) / 29.1;//speed of sound = 343 m/s, but the datasheet gives this as the formula to convert to cm
}

/**
Packages data to send to the heli based on the current
distance to the ground.
*/
void packageAndSend()
{
  long dist = distance;

  //throttle should be large if distance is small and small if distance is large
  dist = constrain(dist, MIN_DIST, MAX_DIST);
  Serial.print("Dist: "); Serial.print(dist); Serial.print(" ");
  old_throttle = throttle;
  throttle = map(dist, MIN_DIST, MAX_DIST, MAX_THROTTLE, 0);//126 is the maximum value held in the seven bits of data that are actually read by the heli
  
  /*if the new throttle value is too different from the old one, move it closer to the old one*/
  
  
  //Lower lim is 0 if old_throttle <= DELTA_THROTTLE, otherwise it is just old - delta
  //This is important to check becuase throttle is unsigned
  unsigned int lower_lim = (old_throttle <= DELTA_THROTTLE) ? 0 : (old_throttle - DELTA_THROTTLE);
  
  //Upper lim is whichever is smaller: 126 or old + delta
  unsigned int upper_lim = min(old_throttle + DELTA_THROTTLE, MAX_THROTTLE);
  
  throttle = constrain(throttle, lower_lim, upper_lim);//Make sure that the difference between old throttle and new throttle is no larger than delta_throttle
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
  
  //send the yaw byte
  for (int i = 7; i >=0; i--)
  {
    b = ((YAW_STATIONARY + yaw) & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
  
  //send the pitch byte
  for (int i = 7; i >=0; i--)
  {
    b = ((PITCH_STATIONARY + pitch) & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
   
  //send the throttle byte
  for (int i = 7; i >=0; i--)
  {
    b = (throttle & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
  
  //send the calibration byte
  for (int i = 7; i >=0; i--)
  {
    b = (CAL_BYTE & (1 << i)) >> i;    
    if (b > 0) sendOne(); else sendZero();
  }
}
