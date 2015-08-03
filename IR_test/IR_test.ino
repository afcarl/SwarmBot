/*
This code is to test the IR LED - pulsing it at about 38kHz, used in conjungtion
with another sketch (IR_read) on another Arduino. It sends a message over the IR
for the other Arduino to read and print over Serial.
*/

const unsigned int LED_PIN = 9;           // the pin that the LED is attached to

void setup() 
{
  pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
  //Delay for a bit (2 sec).
  delay(2000);
  
  //Send out the IR message
  sendIR();
}

/**
Send the IR signal
*/
void sendIR()
{
  pulseIR(2080);
  delay(27);
  pulseIR(440);
  delayMicroseconds(1500);
  pulseIR(460);
  delayMicroseconds(3440);
  pulseIR(480);
  
  delay(65); //wait 65 milliseconds before sending it again
  
  pulseIR(2000);
  delay(27);
  pulseIR(440);
  delayMicroseconds(1500);
  pulseIR(460);
  delayMicroseconds(3440);
  pulseIR(480);
}

/**
Pulse the IR. Sends a 38kHz pulse over IR LED by writing high, then low
very quickly again and again for however many microseconds you tell it to.
*/
void pulseIR(long microsecs)
{
  cli(); //Turn off any background interrupts since this timing is so precise
  
  //Write high, then low again and again as long as we stil have microsecs to work with
  while (microsecs > 0)
  {
    //38kHz is about 13 microseconds high and 13 low.
    digitalWrite(LED_PIN, HIGH);  //this takes about 3 mics to happen
    delayMicroseconds(10);        //hang out for 10 mics (change to 9 if not working)
    digitalWrite(LED_PIN, LOW);   //this also takes about 3 mics
    delayMicroseconds(10);        //hang out for 10 mics (change to 9 if not working)
    
    //so 26 mics altogether
    microsecs -= 26;
  }
  
  sei(); //Turn the interrupts back on
}

