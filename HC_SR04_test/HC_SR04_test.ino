const unsigned int TRIG_PIN = 13;
const unsigned int ECHO_PIN = 12;
const unsigned int LED = 11;

void setup()
{
  Serial.begin (9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED, OUTPUT);
}

void loop()
{
  long duration, distance;
  
  /*
  According to the datasheet, the module sends a pulse when the trigger receives
  at least 10 microseconds of a high signal. So send a 2 us low, then 10 us high.
  */ 
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);//Measures the amount of time the ECHO_PIN is HIGH - that time is the distance to the nearest object * 2
  distance = (duration / 2) / 29.1;//speed of sound = 343 m/s, but the datasheet gives this as the formula to convert to cm
  
  if (distance < 4)  // This is where the LED On/Off happens
    digitalWrite(LED, HIGH);
  else 
    digitalWrite(led,LOW);
  
  if (distance >= 200 || distance <= 0)
  {
    Serial.println("Out of range");
  }
  else
  {
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  delay(500); //SUPER IMPORTANT! Have to make sure you don't trigger the module at too great a frequency - or you will
  //get overlap between triggers and readings.
}
