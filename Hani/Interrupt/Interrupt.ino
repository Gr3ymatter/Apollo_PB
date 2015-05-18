int pbIn = 4;                  // Interrupt 0 is on DIGITAL PIN 19!
int ledOut = 47;                // The output LED pin
volatile int state = LOW;      // The input state toggle
 
void setup()
{                
  // Set up the digital pin 2 to an Interrupt and Pin 4 to an Output
  pinMode(ledOut, OUTPUT);
 
  //Attach the interrupt to the input pin and monitor for ANY Change 
  attachInterrupt(pbIn, stateChange, CHANGE);
}
 
void loop()                     
{
  //Simulate a long running process or complex task
//  for (int i = 0; i < 100; i++)
//  {
//    // do nothing but waste some time
//    delay(10); 
//  }
}
 
void stateChange()
{
  state = !state;
  digitalWrite(ledOut, state);  
  delay(1000);
}
