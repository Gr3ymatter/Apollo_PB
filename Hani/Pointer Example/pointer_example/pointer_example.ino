void setup() {
  // put your setup code here, to run once:
 Serial.print(9600);
}

uint8_t var = 10;

void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.print("Var value outside function = ");
  Serial.println(var);
  
  add(&var, 5);
  
  Serial.print("Var value AFTER function = ");
  Serial.println(var);
  
  delay(1000);
  
  
}


void add(uint8_t *var, uint8_t addValue)
{
 *var +=  addValue;
 Serial.print("Var value inside function = ");
 Serial.println(*var);
 
}
