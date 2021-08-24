void setup() 
{
  pinMode(2, OUTPUT);
  Serial.begin(9600);

}

void loop() 
{
//  digitalWrite(2, HIGH);
//
//  delay(500);

  Serial.println("Hello World!");

  delay(500);
}
