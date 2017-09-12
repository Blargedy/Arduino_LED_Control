int analogPin = A4;

void setup()
{
  pinMode(analogPin, INPUT);
  
  Serial.begin(9600);
}

void loop()
{
  Serial.println(analogRead(analogPin));
  delay(100);
}
