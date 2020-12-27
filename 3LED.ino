/*
int redPin = A0;
int greenPin = A1;
int bluePin = A2;
*/

int redPin = 9;
int greenPin = 10;
int bluePin = 11;

void setup()
{
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
}
 
void loop()
{
  setColor(255, 0, 0); // red
  delay(1000);
  setColor(0, 255, 0); // green
  delay(1000);
  setColor(0, 0, 255); // blue
  delay(1000);
  setColor(255, 255, 0); // yellow
  delay(1000); 
  setColor(80, 0, 80); // purple
  delay(1000);
  setColor(0, 255, 255); // aqua
  delay(1000);
}
 
void setColor(int red, int green, int blue)
{
  
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue); 
  

  /*
  digitalWrite(redPin, 255-red);
  digitalWrite(greenPin, 255-green);
  digitalWrite(bluePin, 255-blue); 
  */
}
