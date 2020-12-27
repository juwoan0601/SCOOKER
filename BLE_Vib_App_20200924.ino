#include <SoftwareSerial.h> 

// PIN info
int vib = 3; // Vibration Sensor input pin
int BT_TX = 7;
int BT_RX = 8;
int redPin = 9;
int greenPin = 10;
int bluePin = 11;

SoftwareSerial BTSerial(BT_TX, BT_RX); // Software Serial (TX,RX) 

void setup(){
  pinMode(vib, INPUT); //센서핀 입력
  Serial.begin(9600);
  BTSerial.begin(9600);
  Serial.println("----------------------vibration demo------------------------");
  setColor(255,255,255);
  delay(2000);
  setColor(0,0,0);
  delay(1000);
}

void loop(){
  // Sensing Process
  long measurement =TP_init();
  char data[100] = {0} ;
  delay(50);
  vibLevel(measurement);
  Serial.print("measurment = ");
  Serial.print(measurement);
  //mes_scaleDown = abs(measurement)/ 2000;
  int mes_scaleDown;
  mes_scaleDown = abs(measurement);
  itoa(mes_scaleDown, data, 10);
  Serial.print("Buffer = ");
  Serial.println(data);
  
  BTSerial.write(data);
  BTSerial.write("\n");
}

long TP_init(){
  delay(10);
  long measurement=pulseIn (vib, HIGH);
  return measurement;
}

void setColor(int red, int green, int blue){
  analogWrite(redPin, 255-red);
  analogWrite(greenPin, 255-green);
  analogWrite(bluePin, 255-blue); 
}

void vibLevel(long val){
  if(val < 100) {
    setColor(0,255,0); // green
  }
  else if((val>100)&&(val<10000)) setColor(0,0,255); // blue
  else if((val>10000)&&(val<5000)) setColor(255,0,0); // red
  else if((val<50000)) setColor(255,0,0);
  else setColor(255,255,255);
}
