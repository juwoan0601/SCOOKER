#include <SoftwareSerial.h>

SoftwareSerial BTSerial(7, 8); // Software Serial (TX,RX) 
int vib = 3; // Vibration Sensor input pin

void setup(){
  pinMode(vib, INPUT); //센서핀 입력
  Serial.begin(9600);
  BTSerial.begin(9600);
  Serial.println("----------------------vibration demo------------------------");
}

void loop(){
  // Sensing Process
  long measurement =TP_init();
  int mes_scaleDown;
  char data[100] = {0} ;

  delay(50);
  
  Serial.print("measurment = ");
  Serial.print(measurement);
  //mes_scaleDown = abs(measurement)/ 2000;
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
