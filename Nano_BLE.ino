#include <SoftwareSerial.h>
#define BT_RX 7
#define BT_TX 8
 
SoftwareSerial HM10(BT_RX,BT_TX);  // RX핀(7번)은 HM10의 TX에 연결 
                                   // TX핀(8번)은 HM10의 RX에 연결  
void setup() {  
  Serial.begin(9600);
  HM10.begin(9600);
}
void loop() {
  if (HM10.available()) {
    Serial.write(HM10.read());
  }
  if (Serial.available()) {
    HM10.write(Serial.read());
  }
}
