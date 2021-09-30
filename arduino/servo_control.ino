#include <Servo.h>
Servo servo1;
Servo servo2;
float s1_angle = 0;
float s2_angle = 180;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  servo1.attach(10);
  servo2.attach(9);
  servo1.write(s1_angle);
  servo2.write(s2_angle);
}

void loop() {
  if (Serial.available() > 0) {
    // put your main code here, to run repeatedly:
    delay(2);
    String comdata = "";
    while (Serial.available() > 0) {
      comdata += char(Serial.read());
      delay(2);
    }

    //input data:a0b180
    s1_angle = (comdata.substring(comdata.indexOf('a')+1,comdata.indexOf('b'))).toFloat();
    s2_angle = (comdata.substring(comdata.indexOf('b')+1)).toFloat();
    Serial.println(s1_angle);
    Serial.println(s2_angle);
    servo1.write(s1_angle);
    servo2.write(s2_angle);
    
//    if (comdata.charAt(0) == 'a') {
//      comdata.remove(0,1);
//      servo1.write(comdata.toInt());
//    } else if (comdata.charAt(0) == 'b'){
//      comdata.remove(0,1);
//      servo2.write(comdata.toInt());
//    }
    //delay(5);
  }

}
