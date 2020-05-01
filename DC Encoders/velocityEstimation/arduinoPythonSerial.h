#ifndef ARDUINO_PYTHON_SERIAL_H
#define ARDUINO_PYTHON_SERIAL_H

#define START "start"
#define STOP "stop"

void sendStart(){
  Serial.println(START);
}

void sendStop(){
  Serial.println(STOP);
}
void beginComms(){
  unsigned long timer;
  bool received = false;
  while(!received){
    sendStart();
    timer = micros();
    while((micros() - timer) < 1e6){
      if(Serial.available()){
         received = true;
      }
    }
  }
}

void endComms(){
  unsigned long timer;
  bool complete = false;
  while(!complete){
    sendStop();
    timer = micros();
    while(micros() - timer < 1e6){
      if(Serial.available()){
        int bytesRead = Serial.read();
        if((bytesRead - 48) == 2){
          complete = true;
        }
      }
    }
  }
}

void sendData(unsigned long right, unsigned long left, unsigned long t, float volts)
{
  Serial.print(right);Serial.print(",");
  Serial.print(left);Serial.print(",");
  Serial.print(t);Serial.print(',');
  Serial.println(volts);
}

#endif
