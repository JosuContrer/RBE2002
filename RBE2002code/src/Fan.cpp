#include "Fan.h"
#include "globalPins.h"
#include <Arduino.h>

void fanInitialize(){
  pinMode(FANPIN, OUTPUT);
  digitalWrite(FANPIN, LOW);
}

void fanState(bool on){
  if(on == true){
    digitalWrite(FANPIN, HIGH);
  }
  else{
    digitalWrite(FANPIN, LOW);
  }
}
