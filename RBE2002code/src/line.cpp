#include "line.h"
#include <Arduino.h>
Line::Line(int pinNum,int arraySize){
  aSize=arraySize;
  pin=pinNum;
  initialize();
}
void Line::initialize(){
  pinMode(pin,INPUT);
  for(int i=0;i<aSize;i++){
    array[i]=0;
  }
}



int Line::limit(){
  int aS=aSize-1;
  for(int i=1; i < aSize-1; i++){
    array[aS-i]=array[aS-i-1]; //shift array values to make room for new
  }
  array[0]=digitalRead(pin);
  int sum=0;
  for(int i=0;i<aSize;i++){
    sum+=array[i];
  }
  if (sum>=aSize-1){
    return 1;
  }
  else{
    return 0;
  }
}
