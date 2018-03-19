#include <Arduino.h>
#include "FireSensor.h"

//Object Creation
FireSensor fireSensor;

void setup() {
    //Fire Sensor
    fireSensor.initialize();
}

void loop() {
    //Fire Sensor
    fireSensor.useSensor();
    fireSensor.showAll();
}
