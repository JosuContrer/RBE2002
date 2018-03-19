#include <Arduino.h>
#include "FireSensor.h"

FireSensor fireSensor;

void setup() {
    // put your setup code here, to run once:
    fireSensor.initialize();
}

void loop() {
    // put your main code here, to run repeatedly:
    fireSensor.useSensor();
    fireSensor.showAll();
}
