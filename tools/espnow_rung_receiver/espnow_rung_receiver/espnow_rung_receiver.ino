// Open this sketch in Arduino IDE. Configure RobotMac in Settings.h first.
#include <WiFi.h>
#include "ReceiverImpl.h"

void setup() { PassiveReceiver::begin(); }
void loop() { PassiveReceiver::service(); }
