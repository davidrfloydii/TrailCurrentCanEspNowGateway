#include "globals.h"
#include "canHelper.h"
#include "espNowHelper.h"

void setup() {
  Serial.begin(115200);
  canHelper::initialize();
  espNowHelper::initialize();
}

void loop() {
  canHelper::checkCanBusForMessages();
  if (newDataToSend) {
      espNowHelper::sendData();
      newDataToSend = false;
  }
  delay(10);
}
