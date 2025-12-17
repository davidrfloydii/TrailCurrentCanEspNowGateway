#include "globals.h"
#include "canHelper.h"
#include "espNowHelper.h"

void setup()
{
  Serial.begin(115200);
  canHelper::initialize();
  espNowHelper::initialize();
}

void loop()
{
  canHelper::checkCanBusForMessages();
  delay(10);
}
