#include "three_panel_display.h"

ThreePanelDisplay display = ThreePanelDisplay();

void setup()
{
  Serial.begin(115200);
  display.setup();

  display.setPanelTitle(0, "MC");
  display.setPanelContent(0, "Multicast");

  display.setPanelTitle(1, "Log");
  display.setPanelContent(1, "Logging");

  display.setPanelTitle(2, "RTC");
  display.setPanelContent(2, "Time");
}

void loop() { display.loop(); }
