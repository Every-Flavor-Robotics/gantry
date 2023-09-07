#ifndef _THREE_PANEL_DISPLAY_H
#define _THREE_PANEL_DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define TFT_CS 5
#define TFT_RST 23
#define TFT_DC 16
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_BL 4  // Display backlight control pin

class ThreePanelDisplay
{
 public:
  // Constructor, initializes display
  ThreePanelDisplay();

  void setup();
  void loop();

  // Public API for setting panel texts
  void setPanelTitle(uint8_t panelIndex, const String& title);
  void setPanelContent(uint8_t panelIndex, const String& content);

 private:
  // Helper class for individual panels
  class Panel
  {
   public:
    Panel(int x, int y, int width, int height);
    void setTitle(const String& title);
    void setContent(const String& content);
    void draw(Adafruit_ST7789& tft, GFXcanvas16& canvas);

   private:
    int x, y, width, height;
    String title;
    String content;
  };

  bool updateDisplay = false;
  Panel panels[3];  // Three panels

  // Adafruit display object
  Adafruit_ST7789 tft;

  // Internal methods
  void initDisplay();
  void drawIndicators();

  // Constants
  static const int SCREEN_WIDTH = 135;
  static const int SCREEN_HEIGHT = 240;
  static const int PADDING = 5;
  static const int PANEL_HEIGHT = 240 / 3 - 10;
  static const int TEXT_SIZE = 2;
};

#endif  // _THREE_PANEL_DISPLAY_H
