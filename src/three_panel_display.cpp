#include "three_panel_display.h"

bool changed = true;
ThreePanelDisplay::ThreePanelDisplay()
    : tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST),
      panels{Panel(0, PADDING, SCREEN_WIDTH, PANEL_HEIGHT),
             Panel(0, PANEL_HEIGHT + 2 * PADDING, SCREEN_WIDTH, PANEL_HEIGHT),
             Panel(0, 2 * (PANEL_HEIGHT + PADDING), SCREEN_WIDTH, PANEL_HEIGHT)}
{
}

void ThreePanelDisplay::setup() { initDisplay(); }

void ThreePanelDisplay::initDisplay()
{
  tft.init(SCREEN_WIDTH, SCREEN_HEIGHT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(0);
}

void ThreePanelDisplay::setPanelTitle(uint8_t panelIndex, const String &title)
{
  if (panelIndex < 3)
  {
    panels[panelIndex].setTitle(title);
    updateDisplay = true;
  }
}

void ThreePanelDisplay::setPanelContent(uint8_t panelIndex,
                                        const String &content)
{
  if (panelIndex < 3)
  {
    panels[panelIndex].setContent(content);
    updateDisplay = true;
  }
}

void ThreePanelDisplay::Panel::setTitle(const String &title)
{
  this->title = title;
}

void ThreePanelDisplay::Panel::setContent(const String &content)
{
  this->content = content;
}

ThreePanelDisplay::Panel::Panel(int x, int y, int width, int height)
    : x(x), y(y), width(width), height(height), title(""), content("")
{
}

void ThreePanelDisplay::Panel::draw(Adafruit_ST7789 &tft, GFXcanvas16 &canvas)
{
  // Draw the rounded rectangle
  canvas.fillRoundRect(x + PADDING, y, width - 2 * PADDING, height, 10,
                       ST77XX_CYAN);

  // Calculate width of the title and content
  int16_t titleWidth = title.length() * 6 * TEXT_SIZE;
  int16_t contentWidth = content.length() * 6 * TEXT_SIZE;

  // Calculate starting cursor positions for centered text
  int titleX = x + (width - titleWidth) / 2;
  int titleY = y + height / 4 - (TEXT_SIZE * 4) / 2;
  int contentX = x + (width - contentWidth) / 2;
  int contentY = y + 3 * height / 4 - (TEXT_SIZE * 4) / 2;

  // Draw the title centered
  canvas.setCursor(titleX, titleY);
  canvas.setTextColor(ST77XX_BLACK);
  canvas.print(title);

  // Draw the content centered and one line below the title
  canvas.setCursor(contentX, contentY);
  canvas.setTextColor(ST77XX_BLACK);
  canvas.print(content);
}

void ThreePanelDisplay::drawIndicators()
{
  GFXcanvas16 *canvas = new GFXcanvas16(SCREEN_WIDTH, SCREEN_HEIGHT);
  canvas->fillScreen(ST77XX_BLACK);
  canvas->setTextSize(TEXT_SIZE);
  canvas->setTextColor(ST77XX_WHITE);

  for (auto &panel : panels)
  {
    panel.draw(tft, *canvas);
  }

  tft.startWrite();
  tft.drawRGBBitmap(0, 0, canvas->getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);
  tft.endWrite();

  delete canvas;
}

void ThreePanelDisplay::loop()
{
  if (updateDisplay)
  {
    drawIndicators();
    updateDisplay = false;
  }
}
