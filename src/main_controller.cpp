#include <Adafruit_ADS1X15.h>
#include <HTTPClient.h>
#include <Wire.h>

#include "three_panel_display.h"

// Right Button
bool right_button_pressed = false;
volatile unsigned long right_last_press_time = 0;  // Debouncing variable
const unsigned long right_debounce_delay = 250;    // Debounce delay in ms
void on_right_button()
{
  unsigned long current_press_time = millis();
  if (current_press_time - right_last_press_time > right_debounce_delay)
  {
    right_button_pressed = true;
    right_last_press_time = current_press_time;
  }
}

// Left Button
bool left_button_pressed = false;
volatile unsigned long left_last_press_time = 0;  // Debouncing variable
const unsigned long left_debounce_delay = 250;    // Debounce delay in ms
void on_left_button()
{
  unsigned long current_press_time = millis();
  if (current_press_time - left_last_press_time > left_debounce_delay)
  {
    left_button_pressed = true;
    left_last_press_time = current_press_time;
  }
}

ThreePanelDisplay display = ThreePanelDisplay();
Adafruit_ADS1015 ads1015;

void set_display_mode(const String& mode) { display.setPanelContent(0, mode); }

int adc0, adc1, adc2, adc3;
int adc0_zero, adc1_zero, adc2_zero, adc3_zero;
const int MAX_JOY = 1400;
const int MIN_JOY = 40;

void read_sensor_data()
{
  adc0 = ads1015.readADC_SingleEnded(0);
  adc1 = ads1015.readADC_SingleEnded(1);
  adc2 = ads1015.readADC_SingleEnded(2);
  adc3 = ads1015.readADC_SingleEnded(3);
}

void set_joystick_zeros()
{
  // For each adc, average the next 50 readings and set that as the zero reading
  int adc0_sum = 0;
  int adc1_sum = 0;
  int adc2_sum = 0;
  int adc3_sum = 0;

  for (int i = 0; i < 100; i++)
  {
    adc0_sum += ads1015.readADC_SingleEnded(0);
    adc1_sum += ads1015.readADC_SingleEnded(1);
    adc2_sum += ads1015.readADC_SingleEnded(2);
    adc3_sum += ads1015.readADC_SingleEnded(3);
  }

  adc0_zero = adc0_sum / 100;
  adc1_zero = adc1_sum / 100;
  adc2_zero = adc2_sum / 100;
  adc3_zero = adc3_sum / 100;

  Serial.println(adc0_zero);
  Serial.println(ads1015.readADC_SingleEnded(0));
}

void get_joy_positions(int& x1, int& y1, int& x2, int& y2)
{
  //   Serial.print(adc0_zero);
  //   Serial.print(", ");
  //   Serial.print(adc0);
  //   Serial.print(", ");
  x1 = adc0 - adc0_zero;
  y1 = adc1 - adc1_zero;
  x2 = adc2 - adc2_zero;
  y2 = adc3 - adc3_zero;

  //   Serial.println(x1);

  //   Deadband to remove noise
  if (abs(x1) < 10)
  {
    x1 = 0;
  }
  if (abs(y1) < 10)
  {
    y1 = 0;
  }
  if (abs(x2) < 10)
  {
    x2 = 0;
  }
  if (abs(y2) < 10)
  {
    y2 = 0;
  }
  //   Scale to -100 to 100
  x1 = map(x1, -700, 700, -100, 100);
  y1 = map(y1, -700, 700, -100, 100);
  x2 = map(x2, -700, 700, -100, 100);
  y2 = map(y2, -700, 700, -100, 100);
  if (x1 > 100)
  {
    x1 = 100;
  }
  if (x1 < -100)
  {
    x1 = -100;
  }
  if (y1 > 100)
  {
    y1 = 100;
  }
  if (y1 < -100)
  {
    y1 = -100;
  }
  if (x2 > 100)
  {
    x2 = 100;
  }
  if (x2 < -100)
  {
    x2 = -100;
  }
  if (y2 > 100)
  {
    y2 = 100;
  }
  if (y2 < -100)
  {
    y2 = -100;
  }

  Serial.print("Joy1: ");
  Serial.print(x1);
  Serial.print(", ");
  Serial.print(y1);
  Serial.print("  Joy2: ");
  Serial.print(x2);
  Serial.print(", ");
  Serial.println(y2);
}

void setup()
{
  Serial.begin(115200);

  //   Setup I2c on 13, 12
  Wire.begin(13, 15);
  ads1015.begin(0x48, &Wire);

  display.setup();

  display.setPanelTitle(0, "Mode");
  display.setPanelContent(0, "Test");

  display.setPanelTitle(1, "");
  display.setPanelContent(1, "");

  display.setPanelTitle(2, "");
  display.setPanelContent(2, "");

  //   Setup the right button
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(17), []() { on_right_button(); }, RISING);

  //   Setup the left button
  pinMode(27, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(27), []() { on_left_button(); }, RISING);

  set_joystick_zeros();
}

void loop()
{
  display.loop();
  read_sensor_data();
  if (right_button_pressed)
  {
    right_button_pressed = false;
    Serial.println("Right button pressed");
  }
  if (left_button_pressed)
  {
    left_button_pressed = false;
    Serial.println("Left button pressed");
  }

  int joy1_x, joy1_y, joy2_x, joy2_y;
  get_joy_positions(joy1_x, joy1_y, joy2_x, joy2_y);
  //   Serial.print(adc3_zero);
  //   Serial.print(", ");
  //   Serial.println(adc3);
}