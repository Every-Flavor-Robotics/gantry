#include <Arduino.h>
#include <ESPmDNS.h>

#include "configurable.h"
#include "gantry.h"
#include "motorgo_mini.h"
#include "web_server.h"

#define LOOP_HZ 400
#define LOOP_US 1000000 / LOOP_HZ

Gantry::Gantry gantry("gantry");

// Motor parameters
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

// PID parameters
MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;
MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

// Setup configurables
ESPWifiConfig::Configurable<float> position_pid_p_ch0(
    position_pid_params_ch0.p, "/ch0/position/p",
    "P Gain, Position, Channel 0");
ESPWifiConfig::Configurable<float> position_pid_i_ch0(
    position_pid_params_ch0.i, "/ch0/position/i",
    "I Gain, Position, Channel 0");
ESPWifiConfig::Configurable<float> position_pid_d_ch0(
    position_pid_params_ch0.d, "/ch0/position/d",
    "D Gain, Position, Channel 0");

ESPWifiConfig::Configurable<float> position_pid_p_ch1(
    position_pid_params_ch1.p, "/ch1/position/p",
    "P Gain, Position, Channel 1");

ESPWifiConfig::Configurable<float> position_pid_i_ch1(
    position_pid_params_ch1.i, "/ch1/position/i",
    "I Gain, Position, Channel 1");

ESPWifiConfig::Configurable<float> position_pid_d_ch1(
    position_pid_params_ch1.d, "/ch1/position/d",
    "D Gain, Position, Channel 1");

ESPWifiConfig::Configurable<float> velocity_pid_p_ch0(
    velocity_pid_params_ch0.p, "/ch0/velocity/p",
    "P Gain, Velocity, Channel 0");

ESPWifiConfig::Configurable<float> velocity_pid_i_ch0(
    velocity_pid_params_ch0.i, "/ch0/velocity/i",
    "I Gain, Velocity, Channel 0");

ESPWifiConfig::Configurable<float> velocity_pid_d_ch0(
    velocity_pid_params_ch0.d, "/ch0/velocity/d",
    "D Gain, Velocity, Channel 0");

ESPWifiConfig::Configurable<float> velocity_pid_p_ch1(
    velocity_pid_params_ch1.p, "/ch1/velocity/p",
    "P Gain, Velocity, Channel 1");

ESPWifiConfig::Configurable<float> velocity_pid_i_ch1(
    velocity_pid_params_ch1.i, "/ch1/velocity/i",
    "I Gain, Velocity, Channel 1");

ESPWifiConfig::Configurable<float> velocity_pid_d_ch1(
    velocity_pid_params_ch1.d, "/ch1/velocity/d",
    "D Gain, Velocity, Channel 1");

// Print message at a given frequency, provided in Hz
void freq_print(float freq, const char* message)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000.0 / freq)
  {
    Serial.print(message);
    last_print_time = now;
  }
}

void freq_println(float freq, const char* message)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000.0 / freq)
  {
    Serial.println(message);
    last_print_time = now;
  }
}

void setup()
{
  Serial.begin(115200);

  delay(1000);

  //   while (digitalRead(0) == HIGH)
  //   {
  //     delay(100);
  //   }

  //   Setup configurable callbacks

  // Setup motor parameters
  motor_params_ch0.pole_pairs = 11;
  motor_params_ch0.power_supply_voltage = 5.0;
  motor_params_ch0.voltage_limit = 5.0;
  motor_params_ch0.current_limit = 320;
  motor_params_ch0.velocity_limit = 100.0;
  motor_params_ch0.calibration_voltage = 3.0;

  motor_params_ch1.pole_pairs = 11;
  motor_params_ch1.power_supply_voltage = 5.0;
  motor_params_ch1.voltage_limit = 5.0;
  motor_params_ch1.current_limit = 320;
  motor_params_ch1.velocity_limit = 100.0;
  motor_params_ch1.calibration_voltage = 3.0;

  // Set PID parameters
  position_pid_params_ch0.p = 4.0;
  position_pid_params_ch0.i = 0.0;
  position_pid_params_ch0.d = 0.0;
  position_pid_params_ch0.output_ramp = 10000.0;
  position_pid_params_ch0.lpf_time_constant = 0.1;
  position_pid_params_ch0.limit = 100.0;

  velocity_pid_params_ch0.p = 10.0;
  velocity_pid_params_ch0.i = 0.4;
  velocity_pid_params_ch0.d = 0.00;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.limit = 100.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.14;

  position_pid_params_ch1.p = 8.0;
  position_pid_params_ch1.i = 0.0;
  position_pid_params_ch1.d = 0.008;
  position_pid_params_ch1.output_ramp = 10000.0;
  position_pid_params_ch1.lpf_time_constant = 0.1;
  position_pid_params_ch1.limit = 100.0;

  velocity_pid_params_ch1.p = 6.0;
  velocity_pid_params_ch1.i = 1.2;
  velocity_pid_params_ch1.d = 0.018;
  velocity_pid_params_ch1.output_ramp = 10000.0;
  velocity_pid_params_ch1.limit = 100.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.14;

  // Setup gantry
  //   gantry = new Gantry::Gantry("gantry");
  gantry.setup(motor_params_ch0, motor_params_ch1, position_pid_params_ch0,
               position_pid_params_ch1, velocity_pid_params_ch0,
               velocity_pid_params_ch1);

  delay(1000);

  // Start server
  ESPWifiConfig::WebServer::getInstance().start();

  if (!MDNS.begin("gantry"))
  {  // Start the mDNS responder for esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  MDNS.addService("http", "tcp", 8080);
  MDNS.addServiceTxt("http", "tcp", "gantry", "gantry0");
}

int i = 0;
// Constrain loop speed to 250 Hz
unsigned long last_loop_time = 0;
float vel = 30;
void loop()
{
  gantry.loop();

  //   // print velocity for both channels
  //   char str_buffer[100];
  //   sprintf(str_buffer, "Ch0: %f, Ch1: %f\n",
  //   motorgo_mini->get_ch0_velocity(),
  //           motorgo_mini->get_ch1_velocity());

  //   freq_print(50, str_buffer);

  // Delay necessary amount micros
  unsigned long now = micros();
  unsigned long loop_time = now - last_loop_time;

  if (loop_time < LOOP_US)
  {
    delayMicroseconds(LOOP_US - loop_time);
  }

  last_loop_time = now;
}