#include <Arduino.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>

#include "configurable.h"
#include "gantry.h"
#include "motorgo_mini.h"
#include "web_server.h"

TaskHandle_t loop_motorgo_task;

Gantry::Gantry gantry("gantry");

// Motor parameters
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

// PID parameters
MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;
MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

void on_pid_update(const float& updated_value);

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
ESPWifiConfig::Configurable<float> position_pid_lpf_ch0(
    position_pid_params_ch0.lpf_time_constant, "/ch0/position/lpf",
    "LPF, Position, Channel 0");

ESPWifiConfig::Configurable<float> position_pid_p_ch1(
    position_pid_params_ch1.p, "/ch1/position/p",
    "P Gain, Position, Channel 1");

ESPWifiConfig::Configurable<float> position_pid_i_ch1(
    position_pid_params_ch1.i, "/ch1/position/i",
    "I Gain, Position, Channel 1");

ESPWifiConfig::Configurable<float> position_pid_d_ch1(
    position_pid_params_ch1.d, "/ch1/position/d",
    "D Gain, Position, Channel 1");

ESPWifiConfig::Configurable<float> position_pid_lpf_ch1(
    position_pid_params_ch1.lpf_time_constant, "/ch1/position/lpf",
    "LPF, Position, Channel 1");

ESPWifiConfig::Configurable<float> velocity_pid_p_ch0(
    velocity_pid_params_ch0.p, "/ch0/velocity/p",
    "P Gain, Velocity, Channel 0");

ESPWifiConfig::Configurable<float> velocity_pid_i_ch0(
    velocity_pid_params_ch0.i, "/ch0/velocity/i",
    "I Gain, Velocity, Channel 0");

ESPWifiConfig::Configurable<float> velocity_pid_d_ch0(
    velocity_pid_params_ch0.d, "/ch0/velocity/d",
    "D Gain, Velocity, Channel 0");

ESPWifiConfig::Configurable<float> velocity_pid_lpf_ch0(
    velocity_pid_params_ch0.lpf_time_constant, "/ch0/velocity/lpf",
    "LPF, Velocity, Channel 0");

ESPWifiConfig::Configurable<float> velocity_pid_p_ch1(
    velocity_pid_params_ch1.p, "/ch1/velocity/p",
    "P Gain, Velocity, Channel 1");

ESPWifiConfig::Configurable<float> velocity_pid_i_ch1(
    velocity_pid_params_ch1.i, "/ch1/velocity/i",
    "I Gain, Velocity, Channel 1");

ESPWifiConfig::Configurable<float> velocity_pid_d_ch1(
    velocity_pid_params_ch1.d, "/ch1/velocity/d",
    "D Gain, Velocity, Channel 1");

ESPWifiConfig::Configurable<float> velocity_pid_lpf_ch1(
    velocity_pid_params_ch1.lpf_time_constant, "/ch1/velocity/lpf",
    "LPF, Velocity, Channel 1");

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

void loop_motorgo(void* pvParameters);

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
  motor_params_ch0.calibration_voltage = 1.0;

  motor_params_ch1.pole_pairs = 11;
  motor_params_ch1.power_supply_voltage = 5.0;
  motor_params_ch1.voltage_limit = 5.0;
  motor_params_ch1.current_limit = 320;
  motor_params_ch1.velocity_limit = 100.0;
  motor_params_ch1.calibration_voltage = 1.0;

  // Set PID parameters
  position_pid_params_ch0.p = 30.0;
  position_pid_params_ch0.i = 0.0;
  position_pid_params_ch0.d = 0.0;
  position_pid_params_ch0.output_ramp = 10000.0;
  position_pid_params_ch0.lpf_time_constant = 0.05;
  position_pid_params_ch0.limit = 100.0;

  velocity_pid_params_ch0.p = 3.5;
  velocity_pid_params_ch0.i = 0.0;
  velocity_pid_params_ch0.d = 0.0;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.05;
  velocity_pid_params_ch0.limit = 100.0;

  position_pid_params_ch1.p = 30.0;
  position_pid_params_ch1.i = 0.0;
  position_pid_params_ch1.d = 0.0;
  position_pid_params_ch1.output_ramp = 10000.0;
  position_pid_params_ch1.lpf_time_constant = 0.05;
  position_pid_params_ch1.limit = 100.0;

  velocity_pid_params_ch1.p = 3.0;
  velocity_pid_params_ch1.i = 0.0;
  velocity_pid_params_ch1.d = 0.008;
  velocity_pid_params_ch1.output_ramp = 10000.0;
  velocity_pid_params_ch1.limit = 100.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.05;

  // Setup motor parameters
  //   motor_params_ch0.pole_pairs = 11;
  //   motor_params_ch0.power_supply_voltage = 5.0;
  //   motor_params_ch0.voltage_limit = 5.0;
  //   motor_params_ch0.current_limit = 320;
  //   motor_params_ch0.velocity_limit = 100.0;
  //   motor_params_ch0.calibration_voltage = 3.0;

  //   motor_params_ch1.pole_pairs = 11;
  //   motor_params_ch1.power_supply_voltage = 5.0;
  //   motor_params_ch1.voltage_limit = 5.0;
  //   motor_params_ch1.current_limit = 320;
  //   motor_params_ch1.velocity_limit = 100.0;
  //   motor_params_ch1.calibration_voltage = 3.0;

  // Set PID parameters
  //   position_pid_params_ch0.p = 5.0;
  //   position_pid_params_ch0.i = 0.0;
  //   position_pid_params_ch0.d = 0.0;
  //   position_pid_params_ch0.output_ramp = 10000.0;
  //   position_pid_params_ch0.lpf_time_constant = 0.1;
  //   position_pid_params_ch0.limit = 100.0;

  //   velocity_pid_params_ch0.p = 15.0;
  //   velocity_pid_params_ch0.i = 1.2;
  //   velocity_pid_params_ch0.d = 0.00;
  //   velocity_pid_params_ch0.output_ramp = 10000.0;
  //   velocity_pid_params_ch0.limit = 100.0;
  //   velocity_pid_params_ch0.lpf_time_constant = 0.11;

  //   position_pid_params_ch1.p = 4.0;
  //   position_pid_params_ch1.i = 0.0;
  //   position_pid_params_ch1.d = 0.00;
  //   position_pid_params_ch1.output_ramp = 10000.0;
  //   position_pid_params_ch1.lpf_time_constant = 0.10;
  //   position_pid_params_ch1.limit = 100.0;

  //   velocity_pid_params_ch1.p = 15.0;
  //   velocity_pid_params_ch1.i = 0.8;
  //   velocity_pid_params_ch1.d = 0.0;
  //   velocity_pid_params_ch1.output_ramp = 10000.0;
  //   velocity_pid_params_ch1.limit = 100.0;
  //   velocity_pid_params_ch1.lpf_time_constant = 0.12;

  // Setup gantry
  //   gantry = new Gantry::Gantry("gantry");
  gantry.setup(motor_params_ch0, motor_params_ch1, position_pid_params_ch0,
               position_pid_params_ch1, velocity_pid_params_ch0,
               velocity_pid_params_ch1);

  delay(1000);

  position_pid_p_ch0.set_post_callback(on_pid_update);
  position_pid_i_ch0.set_post_callback(on_pid_update);
  position_pid_d_ch0.set_post_callback(on_pid_update);
  position_pid_lpf_ch0.set_post_callback(on_pid_update);

  position_pid_p_ch1.set_post_callback(on_pid_update);
  position_pid_i_ch1.set_post_callback(on_pid_update);
  position_pid_d_ch1.set_post_callback(on_pid_update);
  position_pid_lpf_ch1.set_post_callback(on_pid_update);

  velocity_pid_p_ch0.set_post_callback(on_pid_update);
  velocity_pid_i_ch0.set_post_callback(on_pid_update);
  velocity_pid_d_ch0.set_post_callback(on_pid_update);
  velocity_pid_lpf_ch0.set_post_callback(on_pid_update);

  velocity_pid_p_ch1.set_post_callback(on_pid_update);
  velocity_pid_i_ch1.set_post_callback(on_pid_update);
  velocity_pid_d_ch1.set_post_callback(on_pid_update);
  velocity_pid_lpf_ch1.set_post_callback(on_pid_update);

  // Start server
  ESPWifiConfig::WebServer::getInstance().start();

  if (!MDNS.begin("gantry"))
  {  // Start the mDNS responder for esp32.local  MDNS.addService("http", "tcp",
     // 8080);

    Serial.println("Error setting up MDNS responder!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  MDNS.addService("http", "tcp", 8080);
  MDNS.addServiceTxt("http", "tcp", "gantry", "sentry0");

  xTaskCreatePinnedToCore(
      loop_motorgo,       /* Task function. */
      "Loop MotorGo",     /* name of task. */
      10000,              /* Stack size of task */
      NULL,               /* parameter of the task */
      1,                  /* priority of the task */
      &loop_motorgo_task, /* Task handle to keep track of created task */
      1);                 /* pin task to core 1 */
}

void loop_motorgo(void* pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    gantry.loop();

    //   Print loop frequency
    esp_task_wdt_reset();
  }
}

int i = 0;
// Constrain loop speed to 250 Hz
unsigned long last_loop_time = 0;
float vel = 30;
void loop()
{
  // Do nothing
}

void on_pid_update(const float& updated_value)
{
  gantry.update_pid_params(position_pid_params_ch0, position_pid_params_ch1,
                           velocity_pid_params_ch0, velocity_pid_params_ch1);
}
