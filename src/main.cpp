#include <Arduino.h>

#include "motorgo_mini.h"
#include "trajectory_utils.cpp"

#define LOOP_HZ 1000
#define LOOP_US 1000000 / LOOP_HZ

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;

waypoint_t start = {0.0, 0.0};
waypoint_t end = {5.0, 0.5};
waypoint_t* trajectory;

int cur_waypoint = 0;

int update_waypoint(int cur_waypoint, int joystick_input);

void setup()
{
  Serial.begin(115200);

  // Setup motor parameters
  motor_params_ch0.pole_pairs = 11;
  motor_params_ch0.power_supply_voltage = 9.0;
  motor_params_ch0.voltage_limit = 9.0;
  motor_params_ch0.current_limit = 320;
  motor_params_ch0.velocity_limit = 100.0;
  motor_params_ch0.calibration_voltage = 2.0;

  motor_params_ch1.pole_pairs = 11;
  motor_params_ch1.power_supply_voltage = 9.0;
  motor_params_ch1.voltage_limit = 9.0;
  motor_params_ch1.current_limit = 320;
  motor_params_ch1.velocity_limit = 100.0;
  motor_params_ch1.calibration_voltage = 2.0;

  // Setup PID parameters
  position_pid_params_ch0.p = 2.0;
  position_pid_params_ch0.i = 0.0;
  position_pid_params_ch0.d = 0.0;
  position_pid_params_ch0.output_ramp = 10000.0;
  position_pid_params_ch0.lpf_time_constant = 0.1;

  position_pid_params_ch1.p = 2.0;
  position_pid_params_ch1.i = 0.0;
  position_pid_params_ch1.d = 0.0;
  position_pid_params_ch1.output_ramp = 10000.0;
  position_pid_params_ch1.lpf_time_constant = 0.1;

  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup both channels
  motorgo_mini->init_ch0(motor_params_ch0, false, false);
  motorgo_mini->init_ch1(motor_params_ch1, false, false);

  // Set position controller parameters
  motorgo_mini->set_position_controller_ch0(position_pid_params_ch0);
  motorgo_mini->set_position_controller_ch1(position_pid_params_ch1);

  // Set closed-loop velocity control mode
  motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Position);
  motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Position);

  // Define array for trajectory
  int trajectory_len = get_trajectory_len(start, end, 0.1);

  // Allocate memory for trajectory
  trajectory = new waypoint_t[trajectory_len];
  linear_interpolate(start, end, trajectory, 0.1);
}

int i = 0;
// Constrain loop speed to 250 Hz
unsigned long last_loop_time = 0;
void loop()
{
  // Run Ch0
  motorgo_mini->loop_ch0();
  motorgo_mini->loop_ch1();

  //   Spin forward
  //   Enable
  motorgo_mini->set_target_position_ch0(trajectory[cur_waypoint].q0);
  motorgo_mini->set_target_position_ch1(trajectory[cur_waypoint].q1);

  // Update current waypoint from joystick
  // TODO: Read joystick data
  cur_waypoint = update_waypoint(cur_waypoint, 0);

  // Delay necessary amount micros
  unsigned long now = micros();
  unsigned long loop_time = now - last_loop_time;

  if (loop_time < LOOP_US)
  {
    delayMicroseconds(LOOP_US - loop_time);

    // Serial.print("Loop time: ");
  }

  last_loop_time = now;
}
