#include "gantry.h"

#include "Preferences.h"

namespace Gantry
{

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

// Constructor implementation
Gantry::Gantry(String robot_name)
    : name(robot_name),
      motorgo_mini(MotorGo::MotorGoMini()),
      config_mode(mode_int, "/mode", "Mode"),
      config_target_speed(target_speed, "/target_speed", "Target Speed"),
      config_speed_multiplier_q0(speed_multiplier_q0, "/speed_multiplier/q0",
                                 "Speed Multiplier q0"),
      config_speed_multiplier_q1(speed_multiplier_q1, "/speed_multiplier/q1",
                                 "Speed Multiplier q1"),
      config_target_waypoint_index(target_waypoint_index, "/target_waypoint",
                                   "Target Waypoint Index"),
      position_q0([this]() { return motorgo_mini.get_ch0_position(); },
                  "/position/q0", "Position q0"),
      position_q1([this]() { return motorgo_mini.get_ch1_position(); },
                  "/position/q1", "Position q1"),
      next_waypoint_q0([this]() { return get_next_waypoint_q0(); },
                       "/next_waypoint/q0", "Next Waypoint q0"),
      next_waypoint_q1([this]() { return get_next_waypoint_q1(); },
                       "/next_waypoint/q1", "Next Waypoint q1"),
      previous_waypoint_q0([this]() { return get_previous_waypoint_q0(); },
                           "/previous_waypoint/q0", "Previous Waypoint q0"),
      previous_waypoint_q1([this]() { return get_previous_waypoint_q1(); },
                           "/previous_waypoint/q1", "Previous Waypoint q1"),
      add_waypoint([this]() { return add_waypoint_callback(); },
                   "/add_waypoint", "Add Waypoint"),
      save_trajectory([this]() { return save_trajectory_callback(); },
                      "/save_trajectory", "Save Trajectory"),
      trajectory_length([this]() { return trajectory.length; },
                        "/trajectory_length", "Trajectory Length")
{
  config_mode.set_post_callback([this](int mode) { on_mode_change(mode); });

  config_target_speed.set_post_callback([this](float target_speed)
                                        { on_target_speed_change(); });
  config_speed_multiplier_q0.set_post_callback(
      [this](float multiplier) { on_speed_multiplier_q0_change(); });
  config_speed_multiplier_q1.set_post_callback(
      [this](float multiplier) { on_speed_multiplier_q1_change(); });

  //   config_target_waypoint_index.set_post_callback(
  //       [this](int index) { on_target_waypoint_index_change(); });
}

// Define other class member functions

void Gantry::setup(MotorGo::MotorParameters motor_params_ch0,
                   MotorGo::MotorParameters motor_params_ch1,
                   MotorGo::PIDParameters position_pid_params_ch0,
                   MotorGo::PIDParameters position_pid_params_ch1,
                   MotorGo::PIDParameters velocity_pid_params_ch0,
                   MotorGo::PIDParameters velocity_pid_params_ch1)
{
  motorgo_mini.init_ch0(motor_params_ch0, false, false);
  this->motor_params_ch0 = motor_params_ch0;
  this->position_pid_params_ch0 = position_pid_params_ch0;
  this->position_pid_params_ch1 = position_pid_params_ch1;
  this->velocity_pid_params_ch0 = velocity_pid_params_ch0;
  this->velocity_pid_params_ch1 = velocity_pid_params_ch1;

  //   Set controller parameters
  //   Update the velocity limit based on the target speed and speed
  //   multiplier
  position_pid_params_ch0.limit = target_speed * speed_multiplier_q0;
  motorgo_mini.set_velocity_controller_ch0(velocity_pid_params_ch0);
  motorgo_mini.set_position_controller_ch0(position_pid_params_ch0);
  motorgo_mini.set_control_mode_ch0(MotorGo::ControlMode::Position);

  motorgo_mini.init_ch1(motor_params_ch1, false, false);
  this->motor_params_ch1 = motor_params_ch1;
  position_pid_params_ch1.limit = target_speed * speed_multiplier_q1;
  motorgo_mini.set_velocity_controller_ch1(velocity_pid_params_ch1);
  motorgo_mini.set_position_controller_ch1(position_pid_params_ch1);
  motorgo_mini.set_control_mode_ch1(MotorGo::ControlMode::Position);

  //   Zero the position
  zero();

  //   Using preferences, try to load the trajectory
  //   Preferences preferences;
  //   preferences.begin("gantry", false);
  //   //   Try to load trajectory
  //   if (preferences.isKey("traj"))
  //   {
  //     Serial.println("Trajectory found");
  //     preferences.getBytes("traj", &trajectory.raw, TRAJECTORY_DATA_LEN);
  //   }
  //   else
  //   {
  //     Serial.println("Trajectory not found");
  //   }

  //   preferences.end();

  // Trajectory is empty
  if (trajectory.length == 0)
  {
    Serial.println("Trajectory is empty");
    // Set first waypoint to the origin
    trajectory.length = 1;
    trajectory.waypoints[0] = {0.0, 0.0};
  }
}

void Gantry::update_pid_params(MotorGo::PIDParameters position_pid_params_ch0,
                               MotorGo::PIDParameters position_pid_params_ch1,
                               MotorGo::PIDParameters velocity_pid_params_ch0,
                               MotorGo::PIDParameters velocity_pid_params_ch1)
{
  this->position_pid_params_ch0 = position_pid_params_ch0;
  this->position_pid_params_ch1 = position_pid_params_ch1;
  this->velocity_pid_params_ch0 = velocity_pid_params_ch0;
  this->velocity_pid_params_ch1 = velocity_pid_params_ch1;

  //   Set controller parameters
  motorgo_mini.disable_ch0();
  motorgo_mini.disable_ch1();

  motorgo_mini.set_velocity_controller_ch0(this->velocity_pid_params_ch0);
  motorgo_mini.set_position_controller_ch0(this->position_pid_params_ch0);
  motorgo_mini.set_velocity_controller_ch1(this->velocity_pid_params_ch1);
  motorgo_mini.set_position_controller_ch1(this->position_pid_params_ch1);

  motorgo_mini.enable_ch0();
  motorgo_mini.enable_ch1();
}

void Gantry::zero()
{
  motorgo_mini.zero_position_ch0();
  motorgo_mini.zero_position_ch1();
}

void Gantry::loop()
{
  if (mode == Mode::IDLE)
  {
    return;
  }
  else if (mode == Mode::TRAJECTORY_TRACK)
  {
    // Set position to the target waypoint

    if (target_waypoint_index >= trajectory.length)
    {
      target_waypoint_index = trajectory.length - 1;
    }
    else if (target_waypoint_index < 0)
    {
      target_waypoint_index = 0;
    }

    position_t target_waypoint = trajectory.waypoints[target_waypoint_index];
    motorgo_mini.set_target_position_ch0(target_waypoint.q0);
    motorgo_mini.set_target_position_ch1(target_waypoint.q1);

    // Print encoder valules
    // char str_buffer[100];
    // sprintf(str_buffer, "Ch0: %f, Ch1: %f\n",
    // motorgo_mini.get_ch0_velocity(),
    //         motorgo_mini.get_ch1_velocity());

    // freq_print(5, str_buffer);
  }
  else if (mode == Mode::TRAJECTORY_RECORD)
  {
    // Do something
  }
  motorgo_mini.loop_ch0();
  motorgo_mini.loop_ch1();
}

void Gantry::on_mode_change(int mode)
{
  if (mode == 0)
  {
    Serial.println("Idle mode enabled");
    this->mode = Mode::IDLE;
  }
  else if (mode == 1)
  {
    Serial.println("Trajectory record enabled");

    this->mode = Mode::TRAJECTORY_RECORD;
    enable_trajectory_record();
  }
  else if (mode == 2)
  {
    Serial.println("Trajectory track enabled");

    enable_trajectory_track();
    this->mode = Mode::TRAJECTORY_TRACK;
    // on_target_waypoint_index_change();
  }
}

void Gantry::on_speed_multiplier_q0_change()
{
  Serial.print("Speed multiplier q0: ");
  Serial.println(speed_multiplier_q0);
  position_pid_params_ch0.limit = target_speed * speed_multiplier_q0;

  Serial.print("Velocity limit q0: ");
  Serial.println(position_pid_params_ch0.limit, 10);
  motorgo_mini.set_position_controller_ch0(position_pid_params_ch0);
}

void Gantry::on_speed_multiplier_q1_change()
{
  Serial.print("Speed multiplier q1: ");
  Serial.println(speed_multiplier_q1);
  position_pid_params_ch1.limit = target_speed * speed_multiplier_q1;
  Serial.print("Velocity limit q1: ");
  Serial.println(position_pid_params_ch1.limit, 10);
  motorgo_mini.set_position_controller_ch1(position_pid_params_ch1);
}

void Gantry::on_target_speed_change()
{
  Serial.print("Target speed updated to: ");
  Serial.println(target_speed);
  on_speed_multiplier_q0_change();
  on_speed_multiplier_q1_change();
}

// void Gantry::on_target_waypoint_index_change()
// {
//   Serial.print("Handling new target waypoint index: ");
//   Serial.println(target_waypoint_index);
//   Serial.println("---------------------------------------");

//   //   Compute delta per timestep to achieve desired velocity
//   //   First, determine direction of rotation by comparing current position
//   //   to
//   // target waypoint

//   //   Get current position
//   position_t current_position = {motorgo_mini.get_ch0_position(),
//                                  motorgo_mini.get_ch1_position()};
//   //   Get target waypoint
//   position_t target_waypoint = trajectory.waypoints[target_waypoint_index];

//   //   //   Compute sign
//   q0_sign = (target_waypoint.q0 - current_position.q0) > 0 ? 1.0 : -1.0;
//   q1_sign = (target_waypoint.q1 - current_position.q1) > 0 ? 1.0 : -1.0;

//   //   //   Compute delta based on loop speed (60 hz) and position controller
//   //   limit delta_q0 = q0_sign * position_pid_params_ch0.limit / 60.0;
//   delta_q1
//   //   = q1_sign * position_pid_params_ch1.limit / 60.0;

//   //   //   Print delta with high precision
//   //   Serial.print("Delta q0: ");
//   //   Serial.println(delta_q0, 10);
//   //   Serial.print("Delta q1: ");
//   //   Serial.println(delta_q1, 10);
// }

position_t Gantry::get_waypoint_by_offset(int offset)
{
  if (trajectory.length == 0)
  {
    return {0.0, 0.0};
  }

  // Confirm that the offset is within the trajectory
  if (target_waypoint_index + offset < trajectory.length &&
      target_waypoint_index + offset >= 0)
  {
    return trajectory.waypoints[target_waypoint_index + offset];
  }
  else
  {
    return trajectory.waypoints[target_waypoint_index];
  }
}

float Gantry::get_next_waypoint_q0()
{
  position_t next_waypoint = get_waypoint_by_offset(1);
  return next_waypoint.q0;
}

float Gantry::get_next_waypoint_q1()
{
  position_t next_waypoint = get_waypoint_by_offset(1);
  return next_waypoint.q1;
}

float Gantry::get_previous_waypoint_q0()
{
  position_t previous_waypoint = get_waypoint_by_offset(-1);
  return previous_waypoint.q0;
}

float Gantry::get_previous_waypoint_q1()
{
  position_t previous_waypoint = get_waypoint_by_offset(-1);
  return previous_waypoint.q1;
}

void Gantry::enable_trajectory_record()
{
  // Disable both motors
  motorgo_mini.disable_ch0();
  motorgo_mini.disable_ch1();

  //   Reset temp trajectory
  temp_trajectory.length = 0;
  //   temp_trajectory.waypoints[0] = {0.0, 0.0};
}

void Gantry::enable_trajectory_track()
{
  // Enable both motors
  motorgo_mini.enable_ch0();
  motorgo_mini.enable_ch1();

  // Set the target waypoint to the first waypoint
  //   target_waypoint_index = 1;

  //   Print the trajectory
  for (int i = 0; i < trajectory.length; i++)
  {
    Serial.print(trajectory.waypoints[i].q0);
    Serial.print(", ");
    Serial.println(trajectory.waypoints[i].q1);
  }
  //   Print length
  Serial.print("Trajectory length: ");
  Serial.println(trajectory.length);
}

bool Gantry::add_waypoint_callback()
{
  // Check if we are in trajectory record mode
  if (mode != Mode::TRAJECTORY_RECORD)
  {
    return false;
  }

  //   Confirm that the trajectory is not full
  if (temp_trajectory.length >= 10)
  {
    return false;
  }

  //   Add the current position to the trajectory
  position_t current_position = {motorgo_mini.get_ch0_position(),
                                 motorgo_mini.get_ch1_position()};
  temp_trajectory.waypoints[temp_trajectory.length] = current_position;
  temp_trajectory.length++;

  Serial.print("Added waypoint to trajectory: ");
  Serial.print(current_position.q0);
  Serial.print(", ");
  Serial.println(current_position.q1);

  return true;
}

bool Gantry::save_trajectory_callback()
{
  if (mode != Mode::TRAJECTORY_RECORD)
  {
    return false;
  }

  //   Serial.println("Temp");
  //   for (int i = 0; i < temp_trajectory.length; i++)
  //   {
  //     Serial.print(temp_trajectory.waypoints[i].q0);
  //     Serial.print(", ");
  //     Serial.println(temp_trajectory.waypoints[i].q1);
  //   }

  //   Copy the temp trajectory to the trajectory
  trajectory = temp_trajectory;

  //   Serial.print("Same");
  //   //   Print the trajectory
  //   for (int i = 0; i < trajectory.length; i++)
  //   {
  //     Serial.print(trajectory.waypoints[i].q0);
  //     Serial.print(", ");
  //     Serial.println(trajectory.waypoints[i].q1);
  //   }

  //   Use preferences to write
  //   Preferences preferences;
  //   preferences.begin("gantry", true);
  //   preferences.putBytes("traj", &trajectory.raw, TRAJECTORY_DATA_LEN);
  //   preferences.end();

  //   Serial.print("Saved trajectory of length ");
  //   Serial.println(trajectory.length);

  //   Print the trajectory
  //   for (int i = 0; i < trajectory.length; i++)
  //   {
  //     Serial.print(trajectory.waypoints[i].q0);
  //     Serial.print(", ");
  //     Serial.println(trajectory.waypoints[i].q1);
  //   }

  return true;
}

}  // namespace Gantry
