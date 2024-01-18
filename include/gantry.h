#ifndef GANTRY_H
#define GANTRY_H

#include "configurable.h"
#include "motorgo_mini.h"
#include "readable.h"
#include "trajectory_utils.h"

#define MAX_TRAJECTORY_LEN 10

namespace Gantry
{

// using ZeroCallbackType = void (*)(const String&);

typedef struct
{
  float q0;
  float q1;
} position_t;

typedef struct
{
  float qdot0;
  float qdot1;
} velocity_t;

// Create enum for current mode
enum class Mode
{
  IDLE,
  TRAJECTORY_RECORD,
  TRAJECTORY_TRACK,
};

// Type for saving and loading calibration parameters to/from EEPROM
const int TRAJECTORY_DATA_LEN =
    MAX_TRAJECTORY_LEN * sizeof(position_t) + sizeof(int);
typedef union
{
  struct __attribute__((packed))
  {
    // Trajectory waypoints
    position_t waypoints[MAX_TRAJECTORY_LEN];
    int length = 0;
  };

  uint8_t raw[TRAJECTORY_DATA_LEN];
} trajectory_data_t;

class Gantry
{
 private:
  MotorGo::MotorGoMini motorgo_mini;
  String name;

  //   Current operation mode
  Mode mode = Mode::IDLE;
  int mode_int = 0;
  ESPWifiConfig::Configurable<int> config_mode;

  //   Trajectory is a sequence of position_t waypoints
  // Max length: 10
  trajectory_data_t trajectory;
  //   Used for trajectory recording
  trajectory_data_t temp_trajectory;

  // Target speed is shared across all actuators, represeting a max speed
  // Speed multiplier is a multiplier for the target speed to adjust the target
  // speed so all actuators reach the goal at the same time
  float target_speed = 0.0;
  float speed_multiplier_q0 = 1.0;
  float speed_multiplier_q1 = 1.0;

  float q0_sign = 0.0;
  float q1_sign = 0.0;

  ESPWifiConfig::Configurable<float> config_target_speed;
  ESPWifiConfig::Configurable<float> config_speed_multiplier_q0;
  ESPWifiConfig::Configurable<float> config_speed_multiplier_q1;

  int target_waypoint_index = 0;
  ESPWifiConfig::Configurable<int> config_target_waypoint_index;

  MotorGo::MotorParameters motor_params_ch0;
  MotorGo::MotorParameters motor_params_ch1;

  MotorGo::PIDParameters position_pid_params_ch0;
  MotorGo::PIDParameters position_pid_params_ch1;
  MotorGo::PIDParameters velocity_pid_params_ch0;
  MotorGo::PIDParameters velocity_pid_params_ch1;

  //   Define readables for current position
  ESPWifiConfig::Readable<float> position_q0;
  ESPWifiConfig::Readable<float> position_q1;
  ESPWifiConfig::Readable<float> next_waypoint_q0;
  ESPWifiConfig::Readable<float> next_waypoint_q1;
  ESPWifiConfig::Readable<float> previous_waypoint_q0;
  ESPWifiConfig::Readable<float> previous_waypoint_q1;

  //   Readable for writing waypoint
  ESPWifiConfig::Readable<bool> add_waypoint;
  ESPWifiConfig::Readable<bool> save_trajectory;
  ESPWifiConfig::Readable<int> trajectory_length;

  position_t get_waypoint_by_offset(int offset);
  float get_next_waypoint_q0();
  float get_next_waypoint_q1();
  float get_previous_waypoint_q0();
  float get_previous_waypoint_q1();

  void on_mode_change(int mode);
  void on_target_speed_change();
  void on_speed_multiplier_q0_change();
  void on_speed_multiplier_q1_change();

  void on_target_waypoint_index_change();

  void enable_trajectory_record();
  void enable_trajectory_track();

  bool add_waypoint_callback();
  bool save_trajectory_callback();

  void define_pid_endpoint(const char* path, float& pid_param);

 public:
  explicit Gantry(String robot_name);

  void setup(MotorGo::MotorParameters motor_params_ch0,
             MotorGo::MotorParameters motor_params_ch1,
             MotorGo::PIDParameters position_pid_params_ch0,
             MotorGo::PIDParameters position_pid_params_ch1,
             MotorGo::PIDParameters velocity_pid_params_ch0,
             MotorGo::PIDParameters velocity_pid_params_ch1);

  void update_pid_params(MotorGo::PIDParameters position_pid_params_ch0,
                         MotorGo::PIDParameters position_pid_params_ch1,
                         MotorGo::PIDParameters velocity_pid_params_ch0,
                         MotorGo::PIDParameters velocity_pid_params_ch1);
  void zero();
  void loop();
};

}  // namespace Gantry
#endif  // GANTRY_H
