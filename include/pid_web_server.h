#ifndef PID_WEBSERVER_H
#define PID_WEBSERVER_H

#include <Arduino_JSON.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

#include "motorgo_mini.h"

using CallbackType = void (*)(const String&);

class PIDWebServer
{
 private:
  AsyncWebServer server;
  String session_id;

  int target_waypoint;
  MotorGo::PIDParameters position_pid_params_ch0;
  MotorGo::PIDParameters position_pid_params_ch1;
  MotorGo::PIDParameters velocity_pid_params_ch0;
  MotorGo::PIDParameters velocity_pid_params_ch1;

  CallbackType onSessionIdReceived = nullptr;
  CallbackType onWaypointSet = nullptr;
  CallbackType onPidChanged = nullptr;

  void handle_pid_post(AsyncWebServerRequest* request, uint8_t* data,
                       size_t len, float& value, CallbackType callback);
  void handle_target_waypoint_post(AsyncWebServerRequest* request,
                                   uint8_t* data, size_t len);

  void define_pid_endpoint(const char* path, float& pid_param);

 public:
  PIDWebServer(int port = 8080);
  ~PIDWebServer();

  void setSessionIdCallback(CallbackType callback);
  void setWaypointCallback(CallbackType callback);
  void setPidCallback(CallbackType callback);

  void start();
  void stop();

  MotorGo::PIDParameters get_position_pid_params_ch0();
  MotorGo::PIDParameters get_position_pid_params_ch1();
  MotorGo::PIDParameters get_velocity_pid_params_ch0();
  MotorGo::PIDParameters get_velocity_pid_params_ch1();

  int get_target_waypoint();
};

#endif  // PID_WEBSERVER_H
