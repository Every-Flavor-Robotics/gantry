#include "pid_web_server.h"

void PIDWebServer::define_pid_endpoint(const char* path, float& pid_param)
{
  server.on(
      path, HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
      [&, this](AsyncWebServerRequest* request, uint8_t* data, size_t len,
                size_t index, size_t total)
      { handle_pid_post(request, data, len, pid_param, onPidChanged); });
}

void PIDWebServer::handle_pid_post(AsyncWebServerRequest* request,
                                   uint8_t* data, size_t len, float& value,
                                   CallbackType callback)
{
  Serial.println("PID POST request received");
  String payload = String((char*)data);

  JSONVar json_object = JSON.parse(payload);

  if (JSON.typeof(json_object) != "undefined" &&
      json_object.hasOwnProperty("value"))
  {
    value = static_cast<float>((double)json_object["value"]);
    Serial.print("PID value set to: ");
    Serial.println(value);

    request->send(200, "text/plain", "PID value set successfully.");
    if (callback) callback(payload);
  }
  else
  {
    request->send(400, "text/plain", "Invalid request.");
  }
}

void PIDWebServer::handle_target_waypoint_post(AsyncWebServerRequest* request,
                                               uint8_t* data, size_t len)
{
  String payload = String((char*)data);
  JSONVar json_object = JSON.parse(payload);

  if (JSON.typeof(json_object) != "undefined" &&
      json_object.hasOwnProperty("value"))
  {
    target_waypoint = (int)json_object["value"];
    request->send(200, "text/plain", "Waypoint value set successfully.");
    if (onWaypointSet) onWaypointSet(payload);
  }
  else
  {
    request->send(400, "text/plain", "Invalid request.");
  }
}

PIDWebServer::PIDWebServer(int port) : server(port)
{
  // Set default PID parameters
  position_pid_params_ch0.p = 0.0;
  position_pid_params_ch0.i = 0.0;
  position_pid_params_ch0.d = 0.0;
  position_pid_params_ch0.output_ramp = 10000.0;
  position_pid_params_ch0.lpf_time_constant = 0.15;

  position_pid_params_ch1.p = 0.0;
  position_pid_params_ch1.i = 0.0;
  position_pid_params_ch1.d = 0.0;
  position_pid_params_ch1.output_ramp = 10000.0;
  position_pid_params_ch1.lpf_time_constant = 0.15;

  velocity_pid_params_ch0.p = 0.0;
  velocity_pid_params_ch0.i = 0.0;
  velocity_pid_params_ch0.d = 0.0;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.15;

  velocity_pid_params_ch1.p = 0.0;
  velocity_pid_params_ch1.i = 0.0;
  velocity_pid_params_ch1.d = 0.0;
  velocity_pid_params_ch1.output_ramp = 10000.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.15;
}

PIDWebServer::~PIDWebServer() { stop(); }

void PIDWebServer::setSessionIdCallback(CallbackType callback)
{
  onSessionIdReceived = callback;
}

void PIDWebServer::setWaypointCallback(CallbackType callback)
{
  onWaypointSet = callback;
}

void PIDWebServer::setPidCallback(CallbackType callback)
{
  onPidChanged = callback;
}

// ... Implementation for other callback setters

void PIDWebServer::start()
{
  // Setup GPIO pin for Green LED
  pinMode(8, OUTPUT);

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Establishing connection to WiFi...");
    // Print Credentials
    Serial.println(WIFI_SSID);
    Serial.println(WIFI_PASSWORD);
  }

  Serial.println("Connected to WiFi");
  // Print IP
  Serial.println(WiFi.localIP());

  //... Endpoint definitions, e.g.:
  server.on(
      "/", HTTP_POST, [](AsyncWebServerRequest* request) {}, nullptr,
      [this](AsyncWebServerRequest* request, uint8_t* data, size_t len,
             size_t index, size_t total)
      {
        // Check content type
        if (request->contentType() != "application/json")
        {
          request->send(400, "text/plain",
                        "Content-Type must be application/json");
          return;
        }

        // Convert received data into a String
        String request_body = String((char*)data);

        // Parse the JSON body
        JSONVar json_object = JSON.parse(request_body);

        // Check if parsing succeeded
        if (JSON.typeof(json_object) == "undefined")
        {
          request->send(500, "text/plain", "Failed to parse JSON");
          return;
        }

        // Check if the JSON contains the session_id field
        if (json_object.hasOwnProperty("session_id"))
        {
          this->session_id = (const char*)json_object["session_id"];
          request->send(200, "text/plain", "Session ID received");
        }
        else
        {
          request->send(400, "text/plain",
                        "No session_id provided in JSON body");
        }

        if (onSessionIdReceived) onSessionIdReceived(this->session_id);
      });

  // Handle GET request for checking session_id
  server.on(
      "/", HTTP_GET,
      [this](AsyncWebServerRequest* request)  // Capture 'this' pointer
      {
        if (request->hasHeader("session_id"))
        {
          String received_session_id = request->header("session_id");
          if (received_session_id ==
              this->session_id)  // Use 'this' to access member variable
          {
            request->send(200, "application/json", "{\"status\": \"success\"}");
            // Blink GPIO 8
            digitalWrite(8, HIGH);
            delayMicroseconds(100);
            digitalWrite(8, LOW);
          }
          else
          {
            request->send(403, "text/plain", "Forbidden: Incorrect session ID");
          }
        }
        else
        {
          request->send(400, "text/plain", "No session_id provided");
        }
      });

  // Route for target waypoint
  server.on(
      "/target_waypoint", HTTP_POST, [](AsyncWebServerRequest* request) {},
      nullptr,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_target_waypoint_post(request, data, len); });

  // Endpoints for PID parameters
  // ch0 - Position
  define_pid_endpoint("/pid/ch0/position/p", position_pid_params_ch0.p);
  define_pid_endpoint("/pid/ch0/position/i", position_pid_params_ch0.i);
  define_pid_endpoint("/pid/ch0/position/d", position_pid_params_ch0.d);

  // ch1 - Position
  define_pid_endpoint("/pid/ch1/position/p", position_pid_params_ch1.p);
  define_pid_endpoint("/pid/ch1/position/i", position_pid_params_ch1.i);
  define_pid_endpoint("/pid/ch1/position/d", position_pid_params_ch1.d);

  // ch0 - Velocity
  define_pid_endpoint("/pid/ch0/velocity/p", velocity_pid_params_ch0.p);
  define_pid_endpoint("/pid/ch0/velocity/i", velocity_pid_params_ch0.i);
  define_pid_endpoint("/pid/ch0/velocity/d", velocity_pid_params_ch0.d);

  // ch1 - Velocity
  define_pid_endpoint("/pid/ch1/velocity/p", velocity_pid_params_ch1.p);
  define_pid_endpoint("/pid/ch1/velocity/i", velocity_pid_params_ch1.i);
  define_pid_endpoint("/pid/ch1/velocity/d", velocity_pid_params_ch1.d);

  server.begin();
}

void PIDWebServer::stop() { server.end(); }

MotorGo::PIDParameters PIDWebServer::get_position_pid_params_ch0()
{
  return position_pid_params_ch0;
}

MotorGo::PIDParameters PIDWebServer::get_position_pid_params_ch1()
{
  return position_pid_params_ch1;
}

MotorGo::PIDParameters PIDWebServer::get_velocity_pid_params_ch0()
{
  return velocity_pid_params_ch0;
}

MotorGo::PIDParameters PIDWebServer::get_velocity_pid_params_ch1()
{
  return velocity_pid_params_ch1;
}

int PIDWebServer::get_target_waypoint() { return target_waypoint; }
