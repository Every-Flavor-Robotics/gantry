#include <Arduino_JSON.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

#include "motorgo_mini.h"

// Replace with your network credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

int target_waypoint_variable = 0;
MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;
MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

AsyncWebServer server(8080);
String session_id = "";  // Variable to store the session ID

// Helper function to handle PID POST requests
void handle_pid_post(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                     float& value)
{
  Serial.println("PID POST request received");
  String payload = String((char*)data);

  JSONVar json_object = JSON.parse(payload);

  if (JSON.typeof(json_object) != "undefined" &&
      json_object.hasOwnProperty("value"))
  {
    value = static_cast<float>((double)json_object["value"]);
    request->send(200, "text/plain", "PID value set successfully.");
  }
  else
  {
    request->send(400, "text/plain", "Invalid request.");
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(8, OUTPUT);

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Establishing connection to WiFi...");
  }

  Serial.println("Connected to WiFi");
  // Print IP
  Serial.println(WiFi.localIP());

  // Define endpoints
  // Handle POST request for setting session_id
  server.on(
      "/", HTTP_POST, [](AsyncWebServerRequest* request) {}, NULL,
      [](AsyncWebServerRequest* request, uint8_t* data, size_t len,
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
          session_id = (const char*)json_object["session_id"];
          request->send(200, "text/plain", "Session ID received");
        }
        else
        {
          request->send(400, "text/plain",
                        "No session_id provided in JSON body");
        }
      });

  // Handle GET request for checking session_id
  server.on("/", HTTP_GET,
            [](AsyncWebServerRequest* request)
            {
              if (request->hasHeader("session_id"))
              {
                String received_session_id = request->header("session_id");
                if (received_session_id == session_id)
                {
                  request->send(200, "application/json", "{}");
                  // Blink GPIO 8
                  digitalWrite(8, HIGH);
                  delayMicroseconds(100);
                  digitalWrite(8, LOW);
                }
                else
                {
                  request->send(403, "text/plain",
                                "Forbidden: Incorrect session ID");
                }
              }
              else
              {
                request->send(400, "text/plain", "No session_id provided");
              }
            });

  // Handle POST request for setting target waypoint
  server.on(
      "/target_waypoint", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [](AsyncWebServerRequest* request, uint8_t* data, size_t len,
         size_t index, size_t total)
      {
        String payload = String((char*)data);
        JSONVar json_object = JSON.parse(payload);

        if (JSON.typeof(json_object) != "undefined" &&
            json_object.hasOwnProperty("value"))
        {
          target_waypoint_variable = (int)json_object["value"];
          request->send(200, "text/plain", "Waypoint value set successfully.");
        }
        else
        {
          request->send(400, "text/plain", "Invalid request.");
        }
      });

  // ch0 - Position
  server.on(
      "/pid/ch0/position/p", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, position_pid_params_ch0.p); });
  server.on("/pid/ch0/position/i", HTTP_POST, nullptr, nullptr,
            [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
                size_t index, size_t total) {
              handle_pid_post(request, data, len, position_pid_params_ch0.i);
            });
  server.on(
      "/pid/ch0/position/d", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, position_pid_params_ch0.d); });

  // ch1 - Position
  server.on(
      "/pid/ch1/position/p", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, position_pid_params_ch1.p); });
  server.on(
      "/pid/ch1/position/i", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, position_pid_params_ch1.i); });
  server.on(
      "/pid/ch1/position/d", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, position_pid_params_ch1.d); });

  // ch0 - Velocity
  server.on(
      "/pid/ch0/velocity/p", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, velocity_pid_params_ch0.p); });
  server.on(
      "/pid/ch0/velocity/i", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, velocity_pid_params_ch0.i); });
  server.on(
      "/pid/ch0/velocity/d", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, velocity_pid_params_ch0.d); });

  // ch1 - Velocity
  server.on(
      "/pid/ch1/velocity/p", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, velocity_pid_params_ch1.p); });
  server.on(
      "/pid/ch1/velocity/i", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, velocity_pid_params_ch1.i); });
  server.on(
      "/pid/ch1/velocity/d", HTTP_POST, [](AsyncWebServerRequest* request) {},
      NULL,
      [&](AsyncWebServerRequest* request, uint8_t* data, size_t len,
          size_t index, size_t total)
      { handle_pid_post(request, data, len, velocity_pid_params_ch1.d); });

  server.begin();
}

void loop()
{
  //   Serial.println(pid_ch0_position_p);
  //   delay(1000);
}
