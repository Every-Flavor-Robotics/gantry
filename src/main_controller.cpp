// #include <ESPmDNS.h>
// #include <WiFi.h>

// #include <vector>

// struct GantryService
// {
//   IPAddress ip;
//   uint16_t port;
// };

// std::vector<GantryService> gantry_services;

// bool service_exists(const IPAddress& ip, const uint16_t& p)
// {
//   for (const auto& service : gantry_services)
//   {
//     if (service.ip == ip && service.port == p)
//     {
//       return true;
//     }
//   }
//   return false;
// }

// void setup()
// {
//   Serial.begin(115200);
//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(1000);
//     Serial.println("Connecting to WiFi...");
//   }
//   Serial.println("Connected to WiFi");

//   if (!MDNS.begin("esp32"))
//   {
//     Serial.println("Error setting up mDNS responder!");
//     while (1)
//     {
//       delay(1000);
//     }
//   }
//   Serial.println("mDNS responder started");
// }

// void loop()
// {
//   int n = MDNS.queryService("http", "tcp");

//   if (n == 0)
//   {
//     Serial.println("No services found");
//     delay(5000);
//     return;
//   }

//   for (int i = 0; i < n; ++i)
//   {
//     if (String(MDNS.hostname(i)).indexOf("gantry") != -1)
//     {
//       bool txt_found = false;

//       // Accessing TXT records
//       int txt_records = MDNS.hasTxt(i, "gantry");
//       for (int j = 0; j < txt_records; j++)
//       {
//         if (String(MDNS.txtKey(i, j)) == "gantry" &&
//             String(MDNS.txt(i, j)) == "gantry0")
//         {
//           txt_found = true;
//           break;
//         }
//       }

//       if (txt_found && !service_exists(MDNS.IP(i), MDNS.port(i)))
//       {
//         GantryService service;
//         service.ip = MDNS.IP(i);
//         service.port = MDNS.port(i);
//         gantry_services.push_back(service);

//         Serial.println("Service found with specific TXT value");
//         Serial.print("Hostname: ");
//         Serial.println(MDNS.hostname(i));
//         Serial.print("IP Address: ");
//         Serial.println(service.ip.toString());
//         Serial.print("Port: ");
//         Serial.println(service.port);
//       }
//     }
//   }

//   delay(1000);  // Wait 1 seconds before next query
// }

#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();  // Create instance

void setup()
{
  tft.init();  // Initialize the display
               //   Rotate 180 degrees
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);  // Fill screen with black

  //   tft.setTextSize(2);           // Set text size. 1 is default.

  delay(1000);
}

void loop()
{
  tft.fillScreen(TFT_WHITE);    // Fill screen with black
  tft.setTextColor(TFT_BLACK);  // Set color of the text

  tft.drawString("Hello World", 0, 200,
                 2);  // Draw string at x=0, y=0 with font size 2

  delay(1000);
}
