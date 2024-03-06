#include "M5StickCPlus2.h"
#include "MahonyAHRS_Nathan.h"
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <algorithm>
#include <array>

#include <sstream>
#include <string>

// Replace with your network credentials
const char *ssid = "SSID-A7F77D";
const char *password = "gSmyzdmX";

constexpr int loop_rate = 250;
constexpr int max_records = loop_rate * 1;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Structure to store IMU data
struct IMUData {
  Quaternion orientation;
  Vector3f world_accel;
  Vector3f angular_velocity;
};

// Vector to store IMU records
std::array<IMUData, max_records> imu_data_records;
int write_idx = 0;

Mahony ahrs_filter;

String imu_data_to_string(const IMUData &data) {
  String str;
  str += String(data.orientation.w) + ", " + String(data.orientation.x) + ", " +
         String(data.orientation.y) + ", " + String(data.orientation.z) + ", " +
         String(data.world_accel.x) + ", " + String(data.world_accel.y) + ", " +
         String(data.world_accel.z) + ", " + String(data.angular_velocity.x) +
         ", " + String(data.angular_velocity.y) + ", " +
         String(data.angular_velocity.z);
  return str;
}

void setup() {
  // Initialize M5StickCPlus2
  auto cfg = M5.config();
  StickCP2.begin(cfg);
  StickCP2.Display.setRotation(1);
  StickCP2.Display.setTextColor(GREEN);
  StickCP2.Display.setTextDatum(top_left);
  StickCP2.Display.setFont(&fonts::FreeSansBold9pt7b);
  StickCP2.Display.setTextSize(1);

  // Initialize Serial Monitor
  Serial.begin(1000000);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  String ip = WiFi.localIP().toString();
  StickCP2.Display.drawString(ip, 10, 40);

  // IMU initialization
  StickCP2.Imu.begin();
  ahrs_filter.begin(loop_rate, 1.0);

  // Define web server route
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String data_string = "qw, qx, qy, qz, ax, ay, az, wx, wy, wz\n";
    for (int i = 0; i < max_records; i++) {
      int offset_idx = (i + write_idx) % max_records;
      const auto &data = imu_data_records[offset_idx];
      data_string += String(i) + ", " + imu_data_to_string(data) + "\n";
    }
    request->send(200, "text/plain", data_string);
  });

  // Start server
  server.begin();
}

void loop() {
  auto imu_update = StickCP2.Imu.update();
  if (imu_update) {
    auto data = StickCP2.Imu.getImuData();

    IMUData imu_data;
    imu_data.orientation = Quaternion(ahrs_filter.q0, ahrs_filter.q1,
                                      ahrs_filter.q2, ahrs_filter.q3);
    imu_data.world_accel = imu_data.orientation.rotate(
        Vector3f(data.accel.x, data.accel.y, data.accel.z));
    imu_data.angular_velocity = Vector3f(data.gyro.x, data.gyro.y, data.gyro.z);

    // Here you might want to limit the size of `imu_data_records` to avoid
    // running out of memory
    if (write_idx == max_records) {
      write_idx = 0;
    }
    imu_data_records.at(write_idx) = imu_data;
    write_idx++;

    Serial.printf("wxyz: %f %f %f %f ", imu_data.orientation.w,
                  imu_data.orientation.x, imu_data.orientation.y,
                  imu_data.orientation.z);
    Serial.printf("body_accel_world_frame: %f %f  %f\n", imu_data.world_accel.x,
                  imu_data.world_accel.y, imu_data.world_accel.z);
  }

  delayMicroseconds(50); // Adjust delay to control data collection rate
}
