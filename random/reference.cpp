#include <Arduino.h>

// Include the necessary libraries
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// Replace with your network credentials
const char *ssid = "SSID-A7F77D";
const char *password = "gSmyzdmX";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start server and define routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(
        200, "text/plain",
        "IMU Data Here"); // Replace "IMU Data Here" with actual IMU data
  });

  // Start server
  server.begin();
}

void loop() {
  // Your loop code here to update IMU data
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  delay(1000);
}

/**
 * @file display.ino
 * @author SeanKwok (shaoxiang@m5stack.com)
 * @brief M5StickCPlus2 Display Test
 * @version 0.1
 * @date 2023-12-09
 *
 *
 * @Hardwares: M5StickCPlus2
 * @Platform Version: Arduino M5Stack Board Manager v2.0.9
 * @Dependent Library:
 * M5GFX: https://github.com/m5stack/M5GFX
 * M5Unified: https://github.com/m5stack/M5Unified
 * M5StickCPlus2: https://github.com/m5stack/M5StickCPlus2
 */

#include "M5StickCPlus2.h"

void draw_function(LovyanGFX *gfx) {
  int x = rand() % gfx->width();
  int y = rand() % gfx->height();
  int r = (gfx->width() >> 4) + 2;
  uint16_t c = rand();
  gfx->fillRect(x - r, y - r, r * 2, r * 2, c);
}

void setup() {
  auto cfg = M5.config();
  StickCP2.begin(cfg);
  int textsize = StickCP2.Display.height() / 60;
  if (textsize == 0) {
    textsize = 1;
  }
  StickCP2.Display.setTextSize(textsize);
}

void loop() {
  int x = rand() % StickCP2.Display.width();
  int y = rand() % StickCP2.Display.height();
  int r = (StickCP2.Display.width() >> 4) + 2;
  uint16_t c = rand();
  StickCP2.Display.fillCircle(x, y, r, c);
  draw_function(&StickCP2.Display);
}

/**
 * @file imu.ino
 * @author SeanKwok (shaoxiang@m5stack.com)
 * @brief M5StickCPlus2 get IMU data
 * @version 0.1
 * @date 2023-12-19
 *
 *
 * @Hardwares: M5StickCPlus2
 * @Platform Version: Arduino M5Stack Board Manager v2.0.9
 * @Dependent Library:
 * M5GFX: https://github.com/m5stack/M5GFX
 * M5Unified: https://github.com/m5stack/M5Unified
 * M5StickCPlus2: https://github.com/m5stack/M5StickCPlus2
 */

#include "M5StickCPlus2.h"
#include "MahonyAHRS_Nathan.h"

int update_count = 0;
int last_disp_time = 0;
char buffer[100];
int last_update = 0;

int loop_rate = 250;
float dt = 1.0 / loop_rate;
int display_rate = 10;

long loop_start;

Mahony ahrs_filter;

Quaternion orientation;

float vx = 0, vy = 0, vz = 0;
float px = 0, py = 0, pz = 0;

void setup() {
  auto cfg = M5.config();
  StickCP2.begin(cfg);
  StickCP2.Display.setRotation(1);
  StickCP2.Display.setTextColor(GREEN);
  StickCP2.Display.setTextDatum(top_left);
  StickCP2.Display.setFont(&fonts::FreeSansBold9pt7b);
  StickCP2.Display.setTextSize(1);

  ahrs_filter.begin(loop_rate, 1.0);

  Serial.begin(1000000);

  loop_start = micros();
}

void loop(void) {
  auto imu_update = StickCP2.Imu.update();
  if (imu_update) {
    last_update = micros();
    auto data = StickCP2.Imu.getImuData(); // about 1ms

    ahrs_filter.updateIMU(data.gyro.x, data.gyro.y, data.gyro.z, data.accel.x,
                          data.accel.y, data.accel.z);
    orientation.w = ahrs_filter.q0;
    orientation.x = ahrs_filter.q1;
    orientation.y = ahrs_filter.q2;
    orientation.z = ahrs_filter.q3;

    Quaternion z_body_frame(0, 0, 0, 1);
    Quaternion z_world_frame =
        orientation * z_body_frame * orientation.inverse();

    Vector3f accel_body_frame{data.accel.x, data.accel.y, data.accel.z};
    Vector3f accel_world_frame = orientation.rotate(accel_body_frame);
    accel_world_frame.z -= 1.0;

    Serial.printf("wxyz: %f %f %f %f\n", orientation.w, orientation.x,
                  orientation.y, orientation.z);
    Serial.printf("z_body_in_world: %f %f %f\n", z_world_frame.x,
                  z_world_frame.y, z_world_frame.z);
    Serial.printf("body_accel_world_frame: %f %f  %f\n", accel_world_frame.x,
                  accel_world_frame.y, accel_world_frame.z);

    if (update_count % (loop_rate / display_rate) == 0) {
      int start = millis();
      // Takes 2-3 ms
      // sprintf(buffer, "C: %d I: %d P: %d D: %d", update_count, imu_time,
      // print_time, last_disp_time);
      sprintf(buffer, "Rate: %0.2fHz",
              (float)update_count * 1000000.0 / (micros() - loop_start));
      StickCP2.Display.fillRect(10, 40, 160, 15,
                                TFT_BLUE); // Adjust the size according to your
      StickCP2.Display.drawString(buffer, 10, 40);
      int end = millis();
      last_disp_time = end - start;
    }
    update_count++;
  }
  delayMicroseconds(50);
}
