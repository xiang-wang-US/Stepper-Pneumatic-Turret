// ***** Necessary Libraries *****
#include "esp_camera.h"
#include "board_config.h" 
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AccelStepper.h>

// ***** Configuration *****
// WiFi Credentials
const char* ssid = "";
const char* password = "";

// UDP Network Settings
const unsigned int localUdpPort = 1234; // Port from Python script

// ***** Motor & Camera Configuration *****
const int STEPS_PER_REVOLUTION = 200; // Steps per full 360° rotation 
const int MICROSTEPPING = 16;         // Microsteps  
const float CAMERA_FOV = 75.0;        // F0V in degrees

// ***** Calculated Global Constants *****
const float STEPS_PER_DEGREE = (STEPS_PER_REVOLUTION * MICROSTEPPING) / 360.0;

// ***** Pin Definitions for Stepper Motors *****
#define MOTOR1_STEP_PIN 35 // DOUBLE CHECK PLS
#define MOTOR1_DIR_PIN  36
#define MOTOR2_STEP_PIN 38 
#define MOTOR2_DIR_PIN  39

#define ALWAYS_ON_PIN 2

// ***** Global Objects *****
WiFiUDP udp;
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN); 
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN); 

// ***** Function Prototypes *****
void startCameraServer();
long angleToSteps(float angle);
void moveMotorBlocking(AccelStepper &motor, long steps);
float normalizedToAngle(float normalizedCoord);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode(ALWAYS_ON_PIN, OUTPUT);
  digitalWrite(ALWAYS_ON_PIN, HIGH);

  // ***** Camera Initialization *****
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA; 
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA); 

  // ***** Motor Configuration *****
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(500);

  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);

  // ***** WiFi & Network Setup *****
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  startCameraServer();
  Serial.print("Camera Stream Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect.");

  udp.begin(localUdpPort);
  Serial.printf("Listening for coordinates on port %d\n", localUdpPort);
  Serial.println("********************************************************************************");
}

void loop() {
  char incomingPacket[255];
  
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    Serial.printf("Received coordinate: '%s'\n", incomingPacket);

    char* normalizedXStr = strtok(incomingPacket, ",");
    char* normalizedYStr = strtok(NULL, ",");

    if (normalizedXStr != NULL && normalizedYStr != NULL) {
      float normalizedX = atof(normalizedXStr);
      float normalizedY = atof(normalizedYStr);

      // Convert coordinates to angles
      float angleX = normalizedToAngle(normalizedX); 
      float angleY = normalizedToAngle(normalizedY); 

      // Convert angles to steps
      long stepsX = angleToSteps(angleX);
      long stepsY = angleToSteps(angleY);

      Serial.printf("Pan: %.2f deg -> %ld steps. Tilt: %.2f deg -> %ld steps.\n", angleX, stepsX, angleY, stepsY);

      if (stepsX != 0) {
        moveMotorBlocking(motor1, stepsX);
      }
      if (stepsY != 0) {
        moveMotorBlocking(motor2, stepsY);
      }
      Serial.println("Motor movement complete!");

    } else {
      Serial.println("Messed up packet!");
    }
  }
}

/**
 * @brief Converts a normalized coordinate (-1.0 to 1.0) to an angle.
 */
float normalizedToAngle(float normalizedCoord) {
  // Invert the angle because we want to bring the coordinate back to zero.
  return -normalizedCoord * (CAMERA_FOV / 2.0);
}

/**
 * @brief Converts a desired angle into the corresponding number of motor steps.
 */
long angleToSteps(float angle) {
  return round(angle * STEPS_PER_DEGREE);
}

/**
 * @brief Moves the motor a specified number of steps and waits for it to finish.
 */
void moveMotorBlocking(AccelStepper &motor, long steps) {
  motor.moveTo(motor.currentPosition() + steps);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
}
