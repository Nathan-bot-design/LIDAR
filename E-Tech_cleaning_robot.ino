#include <E_Tech mobileplatform.h.h>
#include <E_Tech _cleaning _robot_pins.h.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <NTPClient.h>

E_Tech_mobileplatform platform;
MPU6050 mpu;
TwoWire Wire2 = TwoWire(1);  // Create a second TwoWire instance
Adafruit_SSD1306 display(128, 64, &Wire2);

int16_t ax, ay, az;
int16_t gx, gy, gz;
float angleZ = 0;  // Yaw angle
float yawOffset = 0;
unsigned long previousMillis = 0;
const float threshold = 0.5;       // Threshold to ignore small gyro changes
const float dt = 0.01;             // Time step in seconds
const int distanceThreshold = 30;  // Distance threshold in cm

int cursor_y = 20;  // Start at the first option

// Replace with your network credentials
//const char* ssid = "Engineer";
//const char* password = "engineer123";

// Define NTP Client to get time
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "pool.ntp.org");
enum State {
  MOVE_FORWARD,
  TURNING,
  MOVE_FORWARD1,
  TURNING1,
  MOVE_FORWARD2,
  TURNING2,
  MOVE_FORWARD3,
  TURNING3,
};

State currentState = MOVE_FORWARD;
float targetAngle = 0;  // Target angle for turning

void setup() {
  Serial.begin(115200);
  platform.setup();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("E-Tech"));
  display.println(F("Cleaning"));
  display.println(F("Robot"));
  display.display();
  delay(1000);
  // Connect to Wi-Fi
/*  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Connecting to WiFi...");
    display.display();
  }
  Serial.println("Connected to WiFi");
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Connected to WiFi");
  display.display();

  // Initialize NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(10800);  // Set your timezone offset in seconds (UTC+3 for Nairobi)
*/

  // Setup complete
  Serial.println("Setup complete");
  // Calibrate MPU6050 to get the yaw offset
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Setup complete");
  display.display();
  calibrateMPU();



  // Initially turn off both relays
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("MPU6050 connection failed!");
    display.display();
    while (1)
      ;
  }
  Serial.println("MPU6050 connected!");
      display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("MPU6050 connected!");
    display.display();
}

void loop() {
  // Update time from NTP
  /* timeClient.update();

  // Get current time
  String currentTime = timeClient.getFormattedTime();
  String amPmTime = formatTimeForDisplay(currentTime);
  Serial.print("Current Time: ");
  Serial.println(amPmTime); // Debugging
  
  // Get battery voltage and percentage
  int batteryRaw = analogRead(BATTERY_PIN);
  float voltage = (batteryRaw / 4095.0) * 3.3 * (MAX_VOLTAGE / MIN_VOLTAGE);
  int batteryPercentage = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100;
  if (batteryPercentage > 100) batteryPercentage = 100;
  if (batteryPercentage < 0) batteryPercentage = 0;

  // Clear the display
  display.clearDisplay();
  // Draw current time at the top middle of the screen
  display.setTextSize(0); // Shrink text size to half
  display.setTextColor(WHITE);

  // Calculate text width and height
  int textWidth = amPmTime.length() * 6; // Approximate width for text size 0
  int textHeight = 8; // Text size 0 is approximately 8 pixels high
  
  // Set cursor to the top middle of the screen
  int x = 0;
  int y = 0; // Position slightly below the top edge

  display.setCursor(x, y);
  display.print(amPmTime);

  // Draw battery icon in the top right corner
  drawBattery(batteryPercentage);
  
  // Show the display
  display.display();

  delay(1000); // Update every second*/
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  unsigned long currentMillis = millis();

  // Update every 10 milliseconds (0.01 seconds)
  if (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;

    // Read raw gyro values
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw values to yaw angle
    float gyroZ = gz / 43.67;  // Convert to degrees/sec

    // Only update angleZ if gyroZ exceeds the threshold
    if (abs(gyroZ) > threshold) {
      angleZ += (gyroZ - yawOffset) * dt;  // Integrate to get angle over time
    }

    // Normalize angle to 0-360 degrees
    angleZ = normalizeAngle(angleZ);

    // State machine
    switch (currentState) {
      case MOVE_FORWARD:
        moveForward();
        Serial.print("Distance :");
        Serial.println(getDistance());
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 5);
        display.print("DISTANCE");
        display.setCursor(0, 35);
        display.print((getDistance()));
        display.display();
        if (getDistance() < 35) {
          stopMotors();
          targetAngle = normalizeAngle(angleZ + 80);
          currentState = TURNING;
        }
        break;

      case TURNING:
        if (angleZ < targetAngle - 1 || angleZ > targetAngle + 1) {

          // Print the yaw angle
          Serial.print("Turning Angle: ");
          Serial.println(angleZ);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0, 5);
          display.print("ANGLE");
          display.setCursor(0, 35);
          display.print(angleZ);
          display.display();
          turnRight();
        } else {
          stopMotors();
          currentState = MOVE_FORWARD1;
        }
        break;

      case MOVE_FORWARD1:
        moveForward();
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("FORWARD 1");
        delay(1000);
        targetAngle = normalizeAngle(angleZ + 80);
        currentState = TURNING1;
        break;

      case TURNING1:
        if (angleZ < targetAngle - 1 || angleZ > targetAngle + 1) {
          // Print the yaw angle
          Serial.print("Turning Angle: ");
          Serial.println(angleZ);
          Serial.print("Turning Angle: ");
          Serial.println(angleZ);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0, 5);
          display.print("ANGLE");
          display.setCursor(0, 35);
          display.print(angleZ);
          display.display();
          turnRight();
        } else {
          stopMotors();
          currentState = MOVE_FORWARD2;
        }
        break;

      case MOVE_FORWARD2:
        moveForward();
        Serial.print("Distance :");
        Serial.println(getDistance());
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 5);
        display.print("DISTANCE");
        display.setCursor(0, 35);
        display.print((getDistance()));
        display.display();
        if (getDistance() < 35) {
          stopMotors();
          targetAngle = normalizeAngle(angleZ - 90);
          currentState = TURNING2;
        }
        break;

      case TURNING2:
        if (angleZ > targetAngle + 1 || angleZ < targetAngle - 1) {
          // Print the yaw angle
          Serial.print("Turning Angle: ");
          Serial.println(angleZ);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0, 5);
          display.print("ANGLE");
          display.setCursor(0, 35);
          display.print(angleZ);
          display.display();
          turnLeft();
        } else {
          stopMotors();
          currentState = MOVE_FORWARD3;
        }
        break;

      case MOVE_FORWARD3:
        moveForward();
        delay(1000);
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("FORWARD 2");
        targetAngle = normalizeAngle(angleZ - 90);
        currentState = TURNING3;
        break;

      case TURNING3:
        if (angleZ > targetAngle + 1 || angleZ < targetAngle - 1) {
          // Print the yaw angle
          Serial.print("Turning Angle: ");
          Serial.println(angleZ);
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0, 5);
          display.print("ANGLE");
          display.setCursor(0, 35);
          display.print(angleZ);
          display.display();
          turnLeft();
        } else {
          stopMotors();
          currentState = MOVE_FORWARD;
        }
        break;
    }
  }
}

float normalizeAngle(float angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

void calibrateMPU() {
  const int numReadings = 1000;
  long total = 0;
  Serial.println("Calibrating MPU6050...");
  for (int i = 0; i < numReadings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    total += gz;
    delay(2);
  }
  yawOffset = (float)total / numReadings / 43.67;
  Serial.print("Yaw Offset: ");
  Serial.println(yawOffset);
  delay(1000);
}
/*void drawBattery(int percentage) {
  // Draw the battery icon smaller in the top right corner
  int batteryWidth = 16; // Reduced width
  int batteryHeight = 5; // Reduced height
  
  display.drawRect(SCREEN_WIDTH - batteryWidth - 4, 2, batteryWidth, batteryHeight, WHITE); // Battery outer rectangle
  display.drawRect(SCREEN_WIDTH - 4, 6, 2, 4, WHITE); // Battery terminal
  
  // Map the percentage to the width of the battery fill
  int width = map(percentage, 0, 100, 0, batteryWidth - 2);
  display.fillRect(SCREEN_WIDTH - batteryWidth - 3, 3, width, batteryHeight - 2, WHITE); // Fill battery
}*/

/*String formatTimeForDisplay(String time) {
  int hours = time.substring(0, 2).toInt();
  int minutes = time.substring(3, 5).toInt();
  String amPm = "AM";
  
  if (hours >= 12) {
    amPm = "PM";
    if (hours > 12) hours -= 12;
  } else if (hours == 0) {
    hours = 12;
  }
  
  // Format time string
  String formattedTime = String(hours) + ":" + (minutes < 10 ? "0" : "") + String(minutes) + " " + amPm;
  return formattedTime;
}*/
