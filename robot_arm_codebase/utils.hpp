#include <LittleFS.h>
#include <Vector.h>

#define BOOT_BTN 0
#define LOWER_JOINT_LIMIT_SWITCH 1
#define UPPER_JOINT_LIMIT_SWITCH 2
// #define BASE_LIMIT_SWITCH 3

#define usrKey_up digitalRead(BOOT_BTN)    // --> key 1
#define usrKey_down !digitalRead(BOOT_BTN) // --> key 0

bool DEBUG = false;

long updteInterval = 0.2;

template <typename T>
void syslog(String varName, T const &variable, bool space = true, bool newLine = false)
{
  if (!DEBUG)
    return;
  String output = String(varName) + ":" + String(variable);
  if (space)
    output += " ";
  if (newLine)
    output += "\r\n";
  Serial.print(output);
}

void syslog(String stringPrint, bool newLine = true)
{
  if (!DEBUG)
    return;
  Serial.print(stringPrint);
  if (newLine)
    Serial.println();
}

void syserr(String errorMessage, bool block = true)
{
  Serial.println(errorMessage);
  digitalWrite(RGB_BUILTIN, LOW);
  delay(1);
  rgbLedWrite(RGB_BUILTIN, 255, 0, 0);
  while (1)
  {
    for (uint8_t i = 0; i < 5; i++)
    {
      rgbLedWrite(RGB_BUILTIN, 255, 0, 0);
      delay(100);
      digitalWrite(RGB_BUILTIN, LOW);
      delay(100);
    }
    if (!block)
      break;
    delay(1000);
  }
}

class Stats
{
private:
  unsigned long statBegin;
  unsigned long lastMeasurement;

public:
  Stats()
  {
    statBegin = millis();
    lastMeasurement = 0;
  }

  void begin()
  {
    statBegin = millis();
  }

  unsigned long report(bool loop = true)
  {
    lastMeasurement = millis() - statBegin;
    syslog("Elapsed", lastMeasurement, true, true);
    if (loop)
      begin();
    return lastMeasurement;
  }
};

void initComms()
{
  pinMode(BOOT_BTN, INPUT_PULLUP);
  digitalWrite(RGB_BUILTIN, LOW);

  pinMode(LOWER_JOINT_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(UPPER_JOINT_LIMIT_SWITCH, INPUT_PULLUP);

  Serial.begin(115200);

  syslog(__FUNCTION__);
  delay(50);

  if (DEBUG)
    delay(2000);
}

#define upperJointAtLimit !digitalRead(UPPER_JOINT_LIMIT_SWITCH)
#define lowerJointAtLimit !digitalRead(LOWER_JOINT_LIMIT_SWITCH)
// #define baseAtLimit !digitalRead(BASE_LIMIT_SWITCH)

void blink(uint8_t r = 255, uint8_t g = 255, uint8_t b = 255)
{
  digitalWrite(RGB_BUILTIN, LOW);
  delay(1);
  rgbLedWrite(RGB_BUILTIN, r, g, b);
  delay(100);
  digitalWrite(RGB_BUILTIN, LOW);
}

String resetReasonName(esp_reset_reason_t r)
{
  switch (r)
  {
  case ESP_RST_UNKNOWN:
    return "Unknown";
  case ESP_RST_POWERON:
    return "PowerOn"; // Power on or RST pin toggled
  case ESP_RST_EXT:
    return "ExtPin"; // External pin - not applicable for ESP32
  case ESP_RST_SW:
    return "Reboot"; // esp_restart()
  case ESP_RST_PANIC:
    return "Crash"; // Exception/panic
  case ESP_RST_INT_WDT:
    return "WDT_Int"; // Interrupt watchdog (software or hardware)
  case ESP_RST_TASK_WDT:
    return "WDT_Task"; // Task watchdog
  case ESP_RST_WDT:
    return "WDT_Other"; // Other watchdog
  case ESP_RST_DEEPSLEEP:
    return "Sleep"; // Reset after exiting deep sleep mode
  case ESP_RST_BROWNOUT:
    return "BrownOut"; // Brownout reset (software or hardware)
  case ESP_RST_SDIO:
    return "SDIO"; // Reset over SDIO
  default:
    return "";
  }
}

void listFiles()
{
  File root = LittleFS.open("/");
  File file = root.openNextFile();

  Serial.println("Listing files in LittleFS:");

  while (file)
  {
    Serial.print("FILE: ");
    Serial.print(file.name());
    Serial.print("\tSIZE: ");
    Serial.println(file.size());
    file = root.openNextFile(); // Move to next file
  }
}

void initLittleFS()
{
  if (!LittleFS.begin())
    syserr("An Error has occurred while mounting LittleFS");
}

Stats globalStats;

void NOT_IMPLEMENTED();
void checkSerial();
bool isHomed = false;
