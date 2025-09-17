#include "config.h"
#include "Automata.h"
#include "ArduinoJson.h"
#include <Adafruit_INA219.h>
#include <WiFiUdp.h>
#include <Adafruit_AHTX0.h>
#define I2C_SDA_PIN D9
#define I2C_SCL_PIN D10
// const char* HOST = "192.168.29.67";
// int PORT = 8080;

const char *HOST = "raspberry.local";
int PORT = 8010;

Preferences preferences;
Automata automata("Battery Bkp", HOST, PORT);
JsonDocument doc;
Adafruit_AHTX0 aht;
Adafruit_INA219 ina219_a(0x40);

long start = millis();
float c1_shunt = 0;

float c1_volt = 0;

float c1_curr = 0;

float c1_pow = 0;

float targetCapacity = 15;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
float totalEnergy = 0;
float percent = 0;
float capacity_mAh = 0;
float dischargingTimeHours = 0;
int nextPowerReadTime = 0;
bool onOff = true;

// Pin definitions for ADC inputs
const int adcPin1 = D1;
const int adcPin2 = D2;
const int adcPin3 = D3;
const int adcPin4 = D4;
// Reference voltage for ESP32 ADC (default is 3.3V)
const float referenceVoltage = 3.3;
unsigned long lastUpdateTime = 0;
// Voltage divider ratios based on resistor values
const float dividerRatio1 = 2.0;  // 10kΩ / 10kΩ
const float dividerRatio2 = 11.0; // 100kΩ / 10kΩ
const float dividerRatio3 = 20.6; // 100kΩ / 5.1kΩ
const float dividerRatio4 = 51.0; // 100kΩ / 2kΩ
float actualCell1;
float actualCell2;
float actualCell3;
float actualCell4;
// Calibration factors (adjust these based on multimeter readings)
float calibrationFactor1 = 1.0614;
float calibrationFactor2 = 1.1798;
float calibrationFactor3 = 1.1232;
float calibrationFactor4 = 1.4077;

String isDischarge = "DISCHARGE";

void action(const Action action)
{

  if (action.data.containsKey("reset"))
  {
    // reset = !reset;
    percent = 100.0;
    preferences.putFloat("percent", percent);

    preferences.putFloat("totalEnergy", 0);
    preferences.putFloat("capacity_mAh", 0);
    // startTime = now;
    // startTimeStr = getTimeStr();
    totalEnergy = 0;
    capacity_mAh = 0;
  }

  String jsonString;
  serializeJson(action.data, jsonString);
  Serial.println(jsonString);
}
void getData()
{
  percent = preferences.getFloat("percent", 100.0); // default to full charge

  totalEnergy = preferences.getFloat("totalEnergy", 0);
  capacity_mAh = preferences.getFloat("capacity_mAh", 0);
}
void saveData()
{
  preferences.putFloat("percent", percent);

  preferences.putFloat("totalEnergy", totalEnergy);
  preferences.putFloat("capacity_mAh", capacity_mAh);
}
void sendData()
{
  saveData();
  automata.sendData(doc);
}

void setup()
{
  Serial.begin(115200);
  // pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("waiting");
  delay(1000);
  if (!ina219_a.begin())
  {
    Serial.println("Failed to find INA219_B chip");
  }
  preferences.begin("dummy", false);
  if (aht.begin())
  {
    Serial.println("Found AHT20");
  }
  automata.begin();
  getData();

  // automata.addAttribute("C1", "V1", "V", "DATA|AUX");
  // automata.addAttribute("C2", "V2", "V", "DATA|AUX");
  // automata.addAttribute("C3", "V3", "V", "DATA|AUX");
  // automata.addAttribute("C4", "V4", "V", "DATA|AUX");

  automata.addAttribute("C1_CURR", "Current", "A", "DATA|CHART");
  automata.addAttribute("C1_POWER", "Power", "W", "DATA|MAIN");

  automata.addAttribute("busVoltage", "Voltage", "V", "DATA|MAIN");
  automata.addAttribute("current", "Current", "A", "DATA|MAIN");
  automata.addAttribute("temp", "Temp", "C", "DATA|MAIN");
  automata.addAttribute("humid", "Humidity", "%", "DATA|MAIN");
  automata.addAttribute("power", "Power", "W", "DATA|CHART");
  automata.addAttribute("totalEnergy", "Energy", "Wh", "DATA|MAIN");

  automata.addAttribute("percent", "Percent", "%", "DATA|AUX");
  automata.addAttribute("capacity", "Capacity", "Ah", "DATA|MAIN");
  automata.addAttribute("reset", "Reset", "", "ACTION|MENU|BTN");
  automata.addAttribute("dischargingTime", "Time Left", "Hr", "DATA|MAIN");
  // automata.addAttribute("onOff", "OnOff", "", "ACTION|SWITCH");

  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);

  // brightness = preferences.getInt("bright", 2);
  // presets = preferences.getInt("presets", 1);
  // onOff = preferences.getBool("onOff", true);
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readPow()
{
  // Read INA219 data
  c1_shunt = ina219_a.getShuntVoltage_mV();
  c1_volt = ina219_a.getBusVoltage_V();
  c1_curr = ina219_a.getCurrent_mA() * 20.0;
  c1_curr = c1_curr / 1000; // Scaled and converted to A
  c1_pow = c1_volt * c1_curr;

  // Assign readings to global variables
  busvoltage = c1_volt;
  shuntvoltage = c1_shunt;
  current_mA = c1_curr;
  power_mW = busvoltage * current_mA;
  loadvoltage = busvoltage + (shuntvoltage / 1000.0);

  // Time tracking
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  float timeInterval = (currentMillis - previousMillis) / 1000.0;

  totalEnergy += power_mW * (timeInterval / (60 * 60));
  capacity_mAh += current_mA * (timeInterval / (60 * 60));
  isDischarge = current_mA < 0 ? "DISCHARGING" : "CHARGING";
  percent = mapf(busvoltage, 12.8, 16.6, 0.0, 100.0);

  if (isDischarge == "DISCHARGING")
  {
    dischargingTimeHours = (targetCapacity - abs(capacity_mAh)) / abs(current_mA);
  }
  if (percent > 100)
  {
    percent = 100;
  }
  if (percent < 0)
    percent = 0;
  previousMillis = currentMillis;
}

float readVoltage(int pin, float ratio)
{
  int rawADC = analogRead(pin);
  float voltage = (rawADC / 4095.0) * referenceVoltage;
  return voltage * ratio;
}

void readCell()
{
  float cell1Voltage = readVoltage(adcPin1, dividerRatio1);
  float cell2Voltage = readVoltage(adcPin2, dividerRatio2);
  float cell3Voltage = readVoltage(adcPin3, dividerRatio3);
  float cell4Voltage = readVoltage(adcPin4, dividerRatio4);

  // Calculate actual cell voltages
  actualCell1 = cell1Voltage;
  actualCell2 = cell2Voltage - cell1Voltage;
  actualCell3 = cell3Voltage - cell2Voltage;
  actualCell4 = cell4Voltage - cell3Voltage;

  // Print the voltages to Serial Monitor
  Serial.print("Cell 1 Voltage: ");
  Serial.print(actualCell1);
  Serial.println(" V");
  Serial.print("Cell 2 Voltage: ");
  Serial.print(actualCell2);
  Serial.println(" V");
  Serial.print("Cell 3 Voltage: ");
  Serial.print(actualCell3);
  Serial.println(" V");
  Serial.print("Cell 4 Voltage: ");
  Serial.print(actualCell4);
  Serial.println(" V");
}
void loop()
{
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  doc["temp"] = String(temp.temperature, 2);
  doc["humid"] = String(humidity.relative_humidity, 2);
  readPow();
  // readCell();
  // doc["C1"] = String(actualCell1 * calibrationFactor1, 2);
  // doc["C2"] = String(actualCell2 * calibrationFactor2, 2);
  // doc["C3"] = String(actualCell3 * calibrationFactor3, 2);
  // doc["C4"] = String(actualCell4 * calibrationFactor4, 2);
  doc["C1_CURR"] = String(c1_curr, 2);
  doc["C1_POWER"] = String(c1_pow, 2);
  // doc["shuntVoltage"] = String(shuntvoltage, 3);
  doc["busVoltage"] = String(busvoltage, 2);
  doc["current"] = String(current_mA, 2);
  doc["power"] = String(power_mW, 2);
  doc["totalEnergy"] = String(totalEnergy, 2);
  // doc["loadVoltage"] = String(loadvoltage, 3);
  doc["percent"] = String(percent, 2);
  doc["capacity"] = String(capacity_mAh, 2);
  doc["dischargingTime"] = String(dischargingTimeHours, 2);

  if ((millis() - start) > 800)
  {

    // digitalWrite(LED_BUILTIN, LOW);
    automata.sendLive(doc);
    delay(50);
    start = millis();
  }

  // digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}
