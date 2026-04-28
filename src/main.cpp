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
#define RELAY_PIN D4

const char *HOST = "raspberry.local";
int PORT = 8010;

Preferences preferences;
Automata automata("LiFePO4 816WH", "SENSOR|BATTERY", HOST, PORT, HOST, 1883);
JsonDocument doc;
Adafruit_AHTX0 aht;
Adafruit_INA219 ina219_a(0x40);
Adafruit_INA219 ina219_b(0x41);

long start = millis();
float c1_shunt = 0;
float c2_shunt = 0;
float c1_volt = 0;
float c2_volt = 0;
float c1_curr = 0;
float c2_curr = 0;
float c1_pow = 0;
float c2_pow = 0;

float targetCapacity = 30;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
float totalEnergy = 0;
float percent = 0;
float capacity_mAh = 0;
float chargingTimeHours = 0;
float dischargingTimeHours = 0;
int nextPowerReadTime = 0;
bool onOff = true;
bool channel1 = false;

const float MAX_ENERGY_Wh = 816.0;
String isDischarge = "DISCHARGE";

void action(const Action action)
{
  if (action.data.containsKey("channel1"))
  {
    channel1 = action.data["channel1"];
    preferences.putBool("channel1", channel1);
  }
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
  // channel1 = preferences.getBool("channel1", false);
  totalEnergy = preferences.getFloat("totalEnergy", 0);
  capacity_mAh = preferences.getFloat("capacity_mAh", 0);
}
void saveData()
{
  preferences.putFloat("percent", percent);
  // preferences.putBool("channel1", channel1);
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
  pinMode(RELAY_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("waiting");
  delay(1000);
  if (!ina219_b.begin())
  {
    Serial.println("Failed to find INA219_B chip");
  }
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
  automata.useMQTT();
  getData();

  // automata.addAttribute("C1", "V1", "V", "DATA|AUX");
  // automata.addAttribute("C2", "V2", "V", "DATA|AUX");
  // automata.addAttribute("C3", "V3", "V", "DATA|AUX");
  // automata.addAttribute("C4", "V4", "V", "DATA|AUX");
  // automata.addAttribute("channel1", "Port 1", "On/Off", "ACTION|SWITCH");
  automata.addAttribute("channel1", "Port 1", "On/Off", "ACTION|MENU|SWITCH");
  automata.addAttribute("C1_CURR", "C1", "A", "DATA|CHART");
  automata.addAttribute("C1_POWER", "P1", "W", "DATA|MAIN");
  automata.addAttribute("C2_POWER", "P2", "W", "DATA|MAIN");
  automata.addAttribute("C2_CURR", "C2", "A", "DATA|MAIN");
  automata.addAttribute("busVoltage", "Voltage", "V", "DATA|MAIN");
  automata.addAttribute("current", "Current", "A", "DATA|MAIN");
  automata.addAttribute("temp", "Temp", "C", "DATA|MAIN");
  automata.addAttribute("humid", "Humidity", "%", "DATA|MAIN");
  automata.addAttribute("power", "Power", "W", "DATA|CHART");
  automata.addAttribute("totalEnergy", "Energy", "Wh", "DATA|MAIN");
  automata.addAttribute("status", "Status", "", "DATA|MAIN");
  automata.addAttribute("percent", "Percent", "%", "DATA|MAIN");
  automata.addAttribute("capacity", "Capacity", "Ah", "DATA|MAIN");
  automata.addAttribute("reset", "Reset", "", "ACTION|MENU|BTN");
  automata.addAttribute("dischargingTime", "Runtime", "Hr", "DATA|MAIN");
  // automata.addAttribute("onOff", "OnOff", "", "ACTION|SWITCH");
  automata.addAttribute("chargingTime", "Charge ETA", "Hr", "DATA|MAIN");
  automata.addAttribute("capacityInfo", "Capacity", "30 AH", "DATA|INFO");
  automata.addAttribute("configInfo", "Config", "8s2p", "DATA|INFO");
  automata.addAttribute("energyInfo", "Energy", "816 WH", "DATA|INFO");
  automata.addAttribute("relayStatus", "Relay", "", "DATA|MAIN");
  automata.addAttribute("sensorInfo", "Sensors", "Current, Volt, Temp, Humidity", "DATA|INFO");

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

  c2_shunt = ina219_b.getShuntVoltage_mV();
  c2_volt = ina219_b.getBusVoltage_V();
  c2_curr = ina219_b.getCurrent_mA() * 60;

  c1_curr = c1_curr / 1000; // Scaled and converted to A
  c1_pow = c1_volt * c1_curr;

  c2_curr = c2_curr / 1000;
  c2_pow = c2_volt * c2_curr;

  // Assign readings to global variables
  busvoltage = (c1_volt + c2_volt) / 2;
  shuntvoltage = (c1_shunt + c2_shunt) / 2;
  float curr = 0;
  if (c1_curr > c2_curr)
    curr = c1_curr + c2_curr;
  else
    curr = c2_curr + c1_curr;

  current_mA = curr;

  power_mW = busvoltage * current_mA;
  loadvoltage = busvoltage + (shuntvoltage / 1000.0);

  // Time tracking
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  float timeInterval = (currentMillis - previousMillis) / 1000.0;

  totalEnergy += -power_mW * (timeInterval / (60 * 60));
  capacity_mAh += current_mA * (timeInterval / (60 * 60));
  isDischarge = c1_curr > 0.5 ? "CHARGING" : "DISCHARGE";
  // percent = mapf(busvoltage, 12.8, 16.6, 0.0, 100.0);
  percent = 100.0 * (1.0 - (totalEnergy / MAX_ENERGY_Wh));
  if (isDischarge == "CHARGING" && c1_curr > 0.5)
  {
    float remainingCapacity = targetCapacity - capacity_mAh;

    if (remainingCapacity > 0)
      chargingTimeHours = remainingCapacity / current_mA;
    else
      chargingTimeHours = 0;
  }
  else
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

void loop()
{
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  doc["temp"] = String(temp.temperature, 2);
  doc["humid"] = String(humidity.relative_humidity, 2);
  readPow();
  // readCell();
  int etaHours = (int)chargingTimeHours;
  int etaMinutes = (int)((chargingTimeHours - etaHours) * 60);
  if (chargingTimeHours < 0 || chargingTimeHours > 1000)
  {
    etaHours = 0;
    etaMinutes = 0;
  }
  char etaStr[6];
  snprintf(etaStr, sizeof(etaStr), "%02d:%02d", etaHours, etaMinutes);
  doc["channel1"] = channel1;
  // doc["C1"] = String(actualCell1 * calibrationFactor1, 2);
  // doc["C2"] = String(actualCell2 * calibrationFactor2, 2);
  // doc["C3"] = String(actualCell3 * calibrationFactor3, 2);
  // doc["C4"] = String(actualCell4 * calibrationFactor4, 2);
  doc["C1_CURR"] = String(c1_curr, 2);
  doc["C1_POWER"] = String(c1_pow, 2);
  doc["C2_CURR"] = String(c2_curr, 2);
  doc["C2_POWER"] = String(c2_pow, 2);
  doc["chargingTime"] = String(etaStr);
  // doc["shuntVoltage"] = String(shuntvoltage, 3);
  doc["busVoltage"] = String(busvoltage, 2);
  doc["current"] = String(current_mA, 2);
  doc["power"] = String(power_mW, 2);
  doc["totalEnergy"] = String(totalEnergy, 2);
  // doc["loadVoltage"] = String(loadvoltage, 3);
  doc["percent"] = String(percent, 2);
  doc["capacity"] = String(capacity_mAh, 2);
  doc["dischargingTime"] = String(dischargingTimeHours, 2);
  doc["status"] = isDischarge;
  doc["relayStatus"] = digitalRead(RELAY_PIN);

  if (isDischarge == "CHARGING")
    channel1 = true;

  if (channel1)
  {
    digitalWrite(RELAY_PIN, HIGH);
  }
  else
    digitalWrite(RELAY_PIN, LOW);

  if ((millis() - start) > 2000)
  {

    // digitalWrite(LED_BUILTIN, LOW);
    automata.sendLive(doc);
    delay(50);
    start = millis();
  }

  // digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}
