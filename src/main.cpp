#include <Arduino.h>
#include <Wire.h>

// pin definitions
#define MODEM_PWRKEY          4
#define MODEM_POWER_ON       25
#define MODEM_TX             27
#define MODEM_RX             26
#define MODEM_DTR            32
#define MODEM_RI             33

#define I2C_SDA              21
#define I2C_SCL              22

#define LED_GPIO             12
#define LED_ON               LOW
#define LED_OFF              HIGH

#include <axp20x.h>         // https://github.com/lewisxhe/AXP202X_Library

AXP20X_Class axp;

bool setupPMU()
{
  // For more information about the use of AXP192, please refer to AXP202X_Library https://github.com/lewisxhe/AXP202X_Library
  Wire.begin(I2C_SDA, I2C_SCL);
  if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
    Serial.println("AXP Power begin failed");
    return false;
  }

  //! Turn off unused power
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
  axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

  //! Do not turn off DC3, it is powered by esp32
  // axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON);

  // Set the charging indicator to turn off
  // Turn it off to save current consumption
  axp.setChgLEDMode(AXP20X_LED_OFF);

  // Set the charging indicator to flash once per second
  // axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);

  //! Use axp192 adc get voltage info
  axp.adc1Enable(AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);

  float vbus_v = axp.getVbusVoltage();
  float vbus_c = axp.getVbusCurrent();
  float batt_v = axp.getBattVoltage();
  // axp.getBattPercentage();   // axp192 is not support percentage
  Serial.printf("VBUS:%.2f mV %.2f mA, BATTERY: %.2f\n", vbus_v, vbus_c, batt_v);

  return true;
}

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
//#define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>

#include "driver/adc.h"
#include <esp_wifi.h>

HardwareSerial serialGsm(1);
#define SerialAT serialGsm
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI false

#ifdef DUMP_AT_COMMANDS
#include "StreamDebugger.h"
StreamDebugger debugger(serialGsm, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(serialGsm);
#endif

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD 115200

// Initialize GSM client
TinyGsmClient client(modem);

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

void setupModem()
{
#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    // Initialize the indicator as an output
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LED_OFF);
}

void turnOffNetlight()
{
    Serial.println("Turning off SIM800 Red LED ... ");
    modem.sendAT("+CNETLIGHT=0");
    Serial.println("OK");
}

void turnOnNetlight()
{
    Serial.print("Turning on SIM800 Red LED ... ");
    modem.sendAT("+CNETLIGHT=1");
    Serial.println("OK");
}

#define ADC_BAT 35 // TCALL 35
int bat_mv = 0;

void getBatteryFromADC()
{
  bat_mv = 0;
  uint32_t oversample = 0;
  for (size_t i = 0; i < 100; i++)
  {
    oversample += (uint32_t)analogRead(ADC_BAT);
  }
  bat_mv = (int)oversample / 100;
  bat_mv = ((float)bat_mv / 4096) * 3600 * 2;

  Serial.print("Battery from ADC: ");
  Serial.print(bat_mv);
  Serial.println("mV");
}

void setup()
{
  Serial.begin(115200);

  delay(4 * 1000);

  // Start power management
  if (!setupPMU()) {

    esp_restart();
  }

  // Some start operations
  Serial.print("Setting up modem ... ");
  setupModem();
  Serial.println("OK");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  Serial.print("Initializing modem ... ");
  if (!modem.restart()) {
    Serial.println("FAIL. Restarting NOW!");
    esp_restart();
    return;
  }
  Serial.println("OK");

  Serial.print("modem.getSimStatus() = ");
  SimStatus simStatus = modem.getSimStatus();
  Serial.print(simStatus);
  switch(simStatus) {
    case SimStatus::SIM_ERROR:
      Serial.println(" (SIM_ERROR)");
      break;
    case SimStatus::SIM_READY:
      Serial.println(" (SIM_READY)");
      break;
    case SimStatus::SIM_LOCKED:
      Serial.println(" (SIM_LOCKED)");
      break;
    case SimStatus::SIM_ANTITHEFT_LOCKED:
      Serial.println(" (SIM_ANTITHEFT_LOCKED)");
      break;
    default:
      Serial.println(" (unkown)");
      break;
  }

  Serial.print("Waiting for network ... ");
  if (!modem.waitForNetwork()) {
   Serial.println("FAIL. Restarting NOW!");
   esp_restart();
   return;
  }
  Serial.println("OK");

  Serial.print("GSM operator = ");
  Serial.println(modem.getOperator());

  Serial.print("GSM signal quality = ");
  Serial.println(modem.getSignalQuality());

  //Set ringer sound level
  SerialAT.print("AT+CRSL=0\r\n");
  delay(2);

  //Set loud speaker volume level
  SerialAT.print("AT+CLVL=0\r\n");
  delay(2);

  // Calling line identification presentation
  SerialAT.print("AT+CLIP=1\r\n");
  delay(2);

  //Set RI Pin input
  pinMode(MODEM_RI, INPUT);
}

int16_t last_signal_quality = -1;

void loop()
{
  Serial.print("Waiting for call in ... ");
  digitalWrite(LED_GPIO, LED_OFF);
  while (digitalRead(MODEM_RI)) {
    delay(100);
    if (modem.getSignalQuality() == 0) {
      Serial.print("😟");
    }
    if (!modem.waitForNetwork()) {
      Serial.println("🧨");
      esp_restart();
      return;
    }
  }
  Serial.println("OK");

  digitalWrite(LED_GPIO, LED_ON);
  delay(1 * 1000);
  Serial.print("Hanging up ... ");
  SerialAT.println("ATH");
  Serial.println("OK");

  // Make the LED blink three times before going to sleep
  int i = 3;
  while (i--) {
    digitalWrite(LED_GPIO, LED_ON);
    modem.sendAT("+SPWM=0,1000,80");
    delay(500);
    digitalWrite(LED_GPIO, LED_OFF);
    modem.sendAT("+SPWM=0,1000,0");
    delay(500);
  }
}
