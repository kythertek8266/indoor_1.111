// including libraries
#include "Arduino.h"
#include "Wire.h"
#include "uRTCLib.h"
#include "SPIFFS.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ESPAsyncWebServer.h>         // Built-in
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#include <EEPROM.h>
#include <FunctionalInterrupt.h>
#include <LiquidCrystal.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <ESPmDNS.h>                   // Built-in
#include <esp32-hal-timer.h>

static hw_timer_t *timer = NULL;

uint16_t packetIdPub;
uint16_t packetIdSub;
uint16_t packetIdUnsub;
////////////////////////////////////////////////////////////////////////////
// defining BLE UUIDs
#define ble_service1_uuid       "fc431b78-f640-11e9-802a-5aa538984bd8"
#define gasreading_char_uuid    "fc43206e-f640-11e9-802a-5aa538984bd8"
#define currenttemp_char_uuid   "fc4321d6-f640-11e9-802a-5aa538984bd8"
#define settemp_char_uuid       "fc432316-f640-11e9-802a-5aa538984bd8"
#define ble_service2_uuid       "fc43244c-f640-11e9-802a-5aa538984bd8"
#define valvestatus_char_uuid   "fc432578-f640-11e9-802a-5aa538984bd8"
#define flamestatus_char_uuid   "fc4326a4-f640-11e9-802a-5aa538984bd8"
#define buzzerstatus_char_uuid  "fc43297e-f640-11e9-802a-5aa538984bd8"
////////////////////////////////////////////////////////////////////////////

//defining pins and global variables
///////////////////////Rotary Encoder//////////////////////////////////////
#define CLK 19
#define DT 18
#define SW 5
bool currentStateCLK;
bool lastStateCLK;
boolean up = false;
boolean down = false;
boolean middle = false;

/////// Button /////////
byte debounce = 20;          // ms debounce p<eriod to prevent flickering when pressing or releasing the button
byte DCgap = 250;            // max ms between clicks for a double click event
int holdTime = 1000;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 3000;    // ms long hold period: how long to wait for press+hold event

boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the button was pressed down
long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

////////////////////////////////////////////////////////////////////////////


///////////////////////Menu//////////////////////////////////////
byte menuitem = 1;    //Main Menu items; 1=Turn ON/OFF, 2=Set Temp, 3=Set Schedule, 4=Back
byte schmenuitem = 0; // Schecule 1, 2, 3, 4
byte page = 1;
byte lastMenuItem = 1;
byte onhr[] = {0, 0, 0, 0};  //Schedule array
byte onmin[] = {0, 0, 0, 0};
byte offhr[] = {0, 0, 0, 0};
byte offmin[] = {0, 0, 0, 0};
byte settemp[] = {35, 35, 35, 35};
int schtimer = 10000; //schedule blink timers
bool blinkk = 0;
bool menuactive = 0;
int disconnectcounter = 0;
bool disc = 0;
int mqttdebug = 0;
int schdefaulttemp = 0;
bool schoverride = 0;
bool schon = 0;
int turnondow;
int turnonp;
int overridetimer = 30;
unsigned long overridestarttime = 0;
////////////////////////////////////////////////////////////////////////////


uint32_t tStart;
unsigned long startMillis;  // global variables available anywhere in the program
unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long period1 = 300000;
unsigned long lastmenuactive = 0;
unsigned long lastschcheck = 0;
//////////////MQTT topics - "MAC-address/geyser/server/variable"//////////////
String TOPIC = "/geyser/server";
String TOPIC2 = "/geyser/server/valvestatus";
String TOPIC3 = "/geyser/server/flamestatus";
String TOPIC4 = "/geyser/server/currenttemp";
String TOPIC5 = "/geyser/server/settemp";
String TOPIC6 = "/geyser/server/buzzerstatus";

String TOPIC7 = "/geyser/server/ontime";
String TOPIC8 = "/geyser/server/offtime";
String TOPIC9 = "/geyser/server/dow";
String TOPIC10 = "/geyser/server/schtemp";

String TOPIC11 = "/geyser/server/debug";
String TOPIC12 = "/geyser/server/fwversion";
String TOPIC13 = "/geyser/server/dcounter";
String TOPIC14 = "/geyser/server/schoverride";
String TOPIC15 = "/geyser/server/schstatus";

String mqtt_onhr, mqtt_onmin, mqtt_offhr, mqtt_offmin, mqtt_schtemp;
int mqtt_ontime = 0;
int mqtt_offtime = 0;
int mqtt_dow = 0;
String mac;
const char *topic, *topic2, *topic3, *topic4, *topic5, *topic6, *topic7, *topic8, *topic9, *topic10, *topic11, *topic12, *topic13, *topic14, *topic15; //after adding MAC
////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////BLE setup variables//////////////////////////////////////
static NimBLEServer* NimBLE_Server;
NimBLEService *NimBLE_Service1 = NULL;
NimBLEService *NimBLE_Service2 = NULL;
NimBLECharacteristic *gasreading_Char = NULL;
NimBLECharacteristic *valvestatus_Char = NULL;
NimBLECharacteristic *flamestatus_Char = NULL;
NimBLECharacteristic *currenttemp_Char = NULL;
NimBLECharacteristic *settemp_Char = NULL;
NimBLECharacteristic *buzzerstatus_Char = NULL;
NimBLEAdvertising *NimBLE_Advertising = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
/////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////WIFI and MQTT variables//////////////////////////////////////
const char *wifi_ssid = "KytherTek";
const char *wifi_pswd = "x11y22z99";

#define MQTT_HOST IPAddress(3, 145, 20, 123)
#define MQTT_PORT 1883
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool conn = 0;

String FirmwareVer = {
  "1.11"
};
#define URL_fw_Version "http://ezgeyser.atwebpages.com/test_bin_version.txt"
#define URL_fw_Bin "http://ezgeyser.atwebpages.com/test_firmware.bin" // bin for your board 
unsigned long lastupdatecheck = 0;
unsigned long mqttlastsent = 0;
#define BUFFER_LEN  256
long lastMsg = 0;
char msg[BUFFER_LEN];
char topicname[256];
int value = 0;
byte mac2[6];
char mac_Id[18];
int count = 1;
//=============================================================================================================================

bool sstart = 0;
unsigned long delaytimer = 0;
////////////////////////////////////////BLE,MQTT data variables//////////////////////////////////////
int  ct = 0;   //current temp
int st = 35;
int tempst = 35;
String vs, fst, bs, gr;      //valve status , flame status, buzzer status, gas reading
String rxtopic, rxpayload, rxlength;
std::string s1 = "", s2 = "", s3 = "", s4 = "", s5 = "", s6 = "";
char tempchar[2];   //for converting int to std::string
WiFiClient wificlient;
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
bool ch = true;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// Schedule variables //////////////////////////////////////
bool scheduleon = 0;
int schofftime = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool sent1 = 0, sent2 = 0, sent3 = 0, sent4 = 0, sent5 = 0, sent6 = 0, sent7 = 0;

//////////////////////////////////////// lcd pinouts //////////////////////////////////////
const byte rs = 13, en = 12, d4 = 14, d5 = 27, d6 = 26, d7 = 25;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int lcdtimer = 10000;
byte a = 0;

//RTC pinouts
uRTCLib rtc;        //RTC address - dont touch this
const int rtc_sda = 21;  //gpio 21
const int rtc_scl = 22;  //gpio 22
int year, month, day, hour, minute, sec, rtcdow;

unsigned long initial1 = 10000;
//, initial2;

////////////////////////////// ???variables//////////////////////////////////////
int ist = 0;
int statusCode;
const char* ssid = "text";
const char* passphrase = "text";
String sr;
String webpage = "";
////////////////////////////////////////////////////////////////////////////////////////////////

String epass = "";
String esid = "";
//Function Decalration for web server
bool testWifi(void);
void launchWeb(void);
void setupAP(void);

int testcounter = millis();
bool teststate = 0;





String       DataFile = "params.txt";            // Storage file name on flash
String       Time_str, DoW_str;                  // For Date and Time

const char* Timezone   = "GMT0BST,M3.5.0/01,M10.5.0/02";
int    UnixTime             = 0;          // Time now (when updated) of the current time

String TimerState           = "OFF";      // Current setting of the timer
bool   ManualOverride       = false;      // Manual override
#define NumOfEvents     10             // Number of events per-day, 4 is a practical limit
struct settings {
  String DoW;                // Day of Week for the programmed event
  String Start[NumOfEvents]; // Start time
  String Stop[NumOfEvents];  // End time
  String Temp[NumOfEvents];  // Required temperature during the Start-End times
};
settings     Timer[7];                           // Timer settings, 7-days of the week



//#########################################################################################
boolean SetupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");                               // (gmtOffset_sec, daylightOffset_sec, ntpServer)
  setenv("TZ", Timezone, 1);                                       // setenv()adds "TZ" variable to the environment, only used if set to 1, 0 means no change
  tzset();
  delay(200);
  bool TimeStatus = UpdateLocalTime();
  Serial.print("Time Status");
  Serial.println(TimeStatus);
  return TimeStatus;
}
//#########################################################################################
boolean UpdateLocalTime() {
  struct tm timeinfo;
  time_t now;
  char  time_output[30];
  while (!getLocalTime(&timeinfo, 15000)) {                        // Wait for up to 15-sec for time to synchronise
    return false;
  }
  time(&now);
  UnixTime = now;
  //See http://www.cplusplus.com/reference/ctime/strftime/
  strftime(time_output, sizeof(time_output), "%H:%M", &timeinfo);  // Creates: '14:05'
  Time_str = time_output;
  Serial.print("Time String");
  Serial.println(Time_str);
  strftime(time_output, sizeof(time_output), "%w", &timeinfo);     // Creates: '0' for Sun
  DoW_str  = time_output;
  Serial.print("DoW String");
  Serial.println(DoW_str);
  return true;
}
//#########################################################################################
String ConvertUnixTime(int unix_time) {
  time_t tm = unix_time;
  struct tm *now_tm = localtime(&tm);
  char output[40];
  strftime(output, sizeof(output), "%H:%M", now_tm);               // Returns 21:12
  Serial.print("Converted Time");
  Serial.println(output);
  return output;
}
//#########################################################################################
void StartSPIFFS() {
  Serial.println("Starting SPIFFS");
  boolean SPIFFS_Status;
  SPIFFS_Status = SPIFFS.begin();
  if (SPIFFS_Status == false)
  { // Most likely SPIFFS has not yet been formated, so do so
    Serial.println("Formatting SPIFFS (it may take some time)...");
    SPIFFS.begin(true); // Now format SPIFFS
    File datafile = SPIFFS.open("/" + DataFile, "r");
    if (!datafile || !datafile.isDirectory()) {
      Serial.println("SPIFFS failed to start..."); // Nothing more can be done, so delete and then create another file
      SPIFFS.remove("/" + DataFile); // The file is corrupted!!
      datafile.close();
    }
  }
  else Serial.println("SPIFFS Started successfully...");
}
//#########################################################################################
void Initialise_Array() {
  Timer[0].DoW = "Sun"; Timer[1].DoW = "Mon"; Timer[2].DoW = "Tue"; Timer[3].DoW = "Wed"; Timer[4].DoW = "Thu"; Timer[5].DoW = "Fri"; Timer[6].DoW = "Sat";
}
//#########################################################################################

//#########################################################################################

void RecoverSettings() {
  String Entry;

  String temp[NumOfEvents];
  String star[NumOfEvents];
  String sto[NumOfEvents];
  String Mon[NumOfEvents];
  String Tue[NumOfEvents];
  String Wed[NumOfEvents];
  String Thur[NumOfEvents];
  String Fri[NumOfEvents];
  String Sat[NumOfEvents];
  String Sun[NumOfEvents];
  Serial.println("Reading settings...");
  File dataFile = SPIFFS.open("/" + DataFile, "r");
  if (dataFile) { // if the file is available, read it
    Serial.println("Recovering settings...");
    while (dataFile.available()) {

      for (int i = 0; i < NumOfEvents; i++)
      {
        temp[i] = dataFile.readStringUntil('\n'); temp[i].trim();
        star[i] = dataFile.readStringUntil('\n'); star[i].trim();
        sto[i] = dataFile.readStringUntil('\n'); sto[i].trim();

        Mon[i] = dataFile.readStringUntil('\n'); Mon[i].trim();
        Tue[i] = dataFile.readStringUntil('\n'); Tue[i].trim();
        Wed[i] = dataFile.readStringUntil('\n'); Wed[i].trim();
        Thur[i] = dataFile.readStringUntil('\n'); Thur[i].trim();
        Fri[i] = dataFile.readStringUntil('\n'); Fri[i].trim();
        Sat[i] = dataFile.readStringUntil('\n'); Sat[i].trim();
        Sun[i] = dataFile.readStringUntil('\n'); Sun[i].trim();
      }
      dataFile.close();
      Serial.println("Settings recovered...");
    }
    for (int i = 0; i < NumOfEvents; i++)
    {
      if (Mon[i] == "1")
      {
        Timer[1].Temp[i] = temp[i];
        Timer[1].Start[i] = star[i];
        Timer[1].Stop[i] = sto[i];
      }

      if (Tue[i] == "1")
      {
        Timer[2].Temp[i] = temp[i];
        Timer[2].Start[i] = star[i];
        Timer[2].Stop[i] = sto[i];
      }

      if (Wed[i] == "1")
      {
        Timer[3].Temp[i] = temp[i];
        Timer[3].Start[i] = star[i];
        Timer[3].Stop[i] = sto[i];
      }

      if (Thur[i] == "1")
      {
        Timer[4].Temp[i] = temp[i];
        Timer[4].Start[i] = star[i];
        Timer[4].Stop[i] = sto[i];
      }

      if (Fri[i] == "1")
      {
        Timer[5].Temp[i] = temp[i];
        Timer[5].Start[i] = star[i];
        Timer[5].Stop[i] = sto[i];
      }

      if (Sat[i] == "1")
      {
        Timer[6].Temp[i] = temp[i];
        Timer[6].Start[i] = star[i];
        Timer[6].Stop[i] = sto[i];
      }

      if (Sun[i] == "1")
      {
        Timer[0].Temp[i] = temp[i];
        Timer[0].Start[i] = star[i];
        Timer[0].Stop[i] = sto[i];
      }
    }
    for (byte dow = 0; dow < 7; dow++) {
      for (byte p = 0; p < NumOfEvents; p++) {

        Serial.print(dow);
        Serial.print(". ");
        Serial.print(Timer[dow].Temp[p]);
        Serial.print(" ");
        Serial.print(Timer[dow].Start[p]);
        Serial.print("-");
        Serial.print(Timer[dow].Stop[p]);
        Serial.println(";  ");
      }
    }
  }
}
//#########################################################################################

void SetupDeviceName(const char *DeviceName) {
  if (MDNS.begin(DeviceName)) { // The name that will identify your device on the network
    Serial.println("mDNS responder started");
    Serial.print("Device name: ");
    Serial.println(DeviceName);
    //MDNS.addService("n8i-mlp", "tcp", 23); // Add service
    MDNS.addService("http", "tcp", 80);
  }
  else
    Serial.println("Error setting up MDNS responder");
}
//#########################################################################################


void CheckTimerEvent() {
  String TimeNow;
  String h;
  String m;
  rtc.refresh();
  if (rtc.hour() < 10) h = "0" + String(rtc.hour());
  else h = String(rtc.hour());
  if (rtc.minute() < 10) m = "0" + String(rtc.minute());
  else m = String(rtc.minute());

  TimeNow = h + ":" + m;                             // Switch timer off until decided by the schedule
  DoW_str = rtc.dayOfWeek();
  if (DoW_str == "7")
    DoW_str = "0";
  //Serial.print(DoW_str);
  for (byte dow = 0; dow < 7; dow++) {                     // Look for any valid timer events, if found turn the heating on
    for (byte p = 0; p < NumOfEvents; p++) {
      // Now check for a scheduled ON time, if so Switch the Timer ON and check the temperature against target temperature
      if (String(dow) == DoW_str && (TimeNow >= Timer[dow].Start[p] && TimeNow < Timer[dow].Stop[p] && Timer[dow].Start[p] != "") && !schon)
      {
        if (!schoverride)
        {
          Serial.println("Scheduled Turn On");
          st = Timer[dow].Temp[p].toInt();

          if (mqttClient.connected())
          {
            packetIdUnsub = mqttClient.unsubscribe(topic5);
            delay(100);
            s5 = String(st).c_str();
            packetIdPub = mqttClient.publish(topic5, 1, false, String(s5.c_str()).c_str());
            packetIdSub = mqttClient.subscribe(topic5, 1);
            delay(100);
          }

          TimerState = "ON";
          turnondow = dow;
          turnonp = p;
          schon = 1;
          Serial.print("dow: ");
          Serial.println(dow);
          Serial.print("p: ");
          Serial.println(p);
          EEPROM.write(97, st);
          EEPROM.commit();
        }
        else
        {
          Serial.println("Skipping Schedule because of Override");
        }
      }

      else if (String(dow) == DoW_str && (TimeNow >= Timer[turnondow].Stop[turnonp] && Timer[turnondow].Start[turnonp] != "") && schon)
      {
        if (!schoverride)
        {
          Serial.println("Scheduled Turn off");
          st = schdefaulttemp;

          if (mqttClient.connected())
          {
            packetIdUnsub = mqttClient.unsubscribe(topic5);
            delay(100);
            s5 = String(st).c_str();
            packetIdPub = mqttClient.publish(topic5, 1, false, String(s5.c_str()).c_str());
            packetIdSub = mqttClient.subscribe(topic5, 1);
            delay(100);
          }

          TimerState = "OFF";
          schon = 0;
          Serial.print("dow2: ");
          Serial.println(dow);
          Serial.print("p2: ");
          Serial.println(p);
          EEPROM.write(97, st);
          EEPROM.commit();
        }
        else
        {
          Serial.println("Skipping scheduled turn off because of override");
        }
      }
    }
  }
}


void IRAM_ATTR resetModule() {
  ets_printf("----- WATCHDOG REBOOT -----\n");
  ESP.restart();
}

void wdt_enable(const unsigned long durationMs) {
  //timer 0, div 80
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &resetModule, true);
  //set time in us
  timerAlarmWrite(timer, durationMs * 1000, false);
  //enable interrupt
  timerAlarmEnable(timer);
}

void wdt_disable() {
  if (timer != NULL) {
    //disable interrupt
    timerDetachInterrupt(timer);
    timerEnd(timer);
    timer = NULL;
  }
}

void wdt_reset() {
  //reset timer (feed watchdog)
  if (timer != NULL) {
    timerWrite(timer, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////////


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi");
  //------------------------ Read eeprom for ssid and pass
  //Serial.println("Reading EEPROM ssid");

  //String esid;
  esid = "";
  epass = "";


  for (int a = 0; a < 32; a++)
  {
    char x = char(EEPROM.read(a));
    if (x < 127)
      conn = 1;
  }

  if (conn)
  {
    for (int i = 0; i < 32; ++i)
    {
      esid += char(EEPROM.read(i));
    }
    //Serial.println();
    //Serial.print("SSID: ");
    //Serial.println(esid);
    //Serial.println("Reading EEPROM pass");


    for (int i = 32; i < 96; ++i)
    {
      epass += char(EEPROM.read(i));
    }
    //Serial.print("PASS: ");
    //Serial.println(epass);
    WiFi.begin(esid.c_str(), epass.c_str());
  }
}

void connectToMqtt() {
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
  }
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      setRTC();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  packetIdSub =  mqttClient.subscribe(topic5, 1);
  Serial.println("TOPIC 5 SUBed");
  Serial.println(packetIdSub);

  packetIdSub =  mqttClient.subscribe(topic11, 1);
  Serial.println("TOPIC 11 SUBed");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {

  if (len > 0)
  {
    String payloadstring = "";
    Serial.print("Message arrived");
    for (int i = 0; i < len; i++) {
      payloadstring = payloadstring + (char)payload[i];
    }

    rxtopic = String((char*)topic);
    //Serial.println("saved topic:  " + rxtopic);
    rxpayload = String(atoi((char*)payload));
    //Serial.println("saved payload:  " + rxpayload);
    rxlength = String(len);
    //Serial.println("saved length:   " + rxlength);


    if (rxtopic == TOPIC5) //settemp
    {

      st = rxpayload.toInt(); // updating set temp if topic recieved from cloud
      tempst = st;
      schon = 0;
      schoverride = 1;
      overridestarttime = millis();
      overridetimer = 60;
    }

    if (rxtopic == TOPIC11) //debug
    {
      mqttdebug = rxpayload.toInt();
    }
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged - ");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

AsyncWebServer server(80);
void setup_wifi() {

  esid = "";
  epass = "";

  conn = 0;
  for (int a = 0; a < 32; a++)
  {
    char x = char(EEPROM.read(a));
    if (x < 127)
      conn = 1;
  }

  if (conn)
  {
    for (int i = 0; i < 32; ++i)
    {
      esid += char(EEPROM.read(i));
    }
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(esid);
    //Serial.println("Reading EEPROM pass");


    for (int i = 32; i < 96; ++i)
    {
      epass += char(EEPROM.read(i));
    }
    Serial.print("PASS: ");
    Serial.println(epass);
    WiFi.begin(esid.c_str(), epass.c_str());
  }
  String ap_ssid = "EzGeyser_";
  ap_ssid = ap_ssid + mac_Id[12];
  ap_ssid = ap_ssid + mac_Id[13];
  ap_ssid = ap_ssid + mac_Id[14];
  ap_ssid = ap_ssid + mac_Id[15];
  ap_ssid = ap_ssid + mac_Id[16];

  String ap_pass = "";
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid.c_str(), ap_pass.c_str());

  //const char* ssid ="KytherTek"; //Replace with your WiFi Name
  //const char* password ="x11y22z99"; // Replace with your WiFi Password
  // WiFi.begin(ssid, password);

  launchWeb();
  Serial.println("Succesfully Connected!!!");
}

void setup_mqtt_topics() {
  //WiFi.mode(WIFI_MODE_STA);
  mac = WiFi.macAddress();
  Serial.println(mac);
  TOPIC2 = String(mac) + TOPIC2;
  TOPIC3 = String(mac) + TOPIC3;
  TOPIC4 = String(mac) + TOPIC4;
  TOPIC5 = String(mac) + TOPIC5;
  TOPIC6 = String(mac) + TOPIC6;

  TOPIC7 = String(mac) + TOPIC7;
  TOPIC8 = String(mac) + TOPIC8;
  TOPIC9 = String(mac) + TOPIC9;
  TOPIC10 = String(mac) + TOPIC10;

  TOPIC11 = String(mac) + TOPIC11;
  TOPIC12 = String(mac) + TOPIC12;
  TOPIC13 = String(mac) + TOPIC13;
  TOPIC14 = String(mac) + TOPIC14;
  TOPIC15 = String(mac) + TOPIC15;
  delay(100);

  topic2 = (char*)TOPIC2.c_str();
  //Serial.println(topic2);
  topic3 = (char*)TOPIC3.c_str();
  //Serial.println(topic3);
  topic4 = (char*)TOPIC4.c_str();
  //Serial.println(topic4);
  topic5 = (char*)TOPIC5.c_str();
  //Serial.println(topic5);
  topic6 = (char*)TOPIC6.c_str();
  //Serial.println(topic6);

  topic7 = (char*)TOPIC7.c_str();
  topic8 = (char*)TOPIC8.c_str();
  topic9 = (char*)TOPIC9.c_str();
  topic10 = (char*)TOPIC10.c_str();

  topic11 = (char*)TOPIC11.c_str();
  topic12 = (char*)TOPIC12.c_str();
  topic13 = (char*)TOPIC13.c_str();
  topic14 = (char*)TOPIC14.c_str();
  topic15 = (char*)TOPIC15.c_str();
  Serial.println("MQTT Topics INITIALIZED");
  delay(100);
}
void launchWeb()
{
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  createWebServer();
  // Start the server
  server.begin();
  Serial.println("Server started");
  //  button1.checkPressed();
}

void createWebServer()
{
  {
    server.on("/homepage", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      Homepage();
      request->send(200, "text/html", webpage);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      Homepage();
      request->send(200, "text/html", webpage);
    });


    server.on("/timer", HTTP_GET, [](AsyncWebServerRequest * request) {
      TimerSet();
      request->send(200, "text/html", webpage);
    });



    server.on("/resetschedule", HTTP_GET, [](AsyncWebServerRequest * request) {
      for (byte dow = 0; dow < 7; dow++) {
        for (byte p = 0; p < NumOfEvents; p++) {
          Timer[dow].Temp[p] = "";
          Timer[dow].Start[p] = "";
          Timer[dow].Stop[p] = "";
        }
      }
      File dataFile = SPIFFS.open("/" + DataFile, "w");
      if (dataFile) { // Save settings
        Serial.println("Saving settings...");

        for (int i = 0; i < NumOfEvents; i++)
        {
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
          dataFile.println("");
        }
        dataFile.close();
        Serial.println("Schedules Cleared");
      }
      schon = 0;
      TimerState = "OFF";
      request->redirect("/timer");                       // Go back to home page
    });

    server.on("/handletimer", HTTP_GET, [](AsyncWebServerRequest * request) {
      String temp[NumOfEvents];
      String star[NumOfEvents];
      String sto[NumOfEvents];
      String Mon[NumOfEvents];
      String Tue[NumOfEvents];
      String Wed[NumOfEvents];
      String Thur[NumOfEvents];
      String Fri[NumOfEvents];
      String Sat[NumOfEvents];
      String Sun[NumOfEvents];

      for (int i = 0; i < NumOfEvents; i++)
      {
        temp[i] = request->arg(String(i + 1) + ".Temp");
        star[i] = request->arg(String(i + 1) + ".Start");
        sto[i] = request->arg(String(i + 1) + ".Stop");

        Mon[i] = request->arg(String(i + 1) + ".Mon");
        Tue[i] = request->arg(String(i + 1) + ".Tue");
        Wed[i] = request->arg(String(i + 1) + ".Wed");
        Thur[i] = request->arg(String(i + 1) + ".Thu");
        Fri[i] = request->arg(String(i + 1) + ".Fri");
        Sat[i] = request->arg(String(i + 1) + ".Sat");
        Sun[i] = request->arg(String(i + 1) + ".Sun");
      }


      for (int i = 0; i < NumOfEvents; i++)
      {
        if (Mon[i] == "1")
        {
          Timer[1].Temp[i] = temp[i];
          Timer[1].Start[i] = star[i];
          Timer[1].Stop[i] = sto[i];
        }

        if (Tue[i] == "1")
        {
          Timer[2].Temp[i] = temp[i];
          Timer[2].Start[i] = star[i];
          Timer[2].Stop[i] = sto[i];
        }

        if (Wed[i] == "1")
        {
          Timer[3].Temp[i] = temp[i];
          Timer[3].Start[i] = star[i];
          Timer[3].Stop[i] = sto[i];
        }

        if (Thur[i] == "1")
        {
          Timer[4].Temp[i] = temp[i];
          Timer[4].Start[i] = star[i];
          Timer[4].Stop[i] = sto[i];
        }

        if (Fri[i] == "1")
        {
          Timer[5].Temp[i] = temp[i];
          Timer[5].Start[i] = star[i];
          Timer[5].Stop[i] = sto[i];
        }

        if (Sat[i] == "1")
        {
          Timer[6].Temp[i] = temp[i];
          Timer[6].Start[i] = star[i];
          Timer[6].Stop[i] = sto[i];
        }

        if (Sun[i] == "1")
        {
          Timer[0].Temp[i] = temp[i];
          Timer[0].Start[i] = star[i];
          Timer[0].Stop[i] = sto[i];
        }
      }

      for (byte dow = 0; dow < 7; dow++) {
        for (byte p = 0; p < NumOfEvents; p++) {

          Serial.print(dow);
          Serial.print(". ");
          Serial.print(Timer[dow].Temp[p]);
          Serial.print(" ");
          Serial.print(Timer[dow].Start[p]);
          Serial.print("-");
          Serial.print(Timer[dow].Stop[p]);
          Serial.println(";  ");
        }
      }

      Serial.println("Getting ready to Save settings...");
      File dataFile = SPIFFS.open("/" + DataFile, "w");
      if (dataFile) { // Save settings
        Serial.println("Saving settings...");
        for (int i = 0; i < NumOfEvents; i++)
        {
          dataFile.println(temp[i]);
          dataFile.println(star[i]);
          dataFile.println(sto[i]);
          dataFile.println(Mon[i]);
          dataFile.println(Tue[i]);
          dataFile.println(Wed[i]);
          dataFile.println(Thur[i]);
          dataFile.println(Fri[i]);
          dataFile.println(Sat[i]);
          dataFile.println(Sun[i]);
        }
        dataFile.close();
        Serial.println("Settings saved...");
      }
      schon = 0;
      TimerState = "OFF";
      request->redirect("/homepage");                       // Go back to home page
    });



    server.on("/setup", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      wifisetting();
      request->send(200, "text/html", webpage);
    });

    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      request->redirect("/homepage");       // Go to home page
    });

    server.on("/help", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      request->redirect("/homepage");       // Go to home page
    });

    server.on("/devicesetup", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      devicesetup();
      request->send(200, "text/html", webpage);
    });

    server.on("/defaulttemp", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      String defaulttemp = request->arg("defaulttemp");
      schdefaulttemp = defaulttemp.toInt();
      EEPROM.write(96, schdefaulttemp);
      EEPROM.commit();
      request->redirect("/homepage");       // Go to home page
    });


    server.on("/settemp", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      String setttemp = request->arg("settemp");

      st = setttemp.toInt();
      tempst = st;
      schon = 0;
      schoverride = 1;
      overridestarttime = millis();
      overridetimer = 60;

      if (mqttClient.connected())
      {
        packetIdUnsub = mqttClient.unsubscribe(topic5);
        delay(100);
        s5 = String(st).c_str();
        packetIdPub = mqttClient.publish(topic5, 1, false, String(s5.c_str()).c_str());
        packetIdSub = mqttClient.subscribe(topic5, 1);
        delay(100);
      }
      request->redirect("/homepage");       // Go to home page
    });

    server.on("/setting", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      String qsid = request->arg("ssid");
      String qpass = request->arg("pass");
      if (qsid.length() > 0 && qpass.length() > 0)
      {
        Serial.println("clearing eeprom");
        for (int i = 0; i < 96; ++i)
        {
          EEPROM.write(i, 0);
        }
        Serial.println(qsid);
        Serial.println("");
        Serial.println(qpass);
        Serial.println("");

        Serial.println("writing eeprom ssid:");
        for (unsigned int i = 0; i < qsid.length(); ++i)
        {
          EEPROM.write(i, qsid[i]);
          Serial.print("Wrote: ");
          Serial.println(qsid[i]);
        }
        Serial.println("writing eeprom pass:");
        for (unsigned int i = 0; i < qpass.length(); ++i)
        {
          EEPROM.write(32 + i, qpass[i]);
          Serial.print("Wrote: ");
          Serial.println(qpass[i]);
        }
        EEPROM.commit();

        webpage = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
        statusCode = 200;
        request->send(statusCode, "application/json", webpage);
        request->redirect("/homepage");       // Go to home page
        delay(1000);
        ESP.restart();
      }
      else
      {
        webpage = "{\"Error\":\"404 not found\"}";
        statusCode = 404;
        Serial.println("Sending 404");
      }
      //request->sendHeader("Access-Control-Allow-Origin", "*");
      request->send(statusCode, "application/json", webpage);
    });
  }
}

void Homepage() {
  webpage = "";
  append_HTML_header(1);
  webpage += "<h2>EzGeyser_";
  webpage += mac_Id;
  webpage += "</h2><br>";


  if (deviceConnected)
  {
    webpage += "<div class='numberCircle'><span class=" + String((vs == "ON" ? "'on'>" : "'off'>")) + String(ct) + "&deg;</span></div><br>";

    webpage += "</p><form name='myform' action='settemp' method='get'>";
    webpage += "<table class='centre'>";
    webpage += "<tr><td>Override Temperature: </td>";
    webpage += "<td class='select'> <Select onchange='myform.submit()' name='settemp'>";

    if (st == 0)
      webpage += "<option value='0' selected='selected'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
    else if (st == 30)
      webpage += "<option value='0'>OFF </option><option value='30' selected='selected'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
    else if (st == 35)
      webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35' selected='selected'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
    else if (st == 40)
      webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40' selected='selected'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
    else if (st == 45)
      webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45' selected='selected'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
    else if (st == 55)
      webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55' selected='selected'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
    else if (st == 65)
      webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65' selected='selected'>Very Hot (65)</option>";
    else
      webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";

    webpage += "</select></td>";
    webpage += "</tr>";
    webpage += "</table>";
    webpage += "</form><br>";

    webpage += "<table class='centre'>";
    webpage += "<tr>";
    webpage += "<td> Current Temperature </td>";
    webpage += "<td> Target Temperature  </td>";
    webpage += "<td>     Geyser     </td>";
    webpage += "<td>     Pilot     </td>";
    webpage += "<td> Schedule </td>";
    webpage += "</tr>";
    webpage += "<tr>";
    webpage += "<td class='large'>" + String(ct)       + "&deg;</td>";
    webpage += "<td class='large'>" + String(st) + "&deg;</td>";
    webpage += "<td class='large'><span class=" + String((vs == "ON" ? "'on'>" : "'off'>")) + vs + "</span></td>"; // (condition ? that : this) if this then that else this
    webpage += "<td class='large'><span class=" + String((fst == "ON" ? "'on'>" : "'off'>")) + fst + "</span></td>"; // (condition ? that : this) if this then that else this
    webpage += "<td class='large'><span class=" + String((TimerState == "ON" ? "'on'>" : "'off'>")) + TimerState + "</span></td>";
    webpage += "</tr>";
    webpage += "</table>";
    webpage += "<br>";


    if (schoverride) {
      webpage += "<h2 style='color:Red;'>MANUAL OVERRIDE ON";
      webpage += "</h2>";
      webpage += "<h3 style='color:Red;'>Returning to default temp in ";

      int timeremaining2 = ((overridetimer * 1000) - (millis() - overridestarttime)) / 1000;
      int remaininghr = timeremaining2 / 60;
      int remainingmin = timeremaining2 % 60;

      if (remaininghr > 1)
      {
        webpage += String(remaininghr);
        webpage += " hrs ";
      }
      else if (remaininghr > 0)
      {
        webpage += String(remaininghr);
        webpage += " hr ";
      }

      if (remainingmin > 1)
      {
        webpage += String(remainingmin);
        webpage += " mins ";
      }
      else if (remainingmin > 0)
      {
        webpage += String(remainingmin);
        webpage += " min ";
      }

      webpage += "</h3><br>";
    }

    if (bs == "1")
    {
      webpage += "<br>";
      webpage += "<br>";
      webpage += "<h2 style='color:Red;'>ERROR: NO GAS";
      webpage += "</h2><br>";
    }
  }
  else
  {
    webpage += "<br>";
    webpage += "<br>";
    webpage += "<h1 style='color:Red;'>Geyser Disconnected";
    webpage += "</h1><br>";
  }

  append_HTML_footer();
}

void TimerSet() {
  webpage = "";
  append_HTML_header(0);
  webpage += "<h2>Schedule Setup</h2><br>";
  webpage += "<h3>Enter required temperatures and time (use Clock symbol for ease of time entry)</h3><br>";
  webpage += "<FORM action='/handletimer'>";
  webpage += "<table class='centre'>";

  webpage += "<col><col><col><col>";
  webpage += "<tr>";
  webpage += "<td> SchID </td>";
  webpage += "<td>  Set Temp  </td>";
  webpage += "<td>     Start      </td>";
  webpage += "<td>     Stop     </td>";
  webpage += "</tr>";

  String Entry;

  String temp[NumOfEvents];
  String star[NumOfEvents];
  String sto[NumOfEvents];
  String Mon[NumOfEvents];
  String Tue[NumOfEvents];
  String Wed[NumOfEvents];
  String Thur[NumOfEvents];
  String Fri[NumOfEvents];
  String Sat[NumOfEvents];
  String Sun[NumOfEvents];
  Serial.println("Reading settings...");
  File dataFile = SPIFFS.open("/" + DataFile, "r");
  if (dataFile) { // if the file is available, read it
    Serial.println("Recovering settings...");
    while (dataFile.available()) {

      for (int i = 0; i < NumOfEvents; i++)
      {
        temp[i] = dataFile.readStringUntil('\n'); temp[i].trim();
        star[i] = dataFile.readStringUntil('\n'); star[i].trim();
        sto[i] = dataFile.readStringUntil('\n'); sto[i].trim();

        Mon[i] = dataFile.readStringUntil('\n'); Mon[i].trim();
        Tue[i] = dataFile.readStringUntil('\n'); Tue[i].trim();
        Wed[i] = dataFile.readStringUntil('\n'); Wed[i].trim();
        Thur[i] = dataFile.readStringUntil('\n'); Thur[i].trim();
        Fri[i] = dataFile.readStringUntil('\n'); Fri[i].trim();
        Sat[i] = dataFile.readStringUntil('\n'); Sat[i].trim();
        Sun[i] = dataFile.readStringUntil('\n'); Sun[i].trim();
      }
      dataFile.close();
      Serial.println("Settings recovered...");
    }
  }

  for (int i = 1; i < 11; i++)
  {
    webpage += "<tr>";
    webpage += "<td>";
    webpage += String(i);
    webpage += "</td>";
    webpage += "<td class='select'> <select name='" + String(i) + ".Temp' td>";
    if (temp[i - 1] == "0")
      webpage += "<option value='0' selected='selected'>OFF</option><option value='35'>Warm</option><option value='45'>Hot</option><option value='55'>Hot+</option><option value='65'>Very Hot</option></select></td>";
    else if (temp[i - 1] == "35")
      webpage += "<option value='0'>OFF</option><option value='35' selected='selected'>Warm</option><option value='45'>Hot</option><option value='55'>Hot+</option><option value='65'>Very Hot</option></select></td>";
    else if (temp[i - 1] == "45")
      webpage += "<option value='0'>OFF</option><option value='35'>Warm</option><option value='45' selected='selected'>Hot</option><option value='55'>Hot+</option><option value='65'>Very Hot</option></select></td>";
    else if (temp[i - 1] == "55")
      webpage += "<option value='0'>OFF</option><option value='35'>Warm</option><option value='45'>Hot</option><option value='55' selected='selected'>Hot+</option><option value='65'>Very Hot</option></select></td>";
    else if (temp[i - 1] == "65")
      webpage += "<option value='0'>OFF</option><option value='35'>Warm</option><option value='45'>Hot</option><option value='55'>Hot+</option><option value='65' selected='selected'>Very Hot</option></select></td>";
    else
      webpage += "<option value='0'>OFF</option><option value='35'>Warm</option><option value='45'>Hot</option><option value='55'>Hot+</option><option value='65'>Very Hot</option></select></td>";

    webpage += "<td><input type='time' name='" +  String(i) + ".Start' value='" + star[i - 1] + "'></td>";
    webpage += "<td><input type='time' name='"  + String(i) + ".Stop' value='" + sto[i - 1] + "'></td>";
    webpage += "</tr>";

    webpage += "<td colspan='4'>";
    webpage += "<input type='checkbox' value='1' "; if (Mon[i - 1] == "1") webpage += "checked "; webpage += "name='" + String(i) + ".Mon'>Mon  ";
    webpage += "<input type='checkbox' value='1' "; if (Tue[i - 1] == "1") webpage += "checked "; webpage += "name='" + String(i) + ".Tue'>Tue  ";
    webpage += "<input type='checkbox' value='1' "; if (Wed[i - 1] == "1") webpage += "checked "; webpage += "name='" + String(i) + ".Wed'>Wed  ";
    webpage += "<input type='checkbox' value='1' "; if (Thur[i - 1] == "1") webpage += "checked "; webpage += "name='" + String(i) + ".Thu'>Thu  ";
    webpage += "<input type='checkbox' value='1' "; if (Fri[i - 1] == "1") webpage += "checked "; webpage += "name='" + String(i) + ".Fri'>Fri  ";
    webpage += "<input type='checkbox' value='1' "; if (Sat[i - 1] == "1") webpage += "checked "; webpage += "name='" + String(i) + ".Sat'>Sat  ";
    webpage += "<input type='checkbox' value='1' "; if (Sun[i - 1] == "1") webpage += "checked "; webpage += "name='" + String(i) + ".Sun'>Sun  ";
    webpage += "<hr style='width:75%;text-align:center'>";
  }


  webpage += "</table>";
  webpage += "<br><input type='submit' value='Enter'>";
  webpage += "</form>";

  webpage += "<FORM action='/resetschedule'>";
  webpage += "<table class='centre'>";
  webpage += "<col>";
  webpage += "<tr>";
  webpage += "<input type='submit' value='Reset Schedules'><br><br>";
  webpage += "</div></form>";
  append_HTML_footer();
}

void wifisetting()
{
  webpage = "";
  append_HTML_header(0);
  webpage += "<h3>Setup Wifi</h3><br>";
  webpage += "</p><form method='get' action='setting'>";
  webpage += "<table class='centre'>";
  webpage += "<tr><td>Wifi: </td>";
  webpage += "<td class='select'> <select name='ssid'>";



  if (WiFi.status() != WL_CONNECTED)
  {
    xTimerStop(wifiReconnectTimer, 0);
    xTimerStop(mqttReconnectTimer, 0);
    WiFi.disconnect();
  }

  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    // Print SSID and RSSI for each network found
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(WiFi.SSID(i));
    Serial.print(" (");
    Serial.print(WiFi.RSSI(i));
    Serial.print(")");
    Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
    webpage += "<option value='";
    webpage += WiFi.SSID(i);
    webpage += "'>";
    webpage += WiFi.SSID(i);
    webpage += " (";
    webpage += WiFi.RSSI(i);
    webpage += ") ";
    webpage += "</option>";
    //delay(10);
  }
  webpage += "</select></td>";
  webpage += "</tr>";
  webpage += "</table>";
  webpage += "</p><form method='get' action='setting'><label>Password: </label><input name='pass' length=64><input type='submit'></form>";
  //webpage += "</form>";

  append_HTML_footer();
}

void devicesetup()
{
  webpage = "";
  append_HTML_header(0);
  webpage += "<h3>Device Configuration</h3><br>";
  webpage += "</p><form method='get' action='defaulttemp'>";
  webpage += "<table class='centre'>";
  webpage += "<tr><td>Default Temp when Schedule OFF: </td>";
  webpage += "<td class='select'> <select name='defaulttemp'>";
  int tempeeprom = EEPROM.read(96);


  if (tempeeprom == 0)
    webpage += "<option value='0' selected='selected'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
  else if (tempeeprom == 30)
    webpage += "<option value='0'>OFF </option><option value='30' selected='selected'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
  else if (tempeeprom == 35)
    webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35' selected='selected'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
  else if (tempeeprom == 40)
    webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40' selected='selected'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
  else if (tempeeprom == 45)
    webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45' selected='selected'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
  else if (tempeeprom == 55)
    webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55' selected='selected'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";
  else if (tempeeprom == 65)
    webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65' selected='selected'>Very Hot (65)</option>";
  else
    webpage += "<option value='0'>OFF </option><option value='30'>Warm (30)</option><option value='35' selected='selected'>Warm+ (35)</option><option value='40'>Hot (40)</option><option value='45'>Hot+ (45)</option><option value='55'>Hot++ (55)</option><option value='65'>Very Hot (65)</option>";

  webpage += "</select></td>";
  webpage += "</tr>";
  webpage += "</table>";
  webpage += "</p><form method='get' action='defaulttempsetting'><label></label><input type='submit'></form>";
  //webpage += "</form>";

  rtc.refresh();
  webpage += "<br><h4>Time: ";
  webpage += String(rtc.hour());
  webpage += ":";
  webpage += String(rtc.minute());
  webpage += ":";
  webpage += String(rtc.second());
  webpage += " - ";
  webpage += String(rtc.dayOfWeek());
  webpage += "</h4>";
  webpage += "<h4>Firmware Version: v";
  webpage += FirmwareVer;
  webpage += "</h4>";


  append_HTML_footer();
}


void append_HTML_header(bool refreshMode) {
  webpage  = "<!DOCTYPE html><html lang='en'>";
  webpage += "<head>";
  webpage += "<title>EzGeyzer</title>";
  webpage += "<meta charset='UTF-8'>";
  if (refreshMode) webpage += "<meta http-equiv='refresh' content='10'>"; // 5-secs refresh time, test needed to prevent auto updates repeating some commands
  webpage += "<script src=\"https://code.jquery.com/jquery-3.2.1.min.js\"></script>";
  webpage += "<style>";
  webpage += "body             {width:68em;margin-left:auto;margin-right:auto;font-family:Arial,Helvetica,sans-serif;font-size:14px;color:blue;background-color:#e1e1ff;text-align:center;}";
  webpage += ".centre          {margin-left:auto;margin-right:auto;}";
  webpage += "h2               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.4em;}";
  webpage += "h3               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.2em;}";
  webpage += "h4               {margin-top:0.3em;margin-bottom:0.3em;font-size:0.8em;}";
  webpage += ".on              {color: red;}";
  webpage += ".off             {color: limegreen;}";
  webpage += ".topnav          {overflow: hidden;background-color:lightcyan;}";
  webpage += ".topnav a        {float:left;color:blue;text-align:center;padding:1em 1.14em;text-decoration:none;font-size:1.3em;}";
  webpage += ".topnav a:hover  {background-color:deepskyblue;color:white;}";
  webpage += ".topnav a.active {background-color:lightblue;color:blue;}";
  webpage += "table tr, td     {padding:0.2em 0.5em 0.2em 0.5em;font-size:1.0em;font-family:Arial,Helvetica,sans-serif;}";
  webpage += "col:first-child  {background:lightcyan}col:nth-child(2){background:#CCC}col:nth-child(8){background:#CCC}";
  webpage += "tr:first-child   {background:lightcyan}";
  webpage += ".large           {font-size:1.8em;padding:0;margin:0}";
  webpage += ".medium          {font-size:1.4em;padding:0;margin:0}";
  webpage += ".ps              {font-size:0.7em;padding:0;margin:0}";
  webpage += "#outer           {width:100%;display:flex;justify-content:center;}";
  webpage += "footer           {padding:0.08em;background-color:cyan;font-size:1.1em;}";
  webpage += ".numberCircle    {border-radius:50%;width:2.7em;height:2.7em;border:0.11em solid blue;padding:0.2em;color:blue;text-align:center;font-size:3em;";
  webpage += "                  display:inline-flex;justify-content:center;align-items:center;}";
  webpage += ".wifi            {padding:3px;position:relative;top:1em;left:0.36em;}";
  webpage += ".wifi, .wifi:before {display:inline-block;border:9px double transparent;border-top-color:currentColor;border-radius:50%;}";
  webpage += ".wifi:before     {content:'';width:0;height:0;}";
  webpage += "</style></head>";
  webpage += "<body>";
  webpage += "<div class='topnav'>";
  webpage += "<a href='/'>Status</a>";
  //webpage += "<a href='graphs'>Graph</a>";
  webpage += "<a href='timer'>Schedule</a>";
  webpage += "<a href='setup'>WiFi</a>";
  webpage += "<a href='devicesetup'>Setup</a>";
  webpage += "<a href=''></a>";
  webpage += "<a href=''></a>";
  webpage += "<a href=''></a>";
  webpage += "<a href=''></a>";
  webpage += "<div class='wifi'/></div><span>";
  if (WiFi.status() == WL_CONNECTED)
    webpage += WiFiSignal();
  else
    webpage += "  WiFi Disconnected";
  webpage += "</span>";
  webpage += "</div><br>";
}
//#########################################################################################
void append_HTML_footer() {
  webpage += "<footer>";
  webpage += "<p class='medium'>Powered by KytherTek</p>";
  //webpage += "<p class='ps'><i>Copyright &copy;&nbsp;D L Bird " + String(Year) + " V" + version + "</i></p>";
  webpage += "</footer>";
  webpage += "</body></html>";
}
String WiFiSignal() {
  float Signal = WiFi.RSSI();
  Signal = 90 / 40.0 * Signal + 212.5; // From Signal = 100% @ -50dBm and Signal = 10% @ -90dBm and y = mx + c
  if (Signal > 100) Signal = 100;
  return " " + String(Signal, 0) + "%";
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////LCD FUNCTIONS////////////////////////////////////////////////////////////////////////////
//LCD function is initialized to display data on 16x2 LCD
void setup_16x2_lcd()
{
  Serial.println("Setting up 16x2 LCD");
  lcd.begin(16, 2);   //col,row
  lcd.display();      //turn on lcd
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("    EzGeyser    ");
}

void disp_temp_lcd(int ct, int st) {
  lcd.setCursor(0, 0);
  lcd.print("  TEMPERATURE:  ");
  lcd.setCursor(0, 1);
  lcd.print("NOW: ");

  if (ct >= 0 && ct <= 27)
    lcd.print("Cold(");
  else if (ct >= 28 && ct <= 32)
    lcd.print("Warm(");
  else if (ct >= 33 && ct <= 37)
    lcd.print("Warm+(");
  else if (ct >= 38 && ct <= 42)
    lcd.print("Hot(");
  else if (ct >= 43 && ct <= 50)
    lcd.print("Hot+(");
  else if (ct >= 51 && ct <= 60)
    lcd.print("Hot++(");
  else if (ct > 60)
    lcd.print("V Hot(");

  lcd.print(String(ct));
  lcd.print((char)223);
  lcd.print("C) ");

}

void disp_valve_lcd() {
  lcd.setCursor(0, 0);
  lcd.print("Geyser: " + vs + "    ");
}

void disp_flame_lcd() {
  lcd.setCursor(0, 1);
  lcd.print("Pilot:  " + fst + "    ");
}

void disp_buzzer_lcd()
{
  if (bs == "1")
  {
    ble_update_buzzerstatus();
    lcd.setCursor(0, 0);
    lcd.print("     ERROR:     ");
    lcd.setCursor(0, 1);
    lcd.print("     NO GAS     ");
  }
  else
  {
    if (vs == "ON")
    {
      lcd.setCursor(0, 0);
      lcd.print("STATUS:  HEATING");
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("STATUS: OFF     ");
    }

    lcd.setCursor(0, 1);
    lcd.print("TARGET:");
    if (st == 0)
    {
      lcd.print("OFF      ");
    }
    else if (st == 30)
    {
      lcd.print("Warm(30) ");
    }
    else if (st == 35)
    {
      lcd.print("Warm+(35)");
    }
    else if (st == 40)
    {
      lcd.print("Hot(40)  ");
    }
    else if (st == 45)
    {
      lcd.print("Hot+(45) ");
    }
    else if (st == 55)
    {
      lcd.print("Hot++(55)");
    }
    else if (st == 65)
    {
      lcd.print("V Hot(65)");
    }

  }
}

void disp_lcd(int ct, int st)
{
  if (deviceConnected)
  {
    if (lcdtimer == 10000)
      lcdtimer = millis();
    if (millis() - lcdtimer > 3000)
    {
      lcd.clear();
      a++;
      lcdtimer = 10000;
      if (a > 1)
        a = 0;
    }


    s5 = String(st).c_str();
    settemp_Char->setValue(s5);

    if (a == 0)
    {
      disp_temp_lcd(ct, st);
    }
    else if (a == 1)
    {
      disp_buzzer_lcd();
    }
    disc = 0;
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print("--DISCONNECTED--");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    if (disc == 0)
    {
      disc = 1;
      disconnectcounter++;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////// BLE FUNCTIONS ////////////////////////////////////////////////////////////////////////////
//Server Callback
// Generic BLE class used as it is
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(NimBLEServer* pServer) {
      deviceConnected = false;
      NimBLE_Advertising->start(0);
      Serial.println("start advertising on Disconnect");
      delay(500);
    }
};
//Characteristic Callback -
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pcharacteristic) {
      //when client transmits a value
    }
    void onRead(NimBLECharacteristic *pcharacteristic) {
      //when client requests a value
    }
};

// Setting up BLE server
void setup_BLE_server() {
  Serial.println("Setting up BLE server");
  //initialize BLE environment
  NimBLEDevice::init("BLEGEYZER");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_SCAN);
  //create server
  NimBLE_Server = NimBLEDevice::createServer();
  NimBLE_Server->setCallbacks(new ServerCallbacks());

  //create service
  NimBLE_Service1 = NimBLE_Server->createService(ble_service1_uuid);
  NimBLE_Service2 = NimBLE_Server->createService(ble_service2_uuid);

  //create characteristics
  gasreading_Char = NimBLE_Service1->createCharacteristic(gasreading_char_uuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  valvestatus_Char = NimBLE_Service2->createCharacteristic(valvestatus_char_uuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  flamestatus_Char = NimBLE_Service2->createCharacteristic(flamestatus_char_uuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  currenttemp_Char = NimBLE_Service1->createCharacteristic(currenttemp_char_uuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  settemp_Char = NimBLE_Service1->createCharacteristic(settemp_char_uuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  buzzerstatus_Char = NimBLE_Service2->createCharacteristic(buzzerstatus_char_uuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

  //create callbacks
  gasreading_Char->setCallbacks(new CharacteristicCallbacks());
  valvestatus_Char->setCallbacks(new CharacteristicCallbacks());
  flamestatus_Char->setCallbacks(new CharacteristicCallbacks());
  currenttemp_Char->setCallbacks(new CharacteristicCallbacks());
  settemp_Char->setCallbacks(new CharacteristicCallbacks());
  buzzerstatus_Char->setCallbacks(new CharacteristicCallbacks());

  //start the service
  NimBLE_Service1->start();
  NimBLE_Service2->start();

  //create advertisement
  //BLE_Advertising = BLE_Server->getAdvertising();    // this still is working for backward compatibility
  NimBLE_Advertising = NimBLEDevice::getAdvertising();
  NimBLE_Advertising->addServiceUUID(ble_service1_uuid);
  //Serial.println(BLE_Service1->getUUID().toString().c_str());
  NimBLE_Advertising->addServiceUUID(ble_service2_uuid);
  //Serial.println(BLE_Service2->getUUID().toString().c_str());
  NimBLE_Advertising->setScanResponse(true);
  NimBLE_Advertising->setMinPreferred(0x06);          // functions that help with iPhone connections issue
  NimBLE_Advertising->setMinPreferred(0x12);
  NimBLE_Advertising->start(0);
  //BLEDevice::startAdvertising();
  Serial.println("BLE Server Service/s and Characteristic/s INITIALIZED");
  Serial.println("Waiting for client device to notify...");
}
/////////////// These fucntion will get the values of gas,valve,currenttemp,settemp and flame from outdoor unit and will store the values in BLE characteristics variables.
void ble_update_gasreading() {
  s1 = gasreading_Char->getValue();
  if (s1.length() > 0) {
    //Serial.print("Gas Reading:   ");
    //for (int i = 0; i < s1.length(); i++)
    //Serial.print(s1[i]);
    gasreading_Char->setValue(s1);
    gr = String(s1.c_str());
    //gasreading_Char->notify();
    //Serial.print("\t\tBLE Value updated - ");
    //delay(20);  //minimum should be 10ms
  }
}
void ble_update_valvestatus() {
  s2 = valvestatus_Char->getValue();
  if (s2.length() > 0) {
    //Serial.print("\tValve Status:  ");
    //for (int i = 0; i < s2.length(); i++)
    //Serial.print(s2[i]);
    valvestatus_Char->setValue(s2);
    vs = String(s2.c_str());
    //valvestatus_Char->notify();
    //Serial.print("\t\tBLE Value updated - ");
    //delay(20);  //minimum should be 10ms
  }
}
void ble_update_flamestatus() {
  s3 = flamestatus_Char->getValue();
  if (s3.length() > 0) {
    //Serial.print("\tFlame Status:  ");
    //for (int i = 0; i < s3.length(); i++)
    //Serial.print(s3[i]);
    flamestatus_Char->setValue(s3);
    fst = String(s3.c_str());
    //     fst = flamestatus_Char->getValue();
    //flamestatus_Char->notify();
    //Serial.print("\t\tBLE Value updated - ");
    //Serial.println();
    //delay(20);  //minimum should be 10ms
  }
}
int ble_update_currenttemp() {
  s4 = currenttemp_Char->getValue();
  if (s4.length() > 0) {
    //Serial.print("Current Temp:  ");
    //for (int i = 0; i < s4.length(); i++)
    //Serial.print(s4[i]);
    currenttemp_Char->setValue(s4);
    //currenttemp_Char->notify();
    //Serial.print("\t\tBLE Value updated - ");
    //delay(20);  //minimum should be 10ms

    //convert to int to show on display LCD
    ct = String(s4.c_str()).toInt();
  }
  return ct;
}
void ble_update_buzzerstatus() {
  s6 = buzzerstatus_Char->getValue();
  if (s6.length() > 0) {
    //Serial.print("\tBuzzer Status: ");
    //for (int i = 0; i < s6.length(); i++)
    //Serial.print(s6[i]);
    buzzerstatus_Char->setValue(s6);
    bs = String(s6.c_str()).toInt();
    //buzzerstatus_Char->notify();
    //Serial.print("\t\tBLE Value updated - ");
    // delay(20);  //minimum should be 10ms
  }
}
void ble_update_settemp(int st) {

  s5 = String(st).c_str();
  settemp_Char->setValue(s5);
  // }
  settemp_Char->notify();
  //Serial.print("\tSet temp:      ");
  //Serial.println(s5.c_str());
  //settemp = rxpayload.c_str();
  //Serial.println();
  //Serial.print("\t\tBLE Value updated - ");
  //Serial.println("Topic 5 SUBed...");
  //delay(20);  //minimum should be 10ms

  //convert to int to show on display TM1637
  //st = String(s5.c_str()).toInt();
  //return st;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////RTC Functions/////////////////////////////////////////////////////////////////////////////////////////////////////
void setRTC() {

  Serial.println("Setting RTC");
  const long  gmtOffset_sec = 18000;
  const char* ntpServer = "pool.ntp.org";
  const int   daylightOffset_sec = 0;
  //
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }
  else
  {
    Serial.println("Server connected");
    year = timeinfo.tm_year - 100;
    month = timeinfo.tm_mon + 1;
    day = timeinfo.tm_mday;
    hour = timeinfo.tm_hour;
    minute = timeinfo.tm_min;
    sec = timeinfo.tm_sec;
    rtcdow = timeinfo.tm_wday;

    Serial.println(year);
    Serial.println(month);
    Serial.println(day);
    Serial.println(hour);
    Serial.println(minute);
    Serial.println(rtc.second());
    Serial.println(rtcdow);

    if (!(year == 0))
    {
      rtc.set(sec, minute, hour, rtcdow, day, month, year);
      Serial.println("RTC updated");
    }
  }
}

// RTC library is used as it is so no editing is required here
void setup_rtc() {
  rtc.set_rtc_address(0x68);
  Wire.begin(rtc_sda, rtc_scl);
  Serial.println("Setting RTC");
}

void show_datetime() {
  rtc.refresh();
  Serial.print("(D/M/Y) ");
  Serial.print(rtc.day());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.year());

  Serial.print("\t (H:M:S) ");
  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.print(rtc.second());

  Serial.print("\t DOW: ");
  Serial.println(rtc.dayOfWeek());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void repeatedCall() {
  Serial.print(" Active fw version:");
  Serial.println(FirmwareVer);
  if (FirmwareVersionCheck() && WiFi.status() == WL_CONNECTED) {
    firmwareUpdate();
    return;
  }
}
int FirmwareVersionCheck(void) {
  String payload;
  int httpCode;
  String fwurl = URL_fw_Version;
  Serial.println(fwurl);

  WiFiClient *tempclient = new WiFiClient;

  if (tempclient)
  {
    HTTPClient https;

    if (https.begin( * tempclient, fwurl))
    { // HTTPS
      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK) // if version received
      {
        payload = https.getString(); // save received version
      } else {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();
    }
    delete tempclient;
  }

  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    if (payload.equals(FirmwareVer)) {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);
      return 0;
    }
    else
    {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  }
  return 0;
}
void firmwareUpdate(void) {
  wdt_disable();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SOFTWARE  UPDATE");
  WiFiClient fwclient;

  t_httpUpdate_return ret = httpUpdate.update(fwclient, URL_fw_Bin);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////// LCD MENU ////////////////////////////////////////////////////////////////////////////
void menu()
{
  drawMenu();
  rotary();
  if (up && page == 1)
  {
    up = false;

    if (tempst == 0) tempst = 0;
    else if (tempst == 30) tempst = 0;
    else if (tempst == 35) tempst = 30;
    else if (tempst == 40) tempst = 35;
    else if (tempst == 45) tempst = 40;
    else if (tempst == 55) tempst = 45;
    else if (tempst == 65) tempst = 55;

    if (tempst < 0) tempst = 0;
    if (tempst > 65) tempst = 65;
    overridetimer = 30;
  }

  if (up && page == 2)
  {
    up = false;
    overridetimer -= 30;
    if (overridetimer < 30)
      overridetimer = 30;
    if (overridetimer > 360)
      overridetimer = 360;
  }

  if (down && page == 1)
  {
    down = false;

    if (tempst == 0) tempst = 30;
    else if (tempst == 30) tempst = 35;
    else if (tempst == 35) tempst = 40;
    else if (tempst == 40) tempst = 45;
    else if (tempst == 45) tempst = 55;
    else if (tempst == 55) tempst = 65;
    else if (tempst == 65) tempst = 65;

    if (tempst < 0) tempst = 0;
    if (tempst > 65) tempst = 65;

  }

  if (down && page == 2)
  {
    down = false;
    overridetimer += 30;
    if (overridetimer < 30)
      overridetimer = 30;
    if (overridetimer > 360)
      overridetimer = 360;

  }


  if (middle) //Middle Button is Pressed
  {
    middle = false;

    if (page == 1)
    {
      lcd.clear();
      page = 2;
    }

    else if (page == 2)
    {
      st = tempst;
      schon = 0;
      schoverride = 1;
      overridestarttime = millis();

      if (mqttClient.connected())
      {
        packetIdUnsub = mqttClient.unsubscribe(topic5);
        delay(100);
        s5 = String(st).c_str();
        packetIdPub = mqttClient.publish(topic5, 1, false, String(s5.c_str()).c_str());
        packetIdSub = mqttClient.subscribe(topic5, 1);
        delay(100);
      }


      lcd.clear();
      menuactive = 0;
      page = 1;
      menuitem = 1;
      schmenuitem = 0;
    }
  }



  //
  //    if (page == 1 || page == 2)
  //      lcd.clear();
  //
  //    //if (page == 1 && menuitem == 4) // go back
  //    if (page == 1 && menuitem == 3) // go back
  //    {
  //      lcd.clear();
  //      menuactive = 0;
  //      page = 1;
  //      menuitem = 1;
  //      schmenuitem = 0;
  //    }
  //
  //
  //    //else if (page == 1 && menuitem <= 4) {
  //    else if (page == 1 && menuitem <= 3) {
  //      page = 2;
  //    }
  //    else if (page == 2 && menuitem == 2)
  //    {
  //      if (TimerState == "ON")
  //      {
  //        schoverride = 1;
  //        TimerState = "OFF";
  //      }
  //      EEPROM.write(97, st);
  //      EEPROM.commit();
  //
  //
  //      if (mqttClient.connected())
  //      {
  //        packetIdUnsub = mqttClient.unsubscribe(topic5);
  //        delay(100);
  //        s5 = String(st).c_str();
  //        packetIdPub = mqttClient.publish(topic5, 1, false, String(s5.c_str()).c_str());
  //        packetIdSub = mqttClient.subscribe(topic5, 1);
  //        delay(100);
  //      }
  //
  //
  //      lcd.clear();
  //      menuactive = 0;
  //      page = 1;
  //      menuitem = 1;
  //      schmenuitem = 0;
  //    }
  //
  //
  //    //    else if (page == 2 && menuitem == 3)
  //    //    {
  //    //      page = 3;
  //    //    }
  //
  //    //    else if (page == 2 && menuitem == 4)
  //    //    {
  //    //
  //    //    }
  //    //    else if (page == 3)
  //    //    {
  //    //      page = 4;
  //    //      schtimer = 10000;
  //    //    }
  //    //    else if (page == 4)
  //    //    {
  //    //      page = 5;
  //    //      schtimer = 10000;
  //    //    }
  //    //    else if (page == 5)
  //    //    {
  //    //      page = 6;
  //    //      schtimer = 10000;
  //    //    }
  //    //    else if (page == 6)
  //    //    {
  //    //      page = 7;
  //    //      schtimer = 10000;
  //    //    }
  //    //
  //    //    else if (page == 7)
  //    //    {
  //    //      String str = {""};      //Schedule array
  //    //      for (int i = 0; i < 1; i++) //NO. OF SCHEDULES IN A DAY
  //    //      {
  //    //        if (onhr[i] < 10)
  //    //          str = {str + "0" + String(onhr[i])};
  //    //        else
  //    //          str = String(str + onhr[i]);
  //    //
  //    //        if (onmin[i] < 10)
  //    //          str = {str + "0" + String(onmin[i])};
  //    //        else
  //    //          str = {str + String(onmin[i])};
  //    //
  //    //        if (offhr[i] < 10)
  //    //          str = {str + "0" + String(offhr[i])};
  //    //        else
  //    //          str = String(str + offhr[i]);
  //    //
  //    //        if (offmin[i] < 10)
  //    //          str = {str + "0" + String(offmin[i])};
  //    //        else
  //    //          str = {str + String(offmin[i])};
  //    //
  //    //        if (settemp[i] < 10)
  //    //          str = {str + "0" + String(settemp[i])};
  //    //        else
  //    //          str = {str + String(settemp[i])};
  //    //      }
  //    //      //str = {str + ":" + str + ":" + str + ":" + str + ":" + str + ":" + str + ":" + str};
  //    //
  //    //      page = 1;
  //    //      //menuitem = 1;
  //    //      schmenuitem = 0;
  //    //    }
  //    else if (page == 2 && menuitem != 3)
  //    {
  //      page = 1;
  //    }
  //  }
}
void drawMenu()
{
  if (page == 1)
  {
    lcd.setCursor(0, 0);
    lcd.print("Set Temperature");
    lcd.setCursor(0, 1);

    if (tempst == 0) {
      lcd.print("       OFF      ");
    }
    else if (tempst == 30) {
      lcd.print("  WARM (30");
      lcd.print((char)223);
      lcd.print("C)   ");
    }
    else if (tempst == 35) {
      lcd.print("  WARM+ (35");
      lcd.print((char)223);
      lcd.print("C)  ");
    }
    else if (tempst == 40) {
      lcd.print("   HOT (40");
      lcd.print((char)223);
      lcd.print("C)   ");
    }
    else if (tempst == 45) {
      lcd.print("  HOT+ (45");
      lcd.print((char)223);
      lcd.print("C)   ");
    }
    else if (tempst == 55) {
      lcd.print("  HOT++ (55");
      lcd.print((char)223);
      lcd.print("C)  ");
    }
    else if (tempst == 65) {
      lcd.print("VERY HOT (65");
      lcd.print((char)223);
      lcd.print("C) ");
    }
  }

  else if (page == 2)
  {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Timer: ");

    if (overridetimer == 30)
    {
      lcd.print("30mins   ");
    }
    else if (overridetimer == 60)
    {
      lcd.print("1hr      ");
    }
    else if (overridetimer == 90)
    {
      lcd.print("1hr 30min");
    }
    else if (overridetimer == 120)
    {
      lcd.print("2hr      ");
    }
    else if (overridetimer == 150)
    {
      lcd.print("2hr 30min");
    }
    else if (overridetimer == 180)
    {
      lcd.print("3hr      ");
    }
    else if (overridetimer == 210)
    {
      lcd.print("3hr 30min");
    }
    else if (overridetimer == 240)
    {
      lcd.print("4hr      ");
    }
    else if (overridetimer == 270)
    {
      lcd.print("4hr 30min");
    }
    else if (overridetimer == 300)
    {
      lcd.print("5hr      ");
    }
    else if (overridetimer == 330)
    {
      lcd.print("5hr 30min");
    }
    else if (overridetimer == 360)
    {
      lcd.print("6hr      ");
    }
  }
  //  if (page == 1)
  //  {
  //    if (menuitem == 1)
  //    {
  //      //lcd.clear();
  //      lcd.setCursor(0, 0);
  //      if (st > 0)
  //        lcd.print("> Turn OFF       ");
  //      else
  //        lcd.print("> Turn ON       ");
  //      lcd.setCursor(0, 1);
  //      lcd.print("  Set Temp      ");
  //    }
  //    else if (menuitem == 2)
  //    {
  //      //lcd.clear();
  //      lcd.setCursor(0, 0);
  //      if (st > 0)
  //        lcd.print("  Turn OFF       ");
  //      else
  //        lcd.print("  Turn ON       ");
  //      lcd.setCursor(0, 1);
  //      lcd.print("> Set Temp      ");
  //    }
  //    else if (menuitem == 3)
  //    {
  //      //lcd.clear();
  //      lcd.setCursor(0, 0);
  //      lcd.print("> Back          ");
  //      lcd.setCursor(0, 1);
  //      lcd.print("                ");
  //    }
  //    //    else if (menuitem == 4)
  //    //    {
  //    //      //lcd.clear();
  //    //      lcd.setCursor(0, 0);
  //    //      lcd.print("  Set Schedule  ");
  //    //      lcd.setCursor(0, 1);
  //    //      lcd.print("> Back          ");
  //    //
  //    //    }
  //
  //  }
  //  else if (page == 2 && menuitem == 1)
  //  {
  //    lcd.clear();
  //    if (st > 0)
  //      st = 0;
  //    else
  //      st = schdefaulttemp;
  //    //    lcd.setCursor(0, 0);
  //    //    lcd.print("Turned on");
  //    //    delay(1000);
  //    //    lcd.clear();
  //
  //
  //    lcd.clear();
  //    menuactive = 0;
  //    page = 1;
  //    menuitem = 1;
  //    schmenuitem = 0;
  //    //frame = 1;
  //  }
  //
  //  else if (page == 2 && menuitem == 2)
  //  {
  //    lcd.setCursor(0, 0);
  //    lcd.print("Set Temperature");
  //    lcd.setCursor(0, 1);
  //
  //    if (st == 0) {
  //      lcd.print("       OFF      ");
  //    }
  //    else if (st == 30) {
  //      lcd.print("  WARM (30");
  //      lcd.print((char)223);
  //      lcd.print("C)   ");
  //    }
  //    else if (st == 35) {
  //      lcd.print("  WARM+ (35");
  //      lcd.print((char)223);
  //      lcd.print("C)  ");
  //    }
  //    else if (st == 40) {
  //      lcd.print("   HOT (40");
  //      lcd.print((char)223);
  //      lcd.print("C)   ");
  //    }
  //    else if (st == 45) {
  //      lcd.print("  HOT+ (45");
  //      lcd.print((char)223);
  //      lcd.print("C)   ");
  //    }
  //    else if (st == 55) {
  //      lcd.print("  HOT++ (55");
  //      lcd.print((char)223);
  //      lcd.print("C)  ");
  //    }
  //    else if (st == 65) {
  //      lcd.print("VERY HOT (65");
  //      lcd.print((char)223);
  //      lcd.print("C) ");
  //    }
  //  else if (page == 2 && menuitem == 3)
  //  {
  //    if (schmenuitem == 0)
  //    {
  //      //lcd.clear();
  //      lcd.setCursor(0, 0);
  //      lcd.print("> Schedule 1    ");
  //      lcd.setCursor(0, 1);
  //      lcd.print("  Schedule 2    ");
  //    }
  //    else if (schmenuitem == 1)
  //    {
  //      lcd.setCursor(0, 0);
  //      lcd.print("  Schedule 1    ");
  //      lcd.setCursor(0, 1);
  //      lcd.print("> Schedule 2    ");
  //    }
  //    else if (schmenuitem == 2)
  //    {
  //      lcd.setCursor(0, 0);
  //      lcd.print("> Schedule 3    ");
  //      lcd.setCursor(0, 1);
  //      lcd.print("  Schedule 4    ");
  //    }
  //    else if (schmenuitem == 3)
  //    {
  //      lcd.setCursor(0, 0);
  //      lcd.print("  Schedule 3    ");
  //      lcd.setCursor(0, 1);
  //      lcd.print("> Schedule 4    ");
  //    }
  //  }
  //  else if (page == 2 && menuitem == 4)
  //  {
  //  }

  //  else if (page == 3)
  //  {
  //    lcd.setCursor(0, 0);
  //    lcd.print("ON Time:        ");
  //    if (schtimer == 10000)
  //      schtimer = millis();
  //    if (millis() - schtimer > 500)
  //    {
  //      blinkk = !blinkk;
  //      //Serial.println(millis() - schtimer);
  //      schtimer = 10000;
  //    }
  //
  //    if (blinkk)
  //    {
  //      lcd.setCursor(10, 0);
  //      if (onhr[schmenuitem] < 10)
  //        lcd.print("0");
  //      lcd.print(String(onhr[schmenuitem]));
  //    }
  //    else
  //    {
  //      lcd.setCursor(10, 0);
  //      lcd.print("  ");
  //    }
  //
  //    lcd.setCursor(12, 0);
  //    lcd.print(":");
  //    if (onmin[schmenuitem] < 10)
  //      lcd.print("0");
  //    lcd.print(String(onmin[schmenuitem]));
  //
  //  }
  //  else if (page == 4)
  //  {
  //    lcd.setCursor(0, 0);
  //    lcd.print("ON Time:        ");
  //    lcd.setCursor(10, 0);
  //    if (onhr[schmenuitem] < 10)
  //      lcd.print("0");
  //    lcd.print(String(onhr[schmenuitem]));
  //    lcd.print(":");
  //
  //    if (schtimer == 10000)
  //      schtimer = millis();
  //    if (millis() - schtimer > 500)
  //    {
  //      blinkk = !blinkk;
  //      //Serial.println(millis() - schtimer);
  //      schtimer = 10000;
  //    }
  //    if (blinkk)
  //    {
  //      lcd.setCursor(13, 0);
  //      if (onmin[schmenuitem] < 10)
  //        lcd.print("0");
  //      lcd.print(String(onmin[schmenuitem]));
  //    }
  //    else
  //    {
  //      lcd.setCursor(13, 0);
  //      lcd.print("  ");
  //    }
  //  }
  //
  //  else if (page == 5)
  //  {
  //    lcd.setCursor(0, 0);
  //    lcd.print("OFF Time:       ");
  //    if (schtimer == 10000)
  //      schtimer = millis();
  //    if (millis() - schtimer > 500)
  //    {
  //      blinkk = !blinkk;
  //      //Serial.println(millis() - schtimer);
  //      schtimer = 10000;
  //    }
  //
  //    if (blinkk)
  //    {
  //      lcd.setCursor(10, 0);
  //      if (offhr[schmenuitem] < 10)
  //        lcd.print("0");
  //      lcd.print(String(offhr[schmenuitem]));
  //    }
  //    else
  //    {
  //      lcd.setCursor(10, 0);
  //      lcd.print("  ");
  //    }
  //
  //    lcd.setCursor(12, 0);
  //    lcd.print(":");
  //    if (offmin[schmenuitem] < 10)
  //      lcd.print("0");
  //    lcd.print(String(offmin[schmenuitem]));
  //
  //  }
  //  else if (page == 6)
  //  {
  //    lcd.setCursor(0, 0);
  //    lcd.print("OFF Time:       ");
  //    lcd.setCursor(10, 0);
  //    if (offhr[schmenuitem] < 10)
  //      lcd.print("0");
  //    lcd.print(String(offhr[schmenuitem]));
  //    lcd.print(":");
  //
  //    if (schtimer == 10000)
  //      schtimer = millis();
  //    if (millis() - schtimer > 500)
  //    {
  //      blinkk = !blinkk;
  //      //Serial.println(millis() - schtimer);
  //      schtimer = 10000;
  //    }
  //    if (blinkk)
  //    {
  //      lcd.setCursor(13, 0);
  //      if (offmin[schmenuitem] < 10)
  //        lcd.print("0");
  //      lcd.print(String(offmin[schmenuitem]));
  //    }
  //    else
  //    {
  //      lcd.setCursor(13, 0);
  //      lcd.print("  ");
  //    }
  //  }
  //  else if (page == 7)
  //  {
  //    lcd.setCursor(0, 0);
  //    lcd.print("Temp: " + String(settemp[schmenuitem]) + "             ");
  //  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////// ROTARY FUNCTIONS ////////////////////////////////////////////////////////////////////////////
void rotary() {
  currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK )
  {
    static unsigned long lastInterrupt = 0;
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterrupt > 300)
    {
      // Serial.println("change");
      if (digitalRead(DT) != currentStateCLK) {
        // value ++;
        down = true;
        // Serial.println("enc+");
        //delay(250);
        lastmenuactive = millis();

      } else {
        // Encoder is rotating CW so increment
        // value --;
        up = true;
        //  Serial.println("enc--");
        lastmenuactive = millis();
      }
    }
    lastStateCLK = currentStateCLK;
  }
}

int checkButton()
{
  int event = 0;
  buttonVal = digitalRead(SW);
  // Button pressed down
  if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
  {
    downTime = millis();
    ignoreUp = false;
    waitForUp = false;
    singleOK = true;
    holdEventPast = false;
    longHoldEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
    else  DConUp = false;
    DCwaiting = false;
  }
  // Button released
  else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce)
  {
    if (not ignoreUp)
    {
      upTime = millis();
      if (DConUp == false) DCwaiting = true;
      else
      {
        //        Serial.println("double click");
        //        event = 2;
        DConUp = false;
        DCwaiting = false;
        singleOK = false;
      }
    }
  }
  // Test for normal click event: DCgap expired
  if ( buttonVal == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
  {
    lastmenuactive = millis();
    Serial.println("click");
    event = 1;
    DCwaiting = false;
  }
  // Test for hold
  if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
    // Trigger "normal" hold
    if (not holdEventPast)
    {
      lastmenuactive = millis();
      Serial.println("Hold");
      event = 2;
      waitForUp = true;
      ignoreUp = true;
      DConUp = false;
      DCwaiting = false;
      //downTime = millis();
      holdEventPast = true;
      delay(500);
    }
    // Trigger "long" hold
    //    if ((millis() - downTime) >= longHoldTime)
    //    {
    //      if (not longHoldEventPast)
    //      {
    //        Serial.println("long hold");
    //        event = 4;
    //        longHoldEventPast = true;
    //      }
    //    }
  }
  buttonLast = buttonVal;
  return event;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
  EEPROM.begin(512); //Initialasing EEPROM
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  lastStateCLK = digitalRead(CLK);
  Serial.begin(115200);
  setup_16x2_lcd();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);



  Serial.println("========== BOOTING DEVICE ==========");

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  setup_BLE_server();
  setup_mqtt_topics();
  setup_rtc();
  WiFi.macAddress(mac2);
  snprintf(mac_Id, sizeof(mac_Id), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]);
  Serial.print(mac_Id);
  setup_wifi();

  Serial.println("RTC INITIALIZED");

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setKeepAlive(120);
  mqttClient.setClientId(mac_Id);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  //====================================================================================================================

  StartSPIFFS();                          // Start SPIFFS filing system
  Initialise_Array();                     // Initialise the array for storage and set some values
  RecoverSettings();                      // Recover settings from LittleFS
  SetupDeviceName("ezgeyser");            // Set logical device name

  schdefaulttemp = EEPROM.read(96);
  if (schdefaulttemp > 65)
    schdefaulttemp = 35;
  if (schdefaulttemp < 0)
    schdefaulttemp = 35;
  st = schdefaulttemp;

  Serial.println("=========== BOOTING DONE ===========");
  //lcd.begin(16, 2);
  Serial.print(" FREE HEAP after setup: ");
  Serial.println(ESP.getFreeHeap());

  lcd.setCursor(0, 1);
  lcd.print("   BOOTING UP   ");
  NimBLE_Advertising->start(0);
  delay(5000);
  wdt_enable(8000);
  if ((WiFi.status() == WL_CONNECTED))
  {
    repeatedCall();
    lastupdatecheck = millis();
  }
}

void loop()
{
  // BLE disconnecting
  if (st <= 0) st = 0;
  else if (st > 0 && st <= 30) st = 30;
  else if (st > 30 && st <= 35) st = 35;
  else if (st > 35 && st <= 40) st = 40;
  else if (st > 40 && st <= 45) st = 45;
  else if (st > 45 && st <= 55) st = 55;
  else if (st > 55 && st <= 65) st = 65;
  else if (st > 65) st = 65;

  if (schoverride)
  {
    if ((millis() - overridestarttime) > (overridetimer * 1000))
    {
      Serial.println("Turning Override off");
      schoverride = 0;
      st = schdefaulttemp;
    }
    else
    {
      Serial.print("OVERRIDE ON. RETURNING TO DEFAULT TEMP, IN: ");
      int timeremaining = ((overridetimer * 1000) - (millis() - overridestarttime)) / 1000;
      Serial.print(timeremaining);
      Serial.println("secs");
    }
  }
  if (deviceConnected)
  {
    ble_update_valvestatus();
    ble_update_flamestatus();
    ct = ble_update_currenttemp();
    ble_update_buzzerstatus();
    ble_update_settemp(st);


    CheckTimerEvent();

    if (millis() - mqttlastsent > 10000)
    {
      if (!sstart)
      {
        delaytimer = millis();
        sstart = 1;
      }
      if ((WiFi.status() == WL_CONNECTED))
      {
        if (mqttClient.connected())
        {
          if (sstart)
          {
            if (mqttdebug == 1)
            {
              if (millis() - delaytimer > 8000)
              {
                packetIdPub = mqttClient.publish(topic15, 1, false, TimerState.c_str()); //schstatus
                mqttlastsent = millis();
                sstart = 0;
                sent7 = 0;
                sent6 = 0;
                sent5 = 0;
                sent4 = 0;
                sent3 = 0;
                sent2 = 0;
                sent1 = 0;
                Serial.print("FREE HEAP: ");
                Serial.println(ESP.getFreeHeap());
              }
              else if (millis() - delaytimer > 7000 && !sent7)
              {
                packetIdPub = mqttClient.publish(topic14, 1, false, String(schoverride).c_str());
                sent7 = 1;
              }
              else if (millis() - delaytimer > 6000 && !sent6)
              {
                packetIdPub = mqttClient.publish(topic13, 1, false, String(disconnectcounter).c_str()); //disconnectcounter
                sent6 = 1;
              }
              else if (millis() - delaytimer > 5000 && !sent5)
              {
                packetIdPub = mqttClient.publish(topic12, 1, false, FirmwareVer.c_str()); //fwver
                sent5 = 1;
              }
              else if (millis() - delaytimer > 4000 && !sent4)
              {
                packetIdPub = mqttClient.publish(topic2, 1, false, String(s2.c_str()).c_str());
                sent4 = 1;
              }

              else if (millis() - delaytimer > 3000 && !sent3)
              {
                packetIdPub = mqttClient.publish(topic3, 1, false, String(s3.c_str()).c_str());
                sent3 = 1;
              }

              else if (millis() - delaytimer > 2000 && !sent2)
              {
                packetIdPub = mqttClient.publish(topic4, 1, false, String(s4.c_str()).c_str());
                sent2 = 1;
              }
              else if (millis() - delaytimer > 1000 && !sent1)
              {
                Serial.println("TIME TO PUBLISH - DEBUG MODE");
                packetIdPub = mqttClient.publish(topic6, 1, false, String(s6.c_str()).c_str());
                sent1 = 1;
              }
            }
            else
            {
              if (millis() - delaytimer > 4000)
              {
                packetIdPub = mqttClient.publish(topic2, 1, false, String(s2.c_str()).c_str());
                mqttlastsent = millis();
                sstart = 0;
                sent3 = 0;
                sent2 = 0;
                sent1 = 0;
                Serial.print("FREE HEAP: ");
                Serial.println(ESP.getFreeHeap());
              }

              else if (millis() - delaytimer > 3000 && !sent3)
              {
                packetIdPub = mqttClient.publish(topic3, 1, false, String(s3.c_str()).c_str());
                sent3 = 1;
              }

              else if (millis() - delaytimer > 2000 && !sent2)
              {
                packetIdPub = mqttClient.publish(topic4, 1, false, String(s4.c_str()).c_str());
                sent2 = 1;
              }
              else if (millis() - delaytimer > 1000 && !sent1)
              {
                Serial.println("TIME TO PUBLISH");
                packetIdPub = mqttClient.publish(topic6, 1, false, String(s6.c_str()).c_str());
                sent1 = 1;
              }
            }
          }
        }
      }
    }
  }
  else
  {
    if (!NimBLE_Advertising->isAdvertising())
    {
      NimBLE_Advertising->start(0);
      Serial.println("start advertising in loop");
      delay(500);
    }
  }

  if (millis() - lastupdatecheck > 600000)
  {
    if ((WiFi.status() == WL_CONNECTED))
    {
      repeatedCall();
      lastupdatecheck = millis();
    }
  }

  byte r = checkButton();
  if (r == 2 && !menuactive)
  {
    menuactive = 1;
  }
  else if (r == 1)
    middle = 1;
  if (menuactive)
    menu();
  else
    disp_lcd(ct, st);

  if (menuactive && millis() - lastmenuactive > 15000)
  {
    menuactive = 0;
  }
  wdt_reset();
}
