/* 
 ESP32 Dev board with a nRF24L01+ connected on pins

 /!\ YOU DO NEED A HIGH-AMP USB connection, otherwise WiFi will not work

CE   -> GPIO21
CSN  -> GPIO05
MISO -> GPIO19
MOSI -> GPIO23
CLK  -> GPIO18
IRQ  -> unconnected

 This node implements an MQTT client, an RF24 master, and a bluetooth
 scanner. The goal is to be able to control other rf24 modules and 
 watch for bluetooth devices. All this is relayed then via MQTT to 
 the server (jarvis?). 
 The MQTT server probably will have an application or interface to
 deal with this information.

 WiFi and MQTT information is stored on the board flash. When flashing
 onto new boards, the first time we use SPIFFS require formatting with
 SPIFS.begin(true) check below in setup_spiffs()
 
*/
#include <rom/rtc.h> // not sure why i need this ?
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "FS.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <TimeLib.h>

#define useCredentialsFile
#ifdef useCredentialsFile
#include "credentials.h"
#else
mySSID = "    ";
myPASSWORD = "   ";
#endif

//
// Physical connections 
//
#define HW_CSN    5    // icsp
#define HW_CE    21    // icsp
// 
// SW Logic and firmware definitions
// 
#define       UNIQUE_ID           0        // master is 0, unoR3 debugger is 1, promicro_arrosoir is 2, etc
#define       uS_TO_S_FACTOR      1000000  // Conversion factor for micro seconds to seconds 
#define       MOD                 "esp32/"
#define       CAT                 "feeds/"
#define       T_SWITCH1           "switch1"
#define       T_SWITCH2           "switch2"
#define       T_SWITCH3           "switch3"
#define       T_SWITCH4           "switch4"
#define       T_SCHEDULE1         "sched1"
#define       T_SCHEDULE2         "sched2"
#define       T_SCHEDULE3         "sched3"
#define       T_SCHEDULE4         "sched4"
#define       T_DURATION1         "dur1"
#define       T_DURATION2         "dur2"
#define       T_DURATION3         "dur3"
#define       T_DURATION4         "dur4"
#define       T_TEMP              "temp"
#define       T_COMMAND           "provideStatus"
#define       T_CURSTATUS         "currentStatus"
#define       THIS_NODE_ID        32   // master is 0, unoR3 debugger is 1, promicro_arrosoir is 2, etc
#define       DEFAULT_ACTIVATION  600  // 10h from now we activate (in case radio is down and can't program)
#define       DEFAULT_DURATION    10   // max 10s of activation time by default
#define       MAX_NODES           5

// REMINDER!! 3Node and 4Node are used by testing sketches ping/pong
const uint8_t addresses[][6] = {
                "0Node", // master writes broadcasts here
                "1Node", // unor3 writes here
                "2Node", // unor3 reads here
                "3Node", // arrosoir reads here
                "4Node", // arrosoir writes here
                "5Node"};// not yet used by anybody

struct relayctl {
  uint32_t uptime = 0;                      // current running time of the machine (millis())  4 bytes  
  uint32_t sched1 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr1   4 bytes
  uint32_t sched2 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr2   4 bytes
  uint16_t maxdur1 = DEFAULT_DURATION;      // max duration nbr1 is ON                         2 bytes
  uint16_t maxdur2 = DEFAULT_DURATION;      // max duration nbr2 is ON                         2 bytes
   int8_t  temp_thres = 99;                 // temperature at which the syatem is operational  1 byte
   int8_t  temp_now   = 20;                 // current temperature read on the sensor          1 byte
  uint8_t  battery    =  0;                 // current temperature read on the sensor          1 byte
  bool     state1 = false;                  // state of relay output 1                         1 byte
  bool     state2 = false;                  // "" 2                                            1 byte
  bool     waterlow = false;                // indicates whether water is low                  1 byte
  uint8_t  nodeid = 32;                     // nodeid is the identifier of the slave           1 byte
} __attribute__ ((packed)) myData;

relayctl              rfNodes[MAX_NODES];
short                 m_nGotNodes = 0;
const static char     m_sHR[]  = "- - - - -";
uint64_t              chipid;  
String                g_nwSSID = "", 
                      g_nwPASS = "", 
                      g_nwMQTT = "192.168.1.109",
                      g_tgCHAT = "60001082";
long                  lastMsg = 0;
char                  msg[50];
int                   value = 0;
int                   g_Bot_mtbs = 5000;    //mean time between scan messages
long                  g_Bot_lasttime = 0;   //last time messages' scan has been done
const uint16_t        TIMESTAMPREQ = 4681;
WiFiClient            espClient;
PubSubClient          mqttClient(espClient);
WiFiClientSecure      botClient;
UniversalTelegramBot  bot(BOTtoken, botClient);
RF24                  radio(HW_CE, HW_CSN); // Set up nRF24L01 radio on SPI bus plus pins HW_CE and HW_CSN

void print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1,  Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3,  Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4,  Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5,  Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6,  Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9,  RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

void verbose_print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1  : Serial.println ("Vbat power on reset");break;
    case 3  : Serial.println ("Software reset digital core");break;
    case 4  : Serial.println ("Legacy watch dog reset digital core");break;
    case 5  : Serial.println ("Deep Sleep reset digital core");break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core");break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core");break;
    case 10 : Serial.println ("Instrusion tested to reset CPU");break;
    case 11 : Serial.println ("Time Group reset CPU");break;
    case 12 : Serial.println ("Software reset CPU");break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU");break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println ("NO_MEAN");
  }
}


void setup_spiffs()
{
  Serial.println("Mounting SPIFFS...");
  // if (!SPIFFS.begin(true)) { this is only required on new boards off the shelf
  if (!SPIFFS.begin()) {
    Serial.println(("Failed to mount file system! Has it been formatted? "));
    if (!SPIFFS.begin(true)) 
      Serial.println(("Failed to format file system! I give up! "));
    return;
  }
  if (!loadConfig()) {
    Serial.println(("Failed to load config"));
  } else {
    Serial.println(("Config loaded"));
  }
  if (!loadNodes()) {
    Serial.println(("Failed to load nodes"));
  } else {
    Serial.println(("Nodes loaded"));
  }
  Serial.println(m_sHR);  
}


bool loadConfig() 
{
  Serial.println(F("Loading configuration..."));
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println(F("Failed to open config file"));
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println(F("Config file size is too large"));
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println(F("Failed to parse config file"));
    return false;
  }

  if (json.containsKey("ssid")) 
  {
    //const char* nwSSID = json["ssid"];
    g_nwSSID = String((const char*)json["ssid"]);
  }
  if (json.containsKey("pass")) 
  {
    //const char* nwPASS = json["pass"];
    g_nwPASS = String((const char*)json["pass"]);
  }
  if (json.containsKey("mqtt")) 
  {
    //const char* nwMQTT = json["mqtt"];
    g_nwMQTT = String((const char*)json["mqtt"]);
  }
  if (json.containsKey("chat")) 
  {
    //const char* tgCHAT = json["chat"];
    g_tgCHAT = String((const char*)json["chat"]);
  }
  
  Serial.println(("["+g_nwSSID+"]"));  
  Serial.println(("["+g_nwPASS+"]"));
  Serial.println(("["+g_nwMQTT+"]"));
  Serial.println(("["+g_tgCHAT+"]"));
  if (g_nwSSID.length() < 4 || g_nwPASS.length() < 6)
  {
    Serial.println(F("SSID or PSK were too short, defaulting to hard-coded nw."));
    g_nwSSID = mySSID;
    g_nwPASS = myPASSWORD;
  }
  return true;
}

bool saveConfig() 
{
  Serial.println(("Saving configuration into spiffs..."));
  char cSSID[g_nwSSID.length()+1], 
      cPASS[g_nwPASS.length()+1], 
      cMQTT[g_nwMQTT.length()+1], 
      cCHAT[g_tgCHAT.length()+1];
  g_nwSSID.toCharArray(cSSID, g_nwSSID.length()+1);    
  g_nwPASS.toCharArray(cPASS, g_nwPASS.length()+1);    
  g_nwMQTT.toCharArray(cMQTT, g_nwMQTT.length()+1);
  g_tgCHAT.toCharArray(cCHAT, g_tgCHAT.length()+1);
  Serial.print(F("Saving new SSID:["));
  Serial.print(cSSID);
  Serial.println(']');
  Serial.print(F("Saving new PASS:["));
  Serial.print(cPASS);
  Serial.println(']');
  Serial.print(F("Saving new MQTT:["));
  Serial.print(cMQTT);
  Serial.println(']');
  Serial.print(F("Saving new CHAT:["));
  Serial.print(cCHAT);
  Serial.println(']');
  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["ssid"] = cSSID;
  json["pass"] = cPASS;
  json["mqtt"] = cMQTT;
  json["chat"] = cCHAT;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println(F("Failed to open config file for writing"));
    return false;
  }
  json.printTo(configFile);
  return true;
}


bool loadNodes() {
  Serial.println(F("Loading nodes..."));
  File configFile = SPIFFS.open("/nodes.json", "r");
  if (!configFile) {
    Serial.println(F("Failed to open nodes file"));
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println(F("Nodes file size is too large"));
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  //StaticJsonBuffer<200> jsonBuffer;
  //JsonObject& json = jsonBuffer.parseObject(buf.get());

  const size_t bufferSize = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(1) + 2*JSON_OBJECT_SIZE(3) + 90;
  DynamicJsonBuffer jsonBuffer(size);

  //JsonObject& root = jsonBuffer.parseObject(json);//this was the new code
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  if (!root.success()) {
    Serial.println(F("Failed to parse nodes file"));
    return false;
  }
  
  m_nGotNodes = root["nodelist"].size();
  
  for (int j = 0; j < m_nGotNodes; j++)
  {
    JsonObject& theNode = root["nodelist"][j];
    relayctl& theStruct = rfNodes[j];
    
    theStruct.sched1      = theNode["sched1"];
    theStruct.sched2      = theNode["sched2"];
    theStruct.maxdur1     = theNode["maxdur1"];
    theStruct.maxdur2     = theNode["maxdur2"];
    theStruct.temp_thres  = theNode["temp"];
    theStruct.state1      = theNode["state1"];
    theStruct.state2      = theNode["state2"];
    theStruct.nodeid      = theNode["nodeid"];
  }
  return true;
}

bool saveNodes()
{
  Serial.println("Saving nodes...");
  const size_t bufferSize = JSON_ARRAY_SIZE(m_nGotNodes) + JSON_OBJECT_SIZE(1) + m_nGotNodes*JSON_OBJECT_SIZE(8);
  DynamicJsonBuffer jsonBuffer(bufferSize);

  JsonObject& root = jsonBuffer.createObject();

  JsonArray& nodelist = root.createNestedArray("nodelist");

  for (int k=0; k<m_nGotNodes; k++)
  {
    relayctl& theStruct = rfNodes[k];
    
    if ( theStruct.nodeid == 0) continue; 
    
    JsonObject& theNode = nodelist.createNestedObject();
    theNode["sched1"] = theStruct.sched1;
    theNode["sched2"] = theStruct.sched2;
    theNode["maxdur1"] = theStruct.maxdur1;
    theNode["maxdur2"] = theStruct.maxdur2;;
    theNode["temp"] = theStruct.temp_thres;
    theNode["state1"] = theStruct.state1;;
    theNode["state2"] = theStruct.state2;
    theNode["nodeid"] = theStruct.nodeid;
  }
  //root.printTo(Serial);
  if (mqttClient.connected())
  {
    Serial.println("Publishing ");
    char jsonStr[root.measureLength()+1];
    root.printTo((char*)jsonStr, root.measureLength()+1);
    Serial.println(jsonStr);
    Serial.println(mqttClient.publish(MOD CAT T_CURSTATUS, "status", 6));
    Serial.println(mqttClient.publish(MOD CAT T_CURSTATUS, jsonStr, root.measureLength()+1));
  } else Serial.println("Could not publish nodes, client was disconnected");
  
  File configFile = SPIFFS.open("/nodes.json", "w");
  if (!configFile) {
    Serial.println(F("Failed to open nodes file for writing"));
    return false;
  }
  root.printTo(configFile);
  return true;
}

bool storeNewNode(relayctl& one)
{
  //TODO: check stored nodes by their ID, if there is room, store this one.
  if (one.nodeid == 0) 
    return false;

  for (int i=0; i< MAX_NODES; i++)
  {
    //find the first free spot
    relayctl& theStruct = rfNodes[i];
    if ( theStruct.nodeid == 0 ||  theStruct.nodeid == one.nodeid )
    {
      theStruct.sched1      = one.sched1;
      theStruct.sched2      = one.sched2;
      theStruct.maxdur1     = one.maxdur1;
      theStruct.maxdur2     = one.maxdur2;
      theStruct.temp_thres  = one.temp_thres;
      theStruct.state1      = one.state1;
      theStruct.state2      = one.state2;
      theStruct.nodeid      = one.nodeid;
      m_nGotNodes++;
      return true;
    }
  }
  return false;
}


void WiFiEvent(WiFiEvent_t event){
    switch(event) {
        case SYSTEM_EVENT_AP_START:
            Serial.println(F("AP Started"));
            WiFi.softAPsetHostname(mySSID);
            break;
        case SYSTEM_EVENT_AP_STOP:
            Serial.println(F("AP Stopped"));
            break;
        case SYSTEM_EVENT_STA_START:
            Serial.println(F("STA Started"));
            WiFi.setHostname("ESP32_meganode");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            Serial.println(F("STA Connected"));
            WiFi.enableIpV6();
            break;
        case SYSTEM_EVENT_AP_STA_GOT_IP6:
            Serial.print(F("STA IPv6: "));
            Serial.println(WiFi.localIPv6());
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.print(F("STA IPv4: "));
            Serial.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println(F("STA Disconnected"));
            break;
        case SYSTEM_EVENT_STA_STOP:
            Serial.println(F("STA Stopped"));
            break;
        default:
            Serial.println(F("Unknown WiFi event!"));
            break;
    }
}

void setup_wifi() 
{
  delay(10);

  // Connect to WiFi network
  Serial.println(F("Connecting WiFi..."));
  
  WiFi.mode(WIFI_OFF);
  yield();
  delay(50); 
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);
  yield();
  
  char cSSID[g_nwSSID.length()+1], cPASS[g_nwPASS.length()+1];
  g_nwSSID.toCharArray(cSSID, g_nwSSID.length()+1);    
  g_nwPASS.toCharArray(cPASS, g_nwPASS.length()+1);    
  WiFi.begin(cSSID, cPASS);
  
  int timeout = 40;
  while (WiFi.status() != WL_CONNECTED) 
  {
    yield();
    //Serial.print(".");
    if (timeout == 30) // a basic connect timeout sorta thingy
    {
      Serial.print(F("Failed to connect to WiFi nw. Status is now "));
      Serial.println(WiFi.status());
      Serial.println(F("Going to try first hardcoded wifi network"));
      WiFi.mode(WIFI_OFF);
      yield();
      delay(50); 
      WiFi.mode(WIFI_STA);
      WiFi.begin(mySSID, myPASSWORD);
    }
    if (timeout == 20) // a basic connect timeout sorta thingy
    {
      Serial.print(F("Failed to connect to WiFi nw. Status is now "));
      Serial.println(WiFi.status());
      Serial.println(F("Going to try secondary hardcoded wifi network"));
      WiFi.mode(WIFI_OFF);
      yield();
      delay(50); 
      WiFi.mode(WIFI_STA);
      WiFi.begin(mySSID2, myPASSWORD2);
    }
    if (timeout == 10) // a basic connect timeout sorta thingy
    {
      Serial.print(F("Failed to connect to WiFi nw. Status is now "));
      Serial.println(WiFi.status());
      Serial.println(F("Going to try thirdly hardcoded wifi network"));
      WiFi.mode(WIFI_OFF);
      yield();
      delay(50); 
      WiFi.mode(WIFI_STA);
      WiFi.begin(mySSID3, myPASSWORD3);
    }
    if (--timeout < 1) // a basic authentication-timeout sorta thingy
    {
      break;
    }
    delay(1000);
  }
  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.println(F("WiFi connection FAILED."));
  }
  else 
    Serial.println(F("WiFi connected"));
  Serial.println(m_sHR);
}


void refreshRF24()
{
  radio.stopListening();
  radio.write("status", 6, false);
  radio.startListening();
} // refreshRF24

void setup_rf24()
{  
  Serial.println(F("Setting up RF24..."));
  //
  // Setup and configure rf radio
  //
  /*SPI.begin();
  SPI.setFrequency(2000000);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(0);*/
  if (radio.begin())
  {
    Serial.println(F("RF24 beginned OK"));
    Serial.print(F("RF24 chip connected:"));
    Serial.println(radio.isChipConnected());
    radio.setCRCLength( RF24_CRC_16 ) ;
    radio.setRetries( 15, 5 ) ;
    radio.setAutoAck( true ) ;
    radio.setPALevel( RF24_PA_MAX ) ;
    radio.setChannel( 108 ) ;
    radio.setDataRate( RF24_250KBPS ) ;
    radio.enableDynamicPayloads(); //dont work with my modules :-/
  
    //
    // Open pipes to other nodes for communication
    //
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
    radio.openReadingPipe(2,addresses[4]);
    radio.openReadingPipe(3,addresses[5]);

    radio.powerUp() ;
    
    Serial.println(F("Signal quality:"));  
    bool goodSignal = radio.testRPD();
    Serial.println(goodSignal ? "Strong signal > 64dBm" : "Weak signal < 64dBm" );
    
    refreshRF24();    

    // This fails on any other architecture except AVR
    // OK Jo, this actually works but causes device reboot
    // So u can use to debug sometimes but when deploying,
    // must comment this out!
    //radio.printDetails();
  }
  else
    Serial.println(F("ERROR - RF24 chip is not connected."));
  Serial.println(m_sHR);
} // setup_rf24


void reconnect() 
{
  short max_retries = 0;
  // Loop until we're reconnected
  while (!mqttClient.connected()) 
  {
    if (max_retries++ > 2) 
    {
      break;
    }
    Serial.print(("Connect to MQTT broker... ")); 
    // Attempt to connect
    if (mqttClient.connect("ESP32Client")) 
    {
      Serial.println(("connected"));
      
      mqttClient.setCallback(callback);
      
      // Once connected, publish an announcement...
      mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      if (!mqttClient.subscribe(MOD CAT T_COMMAND)) Serial.println("KO "  T_COMMAND); 
      if (!mqttClient.subscribe(MOD CAT T_SWITCH1)) Serial.println("KO "  T_SWITCH1); 
      if (!mqttClient.subscribe(MOD CAT T_SWITCH2)) Serial.println("KO "  T_SWITCH2); 
      if (!mqttClient.subscribe(MOD CAT T_SWITCH3) )Serial.println("KO "  T_SWITCH3); 
      if (!mqttClient.subscribe(MOD CAT T_SWITCH4) )Serial.println("KO "  T_SWITCH4); 
      Serial.println(("MQTT subscribed"));
    } 
    else 
    {
      Serial.print(("failed, rc="));
      Serial.print(mqttClient.state());
      Serial.println((" try again in .5 seconds"));
      // Wait 1 seconds before retrying
      delay(500);
    }
  }
}


void printState(relayctl& myData, uint8_t node)
{
  Serial.print(("Plug 1 will be on after "));
  Serial.print(myData.sched1);
  Serial.print(("min of uptime, during "));
  Serial.print(myData.maxdur1);
  Serial.print(("s(currently "));
  Serial.print(myData.state1);
  Serial.print((")\nPlug 2 will be on after "));
  Serial.print(myData.sched2);
  Serial.print(("min of uptime, during "));
  Serial.print(myData.maxdur2);
  Serial.print(("s(currently "));
  Serial.print(myData.state2);
  Serial.print((")\nTemperature: "));
  Serial.print(myData.temp_now);
  Serial.print(("/"));
  Serial.print(myData.temp_thres);
  Serial.print(("\nCurrent uptime: "));
  Serial.print(myData.uptime);
  Serial.print(("min ("));
  Serial.print(myData.uptime/60);
  Serial.print(("h)\nBattery:"));
  Serial.print(myData.battery);
  Serial.print(("V\nWaterLow:"));
  Serial.print(myData.waterlow);
  Serial.print(("\nNodeID#:"));
  Serial.print(myData.nodeid);
  Serial.println();
}

void doScan()
{
  if (radio.isChipConnected())
  {
    const uint8_t num_channels = 126;
    uint8_t       values[num_channels];
  
    radio.begin();
    radio.setAutoAck(false);
    radio.stopListening();
    // Print out header, high then low digit
    int i = 0;
    while ( i < num_channels )
    {
      printf("%x",i>>4);
      ++i;
    }
    Serial.println();
    i = 0;
    while ( i < num_channels )
    {
      printf("%x",i&0xf);
      ++i;
    }
    Serial.println();
    
    const int num_reps = 100;
    for (int loops=0; loops<10; loops++)
    {
      // Clear measurement values
      memset(values,0,sizeof(values));
    
      // Scan all channels num_reps times
      int rep_counter = num_reps;
      while (rep_counter--)
      {
        int i = num_channels;
        while (i--)
        {
          // Select this channel
          radio.setChannel(i);
    
          // Listen for a little
          radio.startListening();
          delayMicroseconds(128);
          radio.stopListening();
    
          // Did we get a carrier?
          if ( radio.testCarrier() ){
            ++values[i];
          }
        }
      }
    
      // Print out channel measurements, clamped to a single hex digit
      int i = 0;
      //while ( i < num_channels )
        printf("%x",_min(0xf,values[i++]));
      Serial.println();
    }
  }
  else 
    Serial.println("ERROR - RF24 chip is not connected! Cannot scan.");
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print(("Message arrived ["));
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  String sTopic(topic), sPayload = "";
  for (int i = 0; i < length; i++) 
  {
    if (!isDigit( payload[i] ))
      break;
    sPayload += (char)payload[i];
  }
  if (sTopic.indexOf(T_SWITCH1)>=0) 
  {
    myData.state1 = ((char)payload[0] == '1')?true:false;
//    digitalWrite(HW_RELAY1, myData.state1);   
  }
  if (sTopic.indexOf(T_SWITCH2)>=0) 
  {
    myData.state2 = ((char)payload[0] == '1')?true:false;
//    digitalWrite(HW_RELAY2, myData.state2);   
  }
  if (sTopic.indexOf(T_TEMP)>=0) 
  {
    myData.temp_thres = sPayload.toInt();
  }
  if (sTopic.indexOf(T_SCHEDULE1)>=0) 
  {
    myData.sched1 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_SCHEDULE2)>=0) 
  {
    myData.sched2 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_DURATION1)>=0) 
  {
    myData.maxdur1 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_DURATION2)>=0) 
  {
    myData.maxdur2 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_COMMAND)>=0) 
  {
    /*String sUp = String(myData.uptime);
    String sS1 = String(myData.sched1);
    String sS2 = String(myData.sched2);
    String sD1 = String(myData.maxdur1);
    String sD2 = String(myData.maxdur2);
    String sT1 = String(myData.temp_thres);
    String sT2 = String(myData.temp_now);
    String sB1 = String(myData.state1);
    String sB2 = String(myData.state2);
    String sWL = String(myData.waterlow);
    String sStatus("Uptime:"+sUp+" Schedule1:"+sS1+" Schedul2:"+sS2+" Duration1:"+sD1+" Duration2:"+sD2+" Temperature:"+sT1+"/"+sT2+" WaterLow:"+sWL);
    char qS[sStatus.length()] ;
    sStatus.toCharArray(qS, sStatus.length());
    Serial.println(sStatus);
    mqttClient.publish(MOD CAT T_CURSTATUS, (uint8_t*)qS, sStatus.length());*/
    mqttClient.publish(MOD CAT T_CURSTATUS, (uint8_t*)&myData, sizeof(myData));
  }
}

void printInfos()
{
  Serial.println( "Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
  Serial.println(F("CPU0 reset reason:"));
  print_reset_reason(rtc_get_reset_reason(0));
  verbose_print_reset_reason(rtc_get_reset_reason(0));

  Serial.println(F("CPU1 reset reason:"));
  print_reset_reason(rtc_get_reset_reason(1));
  verbose_print_reset_reason(rtc_get_reset_reason(1));
  Serial.println(m_sHR);
  
  chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  //print_system_info(Serial);

  printf("ESP32 SDK version:%s, RAM left %d\n", system_get_sdk_version(), system_get_free_heap_size());
/*
  Serial.print(("wifi_get_opmode(): "));
  Serial.print(wifi_get_opmode());
  Serial.print((" - "));
  Serial.println(OP_MODE_NAMES[wifi_get_opmode()]);

  Serial.print(("wifi_get_opmode_default(): "));
  Serial.print(wifi_get_opmode_default());
  Serial.print((" - "));
  Serial.println(OP_MODE_NAMES[wifi_get_opmode_default()]);

  Serial.print(("wifi_get_broadcast_i(): "));
  Serial.println(wifi_get_broadcast_i());
  print_wifi_general(Serial);
  */
  Serial.println();
  WiFi.printDiag(Serial);
  Serial.print(F("WiFi MAC Address: "));
  Serial.println(WiFi.macAddress());
   
  Serial.println(m_sHR);  
}

/*
 * © Francesco Potortì 2013 - GPLv3
 *
 * Send an HTTP packet and wait for the response, return the Unix time
 */
unsigned long webUnixTime (Client &client)
{
  unsigned long time = 0;

  // Just choose any reasonably busy web server, the load is really low
  if (client.connect("http://www.google.com/index.html", 80))
  {
      // Make an HTTP 1.1 request which is missing a Host: header
      // compliant servers are required to answer with an error that includes
      // a Date: header.
      client.print(F("GET / HTTP/1.1 \r\n\r\n"));

      char buf[5];      // temporary buffer for characters
      client.setTimeout(5000);
      if (client.find((char *)"\r\nDate: ") // look for Date: header
          && client.readBytes(buf, 5) == 5) // discard
      {
        unsigned day = client.parseInt();    // day
        client.readBytes(buf, 1);    // discard
        client.readBytes(buf, 3);    // month
        int year = client.parseInt();    // year
        byte hour = client.parseInt();   // hour
        byte minute = client.parseInt(); // minute
        byte second = client.parseInt(); // second
  
        int daysInPrevMonths;
        switch (buf[0])
        {
          case 'F': daysInPrevMonths =  31; break; // Feb
          case 'S': daysInPrevMonths = 243; break; // Sep
          case 'O': daysInPrevMonths = 273; break; // Oct
          case 'N': daysInPrevMonths = 304; break; // Nov
          case 'D': daysInPrevMonths = 334; break; // Dec
          default:
            if (buf[0] == 'J' && buf[1] == 'a')
              daysInPrevMonths = 0;   // Jan
            else if (buf[0] == 'A' && buf[1] == 'p')
              daysInPrevMonths = 90;    // Apr
            else switch (buf[2])
            {
              case 'r': daysInPrevMonths =  59; break; // Mar
              case 'y': daysInPrevMonths = 120; break; // May
              case 'n': daysInPrevMonths = 151; break; // Jun
              case 'l': daysInPrevMonths = 181; break; // Jul
              default: // add a default label here to avoid compiler warning
              case 'g': daysInPrevMonths = 212; break; // Aug
            }
          }

      // This code will not work after February 2100
      // because it does not account for 2100 not being a leap year and because
      // we use the day variable as accumulator, which would overflow in 2149
      day += (year - 1970) * 365; // days from 1970 to the whole past year
      day += (year - 1969) >> 2;  // plus one day per leap year 
      day += daysInPrevMonths;  // plus days for previous months this year
      if (daysInPrevMonths >= 59  // if we are past February
          && ((year & 3) == 0)) // and this is a leap year
        day += 1;     // add one day
      // Remove today, add hours, minutes and seconds this month
      time = (((day-1ul) * 24 + hour) * 60 + minute) * 60 + second;
    }
  }
  delay(10);
  client.flush();
  client.stop();

  return time;
}

void setup()
{
  WiFi.mode(WIFI_OFF);
  yield();
#if defined(ARDUINO_AVR_LEONARDO) 
  /*a little something to tell we're alive*/
  for (int ii = 0; ii<= 5; ii++) 
  {  
    /*blinks the LEDS on the micro*/
    RXLED1;
    TXLED0; //TX LED is not tied to a normally controlled pin
    delay(500);              // wait for a second
    TXLED1;
    RXLED0;
    delay(500);              // wait for a second
  }
  TXLED0; 
  RXLED0;
#endif
  delay(500);
  
  //
  // Print preamble
  //
  Serial.begin(115200);
  delay(100);
  Serial.println(F("ESP32 - WIFI BLE MQTT & RF24 Master"));  
  delay(100);
  Serial.println(F("Warning! You *DO NEED* a high-amp usb connection otherwise WiFi won't work!")); 
  delay(100);
  Serial.println(F("Warning! Always query the controller before attempting to program it!"));  
  delay(100);
  Serial.println(F("Warning! Always select the target node so that somebody hears you!"));  
  delay(100);
  Serial.println(m_sHR);  

  printInfos();

  Serial.println(F("Starting system setup..."));

  memset(&rfNodes, 0, sizeof(rfNodes));

  setup_spiffs();
  
  setup_wifi();
  
  yield();
  
  char cMQTTserver[g_nwMQTT.length()+1];
  g_nwMQTT.toCharArray(cMQTTserver, g_nwMQTT.length()+1);
  mqttClient.setServer(cMQTTserver, 1883);
  mqttClient.setCallback(callback);

  setTime( webUnixTime( espClient) );

  setup_rf24();
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED) 
  {
    if (millis() > g_Bot_lasttime + g_Bot_mtbs)  
    {
      long newMessageIndex = 0;
      newMessageIndex = bot.getUpdates(bot.last_message_received + 1);
      if (newMessageIndex > 0 && newMessageIndex == 1)
      {
        for (int i=0; i<newMessageIndex; i++) 
        {
          telegramMessage& tm = bot.messages[i];
          
          if ( tm.chat_id.length() < 5 || tm.text.length() < 5 ) 
          {
            Serial.println("ERROR! Ignoring empty telegram message");
            Serial.println("ERROR! LastMessageReceived " + String(bot.last_message_received));
            newMessageIndex = bot.getUpdates(bot.last_message_received + 1);
            Serial.println("ERROR! LastMessageReceived " + String(bot.last_message_received));
            continue; 
          }
          Serial.print("chatter " + tm.chat_id);
          Serial.println(", text " + tm.text);
          if (tm.chat_id.length()>5 && tm.chat_id!=g_tgCHAT)
            bot.sendMessage("60001082", "Usuario estranho conversando com o regador:<pre>" + 
            tm.chat_id + 
            "</pre>" + tm.from_name, "HTML");
          else if (tm.text=="/help")
            bot.sendMessage(g_tgCHAT, "Visite <a href=\"http://" + WiFi.localIP().toString() + "\">este link</a> para manipular o regador.", "HTML");
          else if (tm.text.indexOf("IP") >= 0)
            bot.sendMessage(g_tgCHAT, "Meu endereco local e <pre>" + WiFi.localIP().toString() + "</pre>", "HTML");
          else if (tm.text.indexOf("uptime") >= 0)
          {
            bot.sendMessage(g_tgCHAT, "Estou funcionando já fazem " + String((millis() / 1000 / 3600)) + "horas.");
          }
          else if (tm.text.indexOf("rfnodes") >= 0)
          {
            bot.sendMessage(g_tgCHAT, "Refreshing RF24 nodes... just a sec ", "");
            refreshRF24();
          }
          else
            bot.sendMessage(tm.chat_id, "Vc escreveu <pre>" + tm.text + "</pre> mas ainda nao estou programado para fazer nada com esta informacao.", "HTML");
          
        }
        newMessageIndex = bot.getUpdates(bot.last_message_received + 1);
      }
      g_Bot_lasttime = millis();
    } 
  }

  /* 
   *  Because I dunno how I will eventually need to debug this thing,
   *  I am placing serial iface support here too. Sucks. 
   */
  while (Serial.available())
  {
    String s1 = Serial.readString();//readStringUntil('\n');
    Serial.println(F("CAREFUL, end of line is only NL and no CR!!!"));
    Serial.print(F("You typed:"));
    Serial.println(s1);
    if (s1.indexOf("setnewssid ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwSSID = s1.substring(0, s1.length());
      Serial.println(("new ssid is now [" + g_nwSSID + "]"));
    }
    else if (s1.indexOf("setnewpass ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwPASS = s1.substring(0, s1.length());
      Serial.println(("new pass is now [" + g_nwPASS + "]"));
    }
    else if (s1.indexOf("setnewmqtt ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwMQTT = s1.substring(0, s1.length());
      
      char cMQTT[g_nwMQTT.length()+1];
      g_nwMQTT.toCharArray(cMQTT, g_nwMQTT.length()+1);
      
      mqttClient.setServer(cMQTT, 1883);
      
      Serial.println(("new mqtt is now [" + g_nwMQTT + "]"));
    }
    else if (s1.indexOf("save")>=0)
    {
      saveConfig();
    }
    else if (s1.indexOf("reboot please")>=0)
    {
      ESP.restart();
    }
    else if (s1.indexOf("scan")>=0)
    {
      Serial.println(("Scanning RF24 channels:"));  
      doScan();
      Serial.println(m_sHR);  
      Serial.println(("REMEMBER YOU MUST RESTART THIS NODE NOW!!!"));
      s1 = "";
    }
    else if (s1.indexOf("debug")>=0)
    {
      printInfos();
    }      
    else if ((s1.indexOf("setnewpass")!=0) && (s1.indexOf("setnewssid")!=0) && (s1.indexOf("setnewmqtt")!=0))
    {
      Serial.println(F("** Serial interface expects:\n\r"\
        "** 0 - setnewssid: set a new SSID for the module\n\r"\
        "** 1 - setnewpass: set a new PSK key for the nw\n\r"\
        "** 2 - setnewmqtt: set a new MQTT server\n\r"\
        "** 3 - setnewcfg : save the configuration into a file on the flash"));
    }
  }

  
  if (radio.available() && radio.isChipConnected())
  {
    uint8_t pipeNumber = 0; // ZERO is my own pipe, validate it
    {
      while (radio.available(&pipeNumber))
      {
        uint8_t len = radio.getDynamicPayloadSize();
        Serial.print(F("Recvd "));
        Serial.print(len);
        Serial.print(F(" bytes ("));
        Serial.print(sizeof(relayctl));
        Serial.print(F(") on pipe "));
        Serial.println(pipeNumber);
        
        if ( len == sizeof(relayctl) )
        {
          relayctl oTemp;
          radio.read( &oTemp, len);
          printState(oTemp, pipeNumber);
          if (! storeNewNode( oTemp ) )
            Serial.println("This node was not stored!!!");
          else
            saveNodes();
        }
        else if ( len == 2 )
        {
          uint16_t tr = 0L;
          radio.read( &tr, len);
          if ( tr == TIMESTAMPREQ )
          {
            Serial.print(F("Sending out new timestamp "));
            time_t thetime = now();
            Serial.println(thetime);
            radio.stopListening();
            radio.write(&thetime, sizeof(time_t));
            radio.startListening();
          }
          else
          {
            Serial.print(F("This was not a request for timestamp "));
            Serial.println(tr, BIN);
            Serial.println(TIMESTAMPREQ, BIN);
            Serial.println(tr);
            Serial.println(TIMESTAMPREQ);
          }
        }
        else
        {
          uint8_t* rx_data = NULL;
          rx_data = (uint8_t*)calloc(len+1, sizeof(char));
          radio.read( rx_data, len );
      
          // Put a zero at the end for easy printing
          rx_data[len+1] = 0;
          // Spew it
          Serial.println(F("Recvd msg size != datastructure, trying to print as text:"));
          Serial.write(rx_data, len);
          Serial.println();
          //free(rx_data);
          rx_data = NULL;
        }
      }
    }
    radio.startListening();
  }


  
  if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) 
  {
    reconnect();
  }
  if (mqttClient.connected()) 
  {
    mqttClient.loop();
    
    /*long now = millis();
    if (now - lastMsg > 10000) {
      lastMsg = now;
      ++value;
      snprintf (msg, 75, "hello world #%ld", value);
      Serial.print(F("Publish message: "));
      Serial.println(msg);
      mqttClient.publish("outTopic", msg);
    }*/
  }
}


