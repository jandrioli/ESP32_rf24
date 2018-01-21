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
#define       UNIQUE_ID 0             // master is 0, unoR3 debugger is 1, promicro_arrosoir is 2, etc
#define       uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds 
#define       MOD                "esp1/"
#define       CAT                "feeds/"
#define       T_SWITCH1          "switch1"
#define       T_SWITCH2          "switch2"
#define       T_SWITCH3          "switch3"
#define       T_SWITCH4          "switch4"
#define       T_SCHEDULE1        "sched1"
#define       T_SCHEDULE2        "sched2"
#define       T_SCHEDULE3        "sched3"
#define       T_SCHEDULE4        "sched4"
#define       T_DURATION1        "dur1"
#define       T_DURATION2        "dur2"
#define       T_DURATION3        "dur3"
#define       T_DURATION4        "dur4"
#define       T_TEMP             "temp"
#define       T_COMMAND          "provideStatus"
#define       T_CURSTATUS        "currentStatus"
#define       THIS_NODE_ID 3                  // master is 0, unoR3 debugger is 1, promicro_arrosoir is 2, etc
#define       DEFAULT_ACTIVATION 600          // 10h from now we activate (in case radio is down and can't program)
#define       DEFAULT_DURATION 10             // max 10s of activation time by default


// WARNING!! 3Node and 4Node are used by my testing sketches ping/pong
const uint8_t addresses[][6] = {
                "0Node", // master writes broadcasts here
                "1Node", // unor3 writes here
                "2Node", // unor3 reads here
                "3Node", // arrosoir reads here
                "4Node", // arrosoir writes here
                "5Node"};// not yet used by anybody
/**
 * exchange data via radio more efficiently with data structures.
 * we can exchange max 32 bytes of data per msg. 
 * schedules are reset every 24h (last for a day) so an INTEGER is
 * large enough to store the maximal value of a 24h-schedule.
 * temperature threshold is rarely used 
 */
struct relayctl {
  uint32_t uptime = 0;                      // current running time of the machine (millis())  4 bytes  
  uint32_t sched1 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr1   4 bytes
  uint32_t sched2 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr2   4 bytes
  uint16_t maxdur1 = DEFAULT_DURATION;      // max duration nbr1 is ON                         2 bytes
  uint16_t maxdur2 = DEFAULT_DURATION;      // max duration nbr2 is ON                         2 bytes
   int8_t  temp_thres = 999;                // temperature at which the syatem is operational  1 byte
   int8_t  temp_now   = 20;                 // current temperature read on the sensor          1 byte
  uint8_t  battery    =  0;                 // current temperature read on the sensor          1 byte
  bool     state1 = false;                  // state of relay output 1                         1 byte
  bool     state2 = false;                  // "" 2                                            1 byte
  bool     waterlow = false;                // indicates whether water is low                  1 byte
  uint8_t  nodeid = 32;                     // nodeid is the identifier of the slave           1 byte
} myData;

enum { 
              STEP_BTON, 
              STEP_BTOFF, 
              STEP_STA, 
              STEP_AP, 
              STEP_AP_STA, 
              STEP_OFF, 
              STEP_BT_STA, 
              STEP_END 
     };

uint64_t      chipid;  
String        g_nwSSID = "", 
              g_nwPASS = "", 
              g_nwMQTT = "192.168.8.1";
long          lastMsg = 0;
char          msg[50];
int           value = 0;
WiFiClient    espClient;
PubSubClient  client(espClient);
RF24          radio(HW_CE, HW_CSN); // Set up nRF24L01 radio on SPI bus plus pins HW_CE and HW_CSN

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
/*  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  printf("Flash real id:   %08X\n", ESP.getFlashChipId());
  printf("Flash real size: %u\n\n", realSize);

  printf("Flash ide  size: %u\n", ideSize);
  printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
  printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  if(ideSize != realSize) 
  {
      Serial.println("Flash Chip configuration wrong!\n");
  } 
  else */
  {
      Serial.println("Mounting SPIFFS...");
      // if (!SPIFFS.begin(true)) { this is only required on new boards off the shelf
      if (!SPIFFS.begin()) {
        Serial.println("Failed to mount file system! Has it been formatted? ");
        return;
      }
      if (!loadConfig()) {
        Serial.println("Failed to load config");
      } else {
        Serial.println("Config loaded");
      }
  }
  Serial.println(F("- - - - -"));  
}


bool loadConfig() {
  Serial.println("Loading configuration...");
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  const char* nwSSID = json["ssid"];
  const char* nwPASS = json["pass"];
  const char* nwMQTT = json["mqtt"];
  //Serial.println((nwSSID));  
  //Serial.println((nwPASS));
  //Serial.println((nwMQTT));

  g_nwSSID = String(nwSSID);
  g_nwPASS = String(nwPASS);
  g_nwMQTT = String(nwMQTT);

  Serial.println(("["+g_nwSSID+"]"));  
  Serial.println(("["+g_nwPASS+"]"));
  Serial.println(("["+g_nwMQTT+"]"));
  if (g_nwSSID.length() < 4 || g_nwPASS.length() < 6)
  {
    Serial.println("SSID or PSK were too short, defaulting to hard-coded nw.");
    g_nwSSID = mySSID;
    g_nwPASS = myPASSWORD;
  }
  return true;
}

bool saveConfig() 
{
  Serial.println("Saving configuration into spiffs...");
  char cSSID[g_nwSSID.length()+1], cPASS[g_nwPASS.length()+1], cMQTT[g_nwMQTT.length()+1];
  g_nwSSID.toCharArray(cSSID, g_nwSSID.length()+1);    
  g_nwPASS.toCharArray(cPASS, g_nwPASS.length()+1);    
  g_nwMQTT.toCharArray(cMQTT, g_nwMQTT.length()+1);
  Serial.print("Saving new SSID:[");
  Serial.print(cSSID);
  Serial.println(']');
  Serial.print("Saving new PASS:[");
  Serial.print(cPASS);
  Serial.println(']');
  Serial.print("Saving new MQTT:[");
  Serial.print(cMQTT);
  Serial.println(']');
  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["ssid"] = cSSID;
  json["pass"] = cPASS;
  json["mqtt"] = cMQTT;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }
  json.printTo(configFile);
  return true;
}


void WiFiEvent(WiFiEvent_t event){
    switch(event) {
        case SYSTEM_EVENT_AP_START:
            Serial.println("AP Started");
            WiFi.softAPsetHostname(mySSID);
            break;
        case SYSTEM_EVENT_AP_STOP:
            Serial.println("AP Stopped");
            break;
        case SYSTEM_EVENT_STA_START:
            Serial.println("STA Started");
            WiFi.setHostname("ESP32_meganode");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            Serial.println("STA Connected");
            WiFi.enableIpV6();
            break;
        case SYSTEM_EVENT_AP_STA_GOT_IP6:
            Serial.print("STA IPv6: ");
            Serial.println(WiFi.localIPv6());
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.print("STA IPv4: ");
            Serial.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("STA Disconnected");
            break;
        case SYSTEM_EVENT_STA_STOP:
            Serial.println("STA Stopped");
            break;
        default:
            Serial.println("Unknown WiFi event!");
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
      Serial.println();
      WiFi.printDiag(Serial);
      Serial.print("Failed to connect to WiFi nw. Status is now ");
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
      Serial.println();
      WiFi.printDiag(Serial);
      Serial.print("Failed to connect to WiFi nw. Status is now ");
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
      Serial.println();
      WiFi.printDiag(Serial);
      Serial.print("Failed to connect to WiFi nw. Status is now ");
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
    Serial.println("WiFi connection FAILED.");
    WiFi.printDiag(Serial);
  }
  else 
    Serial.println(F("WiFi connected"));
  Serial.println(F("- - - - -"));
}



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
    Serial.println("RF24 beginned OK");
    Serial.print("RF24 chip connected:");
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
    
    radio.stopListening();

    // This fails on any other architecture except AVR
    // OK Jo, this actually works but causes device reboot
    // So u can use to debug sometimes but when deploying,
    // must comment this out!
    //radio.printDetails();
    
    radio.write("status", 6, false);
    
    //
    // Start listening
    //
    radio.startListening();
  }
  else
    Serial.println("ERROR - RF24 chip is not connected.");
  Serial.println(F("- - - - -"));
}

void setup()
{
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
  delay(500);
  Serial.println(F("Warning! Always query the controller before attempting to program it!"));  
  delay(500);
  Serial.println(F("Warning! Always select the target node so that somebody hears you!"));  
  delay(500);
  Serial.println(F("- - - - -"));  
  chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.
  Serial.println(F("- - - - -"));  

  Serial.println("CPU0 reset reason:");
  print_reset_reason(rtc_get_reset_reason(0));
  verbose_print_reset_reason(rtc_get_reset_reason(0));

  Serial.println("CPU1 reset reason:");
  print_reset_reason(rtc_get_reset_reason(1));
  verbose_print_reset_reason(rtc_get_reset_reason(1));
  Serial.println(F("- - - - -"));

  Serial.println("Starting system setup...");

  setup_spiffs();                                   // read ssid, psk, ip
  
  setup_wifi();
  
  yield();
  
  char cMQTTserver[g_nwMQTT.length()+1];
  g_nwMQTT.toCharArray(cMQTTserver, g_nwMQTT.length()+1);
  client.setServer(cMQTTserver, 1883);
  client.setCallback(callback);

  setup_rf24();
}

void loop()
{
  
  if (!client.connected()) 
  {
    reconnect();
  }
  if (client.connected()) 
  {
    client.loop();
    
    long now = millis();
    if (now - lastMsg > 10000) {
      lastMsg = now;
      ++value;
      snprintf (msg, 75, "hello world #%ld", value);
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish("outTopic", msg);
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
    Serial.print("You typed:");
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
      
      char cSSID[g_nwSSID.length()+1], cPASS[g_nwPASS.length()+1], cMQTT[g_nwMQTT.length()+1];
      g_nwSSID.toCharArray(cSSID, g_nwSSID.length()+1);    
      g_nwPASS.toCharArray(cPASS, g_nwPASS.length()+1);    
      g_nwMQTT.toCharArray(cMQTT, g_nwMQTT.length()+1);
      
      client.setServer(cMQTT, 1883);
      
      Serial.println(("new mqtt is now [" + g_nwMQTT + "]"));
    }
    else if (s1.indexOf("save")>=0)
    {
      saveConfig();
    }
    else if ((s1.indexOf("setnewpass")!=0) && (s1.indexOf("setnewssid")!=0) && (s1.indexOf("setnewmqtt")!=0))
    {
      Serial.println("** Serial interface expects:\n\r"\
        "** 0 - setnewssid: set a new SSID for the module\n\r"\
        "** 1 - setnewpass: set a new PSK key for the nw\n\r"\
        "** 2 - setnewmqtt: set a new MQTT server\n\r"\
        "** 3 - setnewcfg : save the configuration into a file on the flash");
    }
    else if (s1.indexOf("scan")>=0)
    {
      Serial.println(F("Scanning RF24 channels:"));  
      doScan();
      Serial.println(F("- - - - -"));  
      Serial.println(F("REMEMBER YOU MUST RESTART THIS NODE NOW!!!"));
      s1 = "";
    }
  }

  
  if (radio.available() && radio.isChipConnected())
  {
    uint8_t pipeNumber = 0; // ZERO is my own pipe, validate it
    {
      while (radio.available(&pipeNumber))
      {
        
        // Fetch the payload, and see if this was the last one.
        uint8_t len = radio.getDynamicPayloadSize();
        Serial.print("Recvd ");
        Serial.print(len);
        Serial.print(" bytes (");
        Serial.print(sizeof(relayctl));
        Serial.print(") on pipe ");
        Serial.println(pipeNumber);
        
        if ( len == sizeof(relayctl) )
        {
          relayctl oTemp;
          radio.read( &oTemp, len);
          printState(oTemp, pipeNumber);
        }
        else
        {
          char* rx_data = NULL;
          rx_data = (char*)calloc(len+1, sizeof(char));
          radio.read( rx_data, len );
      
          // Put a zero at the end for easy printing
          rx_data[len+1] = 0;
          // Spew it
          Serial.print(F("Recvd msg size doesnt match my datastructure, trying to print as text:"));
          Serial.write(rx_data);
          //free(rx_data);
          rx_data = NULL;
        }
      }
    }
  }
}


void reconnect() {
  short max_retries = 0;
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    if (max_retries++ > 2) 
    {
      break;
    }
    Serial.print("Connect to MQTT broker... "); 
    // Attempt to connect
    if (client.connect("ESP32Client")) 
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      if (!client.subscribe(MOD CAT T_COMMAND)) Serial.println("KO "  T_COMMAND); 
      if (!client.subscribe(MOD CAT T_SWITCH1)) Serial.println("KO "  T_SWITCH1); 
      if (!client.subscribe(MOD CAT T_SWITCH2)) Serial.println("KO "  T_SWITCH2); 
      if (!client.subscribe(MOD CAT T_SWITCH3) )Serial.println("KO "  T_SWITCH3); 
      if (!client.subscribe(MOD CAT T_SWITCH4) )Serial.println("KO "  T_SWITCH4); 
      Serial.println("subscribed");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 seconds");
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
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
    String sUp = String(myData.uptime);
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
    client.publish(MOD CAT T_CURSTATUS, (uint8_t*)&myData, sizeof(myData));
    //client.publish(MOD CAT T_CURSTATUS, (uint8_t*)qS, sStatus.length());
  }
}


void printState(relayctl& myData, uint8_t node)
{
  Serial.print("Plug 1 will be on after ");
  Serial.print(myData.sched1);
  Serial.print("min of uptime, during ");
  Serial.print(myData.maxdur1);
  Serial.print("s(currently ");
  Serial.print(myData.state1);
  Serial.print(")\nPlug 2 will be on after ");
  Serial.print(myData.sched2);
  Serial.print("min of uptime, during ");
  Serial.print(myData.maxdur2);
  Serial.print("s(currently ");
  Serial.print(myData.state2);
  Serial.print(")\nTemperature: ");
  Serial.print(myData.temp_now);
  Serial.print("/");
  Serial.print(myData.temp_thres);
  Serial.print("\nCurrent uptime: ");
  Serial.print(myData.uptime);
  Serial.print("min (");
  Serial.print(myData.uptime/60);
  Serial.print("h)\nBattery:");
  Serial.print(myData.battery);
  Serial.print("V\nWaterLow:");
  Serial.print(myData.waterlow);
  Serial.print("\nNodeID#:");
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

    // Get into standby mode
    radio.startListening();
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
      //  printf("%x",min(0xf,values[i++]));
      Serial.println();
    }
  }
  else 
    Serial.println("ERROR - RF24 chip is not connected! Cannot scan.");
}
