/***************************************************************************/
#define TINY_GSM_MODEM_SIM7600
#include <SoftwareSerial.h>
// #include "AHTLAB_SIM7600CE.h"
// #include <HCSR04.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <string.h>
#include <TinyGsmClient.h>
#include "SRF05.h"





//////////////////////////////////////CONSTANT///////////////////////////////////
#define BTN_SIREN_PIN                       14
#define SRF04_DETECT_PERSON_TRIG_PIN        19
#define SRF04_DETECT_PERSON_ECHO_PIN        18
#define SRF04_TRASH_LEVEL_TRIG_PIN          22
#define SRF04_TRASH_LEVEL_ECHO_PIN          21
#define SERVO_PIN                           12
#define GPS_RX_PIN                          4
#define GPS_TX_PIN                          3


#define SerialAT Serial2
#define TINY_GSM_DEBUG Serial

#define GPS_BAUD                            4800


// Thingsboard
#define THINGSBOARD_SERVER                  "demo.thingsboard.io"
// #define ACCESS_TOKEN                        "60UX5zgiNqi1ECmppeCI"   
// #define ACCESS_TOKEN                        "U2ugcQ8rzfU5EvnSexJr" 
#define ACCESS_TOKEN                        "wwzCUEFupCDOcUZrHABa"                              
#define RESPONSE_LEN                        2048
#define THINGSBOARD_PORT                    1883

// Servo:
#define ANGLE_CLOSE                         180 //Close angle of servo
#define ANGLE_OPEN                          0   //Open angle of servo

// WiFi
#define WIFI_SSID       "II&IL LAB"     // Change this to your WiFi SSID
#define WIFI_PASSWORD    "baocaochua"

//  GPRS credentials, if any
const char apn[] = "myAPN";

const char gprsUser[] = "";
const char gprsPass[] = "";


constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

// LED
#define LED_PIN  2

int led_state = LOW;

// GPS
TinyGsm modem(SerialAT);
#define GPS_BUFFER_MAX_LEN    100

float lat2      = 0;
float lon2      = 0;

// Device state:
typedef enum {
  IDLE_STATE,
  OPEN_STATE,
  CLOSE_STATE
} MODE_STATE;
///////////////////////////////////VARIABLES///////////////////////////////////
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);      // The serial connection to the GPS device
Servo myservo;                        // create servo object to control a servo
const char* ssid     = WIFI_SSID;     // Change this to your WiFi SSID
const char* password = WIFI_PASSWORD; // Change this to your WiFi password
unsigned long previousMillis = 0;
unsigned char autoTrigger = 0;
unsigned long autoMillis = 0;
unsigned long autoSendtoServer = 0;
// HCSR04 sr04_detect_person(SRF04_DETECT_PERSON_TRIG_PIN, SRF04_DETECT_PERSON_ECHO_PIN);
// HCSR04 sr04_trash_level(SRF04_TRASH_LEVEL_TRIG_PIN, SRF04_TRASH_LEVEL_ECHO_PIN);
int servoPin = SERVO_PIN;
// WiFiManager wm;
// Initialize underlying client, used to establish a connection
WiFiClient wifiClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(wifiClient);
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// GPRS
TinyGsmClient  client(modem);
Arduino_MQTT_Client mqttClient1(client);
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb1(mqttClient1, MAX_MESSAGE_SIZE);

char response[RESPONSE_LEN];
String data_json;
uint8_t modeRun = IDLE_STATE;
uint32_t timeMillis = 0;
uint8_t timeOpen  = 2;

// @brief Initalizes WiFi connection,
// will endlessly delay until a connection has been successfully established
bool InitWiFi(unsigned int timeout) {
    Serial.println();
    Serial.println("******************************************************");
    Serial.print("Connecting to AP... ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int count =0;
    int maxcount= timeout/500;
    while (WiFi.status() != WL_CONNECTED) {
      // Delay 500ms until a connection has been succesfully established
      delay(500);
      Serial.print(".");
      count ++;
      if (count >= maxcount) break;  
    }
    if (count < maxcount)
    {
      Serial.println("Connected to AP");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    return false;

}     

// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
// @return Returns true as soon as a connection has been established again
bool reconnectWiFi(unsigned int timeout) {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    Serial.println("WiFi connected");
    return true;
  }
  Serial.println("Reconnecting to Wifi...");
  // If we aren't establish a new connection to the given WiFi network
  if (InitWiFi(timeout))  return true;
  
  return false;
}

// @brief Detect peron by SRF04 
uint8_t readSRF04_detect_person()
{
      int distance;
      // Phần previousMillis >= 100 đây chính là thời gian lấy mẫu của cảm biến siêu âm 100ms
      if (millis() - previousMillis >= 10) {
        previousMillis = millis();
        distance = sr04_detect_person.dist();
        // Phần distance < 10 đây là phần cài đặt khoảng cách cảm biến nhận được kích hoạt mở thùng rác
        if (distance < 30 && distance > 1) {  
            autoMillis = millis();
            return 1;
        }
        return 0;
      }
}

// @brief Check trash level: 
uint16_t readSRF04_trash_level()
{
    uint16_t trash_level;
    // Phần previousMillis >= 100 đây chính là thời gian lấy mẫu của cảm biến siêu âm 100ms
    if (millis() - previousMillis >= 10)
    {
      Serial.println("Trash_level:");
      previousMillis = millis();
      trash_level = sr04_trash_level.dist();
      Serial.println(trash_level);
      return trash_level;
    }
}

// connect GPRS
bool connectGPRS()
{
  // Network registration  
  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
  Serial.println(" fail");
  return false;
  }
  Serial.println(" success");

  if (modem.isNetworkConnected()) { Serial.println("Network connected"); }

  // GPRS 
  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" success");

  if (modem.isGprsConnected()) { Serial.println("GPRS connected"); }
  return true;
}
// reconnect GPRS 
bool reconnectGPRS()
{
  if (modem.isGprsConnected()) 
  {
    Serial.println("GPRS connected");
    return true;
  } 
  if (connectGPRS())  return true;
  Serial.println("Reconnect GPRS fail");
  return false;
}

// SIM MQTT connect 
bool SIM_MQTTconnect()
{
 if (!tb1.connect(THINGSBOARD_SERVER, ACCESS_TOKEN, THINGSBOARD_PORT))
 {
  Serial.println("Failed connect to Thingsboard");
  return false;
 }
 Serial.println("Connected to Thingsboard");
 return true;
}

bool getGPS()
{
  Serial.println("Getting GPS location");
  if (!modem.init()) {
    Serial.println("Failed to restart modem, delaying 10s and retrying");
    return false;
    }

    if (modem.getGPS(&lat2,&lon2))
    { 
      Serial.print("Latitude:");
      Serial.println(String(lat2,8));   
      Serial.print("Longitude:"); 
      Serial.println(String(lon2,8));   
    } else {
      Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
      return false;
    }
    return true;
}
//Update data to Thingsboard via WiFi
bool Update2Thingsboard_WiFi()
{
    Serial.println("Update using WiFi");
    // Check WiFi status:
      if (!reconnectWiFi(15000)) {
          return false;                  
      }
    // Connecting to Thingsboard 
      if (!tb.connect(THINGSBOARD_SERVER, ACCESS_TOKEN))
      {
        Serial.println("Connecting to Thingsboard fail");
        return false;
      }
      Serial.println("Connected to Thingsboard");
    // Send Telemetry Data to Thingsboard
      tb.sendAttributeData("rssi", WiFi.RSSI());
      tb.sendTelemetryData("latitude", lat2);     
      tb.sendTelemetryData("longitude", lon2); 
      Serial.println("Updated");
      return true;
}

//Update data to Thingsboard via GPRS
bool Update2Thingsboard_GPRS()
{
    Serial.println("Update using GPRS");
      if (!reconnectGPRS())
      {
        return  false;
      }
      if (SIM_MQTTconnect())
      {
        // Send data to Thingsboard
        tb1.sendTelemetryData("latitude", lat2);     
        tb1.sendTelemetryData("longitude", lon2);
        modem.gprsDisconnect();
        Serial.println("Updated");
        return true;
      } 
      return false;
}
// Setup device:
void setup()
{
  delay(5000);
  // Setup Serial:
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    Serial.println("Start_ESP32");
  
    // GSM
    SerialAT.begin(115200);
    if (!modem.init()) {
    Serial.println("Failed to restart modem, delaying 10s and retrying");
    delay(5000);
    } 
    // // Setup Servo:
    // myservo.attach(servoPin, 1000, 2000);
    // myservo.write(ANGLE_CLOSE);

    // // Setup Siren:
    // pinMode(BTN_SIREN_PIN, OUTPUT);
    // pinMode(A7, INPUT_PULLUP);
    // digitalWrite(BTN_SIREN_PIN, HIGH);

    // Serial.println(" =>Setup End");
}

// Loop Main:
void loop()
{
  
  // Check WiFi status and Send data to Server (Every 1 minute):
  if (millis() - autoSendtoServer >= 5000) {
      autoSendtoServer = millis();
      // Get sensor data 
      Serial.print("Distance:");
      Serial.println();
      if (getGPS())
      {
      if (!Update2Thingsboard_WiFi())
      {
        Update2Thingsboard_GPRS();
      }  
      }
   
  }

  // Detect person -> open Bin:
  // switch(modeRun)
  // {
  //   case IDLE_STATE:
  //     if(readSRF04_detect_person() == 1) {
  //       modeRun = OPEN_STATE;
  //       myservo.write(ANGLE_OPEN);
  //       digitalWrite(BTN_SIREN_PIN, LOW);
  //       delay(50);
  //       digitalWrite(BTN_SIREN_PIN, HIGH);
  //       timeMillis = millis();
  //     }
  //     break;
  //   case OPEN_STATE:
  //     if(millis() - timeMillis > timeOpen*1000) {
  //        timeMillis = millis() ;
  //        modeRun = CLOSE_STATE;
  //     }
  //     if(readSRF04_detect_person() == 1 ) {
  //       timeMillis = millis();
  //     }
  //     break;
  //   case CLOSE_STATE:
  //      myservo.write(ANGLE_CLOSE);
  //      delay(500);
  //      modeRun = IDLE_STATE;
  //     break;
  // }
}
