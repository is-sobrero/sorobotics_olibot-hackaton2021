/*
LICENZA
*/

#define FLOODING_THRESHOLD 10 

#include <Arduino_MKRIoTCarrier.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>  
#include <ArduinoJson.h>  

#include "secrets.h";

MKRIoTCarrier carrier;
WiFiClient mkr1010Client;
PubSubClient mqtt(mkr1010Client);

float gyro_x;
float gyro_y;
float gyro_z;

typedef enum {
  STATE_IDLING, STATE_EARTHQUAKE, STATE_FIRE, STATE_FLOOD
} sysstate_t;

typedef enum {
  IDLE, EARTHQUAKE, FIRE, FLOOD
} eventtype_t;

int moistPin = A5;

char ssid[] = WIFI_SSID;
char pasw[] = WIFI_PASW;

int status = WL_IDLE_STATUS;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pasw);

    // wait 10 seconds for connection:
    delay(3000);
  }
  CARRIER_CASE = false;
  carrier.begin();
  Serial.println("Board Information:");
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.println();
  Serial.println("Network Information:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);
  mqtt.setServer("test.mosquitto.org", 1883);
}

eventtype_t event;
sysstate_t state = STATE_IDLING;

float prec_gyro_x = 0;
float prec_gyro_y = 0;
float prec_gyro_z = 0;

#define GYRO_DERIV_THRESH 75
#define DERIV_EVENT_COUNT 20

uint8_t derivThresCount = 0;

eventtype_t getEvent(){
  float moistValue = map(analogRead(A5), 0, 1023, 100, 0);
  prec_gyro_x = gyro_x;
  prec_gyro_y = gyro_y;
  prec_gyro_z = gyro_z;

  carrier.IMUmodule.readGyroscope(gyro_x, gyro_y, gyro_z);
  
  float dgx_dt = gyro_x - prec_gyro_x;
  float dgy_dt = gyro_y - prec_gyro_y;
  float dgz_dt = gyro_z - prec_gyro_z;
  
  if (abs(dgx_dt) > GYRO_DERIV_THRESH) derivThresCount++;
  if (abs(dgy_dt) > GYRO_DERIV_THRESH) derivThresCount++;
  if (abs(dgz_dt) > GYRO_DERIV_THRESH) derivThresCount++;

  if (derivThresCount > DERIV_EVENT_COUNT){
    derivThresCount = 0;
    return EARTHQUAKE;
  }

  if (moistValue >= FLOODING_THRESHOLD)
    return FLOOD;  
  return IDLE;
}

void reconnect(){

  while(!mqtt.connected()){
    
    Serial.print("Attempting MQTT connection ....");

    if (mqtt.connect("arduinoClient")) { 
      
      Serial.println("Connected to MQTT Broker");
    
    }

    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println("try again in 5 second");
      delay(5000);
    }
    
    
  }
  
}


static char payload[256];
StaticJsonDocument<256> doc;

void loop() {
  // macchina a stati del sistema
  if (!mqtt.connected())
  {
    reconnect();
  }

  mqtt.loop();
  switch(state){
    // Stato iniziale, controlla le condizioni di trigger del robot o dei soccorsi
    case STATE_IDLING:
      switch (getEvent()){
        case IDLE:
        break;
        case FLOOD:
          state = STATE_FLOOD;
          doc["system_status"] = "flood";
          serializeJsonPretty(doc, payload);
          mqtt.publish("retrieverbot/v1/notify", payload);
          // chiamata ai soccorsi gestita dal robot
        break;
        case EARTHQUAKE:
          doc["system_status"] = "earthquake";
          serializeJsonPretty(doc, payload);
          mqtt.publish("retrieverbot/v1/notify", payload);
          state = STATE_EARTHQUAKE;
          // chiama soccorsi
        break;
      }
    break;
    case STATE_FLOOD:
      Serial.println("ALLAGAMENTO");
    break;
    case STATE_EARTHQUAKE:
      Serial.println("TERREMOTO");
    break;
    default:
    break;
  }
}
