#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_log.h>
#include <PubSubClient.h>

const char* ssid = "Artron@Kit";
const char* password = "Kit_Artron";
const char* mqtt_server = "broker.artronshop.co.th";
int mqtt_port = 1883;
const char* mqtt_username = "ArtronShopSmartOfficeESPNOWGateway";
const char* mqtt_password = "123456789";


uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t sync_word[2] = { 0x1A, 0xA1 };

static const char * TAG = "Main";

#define MQTT_TO_NOW_TOPIC "/mqtt-to-now"
#define NOW_TO_MQTT_TOPIC "/now-to-mqtt"

WiFiClient espClient;
PubSubClient client(espClient);

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");

      client.subscribe(MQTT_TO_NOW_TOPIC "/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
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

  uint8_t mac[6];
  sscanf(topic, MQTT_TO_NOW_TOPIC "/%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

  ESP_LOGI(TAG, "Send data to RF [%02X:%02X:%02X:%02X:%02X:%02X]: %*s", 
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
      length, payload);

  // MQTT to RF chip
  Serial2.write(sync_word, 2); // Sync word
  Serial2.write(mac, 6); // TX MAC Address
  Serial2.write(length); // Length
  Serial2.write(payload, length); // Data
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

volatile bool sw_pressed_done = false;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  { // RF to MQTT
    while(Serial2.available()) {
      static uint8_t state = 0;
      static uint8_t mac_address[6];
      static uint8_t len;
      static uint8_t buff[256];
      if (state == 0) {
        if (Serial2.read() == sync_word[0]) {
          state = 1;
        }
      } else if (state == 1) {
        if (Serial2.read() == sync_word[1]) {
          state = 2;
        } else {
          state = 0;
        }
      } else if (state == 2) {
        Serial2.setTimeout(20);
        if (Serial2.readBytes(mac_address, 6) == 6) {
          state = 3;
        } else {
          state = 0;
        }
      } else if (state == 3) {
        len = Serial2.read();
        state = 4;
      } else if (state == 4) {
        Serial2.setTimeout(100);
        if (Serial2.readBytes(buff, len) == len) {
          ESP_LOGI(TAG, "Get data from RF [%02X:%02X:%02X:%02X:%02X:%02X]: %*s", 
            mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5],
            len, buff);
            
          // RF to MQTT
          char topic[50];
          snprintf(topic, sizeof(topic), NOW_TO_MQTT_TOPIC "/%x:%x:%x:%x:%x:%x", 
            mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
          client.publish(topic, buff, len);
        }
        state = 0;
      }
    }
  }

  delay(5);
}
