#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_log.h>

uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t sync_word[2] = { 0x1A, 0xA1 };

static const char * TAG = "Main";

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  ESP_LOGI(TAG, "Data in from RF [%02X:%02X:%02X:%02X:%02X:%02X]: %*s", 
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    len, incomingData);
  //ESP_LOGI(TAG, "Data in: [%d] %*s", len, len, incomingData);

  // RF to UART
  Serial2.write(sync_word, 2); // Sync word
  Serial2.write(mac, 6); // TX MAC Address
  Serial2.write(len); // Length
  Serial2.write(incomingData, len); // Data
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_STA);
  String mac_address = WiFi.macAddress();
  ESP_LOGI(TAG, "MAC Address: %s", mac_address.c_str());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcast_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;    
  esp_now_add_peer(&peerInfo);

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
  { // UART to RF
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
          ESP_LOGI(TAG, "Out to RF: [%02X:%02X:%02X:%02X:%02X:%02X] %.*s", 
            mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5], len, buff);

          // Register peer
          esp_now_peer_info_t peerInfo;
          memset(&peerInfo, 0, sizeof(peerInfo));
          memcpy(peerInfo.peer_addr, mac_address, 6);
          peerInfo.channel = 0;  
          peerInfo.encrypt = false;    
          esp_err_t rst = esp_now_add_peer(&peerInfo);

          if (rst == ESP_OK || rst == ESP_ERR_ESPNOW_EXIST) {
            esp_now_send(mac_address, buff, len);
          }
        }
        state = 0;
      }
    }
  }

  delay(5);
}
