#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

#define LED_R_PIN ()
#define LED_G_PIN (13)
#define SW_PIN    (0)
#define RELAY_PIN (12)


uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const char * verify_code_in = "Sonoff@ArtronShop:";
const char * verify_code_out = "SmartDashboard:";

static const char * TAG = "Main";

#if CORE_DEBUG_LEVEL >= 3
#define ESP_LOGI(TAG, FORMAT, ...) Serial.printf("[%s] " FORMAT "\n", TAG, ## __VA_ARGS__)
#else
#define ESP_LOGI(TAG, FORMAT, ...) ;
#endif

bool output_status = false;
bool skip_write_eeprom = false;
bool req_send_status = false;

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  ESP_LOGI(TAG, "Last Packet Send Status: %s", sendStatus == 0 ? "OK" : "FAIL");
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  if (len < strlen(verify_code_in)) {
    ESP_LOGI(TAG, "data invaild ! : %.*s", len, incomingData);
    return; // skip invaild data
  }
  
  if (strncmp(verify_code_in, (char *) incomingData, strlen(verify_code_in)) != 0) {
    ESP_LOGI(TAG, "verify code invaild ! : %.*s", len, incomingData);
    return; // skip invaild data
  }

  uint8_t * payload = NULL;
  const uint8_t payload_len = len - strlen(verify_code_in);
  if (payload_len > 0) {
    payload = &incomingData[strlen(verify_code_in)];
  }

  if (payload_len > 0) {
    output_status = payload[0] == '1';
  } else { // No payload
    // Send now status
    req_send_status = true;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_G_PIN, OUTPUT);
  digitalWrite(LED_G_PIN, HIGH);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(SW_PIN, INPUT);

  EEPROM.begin(100);
  output_status = EEPROM.read(0) == 1;
  skip_write_eeprom = true;

  Serial.println();
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  req_send_status = true; // Send status in first time
}

volatile bool sw_pressed_done = false;

void loop() {
  if (digitalRead(SW_PIN) == LOW) {
    if (!sw_pressed_done) {
      delay(10);
      if (digitalRead(SW_PIN) == LOW) {
        output_status = !output_status;
        req_send_status = true;
        sw_pressed_done = true;
      }
    }
  } else {
    sw_pressed_done = false;
  }

  if (req_send_status) {
    uint8_t data[strlen(verify_code_out) + 1];
    memcpy(data, verify_code_out, strlen(verify_code_out));
    data[strlen(verify_code_out)] = output_status ? '1' : '0';
    esp_now_send(broadcastAddress, data, sizeof(data));
    req_send_status = false;
  }

  { // Update to relay and LED and save to EEPROM
    static bool last_output_status = false;
    if (output_status != last_output_status) {
      digitalWrite(RELAY_PIN, output_status ? HIGH : LOW);
      digitalWrite(LED_G_PIN, output_status ? LOW : HIGH);
      if (!skip_write_eeprom) {
        ESP_LOGI(TAG, "Write %d to EEPROM", output_status ? 1 : 0);
        EEPROM.write(0, output_status ? 1 : 0);
        EEPROM.commit();
      }
    }
    skip_write_eeprom = false;
    last_output_status = output_status;
  }

  delay(10);
}
