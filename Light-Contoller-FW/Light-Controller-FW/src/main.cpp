#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

#define LED_R_PIN ()
#define LED_G_PIN (13)
#define SW_PIN    (0)
#define RELAY_PIN (12)


uint8_t broadcast_address[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
String broadcast_address_str = "FF:FF:FF:FF:FF:FF";

static const char * TAG = "Main";

#if CORE_DEBUG_LEVEL >= 3
#define ESP_LOGI(TAG, FORMAT, ...) Serial.printf("[%s] " FORMAT "\n", TAG, ## __VA_ARGS__)
#else
#define ESP_LOGI(TAG, FORMAT, ...) ;
#endif

bool output_status = false;
bool skip_write_eeprom = false;
bool req_send_status = false;

// format: <Sender MAC Address>,<Receiver MAC Address>,<Command>,<Payload>
String encode_packet(String receiver_mac, int command, String payload) {
  String sender_mac = WiFi.macAddress();
  sender_mac.toUpperCase();
  return sender_mac + "," + receiver_mac + "," + String(command) + "," + payload;
}

bool decode_packet(String packet, String *sender_mac, String *receiver_mac, int *command, String *payload) {
  char buff_sender_mac[30], buff_receiver_mac[30], buff_payload[100];
  memset(buff_sender_mac, 0, sizeof(buff_sender_mac));
  memset(buff_receiver_mac, 0, sizeof(buff_receiver_mac));
  memset(buff_payload, 0, sizeof(buff_payload));

  // ESP_LOGI(TAG, "Packet: %s", packet.c_str());

  if (sscanf(packet.c_str(), "%[^,],%[^,],%d,%[^,]", buff_sender_mac, buff_receiver_mac, command, buff_payload) < 3) {
    return false;
  }

  *sender_mac = buff_sender_mac;
  *receiver_mac = buff_receiver_mac;
  *payload = buff_payload;

  sender_mac->toUpperCase();
  receiver_mac->toUpperCase();

  return true;
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  ESP_LOGI(TAG, "Last Packet Send Status: %s", sendStatus == 0 ? "OK" : "FAIL");
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  String packet = "";
  for (uint16_t i=0;i<len;i++) {
    packet += (char) incomingData[i];
  }

  String sender_mac, receiver_mac, payload;
  int command;
  if (decode_packet(packet, &sender_mac, &receiver_mac, &command, &payload)) {
    ESP_LOGI(TAG, "Sender: %s, Recv: %s, CMD: %d, Payload: %s", sender_mac.c_str(), receiver_mac.c_str(), command, payload.c_str());
    if (receiver_mac != WiFi.macAddress() && receiver_mac != broadcast_address_str) {
      return; // Skip if receiver mac is not self mac address or broadcast address
    }

    if (command == 1) { // 1: Control light bulb
      output_status = payload[0] == '1';
      req_send_status = true;
    } else if (command == 3) { // 3: Require light bulb status
      // Send now status
      req_send_status = true;
    }
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

  ESP_LOGI(TAG, "MAC Address: %s", WiFi.macAddress().c_str());

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
  esp_now_add_peer(broadcast_address, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

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
    String packet = encode_packet(broadcast_address_str, 2, output_status ? "1" : "0"); // 2: Light bulb status report
    esp_now_send(broadcast_address, (uint8_t *) packet.c_str(), packet.length()); 

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
