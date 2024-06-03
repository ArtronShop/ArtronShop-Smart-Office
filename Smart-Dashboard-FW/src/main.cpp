#include <Arduino.h>
#include <lvgl.h>
#include <ATD3.5-S3.h>
#include "gui/ui.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_log.h>
#include <ArtronShop_SHT3x.h>
#include <PMS.h>

uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
String broadcast_address_str = "FF:FF:FF:FF:FF:FF";
String light_address[5] = { 
  "5C:CF:7F:3C:1F:AF", // หน้า
  "5C:CF:7F:3B:CA:53", // กลาง
  "5C:CF:7F:3B:C7:64", // หลัง
  "5C:CF:7F:AF:08:4D", // ลอยหน้า
  "5C:CF:7F:3C:21:25", // ลอยหลัง
};

String door_sensor_address[2] = { 
  "24:6F:28:25:E8:40", // Door Sensor 1
  ""
};

const char * verify_code_in = "SmartDashboard:";
const char * verify_code_door_in = "DoorCheck@ArtronShop:";
const char * verify_code_out = "Sonoff@ArtronShop:";

const char * ssid = "Artron@Kit";
const char * password = "Kit_Artron";

static const char * TAG = "Main";

ArtronShop_SHT3x sht3x(0x44, &Wire); // ADDR: 0 => 0x44, ADDR: 1 => 0x45
PMS pms(Serial2);
PMS::DATA data;
bool pms_ok = false;

bool cloud_sending = false;
unsigned long last_touch_on_display = 0;

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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

typedef struct {
  uint8_t mac[6];
  uint8_t *incomingData;
  int len;
} NOWMessage;

QueueHandle_t nowQueue;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  ESP_LOGI(TAG, "Data in: %*s", len, incomingData);
  
  NOWMessage data;
  memcpy(data.mac, mac, 6);
  data.incomingData = (uint8_t *) malloc(len);
  if (data.incomingData) {
    memcpy(data.incomingData, incomingData, len);
    data.len = len;
  } else {
    ESP_LOGE(TAG, "no memory");
    data.len = 0;
  }
  if(xQueueSend(nowQueue, (void *) &data, 10) != pdPASS) {
    ESP_LOGE(TAG, "Send now queue fail");
  }
}

void dataInProcess(const uint8_t * mac, const uint8_t *incomingData, int len) {
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

    if (command == 2) { // 2: Light bulb status report
      bool output_status = payload.toInt() == 1;
      lv_obj_t * ui_light_x_sw[] = {
        ui_light1_sw,
        ui_light2_sw,
        ui_light3_sw,
        ui_light4_sw,
        ui_light5_sw,
      };
      lv_obj_t * ui_light_x_img[] = {
        ui_light1_img,
        ui_light2_img,
        ui_light3_img,
        ui_light4_img,
        ui_light5_img,
      };
      for (uint8_t i=0;i<5;i++) {
        if (light_address[i] == sender_mac) {
          if (output_status) {
            lv_obj_add_state(ui_light_x_sw[i], LV_STATE_CHECKED);
            lv_obj_add_state(ui_light_x_img[i], LV_STATE_CHECKED);
          } else {
            lv_obj_clear_state(ui_light_x_sw[i], LV_STATE_CHECKED);
            lv_obj_clear_state(ui_light_x_img[i], LV_STATE_CHECKED);
          }
        }
      }
    } else if (command == 2) { // 4: Door sensor status report
      bool output_status = payload.toInt() == 1;
      lv_obj_t * ui_door_x_status[] = {
        ui_door1_status,
        ui_door2_status
      };
      for (uint8_t i=0;i<1;i++) {
        if (door_sensor_address[i] == sender_mac) {
          ESP_LOGI(TAG, "Door %d : %d", i, output_status ? 1 : 0);
          if (output_status) {
            lv_obj_add_state(ui_door_x_status[i], LV_STATE_CHECKED);
          } else {
            lv_obj_clear_state(ui_door_x_status[i], LV_STATE_CHECKED);
          }
        }
      }
    } else { // No payload
      // ---
    }
  }
}

void light_sw_click_handle(lv_event_t * e) {
  lv_obj_t * target = lv_event_get_target(e);
  int i = (int) lv_event_get_user_data(e);
  String packet = encode_packet(light_address[i], 1, lv_obj_has_state(target, LV_STATE_CHECKED) ? "1" : "0"); // 1: Control light bulb
  esp_now_send(broadcast_address, (uint8_t *) packet.c_str(), packet.length());
}

void report_temp_humi(String payload) {
  static unsigned long timer = 0;
  if ((timer == 0) || ((millis() - timer) >= 5000) || (millis() < timer)) {
    String packet = encode_packet(broadcast_address_str, 6, payload); // 6: Temperature & Humidity sensor status report
    // ESP_LOGI(TAG, "packet = %*s", packet.length(), packet.c_str());
    esp_now_send(broadcast_address, (uint8_t *) packet.c_str(), packet.length()); 
    timer = millis();
  }
}

int count_of_error = 0;
void sensor_update_timer(lv_timer_t * timer) {
  static bool sensor_ready = false;
  if (!sensor_ready) {
    lv_label_set_text(ui_temp_value, "");
    lv_label_set_text(ui_humi_value, "");

    if (sht3x.begin()) {
      sensor_ready = true;
    } else {
      report_temp_humi("-99,-99");

      count_of_error++;
      if (count_of_error > 5) {
        Wire.begin(); // Fixed I2C bus error
        count_of_error = 0;
      }
    }
    return;
  }

  if (sht3x.measure()) {
    lv_label_set_text_fmt(ui_temp_value, "%.01f", sht3x.temperature());
    lv_label_set_text_fmt(ui_humi_value, "%.01f", sht3x.humidity());

    report_temp_humi(String(sht3x.temperature(), 1) + "," + String(sht3x.humidity(), 1));
    sensor_ready = true;
  } else {
    sensor_ready = false;
  }
}

void pm_update() {
  String payload = "";
  if (pms_ok) {
    lv_label_set_text_fmt(ui_pm25_value, "%d", data.PM_AE_UG_2_5);
    lv_label_set_text_fmt(ui_pm10_value, "%d", data.PM_AE_UG_1_0);
    lv_label_set_text_fmt(ui_pm100_value, "%d", data.PM_AE_UG_10_0);
    payload = String(data.PM_AE_UG_2_5) + "," + String(data.PM_AE_UG_1_0) + "," + String(data.PM_AE_UG_10_0);
  } else {
    lv_label_set_text(ui_pm25_value, "");
    lv_label_set_text(ui_pm10_value, "");
    lv_label_set_text(ui_pm100_value, "");
    payload = "-99,-99,-99";
  }

  {
    static unsigned long timer = 0;
    if ((timer == 0) || ((millis() - timer) >= 5000) || (millis() < timer)) {
      String packet = encode_packet(broadcast_address_str, 8, payload); // 8: PM sensor status report
      esp_now_send(broadcast_address, (uint8_t *) packet.c_str(), packet.length()); 
      timer = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);

  nowQueue = xQueueCreate(20, sizeof(NOWMessage));

  WiFi.mode(WIFI_MODE_STA);
  ESP_LOGI(TAG, "MAC Address: %s", WiFi.macAddress().c_str());
  
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

  // Setup peripherals
  Display.begin(0); // rotation number 0
  Touch.begin();
  Sound.begin();
  Wire.begin();
  Serial2.begin(9600, SERIAL_8N1, 1); // RX = IO1
  
  // Map peripheral to LVGL
  Display.useLVGL(); // Map display to LVGL
  Touch.useLVGL(); // Map touch screen to LVGL
  Sound.useLVGL(); // Map speaker to LVGL
  
  // Add load your UI function
  ui_init();

  // Add event handle
  lv_obj_add_event_cb(ui_light1_sw, light_sw_click_handle, LV_EVENT_CLICKED, (void*) 0);
  lv_obj_add_event_cb(ui_light2_sw, light_sw_click_handle, LV_EVENT_CLICKED, (void*) 1);
  lv_obj_add_event_cb(ui_light3_sw, light_sw_click_handle, LV_EVENT_CLICKED, (void*) 2);
  lv_obj_add_event_cb(ui_light4_sw, light_sw_click_handle, LV_EVENT_CLICKED, (void*) 3);
  lv_obj_add_event_cb(ui_light5_sw, light_sw_click_handle, LV_EVENT_CLICKED, (void*) 4);

  // get last status
  String packet;
  packet = encode_packet(broadcast_address_str, 3, ""); // 3: Require light bulb status
  esp_now_send(broadcast_address, (uint8_t *) packet.c_str(), packet.length()); 

  packet = encode_packet(broadcast_address_str, 5, ""); // 5: Require door sensor status
  esp_now_send(broadcast_address, (uint8_t *) packet.c_str(), packet.length()); 
  
  // Add sensor read and update to UI
  sensor_update_timer(NULL);
  pm_update();
  lv_timer_create(sensor_update_timer, 1000, NULL);
}

void loop() {
  Display.loop();

  { // PMS7003
    static unsigned long last_work = 0;
    if (pms.read(data)) {
      pms_ok = true;
      pm_update();
      last_work = millis();
    } else {
      if ((millis() - last_work) > 2000) {
        pms_ok = false;
        pm_update();
        last_work = millis();
      }
    }
  }
  
  { // handle ESP-NOW message
    NOWMessage data;
    if(xQueueReceive(nowQueue, &data, 0) == pdPASS) {
      dataInProcess(data.mac, data.incomingData, data.len);
      free(data.incomingData);
    }
  }

  { // Display sleep
    #define LCD_BL_PIN   (3)

    static bool display_enter_to_sleep = false;
    if ((millis() - last_touch_on_display) > (60 * 1000)) {
      if (!display_enter_to_sleep) {
        Display.off();
        digitalWrite(LCD_BL_PIN, LOW);
        display_enter_to_sleep = true;
      }
    } else {
      if (display_enter_to_sleep) {
        lv_obj_invalidate(lv_scr_act());
        lv_timer_handler();
        Display.on();
        digitalWrite(LCD_BL_PIN, HIGH);
        display_enter_to_sleep = false;
      }
    }
  }

  delay(10);
}
