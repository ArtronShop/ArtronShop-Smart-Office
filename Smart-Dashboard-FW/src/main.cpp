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
uint8_t light_address[3][6] = { 
  { 0x5C, 0xCF, 0x7F, 0x3C, 0x1F, 0xAF }, // หน้า
  { 0x5C, 0xCF, 0x7F, 0x3B, 0xCA, 0x53 }, // กลาง
  { 0x5C, 0xCF, 0x7F, 0x3B, 0xC7, 0x64 }  // หลัง
};

const char * verify_code_in = "SmartDashboard:";
const char * verify_code_out = "Sonoff@ArtronShop:";

static const char * TAG = "Main";

ArtronShop_SHT3x sht3x(0x44, &Wire); // ADDR: 0 => 0x44, ADDR: 1 => 0x45
PMS pms(Serial2);
PMS::DATA data;
bool pms_ok = false;

bool cloud_sending = false;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  ESP_LOGI(TAG, "Data in: %*s", len, incomingData);
  
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
    payload = (uint8_t *) &incomingData[strlen(verify_code_in)];
  }

  if (payload_len > 0) {
    bool output_status = payload[0] == '1';
    lv_obj_t * ui_light_x_sw[] = {
      ui_light1_sw,
      ui_light2_sw,
      ui_light3_sw,
    };
    lv_obj_t * ui_light_x_img[] = {
      ui_light1_img,
      ui_light2_img,
      ui_light3_img,
    };
    for (uint8_t i=0;i<3;i++) {
      if (memcmp(light_address[i], mac, 6) == 0) {
        if (output_status) {
          lv_obj_add_state(ui_light_x_sw[i], LV_STATE_CHECKED);
          lv_obj_add_state(ui_light_x_img[i], LV_STATE_CHECKED);
        } else {
          lv_obj_clear_state(ui_light_x_sw[i], LV_STATE_CHECKED);
          lv_obj_clear_state(ui_light_x_img[i], LV_STATE_CHECKED);
        }
      }
    }
  } else { // No payload
    // ---
  }
}

void light_sw_click_handle(lv_event_t * e) {
  lv_obj_t * target = lv_event_get_target(e);
  int i = (int) lv_event_get_user_data(e);
  uint8_t data[strlen(verify_code_out) + 1];
  memcpy(data, verify_code_out, strlen(verify_code_out));
  data[strlen(verify_code_out)] = lv_obj_has_state(target, LV_STATE_CHECKED) ? '1' : '0';
  esp_now_send(light_address[i], data, sizeof(data));
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
  } else {
    sensor_ready = false;
  }
}

void pm_update() {
  if (pms_ok) {
    lv_label_set_text_fmt(ui_pm25_value, "%d", data.PM_AE_UG_2_5);
    lv_label_set_text_fmt(ui_pm10_value, "%d", data.PM_AE_UG_1_0);
    lv_label_set_text_fmt(ui_pm100_value, "%d", data.PM_AE_UG_10_0);
  } else {
    lv_label_set_text(ui_pm25_value, "");
    lv_label_set_text(ui_pm10_value, "");
    lv_label_set_text(ui_pm100_value, "");
  }
}

void wifi_update_timer(lv_timer_t * timer) {
  if (!WiFi.isConnected()) {
    if (lv_obj_has_flag(ui_wifi_icon, LV_OBJ_FLAG_HIDDEN)) {
      lv_obj_clear_flag(ui_wifi_icon, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(ui_wifi_icon, LV_OBJ_FLAG_HIDDEN);
    }
  } else {
    lv_obj_clear_flag(ui_wifi_icon, LV_OBJ_FLAG_HIDDEN);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_STA);
  /*Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());*/
  
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

  for (uint8_t i=0;i<3;i++) {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, light_address[i], 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;    
    esp_now_add_peer(&peerInfo);
  }
  
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
  
  // get last status
  esp_now_send(broadcast_address, (uint8_t *) verify_code_out, strlen(verify_code_out)); 
  
  // Add sensor read and update to UI
  sensor_update_timer(NULL);
  pm_update();
  lv_timer_create(sensor_update_timer, 1000, NULL);
  
  // Add update wifi status timer
  lv_timer_create(wifi_update_timer, 300, NULL);
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
      }
    }
  }

  { // Status icon
    if (cloud_sending) {
      lv_obj_clear_flag(ui_upload_icon, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(ui_upload_icon, LV_OBJ_FLAG_HIDDEN);
    }
  }
  delay(10);
}