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
uint8_t light_address[5][6] = { 
  { 0x5C, 0xCF, 0x7F, 0x3C, 0x1F, 0xAF }, // หน้า
  { 0x5C, 0xCF, 0x7F, 0x3B, 0xCA, 0x53 }, // กลาง
  { 0x5C, 0xCF, 0x7F, 0x3B, 0xC7, 0x64 }, // หลัง
  { 0x5C, 0xCF, 0x7F, 0xF9, 0xA6, 0xA4 }, // ลอยหน้า
  { 0x5C, 0xCF, 0x7F, 0x3C, 0x21, 0x25 }  // ลอยหลัง
};

uint8_t door_sensor_address[2][6] = { 
  { 0x24, 0x6F, 0x28, 0x25, 0xE8, 0x40 } // Door Sensor 1
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
  String in_data = "";
  for (uint8_t i=0;i<len;i++) {
    in_data += (char) incomingData[i];
  }

  if (in_data.startsWith(verify_code_in)) { // light
    String payload = in_data.substring(strlen(verify_code_in));

    if (payload.length() > 0) {
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
  } else if (in_data.startsWith(verify_code_door_in)) { // door
    String payload = in_data.substring(strlen(verify_code_door_in));
    
    if (payload.length() > 0) {
      bool output_status = payload.toInt() == 1;
      lv_obj_t * ui_door_x_status[] = {
        ui_door1_status,
        ui_door2_status
      };
      for (uint8_t i=0;i<1;i++) {
        if (memcmp(door_sensor_address[i], mac, 6) == 0) {
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

void setup() {
  Serial.begin(115200);

  nowQueue = xQueueCreate(20, sizeof(NOWMessage));

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

  for (uint8_t i=0;i<5;i++) {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, light_address[i], 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;    
    esp_now_add_peer(&peerInfo);
  }

  for (uint8_t i=0;i<1;i++) {
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
  lv_obj_add_event_cb(ui_light4_sw, light_sw_click_handle, LV_EVENT_CLICKED, (void*) 3);
  lv_obj_add_event_cb(ui_light5_sw, light_sw_click_handle, LV_EVENT_CLICKED, (void*) 4);

  // get last status
  esp_now_send(broadcast_address, (uint8_t *) verify_code_out, strlen(verify_code_out)); 
  esp_now_send(broadcast_address, (uint8_t *) verify_code_door_in, strlen(verify_code_door_in)); 
  
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
        Display.on();
        digitalWrite(LCD_BL_PIN, HIGH);
        display_enter_to_sleep = false;
      }
    }
  }

  delay(10);
}
