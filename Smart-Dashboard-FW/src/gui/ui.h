// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: Dashboard

#ifndef _DASHBOARD_UI_H
#define _DASHBOARD_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"
// SCREEN: ui_Index
void ui_Index_screen_init(void);
extern lv_obj_t * ui_Index;
extern lv_obj_t * ui_topbar;
extern lv_obj_t * ui_Container7;
extern lv_obj_t * ui_Sensor_Group;
extern lv_obj_t * ui_Panel5;
extern lv_obj_t * ui_Image4;
extern lv_obj_t * ui_Container4;
extern lv_obj_t * ui_Container3;
extern lv_obj_t * ui_temp_value;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_Container2;
extern lv_obj_t * ui_humi_value;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_Panel6;
extern lv_obj_t * ui_Image5;
extern lv_obj_t * ui_Container1;
extern lv_obj_t * ui_Container5;
extern lv_obj_t * ui_Label14;
extern lv_obj_t * ui_pm25_value;
extern lv_obj_t * ui_Container6;
extern lv_obj_t * ui_Label11;
extern lv_obj_t * ui_pm10_value;
extern lv_obj_t * ui_Label12;
extern lv_obj_t * ui_pm100_value;
extern lv_obj_t * ui_Binary_Sensor_Group;
extern lv_obj_t * ui_sensor_door_group;
extern lv_obj_t * ui_door1_status;
extern lv_obj_t * ui_door1_label;
extern lv_obj_t * ui_door2_status;
extern lv_obj_t * ui_door2_label;
extern lv_obj_t * ui_pir_status;
extern lv_obj_t * ui_pir_label;
extern lv_obj_t * ui_Lamp_Group;
void ui_event_light1_sw(lv_event_t * e);
extern lv_obj_t * ui_light1_sw;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_light1_img;
void ui_event_light2_sw(lv_event_t * e);
extern lv_obj_t * ui_light2_sw;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_light2_img;
void ui_event_light3_sw(lv_event_t * e);
extern lv_obj_t * ui_light3_sw;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_light3_img;
void ui_event_light4_sw(lv_event_t * e);
extern lv_obj_t * ui_light4_sw;
extern lv_obj_t * ui_Label4;
extern lv_obj_t * ui_light4_img;
void ui_event_light5_sw(lv_event_t * e);
extern lv_obj_t * ui_light5_sw;
extern lv_obj_t * ui_Label6;
extern lv_obj_t * ui_light5_img;
extern lv_obj_t * ui____initial_actions0;

LV_IMG_DECLARE(ui_img_temperatures_png);    // assets\temperatures.png
LV_IMG_DECLARE(ui_img_dust_png);    // assets\dust.png
LV_IMG_DECLARE(ui_img_111191404);    // assets\closed-door-with-border-silhouette.png
LV_IMG_DECLARE(ui_img_1857898281);    // assets\opened-door-aperture.png
LV_IMG_DECLARE(ui_img_1821463755);    // assets\bulb-icon.png

LV_FONT_DECLARE(ui_font_Kanit18);
LV_FONT_DECLARE(ui_font_Kanit24);
LV_FONT_DECLARE(ui_font_Segment32);
LV_FONT_DECLARE(ui_font_Segment42);

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
