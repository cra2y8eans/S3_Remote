/**
 * ESP32S3 模块化编程多功能遥控器  分支： main
 *
 *
 * 将遥控器功能分为OLED显示、按钮判断、蜂鸣器、ESP NOW通讯等功能模块，通过freeRTOS任务运行
 * 将解锁操作整合到OLED任务中，将钮子开关判断、电池电量接收和读取整合到ESP NOW任务中
 * 各功能模块通过队列通信，实现模块间解耦
 */

#include "ESP_NOW/sendData.h"
#include "button/button.h"
#include "buzzer/buzzer.h"
#include "oled/oled.h"
#include <Arduino.h>

#define ADC_RESOLUTION 12

void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_RESOLUTION);
  oled_init();
  sendData_init();
  buzzer_init();
  button_init();
  vTaskDelete(NULL);
}
void loop() {
}
