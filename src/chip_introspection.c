#include <stdio.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void print_device_info() {
  printf("ESP-IDF version: %s\n", esp_get_idf_version());

  // Mac address = chip ID
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  printf("MAC address: %02x%02x%02x%02x%02x%02x\n", mac[0], mac[1], mac[2],
         mac[3], mac[4], mac[5]);

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("Chip info:\n");
  printf(" - Si revision %d\n", chip_info.revision);
  printf(" - %d cores\n", chip_info.cores);
  printf(" - BT: %s\n", chip_info.features & CHIP_FEATURE_BT ? "yes" : "no");
  printf(" - BLE: %s\n", chip_info.features & CHIP_FEATURE_BLE ? "yes" : "no");
  printf(" - Flash: %d MB %s\n", spi_flash_get_chip_size() / 1024 / 1024,
         chip_info.features & CHIP_FEATURE_EMB_FLASH ? "embedded" : "external");
}

void print_memory() {
  // NULL for the task handle indicates current task
  ESP_LOGI("MEMORY", "stack %d kB, total RAM %d kB (internal %d, external %d)",
           uxTaskGetStackHighWaterMark(NULL) / 1024,
           heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024,
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
           heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
}
