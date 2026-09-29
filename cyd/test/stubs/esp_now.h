#pragma once
#include <Arduino.h>
#define ESP_OK 0
typedef struct { uint8_t peer_addr[6]; uint8_t channel; bool encrypt; int ifidx; } esp_now_peer_info_t;
void harness_tx(const uint8_t *mac, const uint8_t *d, int n);
inline int  esp_now_init() { return ESP_OK; }
inline bool esp_now_is_peer_exist(const uint8_t *) { return false; }
inline int  esp_now_add_peer(const esp_now_peer_info_t *) { return ESP_OK; }
inline int  esp_now_register_recv_cb(void (*)(const uint8_t *, const uint8_t *, int)) { return ESP_OK; }
inline int  esp_now_send(const uint8_t *mac, const uint8_t *d, size_t n) { harness_tx(mac, d, (int)n); return ESP_OK; }
