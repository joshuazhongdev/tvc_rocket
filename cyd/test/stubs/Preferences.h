#pragma once
#include <Arduino.h>
#include <map>
#include <vector>
inline std::map<std::string, std::vector<uint8_t>> &nvs() {
  static std::map<std::string, std::vector<uint8_t>> m; return m;
}
struct Preferences {
  void begin(const char *, bool = false) {}
  void end() {}
  bool isKey(const char *k) { return nvs().count(k) != 0; }
  size_t getBytesLength(const char *k) { return nvs().count(k) ? nvs()[k].size() : 0; }
  size_t getBytes(const char *k, void *d, size_t n) {
    if (!nvs().count(k) || nvs()[k].size() != n) return 0;
    memcpy(d, nvs()[k].data(), n); return n;
  }
  void putBytes(const char *k, const void *d, size_t n) {
    nvs()[k].assign((const uint8_t *)d, (const uint8_t *)d + n);
  }
  float getFloat(const char *, float dv) { return dv; }
  void putFloat(const char *, float) {}
};
