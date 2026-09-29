#pragma once
#include <Arduino.h>
#include <map>
#include <string>
#define FILE_APPEND "a"
#define FILE_WRITE  "w"
extern bool g_sdPresent;
inline std::map<std::string, std::string> &sdfs() { static std::map<std::string, std::string> m; return m; }
struct File {
  std::string path; bool ok = false;
  operator bool() const { return ok; }
  size_t write(const uint8_t *d, size_t n) { sdfs()[path].append((const char *)d, n); return n; }
  void close() {}
};
struct SDClass {
  bool begin(int, SPIClass &, uint32_t) { return g_sdPresent; }
  bool exists(const char *p) { return sdfs().count(p) != 0; }
  File open(const char *p, const char *mode) {
    File f; f.path = p;
    if (!g_sdPresent) return f;
    if (strcmp(mode, "w") == 0) sdfs()[p] = "";
    else if (!sdfs().count(p)) return f;
    f.ok = true; return f;
  }
};
extern SDClass SD;
