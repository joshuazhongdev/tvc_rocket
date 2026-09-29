#pragma once
#include <string>
#include <vector>
#include <cstring>

struct RocketMock {
  bool inBench = false;
  bool armed = false;
  bool zeroed = true;
  bool inhibit = false;
  bool sfMode = false;
  bool onPad = true;
  float offP = 10.0f, offY = 10.0f;
  std::vector<std::string> accepted;
  std::vector<std::string> swallowed;
  std::string pending;

  static const char *BENCH_CMDS;

  void feed(const uint8_t *d, int n) {
    pending.append((const char *)d, n);
    drain();
  }

  void drain() {
    while (!pending.empty()) {
      size_t nl = pending.find('\n');
      if (nl == std::string::npos) return;
      std::string line = pending.substr(0, nl);
      pending.erase(0, nl + 1);
      while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
      if (line.empty()) continue;
      dispatch(line);
    }
  }

  void dispatch(const std::string &line) {
    char c = line[0];
    std::string arg = line.substr(1);

    if (inBench) {
      if (c == 'q') { inBench = false; accepted.push_back(line); return; }
      if (strchr(BENCH_CMDS, c)) { accepted.push_back(line); return; }
      swallowed.push_back(line);
      return;
    }
    switch (c) {
      case 't':
        if (!armed && onPad) { inBench = true; armed = false; accepted.push_back(line); }
        else swallowed.push_back(line);
        return;
      case 'B':
        if (!onPad || armed) { swallowed.push_back(line); return; }
        armed = true; sfMode = true; inhibit = false; accepted.push_back(line); return;
      case 'A': case 'a':
        if (!onPad || !zeroed || armed) { swallowed.push_back(line); return; }
        armed = true; inhibit = false; accepted.push_back(line); return;
      case 'D': case 'd':
        armed = false; inhibit = true; sfMode = false; accepted.push_back(line); return;
      case 'K': case 'k':
        if (armed) { swallowed.push_back(line); return; }
        zeroed = true; accepted.push_back(line); return;
      case 'O': case 'o': {
        if (armed) { swallowed.push_back(line); return; }
        float a = 0, b = 0;
        if (sscanf(arg.c_str(), "%f,%f", &a, &b) == 2) {
          offP = a < -25 ? -25 : a > 25 ? 25 : a;
          offY = b < -25 ? -25 : b > 25 ? 25 : b;
        }
        accepted.push_back(line); return;
      }
      case 'r': case 'f': case 'b':
        accepted.push_back(line); return;
      default:
        swallowed.push_back(line); return;
    }
  }
};
const char *RocketMock::BENCH_CMDS = "12345678CcGgWwSsLlFfEeMmTtpid?zZ9";
