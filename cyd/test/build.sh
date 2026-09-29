#!/bin/bash
# Fresh copy every run so the tests always exercise the committed source.
set -e
rm -rf build && mkdir -p build
cp ../src/main.cpp ../src/sdlog.h ../src/linkproto.h build/
cp stubs/display.h stubs/touch.h build/     # shadow the two hardware headers
g++ -std=c++17 -I stubs -I build -o test_ui test_ui.cpp
g++ -std=c++17 -I stubs -I ../src -o test_touch test_touch.cpp
