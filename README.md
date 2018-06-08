# esp32-max30100-cpp

MAX30100 C++ Library for ESP32. This is made based on the Raivis Strogonovs' tutorial available at https://morf.lv/implementing-pulse-oximeter-using-max30100 with an Arduino code at https://github.com/xcoder123/MAX30100.

## Adding to your project

You need to clone this project and it's dependency to your project's components folder.
Inside you project folder:
```
mkdir -p components && cd components
git clone https://github.com/aedalzotto/esp32-max30100-cpp.git
git clone https://github.com/aedalzotto/esp32-i2c_device-cpp.git
```
If you prefer you can use submodule instead of cloning:
```
mkdir -p components && cd components
git submodule add https://github.com/aedalzotto/esp32-max30100-cpp.git
git submodule add https://github.com/aedalzotto/esp32-i2c_device-cpp.git
```

There is no need for additional configuration.

## Examples

Examples are available at https://github.com/aedalzotto/esp32-i2c_device-cpp
