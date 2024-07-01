# Projekt ASE

## Board used

ESP32-DevKitC V4 with ESP-WROOM-32E module on it

<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html>

<https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf>

## Bottom IR sensors

Iduino IR sensors with black PCB upon detection detection pull signal line to GND and when not detecting the state may be not well defined due to poor on-board performance. Controller's input pins should be pulled up.
