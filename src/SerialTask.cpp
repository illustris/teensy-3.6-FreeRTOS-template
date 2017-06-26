#include <FreeRTOS.h>
#include <task.h>

#include <Arduino.h>
#include <usb_dev.h>

void SerialTask(void* args) {
  // initialize the serial
  usb_init();
  Serial1.begin(9600);

  for (;;) {
    Serial1.println("Hello World");
    vTaskDelay(1000);
  }
}
