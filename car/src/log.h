/*
 * Stand-in for Zephyr logging: one "[module] message" line per call on the
 * USB serial port, at 115200 baud. Each file names its module once with
 * LOG_MODULE("name").
 */
#ifndef LOG_H
#define LOG_H

#include <Arduino.h>

#define LOG_MODULE(name) static const char *const log_module = name

#define LOG_INF(fmt, ...) Serial.printf("[%s] " fmt "\n", log_module, ##__VA_ARGS__)
#define LOG_WRN(fmt, ...) Serial.printf("[%s] warning: " fmt "\n", log_module, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) Serial.printf("[%s] error: " fmt "\n", log_module, ##__VA_ARGS__)

#endif /* LOG_H */
