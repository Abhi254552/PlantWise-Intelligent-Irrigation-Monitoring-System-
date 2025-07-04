#ifndef __MQTT_H
#define __MQTT_H

#include <stdbool.h>
#include "main.h"

extern volatile bool esp_raw_busy;
extern volatile bool esp_busy;

void wifi_init(void);           
void mqtt_init(void);             

/* publish helpers (raw for JSON, quoted for short strings)           */
void mqtt_publish_raw(const char *topic,
                      const char *payload,
                      uint8_t     retain);
void mqtt_publish     (const char *topic,
                       const char *payload);

/* call from main loop to chew UART bytes (IRQ already does most)     */
void mqtt_process_uart(void);

/* ------------------------------------------------------------------ */
/*        Extras exposed for other modules (relay, etc.)              */
/* ------------------------------------------------------------------ */

/* Send one AT command + CRLF (blocking)                              */
void esp_send(const char *cmd);

/* Enter / leave low-power RF mode (light-sleep + 10 dBm)             */
void esp_wifi_lowpower(bool enable);

/* internal line parser (called by IRQ)                               */
void mqtt_process_line(char *line);

#endif /* __MQTT_H */
