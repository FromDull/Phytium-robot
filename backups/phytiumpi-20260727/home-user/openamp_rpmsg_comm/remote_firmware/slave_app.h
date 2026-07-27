#ifndef SLAVE_APP_H
#define SLAVE_APP_H

#include <stddef.h>
#include <stdint.h>

size_t slave_handle_frame(const uint8_t *data, unsigned int len,
                          uint8_t *reply, size_t reply_size);
int slave_app_init(void);
void slave_app_poll(void);
void slave_app_shutdown(void);

#endif
