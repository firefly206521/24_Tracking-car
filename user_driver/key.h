#ifndef KEY_H
#define KEY_H
#include "ti_msp_dl_config.h"

extern int status;
extern int start_flag;

uint8_t get_key_value(uint32_t key);
#endif /* KEY_H */