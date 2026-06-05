#include "buzzer.h"
#include "ti_msp_dl_config.h"

extern volatile uint32_t sys_tick_ms;
static uint32_t buzzer_start;

void buzzer_init(void)
{
    DL_GPIO_setPins(buzzer_PORT, buzzer_BUZZER_PIN);
}

void buzzer_beep(void)
{
    buzzer_start = sys_tick_ms;
    DL_GPIO_clearPins(buzzer_PORT, buzzer_BUZZER_PIN);
}

void buzzer_tick(void)
{
    if (sys_tick_ms - buzzer_start >= 500)
        DL_GPIO_setPins(buzzer_PORT, buzzer_BUZZER_PIN);
}
