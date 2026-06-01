#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MICEX_SOUND_SAMPLES_BUF_SIZE 2048


typedef enum {
    ST0_WAIT,
    ST1_VALIDATE,
    ST2_OPEN,
    ST3_CLOSE,
    ST4_ERR
} States;

void pv_processor_task(void *pvParam);
void main_estados(void);

#endif