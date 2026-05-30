#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

#include "stateMachine.h"
#define LED GPIO_NUM_11

extern QueueHandle_t XQ;

static States actualState = ST0_WAIT;

static int sequencia_lida[4] = {-1, -1, -1, -1};
static int num_lidos = 0;


static const int PASSWORD_OPEN[4] = {1, 0, 1, 2};
static const int PASSWORD_FECHAR[4] = {0, 0, 1, 2};


void pv_processor_task(void *pvParam)
{
    float * sound_samp_buf_proc;
    sound_samp_buf_proc = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);         
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 0); 
    for(;;) {
        switch (actualState) {
            
            case ST0_WAIT:
                if (xQueueReceive(XQ, (void*)sound_samp_buf_proc, portMAX_DELAY) == pdTRUE) {
                    
            
                    static int sequencia_teste[4] = {1, 0, 1, 2}; 
                    static int indice_teste = 0;

                    int novo_simbolo = sequencia_teste[indice_teste];
                    indice_teste = (indice_teste + 1) % 4; 
                    

                    if (novo_simbolo != -1) {
                        sequencia_lida[num_lidos] = novo_simbolo;
                        num_lidos++;
                        printf("Símbolo %d detectado (%d/4)\n", novo_simbolo, num_lidos);
                    }
                    
                    if (num_lidos == 4) {
                        actualState = ST1_VALIDATE;
                    }
                }
                break;

            case ST1_VALIDATE:
                if (memcmp(sequencia_lida, PASSWORD_OPEN, sizeof(PASSWORD_OPEN)) == 0) {
                    actualState = ST2_OPEN;
                } 
                else if (memcmp(sequencia_lida, PASSWORD_FECHAR, sizeof(PASSWORD_FECHAR)) == 0) {
                    actualState = ST3_CLOSE;

                } 
                else {
                    actualState = ST4_ERR;
                }
        
                num_lidos = 0;
                memset(sequencia_lida, -1, sizeof(sequencia_lida));
                break;
    
            case ST2_OPEN:
                printf("\n>>> PORTA ABERTA <<<\n\n");
                gpio_set_level(LED,1);
                vTaskDelay(pdMS_TO_TICKS(3000));
                gpio_set_level(LED,0);
                

                actualState = ST0_WAIT;
                break;

            case ST3_CLOSE:
                printf("\n>>> PORTA FECHADA <<<\n\n");

                vTaskDelay(pdMS_TO_TICKS(3000));
                actualState = ST0_WAIT;
                break;

            case ST4_ERR:
                printf("\n!!! ERRO!!!\n\n");
                vTaskDelay(pdMS_TO_TICKS(3000));
                actualState = ST0_WAIT;
                break;
        }
    }
}