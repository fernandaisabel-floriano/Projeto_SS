#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "driver/gptimer.h"

#include "stateMachine.h"
#define LED GPIO_NUM_11

extern float volume1;
extern float volume2;
extern float volume3;

static States actualState = ST0_WAIT;

volatile int num_lidos = 0;
volatile int sequencia[4];
volatile int estado_led = 0;
static const int PASSWORD_OPEN[4] = {1, 0, 1, 2};
static const int PASSWORD_FECHAR[4] = {0, 0, 1, 2};

/*tem que estar fora da main pra que a isr de T2 possa reconhecer os timers*/
gptimer_handle_t T1 = NULL;
gptimer_handle_t T2 = NULL;

static bool timer_isr_callback1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    
    estado_led = !estado_led;
    gpio_set_level(LED, estado_led);

    return false;
};
static bool timer_isr_callback2(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gptimer_stop(T2);
    gptimer_stop(T1);
    return false;
}

void timerConfig(){
    gptimer_config_t Timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select the default clock source
    .direction = GPTIMER_COUNT_UP,      // Counting direction is up
    .resolution_hz = 20000   // Resolution is 20 kHz, i.e., 1 tick equals 50 microsecond
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&Timer_config, &T1));
    ESP_ERROR_CHECK(gptimer_new_timer(&Timer_config, &T2));

    gptimer_event_callbacks_t cbs1 = {
        .on_alarm = timer_isr_callback1, 
    };

    gptimer_event_callbacks_t cbs2 = {
        .on_alarm = timer_isr_callback2,
    };


    ESP_ERROR_CHECK(gptimer_register_event_callbacks(T1, &cbs1, NULL));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(T2, &cbs2, NULL));

    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 5000,             // O alarme dispara a cada 0.25 segundo.
        .reload_count = 0,                  // Contagem recomeça no tique 0.
        .flags.auto_reload_on_alarm = true, // Recomeça contagem automaticamente após alarme.
    };
    gptimer_alarm_config_t alarm_config2 = {
        .alarm_count = 100000,              // O alarme dispara depois de 5 segundos.
        .reload_count = 0,                   // reseta o contador
        .flags.auto_reload_on_alarm = false, // Recomeça contagem automaticamente após alarme.
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(T1, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(T2, &alarm_config2));

    ESP_ERROR_CHECK(gptimer_enable(T1));
    ESP_ERROR_CHECK(gptimer_enable(T2));
}

void ledconfig(){
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
}

void main_estados(){

    timerConfig();
    ledconfig();

    int cmp[] = {0,0};

while(1){    
    switch (actualState){

        case ST0_WAIT:
            while(num_lidos <= 3){
                if (volume1 > 1000)
                {
                    sequencia[num_lidos] = 0;
                    num_lidos++;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                else if (volume2 > 1000)
                {
                    sequencia[num_lidos] = 1;
                    num_lidos++;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                else if (volume3 > 1000){
                    sequencia[num_lidos] = 2;
                    num_lidos++;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
            }
            actualState = ST1_VALIDATE;
            break;

        case ST1_VALIDATE:
            cmp[0] = memcmp(sequencia,PASSWORD_OPEN,sizeof(sequencia));
            cmp[1] = memcmp(sequencia,PASSWORD_FECHAR,sizeof(sequencia));
            if (cmp[0] == 0)
            {
                actualState = ST2_OPEN;
            }
            else if (cmp[1] == 0)
            {
                actualState = ST3_CLOSE;
            }
            else
            {
                actualState = ST4_ERR;
            }
            memset((void*)sequencia, 0, sizeof(sequencia));
            cmp[0] = 0;
            cmp[1] = 0;
            num_lidos = 0;
            break;

        case ST2_OPEN:
            estado_led = 1;
            gpio_set_level(LED, estado_led);
            actualState = ST0_WAIT;
            break;

        case ST3_CLOSE:
            estado_led = 0;
            gpio_set_level(LED,estado_led);
            actualState = ST0_WAIT;
            break;

        case ST4_ERR:
            ESP_ERROR_CHECK(gptimer_start(T1));
            ESP_ERROR_CHECK(gptimer_start(T2));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            actualState = ST0_WAIT;
            break;

        default:
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

