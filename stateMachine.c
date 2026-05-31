#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "driver/gptimer.h"

#include "stateMachine.h"
#define LED GPIO_NUM_11

extern volatile float volume1;
extern volatile float volume2;
extern volatile float volume3;

static States actualState = ST0_WAIT;

static int sequencia[4]= {-1,-1,-1,-1};
static int estado_led = 0;
static const int PASSWORD_OPEN[4] = {1, 0, 1, 2};
static const int PASSWORD_FECHAR[4] = {0, 0, 1, 2};

/*tem que estar fora da main pra que a isr de T2 possa reconhecer os timers*/
gptimer_handle_t T1 = NULL;
gptimer_handle_t T2 = NULL;
static volatile bool alarme = false;

static bool timer_isr_callback1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    
    estado_led = !estado_led;
    gpio_set_level(LED, estado_led);

    return false;
};
static bool timer_isr_callback2(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
   alarme = true;
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
    gpio_set_level(LED,0);
}

void main_estados(){
    static int num_lidos = 0;
    static int cmp[] = {0,0};
    static int son_detectado;
    static int silencio = 0;
static bool inicio = false;
    if (!inicio) {
        timerConfig();
        ledconfig();
        inicio = true;
    }
   
  
    
    switch (actualState){
        /*detecta se existe */
        case ST0_WAIT:

            float vtotal = volume1+volume2+volume3;

            printf("VT: %.1f V1: %.1f | V2: %.1f | V3: %.1f s:%d \n",vtotal, volume1, volume2, volume3,silencio);
            printf("nemeros adicionados %d /4 \n",num_lidos); 

            if(num_lidos < 4 && silencio == 0){

              
                son_detectado = -1;
                /*Identifica o som*/
                if (volume1 > 10000000 && volume1>volume2 && volume1>volume3)
                {   sequencia[num_lidos] = 0;
                    son_detectado = 0;
                
                }
                else if (volume2 > 10000000 && volume2 > volume1 && volume2 > volume3)
                {
                    sequencia[num_lidos] = 1;
                    son_detectado = 1;
                    
                }
                else if (volume3 > 10000000 && volume3 > volume1 && volume3 > volume2){
                    sequencia[num_lidos] = 2;
                    son_detectado = 2;
                }

                if(son_detectado != -1){
                    num_lidos++;
                }
                
                /*enquanto houver son + ruido não passa pra frente, é preciso um pequeno tempo de silencio para adicionar outra */
                if(vtotal > 10000000){
                silencio = 1; 
            }
            else{
                silencio = 0;
            }

            }

            if (num_lidos == 4) {
                actualState = ST1_VALIDATE;
            }
            
            break;

            /*compara a sequencia criada e as PASSWORDs e troca de estado de acordo*/
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
            /*reseta a sequencia, as comparações e o nº de valores lidos*/
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
            printf("\n-------ERRO-------\n");
            
            alarme= false;
            gptimer_set_raw_count(T1, 0); 
            gptimer_set_raw_count(T2, 0);
            ESP_ERROR_CHECK(gptimer_start(T1));
            ESP_ERROR_CHECK(gptimer_start(T2));
            
            while(!alarme) {
                vTaskDelay(pdMS_TO_TICKS(50)); 
            }
            
            gptimer_stop(T1);
            gptimer_stop(T2);
            
    
            gpio_set_level(LED, estado_led);
            actualState = ST0_WAIT;
            break;

        default:
            break;
        }
     
    }

