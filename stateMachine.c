#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
static const int PASSWORD_FECHAR[4] = {0, 1, 2,0};
static bool portaAberta = false;

gptimer_handle_t T1 = NULL;


static bool timer_isr_callback1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    
    estado_led = !estado_led;
    gpio_set_level(LED, estado_led);

    return false;
};


void timerConfig(){
    gptimer_config_t Timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select the default clock source
    .direction = GPTIMER_COUNT_UP,      // Counting direction is up
    .resolution_hz = 20000   // Resolution is 20 kHz, i.e., 1 tick equals 50 microsecond
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&Timer_config, &T1));
    

    gptimer_event_callbacks_t cbs1 = {
        .on_alarm = timer_isr_callback1, 
    };

   


    ESP_ERROR_CHECK(gptimer_register_event_callbacks(T1, &cbs1, NULL));
   

    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 5000,             
        .reload_count = 0,                  
        .flags.auto_reload_on_alarm = true, // Recomeça contagem automaticamente após alarme.
    };
   

    ESP_ERROR_CHECK(gptimer_set_alarm_action(T1, &alarm_config1));
    
    ESP_ERROR_CHECK(gptimer_enable(T1));
  
}



// função de configuração do led
void ledconfig(){
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 0);
}

// maquina de estados
void main_estados(){

    static int num_lidos = 0;
    static int cmp[] = {0,0};
    static int son_detectado;
    static int silencio = 1;
    static bool inicio = false;


    if (!inicio) {
        ledconfig();
        timerConfig();
        inicio = true;
    }
   
    switch (actualState){
        
        case ST0_WAIT:
        { 
            float vtotal = volume1 + volume2 + volume3;
            printf("VT: %.0f | V1: %.0f | V2: %.0f | V3: %.0f\n", vtotal, volume1, volume2, volume3);
            
            son_detectado = -1;
            
            /*Identifica o som*/
           if(silencio == 1){
                if (volume1 > 1000 && volume1 > volume2 && volume1 > volume3) son_detectado = 0;
                else if (volume2 > 1000 && volume2 > volume1 && volume2 > volume3) son_detectado = 1;
                else if (volume3 > 1000 && volume3 > volume1 && volume3 > volume2) son_detectado = 2;
           }
          
                
                // Se for um som válido (0, 1 ou 2) 
                if (son_detectado != -1 && num_lidos < 4) {
                    sequencia[num_lidos] = son_detectado;
                    num_lidos++;
                    printf("\n**********Símbolo %d detectado (%d/4)************\n\n", son_detectado, num_lidos);
                    printf("Sequência atual: [%d, %d, %d, %d]\n\n", sequencia[0], sequencia[1],sequencia[2],sequencia[3]);
                }


                if(vtotal > 25000){
                    printf("\n barulho \n");
                    silencio = 0;}
                else{silencio = 1;}
                    
            if (num_lidos == 4) {
                actualState = ST1_VALIDATE;
            }
            break;
        }

        case ST1_VALIDATE:
            printf("\nVALIDATE\n");
            cmp[0] = memcmp(sequencia, PASSWORD_OPEN, sizeof(PASSWORD_OPEN));
            cmp[1] = memcmp(sequencia, PASSWORD_FECHAR, sizeof(PASSWORD_FECHAR));
            
            if (cmp[0] == 0) actualState = ST2_OPEN;
            else if (cmp[1] == 0) actualState = ST3_CLOSE;
            else actualState = ST4_ERR;
            
            memset((void*)sequencia, -1, sizeof(sequencia));
            num_lidos = 0;
            
            break;

        case ST2_OPEN:
            printf("\n PORTA ABERTA \n\n");
            portaAberta = true;
            estado_led = 1;
            gpio_set_level(LED, estado_led);
            actualState = ST0_WAIT;
            break;

        case ST3_CLOSE:
            printf("\n PORTA FECHADA \n\n");
            portaAberta = false;
            estado_led = 0;
            gpio_set_level(LED, estado_led);
            actualState = ST0_WAIT;
            break;

        case ST4_ERR:
            printf("\n !!! Sequência errada  !!!\n");     
            gptimer_start(T1);   
            vTaskDelay(pdMS_TO_TICKS(5000));
            gptimer_stop(T1);
            gptimer_set_raw_count(T1, 0); 

            // para garantir q o Led não acenda se a porta estiver fechada
            if (portaAberta == true) {
                gpio_set_level(LED, 1);
            } else {
                gpio_set_level(LED, 0);
            }
           printf("********************************************\n");
            actualState = ST0_WAIT;
            break;
        default:
            break;
    }
}