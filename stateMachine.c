#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stateMachine.h"

#define LED GPIO_NUM_11

extern volatile float volume1;
extern volatile float volume2;
extern volatile float volume3;

static States actualState = ST0_WAIT;

static int sequencia[4]= {-1,-1,-1,-1};
static int estado_led = 0;
static const int PASSWORD_OPEN[4] = {1, 1, 1, 2};
static const int PASSWORD_FECHAR[4] = {0, 0, 0, };
static bool porta_esta_aberta = false;




void ledconfig(){
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 0);
}

void main_estados(){

    static int num_lidos = 0;
    static int cmp[] = {0,0};
    static int son_detectado;
    static bool inicio = false;

    if (!inicio) {
        ledconfig();
        inicio = true;
    }
   
    switch (actualState){
        
        case ST0_WAIT:
        { 
            float vtotal = volume1 + volume2 + volume3;
            printf("VT: %.0f | V1: %.0f | V2: %.0f | V3: %.0f\n", vtotal/100, volume1/100, volume2/100, volume3/100);
            
            son_detectado = -1;
            
            /*Identifica o som*/
            if (volume1 > 10000000 && volume1 > volume2 && volume1 > volume3) son_detectado = 0;
            else if (volume2 > 10000000 && volume2 > volume1 && volume2 > volume3) son_detectado = 1;
            else if (volume3 > 10000000 && volume3 > volume1 && volume3 > volume2) son_detectado = 2;

          
                
                // Se for um som válido (0, 1 ou 2) 
                if (son_detectado != -1 && num_lidos < 4) {
                    sequencia[num_lidos] = son_detectado;
                    num_lidos++;
                    printf("\n**********Símbolo %d detectado (%d/4)************\n\n", son_detectado, num_lidos);
                    printf("Sequência atual: [%d, %d, %d, %d]\n\n", sequencia[0], sequencia[1], sequencia[2], sequencia[3]);
                }
                

            if (num_lidos == 4) {
                actualState = ST1_VALIDATE;
            }
            break;
        }

        case ST1_VALIDATE:
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
            porta_esta_aberta = true;
            estado_led = 1;
            gpio_set_level(LED, estado_led);
            actualState = ST0_WAIT;
            break;

        case ST3_CLOSE:
            printf("\n PORTA FECHADA \n\n");
            porta_esta_aberta = false;
            estado_led = 0;
            gpio_set_level(LED, estado_led);
            actualState = ST0_WAIT;
            break;

        case ST4_ERR:
            printf("\n Sequência errada \n");
            
            for(int i = 0; i < 10; i++) {
                estado_led = !estado_led;
                gpio_set_level(LED, estado_led);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            
            gpio_set_level(LED, porta_esta_aberta ? 1 : 0);
            
            printf("Sistema pronto. Tente novamente!\n");
            actualState = ST0_WAIT;
            break;

        default:
            break;
    }
}