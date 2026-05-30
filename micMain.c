/* ********************************************************************************************************************************* * Microphone test - ADC in continuous mode and time-domain BP filtering 
 ***********************************************************************************************************************************/ 




 // Bibliotecas padrão do C
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // tarefas
#include "freertos/semphr.h" // semaforos
#include "freertos/queue.h" // filas entre tarefas
#include "esp_adc/adc_continuous.h" // ler continuamente a ADC usando DMA
#include "esp_dsp.h"
#include "esp_private/esp_clk.h"
#include "stateMachine.h"

// config da adc

#define MICEX_ADC_UNIT                    ADC_UNIT_1  
#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#define MICEX_ADC_FRAME_SIZE             512 // tamanho do frame ou seja, o adc vai ler 512 amostras por leitura
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE) // 248
#define MICEX_ADC_SAMPLE_FREQ            (20 * 1000) // adc captura 20000 sa/s

static adc_channel_t channel[1] = {ADC_CHANNEL_3}; // canal adc é o 3.
static TaskHandle_t s_task_handle; // para identificar a tarefa principal(por meio de isr)
static const char *TAG = "MIC_EXAMPLE"; // sistema de print?? ou mensagem da esp32

__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0}; // para alinhar a memoria
__attribute__((aligned(16))) adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES];

#define PROCESSOR_TASK_STACK_SIZE       8192 // tamanho da fila
#define PROCESSOR_TASK_PRIORITY ( tskIDLE_PRIORITY + 4 ) // define prioridade das tarefas nos FreeRTOS
QueueHandle_t XQ; // fila para as tarefas comunicarem entre si.

__attribute__((aligned(16))) float hbpf2k[]={0.000000139618742, 0.000000255721385, 0.000000140050607, -0.000000328918009, -0.000001079871671, -0.000001790994587, -0.000001943975411, -0.000001031882007, 0.000001111877099, 0.000004001347557, 0.000006413035946, 0.000006707873993, 0.000003574260112, -0.000003048702149, -0.000011330185703, -0.000017688431396, -0.000017969871244, -0.000009435702605, 0.000007215862062, 0.000026871405806, 0.000040972415077, 0.000040614490911, 0.000020897907315, -0.000015285006327, -0.000056126887733, -0.000083842525959, -0.000081437626075, -0.000041101266796, 0.000029421836936, 0.000106148839227, 0.000155757573851, 0.000148773462501, 0.000074351529935, -0.000050052623102, -0.000180121406766, -0.000259437345082, -0.000243952642519, -0.000126937980784, 0.000047492844400, 0.000191919289194, 0.000580032052973, 0.000667352843244, 0.000306111499321, -0.000446298721323, -0.001342483904490, -0.001978250973038, -0.001912439692934,-0.000868574004532, 0.001047898364964, 0.003236203885917, 0.004708723047573, 0.004458080398213, 0.001979374693417, -0.002286980559547, -0.006863696045905, -0.009659616406773, -0.008836721670391, -0.003790852332498, 0.004230959606325, 0.012280364027838, 0.016725773018749, 0.014820278024314, 0.006161630397857, -0.006683261470121, -0.018835296877081, -0.024940029322091, -0.021501888240714, -0.008700432996997, 0.009216737308611, 0.025316468417048, 0.032708573103906, 0.027529205755171, 0.010869128872094, -0.011297781514959, -0.030304625508737, -0.038281206662112, -0.031498011959705, -0.012107397246638, 0.012536609979868, 0.032771101414432, 0.040552804004055, 0.032771101414432, 0.012536609979868, -0.012107397246638, -0.031498011959704, -0.038281206662113, -0.030304625508737, -0.011297781514959, 0.010869128872094, 0.027529205755171, 0.032708573103906, 0.025316468417048, 0.009216737308611, -0.008700432996997, -0.021501888240714, -0.024940029322091, -0.018835296877081, -0.006683261470121, 0.006161630397857, 0.014820278024314, 0.016725773018749, 0.012280364027838,0.004230959606325, -0.003790852332499, -0.008836721670391, -0.009659616406773, -0.006863696045905, -0.002286980559547, 0.001979374693417, 0.004458080398213, 0.004708723047573, 0.003236203885917, 0.001047898364964, -0.000868574004532, -0.001912439692934, -0.001978250973038, -0.001342483904490, -0.000446298721323, 0.000306111499321, 0.000667352843244, 0.000580032052973, 0.000191919289194, 0.000047492844400, -0.000126937980784, -0.000243952642519, -0.000259437345082, -0.000180121406766, -0.000050052623102, 0.000074351529935, 0.000148773462501, 0.000155757573851, 0.000106148839227, 0.000029421836936, -0.000041101266796, -0.000081437626075, -0.000083842525959, -0.000056126887733, -0.000015285006327, 0.000020897907315, 0.000040614490911, 0.000040972415077, 0.000026871405806, 0.000007215862062, -0.000009435702605, -0.000017969871244, -0.000017688431396, -0.000011330185703, -0.000003048702149, 0.000003574260112,0.000006707873993, 0.000006413035946, 0.000004001347557, 0.000001111877099, -0.000001031882007, -0.000001943975411, -0.000001790994587, -0.000001079871671, -0.000000328918009, 0.000000140050607, 0.000000255721385, 0.000000139618742}; // FIR (passa-banda)


// Funcões declaradas antecipadamente só para não dar erro mais a baixo.

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);


// funcao main

void app_main(void)

{   // variaveis locais

    esp_err_t ret; // resultado da leitura da adc
    esp_err_t parse_ret; // "" do parsing
    uint32_t ret_num = 0; // "" numero de bytes lidos
    uint32_t sb_count = 0; // "" contador do buffer
    uint32_t num_parsed_samples = 0; // numero de amostras
    
    // Estrutura da ADC

    adc_continuous_evt_cbs_t cbs; // callbacks
    adc_continuous_handle_t handle = NULL; // objeto do adc continuo
 
    // buffer de audio
    float * sound_samp_buf_ADC;
    
    memset(result, 0x00, MICEX_ADC_FRAME_SIZE); // limpa o buffer de entrada
    sound_samp_buf_ADC = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA); // alocar memoria     

    s_task_handle = xTaskGetCurrentTaskHandle(); // guardar o enderço da task principal
    cbs.on_conv_done = s_conv_done_cb; // chamada qnd o adc termina um bloco
    cbs.on_pool_ovf = NULL; // overflow

    esp_log_level_set(TAG,ESP_LOG_DEBUG); // ativa as mensagens detalhadas

    XQ=xQueueCreate(1, sizeof(float)*MICEX_SOUND_SAMPLES_BUF_SIZE); // Criar a queue com 1 slot
    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL ); //criar a task

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle); // cria o adc continua
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL)); // ligar o isr do adc.
    ESP_ERROR_CHECK(adc_continuous_start(handle)); // inicia a leitura do adc

    while (1) {        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // interrupt

        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0); // copia dados do DMA para o array result
            if (ret == ESP_OK) {
                parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
                if (parse_ret == ESP_OK) {
                    for (int i = 0; i < num_parsed_samples; i++) {
                        sound_samp_buf_ADC[sb_count] = (float) parsed_data[i].raw_data;                           
                        sb_count+=1;
                        if(sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) { 
                            xQueueSend(XQ,(void *)sound_samp_buf_ADC,0);
                            sb_count = 0;
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}


static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = MICEX_ADC_BUF_SIZE,
        .conv_frame_size = MICEX_ADC_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = MICEX_ADC_SAMPLE_FREQ,
        .conv_mode = MICEX_ADC_CONV_MODE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = MICEX_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = MICEX_ADC_UNIT;
        adc_pattern[i].bit_width = MICEX_ADC_BIT_WIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    
    *out_handle = handle;
}

