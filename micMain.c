/* ********************************************************************************************************************************* 
 * Microphone test - ADC in continuous mode and time-domain BP filtering 
 * Paulo Pedreiras, Pedro Fonseca, Luis Moutinho 2026/Apr.
 * 
 * Tested:
 *  ESP32-C6 DevKitC-1
 * 
 * - Basic use of the ADC to get and process sound samples.
 * - Uses continuous mode ADC operation, to allow higher frequencies
 * - Signal is processed by a Band-Pass filter, in the time-domain, to identify defined frequencies 
 *  
 * Microphone is a MEMS Adafruit Silicon MEMS Microphone Breakout - SPW2430.
 *     Supplied with 3.3-5V, output at DC pin has a 0.7 V and a 100 mVpp "when talking near". 
 *      In my case I had around 1 V. So the attenuation cannot be 0 dB. 
 *      I have used 2.5 dB (vref/0.7), to get 1.3 to 1.5 volts for Vref+ and avoid saturation
 *      Check other mics to see if this is normal.  
 * 
 *  
 * Bibliography: 

 *      https://docs.espressif.com/projects/esp-idf/en/latest/eidf_component_register(SRCS "micMain.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_adc esp_driver_gpio)sp32/api-reference/peripherals/adc/index.html

 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/index.html

 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/adc_continuous.html 
 *      https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-library.html      
 * 
 * Based on the sample code  provided by EspressIF:
 *      https://github.com/espressif/esp-idf/tree/47faecc3/examples/peripherals/adc/continuous_read 
 * 
 * NOTE: must run idf.py add-dependency "espressif/esp-dsp" when creating a new project using dsp functionality
 ***********************************************************************************************************************************/ 

/* ********************************* 
 * Includes
 ***********************************/
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "driver/gpio.h"


#include "esp_heap_caps.h"
#include "stateMachine.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"      // FreeRTOS includes
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h" // For ESP ADC
#include "esp_dsp.h"                // For ESP DSP functions, conv in the case
#include "esp_private/esp_clk.h"    // For ESP clock functions

/* ********************************
 * Global defines 
 **********************************/
#define LED GPIO_NUM_11
#define MICEX_ADC_UNIT                    ADC_UNIT_1

#include "stateMachine.h"           // <-- ADICIONE ESTA LINHA AQUI!

/* ********************************
 * Global defines 
 **********************************/
#define MICEX_ADC_UNIT                     ADC_UNIT_1

#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5            // Use Vref/0.75, 1.3 ... 1.5 V
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH   // 12 bits resolution (maximum)

#define MICEX_ADC_FRAME_SIZE             512                           /* ADC frame size, in bytes */
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE)    /* Internal buffer, should an integer multiple of the frame size to avoid incomplete frames */
                                         /*2048*/
                                         
#define MICEX_ADC_SAMPLE_FREQ            (20 * 1000)                   /* Sample frequency, in Hz. Notice that there are lower and higher bounds*/
                                         /*fs = 20kHz*/

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048 /* IMPORTANT: If FFT is to be used, must be must be a power of two */
                                              /* For time-domain conv. filters there is no such restriction */
                                              
#define MAX_FILT_IR_LEN                 200     /* Maximum IR filter length */

/* Global variable declarations */
static adc_channel_t channel[1] = {ADC_CHANNEL_3};  // Mic on ADC channel 3
static TaskHandle_t s_task_handle;

static const char *TAG = "MIC_EXAMPLE";

/* ADC - Variables to hold data acquisition and parsing */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0}; // Buffer where the results of a continuous read are placed   
__attribute__((aligned(16))) adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES]; // Buffer where frame parsed data is placed 
volatile float volume1;
volatile float volume2;
volatile float volume3;

/* FreeRTOS tasks and IPC */
#define PROCESSOR_TASK_STACK_SIZE       8192            // Accomodate calls to dsp functions, log, user vars, ...
#define PROCESSOR_TASK_PRIORITY	( tskIDLE_PRIORITY + 4 )
QueueHandle_t XQ;    /* Queue handle */

/*filtro FIR criado no Octave, passa banda de 1400 a 3200 Hz*/
__attribute__((aligned(16))) float h1[] = {
    -0.000576084,     -0.000712760,     -0.000705937,     -0.000538763,     -0.000223461,     0.000195278,     0.000642130,     0.001019974,     0.001226349,    0.001176118,     0.000826439,     0.000197975,     -0.000614671,     -0.001449294,     -0.002101975,     -0.002370373,     -0.002106653,     -0.001267704,     0.000051762,     0.001615897,     0.003081157,     0.004064608,     0.004235290,     0.003408359,     0.001617474,     -0.000857356,     -0.003520979,  -0.005754420,     -0.006951633,     -0.006672908,     -0.004777828,     -0.001500937,     0.002556655,     0.006528286,     0.009461220,     0.010538805,  0.009294607,     0.005764297,     0.000530963,     -0.005358371,     -0.010597261,     -0.013906897,     -0.014343174,     -0.011550778,     -0.005898051,   0.001549915,     0.009226375,     0.015392238,     0.018530542,     0.017716395,     0.012872491,     0.004841967,     -0.004750109,     -0.013826489,-0.020311814,     -0.022610836,     -0.019999675,     -0.012834072,     -0.002517354,     0.008775074,     0.018560685,     0.024603555,     0.025431611,0.020696246,     0.011286040,     -0.000839059,     -0.013062566,     -0.022686142,     -0.027535723,     -0.026460635,     -0.019609341,     -0.008415062,  0.004709562,     0.016902525,     0.025488619,     0.028577673,     0.025488619,     0.016902525,     0.004709562,     -0.008415062,     -0.019609341,     -0.026460635,     -0.027535723,     -0.022686142,     -0.013062566,     -0.000839059,     0.011286040,     0.020696246,     0.025431611,     0.024603555,     0.018560685,     0.008775074,     -0.002517354,     -0.012834072,     -0.019999675,     -0.022610836,     -0.020311814,     -0.013826489,     -0.004750109, 0.004841967,     0.012872491,     0.017716395,     0.018530542,     0.015392238,     0.009226375,     0.001549915,     -0.005898051,     -0.011550778,     -0.014343174,     -0.013906897,     -0.010597261,     -0.005358371,     0.000530963,     0.005764297,     0.009294607,     0.010538805,     0.009461220,     0.006528286,     0.002556655,     -0.001500937,     -0.004777828,     -0.006672908,     -0.006951633,     -0.005754420,     -0.003520979,     -0.000857356,     0.001617474,     0.003408359,     0.004235290,     0.004064608,     0.003081157,     0.001615897,     0.000051762,     -0.001267704,     -0.002106653,     -0.002370373,     -0.002101975,     -0.001449294,     -0.000614671,     0.000197975,     0.000826439,     0.001176118,     0.001226349,     0.001019974,     0.000642130,     0.000195278,     -0.000223461,     -0.000538763,     -0.000705937,     -0.000712760,     -0.000576084,
};

__attribute__((aligned(16))) float h2[] = {
    -0.000347895,     -0.000681975,     -0.000692804,     -0.000334786,     0.000256822,     0.000804992,     0.001007296,     0.000688997,     -0.000077326,    -0.000957856,     -0.001483843,     -0.001283267,     -0.000312747,     0.001043621,     0.002098268,     0.002187646,     0.001058830,     -0.000897871,    -0.002739992,     -0.003402691,     -0.002285422,     0.000315415,     0.003208864,     0.004826128,     0.004045273,     0.000902345,     -0.003238436,   -0.006240690,     -0.006275194,     -0.002888750,     0.002544299,     0.007330527,     0.008771802,     0.005655598,     -0.000889258,     -0.007727093,   -0.011197070,     -0.009054113,     -0.001848271,     0.007078020,     0.013117323,     0.012765617,     0.005618794,     -0.005126011,     -0.014071540,   -0.016328591,     -0.010174082,     0.001780888,     0.013657365,     0.019200976,     0.015077321,     0.002832260,     -0.011617954,     -0.020848258,  -0.019753347,     -0.008362076,     0.007910951,     0.020841265,     0.023572473,     0.014263184,     -0.002743364,     -0.018944124,     -0.025953538,  -0.019868793,     -0.003437718,     0.015173610,     0.026466694,     0.024491925,     0.009999757,     -0.009816093,     -0.024915393,     -0.027536867,  -0.016216494,     0.003396508,     0.021380483,     0.028599507,     0.021380483,     0.003396508,     -0.016216494,     -0.027536867,     -0.024915393, -0.009816093,     0.009999757,     0.024491925,     0.026466694,     0.015173610,     -0.003437718,     -0.019868793,     -0.025953538,     -0.018944124, -0.002743364,     0.014263184,     0.023572473,     0.020841265,     0.007910951,     -0.008362076,     -0.019753347,     -0.020848258,     -0.011617954, 0.002832260,     0.015077321,     0.019200976,     0.013657365,     0.001780888,     -0.010174082,     -0.016328591,     -0.014071540,     -0.005126011,0.005618794,     0.012765617,     0.013117323,     0.007078020,     -0.001848271,     -0.009054113,     -0.011197070,     -0.007727093,     -0.000889258,0.005655598,     0.008771802,     0.007330527,     0.002544299,     -0.002888750,     -0.006275194,     -0.006240690,     -0.003238436,     0.000902345,     0.004045273,     0.004826128,     0.003208864,     0.000315415,     -0.002285422,     -0.003402691,     -0.002739992,     -0.000897871,     0.001058830,     0.002187646,     0.002098268,     0.001043621,     -0.000312747,     -0.001283267,     -0.001483843,     -0.000957856,     -0.000077326,     0.000688997,     0.001007296,     0.000804992,     0.000256822,     -0.000334786,     -0.000692804,     -0.000681975,     -0.000347895,
};

__attribute__((aligned(16))) float h3[] = {
    -0.000044176,     -0.000620211,     -0.000678198,     -0.000100608,     0.000658508,     0.000906276,     0.000316904,     -0.000694028,     -0.001222988,     -0.000652715,     0.000684473,     0.001628127,     0.001156879,     -0.000565601,     -0.002094085,     -0.001866961,     0.000257721,     0.002561237,    0.002796572,     0.000323218,     -0.002938458,     -0.003924272,     -0.001251193,     0.003109459,     0.005186247,     0.002574275,     -0.002944760,   -0.006474498,     -0.004299777,     0.002318204,     0.007641673,     0.006382872,     -0.001126073,     -0.008512691,     -0.008720888,     -0.000693727,    0.008902340,     0.011154840,     0.003145180,     -0.008637053,     -0.013478887,     -0.006163045,     0.007578293,     0.015457283,     0.009609231,  -0.005644536,     -0.016847331,     -0.013277919,     0.002828781,     0.017425892,     0.016909585,     0.000791095,     -0.017016328,     -0.020212895,  -0.005051069,     0.015512513,     0.022892323,     0.009709616,     -0.012896708,     -0.024678473,     -0.014465865,     0.009248708,     0.025357580, 0.018985643,     -0.004744689,     -0.024796595,     -0.022932662,     -0.000354626,     0.022960692,     0.026001405,     0.005725491,     -0.019920844, -0.027947914,     -0.011010172,     0.015850306,     0.028614862,     0.015850306,     -0.011010172,     -0.027947914,     -0.019920844,     0.005725491, 0.026001405,     0.022960692,     -0.000354626,     -0.022932662,     -0.024796595,     -0.004744689,     0.018985643,     0.025357580,     0.009248708,-0.014465865,     -0.024678473,     -0.012896708,     0.009709616,     0.022892323,     0.015512513,     -0.005051069,     -0.020212895,     -0.017016328, 0.000791095,     0.016909585,     0.017425892,     0.002828781,     -0.013277919,     -0.016847331,     -0.005644536,     0.009609231,     0.015457283,     0.007578293,     -0.006163045,     -0.013478887,     -0.008637053,     0.003145180,     0.011154840,     0.008902340,     -0.000693727,     -0.008720888,     -0.008512691,     -0.001126073,     0.006382872,     0.007641673,     0.002318204,     -0.004299777,     -0.006474498,     -0.002944760,     0.002574275,     0.005186247,     0.003109459,     -0.001251193,     -0.003924272,     -0.002938458,     0.000323218,     0.002796572,     0.002561237,     0.000257721,     -0.001866961,     -0.002094085,     -0.000565601,     0.001156879,     0.001628127,     0.000684473,     -0.000652715,     -0.001222988,     -0.000694028,     0.000316904,     0.000906276,     0.000658508,     -0.000100608,     -0.000678198,     -0.000620211,     -0.000044176,
};


/* *************************************************************** 
 * Function prototypes 
 *****************************************************************/
/* Inits the ADC for continuous mode (channels, attenuation, frequency, handles, ...)*/
 static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
 /* Callback of ADC driver. Executed whenever a new frame is available */
static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
/* Task called to process one full buffer of data. A queue + blocking read is used for synchronization and data passing */

     void pv_processor_task(void *pvParam);


/******************************************************************* 
 * The main task 
 *******************************************************************/
void app_main(void)
{
    /* Variable declarations */
    esp_err_t ret;          // Generic return code variable
    esp_err_t parse_ret;    // return code of ADC frame parse function 
    uint32_t ret_num = 0;   // Length of bytes return by a read operation
    uint32_t sb_count = 0;   // For counting the number of acquired samples    
    uint32_t num_parsed_samples = 0;    // To count the number of parsed samples
    
    adc_continuous_evt_cbs_t cbs;   // Variable for setting callback type (internal poll full, or frame conversion completed)    
    adc_continuous_handle_t handle = NULL;  //Handle for ADC          

    float * sound_samp_buf_ADC;   // Buffer to hold sound samples. Sound buffers are float because conv() function requires float parameters - avoid conversions 
    
    /* Variable inits */
    memset(result, 0x00, MICEX_ADC_FRAME_SIZE); // Init frame buffer     
    sound_samp_buf_ADC = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);     

    s_task_handle = xTaskGetCurrentTaskHandle();    // Get handle of the current task

    cbs.on_conv_done = s_conv_done_cb;  // Callback called when one conversion frame is done     
    cbs.on_pool_ovf = NULL;          // Don't set callback for internbal buffer overflow         

    ///* Set log level */
    /* Debug allow to see variable values */
    /* Info only shows the decision */
    /* Verbose shows a trace of calls an some additional vars*/
    esp_log_level_set(TAG,ESP_LOG_DEBUG);

    /* Processor task and Queue inits */
    XQ=xQueueCreate(1, sizeof(float)*MICEX_SOUND_SAMPLES_BUF_SIZE); // Create queue to store one full sample period of sound
    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL );

    /* Init ADC */
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle); // Call init function
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));   // Regiter callbacks
    ESP_ERROR_CHECK(adc_continuous_start(handle));                                  // Start the ADC

    /* Infinite loop - wait for data and process it */
    /* Synchronization with ADC is obtained via the ulTaskNotifyTake(pdTRUE, portMAX_DELAY); call */
    /*     that assures that processing does not proceed until a notification that a frame was acquired*/
    while (1) {        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a new frame

        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK) {
               ESP_LOGV(TAG, "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);                
                /* One frame received. Extract samples from frame and put them on sound sample buffer*/
                parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
                if (parse_ret == ESP_OK) {
                    
                    for (int i = 0; i < num_parsed_samples; i++) {
                        sound_samp_buf_ADC[sb_count] = (float) parsed_data[i].raw_data;                           
                        sb_count+=1;
                        if(sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) { // The sound buffer is full. Process it ... */
                            //ESP_LOGD(TAG, "sound buffer acquired. Time to process ...\n");                
                            xQueueSend(XQ,(void *)sound_samp_buf_ADC,0);     // Places the sound buffer in the queue. If the queue is full skip it (ticksTo Wait set to 0)
                                                                        // The consumer/processing task is automatically waked if blocked in the Queue
                            sb_count = 0;
                        }
                    }

                } else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }
                /*                  
                 * To avoid a task watchdog timeout, add a delay here. 
                 */
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}


/* **********************************************************************************************
 * Task activated when there is a full buffer of sound samples data available
 * The task reads a queue in blocking mode. This wait it awakes whenever the ADC processing code
 *      (the app_main taks in the case) delivers a new full buffer of data. 
 * Note that the use of a Queue and two separate buffers (ADC and processing) decouples the 
 *      acquisition from processing. I.e., processing can take as much time as needed without race conditions
 *      or any other sort of interference. The cost is overhead ...
 ************************************************************************************************/
void pv_processor_task(void *pvParam)
{
    /* Local vars, for auxiliary computations */           
    float * sound_samp_buf_proc;       // Buffer to hold sound samples. Buffers are float because conv() function requires float parameters - avoid conversions     
    float * sinal_filtrado1;
    float * sinal_filtrado2;
    float * sinal_filtrado3;





    sound_samp_buf_proc = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);   

    sinal_filtrado1 = heap_caps_malloc(sizeof(float) * (MICEX_SOUND_SAMPLES_BUF_SIZE+150), MALLOC_CAP_DMA); 
    sinal_filtrado2 = heap_caps_malloc(sizeof(float) * (MICEX_SOUND_SAMPLES_BUF_SIZE+150), MALLOC_CAP_DMA); 
    sinal_filtrado3 = heap_caps_malloc(sizeof(float) * (MICEX_SOUND_SAMPLES_BUF_SIZE+150), MALLOC_CAP_DMA); 

    int lenght_FIR  = sizeof(h1)/sizeof(float);

    /* Infinite processing loop */
    for(;;) {
        /* Waits for new data */
        xQueueReceive(XQ,(void *)sound_samp_buf_proc,portMAX_DELAY); // Reads a sound sample. Blocks if queue is empty.
        ESP_LOGV(TAG, "Process Task got a buffer!");
     


           float soma_amostras = 0;
        for(int i = 0; i<MICEX_SOUND_SAMPLES_BUF_SIZE; i++){
            soma_amostras += sound_samp_buf_proc[i];
        }
        float media_dc = soma_amostras/MICEX_SOUND_SAMPLES_BUF_SIZE;
        for(int i = 0; i<MICEX_SOUND_SAMPLES_BUF_SIZE;i++){
            sound_samp_buf_proc[i] = sound_samp_buf_proc[i] - media_dc;
        }
       
    

        dsps_conv_f32(sound_samp_buf_proc,MICEX_SOUND_SAMPLES_BUF_SIZE,h1,lenght_FIR,sinal_filtrado1);
        dsps_conv_f32(sound_samp_buf_proc,MICEX_SOUND_SAMPLES_BUF_SIZE,h2,lenght_FIR,sinal_filtrado2);
        dsps_conv_f32(sound_samp_buf_proc,MICEX_SOUND_SAMPLES_BUF_SIZE,h3,lenght_FIR,sinal_filtrado3);

            volume1 = 0;
            volume2 = 0; 
            volume3 = 0;


        for (int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
            volume1 += sinal_filtrado1[i] * sinal_filtrado1[i];
            volume2 += sinal_filtrado2[i] * sinal_filtrado2[i];
            volume3 += sinal_filtrado3[i] * sinal_filtrado3[i];
        }

        main_estados();

    }
}

/* ADC Callback - called when one frame was acquired */
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

/* ADC init function */
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

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}