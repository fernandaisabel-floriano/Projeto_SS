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

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"      // FreeRTOS includes
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h" // For ESP ADC
#include "esp_dsp.h"                // For ESP DSP functions, conv in the case
#include "esp_private/esp_clk.h"    // For ESP clock functions
#include "esp_private/esp_clk.h"    // For ESP clock functions
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

/* Impulse reponse filter and related variables */
__attribute__((aligned(16))) float hbpf2k[]={0.000000139618742, 0.000000255721385, 0.000000140050607, -0.000000328918009, -0.000001079871671, -0.000001790994587, -0.000001943975411, -0.000001031882007, 0.000001111877099, 0.000004001347557, 0.000006413035946, 0.000006707873993, 0.000003574260112, -0.000003048702149, -0.000011330185703, -0.000017688431396, -0.000017969871244, -0.000009435702605, 0.000007215862062, 0.000026871405806, 0.000040972415077, 0.000040614490911, 0.000020897907315, -0.000015285006327, -0.000056126887733, -0.000083842525959, -0.000081437626075, -0.000041101266796, 0.000029421836936, 0.000106148839227, 0.000155757573851, 0.000148773462501, 0.000074351529935, -0.000050052623102, -0.000180121406766, -0.000259437345082, -0.000243952642519, -0.000126937980784, 0.000047492844400, 0.000191919289194, 0.000580032052973, 0.000667352843244, 0.000306111499321, -0.000446298721323, -0.001342483904490, -0.001978250973038, -0.001912439692934,-0.000868574004532, 0.001047898364964, 0.003236203885917, 0.004708723047573, 0.004458080398213, 0.001979374693417, -0.002286980559547, -0.006863696045905, -0.009659616406773, -0.008836721670391, -0.003790852332498, 0.004230959606325, 0.012280364027838, 0.016725773018749, 0.014820278024314, 0.006161630397857, -0.006683261470121, -0.018835296877081, -0.024940029322091, -0.021501888240714, -0.008700432996997, 0.009216737308611, 0.025316468417048, 0.032708573103906, 0.027529205755171, 0.010869128872094, -0.011297781514959, -0.030304625508737, -0.038281206662112, -0.031498011959705, -0.012107397246638, 0.012536609979868, 0.032771101414432, 0.040552804004055, 0.032771101414432, 0.012536609979868, -0.012107397246638, -0.031498011959704, -0.038281206662113, -0.030304625508737, -0.011297781514959, 0.010869128872094, 0.027529205755171, 0.032708573103906, 0.025316468417048, 0.009216737308611, -0.008700432996997, -0.021501888240714, -0.024940029322091, -0.018835296877081, -0.006683261470121, 0.006161630397857, 0.014820278024314, 0.016725773018749, 0.012280364027838,0.004230959606325, -0.003790852332499, -0.008836721670391, -0.009659616406773, -0.006863696045905, -0.002286980559547, 0.001979374693417, 0.004458080398213, 0.004708723047573, 0.003236203885917, 0.001047898364964, -0.000868574004532, -0.001912439692934, -0.001978250973038, -0.001342483904490, -0.000446298721323, 0.000306111499321, 0.000667352843244, 0.000580032052973, 0.000191919289194, 0.000047492844400, -0.000126937980784, -0.000243952642519, -0.000259437345082, -0.000180121406766, -0.000050052623102, 0.000074351529935, 0.000148773462501, 0.000155757573851, 0.000106148839227, 0.000029421836936, -0.000041101266796, -0.000081437626075, -0.000083842525959, -0.000056126887733, -0.000015285006327, 0.000020897907315, 0.000040614490911, 0.000040972415077, 0.000026871405806, 0.000007215862062, -0.000009435702605, -0.000017969871244, -0.000017688431396, -0.000011330185703, -0.000003048702149, 0.000003574260112,0.000006707873993, 0.000006413035946, 0.000004001347557, 0.000001111877099, -0.000001031882007, -0.000001943975411, -0.000001790994587, -0.000001079871671, -0.000000328918009, 0.000000140050607, 0.000000255721385, 0.000000139618742};
/*filtro FIR criado no Octave, passa banda de 1400 a 3200 Hz*/
__attribute__((aligned(16))) float h1[] = {0.000512348, 0.000633709, 0.000625060, 0.000472754, 0.000191816,
    -0.000172505, -0.000549128, -0.000852380, -0.000999455, -0.000931285,
    -0.000631960, -0.000140722, 0.000448754, 0.001004418, 0.001385555,
    0.001479795, 0.001237477, 0.000692498, -0.000038702, -0.000782648,
    -0.001353979, -0.001608379, -0.001487357, -0.001039696, -0.000410217,
    0.000203663, 0.000616229, 0.000718855, 0.000523905, 0.000169173,
    -0.000123770, -0.000128752, 0.000291269, 0.001107506, 0.002096302,
    0.002876921, 0.003015512, 0.002171929, 0.000247518, -0.002515740,
    -0.005527161, -0.007950796, -0.008908869, -0.007740337, -0.004245241,
    0.001161463, 0.007447720, 0.013139386, 0.016649900, 0.016694154,
    0.012682414, 0.004982010, -0.005042623, -0.015250457, -0.023176799,
    -0.026624223, -0.024249963, -0.015995086, -0.003228181, 0.011459779,
    0.024786651, 0.033526991, 0.035302896, 0.029220993, 0.016185334,
    -0.001211703, -0.019216915, -0.033752725, -0.041370446, -0.040090805,
    -0.029921661, -0.012916006, 0.007256802, 0.026134125, 0.039486035,
    0.044299861, 0.039486035, 0.026134125, 0.007256802, -0.012916006,
    -0.029921661, -0.040090805, -0.041370446, -0.033752725, -0.019216915,
    -0.001211703, 0.016185334, 0.029220993, 0.035302896, 0.033526991,
    0.024786651, 0.011459779, -0.003228181, -0.015995086, -0.024249963,
    -0.026624223, -0.023176799, -0.015250457, -0.005042623, 0.004982010,
    0.012682414, 0.016694154, 0.016649900, 0.013139386, 0.007447720,
    0.001161463, -0.004245241, -0.007740337, -0.008908869, -0.007950796,
    -0.005527161, -0.002515740, 0.000247518, 0.002171929, 0.003015512,
    0.002876921, 0.002096302, 0.001107506, 0.000291269, -0.000128752,
    -0.000123770, 0.000169173, 0.000523905, 0.000718855, 0.000616229,
    0.000203663, -0.000410217, -0.001039696, -0.001487357, -0.001608379,
    -0.001353979, -0.000782648, -0.000038702, 0.000692498, 0.001237477,
    0.001479795, 0.001385555, 0.001004418, 0.000448754, -0.000140722,
    -0.000631960, -0.000931285, -0.000999455, -0.000852380, -0.000549128,
    -0.000172505, 0.000191816, 0.000472754, 0.000625060, 0.000633709,
    0.000512348};

__attribute__((aligned(16))) float h2[] = {0.000310403, 0.000606989, 0.000613639, 0.000292697, -0.000227715,
    -0.000697031, -0.000855392, -0.000570828, 0.000068210, 0.000764857,
    0.001144527, 0.000952636, 0.000217065, -0.000726304, -0.001383024,
    -0.001362850, -0.000615026, 0.000508638, 0.001421142, 0.001621143,
    0.000986654, -0.000139846, -0.001141262, -0.001488190, -0.001053727,
    -0.000177748, 0.000569465, 0.000775454, 0.000463538, 0.000057231,
    0.000036209, 0.000516416, 0.001073317, 0.000991005, -0.000173995,
    -0.002089016, -0.003613472, -0.003392649, -0.000805670, 0.003327056,
    0.006845872, 0.007305441, 0.003505462, -0.003403452, -0.010058299,
    -0.012451888, -0.008240308, 0.001499366, 0.012244936, 0.018063642,
    0.014832343, 0.002917693, -0.012346735, -0.022983097, -0.022525009,
    -0.009847026, 0.009564655, 0.025916557, 0.030068978, 0.018631056,
    -0.003650048, -0.025777528, -0.035978300, -0.028015420, -0.004928087,
    0.022019253, 0.038895614, 0.036396781, 0.015007992, -0.014848571,
    -0.037961978, -0.042199298, -0.024962964, 0.005243558, 0.033076916,
    0.044273660, 0.033076916, 0.005243558, -0.024962964, -0.042199298,
    -0.037961978, -0.014848571, 0.015007992, 0.036396781, 0.038895614,
    0.022019253, -0.004928087, -0.028015420, -0.035978300, -0.025777528,
    -0.003650048, 0.018631056, 0.030068978, 0.025916557, 0.009564655,
    -0.009847026, -0.022525009, -0.022983097, -0.012346735, 0.002917693,
    0.014832343, 0.018063642, 0.012244936, 0.001499366, -0.008240308,
    -0.012451888, -0.010058299, -0.003403452, 0.003505462, 0.007305441,
    0.006845872, 0.003327056, -0.000805670, -0.003392649, -0.003613472,
    -0.002089016, -0.000173995, 0.000991005, 0.001073317, 0.000516416,
    0.000036209, 0.000057231, 0.000463538, 0.000775454, 0.000569465,
    -0.000177748, -0.001053727, -0.001488190, -0.001141262, -0.000139846,
    0.000986654, 0.001621143, 0.001421142, 0.000508638, -0.000615026,
    -0.001362850, -0.001383024, -0.000726304, 0.000217065, 0.000952636,
    0.001144527, 0.000764857, 0.000068210, -0.000570828, -0.000855392,
    -0.000697031, -0.000227715, 0.000292697, 0.000613639, 0.000606989,
    0.000310403};

__attribute__((aligned(16))) float h3[] = {0.000040659, 0.000552708, 0.000601192, 0.000086274, -0.000578058,
    -0.000782097, -0.000265768, 0.000582486, 0.000997076, 0.000513720,
    -0.000533616, -0.001215387, -0.000825341, 0.000398115, 0.001380757,
    0.001160361, -0.000163421, -0.001425068, -0.001437872, -0.000140071,
    0.001293010, 0.001545412, 0.000424096, -0.000972285, -0.001363249,
    -0.000541917, 0.000521089, 0.000800425, 0.000306202, -0.000083811,
    0.000164967, 0.000474916, -0.000112652, -0.001451479, -0.001948919,
    -0.000214272, 0.002856512, 0.004166846, 0.001354373, -0.004063221,
    -0.007040317, -0.003541803, 0.004675622, 0.010319696, 0.006889716,
    -0.004279170, -0.013602315, -0.011336642, 0.002517842, 0.016374078,
    0.016618234, 0.000826248, -0.018080754, -0.022274108, -0.005766913,
    0.018218384, 0.027693105, 0.012076780, -0.016427018, -0.032192541,
    -0.019289612, 0.012569783, 0.035119609, 0.026745589, -0.006780736,
    -0.035957634, -0.033674638, -0.000530021, 0.034417714, 0.039305100,
    0.008717773, -0.030498014, -0.042979364, -0.016986149, 0.024498394,
    0.044255935, 0.024498394, -0.016986149, -0.042979364, -0.030498014,
    0.008717773, 0.039305100, 0.034417714, -0.000530021, -0.033674638,
    -0.035957634, -0.006780736, 0.026745589, 0.035119609, 0.012569783,
    -0.019289612, -0.032192541, -0.016427018, 0.012076780, 0.027693105,
    0.018218384, -0.005766913, -0.022274108, -0.018080754, 0.000826248,
    0.016618234, 0.016374078, 0.002517842, -0.011336642, -0.013602315,
    -0.004279170, 0.006889716, 0.010319696, 0.004675622, -0.003541803,
    -0.007040317, -0.004063221, 0.001354373, 0.004166846, 0.002856512,
    -0.000214272, -0.001948919, -0.001451479, -0.000112652, 0.000474916,
    0.000164967, -0.000083811, 0.000306202, 0.000800425, 0.000521089,
    -0.000541917, -0.001363249, -0.000972285, 0.000424096, 0.001545412,
    0.001293010, -0.000140071, -0.001437872, -0.001425068, -0.000163421,
    0.001160361, 0.001380757, 0.000398115, -0.000825341, -0.001215387,
    -0.000533616, 0.000513720, 0.000997076, 0.000582486, -0.000265768,
    -0.000782097, -0.000578058, 0.000086274, 0.000601192, 0.000552708,
    0.000040659};

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
                            ESP_LOGD(TAG, "sound buffer acquired. Time to process ...\n");                
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

    sinal_filtrado1 = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA); 
    sinal_filtrado2 = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA); 
    sinal_filtrado3 = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA); 

    int lenght_FIR  = sizeof(h1)/sizeof(float);

    /* Infinite processing loop */
    for(;;) {
        /* Waits for new data */
        xQueueReceive(XQ,(void *)sound_samp_buf_proc,portMAX_DELAY); // Reads a sound sample. Blocks if queue is empty.
        ESP_LOGV(TAG, "Process Task got a buffer!");
     


            float media = 0;
        for (int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
            media += sound_samp_buf_proc[i];
        }
        media = media / MICEX_SOUND_SAMPLES_BUF_SIZE; // Calcula a média (o valor do muro)

        for (int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
            sound_samp_buf_proc[i] -= media; // Remove o muro de todos os pontos!

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
// teste
