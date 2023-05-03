
#include "wifi.h"
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"


static pi_task_t task1;
static unsigned char *imgBuff;
static struct pi_device camera;
static pi_buffer_t buffer;

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

// Performance menasuring variables
static uint32_t start = 0;
static uint32_t captureTime = 0;
static uint32_t transferTime = 0;
static uint32_t encodingTime = 0;

uint32_t capture_arr[5] = {0};
uint32_t image = 0;


static int open_pi_camera_himax(struct pi_device *device)
{
    struct pi_himax_conf cam_conf;

    pi_himax_conf_init(&cam_conf);

    cam_conf.format = PI_CAMERA_QQVGA;
    cam_conf.skip_pads_config = 0;


    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
        return -1;

    // rotate image
    pi_camera_control(device, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    vTaskDelay(500);
    // // pi_camera_reg_get(device, IMG_ORIENTATION, &reg_value);
    // // if (set_value != reg_value)
    // // {
    // //     printf("Failed to rotate camera image\n");
    // //     return -1;
    // // }
    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    // pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

    return 0;
}



static void capture_done_cb(void *arg)
{
  xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}


void camera_task(void *parameters)
{

    printf("Starting camera task...\n");
    uint32_t imgSize = 0;
    uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);

    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff == NULL)
    {
        printf("Failed to allocate Memory for Image \n");
        return;
    }

    if (open_pi_camera_himax(&camera))
    {
        printf("Failed to open camera\n");
        return;
    }


    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);


    printf("Image: \n");

    pi_perf_conf(1 << PI_PERF_CYCLES);

    pi_perf_start();
    start = pi_perf_read(PI_PERF_CYCLES);
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    pi_camera_capture(&camera,imgBuff,resolution);
    captureTime = pi_perf_read(PI_PERF_CYCLES) - start;
    capture_arr[image] = captureTime;
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    pi_perf_stop();
    pi_perf_reset();

    printf("dsfsdf\n\n");


    image = 0;

    while (image < 5)
    {
        pi_perf_start();
        start = pi_perf_read(PI_PERF_CYCLES);
        // pi_camera_capture_async(&camera, imgBuff, resolution, pi_task_callback(&task1, capture_done_cb, NULL));
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        pi_camera_capture(&camera,imgBuff,resolution);
        captureTime = pi_perf_read(PI_PERF_CYCLES) - start;
        capture_arr[image] = captureTime;
        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
        // xEventGroupWaitBits(evGroup, CAPTURE_DONE_BIT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);

        vTaskDelay(5);
        pi_perf_stop();
        pi_perf_reset();
        image++;

    }

    for (uint8_t i = 0; i < 5; i++)
    {
        printf("Image %d = %d cycles\n",i,capture_arr[i]);

    }
   
    pi_perf_start();
    vTaskDelay(5);
    pi_perf_stop();
    uint32_t val = pi_perf_read(PI_PERF_CYCLES);
    pi_perf_reset();

    printf("Tick delay %d\n",val);
    printf("Tick delay %u\n",portTICK_PERIOD_MS);
    

   
    

    while (1)
    {
        pi_yield();
    }
    
    


    // while (1)
    // {

    //     if (wifiClientConnected == 1)
    //     {
    //         pi_perf_start();
    //         start = pi_perf_read(PI_PERF_CYCLES);
    //         pi_camera_capture_async(&camera, imgBuff, resolution, pi_task_callback(&task1, capture_done_cb, NULL));
    //         pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    //         xEventGroupWaitBits(evGroup, CAPTURE_DONE_BIT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);

    //         pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    //         captureTime = pi_perf_read(PI_PERF_CYCLES) - start;


    //         imgSize = captureSize;
    //         start = pi_perf_read(PI_PERF_CYCLES);

    //         // First send information about the image
    //         createImageHeaderPacket(&txp, imgSize);
    //         cpxSendPacketBlocking(&txp);

    //         start = pi_perf_read(PI_PERF_CYCLES);
    //         // Send image
    //         sendBufferViaCPX(&txp, imgBuff, imgSize);

    //         transferTime = pi_perf_read(PI_PERF_CYCLES) - start;
    //         pi_perf_stop();
    //         pi_perf_reset();


    //         printf("capture=%d cycles, encoding=%d ms (%d bytes), transfer=%d ms\n",
    //                             captureTime, encodingTime, imgSize, transferTime);

    //     }
    //     else
    //     {
    //         vTaskDelay(10);
    //     }
    // }
}

#define LED_PIN 2
static pi_device_t led_gpio_dev;
void hb_task(void *parameters)
{
    (void)parameters;
    char *taskname = pcTaskGetName(NULL);

    // Initialize the LED pin
    pi_gpio_pin_configure(&led_gpio_dev, LED_PIN, PI_GPIO_OUTPUT);

    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

    while (1)
    {
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 1);
        vTaskDelay(xDelay);
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 0);
        vTaskDelay(xDelay);
    }
}

void start_example(void)
{
    
    printf("-- WiFi image streamer example --\n");

    struct pi_uart_conf conf;
    struct pi_device device;
    pi_uart_conf_init(&conf);
    conf.baudrate_bps = 115200;


    pi_open_from_conf(&device, &conf);
    if (pi_uart_open(&device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    cpxInit();
    cpxEnableFunction(CPX_F_WIFI_CTRL);

    vTaskDelay(2000);

    setupWiFi();

    header.size = 1024;
    header.data = pmsis_l2_malloc(1024);

    footer.size = 10;
    footer.data = pmsis_l2_malloc(10);

    // We're reusing the same packet, so initialize the route once
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);

    

    evGroup = xEventGroupCreate();

    BaseType_t xTask;

    xTask = xTaskCreate(hb_task, "hb_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS)
    {
        printf("HB task did not start !\n");
        pmsis_exit(-1);
    }

    xTask = xTaskCreate(camera_task, "camera_task", configMINIMAL_STACK_SIZE * 4,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    if (xTask != pdPASS)
    {
            printf("Camera task did not start !\n");
            pmsis_exit(-1);
    }

    xTask = xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    if (xTask != pdPASS)
    {
        printf("RX task did not start !\n");
        pmsis_exit(-1);
    }

    while (1)
    {
        pi_yield();
    }
    pmsis_exit(0);

}

int main(void)
{
  pi_bsp_init();

  // Increase the FC freq to 250 MHz
  pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
  pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

  return pmsis_kickoff((void *)start_example);
}
