#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"
#include "stdio.h"

#include "cpx.h"


#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 324
#define CAM_HEIGHT 244

#define REG_ADDR 0x3010


static pi_task_t task;
static unsigned char *imgBuff;
static struct pi_device camera;
static pi_buffer_t buffer;

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)


#define LED_PIN 2
static pi_device_t led_gpio_dev;

static int open_pi_camera_himax(struct pi_device *device)
{
    struct pi_himax_conf cam_conf;
    // cam_conf.skip_pads_config = 0;

    pi_himax_conf_init(&cam_conf);

    cam_conf.format = PI_CAMERA_QVGA;

    pi_open_from_conf(device, &cam_conf);
    if (pi_camera_open(device))
        return -1;

    // rotate image
    pi_camera_control(device, PI_CAMERA_CMD_START, 0);
    uint8_t set_value = 3;
    uint8_t reg_value;
    pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
    pi_time_wait_us(1000000);
    pi_camera_reg_get(device, IMG_ORIENTATION, &reg_value);

    if (set_value != reg_value)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to rotate camera image\n");
        return -1;
    }

    uint8_t result = 5;
    pi_camera_reg_get(device, REG_ADDR,&result);
    // cpxPrintToConsole(LOG_TO_CRTP, "Register Value: %d\n",result);



    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

    return 0;
}


static void capture_done_cb(void *arg)
{
    // pi_perf_stop();
}



void tx_task(void *parameters)
{
    // INITIALIZE WIFI CONNECTION AND ACCESS POINT
    vTaskDelay(2000);

    // SET UP IMAGE BUFFER AND CAMERA CAPTURE SETTINGS
    // cpxPrintToConsole(LOG_TO_CRTP, "Starting camera task...\n");
    uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);
    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for Image \n");
        return;
    }

    // TURN ON CAMERA
    if (open_pi_camera_himax(&camera))
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
        return;
    }


    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);


    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
    pi_perf_conf(1 << PI_PERF_CYCLES);

    // Initialize the LED pin
    pi_gpio_pin_configure(&led_gpio_dev, LED_PIN, PI_GPIO_OUTPUT);

    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

    uint32_t clock_cycles = 0;
    int32_t cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);


    while (1)
    {
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 1);
        vTaskDelay(xDelay);
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 0);
        vTaskDelay(xDelay);

        pi_perf_reset();
        pi_perf_start();
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        pi_task_callback(&task, capture_done_cb, NULL);
        pi_camera_capture_async(&camera, imgBuff, CAM_WIDTH * CAM_HEIGHT, &task);
        pi_task_wait_on(&task);
        // vTaskDelay(100);
        pi_perf_stop();

        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
       



        
        // // CAPTURE CAMERA IMAGE
        // pi_perf_reset();
        // pi_perf_start();
        // pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        // pi_camera_capture(&camera, imgBuff, resolution);
        // pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
        // pi_perf_stop();
        // vTaskDelay(100);

        clock_cycles = pi_perf_read(PI_PERF_CYCLES);
        cpxPrintToConsole(LOG_TO_CRTP,"Frame Rate: %.5f\n",(double)cur_fc_freq/clock_cycles);

        vTaskDelay(100);
    }

    vTaskDelay(100);
}
/usr/bin/openocd
sudo cp /usr/share/gap8-openocd/openocd/contrib/60-openocd.rules /etc/udev/rules.d

void start_example(void)
{
    // INITIALIZE UART CONNECTION FOR CPX
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
    // cpxPrintToConsole(LOG_TO_CRTP, "\n\n-- WiFi image streamer example --\n\n");


    // CREATE EVENT GROUP
    evGroup = xEventGroupCreate();
    BaseType_t xTask;

    // CAMERA CAPTURE AND DATA TRANSFER TASK
    xTask = xTaskCreate(tx_task, "tx_task", configMINIMAL_STACK_SIZE * 4,
                        NULL, tskIDLE_PRIORITY + 3, NULL);

    if (xTask != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Camera task did not start !\n");
        pmsis_exit(-1);
    }


    while (1)
    {
        pi_yield();
    }
}

int main(void)
{
    pi_bsp_init();

    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

    return pmsis_kickoff((void *)start_example);
}
