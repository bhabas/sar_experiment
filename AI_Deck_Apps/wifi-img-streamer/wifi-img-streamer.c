#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"
#include "stdio.h"

#include "cpx.h"
#include "wifi.h"

#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 324
#define CAM_HEIGHT 244

#define REG_ADDR 0x3010


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
        printf( "Failed to rotate camera image\n");
        return -1;
    }

    uint8_t result = 5;
    pi_camera_reg_get(device, REG_ADDR,&result);
    printf( "Register Value: %d\n",result);



    pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
    pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

    return 0;
}

static int wifiConnected = 0;
static int wifiClientConnected = 0;

static CPXPacket_t CPX_RX_Packet;
void rx_task(void *parameters)
{
    while (1)
    {
        cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &CPX_RX_Packet);

        WiFiCTRLPacket_t * wifiCtrl = (WiFiCTRLPacket_t*) CPX_RX_Packet.data;

        switch (wifiCtrl->cmd)
        {
        case WIFI_CTRL_STATUS_WIFI_CONNECTED:
            printf( "Wifi connected (%u.%u.%u.%u)\n",
                            wifiCtrl->data[0], wifiCtrl->data[1],
                            wifiCtrl->data[2], wifiCtrl->data[3]);
            wifiConnected = 1;
            break;
        case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
            printf( "Wifi client connection status: %u\n", wifiCtrl->data[0]);
            wifiClientConnected = wifiCtrl->data[0];
            break;
        default:
            break;
        }
    }
}

static void capture_done_cb(void *arg)
{
    xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}

typedef struct
{
    uint8_t magic;
    uint16_t width;
    uint16_t height;
    uint8_t depth;
    uint8_t type;
    uint32_t size;
} __attribute__((packed)) img_header_t;


typedef enum
{
    RAW_ENCODING = 0,
    JPEG_ENCODING = 1
} __attribute__((packed)) StreamerMode_t;

pi_buffer_t header;
uint32_t headerSize;
pi_buffer_t footer;
uint32_t footerSize;


static StreamerMode_t streamerMode = RAW_ENCODING;

static CPXPacket_t CPX_TX_Packet;

void createImageHeaderPacket(CPXPacket_t * packet, uint32_t imgSize, StreamerMode_t imgType) {
    img_header_t *imgHeader = (img_header_t *) packet->data;
    imgHeader->magic = 0xBC;
    imgHeader->width = CAM_WIDTH;
    imgHeader->height = CAM_HEIGHT;
    imgHeader->depth = 1;
    imgHeader->type = imgType;
    imgHeader->size = imgSize;
    packet->dataLength = sizeof(img_header_t);
}

void sendBufferViaCPX(CPXPacket_t * packet, uint8_t * buffer, uint32_t bufferSize) {
    uint32_t offset = 0;
    uint32_t size = 0;
    do {
        size = sizeof(packet->data);
        if (offset + size > bufferSize)
        {
        size = bufferSize - offset;
        }
        memcpy(packet->data, &buffer[offset], sizeof(packet->data));
        packet->dataLength = size;
        cpxSendPacketBlocking(packet);
        offset += size;
    } while (size == sizeof(packet->data));
}


void setupWiFi(void) {
    static char ssid[] = "WiFi streaming example";
    printf( "Setting up WiFi AP\n");
    // Set up the routing for the WiFi CTRL packets
    CPX_TX_Packet.route.destination = CPX_T_ESP32;
    CPX_RX_Packet.route.source = CPX_T_GAP8;
    CPX_TX_Packet.route.function = CPX_F_WIFI_CTRL;
    WiFiCTRLPacket_t * wifiCtrl = (WiFiCTRLPacket_t*) CPX_TX_Packet.data;
    
    wifiCtrl->cmd = WIFI_CTRL_SET_SSID;
    memcpy(wifiCtrl->data, ssid, sizeof(ssid));
    CPX_TX_Packet.dataLength = sizeof(ssid);
    cpxSendPacketBlocking(&CPX_TX_Packet);
    
    wifiCtrl->cmd = WIFI_CTRL_WIFI_CONNECT;
    wifiCtrl->data[0] = 0x01;
    CPX_TX_Packet.dataLength = 2;
    cpxSendPacketBlocking(&CPX_TX_Packet);
}


void tx_task(void *parameters)
{
    // INITIALIZE WIFI CONNECTION AND ACCESS POINT
    vTaskDelay(2000);
    setupWiFi();

    // SET UP IMAGE BUFFER AND CAMERA CAPTURE SETTINGS
    printf( "Starting camera task...\n");
    uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;
    uint32_t captureSize = resolution * sizeof(unsigned char);
    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);
    if (imgBuff == NULL)
    {
        printf( "Failed to allocate Memory for Image \n");
        return;
    }

    // TURN ON CAMERA
    if (open_pi_camera_himax(&camera))
    {
        printf( "Failed to open camera\n");
        return;
    }

    


    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    header.size = 1024;
    header.data = pmsis_l2_malloc(1024);

    footer.size = 10;
    footer.data = pmsis_l2_malloc(10);

    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    // INITIALIZE CPX ROUTE
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &CPX_TX_Packet.route);

    while (1)
    {

        // CAPTURE CAMERA IMAGE
        start = xTaskGetTickCount();
        // pi_camera_capture_async(&camera, imgBuff, resolution, pi_task_callback(&task1, capture_done_cb, NULL));
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        pi_camera_capture(&camera, imgBuff, resolution);
        // xEventGroupWaitBits(evGroup, CAPTURE_DONE_BIT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);
        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
        captureTime = xTaskGetTickCount() - start;

        printf( "Camera capture\n");

        // for (size_t i = 0; i < 50; i++)
        // {
        //     printf("%u ",imgBuff[i]);
        // }
        // printf("Value: %u\n",(unsigned int)imgBuff[0]);

        
        printf( "\ncapture=%d ms, encoding=%d ms (%d bytes), transfer=%d ms\n",
                            captureTime, encodingTime, captureSize, transferTime);

        vTaskDelay(100);

    }
}

// ---- LED TASK ---- //
void LED_task(void *parameters)
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
    cpxEnableFunction(CPX_F_WIFI_CTRL);
    printf( "\n\n-- WiFi image streamer example --\n\n");


    // CREATE EVENT GROUP
    evGroup = xEventGroupCreate();
    BaseType_t xTask;

    // INIT LED FLASHING TASK
    xTask = xTaskCreate(LED_task, "LED_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS)
    {
        printf( "LED task did not start !\n");
        pmsis_exit(-1);
    }

    // CAMERA CAPTURE AND DATA TRANSFER TASK
    xTask = xTaskCreate(tx_task, "tx_task", configMINIMAL_STACK_SIZE * 4,
                        NULL, tskIDLE_PRIORITY + 3, NULL);

    if (xTask != pdPASS)
    {
        printf( "Camera task did not start !\n");
        pmsis_exit(-1);
    }

    // DATA RECEPTION TASK
    xTask = xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    if (xTask != pdPASS)
    {
        printf( "RX task did not start !\n");
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
