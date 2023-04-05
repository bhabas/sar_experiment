
// STANDARD INCLUDES
#include "stdio.h"

// GAP8 INCLUDES
#include "pmsis.h"
#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"

// CRAZYFLIE INCLUDES
#include "cpx.h"
#include "wifi.h"


#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 50
#define CAM_HEIGHT 50


static unsigned char *imgBuff;
static pi_buffer_t buffer;

// Performance menasuring variables
static uint32_t start = 0;
static uint32_t captureTime = 0;
static uint32_t transferTime = 0;
static uint32_t encodingTime = 0;

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
                cpxPrintToConsole(LOG_TO_CRTP, "Wifi connected (%u.%u.%u.%u)\n",
                                    wifiCtrl->data[0], wifiCtrl->data[1],
                                    wifiCtrl->data[2], wifiCtrl->data[3]);
                wifiConnected = 1;
                break;

            case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
                cpxPrintToConsole(LOG_TO_CRTP, "Wifi client connection status: %u\n", wifiCtrl->data[0]);
                wifiClientConnected = wifiCtrl->data[0];
                break;

            default:
                break;
        }
    }
}




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

typedef struct
{
    uint8_t magic;
    uint16_t width;
    uint16_t height;
    uint8_t depth;
    uint8_t type;
    uint32_t size;
} __attribute__((packed)) img_header_t;

void createImageHeaderPacket(CPXPacket_t* packet, uint32_t imgSize, StreamerMode_t imgType) {
    img_header_t* imgHeader = (img_header_t*) packet->data;
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

// SETUP WIFI SOURCE
void setupWiFi(void) {
    static char ssid[] = "WiFi streaming example";
    cpxPrintToConsole(LOG_TO_CRTP, "Setting up WiFi AP\n");

    // Set up the routing for the WiFi CTRL packets
    CPX_TX_Packet.route.destination = CPX_T_ESP32;
    CPX_TX_Packet.route.function = CPX_F_WIFI_CTRL;

    CPX_RX_Packet.route.source = CPX_T_GAP8;
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
    // TURN ON DEFAULT WIFI NETWORK
    vTaskDelay(2000);   // Wait for 2000 ticks
    setupWiFi();        // Initialize and connect to the WiFi network

    // ALLOCATE MEMORY FOR IMAGE IN BUFFER
    cpxPrintToConsole(LOG_TO_CRTP, "Starting camera task...\n");    // Log camera task start
    uint32_t resolution = CAM_WIDTH * CAM_HEIGHT;                   // Calculate image resolution
    uint32_t captureSize = resolution * sizeof(unsigned char);      // Calculate size of captured image in bytes
    imgBuff = (unsigned char *)pmsis_l2_malloc(captureSize);        // Allocate memory for the image buffer

    // CHECK IF MEMORY ALLOCATION WAS SUCCESSFUL
    if (imgBuff == NULL)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for Image \n");
        return;
    }

    // INITIALIZE AND SET THE FORMAT OF THE IMAGE BUFFER
    pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff);
    pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

    for (int i = 0; i < resolution; i++) {
        imgBuff[i] = i % 256; // set pixel value to current index value (mod 256 to keep it in uint8_t range)
    }

    // ALLOCATE MEMORY FOR HEADER AND FOOTER
    header.size = 1024;
    header.data = pmsis_l2_malloc(1024);

    footer.size = 10;
    footer.data = pmsis_l2_malloc(10);

    // INITIALIZE THE PACKET ROUTE
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &CPX_TX_Packet.route);

    // MAIN LOOP
    while (1)
    {
        // CHECK IF WIFI CLIENT IS CONNECTED
        if (wifiClientConnected == 1)
        {

            start = xTaskGetTickCount(); // Get the current tick count
            captureTime = xTaskGetTickCount() - start; // Calculate the capture time


            // FILL TX PACKET WITH IMAGE SETTING DATA AND SEND PACKET
            createImageHeaderPacket(&CPX_TX_Packet, captureSize, RAW_ENCODING);
            cpxSendPacketBlocking(&CPX_TX_Packet); // Send the packet

            // SEND IMAGE BUFFER
            start = xTaskGetTickCount(); // Get the current tick count
            sendBufferViaCPX(&CPX_TX_Packet, imgBuff, captureSize);

            transferTime = xTaskGetTickCount() - start; // Calculate the transfer time

            // Log capture, encoding, and transfer times
            cpxPrintToConsole(LOG_TO_CRTP, "capture=%dms, encoding=%d ms (%d bytes), transfer=%d ms\n",
                            captureTime, encodingTime, captureSize, transferTime);

        }
        else
        {
            vTaskDelay(10); // Wait for 10ms before checking the WiFi client connection again
        }
    }

}

// CREATE LED FLASHING TASK
#define LED_PIN 2
static pi_device_t led_gpio_dev;
void LED_task(void *parameters)
{
    (void)parameters; // Indicate that parameters isn't used and avoid compiling issue
    char *taskname = pcTaskGetName(NULL);

    // CONFIGURE LED PIN
    pi_gpio_pin_configure(&led_gpio_dev, LED_PIN, PI_GPIO_OUTPUT);

    // CONVERT MS TO SYSTEM TICKS
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

    // FLASH LED
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

    // INIT UART CONNECTION
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
    cpxPrintToConsole(LOG_TO_CRTP, "\n\n-- WiFi data streamer example --\n\n");


    BaseType_t xTask;
    // LED TASK
    xTask = xTaskCreate(LED_task, "LED_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "LED task did not start !\n");
        pmsis_exit(-1);
    }

    // DATA RECEPTION TASK
    xTask = xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    if (xTask != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "RX task did not start !\n");
        pmsis_exit(-1);
    }

    // DATA TRANSFER TASK
    xTask = xTaskCreate(tx_task, "camera_task", configMINIMAL_STACK_SIZE * 4,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    if (xTask != pdPASS)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "Camera task did not start !\n");
        pmsis_exit(-1);
    }


    while (1)
    {
        /* DO NOTHING */
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
