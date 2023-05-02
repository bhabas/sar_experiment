/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * AI-deck GAP8
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * WiFi image streamer example
 */
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/camera/himax.h"
#include "bsp/buffer.h"

#include "stdio.h"

#include "cpx.h"
#include "wifi.h"

#define MS_2_US 1000
#define IMG_ORIENTATION 0x0101
#define CAM_WIDTH 162
#define CAM_HEIGHT 122

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

static CPXPacket_t txp; // Transfer packet
static CPXPacket_t rxp; // Receive packet


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
    // pi_time_wait_us(1000000);
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

static int wifiConnected = 0;
static int wifiClientConnected = 0;

void rx_task(void *parameters)
{
  while (1)
  {
    cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp);

    WiFiCTRLPacket_t * wifiCtrl = (WiFiCTRLPacket_t*) rxp.data;

    switch (wifiCtrl->cmd)
    {
      case WIFI_CTRL_STATUS_WIFI_CONNECTED:
        printf("Wifi connected (%u.%u.%u.%u)\n",
                          wifiCtrl->data[0], wifiCtrl->data[1],
                          wifiCtrl->data[2], wifiCtrl->data[3]);
        wifiConnected = 1;
        break;
      case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
        printf("Wifi client connection status: %u\n", wifiCtrl->data[0]);
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



pi_buffer_t header;
uint32_t headerSize;
pi_buffer_t footer;
uint32_t footerSize;






void createImageHeaderPacket(CPXPacket_t * packet, uint32_t imgSize) {
  img_header_t *imgHeader = (img_header_t *) packet->data;
  imgHeader->magic = 0xBC;
  imgHeader->width = CAM_WIDTH;
  imgHeader->height = CAM_HEIGHT;
  imgHeader->depth = 1;
  imgHeader->type = 0;
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
    printf("Setting up WiFi AP\n");
    // Set up the routing for the WiFi CTRL packets
    txp.route.destination = CPX_T_ESP32;
    rxp.route.source = CPX_T_GAP8;
    txp.route.function = CPX_F_WIFI_CTRL;
    WiFiCTRLPacket_t * wifiCtrl = (WiFiCTRLPacket_t*) txp.data;
    
    wifiCtrl->cmd = WIFI_CTRL_SET_SSID;
    memcpy(wifiCtrl->data, ssid, sizeof(ssid));
    txp.dataLength = sizeof(ssid);
    cpxSendPacketBlocking(&txp);
    
    wifiCtrl->cmd = WIFI_CTRL_WIFI_CONNECT;
    wifiCtrl->data[0] = 0x01;
    txp.dataLength = 2;
    cpxSendPacketBlocking(&txp);
}

void camera_task(void *parameters)
{
    vTaskDelay(2000);

    setupWiFi();


    printf("Starting camera task...\n");
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

    header.size = 1024;
    header.data = pmsis_l2_malloc(1024);

    footer.size = 10;
    footer.data = pmsis_l2_malloc(10);

    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    // We're reusing the same packet, so initialize the route once
    cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);

    uint32_t imgSize = 0;

    printf("Image: \n");

    pi_perf_conf(1 << PI_PERF_CYCLES | 1 << PI_PERF_ACTIVE_CYCLES);

    while (1)
    {
        pi_perf_start();
        start = pi_perf_read(PI_PERF_CYCLES);
        pi_camera_capture_async(&camera, imgBuff, resolution, pi_task_callback(&task1, capture_done_cb, NULL));
        pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
        xEventGroupWaitBits(evGroup, CAPTURE_DONE_BIT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);
        captureTime = pi_perf_read(PI_PERF_CYCLES) - start;

        pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
        pi_perf_stop();
        pi_perf_reset();


        printf("capture=%d cycles\n",captureTime);

 
        printf("\n\n");
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
    

    printf("Testing2\n");
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

    printf("-- WiFi image streamer example --\n");

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
