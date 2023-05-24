/*
This is an example script that sends an array of float values 
over UART1 to the crazyflie STM micro-processor.
*/

#include "stdio.h"
#include "pmsis.h"

// DEFINE DATA ARRAY
uint8_t value_arr[] = {1,2,3};

static void UART_Task(void)
{
  // SET UART CONFIGURATIONS
  struct pi_uart_conf conf;
  struct pi_device device;
  pi_uart_conf_init(&conf);

  conf.baudrate_bps = 9600;   // Baud Rate - Must match receiver
  conf.enable_tx = 1;         // Enable data transfer (TX)
  conf.enable_rx = 0;         // Disable data reception (RX)

  // OPEN UART CONNECTION
  pi_open_from_conf(&device, &conf);
  printf("[UART] Open\n"); // Print doesn't work here
  if (pi_uart_open(&device))
  {
    printf("[UART] open failed !\n");
    pmsis_exit(-1);
  }
  pi_uart_open(&device);

  uint8_t val = 1;

  while(1)
  {
    // WRITE VALUE ARRAY TO CF OVER UART1
    // value_arr[0] += 1; // Increment first value of array
    pi_uart_write(&device, &val, sizeof(uint8_t));

    // WAIT
    pi_time_wait_us(500000); // Wait 0.5s [500,000 us]
  }

  pmsis_exit(0);
}

int main(void)
{
    return pmsis_kickoff((void *)UART_Task);
}
