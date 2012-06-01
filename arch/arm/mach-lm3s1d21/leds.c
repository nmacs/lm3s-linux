#include <mach/hardware.h>

/***************************************************************************/

void board_red_LED_on(void)
{
  lm3s_configgpio(GPIO_CPU_LED);
  lm3s_gpiowrite(GPIO_CPU_LED, 1);
}

/***************************************************************************/

void board_red_LED_on_and_hang(void)
{
  lm3s_configgpio(GPIO_CPU_LED);
  lm3s_gpiowrite(GPIO_CPU_LED, 1);
  while(1) {}
}

/***************************************************************************/

void board_red_LED_off(void)
{
  lm3s_configgpio(GPIO_CPU_LED);
  lm3s_gpiowrite(GPIO_CPU_LED, 0);
}

/***************************************************************************/

void board_red_LED_toggle(void)
{
  lm3s_configgpio(GPIO_CPU_LED);

  if( lm3s_gpioread(GPIO_CPU_LED, 0) )
    lm3s_gpiowrite(GPIO_CPU_LED, 0);
  else
    lm3s_gpiowrite(GPIO_CPU_LED, 1);
}