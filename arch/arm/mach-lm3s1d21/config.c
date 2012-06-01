#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/io.h>
#include <asm/lm3s_uart.h>
#include <mach/hardware.h>

/***************************************************************************/

static struct lm3s_platform_uart platform_uarts[] = {
  {
    .mapbase  = LM3S_UART0_BASE,
    .irq    = LM3S1D21_UART0_IRQ,
  },
  {
    .mapbase  = LM3S_UART1_BASE,
    .irq    = LM3S1D21_UART1_IRQ,
  },
  {
    .mapbase  = LM3S_UART2_BASE,
    .irq    = LM3S1D21_UART2_IRQ,
  },
  { },
};

static struct platform_device uart_device = {
  .name     = "lm3s-uart",
  .id     = 0,
  .dev.platform_data  = platform_uarts,
};

/***************************************************************************/

static struct platform_device *lm3s_devices[] __initdata = {
  &uart_device,
};

/***************************************************************************/

static int __init init_lm3s1d21(void)
{
  platform_add_devices(lm3s_devices, ARRAY_SIZE(lm3s_devices));
  return 0;
}
arch_initcall(init_lm3s1d21);