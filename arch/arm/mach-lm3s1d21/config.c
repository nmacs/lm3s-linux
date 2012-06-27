#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/io.h>
#include <asm/lm3s_uart.h>
#include <mach/hardware.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>

#include <mach/lm3s_spi.h>

/***************************************************************************/

static struct resource lm3s_spi_resources0[] = {
  {
         .start = LM3S_SSI0_BASE,
         .end = LM3S_SSI0_BASE + SZ_1K - 1,
         .flags = IORESOURCE_MEM,
  }, {
         .start = LM3S1D21_SSI0_IRQ,
         .end = LM3S1D21_SSI0_IRQ,
         .flags = IORESOURCE_IRQ,
  },
};

static int lm3s_spi_cs[] = {GPIO_SSI0_CS_SF, GPIO_SSI0_CS_EE, GPIO_SSI0_CS_ETH};

static struct spi_lm3s_master lm3s_spi_0_data = {
  .chipselect = lm3s_spi_cs,
  .num_chipselect = ARRAY_SIZE(lm3s_spi_cs),
};

struct platform_device lm3s_spi_device0 = {
  .name = "lm3s-spi",
  .id = 0,
  .num_resources = ARRAY_SIZE(lm3s_spi_resources0),
  .resource = lm3s_spi_resources0,
  .dev.platform_data = &lm3s_spi_0_data,
};

static struct mtd_partition uwic_flash_partitions[] = {
  {
    .name = "root",
    .size = MTDPART_SIZ_FULL,
    .offset = 0,
  },
};

static struct flash_platform_data uwic_flash_chip = {
  .name     = "flash",
  .parts    = uwic_flash_partitions,
  .nr_parts = ARRAY_SIZE(uwic_flash_partitions),
  .type     = "m25p32",
};

static struct spi_eeprom uwic_eeprom_chip = {
  .name   = "eeprom",
  .byte_len = 64 * 1024,
  .page_size  = 128,
  .flags    = EE_ADDR2,
};

static struct spi_board_info uwic_spi_board_info[] __initdata = {
  {
    .modalias      = "m25p80",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 0,
    .chip_select   = 0,
    .platform_data = &uwic_flash_chip,
  },
  {
    .modalias      = "at25",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 0,
    .chip_select   = 1,
    .platform_data = &uwic_eeprom_chip,
  },
  {
    .modalias      = "ks8851",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 0,
    .chip_select   = 2,
    .irq           = LM3S1D21_GPIOG_IRQ, // ETH IRQ on PG5
  },
};

/***************************************************************************/

static struct lm3s_platform_uart platform_uarts[] = {
  {
    .mapbase    = LM3S_UART0_BASE,
    .irq        = LM3S1D21_UART0_IRQ,
    .rcgc1_mask = SYSCON_RCGC1_UART0,
  },
  {
    .mapbase  = LM3S_UART1_BASE,
    .irq    = LM3S1D21_UART1_IRQ,
    .rcgc1_mask = SYSCON_RCGC1_UART1,
  },
  {
    .mapbase  = LM3S_UART2_BASE,
    .irq    = LM3S1D21_UART2_IRQ,
    .rcgc1_mask = SYSCON_RCGC1_UART2,
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
  &lm3s_spi_device0,
};

/***************************************************************************/

extern void lm3s_gpioirqenable(uint32_t pinset);

static int __init init_lm3s1d21(void)
{
  platform_add_devices(lm3s_devices, ARRAY_SIZE(lm3s_devices));
  spi_register_board_info(uwic_spi_board_info, ARRAY_SIZE(uwic_spi_board_info));

  lm3s_gpioirqenable(GPIO_ETH_INTRN);

  return 0;
}
arch_initcall(init_lm3s1d21);