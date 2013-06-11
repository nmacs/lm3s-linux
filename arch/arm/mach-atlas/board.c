#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <asm/mach-types.h>
#include <asm/hardware/nvic.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/uart.h>
#include <mach/spi.h>
#include <mach/leds.h>
#include <mach/memory.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <mach/dma.h>
#include <mach/sram.h>
#include <mach/pins.h>

/***************************************************************************/

static struct resource board_spi_resources0[] = {
  {
         .start = STLR_SSI0_BASE,
         .end = STLR_SSI0_BASE + SZ_1K - 1,
         .flags = IORESOURCE_MEM,
  }, {
         .start = STLR_SSI0_IRQ,
         .end = STLR_SSI0_IRQ,
         .flags = IORESOURCE_IRQ,
  },
};

static int board_spi_cs[] = {GPIO_SSI0_CS_SF, GPIO_SSI0_CS_EE, GPIO_SSI0_CS_ETH};

#ifdef CONFIG_STELLARIS_DMA
char __sramdata ssi0_dma_rx_buffer[DMA_MAX_TRANSFER_SIZE];
char __sramdata ssi0_dma_tx_buffer[DMA_MAX_TRANSFER_SIZE];
#endif

static struct spi_stellaris_master board_spi_0_data = {
  .chipselect = board_spi_cs,
  .num_chipselect = ARRAY_SIZE(board_spi_cs),
#ifdef CONFIG_STELLARIS_DMA
	.dma_rx_channel = DMA_CHANNEL_SSI0_RX,
	.dma_tx_channel = DMA_CHANNEL_SSI0_TX,
	.dma_rx_buffer = ssi0_dma_rx_buffer,
	.dma_tx_buffer = ssi0_dma_tx_buffer,
#endif
};

struct platform_device board_spi_device0 = {
  .name = "stellaris-spi",
  .id = 0,
  .num_resources = ARRAY_SIZE(board_spi_resources0),
  .resource = board_spi_resources0,
  .dev.platform_data = &board_spi_0_data,
};

static struct mtd_partition flash_partitions[] = {
  {
    .name = "root",
    .size = MTDPART_SIZ_FULL,
    .offset = 0,
  },
};

static struct flash_platform_data flash_chip = {
  .name     = "flash",
  .parts    = flash_partitions,
  .nr_parts = ARRAY_SIZE(flash_partitions),
  .type     = "m25p32",
};

static struct spi_eeprom eeprom_chip = {
  .name   = "eeprom",
  .byte_len = 64 * 1024,
  .page_size  = 128,
  .flags    = EE_ADDR2,
};

static struct spi_board_info spi_devices[] = {
  {
    .modalias      = "m25p80",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 0,
    .chip_select   = 0,
    .platform_data = &flash_chip,
  },
  {
    .modalias      = "at25",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 0,
    .chip_select   = 1,
    .platform_data = &eeprom_chip,
  },
  {
    .modalias      = "ks8851",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 0,
    .chip_select   = 2,
    .irq           = STLR_GPIOG_IRQ, // ETH IRQ on PG5
  },
};

/***************************************************************************/

#ifdef CONFIG_STELLARIS_DMA
#define UART0_DMA_BUFFER_SIZE 128
#define UART1_DMA_BUFFER_SIZE 512
#define UART2_DMA_BUFFER_SIZE 1024

char __sramdata uart0_dma_tx_buffer[UART0_DMA_BUFFER_SIZE];
char __sramdata uart0_dma_rx_buffer[UART0_DMA_BUFFER_SIZE];

char __sramdata uart1_dma_tx_buffer[UART1_DMA_BUFFER_SIZE];
char __sramdata uart1_dma_rx_buffer[UART1_DMA_BUFFER_SIZE];

char __sramdata uart2_dma_tx_buffer[UART2_DMA_BUFFER_SIZE];
char __sramdata uart2_dma_rx_buffer[UART2_DMA_BUFFER_SIZE];
#endif

static struct stellaris_platform_uart platform_uarts[] = {
  {
    .mapbase    = STLR_UART0_BASE,
    .irq        = STLR_UART0_IRQ,
    .uart_index = 0,
#ifdef CONFIG_STELLARIS_DMA
		.dma_rx_channel = DMA_CHANNEL_UART0_RX,
		.dma_tx_channel = DMA_CHANNEL_UART0_TX,
		.dma_tx_buffer = uart0_dma_tx_buffer,
		.dma_rx_buffer = uart0_dma_rx_buffer,
		.dma_buffer_size = UART0_DMA_BUFFER_SIZE,
#endif
  },
  {
    .mapbase    = STLR_UART1_BASE,
    .irq        = STLR_UART1_IRQ,
    .uart_index = 1,
#ifdef CONFIG_STELLARIS_DMA
		.dma_rx_channel = DMA_CHANNEL_UART1_RX,
		.dma_tx_channel = DMA_CHANNEL_UART1_TX,
		.dma_tx_buffer = uart1_dma_tx_buffer,
		.dma_rx_buffer = uart1_dma_rx_buffer,
		.dma_buffer_size = UART1_DMA_BUFFER_SIZE,
#endif
  },
  {
    .mapbase    = STLR_UART2_BASE,
    .irq        = STLR_UART2_IRQ,
    .uart_index = 2,
#ifdef CONFIG_STELLARIS_DMA
		.dma_rx_channel = DMA_CHANNEL_UART2_RX,
		.dma_tx_channel = DMA_CHANNEL_UART2_TX,
		.dma_tx_buffer = uart2_dma_tx_buffer,
		.dma_rx_buffer = uart2_dma_rx_buffer,
		.dma_buffer_size = UART2_DMA_BUFFER_SIZE,
#endif
  },
  { },
};

static struct platform_device uart_device = {
  .name     = "uart-stellaris",
  .id     = 0,
  .dev.platform_data  = platform_uarts,
};

/***************************************************************************/

static struct platform_device wdt_device = {
	.name		= "stellaris-wdt",
	.id		= -1,
	.num_resources	= 0,
};

/***************************************************************************/

#ifdef CONFIG_ATLAS_CPU_LED
static struct stellaris_led_platdata cpu_led_pdata = {
	.name           = "cpu-led",
	.gpio           = GPIO_CPU_LED,
	.flags          = 0,
	.def_trigger    = "none",
};

static struct platform_device cpu_led = {
	.name	= "stellaris-led",
	.id		= 1,
	.dev		= {
		.platform_data  = &cpu_led_pdata,
	},
};
#endif

/***************************************************************************/

static struct platform_device *devices[] = {
  &uart_device,
  &board_spi_device0,
	&wdt_device,
#ifdef CONFIG_ATLAS_CPU_LED
	&cpu_led,
#endif
};

/***************************************************************************/

#ifdef CONFIG_COPY_TO_SRAM
extern unsigned int __sram_start[], __sram_end[], __sram_load_address[];
#endif

/***************************************************************************/

static void __init board_init(void)
{
	//uint32_t regval;

#ifdef CONFIG_COPY_TO_SRAM
	memcpy(__sram_start, __sram_load_address,
	       (unsigned long)__sram_end - (unsigned long)__sram_start);
#endif

	//regval = SYSCON_DSLPCLKCFG_DSDIVORIDE(0) | SYSCON_DSLPCLKCFG_DSOSCSRC_30KHZ;
	//putreg32(regval, LM3S_SYSCON_DSLPCLKCFG);

	platform_add_devices(devices, ARRAY_SIZE(devices));
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));

	gpioirqenable(GPIO_ETH_INTRN);

	//lm3s_power_init(GPIO_POWER_HOLD);

#ifdef CONFIG_MACH_UWIC_ENABLE_PWRSWITCH
	//lm3s_power_switch_init(GPIO_POWER_FAIL, LM3S1D21_GPIOB_IRQ, CONFIG_MACH_UWIC_POWER_OFF_DELAY);
#endif
}

/***************************************************************************/

static void __init board_init_irq(void)
{
  nvic_init();
}

/***************************************************************************/

static struct sys_timer timer = {
  .init   = stellaris_timer_init,
};

/***************************************************************************/

static void __init board_map_io(void)
{
}

/***************************************************************************/
/***************************************************************************/

MACHINE_START(ATLAS, "atlas")
  .phys_io  = STLR_PERIPH_BASE,
  .io_pg_offst  = (IO_ADDRESS(0) >> 18) & 0xfffc,
  .boot_params  = PHYS_OFFSET + 0x100,
  .map_io   = board_map_io,
  .init_irq = board_init_irq,
  .timer    = &timer,
  .init_machine = board_init,
MACHINE_END