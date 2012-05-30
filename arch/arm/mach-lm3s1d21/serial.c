/**************************************************************************
 * Included Files
 **************************************************************************/

#include <mach/hardware.h>

#define LM3S_CONSOLE_BASE LM3S_UART0_BASE

static int serial_getc(void)
{
	/* Wait for a character from the UART */
	while ((lm3s_getreg32(LM3S_CONSOLE_BASE + LM3S_UART_FR_OFFSET) & UART_FR_RXFE));

	return (int)(lm3s_getreg32(LM3S_CONSOLE_BASE + LM3S_UART_DR_OFFSET) & UART_DR_DATA_MASK);
}

static void serial_putc(const char ch)
{
  /* Then send the character */
  lm3s_putreg32((uint32_t)ch, LM3S_CONSOLE_BASE + LM3S_UART_DR_OFFSET);

  while ((lm3s_getreg32(LM3S_CONSOLE_BASE + LM3S_UART_FR_OFFSET) & UART_FR_TXFE) == 0);
}

static int serial_tstc(void)
{
	/* Test for a character from the UART */
	return (lm3s_getreg32(LM3S_CONSOLE_BASE + LM3S_UART_FR_OFFSET) & UART_FR_RXFE) == 0;
}

static void serial_puts(const char *s)
{
	while (*s)
  {
		serial_putc (*s);
    /* If \n, also do \r */
    if (*s == '\n')
      serial_putc('\r');
    s++;
  }
}

void printascii(const char *str)
{
  serial_puts(str);
}

void printch(char ch)
{
  serial_putc(ch);
}
