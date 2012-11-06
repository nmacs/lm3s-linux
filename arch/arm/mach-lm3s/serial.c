/**************************************************************************
 * Included Files
 **************************************************************************/

#include <mach/hardware.h>

#ifdef CONFIG_LM3S_EARLY_CONSOLE_UART0
#define LM3S_EARLY_CONSOLE_BASE LM3S_UART0_BASE
#endif

int lm3s_uart_getc(uint32_t uart_base)
{
	/* Wait for a character from the UART */
	while ((lm3s_getreg32(uart_base + LM3S_UART_FR_OFFSET) & UART_FR_RXFE));

	return (int)(lm3s_getreg32(uart_base + LM3S_UART_DR_OFFSET) & UART_DR_DATA_MASK);
}

void lm3s_uart_putc(uint32_t uart_base, const char ch)
{
  /* Then send the character */
  lm3s_putreg32((uint32_t)ch, uart_base + LM3S_UART_DR_OFFSET);

  while ((lm3s_getreg32(uart_base + LM3S_UART_FR_OFFSET) & UART_FR_TXFE) == 0);
}

int lm3s_uart_tstc(uint32_t uart_base)
{
	/* Test for a character from the UART */
	return (lm3s_getreg32(uart_base + LM3S_UART_FR_OFFSET) & UART_FR_RXFE) == 0;
}

void lm3s_uart_puts(uint32_t uart_base, const char *s)
{
	while (*s)
  {
		lm3s_uart_putc (uart_base, *s);
    /* If \n, also do \r */
    if (*s == '\n')
      lm3s_uart_putc(uart_base, '\r');
    s++;
  }
}

void printascii(const char *str)
{
#ifdef LM3S_EARLY_CONSOLE_BASE
  lm3s_uart_puts(LM3S_EARLY_CONSOLE_BASE, str);
#endif
}

void printch(char ch)
{
#ifdef LM3S_EARLY_CONSOLE_BASE
  lm3s_uart_putc(LM3S_EARLY_CONSOLE_BASE, ch);
#endif
}
