/**************************************************************************
 * Included Files
 **************************************************************************/

#include <mach/hardware.h>
#include <mach/pins.h>

#if   defined(CONFIG_STELLARIS_EARLY_CONSOLE_UART0)
#define EARLY_CONSOLE_BASE STLR_UART0_BASE
#elif defined(CONFIG_STELLARIS_EARLY_CONSOLE_UART1)
#define EARLY_CONSOLE_BASE STLR_UART1_BASE
#elif defined(CONFIG_STELLARIS_EARLY_CONSOLE_UART2)
#define EARLY_CONSOLE_BASE STLR_UART2_BASE
#endif

int uart_getc(uint32_t uart_base)
{
	/* Wait for a character from the UART */
	while ((getreg32(uart_base + STLR_UART_FR_OFFSET) & UART_FR_RXFE));

	return (int)(getreg32(uart_base + STLR_UART_DR_OFFSET) & UART_DR_DATA_MASK);
}

void uart_putc(uint32_t uart_base, const char ch)
{
  /* Then send the character */
  putreg32((uint32_t)ch, uart_base + STLR_UART_DR_OFFSET);

  while ((getreg32(uart_base + STLR_UART_FR_OFFSET) & UART_FR_TXFE) == 0);
}

int uart_tstc(uint32_t uart_base)
{
	/* Test for a character from the UART */
	return (getreg32(uart_base + STLR_UART_FR_OFFSET) & UART_FR_RXFE) == 0;
}

void uart_puts(uint32_t uart_base, const char *s)
{
	while (*s)
  {
		uart_putc(uart_base, *s);
    /* If \n, also do \r */
    if (*s == '\n')
      uart_putc(uart_base, '\r');
    s++;
  }
}

void printascii(const char *str)
{
#ifdef EARLY_CONSOLE_BASE
  uart_puts(EARLY_CONSOLE_BASE, str);
#endif
}

void printch(char ch)
{
#ifdef EARLY_CONSOLE_BASE
  uart_putc(EARLY_CONSOLE_BASE, ch);
#endif
}
