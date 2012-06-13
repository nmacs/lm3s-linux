/*
 *  lm3s_uart.c -- TI LM3SXX UART driver
 *
 *  (C) Copyright 2012, Max Nekludov <macscomp@gmail.com>
 *
 * * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Based on:
 *
 *  mcf.c -- Freescale ColdFire UART driver
 *
 *  (C) Copyright 2003-2007, Greg Ungerer <gerg@snapgear.com>
 */

#define DEBUG 1
#define VERBOSE_DEBUG 1

/****************************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/lm3s_uart.h>

#include <mach/hardware.h>
#include <mach/timex.h>

/****************************************************************************/

/*
 *  Local per-uart structure.
 */
struct lm3s_serial_port {
  struct uart_port  port;
  uint32_t rcgc1_mask;
};

/****************************************************************************/

static unsigned int lm3s_tx_empty(struct uart_port *port)
{
  uint32_t regval = lm3s_getreg32(port->membase + LM3S_UART_FR_OFFSET);
  return regval & UART_FR_TXFE ? TIOCSER_TEMT : 0;
}

/****************************************************************************/

static unsigned int lm3s_get_mctrl(struct uart_port *port)
{
  // TODO: Implement for UART1
  return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

/****************************************************************************/

static void lm3s_set_mctrl(struct uart_port *port, unsigned int sigs)
{
  // TODO: Implement for UART1
}

/****************************************************************************/

static void lm3s_start_tx(struct uart_port *port)
{
  unsigned long flags;
  uint32_t regval;

  //dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  regval = lm3s_getreg32(port->membase + LM3S_UART_IM_OFFSET);
  regval |= UART_IM_TXIM;
  lm3s_putreg32(regval, port->membase + LM3S_UART_IM_OFFSET);

  regval = lm3s_getreg32(port->membase + LM3S_UART_CTL_OFFSET);
  regval |= UART_CTL_TXE;
  lm3s_putreg32(regval, port->membase + LM3S_UART_CTL_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_stop_tx(struct uart_port *port)
{
  unsigned long flags;
  uint32_t regval;

  //dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  regval = lm3s_getreg32(port->membase + LM3S_UART_IM_OFFSET);
  regval &= ~UART_IM_TXIM;
  lm3s_putreg32(regval, port->membase + LM3S_UART_IM_OFFSET);

  regval = lm3s_getreg32(port->membase + LM3S_UART_CTL_OFFSET);
  regval &= ~UART_CTL_TXE;
  lm3s_putreg32(regval, port->membase + LM3S_UART_CTL_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_stop_rx(struct uart_port *port)
{
  unsigned long flags;
  uint32_t regval;

  //dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  regval = lm3s_getreg32(port->membase + LM3S_UART_IM_OFFSET);
  regval &= ~(UART_IM_RXIM | UART_IM_RTIM);
  lm3s_putreg32(regval, port->membase + LM3S_UART_IM_OFFSET);

  regval = lm3s_getreg32(port->membase + LM3S_UART_CTL_OFFSET);
  regval &= ~UART_CTL_RXE;
  lm3s_putreg32(regval, port->membase + LM3S_UART_CTL_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_break_ctl(struct uart_port *port, int break_state)
{
  unsigned long flags;
  uint32_t regval;

  //dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  regval = lm3s_getreg32(port->membase + LM3S_UART_LCRH_OFFSET);

  if (break_state != 0)
    regval |= UART_LCRH_BRK;
  else
    regval &= ~UART_LCRH_BRK;

  lm3s_putreg32(regval, port->membase + LM3S_UART_LCRH_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_enable_ms(struct uart_port *port)
{
}

/****************************************************************************/

static int lm3s_startup(struct uart_port *port)
{
  unsigned long flags;

  //dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  /* Reset UART, get it into known state... */
  lm3s_putreg32(0, port->membase + LM3S_UART_CTL_OFFSET);
  lm3s_putreg32(0, port->membase + LM3S_UART_LCRH_OFFSET);

  lm3s_putreg32(UART_LCRH_FEN, port->membase + LM3S_UART_LCRH_OFFSET);

  /* Enable the UART transmitter and receiver */
  lm3s_putreg32(UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN,
                port->membase + LM3S_UART_CTL_OFFSET);

  /* Enable RX interrupts now */
  lm3s_putreg32(UART_IM_RXIM | UART_IM_RTIM,
               port->membase + LM3S_UART_IM_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);

  return 0;
}

/****************************************************************************/

static void lm3s_shutdown(struct uart_port *port)
{
  unsigned long flags;

  //dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  /* Disable all interrupts now */
  lm3s_putreg32(0, port->membase + LM3S_UART_IM_OFFSET);

  /* Reset UART, get it into known state... */
  lm3s_putreg32(0, port->membase + LM3S_UART_CTL_OFFSET);
  lm3s_putreg32(0, port->membase + LM3S_UART_LCRH_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_set_termios(struct uart_port *port, struct ktermios *termios,
  struct ktermios *old)
{
  unsigned long flags;
  unsigned int baud, den, brdi, remainder, divfrac;
  uint32_t ctl, lcrh;

  baud = uart_get_baud_rate(port, termios, old, 0, 230400);
  //dev_dbg(port->dev, "%s: membase %x, new baud %u\n", __func__, port->membase, baud);

  /* Calculate BAUD rate from the SYS clock:
   *
   * "The baud-rate divisor is a 22-bit number consisting of a 16-bit integer
   *  and a 6-bit fractional part. The number formed by these two values is
   *  used by the baud-rate generator to determine the bit period. Having a
   *  fractional baud-rate divider allows the UART to generate all the standard
   *  baud rates.
   *
   * "The 16-bit integer is loaded through the UART Integer Baud-Rate Divisor
   *  (UARTIBRD) register ... and the 6-bit fractional part is loaded with the
   *  UART Fractional Baud-Rate Divisor (UARTFBRD) register... The baud-rate
   *  divisor (BRD) has the following relationship to the system clock (where
   *  BRDI is the integer part of the BRD and BRDF is the fractional part,
   *  separated by a decimal place.):
   *
   *    "BRD = BRDI + BRDF = UARTSysClk / (16 * Baud Rate)
   *
   * "where UARTSysClk is the system clock connected to the UART. The 6-bit
   *  fractional number (that is to be loaded into the DIVFRAC bit field in the
   *  UARTFBRD register) can be calculated by taking the fractional part of the
   *  baud-rate divisor, multiplying it by 64, and adding 0.5 to account for
   *  rounding errors:
   *
   *    "UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5)
   *
   * "The UART generates an internal baud-rate reference clock at 16x the baud-
   *  rate (referred to as Baud16). This reference clock is divided by 16 to
   *  generate the transmit clock, and is used for error detection during receive
   *  operations.
   *
   * "Along with the UART Line Control, High Byte (UARTLCRH) register ..., the
   *  UARTIBRD and UARTFBRD registers form an internal 30-bit register. This
   *  internal register is only updated when a write operation to UARTLCRH is
   *  performed, so any changes to the baud-rate divisor must be followed by a
   *  write to the UARTLCRH register for the changes to take effect. ..."
   */

  den       = baud << 4;
  brdi      = port->uartclk / den;
  remainder = port->uartclk - den * brdi;
  divfrac   = ((remainder << 6) + (den >> 1)) / den;

  spin_lock_irqsave(&port->lock, flags);

  ctl = lm3s_getreg32(port->membase + LM3S_UART_CTL_OFFSET);
  lcrh = lm3s_getreg32(port->membase + LM3S_UART_LCRH_OFFSET);

  lcrh &= ~UART_LCRH_WLEN_MASK;
  switch (termios->c_cflag & CSIZE) {
  case CS5: lcrh |= UART_LCRH_WLEN_5BITS; break;
  case CS6: lcrh |= UART_LCRH_WLEN_6BITS; break;
  case CS7: lcrh |= UART_LCRH_WLEN_7BITS; break;
  case CS8:
  default:  lcrh |= UART_LCRH_WLEN_8BITS; break;
  }

  if (termios->c_cflag & PARENB) {
    lcrh |= UART_LCRH_PEN;
    if (termios->c_cflag & PARODD)
      lcrh &= ~UART_LCRH_EPS;
    else
      lcrh |= UART_LCRH_EPS;

    if (termios->c_cflag & CMSPAR)
      lcrh |= UART_LCRH_SPS;
    else
      lcrh &= ~UART_LCRH_SPS;
  } else {
    lcrh &= ~UART_LCRH_PEN;
    lcrh &= ~UART_LCRH_EPS;
    lcrh &= ~UART_LCRH_SPS;
  }

  if (termios->c_cflag & CSTOPB)
    lcrh |= UART_LCRH_STP2;
  else
    lcrh &= ~UART_LCRH_STP2;

  port->read_status_mask = UART_DR_OE;
  if (termios->c_iflag & INPCK)
    port->read_status_mask |= (UART_DR_FE | UART_DR_PE);
  if (termios->c_iflag & (BRKINT | PARMRK))
    port->read_status_mask |= UART_DR_BE;

  port->ignore_status_mask = 0;
  if (termios->c_iflag & IGNPAR)
    port->ignore_status_mask |= UART_DR_FE | UART_DR_PE;
  if (termios->c_iflag & IGNBRK) {
    port->ignore_status_mask |= UART_DR_BE;
    /*
     * If we're ignoring parity and break indicators,
     * ignore overruns too (for real raw support).
     */
    if (termios->c_iflag & IGNPAR)
      port->ignore_status_mask |= UART_DR_OE;
  }

  /* Diable uart befor chage confuguration */
  lm3s_putreg32(0, port->membase + LM3S_UART_CTL_OFFSET);

  /* Write configuration */
  lm3s_putreg32(brdi, port->membase + LM3S_UART_IBRD_OFFSET);
  lm3s_putreg32(divfrac, port->membase + LM3S_UART_FBRD_OFFSET);
  lm3s_putreg32(lcrh, port->membase + LM3S_UART_CTL_OFFSET);
  lm3s_putreg32(ctl, port->membase + LM3S_UART_CTL_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_rx_chars(struct lm3s_serial_port *pp)
{
  struct uart_port *port = &pp->port;
  unsigned char ch;
  unsigned int flag;
  unsigned int status, rxdata;

  //dev_vdbg(port->dev, "%s\n", __func__);

  while( ((lm3s_getreg32(port->membase + LM3S_UART_FR_OFFSET)) & UART_FR_RXFE) == 0 )
  {
    rxdata = lm3s_getreg32(port->membase + LM3S_UART_DR_OFFSET);
    ch = (unsigned char)(rxdata & UART_DR_DATA_MASK);
    status = rxdata & ~UART_DR_DATA_MASK;

    flag = TTY_NORMAL;
    port->icount.rx++;

    if (status & UART_DR_BE) {
      port->icount.brk++;
      if (uart_handle_break(port))
        continue;
    } else if (status & UART_DR_PE) {
      port->icount.parity++;
    } else if (status & UART_DR_OE) {
      port->icount.overrun++;
    } else if (status & UART_DR_FE) {
      port->icount.frame++;
    }

    status &= port->read_status_mask;

    if (status & UART_DR_BE)
      flag = TTY_BREAK;
    else if (status & UART_DR_PE)
      flag = TTY_PARITY;
    else if (status & UART_DR_FE)
      flag = TTY_FRAME;

    if (uart_handle_sysrq_char(port, ch))
      continue;

    uart_insert_char(port, status, UART_DR_OE, ch, flag);
  }

  tty_flip_buffer_push(port->state->port.tty);
}

/****************************************************************************/

static void lm3s_tx_chars(struct lm3s_serial_port *pp)
{
  struct uart_port *port = &pp->port;
  struct circ_buf *xmit = &port->state->xmit;
  uint32_t regval;

  //dev_dbg(port->dev, "%s\n", __func__);

  if (port->x_char) {
    /* Send special char - probably flow control */
    lm3s_putreg32(port->x_char, port->membase + LM3S_UART_DR_OFFSET);
    port->x_char = 0;
    port->icount.tx++;
    return;
  }

  while ((lm3s_getreg32(port->membase + LM3S_UART_FR_OFFSET) & UART_FR_TXFF) == 0) {
    if (xmit->head == xmit->tail)
      break;
    lm3s_putreg32(xmit->buf[xmit->tail], port->membase + LM3S_UART_DR_OFFSET);
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE -1);
    port->icount.tx++;
  }

  if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
    uart_write_wakeup(port);

  if (xmit->head == xmit->tail) {
    regval = lm3s_getreg32(port->membase + LM3S_UART_IM_OFFSET);
    regval &= ~UART_IM_TXIM;
    lm3s_putreg32(regval, port->membase + LM3S_UART_IM_OFFSET);
  }
}

/****************************************************************************/

static irqreturn_t lm3s_interrupt(int irq, void *data)
{
  struct uart_port *port = data;
  struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
  uint32_t isr;

  //dev_vdbg(port->dev, "%s\n", __func__);

  isr = lm3s_getreg32(port->membase + LM3S_UART_MIS_OFFSET);
  lm3s_putreg32(isr, port->membase + LM3S_UART_ICR_OFFSET);

  if (isr & (UART_MIS_RXMIS | UART_MIS_RTMIS))
    lm3s_rx_chars(pp);
  if (isr & UART_MIS_TXMIS)
    lm3s_tx_chars(pp);

  return IRQ_HANDLED;
}

/****************************************************************************/

static void lm3s_config_port(struct uart_port *port, int flags)
{
  port->type = PORT_LM3S;

  /* Clear mask, so no surprise interrupts. */
  lm3s_putreg32(0, port->membase + LM3S_UART_IM_OFFSET);

  if (request_irq(port->irq, lm3s_interrupt, IRQF_DISABLED, "UART", port))
    dev_err(port->dev, "Unable to attach UART %d "
      "interrupt vector=%d\n", port->line, port->irq);
}

/****************************************************************************/

static const char *lm3s_type(struct uart_port *port)
{
  return (port->type == PORT_LM3S) ? "TI LM3S UART" : NULL;
}

/****************************************************************************/

static int lm3s_request_port(struct uart_port *port)
{
  uint32_t regval;
  struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);

  regval = lm3s_getreg32(LM3S_SYSCON_RCGC1);
  regval |= pp->rcgc1_mask;
  lm3s_putreg32(regval, LM3S_SYSCON_RCGC1);

  return 0;
}

/****************************************************************************/

static void lm3s_release_port(struct uart_port *port)
{
  uint32_t regval;
  struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);

  regval = lm3s_getreg32(LM3S_SYSCON_RCGC1);
  regval &= ~pp->rcgc1_mask;
  lm3s_putreg32(regval, LM3S_SYSCON_RCGC1);
}

/****************************************************************************/

static int lm3s_verify_port(struct uart_port *port, struct serial_struct *ser)
{
  if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_LM3S))
    return -EINVAL;
  return 0;
}

/****************************************************************************/

/*
 *  Define the basic serial functions we support.
 */
static struct uart_ops lm3s_uart_ops = {
  .tx_empty = lm3s_tx_empty,
  .get_mctrl  = lm3s_get_mctrl,
  .set_mctrl  = lm3s_set_mctrl,
  .start_tx = lm3s_start_tx,
  .stop_tx  = lm3s_stop_tx,
  .stop_rx  = lm3s_stop_rx,
  .enable_ms  = lm3s_enable_ms,
  .break_ctl  = lm3s_break_ctl,
  .startup  = lm3s_startup,
  .shutdown = lm3s_shutdown,
  .set_termios  = lm3s_set_termios,
  .type   = lm3s_type,
  .request_port = lm3s_request_port,
  .release_port = lm3s_release_port,
  .config_port  = lm3s_config_port,
  .verify_port  = lm3s_verify_port,
};

/****************************************************************************/

static struct lm3s_serial_port lm3s_ports[LM3S_NUARTS];

/****************************************************************************/
#if defined(CONFIG_SERIAL_LM3S_CONSOLE)
/****************************************************************************/

extern void lm3s_uart_putc(uint32_t uart_base, const char ch);

static void lm3s_serial_console_putchar(struct uart_port *port, int ch)
{
  lm3s_uart_putc((uint32_t)port->membase, ch);
}

/****************************************************************************/

static void lm3s_console_write(struct console *co, const char *s, unsigned int count)
{
  struct lm3s_serial_port *uart = &lm3s_ports[co->index];
  unsigned long flags;

  spin_lock_irqsave(&uart->port.lock, flags);
  uart_console_write(&uart->port, s, count, lm3s_serial_console_putchar);
  spin_unlock_irqrestore(&uart->port.lock, flags);
}

/****************************************************************************/

static int __init lm3s_console_setup(struct console *co, char *options)
{
  struct uart_port *port;
  int baud = CONFIG_SERIAL_LM3S_BAUDRATE;
  int bits = 8;
  int parity = 'n';
  int flow = 'n';

  if ((co->index < 0) || (co->index >= LM3S_NUARTS))
    co->index = 0;

  port = &lm3s_ports[co->index].port;
  if (port->membase == 0)
    return -ENODEV;

  if (options)
    uart_parse_options(options, &baud, &parity, &bits, &flow);

  return uart_set_options(port, co, baud, parity, bits, flow);
}

/****************************************************************************/

static struct uart_driver lm3s_driver;

static struct console lm3s_console = {
  .name   = "ttyS",
  .write    = lm3s_console_write,
  .device   = uart_console_device,
  .setup    = lm3s_console_setup,
  .flags    = CON_PRINTBUFFER,
  .index    = -1,
  .data   = &lm3s_driver,
};

static int __init lm3s_console_init(void)
{
  register_console(&lm3s_console);
  return 0;
}
console_initcall(lm3s_console_init);

#define LM3S_CONSOLE &lm3s_console

/****************************************************************************/
#else
/****************************************************************************/

#define LM3S_CONSOLE NULL

/****************************************************************************/
#endif /* CONFIG_MCF_CONSOLE */
/****************************************************************************/

/*
 *  Define the LM3S UART driver structure.
 */
static struct uart_driver lm3s_driver = {
  .owner        = THIS_MODULE,
  .driver_name  = "lm3s",
  .dev_name     = "ttyS",
  .major        = TTY_MAJOR,
  .minor        = 64,
  .nr           = LM3S_NUARTS,
  .cons         = LM3S_CONSOLE,
};

/****************************************************************************/

static int __devinit lm3s_probe(struct platform_device *pdev)
{
  struct lm3s_platform_uart *platp = pdev->dev.platform_data;
  struct uart_port *port;
  struct lm3s_serial_port *pp;
  int i;

  for (i = 0; ((i < LM3S_NUARTS) && (platp[i].mapbase)); i++) {
    pp = lm3s_ports + i;
    port = &pp->port;

    pp->rcgc1_mask = platp[i].rcgc1_mask;

    port->dev = &pdev->dev;
    port->line = i;
    port->type = PORT_LM3S;
    port->mapbase = platp[i].mapbase;
    port->membase = (platp[i].membase) ? platp[i].membase :
      (unsigned char __iomem *) platp[i].mapbase;
    port->iotype = SERIAL_IO_MEM;
    port->irq = platp[i].irq;
    port->uartclk = CLOCK_TICK_RATE;  // TODO: Set proper uart clock
    port->ops = &lm3s_uart_ops;
    port->flags = ASYNC_BOOT_AUTOCONF;

    uart_add_one_port(&lm3s_driver, port);
  }

  return 0;
}

/****************************************************************************/

static int __devexit lm3s_remove(struct platform_device *pdev)
{
  struct uart_port *port;
  int i;

  for (i = 0; (i < LM3S_NUARTS); i++) {
    port = &lm3s_ports[i].port;
    if (port)
      uart_remove_one_port(&lm3s_driver, port);
  }

  return 0;
}

/****************************************************************************/

static struct platform_driver lm3s_platform_driver = {
  .probe    = lm3s_probe,
  .remove   = __devexit_p(lm3s_remove),
  .driver   = {
    .name = "lm3s-uart",
    .owner  = THIS_MODULE,
  },
};

/****************************************************************************/

static int __init lm3s_init(void)
{
  int rc;

  printk(KERN_INFO "Texas Instruments LM3S UART serial driver\n");

  rc = uart_register_driver(&lm3s_driver);
  if (rc)
    return rc;

  rc = platform_driver_register(&lm3s_platform_driver);
  if (rc)
    return rc;

  return 0;
}

/****************************************************************************/

static void __exit lm3s_exit(void)
{
  platform_driver_unregister(&lm3s_platform_driver);
  uart_unregister_driver(&lm3s_driver);
}

/****************************************************************************/

module_init(lm3s_init);
module_exit(lm3s_exit);

MODULE_AUTHOR("Max Nekludov <macscomp@gmail.com>");
MODULE_DESCRIPTION("TI LM3SXX UART driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lm3s-uart");