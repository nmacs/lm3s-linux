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
};

/****************************************************************************/

static unsigned int lm3s_tx_empty(struct uart_port *port)
{
//   return (readb(port->membase + MCFUART_USR) & MCFUART_USR_TXEMPTY) ?
//     TIOCSER_TEMT : 0;
  return 0;
}

/****************************************************************************/

static unsigned int lm3s_get_mctrl(struct uart_port *port)
{
//   struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
//   unsigned long flags;
//   unsigned int sigs;
//
//   spin_lock_irqsave(&port->lock, flags);
//   sigs = (readb(port->membase + MCFUART_UIPR) & MCFUART_UIPR_CTS) ?
//     0 : TIOCM_CTS;
//   sigs |= (pp->sigs & TIOCM_RTS);
//   sigs |= (mcf_getppdcd(port->line) ? TIOCM_CD : 0);
//   sigs |= (mcf_getppdtr(port->line) ? TIOCM_DTR : 0);
//   spin_unlock_irqrestore(&port->lock, flags);
//   return sigs;

  return 0;
}

/****************************************************************************/

static void lm3s_set_mctrl(struct uart_port *port, unsigned int sigs)
{
//   struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
//   unsigned long flags;
//
//   spin_lock_irqsave(&port->lock, flags);
//   pp->sigs = sigs;
//   mcf_setppdtr(port->line, (sigs & TIOCM_DTR));
//   if (sigs & TIOCM_RTS)
//     writeb(MCFUART_UOP_RTS, port->membase + MCFUART_UOP1);
//   else
//     writeb(MCFUART_UOP_RTS, port->membase + MCFUART_UOP0);
//   spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_start_tx(struct uart_port *port)
{
//   struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
//   unsigned long flags;
//
//   spin_lock_irqsave(&port->lock, flags);
//   pp->imr |= MCFUART_UIR_TXREADY;
//   writeb(pp->imr, port->membase + MCFUART_UIMR);
//   spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_stop_tx(struct uart_port *port)
{
//   struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
//   unsigned long flags;
//
//   spin_lock_irqsave(&port->lock, flags);
//   pp->imr &= ~MCFUART_UIR_TXREADY;
//   writeb(pp->imr, port->membase + MCFUART_UIMR);
//   spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_stop_rx(struct uart_port *port)
{
//   struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
//   unsigned long flags;
//
//   spin_lock_irqsave(&port->lock, flags);
//   pp->imr &= ~MCFUART_UIR_RXREADY;
//   writeb(pp->imr, port->membase + MCFUART_UIMR);
//   spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_break_ctl(struct uart_port *port, int break_state)
{
//   unsigned long flags;
//
//   spin_lock_irqsave(&port->lock, flags);
//   if (break_state == -1)
//     writeb(MCFUART_UCR_CMDBREAKSTART, port->membase + MCFUART_UCR);
//   else
//     writeb(MCFUART_UCR_CMDBREAKSTOP, port->membase + MCFUART_UCR);
//   spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_enable_ms(struct uart_port *port)
{
}

/****************************************************************************/

static int lm3s_startup(struct uart_port *port)
{
//   struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
//   unsigned long flags;
//
//   spin_lock_irqsave(&port->lock, flags);
//
//   /* Reset UART, get it into known state... */
//   writeb(MCFUART_UCR_CMDRESETRX, port->membase + MCFUART_UCR);
//   writeb(MCFUART_UCR_CMDRESETTX, port->membase + MCFUART_UCR);
//
//   /* Enable the UART transmitter and receiver */
//   writeb(MCFUART_UCR_RXENABLE | MCFUART_UCR_TXENABLE,
//     port->membase + MCFUART_UCR);
//
//   /* Enable RX interrupts now */
//   pp->imr = MCFUART_UIR_RXREADY;
//   writeb(pp->imr, port->membase + MCFUART_UIMR);
//
//   spin_unlock_irqrestore(&port->lock, flags);

  return 0;
}

/****************************************************************************/

static void lm3s_shutdown(struct uart_port *port)
{
//   struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
//   unsigned long flags;
//
//   spin_lock_irqsave(&port->lock, flags);
//
//   /* Disable all interrupts now */
//   pp->imr = 0;
//   writeb(pp->imr, port->membase + MCFUART_UIMR);
//
//   /* Disable UART transmitter and receiver */
//   writeb(MCFUART_UCR_CMDRESETRX, port->membase + MCFUART_UCR);
//   writeb(MCFUART_UCR_CMDRESETTX, port->membase + MCFUART_UCR);
//
//   spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_set_termios(struct uart_port *port, struct ktermios *termios,
  struct ktermios *old)
{
//   unsigned long flags;
//   unsigned int baud, baudclk;
// #if defined(CONFIG_M5272)
//   unsigned int baudfr;
// #endif
//   unsigned char mr1, mr2;
//
//   baud = uart_get_baud_rate(port, termios, old, 0, 230400);
// #if defined(CONFIG_M5272)
//   baudclk = (MCF_BUSCLK / baud) / 32;
//   baudfr = (((MCF_BUSCLK / baud) + 1) / 2) % 16;
// #else
//   baudclk = ((MCF_BUSCLK / baud) + 16) / 32;
// #endif
//
//   mr1 = MCFUART_MR1_RXIRQRDY | MCFUART_MR1_RXERRCHAR;
//   mr2 = 0;
//
//   switch (termios->c_cflag & CSIZE) {
//   case CS5: mr1 |= MCFUART_MR1_CS5; break;
//   case CS6: mr1 |= MCFUART_MR1_CS6; break;
//   case CS7: mr1 |= MCFUART_MR1_CS7; break;
//   case CS8:
//   default:  mr1 |= MCFUART_MR1_CS8; break;
//   }
//
//   if (termios->c_cflag & PARENB) {
//     if (termios->c_cflag & CMSPAR) {
//       if (termios->c_cflag & PARODD)
//         mr1 |= MCFUART_MR1_PARITYMARK;
//       else
//         mr1 |= MCFUART_MR1_PARITYSPACE;
//     } else {
//       if (termios->c_cflag & PARODD)
//         mr1 |= MCFUART_MR1_PARITYODD;
//       else
//         mr1 |= MCFUART_MR1_PARITYEVEN;
//     }
//   } else {
//     mr1 |= MCFUART_MR1_PARITYNONE;
//   }
//
//   if (termios->c_cflag & CSTOPB)
//     mr2 |= MCFUART_MR2_STOP2;
//   else
//     mr2 |= MCFUART_MR2_STOP1;
//
//   if (termios->c_cflag & CRTSCTS) {
//     mr1 |= MCFUART_MR1_RXRTS;
//     mr2 |= MCFUART_MR2_TXCTS;
//   }
//
//   spin_lock_irqsave(&port->lock, flags);
//   writeb(MCFUART_UCR_CMDRESETRX, port->membase + MCFUART_UCR);
//   writeb(MCFUART_UCR_CMDRESETTX, port->membase + MCFUART_UCR);
//   writeb(MCFUART_UCR_CMDRESETMRPTR, port->membase + MCFUART_UCR);
//   writeb(mr1, port->membase + MCFUART_UMR);
//   writeb(mr2, port->membase + MCFUART_UMR);
//   writeb((baudclk & 0xff00) >> 8, port->membase + MCFUART_UBG1);
//   writeb((baudclk & 0xff), port->membase + MCFUART_UBG2);
// #if defined(CONFIG_M5272)
//   writeb((baudfr & 0x0f), port->membase + MCFUART_UFPD);
// #endif
//   writeb(MCFUART_UCSR_RXCLKTIMER | MCFUART_UCSR_TXCLKTIMER,
//     port->membase + MCFUART_UCSR);
//   writeb(MCFUART_UCR_RXENABLE | MCFUART_UCR_TXENABLE,
//     port->membase + MCFUART_UCR);
//   spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void lm3s_rx_chars(struct lm3s_serial_port *pp)
{
//   struct uart_port *port = &pp->port;
//   unsigned char status, ch, flag;
//
//   while ((status = readb(port->membase + MCFUART_USR)) & MCFUART_USR_RXREADY) {
//     ch = readb(port->membase + MCFUART_URB);
//     flag = TTY_NORMAL;
//     port->icount.rx++;
//
//     if (status & MCFUART_USR_RXERR) {
//       writeb(MCFUART_UCR_CMDRESETERR,
//         port->membase + MCFUART_UCR);
//
//       if (status & MCFUART_USR_RXBREAK) {
//         port->icount.brk++;
//         if (uart_handle_break(port))
//           continue;
//       } else if (status & MCFUART_USR_RXPARITY) {
//         port->icount.parity++;
//       } else if (status & MCFUART_USR_RXOVERRUN) {
//         port->icount.overrun++;
//       } else if (status & MCFUART_USR_RXFRAMING) {
//         port->icount.frame++;
//       }
//
//       status &= port->read_status_mask;
//
//       if (status & MCFUART_USR_RXBREAK)
//         flag = TTY_BREAK;
//       else if (status & MCFUART_USR_RXPARITY)
//         flag = TTY_PARITY;
//       else if (status & MCFUART_USR_RXFRAMING)
//         flag = TTY_FRAME;
//     }
//
//     if (uart_handle_sysrq_char(port, ch))
//       continue;
//     uart_insert_char(port, status, MCFUART_USR_RXOVERRUN, ch, flag);
//   }
//
//   tty_flip_buffer_push(port->state->port.tty);
}

/****************************************************************************/

static void lm3s_tx_chars(struct lm3s_serial_port *pp)
{
//   struct uart_port *port = &pp->port;
//   struct circ_buf *xmit = &port->state->xmit;
//
//   if (port->x_char) {
//     /* Send special char - probably flow control */
//     writeb(port->x_char, port->membase + MCFUART_UTB);
//     port->x_char = 0;
//     port->icount.tx++;
//     return;
//   }
//
//   while (readb(port->membase + MCFUART_USR) & MCFUART_USR_TXREADY) {
//     if (xmit->head == xmit->tail)
//       break;
//     writeb(xmit->buf[xmit->tail], port->membase + MCFUART_UTB);
//     xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE -1);
//     port->icount.tx++;
//   }
//
//   if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
//     uart_write_wakeup(port);
//
//   if (xmit->head == xmit->tail) {
//     pp->imr &= ~MCFUART_UIR_TXREADY;
//     writeb(pp->imr, port->membase + MCFUART_UIMR);
//   }
}

/****************************************************************************/

static irqreturn_t lm3s_interrupt(int irq, void *data)
{
  //struct uart_port *port = data;
  //struct lm3s_serial_port *pp = container_of(port, struct lm3s_serial_port, port);
  //unsigned int isr;

  //isr = readb(port->membase + MCFUART_UISR) & pp->imr;
  //if (isr & MCFUART_UIR_RXREADY)
  //  lm3s_rx_chars(pp);
  //if (isr & MCFUART_UIR_TXREADY)
  //  lm3s_tx_chars(pp);
  return IRQ_HANDLED;
}

/****************************************************************************/

static void lm3s_config_port(struct uart_port *port, int flags)
{
  port->type = PORT_LM3S;

  /* Clear mask, so no surprise interrupts. */
  //writeb(0, port->membase + MCFUART_UIMR);

  if (request_irq(port->irq, lm3s_interrupt, IRQF_DISABLED, "UART", port))
    printk(KERN_ERR "MCF: unable to attach ColdFire UART %d "
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
  /* UARTs always present */
  return 0;
}

/****************************************************************************/

static void lm3s_release_port(struct uart_port *port)
{
  /* Nothing to release... */
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
  int i;

  for (i = 0; ((i < LM3S_NUARTS) && (platp[i].mapbase)); i++) {
    port = &lm3s_ports[i].port;

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