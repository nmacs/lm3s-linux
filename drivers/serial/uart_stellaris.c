/*
 *  uart_stellaris.c -- TI Stellaris UART driver
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

//#define DEBUG 1
//#define VERBOSE_DEBUG 1

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

#include <mach/hardware.h>
#include <mach/uart.h>
#include <mach/timex.h>
#include <mach/sram.h>
#include <mach/dma.h>

#if !defined(CONFIG_UART_CLOCK_TICK_RATE) || CONFIG_UART_CLOCK_TICK_RATE == 0
#define CONFIG_UART_CLOCK_TICK_RATE CLOCK_TICK_RATE
#endif

/****************************************************************************/

#define RX_SLOT_SIZE(pp) ((pp)->dma_buffer_size / 2)

/*
 *  Local per-uart structure.
 */
struct stellaris_serial_port {
	struct uart_port  port;

	uint32_t uart_index;
#ifdef CONFIG_STELLARIS_DMA
	uint32_t dma_buffer_size;

	uint32_t dma_tx_channel;
	void    *dma_tx_buffer;
	uint32_t tx_busy;

	uint32_t dma_rx_channel;
	void    *rx_slot_a;
	void    *rx_slot_b;
	uint32_t rx_busy;

	int   cur_bytes_received;

	struct tasklet_struct rx_tasklet;
	struct timer_list rx_timer;
#endif
	int dtr_gpio;
	int rts_gpio;
	
	int flags;
};

/****************************************************************************/

static int __sram tx_chars(struct stellaris_serial_port *pp);
static void __sram start_rx_dma(struct stellaris_serial_port *pp);
static void stop_rx(struct uart_port *port);

/****************************************************************************/

#ifdef CONFIG_STELLARIS_DMA

static void start_dma_rx(struct stellaris_serial_port *pp)
{
	dev_vdbg(pp->port.dev, "%s\n", __func__);
	dma_start_xfer(pp->dma_rx_channel);
#ifdef CONFIG_ARCH_TM4C
	{
		uint32_t regval2 = getreg32(pp->port.membase + STLR_UART_DMACTL_OFFSET);
		regval2 |= UART_DMACTL_RXDMAE;
		putreg32(regval2, pp->port.membase + STLR_UART_DMACTL_OFFSET);
	}
#endif
}

static void stop_dma_rx(struct stellaris_serial_port *pp)
{
	dev_vdbg(pp->port.dev, "%s\n", __func__);
	dma_stop_xfer(pp->dma_rx_channel);
#ifdef CONFIG_ARCH_TM4C
	{
		uint32_t regval2 = getreg32(pp->port.membase + STLR_UART_DMACTL_OFFSET);
		regval2 &= ~UART_DMACTL_RXDMAE;
		putreg32(regval2, pp->port.membase + STLR_UART_DMACTL_OFFSET);
		putreg32(UART_ICR_DMARX, pp->port.membase + STLR_UART_ICR_OFFSET);
	}
#endif
}

static void start_dma_tx(struct stellaris_serial_port *pp)
{
	dev_vdbg(pp->port.dev, "%s\n", __func__);
	dma_start_xfer(pp->dma_tx_channel);
#ifdef CONFIG_ARCH_TM4C
	{
		uint32_t regval2 = getreg32(pp->port.membase + STLR_UART_DMACTL_OFFSET);
		regval2 |= UART_DMACTL_TXDMAE;
		putreg32(regval2, pp->port.membase + STLR_UART_DMACTL_OFFSET);
	}
#endif
}

static void stop_dma_tx(struct stellaris_serial_port *pp)
{
	dev_vdbg(pp->port.dev, "%s\n", __func__);
	dma_stop_xfer(pp->dma_tx_channel);
#ifdef CONFIG_ARCH_TM4C
	{
		uint32_t regval2 = getreg32(pp->port.membase + STLR_UART_DMACTL_OFFSET);
		regval2 &= ~UART_DMACTL_TXDMAE;
		putreg32(regval2, pp->port.membase + STLR_UART_DMACTL_OFFSET);
		putreg32(UART_ICR_DMATX, pp->port.membase + STLR_UART_ICR_OFFSET);
	}
#endif
}
#endif

static void enable_uart(struct uart_port *port)
{
  uint32_t regval;
  struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);

	dev_vdbg(port->dev, "%s\n", __func__);

	uart_clock_ctrl(pp->uart_index, SYS_ENABLE_CLOCK);
	
#ifdef CONFIG_UART_CLOCK_TICK_RATE
	/* Select alternative clock source */
	putreg32(UART_UARTCC_CS_ALT, port->membase + STLR_UART_UARTCC_OFFSET);
#endif

  /* Clear mask, so no surprise interrupts. */
  putreg32(0, port->membase + STLR_UART_IM_OFFSET);

  regval = getreg32(port->membase + STLR_UART_LCRH_OFFSET);
  regval |= UART_LCRH_FEN;
  putreg32(regval, port->membase + STLR_UART_LCRH_OFFSET);

	regval = getreg32(port->membase + STLR_UART_IFLS_OFFSET);
	regval = (regval & ~UART_IFLS_RXIFLSEL_MASK) | UART_IFLS_RXIFLSEL_18th;
	putreg32(regval, port->membase + STLR_UART_IFLS_OFFSET);

  regval = getreg32(port->membase + STLR_UART_CTL_OFFSET);
  regval |= UART_CTL_UARTEN;
  putreg32(regval, port->membase + STLR_UART_CTL_OFFSET);

#ifdef CONFIG_STELLARIS_DMA
	dev_vdbg(port->dev, "%s enable port dma\n", __func__);
#if defined(CONFIG_ARCH_TM4C)
	putreg32(UART_IM_DMARX | UART_IM_DMATX,
	              port->membase + STLR_UART_IM_OFFSET);
#elif defined(CONFIG_ARCH_LM3S)
	putreg32(UART_DMACTL_TXDMAE | UART_DMACTL_RXDMAE,
	              port->membase + STLR_UART_DMACTL_OFFSET);
#endif
	pp->tx_busy = 0;
	pp->rx_busy = 0;
	tasklet_enable(&pp->rx_tasklet);
#endif
}

static void disable_uart(struct uart_port *port)
{
  uint32_t regval;
  struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);

	dev_vdbg(port->dev, "%s\n", __func__);

  /* Disable all interrupts now */
	putreg32(0, port->membase + STLR_UART_IM_OFFSET);

#ifdef CONFIG_STELLARIS_DMA
	tasklet_disable(&pp->rx_tasklet);
	tasklet_kill(&pp->rx_tasklet);
	del_timer_sync(&pp->rx_timer);
	stop_dma_tx(pp);
#endif

  regval = getreg32(port->membase + STLR_UART_CTL_OFFSET);
  regval &= ~UART_CTL_UARTEN;
  putreg32(regval, port->membase + STLR_UART_CTL_OFFSET);

  uart_clock_ctrl(pp->uart_index, SYS_DISABLE_CLOCK);
}

/****************************************************************************/

static unsigned int tx_empty(struct uart_port *port)
{
  uint32_t regval = getreg32(port->membase + STLR_UART_FR_OFFSET);
  return regval & UART_FR_TXFE ? TIOCSER_TEMT : 0;
}

/****************************************************************************/

static unsigned int get_mctrl(struct uart_port *port)
{
	dev_vdbg(port->dev, "%s\n", __func__);
  // TODO: Implement for UART1
  return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

/****************************************************************************/

static void set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);
//	uint32_t regval = 0;

	dev_vdbg(port->dev, "%s [mmio:%p]\n", __func__, port->membase);

//	regval = getreg32(port->membase + STLR_UART_CTL_OFFSET);

	if (pp->flags & STLR_UART_HAS_RTS)
	{
		if (mctrl & TIOCM_RTS)
		{
			if (pp->rts_gpio)
				gpiowrite(pp->rts_gpio, pp->flags & STLR_UART_INVERT_RTS ? 0 : 1);
//			else
//				regval |= UART_CTL_RTS;
		}
		else
		{
			if (pp->rts_gpio)
				gpiowrite(pp->rts_gpio, pp->flags & STLR_UART_INVERT_RTS ? 1 : 0);
//			else
//				regval &= ~UART_CTL_RTS;
		}
	}

	if (pp->flags & STLR_UART_HAS_DTR)
	{
		if (mctrl & TIOCM_DTR)
		{
			if (pp->dtr_gpio)
				gpiowrite(pp->rts_gpio, pp->flags & STLR_UART_INVERT_DTR ? 0 : 1);
//			else
//				regval |= UART_CTL_DTR;
		}
		else
		{
			if (pp->dtr_gpio)
				gpiowrite(pp->rts_gpio, pp->flags & STLR_UART_INVERT_DTR ? 1 : 0);
//			else
//				regval &= ~UART_CTL_DTR;
		}
	}
	
	//putreg32(regval, port->membase + STLR_UART_CTL_OFFSET);
}

/****************************************************************************/

static void start_tx(struct uart_port *port)
{
  uint32_t regval;
  struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);

  dev_dbg(port->dev, "%s\n", __func__);

	regval = getreg32(port->membase + STLR_UART_IM_OFFSET);

#ifdef CONFIG_STELLARIS_DMA
	if( !pp->tx_busy )
		tx_chars(pp);
  else
		dev_dbg(port->dev, "%s port busy\n", __func__);
#else
	if(tx_chars(pp) )
	{
		regval |= UART_IM_TXIM;
		putreg32(regval, port->membase + STLR_UART_IM_OFFSET);
	}
#endif
}

/****************************************************************************/

static void stop_tx(struct uart_port *port)
{
#ifdef CONFIG_STELLARIS_DMA
	struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);
#else
	uint32_t regval;
#endif

  dev_dbg(port->dev, "%s\n", __func__);

#ifdef CONFIG_STELLARIS_DMA
	stop_dma_tx(pp);
	pp->tx_busy = 0;
#else
  regval = getreg32(port->membase + STLR_UART_IM_OFFSET);
  regval &= ~UART_IM_TXIM;
  putreg32(regval, port->membase + STLR_UART_IM_OFFSET);
#endif
}

/****************************************************************************/

static void stop_rx(struct uart_port *port)
{
#ifdef CONFIG_STELLARIS_DMA
	struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);
#else
	uint32_t regval;
#endif

  dev_dbg(port->dev, "%s\n", __func__);

#ifdef CONFIG_STELLARIS_DMA
	tasklet_kill(&pp->rx_tasklet);
	del_timer_sync(&pp->rx_timer);
	stop_dma_rx(pp);
#else
  regval = getreg32(port->membase + STLR_UART_IM_OFFSET);
  regval &= ~(UART_IM_RXIM | UART_IM_RTIM);
  putreg32(regval, port->membase + STLR_UART_IM_OFFSET);
#endif
}

/****************************************************************************/

static void break_ctl(struct uart_port *port, int break_state)
{
  unsigned long flags;
  uint32_t regval;

  dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  regval = getreg32(port->membase + STLR_UART_LCRH_OFFSET);

  if (break_state != 0)
    regval |= UART_LCRH_BRK;
  else
    regval &= ~UART_LCRH_BRK;

  putreg32(regval, port->membase + STLR_UART_LCRH_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

static void enable_ms(struct uart_port *port)
{
}

/****************************************************************************/

static int startup(struct uart_port *port)
{
  unsigned long flags;
#ifdef CONFIG_STELLARIS_DMA
	struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);
#endif

  dev_dbg(port->dev, "%s\n", __func__);

  spin_lock_irqsave(&port->lock, flags);

  enable_uart(port);

#ifdef CONFIG_STELLARIS_DMA
	start_rx_dma(pp);
#else
  /* Enable RX interrupts now */
  putreg32(UART_IM_RXIM | UART_IM_RTIM,
                port->membase + STLR_UART_IM_OFFSET);
#endif

  spin_unlock_irqrestore(&port->lock, flags);

  return 0;
}

/****************************************************************************/

static void shutdown(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);
	disable_uart(port);
}

/****************************************************************************/

static void set_termios(struct uart_port *port, struct ktermios *termios,
  struct ktermios *old)
{
  unsigned long flags;
  unsigned int baud, den, brdi, remainder, divfrac;
  uint32_t ctl, lcrh;

  baud = uart_get_baud_rate(port, termios, old, 0, 230400);

  dev_dbg(port->dev, "%s: membase %p, new baud %u\n", __func__, port->membase, baud);

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

  ctl = getreg32(port->membase + STLR_UART_CTL_OFFSET);
  lcrh = getreg32(port->membase + STLR_UART_LCRH_OFFSET);

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
  putreg32(0, port->membase + STLR_UART_CTL_OFFSET);

  /* Write configuration */
  putreg32(brdi, port->membase + STLR_UART_IBRD_OFFSET);
  putreg32(divfrac, port->membase + STLR_UART_FBRD_OFFSET);
  putreg32(lcrh, port->membase + STLR_UART_LCRH_OFFSET);
  putreg32(ctl, port->membase + STLR_UART_CTL_OFFSET);

  spin_unlock_irqrestore(&port->lock, flags);
}

/****************************************************************************/

#ifdef CONFIG_STELLARIS_DMA
static void __sram start_rx_dma(struct stellaris_serial_port *pp)
{
	struct uart_port *port = &pp->port;
	void *slot = pp->rx_slot_a;

	dma_setup_xfer(pp->dma_rx_channel,
	               slot,
	               port->membase + STLR_UART_DR_OFFSET,
	               1,
	               DMA_XFER_DEVICE_TO_MEMORY | DMA_XFER_UNIT_BYTE | DMA_XFER_MODE_PINGPONG);
	dma_setup_xfer(pp->dma_rx_channel,
	               (char*)slot + 1,
	               port->membase + STLR_UART_DR_OFFSET,
	               RX_SLOT_SIZE(pp) - 1,
	               DMA_XFER_DEVICE_TO_MEMORY | DMA_XFER_UNIT_BYTE | DMA_XFER_MODE_PINGPONG | DMA_XFER_ALT);

	start_dma_rx(pp);
}

static void __sram rx_chars(struct stellaris_serial_port *pp, int timeout);

static void __sram rx_chars_timeout(unsigned long data)
{
	struct stellaris_serial_port *pp = (struct stellaris_serial_port*)data;
	struct uart_port *port = &pp->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	rx_chars(pp, 1 /*timeout*/);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void __sram do_rx_chars(unsigned long data)
{
	struct stellaris_serial_port *pp = (struct stellaris_serial_port*)data;
	struct uart_port *port = &pp->port;
	struct tty_struct *tty = port->state->port.tty;
	unsigned char* slot = pp->rx_slot_b;
	int bytes_received = pp->cur_bytes_received;
	unsigned long flags;

	dev_vdbg(port->dev, "%s bytes_received %i\n", __func__, bytes_received);

	tty_insert_flip_string(tty, slot, bytes_received);
	port->icount.rx += bytes_received;

	tty_schedule_flip(tty);

	spin_lock_irqsave(&port->lock, flags);
	pp->rx_busy = 0;
	rx_chars(pp, 0);
	spin_unlock_irqrestore(&port->lock, flags);
}
#endif

static void __sram rx_chars(struct stellaris_serial_port *pp, int timeout)
{
	struct uart_port *port = &pp->port;
#ifdef CONFIG_STELLARIS_DMA
	int bytes_received;
#else
  unsigned char ch;
  unsigned int flag;
  unsigned int status, rxdata;
#endif

  dev_vdbg(port->dev, "%s\n", __func__);

#ifdef CONFIG_STELLARIS_DMA
	if (get_units_left(pp->dma_rx_channel, 1) != 0) {
		if (!timeout) {
			mod_timer(&pp->rx_timer, jiffies + 1);
			return;
		}
	}

	if (!timeout)
		del_timer(&pp->rx_timer);

	if( pp->rx_busy )
	{
		if (get_units_left(pp->dma_rx_channel, 1) == 0)
		{
			stop_dma_rx(pp);
			port->icount.overrun++;
			dev_notice(port->dev, "%s: overrun detected [mmio:%p]\n", __func__, port->membase);
		}
		dev_vdbg(port->dev, "%s rx_busy\n", __func__);
		return;
	}

	stop_dma_rx(pp);

	swap(pp->rx_slot_a, pp->rx_slot_b);

	bytes_received  = RX_SLOT_SIZE(pp);
	bytes_received -= get_units_left(pp->dma_rx_channel, 0);
	bytes_received -= get_units_left(pp->dma_rx_channel, 1);

	start_rx_dma(pp);
	
	dev_vdbg(port->dev, "%s bytes_received %i\n", __func__, bytes_received);

	if( bytes_received > 0 )
	{
		pp->cur_bytes_received = bytes_received;
		pp->rx_busy = 1;
		tasklet_schedule(&pp->rx_tasklet);
	}
#else
  while( ((getreg32(port->membase + STLR_UART_FR_OFFSET)) & UART_FR_RXFE) == 0 )
  {
    rxdata = getreg32(port->membase + STLR_UART_DR_OFFSET);
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

    //if (uart_handle_sysrq_char(port, ch))
      //continue;
    //uart_insert_char(port, status, UART_DR_OE, ch, flag);

		{
			struct tty_struct *tty = port->state->port.tty;
			tty_insert_flip_char(tty, ch, flag);
		}
  }

  tty_flip_buffer_push(port->state->port.tty);
#endif
}

/****************************************************************************/

static int __sram tx_chars(struct stellaris_serial_port *pp)
{
  struct uart_port *port = &pp->port;
  struct circ_buf *xmit = &port->state->xmit;
#ifdef CONFIG_STELLARIS_DMA
	size_t xfer_size;
	size_t bytes_to_transmit;
#else
	uint32_t regval;
#endif

  dev_vdbg(port->dev, "%s\n", __func__);

  if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		dev_vdbg(port->dev, "%s TX complete\n", __func__);
#ifdef CONFIG_STELLARIS_DMA
		stop_dma_tx(pp);
		pp->tx_busy = 0;
#else
		regval = getreg32(port->membase + STLR_UART_IM_OFFSET);
    regval &= ~UART_IM_TXIM;
    putreg32(regval, port->membase + STLR_UART_IM_OFFSET);
#endif
    return 0;
  }

  if (port->x_char) {
    /* Send special char - probably flow control */
    putreg32(port->x_char, port->membase + STLR_UART_DR_OFFSET);
    port->x_char = 0;
    port->icount.tx++;
  }

#ifdef CONFIG_STELLARIS_DMA
	pp->tx_busy = 1;
	bytes_to_transmit = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
  xfer_size = min(bytes_to_transmit, pp->dma_buffer_size);
	dma_memcpy(pp->dma_tx_buffer, xmit->buf + xmit->tail, xfer_size);
	xmit->tail = (xmit->tail + xfer_size) & (UART_XMIT_SIZE - 1);
	dma_setup_xfer(pp->dma_tx_channel,
								 port->membase + STLR_UART_DR_OFFSET,
								 pp->dma_tx_buffer,
								 xfer_size,
								 DMA_XFER_MEMORY_TO_DEVICE | DMA_XFER_UNIT_BYTE);
	
	dev_vdbg(port->dev, "%s: dma_ch %x, xfer_size %u, dst %p\n",
					 __func__, pp->dma_tx_channel, xfer_size, port->membase + STLR_UART_DR_OFFSET);
	
	start_dma_tx(pp);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
    uart_write_wakeup(port);
#else
  while ((getreg32(port->membase + STLR_UART_FR_OFFSET) & UART_FR_TXFF) == 0) {
    if (xmit->head == xmit->tail)
      break;
    putreg32(xmit->buf[xmit->tail], port->membase + STLR_UART_DR_OFFSET);
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE -1);
    port->icount.tx++;
  }

  if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
    uart_write_wakeup(port);

  if (xmit->head == xmit->tail) {
		dev_vdbg(port->dev, "%s TX complete\n", __func__);
    regval = getreg32(port->membase + STLR_UART_IM_OFFSET);
    regval &= ~UART_IM_TXIM;
    putreg32(regval, port->membase + STLR_UART_IM_OFFSET);
    return 0;
  }
#endif

  return 1;
}

/****************************************************************************/

static irqreturn_t __sram interrupt(int irq, void *data)
{
  struct uart_port *port = data;
  struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);
  uint32_t isr;

  isr = getreg32(port->membase + STLR_UART_MIS_OFFSET);
  putreg32(isr, port->membase + STLR_UART_ICR_OFFSET);

	dev_vdbg(port->dev, "%s ISR 0x%x\n", __func__, isr);

#ifdef CONFIG_STELLARIS_DMA
#if defined(CONFIG_ARCH_LM3S)
	if (dma_ack_interrupt(pp->dma_rx_channel))
	{
		dev_vdbg(port->dev, "%s RX\n", __func__);
		rx_chars(pp, 0);
	}

	if (dma_ack_interrupt(pp->dma_tx_channel))
	{
		dev_vdbg(port->dev, "%s TX\n", __func__);
		tx_chars(pp);
	}
#elif defined(CONFIG_ARCH_TM4C)
	if (isr & UART_RIS_DMARX)
	{
		dev_vdbg(port->dev, "%s RX\n", __func__);
		rx_chars(pp, 0);
	}

	if (isr & UART_RIS_DMATX)
	{
		dev_vdbg(port->dev, "%s TX\n", __func__);
		stop_dma_tx(pp);
		tx_chars(pp);
	}
#endif
#else
  if (isr & (UART_MIS_RXMIS | UART_MIS_RTMIS))
	{
		dev_vdbg(port->dev, "%s RX\n", __func__);
    rx_chars(pp, 0);
	}

  if (isr & UART_MIS_TXMIS)
	{
		dev_vdbg(port->dev, "%s TX\n", __func__);
    tx_chars(pp);
	}
#endif

	dev_vdbg(port->dev, "%s ISR done\n", __func__);

  return IRQ_HANDLED;
}

/****************************************************************************/

static void config_port(struct uart_port *port, int flags)
{
#ifdef CONFIG_STELLARIS_DMA
	struct stellaris_serial_port *pp = container_of(port, struct stellaris_serial_port, port);
#endif
	dev_vdbg(port->dev, "%s\n", __func__);

	port->type = PORT_STELLARIS;

	if (request_irq(port->irq, interrupt, IRQF_DISABLED, "UART", port))
		dev_err(port->dev, "Unable to attach UART %d "
			"interrupt vector=%d\n", port->line, port->irq);

#ifdef CONFIG_STELLARIS_DMA
	dev_vdbg(port->dev, "%s setup DMA channels\n", __func__);
	dma_setup_channel(pp->dma_tx_channel, DMA_DEFAULT_CONFIG);
	dma_setup_channel(pp->dma_rx_channel, DMA_DEFAULT_CONFIG);
#endif
}

/****************************************************************************/

static const char *get_type(struct uart_port *port)
{
  return (port->type == PORT_STELLARIS) ? "TI Stellaris UART" : NULL;
}

/****************************************************************************/

static int request_port(struct uart_port *port)
{
	dev_vdbg(port->dev, "%s\n", __func__);
  return 0;
}

/****************************************************************************/

static void release_port(struct uart_port *port)
{
	dev_vdbg(port->dev, "%s\n", __func__);
}

/****************************************************************************/

static int verify_port(struct uart_port *port, struct serial_struct *ser)
{
	dev_vdbg(port->dev, "%s\n", __func__);

  if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_STELLARIS))
    return -EINVAL;
  return 0;
}

/****************************************************************************/

/*
 *  Define the basic serial functions we support.
 */
static const struct uart_ops ops = {
  .tx_empty = tx_empty,
  .get_mctrl  = get_mctrl,
  .set_mctrl  = set_mctrl,
  .start_tx = start_tx,
  .stop_tx  = stop_tx,
  .stop_rx  = stop_rx,
  .enable_ms  = enable_ms,
  .break_ctl  = break_ctl,
  .startup  = startup,
  .shutdown = shutdown,
  .set_termios  = set_termios,
  .type   = get_type,
  .request_port = request_port,
  .release_port = release_port,
  .config_port  = config_port,
  .verify_port  = verify_port,
};

/****************************************************************************/

static struct stellaris_serial_port ports[STLR_NUARTS];

/****************************************************************************/
#if defined(CONFIG_SERIAL_STELLARIS_CONSOLE)
/****************************************************************************/

extern void uart_putc(uint32_t uart_base, const char ch);

static void serial_console_putchar(struct uart_port *port, int ch)
{
  uart_putc((uint32_t)port->membase, ch);
}

/****************************************************************************/

static void console_write(struct console *co, const char *s, unsigned int count)
{
  struct stellaris_serial_port *uart = &ports[co->index];
  unsigned long flags;

	spin_lock_irqsave(&uart->port.lock, flags);
	uart_console_write(&uart->port, s, count, serial_console_putchar);
	spin_unlock_irqrestore(&uart->port.lock, flags);
}

/****************************************************************************/

static int __init console_setup(struct console *co, char *options)
{
  struct uart_port *port;
  int baud = CONFIG_SERIAL_STELLARIS_BAUDRATE;
  int bits = 8;
  int parity = 'n';
  int flow = 'n';

  if ((co->index < 0) || (co->index >= STLR_NUARTS))
    co->index = 0;

  port = &ports[co->index].port;
  if (port->membase == 0)
    return -ENODEV;

  if (options)
    uart_parse_options(options, &baud, &parity, &bits, &flow);

  return uart_set_options(port, co, baud, parity, bits, flow);
}

/****************************************************************************/

static struct uart_driver driver;

static struct console console = {
  .name   = "ttyS",
  .write    = console_write,
  .device   = uart_console_device,
  .setup    = console_setup,
  .flags    = CON_PRINTBUFFER,
  .index    = -1,
  .data   = &driver,
};

static int __init stellaris_console_init(void)
{
  register_console(&console);
  return 0;
}
console_initcall(stellaris_console_init);

#define STELLARIS_CONSOLE &console

/****************************************************************************/
#else
/****************************************************************************/

#define STELLARIS_CONSOLE NULL

/****************************************************************************/
#endif /* CONFIG_MCF_CONSOLE */
/****************************************************************************/

/*
 *  Define the STELLARIS UART driver structure.
 */
static struct uart_driver driver = {
  .owner        = THIS_MODULE,
  .driver_name  = "stellaris",
  .dev_name     = "ttyS",
  .major        = TTY_MAJOR,
  .minor        = 64,
  .nr           = STLR_NUARTS,
  .cons         = STELLARIS_CONSOLE,
};

/****************************************************************************/

static int __devinit probe(struct platform_device *pdev)
{
  struct stellaris_platform_uart *platp = pdev->dev.platform_data;
  struct uart_port *port;
  struct stellaris_serial_port *pp;
  int i;

  for (i = 0; ((i < STLR_NUARTS) && (platp[i].mapbase)); i++) {
    pp = ports + i;
    port = &pp->port;

    pp->uart_index = platp[i].uart_index;
#ifdef CONFIG_STELLARIS_DMA
		pp->dma_buffer_size = platp[i].dma_buffer_size;

		pp->dma_tx_channel = platp[i].dma_tx_channel;
		pp->dma_tx_buffer  = platp[i].dma_tx_buffer;
		pp->tx_busy        = 0;

		pp->dma_rx_channel = platp[i].dma_rx_channel;
		pp->rx_slot_a      = platp[i].dma_rx_buffer;
		pp->rx_slot_b      = (char*)pp->rx_slot_a + RX_SLOT_SIZE(pp);
		pp->rx_busy        = 0;

		pp->cur_bytes_received = 0;

		tasklet_init(&pp->rx_tasklet, do_rx_chars, (unsigned long)pp);
		tasklet_disable(&pp->rx_tasklet);
		init_timer(&pp->rx_timer);
		pp->rx_timer.function = rx_chars_timeout;
		pp->rx_timer.data = (unsigned long)pp;
#endif

		pp->dtr_gpio = platp[i].dtr_gpio;
		pp->rts_gpio = platp[i].rts_gpio;

		pp->flags = platp[i].flags;

    port->dev = &pdev->dev;
    port->line = i;
    port->type = PORT_STELLARIS;
    port->mapbase = platp[i].mapbase;
    port->membase = (platp[i].membase) ? platp[i].membase :
      (unsigned char __iomem *) platp[i].mapbase;
    port->iotype = SERIAL_IO_MEM;
    port->irq = platp[i].irq;
    port->uartclk = CONFIG_UART_CLOCK_TICK_RATE;
    port->ops = &ops;
    port->flags = ASYNC_BOOT_AUTOCONF;

    uart_add_one_port(&driver, port);
  }

  return 0;
}

/****************************************************************************/

static int __devexit remove(struct platform_device *pdev)
{
  struct uart_port *port;
  int i;

  for (i = 0; (i < STLR_NUARTS); i++) {
    port = &ports[i].port;
    if (port)
      uart_remove_one_port(&driver, port);
  }

  return 0;
}

/****************************************************************************/

static struct platform_driver platform_driver = {
  .probe    = probe,
  .remove   = __devexit_p(remove),
  .driver   = {
    .name = "uart-stellaris",
    .owner  = THIS_MODULE,
  },
};

/****************************************************************************/

static int __init stellaris_uart_init(void)
{
  int rc;

  printk(KERN_INFO "Texas Instruments Stellaris UART serial driver\n");

  rc = uart_register_driver(&driver);
  if (rc)
    return rc;

  rc = platform_driver_register(&platform_driver);
  if (rc)
    return rc;

  return 0;
}

/****************************************************************************/

static void __exit stellaris_uart_exit(void)
{
  platform_driver_unregister(&platform_driver);
  uart_unregister_driver(&driver);
}

/****************************************************************************/

module_init(stellaris_uart_init);
module_exit(stellaris_uart_exit);

MODULE_AUTHOR("Max Nekludov <macscomp@gmail.com>");
MODULE_DESCRIPTION("TI Stellaris UART driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:uart-stellaris");
