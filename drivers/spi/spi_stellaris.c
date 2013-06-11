/*
 * Copyright (C) 2012 Max Nekludov <macscomp@gmail.com>
 *
 * Based on:
 *
 * drivers/spi/spi_stellaris.c
 *
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */

//#define DEBUG
//#define VERBOSE_DEBUG

#ifndef CONFIG_STELLARIS_DMA
#  define POLLING_MODE
#endif

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>

#include <mach/spi.h>
#include <mach/hardware.h>
#include <mach/sram.h>
#include <mach/dma.h>

/***************************************************************************/

#define DRIVER_NAME             "stellaris-spi"
#define STELLARIS_TXFIFO_WORDS  8
#define CONFIG_SSI_TXLIMIT      (STELLARIS_TXFIFO_WORDS/2)

/***************************************************************************/
/*                         Data structures                                 */
/***************************************************************************/

struct spi_stellaris_config {
  uint32_t cpsdvsr;
  uint32_t cr0;
  uint32_t gpio_chipselect;

  uint32_t mode;
  uint32_t bits_per_word;
  uint32_t speed_hz;
};

/***************************************************************************/

struct spi_stellaris_data {
  struct spi_bitbang bitbang;

  struct completion xfer_done;
  void *base;
  int irq;
  uint32_t *chipselect;

  void  *txbuffer;              /* Source buffer */
  void  *rxbuffer;              /* Destination buffer */

  int      ntxwords;            /* Number of words left to transfer on the Tx FIFO */
  int      nrxwords;            /* Number of words received on the Rx FIFO */
  int      nwords;              /* Number of words to be exchanged */

#ifdef CONFIG_STELLARIS_DMA
	uint32_t dma_rx_flags;
	uint32_t dma_tx_flags;
	uint32_t dma_rx_channel;
	uint32_t dma_tx_channel;
	void *dma_rx_buffer;
	void *dma_tx_buffer;
	uint32_t xfer_size;
#else
  void  (*txword)(struct spi_stellaris_data *priv);
  void  (*rxword)(struct spi_stellaris_data *priv);
#endif
};

/***************************************************************************/

static inline uint32_t ssi_getreg(struct spi_stellaris_data *priv,
                                     unsigned int offset)
{
  return getreg32((uint32_t)priv->base + offset);
}

/***************************************************************************/

static inline void ssi_putreg(struct spi_stellaris_data *priv,
                                 unsigned int offset, uint32_t value)
{
  putreg32(value, (uint32_t)priv->base + offset);
}

/***************************************************************************/

static void ssi_disable(struct spi_stellaris_data *priv)
{
  uint32_t regval;

  regval = ssi_getreg(priv, STLR_SSI_CR1_OFFSET);
  regval &= ~SSI_CR1_SSE;
  ssi_putreg(priv, STLR_SSI_CR1_OFFSET, regval);
}

/***************************************************************************/

static void ssi_enable(struct spi_stellaris_data *priv)
{
  uint32_t regval = ssi_getreg(priv, STLR_SSI_CR1_OFFSET);
  regval  |= SSI_CR1_SSE;
  ssi_putreg(priv, STLR_SSI_CR1_OFFSET, regval);
}

/***************************************************************************/

static void enable_ssi_clock(void)
{
  ssi_clock_ctrl(0, SYS_ENABLE_CLOCK);
}

/***************************************************************************/

static void disable_ssi_clock(void)
{
  ssi_clock_ctrl(0, SYS_DISABLE_CLOCK);
}

/***************************************************************************/

static void ssi_setmode(struct spi_stellaris_config *config, uint32_t mode)
{
  if (mode & SPI_CPHA)
    config->cr0 |= SSI_CR0_SPH;
  else
    config->cr0 &= ~SSI_CR0_SPH;

  if (mode & SPI_CPOL)
    config->cr0 |= SSI_CR0_SPO;
  else
    config->cr0 &= ~SSI_CR0_SPO;
}

/***************************************************************************/

static void ssi_setbits(struct spi_stellaris_config *config, uint32_t bits_per_word)
{
  if (bits_per_word >=4 && bits_per_word <= 16)
  {
    config->cr0 &= ~SSI_CR0_DSS_MASK;
    config->cr0 |= ((bits_per_word - 1) << SSI_CR0_DSS_SHIFT);
  }
}

/***************************************************************************/

static void ssi_setfrequency(struct spi_stellaris_config *config, uint32_t frequency)
{
  uint32_t maxdvsr;
  uint32_t cpsdvsr;
  uint32_t scr;

  /* "The serial bit rate is derived by dividing down the input clock
   *  (FSysClk). The clock is first divided by an even prescale value
   *  CPSDVSR from 2 to 254, which is programmed in the SSI Clock Prescale
   *  (SSI_CPSR) register ... The clock is further divided by a value
   *  from 1 to 256, which is 1 + SCR, where SCR is the value programmed
   *  i n the SSI Control0 (SSICR0) register ...
   *
   * "The frequency of the output clock SSIClk is defined by:
   *
   *    "SSIClk = FSysClk / (CPSDVSR * (1 + SCR))
   *
   * "Note: Although the SSIClk transmit clock can theoretically be 25 MHz,
   *  the module may not be able to operate at that speed. For master mode,
   *  the system clock must be at least two times faster than the SSIClk.
   *  For slave mode, the system clock must be at least 12 times faster
   *  than the SSIClk."
   */

  if (frequency > CLOCK_TICK_RATE/2)
  {
    frequency = CLOCK_TICK_RATE/2;
  }

  /* Find optimal values for CPSDVSR and SCR.  This loop is inefficient,
   * but should not have to execute many times.
   *
   * EXAMPLE 1: CLOCK_TICK_RATE=50,000,0000 and frequency=400,000.
   *
   *   maxcvsr = 125
   *   1. cpsdvsr = 2, scr = 61 -> DONE
   *
   *   This would correspond to an actual frequency of:
   *   50,000,000 / (2 * (62)) = 403,226
   *
   * EXAMPLE 2: CLOCK_TICK_RATE=50,000,0000 and frequency=25,000,000.
   *
   *   maxcvsr = 2
   *   1. cpsdvsr = 2, scr = 0 -> DONE
   *
   *   This would correspond to an actual frequency of:
   *   50,000,000 / (2 * (1)) = 25,000,000
   */

  maxdvsr = CLOCK_TICK_RATE / frequency;
  cpsdvsr = 0;
  do
  {
    cpsdvsr += 2;
    scr = (maxdvsr / cpsdvsr) - 1;
  }
  while (scr > 255);

  config->cpsdvsr = cpsdvsr;
  config->cr0 = (config->cr0 & ~SSI_CR0_SCR_MASK) | (scr << SSI_CR0_SCR_SHIFT);
}

/***************************************************************************/

static int spi_config(struct spi_stellaris_config *config,
                       uint32_t mode, uint32_t bits_per_word, uint32_t speed_hz)
{
  config->cr0 = 0;
  config->cpsdvsr = 0;

  ssi_setmode(config, mode);
  ssi_setbits(config, bits_per_word);
  ssi_setfrequency(config, speed_hz);

  config->mode = mode;
  config->bits_per_word = bits_per_word;
  config->speed_hz = speed_hz;

  return 0;
}

/***************************************************************************/

#ifndef CONFIG_STELLARIS_DMA
static void __sram ssi_txnull(struct spi_stellaris_data *priv)
{
  dev_vdbg(&priv->bitbang.master->dev, "TX: ->0x0000\n");
  ssi_putreg(priv, STLR_SSI_DR_OFFSET, 0x0000);
}

static void __sram ssi_txuint16(struct spi_stellaris_data *priv)
{
  uint16_t *ptr    = (uint16_t*)priv->txbuffer;
  dev_vdbg(&priv->bitbang.master->dev, "TX: %p->%04x\n", ptr, *ptr);
  ssi_putreg(priv, STLR_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

static void __sram ssi_txuint8(struct spi_stellaris_data *priv)
{
  uint8_t *ptr   = (uint8_t*)priv->txbuffer;
  dev_vdbg(&priv->bitbang.master->dev, "TX: %p->%02x\n", ptr, *ptr);
  ssi_putreg(priv, STLR_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

static void __sram ssi_rxnull(struct spi_stellaris_data *priv)
{
  uint32_t regval  = ssi_getreg(priv, STLR_SSI_DR_OFFSET);
  dev_vdbg(&priv->bitbang.master->dev, "RX: discard %04x\n", regval);
}

static void __sram ssi_rxuint16(struct spi_stellaris_data *priv)
{
  uint16_t *ptr    = (uint16_t*)priv->rxbuffer;
  *ptr           = (uint16_t)ssi_getreg(priv, STLR_SSI_DR_OFFSET);
  dev_vdbg(&priv->bitbang.master->dev, "RX: %p<-%04x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}

static void __sram ssi_rxuint8(struct spi_stellaris_data *priv)
{
  uint8_t *ptr   = (uint8_t*)priv->rxbuffer;
  *ptr           = (uint8_t)ssi_getreg(priv, STLR_SSI_DR_OFFSET);
  dev_vdbg(&priv->bitbang.master->dev, "RX: %p<-%02x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}
#endif

/***************************************************************************/

static inline int ssi_txfifofull(struct spi_stellaris_data *priv)
{
  return (ssi_getreg(priv, STLR_SSI_SR_OFFSET) & SSI_SR_TNF) == 0;
}

/***************************************************************************/

static inline int ssi_rxfifoempty(struct spi_stellaris_data *priv)
{
  return (ssi_getreg(priv, STLR_SSI_SR_OFFSET) & SSI_SR_RNE) == 0;
}

/***************************************************************************/

#ifndef CONFIG_STELLARIS_DMA
static int __sram ssi_performtx(struct spi_stellaris_data *priv)
{
#ifndef POLLING_MODE
  uint32_t regval;
#endif
  int ntxd = 0;  /* Number of words written to Tx FIFO */

  /* Check if the Tx FIFO is full */

  if (!ssi_txfifofull(priv))
  {
    /* Not full.. Check if all of the Tx words have been sent */

    if (priv->ntxwords > 0)
    {
      /* No.. Transfer more words until either the Tx FIFO is full or
        * until all of the user provided data has been sent.
        */
#ifdef CONFIG_SSI_TXLIMIT
      /* Further limit the number of words that we put into the Tx
        * FIFO to CONFIG_SSI_TXLIMIT.  Otherwise, we could
        * overrun the Rx FIFO on a very fast SSI bus.
        */
      for (; ntxd < priv->ntxwords && ntxd < CONFIG_SSI_TXLIMIT && !ssi_txfifofull(priv); ntxd++)
#else
      for (; ntxd < priv->ntxwords && !ssi_txfifofull(priv); ntxd++)
#endif
      {
          priv->txword(priv);
      }

      /* Update the count of words to to transferred */

      priv->ntxwords -= ntxd;
    }

    /* Check again... Now have all of the Tx words been sent? */

#ifndef POLLING_MODE
    regval = ssi_getreg(priv, STLR_SSI_IM_OFFSET);
    if (priv->ntxwords > 0)
    {
      /* No.. Enable the Tx FIFO interrupt.  This interrupt occurs
        * when the Tx FIFO is 1/2 full or less.
        */
      regval |= SSI_IM_TX;
    }
    else
    {
      /* Yes.. Disable the Tx FIFO interrupt.  The final stages of
        * the transfer will be driven by Rx FIFO interrupts.
        */

      regval &= ~(SSI_IM_TX|SSI_RIS_ROR);
    }
    ssi_putreg(priv, STLR_SSI_IM_OFFSET, regval);
#endif
  }

  return ntxd;
}

/***************************************************************************/

static void __sram ssi_performrx(struct spi_stellaris_data *priv)
{
#ifndef POLLING_MODE
  uint32_t regval;
#endif

  /* Loop while data is available in the Rx FIFO */
  while (!ssi_rxfifoempty(priv))
  {
    /* Have all of the requested words been transferred from the Rx FIFO? */
    if (priv->nrxwords < priv->nwords)
    {
      /* No.. Read more data from Rx FIFO */
      priv->rxword(priv);
      priv->nrxwords++;
    }
  }
#ifndef POLLING_MODE
  /* The Rx FIFO is now empty.  While there is Tx data to be sent, the
   * transfer will be driven by Tx FIFO interrupts.  The final part
   * of the transfer is driven by Rx FIFO interrupts only.
   */
  regval = ssi_getreg(priv, STLR_SSI_IM_OFFSET);
  if (priv->ntxwords == 0 && priv->nrxwords < priv->nwords)
  {
      /* There are no more outgoing words to send, but there are
      * additional incoming words expected (I would think that this
      * a real corner case, be we will handle it with an extra
      * interrupt, probably an Rx timeout).
      */

    regval |= (SSI_IM_RX|SSI_IM_RT);
  }
  else
  {
    /* No.. there are either more Tx words to send or all Rx words
      * have received.  Disable Rx FIFO interrupts.
      */

    regval &= ~(SSI_IM_RX|SSI_IM_RT);
  }

  ssi_putreg(priv, STLR_SSI_IM_OFFSET, regval);
#endif
}
#endif

/***************************************************************************/

static int __sram spi_transfer_step(struct spi_stellaris_data *priv)
{
  dev_vdbg(&priv->bitbang.master->dev, "%s: ntxwords %d, nrxwords %d, nwords %d, SR %08x\n",
          __func__, priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, STLR_SSI_SR_OFFSET));

#ifdef CONFIG_STELLARIS_DMA
		/* Check if the transfer is complete */
  if (priv->ntxwords == 0)
  {
		dev_dbg(&priv->bitbang.master->dev, "Transfer complete\n");
    /* Wake up the waiting thread */
    complete(&priv->xfer_done);
    return 0;
  }

	priv->xfer_size = min(priv->ntxwords, DMA_MAX_TRANSFER_SIZE);

	if( priv->txbuffer )
	{
		dma_memcpy(priv->dma_tx_buffer, priv->txbuffer, priv->xfer_size);
		priv->txbuffer = (char*)priv->txbuffer + priv->xfer_size;
	}

	dev_vdbg(&priv->bitbang.master->dev, "%s: xfer_size %u\n", __func__, priv->xfer_size);

	dma_setup_xfer(priv->dma_rx_channel, priv->dma_rx_buffer,
								 priv->base + STLR_SSI_DR_OFFSET, priv->xfer_size, priv->dma_rx_flags);
	dma_setup_xfer(priv->dma_tx_channel, priv->base + STLR_SSI_DR_OFFSET,
								 priv->dma_tx_buffer, priv->xfer_size, priv->dma_tx_flags);
	dma_start_xfer(priv->dma_rx_channel);
	dma_start_xfer(priv->dma_tx_channel);
#ifdef CONFIG_ARCH_TM4C
	putreg32(SSI_DMACTL_RXDMAE | SSI_DMACTL_TXDMAE, priv->base + STLR_SSI_DMACTL_OFFSET);
#endif /* CONFIG_ARCH_TM4C */
#else /* CONFIG_STELLARIS_DMA */
  /* Handle outgoing Tx FIFO transfers */
  ssi_performtx(priv);

  /* Handle incoming Rx FIFO transfers */
  ssi_performrx(priv);

  dev_vdbg(&priv->bitbang.master->dev, "ntxwords: %d nrxwords: %d nwords: %d SR: %08x IM: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, STLR_SSI_SR_OFFSET),
          ssi_getreg(priv, STLR_SSI_IM_OFFSET));

	/* Check if the transfer is complete */
  if (priv->nrxwords >= priv->nwords)
  {
#ifndef POLLING_MODE
    /* Yes.. Disable all SSI interrupt sources */
    ssi_putreg(priv, STLR_SSI_IM_OFFSET, 0);

    /* Wake up the waiting thread */
    complete(&priv->xfer_done);
#endif /* POLLING_MODE */

    dev_dbg(&priv->bitbang.master->dev, "Transfer complete\n");

    return 0;
  }
#endif /* CONFIG_STELLARIS_DMA */

  return 1;
}

/***************************************************************************/

#ifndef POLLING_MODE
static irqreturn_t __sram spi_isr(int irq, void *dev_id)
{
	uint32_t regval;
  struct spi_stellaris_data *priv = dev_id;

  dev_vdbg(&priv->bitbang.master->dev, "%s\n", __func__);

  /* Clear pending interrupts */
  regval = ssi_getreg(priv, STLR_SSI_RIS_OFFSET);
  ssi_putreg(priv, STLR_SSI_ICR_OFFSET, regval);

#ifdef CONFIG_STELLARIS_DMA
#if defined(CONFIG_ARCH_LM3S)
	dma_ack_interrupt(priv->dma_tx_channel);

	if( dma_ack_interrupt(priv->dma_rx_channel) )
	{
		if( priv->rxbuffer )
		{
			dma_memcpy(priv->rxbuffer, priv->dma_rx_buffer, priv->xfer_size);
			priv->rxbuffer = (char*)priv->rxbuffer + priv->xfer_size;
		}
		priv->nrxwords += priv->xfer_size;
		priv->ntxwords -= priv->xfer_size;

		spi_transfer_step(priv);
	}
#elif defined(CONFIG_ARCH_TM4C)
	if( regval & SSI_RIS_DMATX)
	{
		uint32_t regval2 = ssi_getreg(priv, STLR_SSI_DMACTL_OFFSET);
		regval2 &= ~SSI_DMACTL_TXDMAE;
		ssi_putreg(priv, STLR_SSI_DMACTL_OFFSET, regval2);
		ssi_putreg(priv, STLR_SSI_ICR_OFFSET, SSI_ICR_DMATX);
	}
	
	if( regval & SSI_RIS_DMARX )
	{
		uint32_t regval2 = ssi_getreg(priv, STLR_SSI_DMACTL_OFFSET);
		regval2 &= ~SSI_DMACTL_RXDMAE;
		ssi_putreg(priv, STLR_SSI_DMACTL_OFFSET, regval2);
		ssi_putreg(priv, STLR_SSI_ICR_OFFSET, SSI_ICR_DMARX);

		if( priv->rxbuffer )
		{
			dma_memcpy(priv->rxbuffer, priv->dma_rx_buffer, priv->xfer_size);
			priv->rxbuffer = (char*)priv->rxbuffer + priv->xfer_size;
		}
		priv->nrxwords += priv->xfer_size;
		priv->ntxwords -= priv->xfer_size;

		spi_transfer_step(priv);
	}
#endif
#else
	spi_transfer_step(priv);
#endif

  return IRQ_HANDLED;
}
#endif

/***************************************************************************/

static int __sram spi_transfer(struct spi_device *spi,
        struct spi_transfer *transfer)
{
  struct spi_stellaris_data *priv_master = spi_master_get_devdata(spi->master);
  struct spi_stellaris_config *dev_priv = spi_get_ctldata(spi);

  dev_dbg(&spi->dev, "%s: tx_buf %p, rx_buf %p, len %u, cr0 0x%x, cpsdvsr 0x%x\n", __func__,
     transfer->tx_buf, transfer->rx_buf, transfer->len,
     dev_priv->cr0, dev_priv->cpsdvsr);

  /* Set up to perform the transfer */

  priv_master->txbuffer     = (uint8_t*)transfer->tx_buf; /* Source buffer */
  priv_master->rxbuffer     = (uint8_t*)transfer->rx_buf; /* Destination buffer */
  priv_master->ntxwords     = transfer->len;              /* Number of words left to send */
  priv_master->nrxwords     = 0;                          /* Number of words received */
  priv_master->nwords       = transfer->len;              /* Total number of exchanges */

#ifdef CONFIG_STELLARIS_DMA
	priv_master->dma_tx_flags = DMA_XFER_MEMORY_TO_DEVICE;
	priv_master->dma_rx_flags = DMA_XFER_DEVICE_TO_MEMORY;

	if (dev_priv->bits_per_word > 8)
	{
		priv_master->dma_tx_flags |= DMA_XFER_UNIT_WORD;
		priv_master->dma_rx_flags |= DMA_XFER_UNIT_WORD;
	}
  else
	{
		priv_master->dma_tx_flags |= DMA_XFER_UNIT_BYTE;
		priv_master->dma_rx_flags |= DMA_XFER_UNIT_BYTE;
	}
#else
  if (!priv_master->txbuffer)
    priv_master->txword = ssi_txnull;
  else
  {
    if (dev_priv->bits_per_word > 8)
      priv_master->txword = ssi_txuint16;
    else
      priv_master->txword = ssi_txuint8;
  }

  if (!priv_master->rxbuffer)
    priv_master->rxword = ssi_rxnull;
  else
  {
    if (dev_priv->bits_per_word > 8)
      priv_master->rxword = ssi_rxuint16;
    else
      priv_master->rxword = ssi_rxuint8;
  }
#endif

  /* Set CR1 */
  ssi_putreg(priv_master, STLR_SSI_CR1_OFFSET, 0);

  /* Set CPDVSR */
  ssi_putreg(priv_master, STLR_SSI_CPSR_OFFSET, dev_priv->cpsdvsr);

  /* Set CR0 */
  ssi_putreg(priv_master, STLR_SSI_CR0_OFFSET, dev_priv->cr0);

#ifndef POLLING_MODE
  init_completion(&priv_master->xfer_done);
#endif

  ssi_enable(priv_master);

#ifndef POLLING_MODE
  spi_transfer_step(priv_master);
#else
  while( spi_transfer_step(priv_master) ) {};
#endif

#ifndef POLLING_MODE
  wait_for_completion(&priv_master->xfer_done);
#endif

  ssi_disable(priv_master);

  return transfer->len;
}

/***************************************************************************/

static int spi_setupxfer(struct spi_device *spi,
         struct spi_transfer *t)
{
  struct spi_stellaris_config *priv_dev = spi_get_ctldata(spi);

  uint32_t mode = spi->mode;
  uint32_t bits_per_word = t ? t->bits_per_word : spi->bits_per_word;
  uint32_t speed_hz = t ? t->speed_hz : spi->max_speed_hz;

  if (!speed_hz)
    speed_hz = spi->max_speed_hz;
  if (!bits_per_word)
    bits_per_word = spi->bits_per_word;

  dev_dbg(&spi->dev, "%s speed_hz %i, mode %i\n", __func__, speed_hz, mode);

  spi_config(priv_dev, mode, bits_per_word, speed_hz);

  return 0;
}

/***************************************************************************/

static void spi_chipselect(struct spi_device *spi, int is_active)
{
  struct spi_stellaris_config *priv_dev = spi_get_ctldata(spi);

  int active = is_active != BITBANG_CS_INACTIVE;
  int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);
  int value = dev_is_lowactive ^ active;

  dev_dbg(&spi->dev, "%s: cs %i [0x%X], value %i\n", __func__,
          spi->chip_select, priv_dev->gpio_chipselect, value);

  gpiowrite(priv_dev->gpio_chipselect, value);
}

/***************************************************************************/

static int stellaris_spi_setup(struct spi_device *spi)
{
	struct spi_stellaris_config *priv_dev;
	struct spi_stellaris_data *priv_master;

  if( spi->chip_select >= spi->master->num_chipselect )
    return -EINVAL;

  priv_dev = kmalloc(sizeof(struct spi_stellaris_config), GFP_KERNEL);
  if( priv_dev == 0 )
    return -ENOMEM;

  priv_master = spi_master_get_devdata(spi->master);

  spi_set_ctldata(spi, priv_dev);

  priv_dev->gpio_chipselect = priv_master->chipselect[spi->chip_select];
  spi_config(priv_dev, spi->mode, spi->bits_per_word, spi->max_speed_hz);
  spi_chipselect(spi, BITBANG_CS_INACTIVE);

  dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
     spi->mode, spi->bits_per_word, spi->max_speed_hz);

  return 0;
}

/***************************************************************************/

static void spi_cleanup(struct spi_device *spi)
{
  kfree(spi_get_ctldata(spi));
}

/***************************************************************************/

static int __devinit spi_probe(struct platform_device *pdev)
{
  struct spi_stellaris_master *platform_info;
  struct spi_master *master;
  struct spi_stellaris_data *priv;
  struct resource *res;
  int ret;

  platform_info = dev_get_platdata(&pdev->dev);
  if (!platform_info) {
    dev_err(&pdev->dev, "can't get the platform data\n");
    return -EINVAL;
  }

  master = spi_alloc_master(&pdev->dev, sizeof(struct spi_stellaris_data));
  if (!master)
    return -ENOMEM;

  platform_set_drvdata(pdev, master);

  master->bus_num = pdev->id;
  master->num_chipselect = platform_info->num_chipselect;

  priv = spi_master_get_devdata(master);
  priv->bitbang.master = spi_master_get(master);
  priv->chipselect = platform_info->chipselect;

#ifdef CONFIG_STELLARIS_DMA
	priv->dma_tx_buffer = platform_info->dma_tx_buffer;
	priv->dma_rx_buffer = platform_info->dma_rx_buffer;
	priv->dma_tx_channel = platform_info->dma_tx_channel;
	priv->dma_rx_channel = platform_info->dma_rx_channel;

	dma_setup_channel(priv->dma_tx_channel, DMA_DEFAULT_CONFIG);
	dma_setup_channel(priv->dma_rx_channel, DMA_DEFAULT_CONFIG);
#endif

  priv->bitbang.chipselect = spi_chipselect;
  priv->bitbang.setup_transfer = spi_setupxfer;
  priv->bitbang.txrx_bufs = spi_transfer;
  priv->bitbang.master->setup = stellaris_spi_setup;
  priv->bitbang.master->cleanup = spi_cleanup;
  priv->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

  init_completion(&priv->xfer_done);

  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!res) {
    dev_err(&pdev->dev, "can't get platform resource\n");
    ret = -ENOMEM;
    goto out_master_put;
  }

  if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
    dev_err(&pdev->dev, "request_mem_region failed\n");
    ret = -EBUSY;
    goto out_master_put;
  }

  priv->base = ioremap(res->start, resource_size(res));
  if (!priv->base) {
    ret = -EINVAL;
    goto out_release_mem;
  }

  priv->irq = platform_get_irq(pdev, 0);
  if (priv->irq <= 0) {
    ret = -EINVAL;
    goto out_iounmap;
  }

#ifndef POLLING_MODE
  ret = request_irq(priv->irq, spi_isr, 0, DRIVER_NAME, priv);
  if (ret) {
    dev_err(&pdev->dev, "can't get irq%d: %d\n", priv->irq, ret);
    goto out_iounmap;
  }
#endif

  enable_ssi_clock();

#ifdef CONFIG_STELLARIS_DMA
#ifdef CONFIG_ARCH_TM4C
	ssi_putreg(priv, STLR_SSI_IM_OFFSET, SSI_IM_DMARX | SSI_IM_DMATX);
#else
	putreg32(priv, STLR_SSI_DMACTL_OFFSET, SSI_DMACTL_RXDMAE | SSI_DMACTL_TXDMAE);
#endif
#endif

  ret = spi_bitbang_start(&priv->bitbang);
  if (ret) {
    dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
    goto out_free_irq;
  }

  dev_info(&pdev->dev, "probed\n");

  return ret;

out_free_irq:
#ifndef POLLING_MODE
  free_irq(priv->irq, priv);
#endif
out_iounmap:
  iounmap(priv->base);
out_release_mem:
  release_mem_region(res->start, resource_size(res));
out_master_put:
  spi_master_put(master);
  kfree(master);
  platform_set_drvdata(pdev, NULL);
  return ret;
}

/***************************************************************************/

static int __devexit spi_remove(struct platform_device *pdev)
{
  struct spi_master *master = platform_get_drvdata(pdev);
  struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  struct spi_stellaris_data *priv = spi_master_get_devdata(master);

  spi_bitbang_stop(&priv->bitbang);

#ifndef POLLING_MODE
  free_irq(priv->irq, priv);
#endif
  iounmap(priv->base);

  spi_master_put(master);

  release_mem_region(res->start, resource_size(res));

  platform_set_drvdata(pdev, NULL);

	disable_ssi_clock();

  return 0;
}

/***************************************************************************/

static struct platform_driver driver = {
  .driver = {
       .name = DRIVER_NAME,
       .owner = THIS_MODULE,
       },
  .probe = spi_probe,
  .remove = __devexit_p(spi_remove),
};

/***************************************************************************/

static int __init spi_init(void)
{
  return platform_driver_register(&driver);
}

/***************************************************************************/

static void __exit spi_exit(void)
{
  platform_driver_unregister(&driver);
}

/***************************************************************************/

module_init(spi_init);
module_exit(spi_exit);

/***************************************************************************/

MODULE_DESCRIPTION("SPI driver for Texas Instruments Stellaris");
MODULE_AUTHOR("Max Nekludov <macscomp@gmail.com>");
MODULE_LICENSE("GPL");
