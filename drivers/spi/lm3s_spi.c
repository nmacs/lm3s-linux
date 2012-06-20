/*
 * Copyright (C) 2012 Max Nekludov <macscomp@gmail.com>
 *
 * Based on:
 *
 * drivers/spi/spi_lm3s.c
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
#define POLLING_MODE

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

#include <mach/lm3s_spi.h>
#include <mach/hardware.h>

/***************************************************************************/

#define DRIVER_NAME        "lm3s-spi"
#define LM3S_TXFIFO_WORDS  8
#define CONFIG_SSI_TXLIMIT (LM3S_TXFIFO_WORDS/2)

/***************************************************************************/
/*                         Data structures                                 */
/***************************************************************************/

struct spi_lm3s_config {
  unsigned int speed_hz;
  unsigned int bits_per_word;
  unsigned int mode;
  uint32_t chipselect;
};

/***************************************************************************/

struct spi_lm3s_data {
  struct spi_bitbang bitbang;

  struct completion xfer_done;
  void *base;
  int irq;
  uint32_t *chipselect;
  struct device *dev;

  uint32_t cpsdvsr;
  uint32_t cr0;

  void  *txbuffer;              /* Source buffer */
  void  *rxbuffer;              /* Destination buffer */

  int      ntxwords;            /* Number of words left to transfer on the Tx FIFO */
  int      nrxwords;            /* Number of words received on the Rx FIFO */
  int      nwords;              /* Number of words to be exchanged */
  uint8_t  nbits;               /* Current number of bits per word */

  void  (*txword)(struct spi_lm3s_data *priv);
  void  (*rxword)(struct spi_lm3s_data *priv);
};

/***************************************************************************/

static inline uint32_t ssi_getreg(struct spi_lm3s_data *priv,
                                     unsigned int offset)
{
  return lm3s_getreg32((uint32_t)priv->base + offset);
}

/***************************************************************************/

static inline void ssi_putreg(struct spi_lm3s_data *priv,
                                 unsigned int offset, uint32_t value)
{
  lm3s_putreg32(value, (uint32_t)priv->base + offset);
}

/***************************************************************************/

static void ssi_disable(struct spi_lm3s_data *priv)
{
  uint32_t regval;

  regval = ssi_getreg(priv, LM3S_SSI_CR1_OFFSET);
  regval &= ~SSI_CR1_SSE;
  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, regval);
}

/***************************************************************************/

static void ssi_enable(struct spi_lm3s_data *priv)
{
  uint32_t regval = ssi_getreg(priv, LM3S_SSI_CR1_OFFSET);
  regval  |= SSI_CR1_SSE;
  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, regval);
}

/***************************************************************************/

static void enable_ssi_clock()
{
  uint32_t regval;
  regval = lm3s_getreg32(LM3S_SYSCON_RCGC1);
  regval |= SYSCON_RCGC1_SSI0;
  lm3s_putreg32(regval, LM3S_SYSCON_RCGC1);
}

/***************************************************************************/

static void disable_ssi_clock()
{
  uint32_t regval;
  regval = lm3s_getreg32(LM3S_SYSCON_RCGC1);
  regval &= ~SYSCON_RCGC1_SSI0;
  lm3s_putreg32(regval, LM3S_SYSCON_RCGC1);
}

/***************************************************************************/

static void ssi_setmode(struct spi_lm3s_data *priv, struct spi_lm3s_config *config)
{
  if (config->mode & SPI_CPHA)
    priv->cr0 |= SSI_CR0_SPH;
  else
    priv->cr0 &= ~SSI_CR0_SPH;

  if (config->mode & SPI_CPOL)
    priv->cr0 |= SSI_CR0_SPO;
  else
    priv->cr0 &= ~SSI_CR0_SPO;
}

/***************************************************************************/

static void ssi_setbits(struct spi_lm3s_data *priv, struct spi_lm3s_config *config)
{
  if (config->bits_per_word >=4 && config->bits_per_word <= 16)
  {
    priv->cr0 &= ~SSI_CR0_DSS_MASK;
    priv->cr0 |= ((config->bits_per_word - 1) << SSI_CR0_DSS_SHIFT);
  }
}

/***************************************************************************/

static void ssi_setfrequency(struct spi_lm3s_data *priv, struct spi_lm3s_config *config)
{
  uint32_t maxdvsr;
  uint32_t cpsdvsr;
  uint32_t scr;
  uint32_t regval;
  uint32_t frequency = config->speed_hz;

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

  priv->cpsdvsr = cpsdvsr;

  regval = priv->cr0;
  regval &= ~SSI_CR0_SCR_MASK;
  regval |= (scr << SSI_CR0_SCR_SHIFT);
  priv->cr0 = regval;

  /* Calcluate the actual frequency */
  //priv->actual = CLOCK_TICK_RATE / (cpsdvsr * (scr + 1));
}

/***************************************************************************/

static void ssi_txnull(struct spi_lm3s_data *priv)
{
  dev_vdbg(priv->dev, "TX: ->0xffff\n");
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, 0xffff);
}

/***************************************************************************/

static void ssi_txuint16(struct spi_lm3s_data *priv)
{
  uint16_t *ptr    = (uint16_t*)priv->txbuffer;
  dev_vdbg(priv->dev, "TX: %p->%04x\n", ptr, *ptr);
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

/***************************************************************************/

static void ssi_txuint8(struct spi_lm3s_data *priv)
{
  uint8_t *ptr   = (uint8_t*)priv->txbuffer;
  dev_vdbg(priv->dev, "TX: %p->%02x\n", ptr, *ptr);
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

/***************************************************************************/

static void ssi_rxnull(struct spi_lm3s_data *priv)
{
  uint32_t regval  = ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  dev_vdbg(priv->dev, "RX: discard %04x\n", regval);
}

/***************************************************************************/

static void ssi_rxuint16(struct spi_lm3s_data *priv)
{
  uint16_t *ptr    = (uint16_t*)priv->rxbuffer;
  *ptr           = (uint16_t)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  dev_vdbg(priv->dev, "RX: %p<-%04x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}

/***************************************************************************/

static void ssi_rxuint8(struct spi_lm3s_data *priv)
{
  uint8_t *ptr   = (uint8_t*)priv->rxbuffer;
  *ptr           = (uint8_t)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  dev_vdbg(priv->dev, "RX: %p<-%02x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}

/***************************************************************************/

static int lm3s_config(struct spi_lm3s_data *priv,
    struct spi_lm3s_config *config)
{
  ssi_setmode(priv, config);
  ssi_setbits(priv, config);
  ssi_setfrequency(priv, config);

  if (config->bits_per_word > 8)
  {
    priv->txword = ssi_txuint16;
    priv->rxword = ssi_rxuint16;
  }
  else
  {
    priv->txword = ssi_txuint8;
    priv->rxword = ssi_rxuint8;
  }

  return 0;
}

/***************************************************************************/

static inline int ssi_txfifofull(struct spi_lm3s_data *priv)
{
  return (ssi_getreg(priv, LM3S_SSI_SR_OFFSET) & SSI_SR_TNF) == 0;
}

/***************************************************************************/

static inline int ssi_rxfifoempty(struct spi_lm3s_data *priv)
{
  return (ssi_getreg(priv, LM3S_SSI_SR_OFFSET) & SSI_SR_RNE) == 0;
}

/***************************************************************************/

static int ssi_performtx(struct spi_lm3s_data *priv)
{
  uint32_t regval;
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
    regval = ssi_getreg(priv, LM3S_SSI_IM_OFFSET);
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
    ssi_putreg(priv, LM3S_SSI_IM_OFFSET, regval);
#endif
  }

  return ntxd;
}

/***************************************************************************/

static inline void ssi_performrx(struct spi_lm3s_data *priv)
{
  uint32_t regval;

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
  regval = ssi_getreg(priv, LM3S_SSI_IM_OFFSET);
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

  ssi_putreg(priv, LM3S_SSI_IM_OFFSET, regval);
#endif
}

/***************************************************************************/

static int spi_lm3s_transfer_step(struct spi_lm3s_data *priv)
{
  int ntxd;

  dev_vdbg(priv->dev, "ntxwords: %d nrxwords: %d nwords: %d SR: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET));

  /* Handle outgoing Tx FIFO transfers */
  ntxd = ssi_performtx(priv);

  /* Handle incoming Rx FIFO transfers */
  ssi_performrx(priv);

  dev_vdbg(priv->dev, "ntxwords: %d nrxwords: %d nwords: %d SR: %08x IM: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET),
          ssi_getreg(priv, LM3S_SSI_IM_OFFSET));

  /* Check if the transfer is complete */
  if (priv->nrxwords >= priv->nwords)
  {
#ifndef POLLING_MODE
    /* Yes.. Disable all SSI interrupt sources */
    ssi_putreg(priv, LM3S_SSI_IM_OFFSET, 0);
#endif

    /* Wake up the waiting thread */
    //complete(&priv->xfer_done);

    dev_dbg(priv->dev, "Transfer complete\n");

    return 0;
  }

  return 1;
}

/***************************************************************************/

static void spi_lm3s_chipselect(struct spi_device *spi, int is_active)
{
  struct spi_lm3s_data *priv = spi_master_get_devdata(spi->master);
  uint32_t chipselect = priv->chipselect[spi->chip_select];
  int active = is_active != BITBANG_CS_INACTIVE;
  int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);
  int value = dev_is_lowactive ^ active;

  dev_dbg(priv->dev, "%s: cs %i, value %i\n", __func__, spi->chip_select, dev_is_lowactive ^ active);

  lm3s_gpiowrite(chipselect, dev_is_lowactive ^ active);
}

/***************************************************************************/

#ifndef POLLING_MODE
static irqreturn_t spi_lm3s_isr(int irq, void *dev_id)
{
  struct spi_lm3s_data *priv = dev_id;

  dev_vdbg(priv->dev, "%s\n", __func__);

  uint32_t regval;
  int ntxd;

  /* Clear pending interrupts */
  regval = ssi_getreg(priv, LM3S_SSI_RIS_OFFSET);
  ssi_putreg(priv, LM3S_SSI_ICR_OFFSET, regval);

  spi_lm3s_transfer_step(priv);

  return IRQ_HANDLED;
}
#endif

/***************************************************************************/

static int spi_lm3s_setupxfer(struct spi_device *spi,
         struct spi_transfer *t)
{
  struct spi_lm3s_data *priv = spi_master_get_devdata(spi->master);
  struct spi_lm3s_config config;

  config.bits_per_word = t ? t->bits_per_word : spi->bits_per_word;
  config.speed_hz  = t ? t->speed_hz : spi->max_speed_hz;
  config.mode = spi->mode;
  config.chipselect = priv->chipselect[spi->chip_select];

  if (!config.speed_hz)
    config.speed_hz = spi->max_speed_hz;
  if (!config.bits_per_word)
    config.bits_per_word = spi->bits_per_word;
  if (!config.speed_hz)
    config.speed_hz = spi->max_speed_hz;

  lm3s_config(priv, &config);

  return 0;
}

/***************************************************************************/

static int spi_lm3s_transfer(struct spi_device *spi,
        struct spi_transfer *transfer)
{
  struct spi_lm3s_data *priv = spi_master_get_devdata(spi->master);

  dev_dbg(priv->dev, "%s: tx_buf %x, rx_buf %x, len %u\n", __func__,
     transfer->tx_buf, transfer->rx_buf, transfer->len);

  /* Set up to perform the transfer */

  priv->txbuffer     = (uint8_t*)transfer->tx_buf; /* Source buffer */
  priv->rxbuffer     = (uint8_t*)transfer->rx_buf; /* Destination buffer */
  priv->ntxwords     = transfer->len;              /* Number of words left to send */
  priv->nrxwords     = 0;                          /* Number of words received */
  priv->nwords       = transfer->len;              /* Total number of exchanges */

  if (!priv->txbuffer)
    priv->txword = ssi_txnull;

  if (!priv->rxbuffer)
    priv->rxword = ssi_rxnull;

  /* Set CR1 */
  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, 0);

  /* Set CPDVSR */
  ssi_putreg(priv, LM3S_SSI_CPSR_OFFSET, priv->cpsdvsr);

  /* Set CR0 */
  ssi_putreg(priv, LM3S_SSI_CR0_OFFSET, priv->cr0);

#ifndef POLLING_MODE
  init_completion(&priv->xfer_done);
#endif

  ssi_enable(priv);

#ifndef POLLING_MODE
  spi_lm3s_transfer_step(priv);
#else
  while( spi_lm3s_transfer_step(priv) ) {};
#endif

#ifndef POLLING_MODE
  wait_for_completion(&priv->xfer_done);
#endif

  ssi_disable(priv);

  return transfer->len;
}

/***************************************************************************/

static int spi_lm3s_setup(struct spi_device *spi)
{
  struct spi_lm3s_data *priv = spi_master_get_devdata(spi->master);

  dev_dbg(priv->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
     spi->mode, spi->bits_per_word, spi->max_speed_hz);

  spi_lm3s_chipselect(spi, BITBANG_CS_INACTIVE);

  return 0;
}

/***************************************************************************/

static void spi_lm3s_cleanup(struct spi_device *spi)
{
}

/***************************************************************************/

static int __devinit spi_lm3s_probe(struct platform_device *pdev)
{
  struct spi_lm3s_master *lm3s_platform_info;
  struct spi_master *master;
  struct spi_lm3s_data *priv;
  struct resource *res;
  int ret;

  lm3s_platform_info = dev_get_platdata(&pdev->dev);
  if (!lm3s_platform_info) {
    dev_err(&pdev->dev, "can't get the platform data\n");
    return -EINVAL;
  }

  master = spi_alloc_master(&pdev->dev, sizeof(struct spi_lm3s_data));
  if (!master)
    return -ENOMEM;

  platform_set_drvdata(pdev, master);

  master->bus_num = pdev->id;
  master->num_chipselect = lm3s_platform_info->num_chipselect;

  priv = spi_master_get_devdata(master);
  priv->dev = &pdev->dev;
  priv->bitbang.master = spi_master_get(master);
  priv->chipselect = lm3s_platform_info->chipselect;

  priv->bitbang.chipselect = spi_lm3s_chipselect;
  priv->bitbang.setup_transfer = spi_lm3s_setupxfer;
  priv->bitbang.txrx_bufs = spi_lm3s_transfer;
  priv->bitbang.master->setup = spi_lm3s_setup;
  priv->bitbang.master->cleanup = spi_lm3s_cleanup;
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
  ret = request_irq(priv->irq, spi_lm3s_isr, 0, DRIVER_NAME, priv);
  if (ret) {
    dev_err(&pdev->dev, "can't get irq%d: %d\n", priv->irq, ret);
    goto out_iounmap;
  }
#endif

  enable_ssi_clock();

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

static int __devexit spi_lm3s_remove(struct platform_device *pdev)
{
  struct spi_master *master = platform_get_drvdata(pdev);
  struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  struct spi_lm3s_data *priv = spi_master_get_devdata(master);

  spi_bitbang_stop(&priv->bitbang);

  free_irq(priv->irq, priv);
  iounmap(priv->base);

  spi_master_put(master);

  release_mem_region(res->start, resource_size(res));

  platform_set_drvdata(pdev, NULL);

  return 0;
}

/***************************************************************************/

static struct platform_driver spi_lm3s_driver = {
  .driver = {
       .name = DRIVER_NAME,
       .owner = THIS_MODULE,
       },
  .probe = spi_lm3s_probe,
  .remove = __devexit_p(spi_lm3s_remove),
};

/***************************************************************************/

static int __init spi_lm3s_init(void)
{
  return platform_driver_register(&spi_lm3s_driver);
}

/***************************************************************************/

static void __exit spi_lm3s_exit(void)
{
  platform_driver_unregister(&spi_lm3s_driver);
}

/***************************************************************************/

module_init(spi_lm3s_init);
module_exit(spi_lm3s_exit);

/***************************************************************************/

MODULE_DESCRIPTION("SPI driver for Texas Instruments LM3SXX");
MODULE_AUTHOR("Max Nekludov <macscomp@gmail.com>");
MODULE_LICENSE("GPL");
