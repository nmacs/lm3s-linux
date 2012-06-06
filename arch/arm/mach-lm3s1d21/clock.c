#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/kernel.h>

#include <mach/hardware.h>
#include <mach/timex.h>

/***************************************************************************/

static void enable_timer(unsigned int num);
static void disable_timer(unsigned int num);
static int find_timer(struct clock_event_device* dev);
static void timer_set_mode(enum clock_event_mode mode,
         struct clock_event_device *clk);
static int timer_set_next_event(unsigned long evt,
         struct clock_event_device *unused);
static void __init clockevents_init(unsigned int irqn);
static void __init clocksource_init(void);
static irqreturn_t timer_interrupt(int irq, void *dev_id);
static cycle_t clock_get_cycles(struct clocksource *cs);

/***************************************************************************/

static struct clock_event_device sysclk_clockevent =   {
  .name   = "sysclk_int",
  .shift    = 32,
  .features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
  .set_mode = timer_set_mode,
  .set_next_event = timer_set_next_event,
  .rating   = 300,
  .cpumask  = cpu_all_mask,
};

/***************************************************************************/

static struct clocksource sysclk_clocksource = {
  .name = "sysclk_poll",
  .rating = 200,
  .read = clock_get_cycles,
  .mask = CLOCKSOURCE_MASK(32),
  .shift  = 20,
  .flags  = CLOCK_SOURCE_IS_CONTINUOUS,
};

/***************************************************************************/

static void enable_timer(unsigned int num)
{
  uint32_t regval;
  regval = lm3s_getreg32(LM3S_TIMER_GPTMCTL(num));
  regval |= TIMER_GPTMCTL_TAEN_MASK;
  lm3s_putreg32(regval, LM3S_TIMER_GPTMCTL(num));
}

/***************************************************************************/

static void disable_timer(unsigned int num)
{
  uint32_t regval;
  regval = lm3s_getreg32(LM3S_TIMER_GPTMCTL(num));
  regval &= ~TIMER_GPTMCTL_TAEN_MASK;
  lm3s_putreg32(regval, LM3S_TIMER_GPTMCTL(num));
}

/***************************************************************************/

static void timer_set_mode(enum clock_event_mode mode,
         struct clock_event_device *clk)
{
  if( clk != &sysclk_clockevent )
  {
    printk(KERN_ERR "%s: unknown clock device %s\n", __func__, clk->name);
    return;
  }

  disable_timer(0);

  switch(mode) {
  case CLOCK_EVT_MODE_PERIODIC:
    printk(KERN_DEBUG "%s\n", "\tCLOCK_EVT_MODE_PERIODIC");
    // Clear configuration register
    lm3s_putreg32(0, LM3S_TIMER_GPTMCFG(0));
    // Setup periodic timer with decrimenting counter
    lm3s_putreg32(TIMER_GPTMTAMR_TAMR_PERIODIC, LM3S_TIMER_GPTMTAMR(0));
    // Set CONFIG_HZ interval
    lm3s_putreg32(CLOCK_TICK_RATE / CONFIG_HZ, LM3S_TIMER_GPTMTAILR(0));
    // Enable timer interrupt
    lm3s_putreg32(TIMER_GPTMIMR_TATOIM_MASK, LM3S_TIMER_GPTMIMR(0));
    // Enable timer
    enable_timer(0);
    break;
  case CLOCK_EVT_MODE_ONESHOT:
    printk(KERN_DEBUG "%s\n", "\tCLOCK_EVT_MODE_ONESHOT");
    lm3s_putreg32(0, LM3S_TIMER_GPTMCFG(0));
    // Setup one shot timer with decrimenting counter
    lm3s_putreg32(TIMER_GPTMTAMR_TAMR_ONESHOT, LM3S_TIMER_GPTMTAMR(0));

    break;
  case CLOCK_EVT_MODE_UNUSED:
    printk(KERN_DEBUG "%s\n", "\tCLOCK_EVT_MODE_UNUSED");
    break;
  case CLOCK_EVT_MODE_SHUTDOWN:
    printk(KERN_DEBUG "%s\n", "\tCLOCK_EVT_MODE_SHUTDOWN");
    break;
  }
}

/***************************************************************************/

static int timer_set_next_event(unsigned long evt,
        struct clock_event_device *clk)
{
  if( clk != &sysclk_clockevent )
  {
    printk(KERN_ERR "%s: unknown clock device %s\n", __func__, clk->name);
    return;
  }

  disable_timer(0);
  lm3s_putreg32(evt, LM3S_TIMER_GPTMTAILR(0));
  enable_timer(0);

  return 0;
}

/***************************************************************************/

static void __init clockevents_init(unsigned int irqn)
{
  sysclk_clockevent.irq = irqn;
  sysclk_clockevent.mult =
    div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC, sysclk_clockevent.shift);
  sysclk_clockevent.max_delta_ns =
    clockevent_delta2ns(0xffffffff, &sysclk_clockevent);
  sysclk_clockevent.min_delta_ns =
    clockevent_delta2ns(0xf, &sysclk_clockevent);

  clockevents_register_device(&sysclk_clockevent);
}

/***************************************************************************/

static cycle_t clock_get_cycles(struct clocksource *cs)
{
  return lm3s_getreg32(LM3S_TIMER_GPTMTAR(1));
}

/***************************************************************************/

static void __init clocksource_init()
{
  disable_timer(1);

  lm3s_putreg32(0, LM3S_TIMER_GPTMCFG(1));
  // Setup periodic timer with incrementing counter
  lm3s_putreg32(TIMER_GPTMTAMR_TAMR_PERIODIC | TIMER_GPTMTAMR_TACDIR_UP, LM3S_TIMER_GPTMTAMR(1));
  lm3s_putreg32(0xFFFFFFFF, LM3S_TIMER_GPTMTAILR(1));
  // Enable timer
  enable_timer(1);

  sysclk_clocksource.mult =
    clocksource_khz2mult(CLOCK_TICK_RATE / 1000, sysclk_clocksource.shift);

  clocksource_register(&sysclk_clocksource);
}

/***************************************************************************/

/*
 * IRQ handler for the timer
 */
static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
  struct clock_event_device *evt = (struct clock_event_device *)dev_id;

  if( evt == &sysclk_clockevent )
  {
    /* clear the interrupt */
    lm3s_putreg32(TIMER_GPTMICR_TATOCINT_MASK, LM3S_TIMER_GPTMICR(0));
    evt->event_handler(evt);
  }
  else
    printk(KERN_ERR "%s: unknown clock device\n", __func__);

  return IRQ_HANDLED;
}

/***************************************************************************/

static struct irqaction timer_irqaction = {
  .name     = "LM3S1D21 Timer Tick",
  .flags    = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
  .handler  = timer_interrupt,
  .dev_id   = &sysclk_clockevent,
};

/***************************************************************************/

void __init lm3s1d21_timer_init()
{
  uint32_t regval;

  regval = lm3s_getreg32(LM3S_SYSCON_RCGC1);
  regval |= SYSCON_RCGC1_TIMER0; // Enable Timer0
  regval |= SYSCON_RCGC1_TIMER1; // Enable Timer1
  lm3s_putreg32(regval, LM3S_SYSCON_RCGC1);

  setup_irq(LM3S1D21_TIMER0_IRQ, &timer_irqaction);
  clocksource_init();
  clockevents_init(LM3S1D21_TIMER0_IRQ);
}