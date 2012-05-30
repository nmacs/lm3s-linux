#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/kernel.h>

static void timer_set_mode(enum clock_event_mode mode,
         struct clock_event_device *clk)
{
  //printk("%s\n", __func__);
  /*switch(mode) {
  case CLOCK_EVT_MODE_PERIODIC:
  case CLOCK_EVT_MODE_ONESHOT:
  case CLOCK_EVT_MODE_UNUSED:
  case CLOCK_EVT_MODE_SHUTDOWN:
  default:
  }*/
}

static int timer_set_next_event(unsigned long evt,
        struct clock_event_device *unused)
{
  //printk("%s\n", __func__);
  return 0;
}

static struct clock_event_device timer0_clockevent =   {
  .name   = "timer0",
  .shift    = 32,
  .features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
  .set_mode = timer_set_mode,
  .set_next_event = timer_set_next_event,
  .rating   = 300,
  .cpumask  = cpu_all_mask,
};

static void __init clockevents_init(unsigned int irqn)
{
  timer0_clockevent.irq = irqn;
  timer0_clockevent.mult =
    div_sc(1000000, NSEC_PER_SEC, timer0_clockevent.shift);
  timer0_clockevent.max_delta_ns =
    clockevent_delta2ns(0xffffffff, &timer0_clockevent);
  timer0_clockevent.min_delta_ns =
    clockevent_delta2ns(0xf, &timer0_clockevent);

  clockevents_register_device(&timer0_clockevent);
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
  //printk("%s\n", __func__);
  struct clock_event_device *evt = &timer0_clockevent;

  /* clear the interrupt */
  //writel(1, timer0_va_base + TIMER_INTCLR);

  evt->event_handler(evt);

  return IRQ_HANDLED;
}

static struct irqaction timer_irqaction = {
  .name   = "LM3S1D21 Timer Tick",
  .flags    = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
  .handler  = timer_interrupt,
};

void __init lm3s1d21_timer_init(unsigned int irqn)
{
  printk("1\n");
  setup_irq(irqn, &timer_irqaction);
  //clocksource_init();
  clockevents_init(irqn);
  printk("2\n");
}