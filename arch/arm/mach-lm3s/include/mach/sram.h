#ifndef MACH_LM3S_SRAM_H
#define MACH_LM3S_SRAM_H

#include <linux/compiler.h>

#ifdef CONFIG_LM3S_COPY_TO_SRAM

#define __sram          __section(.sram.text) __cold notrace
#define __sramdata      __section(.sram.data)

#else

#define __sram
#define __sramdata

#endif

#endif