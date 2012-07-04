#ifndef MACH_LM3S_SRAM_H
#define MACH_LM3S_SRAM_H

#include <linux/compiler.h>

#define __sram          __section(.sram.text) __cold notrace

#endif