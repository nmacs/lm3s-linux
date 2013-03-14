#
# Makefile for the linux kernel.
#

obj-y					:=

obj-$(CONFIG_ARCH_LM3S1D21)  		+= gpio.o serial.o clock.o traps.o power.o

obj-$(CONFIG_CPU_IDLE)	                += cpuidle.o
obj-$(CONFIG_MPU)			+= mpu.o
obj-$(CONFIG_LM3S_DMA)			+= dma.o dma_memcpy.o

obj-$(CONFIG_MACH_UWIC)			+= board-uwic.o
