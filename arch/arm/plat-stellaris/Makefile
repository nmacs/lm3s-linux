#
# Makefile for the linux kernel.
#

obj-y					:= gpio.o serial.o clock.o traps.o syscontrol.o
obj-$(CONFIG_CPU_IDLE)	                += cpuidle.o
obj-$(CONFIG_MPU)			+= mpu.o
obj-$(CONFIG_STELLARIS_DMA)		+= dma.o dma_memcpy.o
