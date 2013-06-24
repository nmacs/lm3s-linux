#
# Makefile for the linux kernel.
#

obj-y					:= gpio.o serial.o time.o traps.o syscontrol.o clock.o
obj-$(CONFIG_CPU_IDLE)	                += cpuidle.o
obj-$(CONFIG_MPU)			+= mpu.o
obj-$(CONFIG_STELLARIS_DMA)		+= dma.o dma_memcpy.o
obj-$(CONFIG_STELLARIS_ADC)		+= adc.o
