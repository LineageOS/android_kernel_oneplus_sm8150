usbvision-objs  := usbvision-core.o usbvision-video.o usbvision-i2c.o usbvision-cards.o

obj-$(CONFIG_VIDEO_USBVISION) += usbvision.o

ccflags-y += -I $(srctree)/drivers/media/i2c
ccflags-y += -I $(srctree)/drivers/media/tuners
