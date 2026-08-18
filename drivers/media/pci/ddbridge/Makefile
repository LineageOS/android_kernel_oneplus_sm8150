# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the ddbridge device driver
#

ddbridge-objs := ddbridge-main.o ddbridge-core.o ddbridge-hw.o \
		ddbridge-i2c.o ddbridge-maxs8.o

obj-$(CONFIG_DVB_DDBRIDGE) += ddbridge.o

ccflags-y += -I $(srctree)/drivers/media/dvb-core/
ccflags-y += -I $(srctree)/drivers/media/dvb-frontends/
ccflags-y += -I $(srctree)/drivers/media/tuners/

# For the staging CI driver cxd2099
ccflags-y += -I $(srctree)/drivers/staging/media/cxd2099/
