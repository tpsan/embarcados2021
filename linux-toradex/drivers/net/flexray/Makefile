#
#  Makefile for the Linux FlexRay drivers.
#

obj-$(CONFIG_FLEXRAY_VFLEXRAY)		+= vflexray.o

obj-$(CONFIG_FLEXRAY_DEV)		+= flexray-dev.o
flexray-dev-y				:= dev.o

obj-$(CONFIG_FLEXRAY_FLEXCARD)		+= flexcard_fr.o

ccflags-$(CONFIG_FLEXRAY_DEBUG_DEVICES) := -DDEBUG
