#
#  Makefile for the Linux Controller Area Network drivers.
#

obj-$(CONFIG_CAN_VCAN)		+= vcan.o
obj-$(CONFIG_CAN_DEV)           += can-dev.o
can-dev-y                       := dev.o

obj-$(CONFIG_CAN_MCP251X)       += mcp251x.o
obj-$(CONFIG_CAN_FLEXCAN)	+= flexcan.o

ccflags-$(CONFIG_CAN_DEBUG_DEVICES) := -DDEBUG
