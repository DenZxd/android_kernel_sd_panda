#
#  Makefile for the Linux Controller Area Network drivers.
#

obj-$(CONFIG_CAN_VCAN)		+= vcan.o
obj-$(CONFIG_CAN_MCP251X)	+= mcp251x.o

ifdef CONFIG_CAN_DEBUG_DEVICES
EXTRA_CFLAGS += -DDEBUG
endif
