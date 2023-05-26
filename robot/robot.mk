# Automatically generated by RTLinux Makefile
# then hacked by trb to clean up and add uei stuff. 

prefix := $(shell /usr/xenomai/bin/xeno-config --prefix)

CWD = $(shell pwd)
UEI_INC = /usr/powerdaq/include

SKIN = native

CC = $(shell /usr/xenomai/bin/xeno-config --cc)
CFLAGS  = $(shell /usr/xenomai/bin/xeno-config --skin=$(SKIN) --cflags) -I$(UEI_INC) -g -O0
#LDFLAGS = $(shell /usr/realtime/bin/xeno-config --skin=$(SKIN) --ldflags) -lnative -lpowerdaq32 -lrtdm
LDFLAGS = $(shell /usr/xenomai/bin/xeno-config --skin=$(SKIN) --ldflags) -lpowerdaq32 -lrtdm

ARCH = x86
# FVV changed ARCH from i386

CC = gcc