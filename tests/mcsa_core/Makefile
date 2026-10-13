# Host tests for motor/mcsa_core.c
#   make test   build and run the C tests
#   make lib    shared library for mcsa_client/test_mcsa_capture_client.py

TOP    := ../..
CC     ?= cc
CFLAGS := -std=gnu99 -O2 -g -Wall -Wextra -Werror -Wdouble-promotion -Wshadow -I$(TOP)/motor
LDLIBS := -lm
CORE   := $(TOP)/motor/mcsa_core.c $(TOP)/motor/mcsa_core.h

.PHONY: all test lib clean

all: test lib

test: test_mcsa_core
	./test_mcsa_core

test_mcsa_core: test_mcsa_core.c $(CORE)
	$(CC) $(CFLAGS) -o $@ test_mcsa_core.c $(TOP)/motor/mcsa_core.c $(LDLIBS)

lib: libmcsa_core.so

libmcsa_core.so: mcsa_testlib.c $(CORE)
	$(CC) $(CFLAGS) -fPIC -shared -o $@ mcsa_testlib.c $(TOP)/motor/mcsa_core.c $(LDLIBS)

clean:
	rm -f test_mcsa_core libmcsa_core.so
