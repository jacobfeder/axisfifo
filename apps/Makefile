CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall -Wextra
LDFLAGS ?=
CFLAGS += 

BIN := axis-fifo-test axis-fifo-eth-loop

all: $(BIN)

%: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

install:
	install -v -m755 $(BIN) $(DESTDIR)/$(PREFIX)/bin

clean:
	$(RM) $(BIN)

.PHONY: install clean
