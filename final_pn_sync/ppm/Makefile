# compiler
CC = gcc
# linker
LD = gcc
# cleaner
RM = rm -f

# local directory for ".h" header files
INCDIR = ./include
# local directory for ".c" source files
CMNSRCDIR = ./common
RXSRCDIR = ./rx_source
TXSRCDIR = ./tx_source
# local directory for ".o" output files
OBJDIR = ./obj
# local directory for the executable output file
BINDIR = ./bin
# shared library path
LIBDIR = /root/RedPitaya/build
# shared library header path
LIBINC = $(LIBDIR)/include
# shared library ".so" (shared object) file path
LIBSO = $(LIBDIR)/lib

# compiler options (-I= for includes, -g = to collect debugging info)
CFLAGS  = -std=gnu99 -Wall -Werror -g -I$(LIBINC) -I$(INCDIR)
# linker flags, -rpath=provide runtime LD_LIBRARY_PATH, -L=links .so with local .o files
LDFLAGS =  -Wl,-rpath,$(LIBSO) -L$(LIBSO)
# standard library dependencies
LDLIBS = -lm -lpthread -lrp
# output executable file name
TARGET1 = $(BINDIR)/ofdm_tx
TARGET2 = $(BINDIR)/ofdm_rx

# get the ".o" names from the source files
TXSRC = $(wildcard $(TXSRCDIR)/*.c)
RXSRC = $(wildcard $(RXSRCDIR)/*.c)
CMNSRC = $(wildcard $(CMNSRCDIR)/*.c)
TXOBJ = $(TXSRC:$(TXSRCDIR)/%.c=$(OBJDIR)/%.o)
RXOBJ = $(RXSRC:$(RXSRCDIR)/%.c=$(OBJDIR)/%.o)
CMNOBJ = $(CMNSRC:$(CMNSRCDIR)/%.c=$(OBJDIR)/%.o)

##------------main makefile script--------------##
all: $(TARGET1) $(TARGET2)

# Linking
$(TARGET1): $(TXOBJ) $(CMNOBJ)
	$(LD) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(TARGET2): $(RXOBJ) $(CMNOBJ)
	$(LD) $(LDFLAGS) $^ $(LDLIBS) -o $@

# compiling
$(TXOBJ): $(OBJDIR)/%.o: $(TXSRCDIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

$(RXOBJ): $(OBJDIR)/%.o: $(RXSRCDIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

$(CMNOBJ): $(OBJDIR)/%.o: $(CMNSRCDIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

# cleaning
.PHONY: clean
clean:
	$(RM) $(TXOBJ) $(RXOBJ) $(CMNOBJ)
##-----------main makefile script--------------##
