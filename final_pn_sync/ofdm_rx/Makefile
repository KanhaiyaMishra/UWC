# compiler
CC = gcc
# linker
LD = gcc
# cleaner
RM = rm -f

# local directory for ".h" header files
INCDIR = ./include
# local directory for ".c" source files
SRCDIR = ./source
# local directory for ".o" output files
OBJDIR = ./obj
# local directory for the executable output file
BINDIR = ../bin
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
TARGET = $(BINDIR)/ofdm_rx

# get the ".o" names from the source files
SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

##------------main makefile script--------------##
all: $(TARGET)

# Linking
$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) $^ $(LDLIBS) -o $@

# compiling
$(OBJECTS): $(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

# cleaning
.PHONY: clean
clean:
	$(RM) $(OBJECTS)
##-----------main makefile script--------------##
