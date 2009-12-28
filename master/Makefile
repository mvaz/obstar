######################################################################
# User Makefile sample for the Source Navigator / Introl Compiler Env.
#
# Copyright (C) 2000 K-TEAM S.A.
#
######################################################################

# Packages needed by the compiler (must be on the network)
# (DO NOT forget the / at the end).
exeprefix = m68k-none-elf
gccversion = 2.95.2
prefix = /home/miguel/estagio/khepera/kros-i686-pc-linux-gnu21/
sft_pack = /home/miguel/estagio/khepera/kteam/
hdw_pack = /home/miguel/estagio/khepera/khepack51/

# Hardware Debug Settings
port	= COM1
speed	= 19200

# Hierachical structure for separating source/object/binary
# (DO NOT forget the / at the end).
srcdir	= ./src/
asmdir = ./
includedir = ./
objdir = ./obj/
libdir = ./lib/
bindir = ./bin/

# Include the master makefile
# (PLEASE do not modify)
include $(sft_pack)etc/Makefile_PIC

# Libraries for linkage
LOADLIBES = $(SUPPLIB) -lgcc -lstdc++ -liostream -lio -lm -lktdebug -lc -lkbus -lglue -lc -lkernel -lc -lkernel -lglue -lc -lm -lgcc

# Assembler Options:
#   --register-prefix-optional: avoid adding a % in front of
#     register name (%a0-%a7,%d0-%d7)
#   -pic: generate position independent code
#   --bitwise-or: do not treat `|' as a comment character
ASFLAGS = --bitwise-or -pic --register-prefix-optional

# Linker Executable (gcc or g++ according to the language)
LINKER =  $(prefix)/bin/$(exeprefix)-g++

# Linker Options:
#   --embedded-relocs: Generate embedded relocs
#   --cref: Output cross reference table
LDFLAGS = --embedded-relocs

# C Compiler Options:
# other gcc options...
CFLAGS    = -Wall -O3

# This is a way to automatically find the main source code 
# and its dependencies, suppress if not needed.
OBJ = $(addsuffix .o,$(basename $(shell ls  *.c *.s *.S *.cc 2>null)))
INC = $(addsuffix .o,$(basename $(shell ls  *.h 2>null)))
EXE = $(addsuffix .s37,$(basename $(shell grep -wl main *.c *.s *.S *.cc 2>null)))

all:               $(bindir)main.s37

#$(bindir)main.s37: $(objdir)led.o $(srcdir)main.c $(srcdir)main.h
$(bindir)main.s37: $(objdir)obstar.o $(objdir)mot.o $(objdir)led.o $(objdir)sens.o $(srcdir)main.c $(srcdir)main.h

$(objdir)obstar.o: $(srcdir)obstar.c $(srcdir)obstar.h $(srcdir)obs.h $(srcdir)tar.h $(srcdir)robot.h $(srcdir)units.h $(srcdir)acquisition.h $(srcdir)config.h $(srcdir)main.h $(srcdir)history.h

$(objdir)sens.o:   $(srcdir)sens.c $(srcdir)sens.h $(srcdir)main.h

$(objdir)mot.o:    $(srcdir)mot.c  $(srcdir)mot.h  $(srcdir)main.h

$(objdir)led.o:    $(srcdir)led.c  $(srcdir)led.h  $(srcdir)main.h

clean:
	@echo Cleaning...
	@rm $(EXE)

