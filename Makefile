# This is a general use makefile for librobotcontrol projects written in C.
# Just change the target name to match your main source code filename.
TARGET = main

# compiler and linker binaries
CC       := g++
LINKER   := g++

# compiler and linker flags
WFLAGS   := -Wall -Wextra -Werror=float-equal -Wuninitialized -Wunused-variable -Wdouble-promotion
CFLAGS   := -g -c -Wall -O2
LDFLAGS  := -pthread -lm -lrt -l:librobotcontrol.so.1

OBJ_DIR  := build
INIT_DIR := init

SRC_DIR := src

# Lista de arquivos a serem excluídos da compilação
EXCLUDED_FILES := CoeffDataManager.cpp testeplus.cpp

SOURCES  := $(shell find $(SRC_DIR) -type f \( -name "*.cc" -o -name "*.cpp" \) $(foreach file,$(EXCLUDED_FILES),-not -name $(file)))
INCLUDES := $(shell find $(SRC_DIR)/inc -name "*.h" -o -name "*.hh")
OBJECTS  := $(SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
OBJECTS  := $(OBJECTS:$(SRC_DIR)/%.cc=$(OBJ_DIR)/%.o)

prefix     := /usr/local
RM         := rm -f
INSTALL    := install -m 4755
INSTALLDIR := install -d -m 755

SYMLINK     := ln -s -f
SYMLINKDIR  := /etc/robotcontrol
SYMLINKNAME := link_to_startup_program

# Create object directory if it doesn't exist
$(shell mkdir -p $(OBJ_DIR))

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) -o $@ $(OBJECTS) $(LDFLAGS)
	@echo "Made: $@"

# compiling commands
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(INCLUDES)
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) -c $< -o $@
	@echo "Compiled: $@"

all: $(TARGET)

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "$(TARGET) Make Debug Complete"
	@echo " "

install:
	@$(MAKE) --no-print-directory
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin
	@echo "$(TARGET) Install Complete"

clean:
	@$(RM) -r $(OBJ_DIR)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Clean Complete"

cleanall:
	@$(RM) -r $(OBJ_DIR)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Clean Complete"
	@sudo bash /OnBoardComp/shells/cleanlogs.sh

uninstall:
	@$(RM) $(DESTDIR)$(prefix)/bin/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"

runonboot:
	@$(MAKE) install --no-print-directory
	@$(SYMLINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(SYMLINKDIR)/$(SYMLINKNAME)
	@echo "$(TARGET) Set to Run on Boot"
	@$(SYMLINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(SYMLINKDIR)/$(SYMLINKNAME)
