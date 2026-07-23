#****************************************************************************
# Makefile
# SnapperGPS
# November 2020
#****************************************************************************

# Set the toolpath parameter to the location of the Arm GNU Toolchain

TOOLCHAIN_PATH = --- INSERT PATH TO ARM GNU TOOLCHAIN HERE (e.g., /Library/Toolchains/) ---

TOOLCHAIN_VERSION = gcc-arm-none-eabi-10-2020-q4-major

# Determine the path to the toolchain binary

TOOLPATH = $(TOOLCHAIN_PATH)/$(TOOLCHAIN_VERSION)/bin/

# Determine the path to the shared code

SHARED = ../../../shared_library/

# These are the locations of the source and header files
# Note: unlike the standard "snapper" firmware, this variant does not use
# USB, so $(SHARED)/emusb is intentionally left out. Its sources reference
# usbconfig.h, which this firmware version does not provide.

INC = $(SHARED)/cmsis $(SHARED)/device/inc $(SHARED)/emlib/inc $(SHARED)/inc $(GENDIR)
SRC = $(SHARED)/device/src $(SHARED)/emlib/src $(SHARED)/src ../src

# Set the name of the output files

FILENAME = uart-snapper

# This is the location of the resulting object files

OBJPATH = ./objects/

# This is the location of build-generated headers (currently just buildtime.h)

GENDIR = ./generated/

# Set the target EFM32HG to that used in Snapper

TARGET = EFM32HG310F64

# The following code generates the list of objects and the search path of source and header files

VPATH = $(SRC)

IFLAGS = $(foreach d, $(INC), -I$d)

_CSRC = $(notdir $(foreach d, $(SRC), $(wildcard $d/*.c)))
_SSRC = $(notdir $(foreach d, $(SRC), $(wildcard $d/*.s)))

_OBJ = $(_CSRC:.c=.o) $(_SSRC:.s=.o)

OBJ = $(sort $(foreach d, $(_OBJ), $(OBJPATH)$d))

DEP = $(OBJ:.o=.d)

# These are the compilation settings

CC = $(TOOLPATH)arm-none-eabi-gcc

CSIZE = $(TOOLPATH)arm-none-eabi-size

COBJCOPY = $(TOOLPATH)arm-none-eabi-objcopy

COBJDUMP = $(TOOLPATH)arm-none-eabi-objdump

CFLAGS = -mcpu=cortex-m0plus -mthumb -g0 -Wall '-D$(TARGET)=1' '-D__STACK_SIZE=512' '-D__HEAP_SIZE=128'

DFLAGS = -MMD

# Finally the build rules

# Keep $(FILENAME).bin as the default goal (the target built when make is
# invoked with no arguments) regardless of the order the rules appear in
# below.

.PHONY: all
all: $(FILENAME).bin

# Regenerate a header with the current Unix time on every invocation of
# make (the .PHONY prerequisite means this target is always considered
# out of date), so that main.c always seeds its clock close to the
# actual flash time without needing to be edited by hand. main.o is
# given an explicit dependency on it below so that it always gets
# recompiled when the timestamp changes.

$(GENDIR)buildtime.h: FORCE
	@mkdir -p $(GENDIR)
	@echo '#define INITIAL_UNIX_TIME $(shell date +%s)' > $@

.PHONY: FORCE
FORCE:

$(OBJPATH)main.o: $(GENDIR)buildtime.h

$(OBJPATH)%.o: %.c
	@mkdir -p $(OBJPATH)
	@echo 'Building' $@
	@$(CC) $(CFLAGS) $(DFLAGS) -O2 -ffunction-sections -fdata-sections -std=c99 -c -o "$@" "$<" $(IFLAGS)

$(OBJPATH)%.o: %.s
	@mkdir -p $(OBJPATH)
	@echo 'Building' $@
	@$(CC) $(CFLAGS) $(DFLAGS) -x assembler-with-cpp -c -o "$@" "$<" $(IFLAGS)

$(FILENAME).bin: $(OBJ)
	@echo 'Building' $(FILENAME).axf
	@$(CC) $(CFLAGS) $(DFLAGS) -T "snapper.ld" -Xlinker --gc-sections -Xlinker -Map="$(FILENAME).map" --specs=nano.specs -o $(FILENAME).axf $(OBJ) -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@$(COBJDUMP) -d -t -S "$(FILENAME).axf" > "$(FILENAME).lst"
	@echo 'Building' $(FILENAME).hex
	@$(COBJCOPY) -O ihex "$(FILENAME).axf" "$(FILENAME).hex"
	@echo 'Building' $(FILENAME).bin
	@$(COBJCOPY) -O binary "$(FILENAME).axf" "$(FILENAME).bin"
	@$(CSIZE) -A "$(FILENAME).axf"

-include $(DEP)

.PHONY: clean
clean:
	rm -f $(OBJPATH)*.o
	rm -f $(OBJPATH)*.d
	rm -rf $(GENDIR)
	rm -f $(FILENAME).axf
	rm -f $(FILENAME).bin
	rm -f $(FILENAME).hex
	rm -f $(FILENAME).map
