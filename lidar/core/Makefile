#**************************************#
#process input parameters

ifndef OPT_LEVEL
OPT_LEVEL = 0
endif
#**************************************#
#Define the used toolchain
TCHAIN =

#make with V=1 will enable verbose messages
ifeq ($(V), 1)
Q =
else
Q = @
endif

#**************************************#
#compiler, assembler and linker to use
GCC = $(TCHAIN)gcc
GAS = $(TCHAIN)as
LD = $(TCHAIN)ld
SIZE = $(TCHAIN)size
OBJCOPY = $(TCHAIN)objcopy
OBJDUMP = $(TCHAIN)objdump
NM = $(TCHAIN)nm
WC = wc

#**************************************#
#some other tools that might be helpfull
MKDIR = mkdir
RM = rm -rf
COPY = cp

#**************************************#
#define directory structure
SRCDIR = src
OUTDIR = build
OBJDIR = $(OUTDIR)/obj
LISTDIR = $(OUTDIR)/list


#**************************************#
#define source files and linker script

#Add sources from spiffs
FLIGHT_SRC = vl53l8cx_api.c
FLIGHT_SRC+= vl53l8cx.c
FLIGHT_SRC+= vl53l8cx_platform.c
FLIGHT_SRC+= vl53l8cx_plugin_detection_thresholds.c
FLIGHT_SRC+= vl53l8cx_plugin_motion_indicator.c
FLIGHT_SRC+= vl53l8cx_plugin_xtalk.c


TARGET = liblidar.so

CSRC = interface_calibrate.c
CSRC+= $(addprefix $(SRCDIR)/, $(FLIGHT_SRC))

#**************************************#
#define additional include directories
ADDINCDIRS = . inc/ 
EXTRAINCDIR = $(addprefix $(SRCDIR)/, $(ADDINCDIRS))

#**************************************#
#define additional libraries
EXTRA_LIBS = wiringPi

#**************************************#
#declare all sourcefiles seperated from path
ASMSRCFILES = $(notdir $(ASMSRC))
CSRCFILES = $(notdir $(CSRC))
ALLSRCFILES = $(ASMSRCFILES) $(CSRCFILES)

ALL_SRC_FILES = $(ASMSRC) $(CSRC)
PATHS = $(addprefix $(SRCDIR)/, $(sort $(dir $(ALL_SRC_FILES))))
VPATH =$(PATHS)

#**************************************#
#declare all object files derived from source files
ASMOBJ = $(addprefix $(OBJDIR)/, $(ASMSRCFILES:.S=.o))
COBJ = $(addprefix $(OBJDIR)/, $(CSRCFILES:.c=.o))
LST = $(ASMSRCFILES:%.S=%.lst) $(CSRC:%.c=%.lst)

#**************************************#
#define some flags to auto generate dependencies
GENDEPFLAGS = -MD -MP -MF $(OUTDIR)/dep/$(@F).d
#**************************************#
#define all the flags
DEBUG = dwarf-2

CFLAGS =  -O$(OPT_LEVEL) -g$(DEBUG) -fPIC
CFLAGS += -ffunction-sections -fdata-sections --std=gnu99
CFLAGS += -Wall -Wextra -Wimplicit -Wpointer-arith -Wcast-align 
CFLAGS +=  $(GENDEPFLAGS)
CFLAGS += -Wredundant-decls -Wshadow -Wnested-externs 
CFLAGS += -Wa,-adhlns=$(LISTDIR)/$(notdir $(subst $(suffix $<),.lst,$<)) 
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIR))

CDEFS = -D

EXTRA_LIBDIRS =

LDFLAGS = -shared -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref
LDFLAGS += -lc -lgcc
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))

#**************************************#
#make-targets
all: makedir build

makedir:
	$(Q)$(MKDIR) -p $(OUTDIR)
	$(Q)$(MKDIR) -p $(OUTDIR)/dep
	$(Q)$(MKDIR) -p $(OBJDIR)
	$(Q)$(MKDIR) -p $(LISTDIR)
	
build: $(OUTDIR)/$(TARGET)

#rule for linking
$(OUTDIR)/$(TARGET): $(ASMOBJ) $(COBJ) $(LDSCRIPT) Makefile
	$(Q)echo
	@echo Linking $@...
	$(Q)$(GCC) $(ASMOBJ) $(COBJ) --output $@ $(LDFLAGS)
	@echo done!

#default rules for compiling assembler files
$(ASMOBJ): $(OBJDIR)/%.o:%.S Makefile
	$(Q)echo
	@echo Assembling $<
	$(Q)$(GAS) -c $(ASFLAGS) $< -o $@	
	
#default rules for compiling C files
$(COBJ) : $(OBJDIR)/%.o : %.c Makefile
	$(Q)echo
	@echo Compiling $<
	$(Q)$(GCC) -c $(CFLAGS) -s $< -o $@  
	
clean:
	@echo -n Cleaning up...
	$(Q)$(RM) $(OUTDIR)
	@echo done!

# Include the dependency files.
-include $(shell mkdir $(OUTDIR)/dep 2>/dev/null) $(wildcard $(OUTDIR)/dep/*)


# Listing of phony targets.
.PHONY : all makedir build codesize clean
