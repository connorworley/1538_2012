# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC603gnu
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/WindRiver/workspace
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/1538_2012



#Global Build Macros
PROJECT_TYPE = DKM
DEFINES = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC603
TOOL_FAMILY = gnu
TOOL = gnu
TOOL_PATH = 
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 

IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = -g
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif


#Project Targets
PROJECT_TARGETS = RAWCBot2012/$(MODE_DIR)/RAWCBot2012.out


#Rules

# RAWCBot2012
ifeq ($(DEBUG_MODE),1)
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
RAWCBot2012/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
RAWCBot2012/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
RAWCBot2012/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
RAWCBot2012/$(MODE_DIR)/% : IDE_DEFINES = 
RAWCBot2012/$(MODE_DIR)/% : PROJECT_TYPE = DKM
RAWCBot2012/$(MODE_DIR)/% : DEFINES = 
RAWCBot2012/$(MODE_DIR)/% : EXPAND_DBG = 0
RAWCBot2012/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
RAWCBot2012/$(MODE_DIR)/% : CPU = PPC603
RAWCBot2012/$(MODE_DIR)/% : TOOL_FAMILY = gnu
RAWCBot2012/$(MODE_DIR)/% : TOOL = gnu
RAWCBot2012/$(MODE_DIR)/% : TOOL_PATH = 
RAWCBot2012/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
RAWCBot2012/$(MODE_DIR)/% : LIBPATH = 
RAWCBot2012/$(MODE_DIR)/% : LIBS = 
RAWCBot2012/$(MODE_DIR)/% : OBJ_DIR := RAWCBot2012/$(MODE_DIR)

RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Autonomous/AutoModeSelector.o : $(PRJ_ROOT_DIR)/Autonomous/AutoModeSelector.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Controllers/AutoModeController.o : $(PRJ_ROOT_DIR)/Controllers/AutoModeController.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Controllers/OperatorController.o : $(PRJ_ROOT_DIR)/Controllers/OperatorController.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/DashboardDataFormat.o : $(PRJ_ROOT_DIR)/DashboardDataFormat.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCBase.o : $(PRJ_ROOT_DIR)/RAWCBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCConstants.o : $(PRJ_ROOT_DIR)/RAWCConstants.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCControlBoard.o : $(PRJ_ROOT_DIR)/RAWCControlBoard.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCLib.o : $(PRJ_ROOT_DIR)/RAWCLib.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCRobot.o : $(PRJ_ROOT_DIR)/RAWCRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Subsystems/Roller.o : $(PRJ_ROOT_DIR)/Subsystems/Roller.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Subsystems/Shooter.o : $(PRJ_ROOT_DIR)/Subsystems/Shooter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/accelFilter.o : $(PRJ_ROOT_DIR)/accelFilter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_RAWCBot2012 = RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Autonomous/AutoModeSelector.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Controllers/AutoModeController.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Controllers/OperatorController.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/DashboardDataFormat.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCBase.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCConstants.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCControlBoard.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCLib.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCRobot.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Subsystems/Roller.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Subsystems/Shooter.o \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/accelFilter.o

RAWCBot2012/$(MODE_DIR)/RAWCBot2012.out : $(OBJECTS_RAWCBot2012)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_RAWCBot2012) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_RAWCBot2012) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

RAWCBot2012/$(MODE_DIR)/RAWCBot2012_compile_file : $(FILE) ;

_clean :: RAWCBot2012/$(MODE_DIR)/RAWCBot2012_clean

RAWCBot2012/$(MODE_DIR)/RAWCBot2012_clean : 
	$(TRACE_FLAG)if [ -d "RAWCBot2012" ]; then cd "RAWCBot2012"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Autonomous/AutoModeSelector.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Controllers/AutoModeController.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Controllers/OperatorController.d \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/DashboardDataFormat.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCBase.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCConstants.d \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCControlBoard.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCLib.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/RAWCRobot.d \
	 RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Subsystems/Roller.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/Subsystems/Shooter.d RAWCBot2012/$(MODE_DIR)/Objects/1538_2012/accelFilter.d
-include $(DEP_FILES)



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

main_all : external_build $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
