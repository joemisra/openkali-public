# Project Name
TARGET = Kali

DEBUG = 0
RELEASE = 1
NDEBUG = 1

# Enable DSP features
DEFS += -DARM_MATH_CM7 \
        -DARM_MATH_DSP \
        -D__FPU_PRESENT=1 \
        -DNDEBUG 

# Link against CMSIS DSP library
#LIBS += -larm_cortexM7lfdp_math
#LDFLAGS += -specs=rdimon.specs -lc -lrdimon

# Sources
CPP_SOURCES = Kali.cpp KaliClock.cpp KaliOscillator.cpp KaliDsp.cpp KaliPlayheadEngine.cpp KaliOptions.cpp KaliState.cpp KaliVersion.cpp DelayPhasor.cpp stringtables.cpp PresetNameGenerator.cpp dpt/daisy_dpt.cpp dpt/dev/DAC7554.cpp KaliMIDI.cpp 

# Library Locations
LIBDAISY_DIR = lib/libDaisy/
DAISYSP_DIR = lib/DaisySP/
#OPT = -Os -Wdouble-promotion -DVERSION_BUILD_DATE=\""$(shell date)"\" -DVERSION=\""$(shell git describe --tag)"\"
# Enable LTO for smaller binaries, remove debug symbols, disable RTTI and exceptions
OPT = -Os -flto -DNDEBUG -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections -DVERSION_BUILD_DATE=\""$(shell date)"\" -DVERSION=\""$(shell git describe --tag)"\" -std=gnu++14

# Ensure the linker also applies LTO and garbage collection
LDFLAGS += -flto -Wl,--gc-sections

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# STM32 Cube Programmer path for Windows
STM32_PROGRAMMER_CLI ?= "/mnt/c/st/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI.exe"

# Program using STM32 Cube Programmer (more reliable on Windows)
program-stm32:
	$(STM32_PROGRAMMER_CLI) -c port=SWD -w ./build/$(TARGET).bin 0x08000000 -v -rst

zip: 
	zip ./archives/Kali_$(shell date +%Y%m%d_%H%M%S).bin.zip ./build/Kali.bin