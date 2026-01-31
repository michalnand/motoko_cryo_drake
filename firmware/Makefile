# path to libs_embedded
EMBEDDED_PATH	= $(HOME)/libs/stm32_libs/libs_embedded/

USER_PATH    	= usr/

COMMON_PATH   	= $(EMBEDDED_PATH)/common/
OS_PATH   		= $(EMBEDDED_PATH)/os/
MATH_PATH   	= $(EMBEDDED_PATH)/tmath/
STM_PATH   		= $(EMBEDDED_PATH)/stm32f7xx/

# paths to stm libs : CMSIS, HAL ... 
DEVICE_PATH		= $(STM_PATH)/device/
HAL_PATH		= $(STM_PATH)/STM32F7xx_HAL_Driver/
DRIVERS_PATH	= $(STM_PATH)/drivers/



PROJECT_DRIVERS_PATH = LibsDrivers/
PROJECT_CONTROL_PATH = LibsControl/
PROJECT_TESTS_PATH	 = tests/



GCCPREFIX  = arm-none-eabi-
OUTPUT     = bin/build


#TARGET     = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -DSTM32F722xx -DUSE_FULL_LL_DRIVER
TARGET     = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -ffast-math -DSTM32F722xx -DUSE_FULL_LL_DRIVER
#TARGET 	   = -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -fsingle-precision-constant -DSTM32F722xx -DUSE_FULL_LL_DRIVER



LDSCRIPT   = $(DEVICE_PATH)/linker_script.ld

# Includes
INC+= -I$(COMMON_PATH)
#INC+= -I$(OS_PATH)
INC+= -I$(MATH_PATH)
INC+= -I$(DEVICE_PATH)
INC+= -I$(DEVICE_PATH)/Include/
INC+= -I$(DEVICE_PATH)/CMSIS/
INC+= -I$(HAL_PATH)/Inc/
INC+= -I$(DRIVERS_PATH)


INC+= -I$(PROJECT_DRIVERS_PATH)
INC+= -I$(PROJECT_CONTROL_PATH)
INC+= -I$(PROJECT_TESTS_PATH)



INC+= -I$(USER_PATH)


# asm sources
ASM_SRCS = $(shell find $(DEVICE_PATH) -name '*.s')

# C sources
C_SRCS  = $(shell find $(DEVICE_PATH) -name '*.c')
C_SRCS+= $(shell find $(HAL_PATH)/Src/ -name 'stm32f7xx_ll*.c')

# cpp sources
CXX_SRCS = $(shell find $(USER_PATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(COMMON_PATH) -name '*.cpp')
#CXX_SRCS+= $(shell find $(OS_PATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(DRIVERS_PATH) -name '*.cpp')


CXX_SRCS+= $(shell find $(PROJECT_DRIVERS_PATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(PROJECT_CONTROL_PATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(PROJECT_TESTS_PATH) -name '*.cpp')




AS 		 = $(GCCPREFIX)gcc
CC       = $(GCCPREFIX)gcc
CXX      = $(GCCPREFIX)g++
LD       = $(GCCPREFIX)g++
OBJCOPY  = $(GCCPREFIX)objcopy
OBJDUMP  = $(GCCPREFIX)objdump
SIZE     = $(GCCPREFIX)size	

	
ASFLAGS	 = $(TARGET) -c	
CCFLAGS  = $(TARGET) $(INC) -c -O2 -Os -Wall -Wextra -Wpedantic -std=c99 -ffreestanding -nostdlib -fdata-sections -ffunction-sections
CXXFLAGS = $(TARGET) $(INC) -c -O2 -Os -Wall -Wextra -Wpedantic -std=c++14 -fno-rtti -fno-exceptions -fdata-sections -ffunction-sections
LDFLAGS = -T$(LDSCRIPT) $(TARGET) -nostdlib -Wl,--gc-sections -lgcc -ffreestanding

ASM_OBJS  = $(ASM_SRCS:.s=.o)	
C_OBJS    = $(C_SRCS:.c=.o)
CXX_OBJS  = $(CXX_SRCS:.cpp=.o)

all: $(OUTPUT)
	$(OBJCOPY) -O ihex $<.elf $(OUTPUT).hex
	$(OBJCOPY) -O binary $<.elf $(OUTPUT).bin
	$(OBJDUMP) --disassemble $<.elf > $(OUTPUT).asm
	$(SIZE) $(OUTPUT).elf

	

$(OUTPUT): $(C_OBJS) $(CXX_OBJS) $(ASM_OBJS)
	$(LD) -o $@.elf $(C_OBJS) $(CXX_OBJS) $(ASM_OBJS) $(LDFLAGS)


%.o: %.s
	$(AS) $(ASFLAGS) $< -o $@

%.o: %.c
	$(CC) $(CCFLAGS) $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@



clean:
	rm -f $(C_OBJS) $(CXX_OBJS) $(OUTPUT)*