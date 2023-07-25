TARGET_ELF = $(BLD_DIR)/stm32f429i-disc1-graphic.elf
TARGET_MAP = $(BLD_DIR)/stm32f429i-disc1-graphic.map

VPATH = $(shell find ./src/ -type d)
VPATH += $(shell find ./inc/ -type d)
BLD_DIR = ./build
OBJ_DIR = $(BLD_DIR)/obj
LNK_DIR = ./lnk

INCLUDE = $(VPATH:%=-I%)
SOURCES_C = $(shell find ./src/ -type f -name "*.c")
SOURCES_ASM = $(shell find ./src/ -type f -name "*.s")
OBJECTS_C = $(notdir $(patsubst %.c, %.o, $(SOURCES_C)))
OBJECTS_ASM = $(notdir $(patsubst %.s, %.o, $(SOURCES_ASM)))
OBJS = $(addprefix $(OBJ_DIR)/, $(OBJECTS_C))
OBJS += $(addprefix $(OBJ_DIR)/, $(OBJECTS_ASM))
#$(info OBJS="$(OBJS)")

MODE = all

CC = arm-none-eabi-gcc
MACH = cortex-m4
CFLAGS = -c -save-temps=obj -MD -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall $(INCLUDE) -O0
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nosys.specs --specs=nano.specs \
		  -T $(LNK_DIR)/STM32F429ZITX.ld -Wl,-Map=$(TARGET_MAP) -Wl,--print-memory-usage \
		  -Wl,--no-warn-rwx-segments


$(TARGET_ELF) : CFLAGS += -DDEBUG -DSTM32F429xx -g
$(TARGET_ELF) : $(OBJS)
	@mkdir -p $(BLD_DIR)
	$(CC) $(LDFLAGS) $(OBJS) -o $(TARGET_ELF)

$(OBJ_DIR)/%.o : %.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o : %.s
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $< -o $@

-include $(OBJ_DIR)/*.d

.PHONY : all
all: $(TARGET_ELF)

.PHONY : clean
clean:
	@rm -rf $(BLD_DIR)