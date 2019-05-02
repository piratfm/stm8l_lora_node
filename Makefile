#######
# makefile for STM8*_StdPeriph_Lib and SDCC compiler
#
# note: paths in this Makefile assume unmodified SPL folder structure
#
# usage:
#   1. if SDCC not in PATH set path -> CC_ROOT
#   2. set correct STM8 device -> DEVICE
#   3. set project paths -> PRJ_ROOT, PRJ_SRC_DIR, PRJ_INC_DIR
#   4. set SPL paths -> SPL_ROOT
#   5. add required SPL modules -> SPL_SOURCE
#   6. add required STM8_EVAL modules -> EVAL_SOURCE, EVAL_COMM_SOURCE, EVAL_STM8L1526_SOURCE, EVAL_STM8L1528_SOURCE
#
#######

# STM8 device (for supported devices see stm8l15x.h)
DEVICE=STM8L15X_MD

# set compiler path & parameters 
CC_ROOT =
#CC      = sdcc
#CFLAGS  = -mstm8 -lstm8 --opt-code-size
CFLAGS  = +debug -pxp -no -l +mods0 -pp -i/media/archive/STM8/CXSTM8/Hstm8 -clDebug -coDebug
CC      = /media/archive/STM8/CXSTM8/cxstm8.exe

# set output folder and target name
OUTPUT_DIR = ./$(DEVICE)
TARGET     = $(OUTPUT_DIR)/$(DEVICE).hex

# set project folder and files (all *.c)
PRJ_ROOT    = .
PRJ_SRC_DIR = $(PRJ_ROOT)
PRJ_INC_DIR = $(PRJ_ROOT)
PRJ_SOURCE  = $(notdir $(wildcard $(PRJ_SRC_DIR)/*.c))
PRJ_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(PRJ_SOURCE:.c=.rel))

# set SPL paths
SPL_ROOT    = std_lib
SPL_SRC_DIR = $(SPL_ROOT)/src
SPL_INC_DIR = $(SPL_ROOT)/inc
SPL_SOURCE  = stm8l15x_usart.c \
              stm8l15x_gpio.c \
              stm8l15x_spi.c \
              stm8l15x_syscfg.c \
              stm8l15x_tim4.c \
              stm8l15x_clk.c
SPL_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(SPL_SOURCE:.c=.rel))

# set path to LMIC routines
LMIC_DIR     = lmic
SPL_SRC_DIR = $(LMIC_DIR)
LMIC_SOURCE  = other.c \
               AES-128_V10.c \
               lmic.c \
               oslmic.c \
               radio.c

LMIC_INC_DIR = $(LMIC_DIR)
LMIC_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(LMIC_SOURCE:.c=.rel))

## set path to STM8L152x_EVAL common routines
#EVAL_COMM_DIR    = $(EVAL_DIR)/Common
#EVAL_COMM_SOURCE  = 
#EVAL_COMM_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(EVAL_COMM_SOURCE:.c=.rel))

## set path to STM8L1526_EVAL board routines
#EVAL_STM8L1526_DIR     = $(EVAL_DIR)/STM8L1526_EVAL
#EVAL_STM8L1526_SOURCE  = 
#EVAL_STM8L1526_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(EVAL_STM8L1526_SOURCE:.c=.rel))

## set path to STM8L1528_EVAL board routines
#EVAL_STM8L1528_DIR     = $(EVAL_DIR)/STM8L1528_EVAL
#EVAL_STM8L1528_SOURCE  = 
#EVAL_STM8L1528_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(EVAL_STM8L1528_SOURCE:.c=.rel))


# collect all include folders
#INCLUDE = -I$(PRJ_SRC_DIR) -I$(SPL_INC_DIR) -I$(EVAL_DIR) -I$(EVAL_COMM_DIR) -I$(EVAL_STM8L1526_DIR) -I$(EVAL_STM8L1528_DIR)

#INCLUDE = -I$(PRJ_SRC_DIR) -I$(SPL_INC_DIR) -I$(LMIC_INC_DIR)
INCLUDE = -i$(PRJ_SRC_DIR) -i$(SPL_INC_DIR) -i$(LMIC_INC_DIR)

# collect all source directories
#VPATH=$(PRJ_SRC_DIR):$(SPL_SRC_DIR):$(EVAL_DIR):$(EVAL_COMM_DIR):$(EVAL_STM8L1526_DIR):$(EVAL_STM8L1528_DIR)
VPATH=$(PRJ_SRC_DIR):$(SPL_SRC_DIR):$(LMIC_SRC_DIR)

.PHONY: clean

all: $(OUTPUT_DIR) $(TARGET) size

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

$(OUTPUT_DIR)/%.rel: %.c
	$(CC) $(CFLAGS) $(INCLUDE) -D$(DEVICE) -c $?

$(OUTPUT_DIR)/%.rel: %.c
	#$(CC) $(CFLAGS) $(INCLUDE) -D$(DEVICE) -c $? -o $@
	$(CC) $(CFLAGS) $(INCLUDE)  $?

#$(TARGET): $(PRJ_OBJECTS) $(SPL_OBJECTS) $(EVAL_OBJECTS) $(EVAL_COMM_OBJECTS) $(EVAL_STM8L1526_OBJECTS) $(EVAL_STM8L1528_OBJECTS)
$(TARGET): $(PRJ_OBJECTS) $(SPL_OBJECTS) $(LMIC_OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $^

size: 
	size $(TARGET)

flash:
	/media/archive/STM8/stm8flash/stm8flash -c stlink -p "stm8l151?6" -w $(TARGET)

clean: 
	rm -fr $(OUTPUT_DIR)

