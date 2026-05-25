# Host unit tests for STM32 firmware logic (runs on PC, no hardware).
#
# Usage:
#   make          # build and run
#   make test     # same
#   make clean

CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -g -O0
LDFLAGS ?= -lm

ifeq ($(shell $(CC) --version 2>/dev/null),)
$(error Host $(CC) not found. Install MinGW-w64 or MSYS2 gcc and add it to PATH.)
endif

ROOT        := ../..
SENSOR_DIR  := $(ROOT)/sensor_drivers
FIRMWARE_INC := $(ROOT)/RobotSTM32FirmWare/Core/Inc
FIRMWARE_SRC := $(ROOT)/RobotSTM32FirmWare/Core/Src

INCLUDES := -I$(SENSOR_DIR) -I$(FIRMWARE_INC)

SRCS := \
	test_main.c \
	test_icm45686_parse.c \
	test_imu_fusion.c \
	test_app_samples.c \
	$(SENSOR_DIR)/icm45686_parse.c \
	$(FIRMWARE_SRC)/imu_fusion.c \
	$(FIRMWARE_SRC)/app_samples.c

OBJS := $(SRCS:.c=.o)
TARGET := run_host_tests

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

# Avoid stale objects when sources move
$(SENSOR_DIR)/icm45686_parse.o: $(SENSOR_DIR)/icm45686_parse.c
$(FIRMWARE_SRC)/imu_fusion.o: $(FIRMWARE_SRC)/imu_fusion.c
$(FIRMWARE_SRC)/app_samples.o: $(FIRMWARE_SRC)/app_samples.c
