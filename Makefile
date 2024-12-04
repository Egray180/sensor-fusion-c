# Compiler and flags
CC = gcc
CFLAGS = -O2 -Wall -I./Inc -I./CMSIS_5/CMSIS/DSP/Include -I./CMSIS_5/CMSIS/Core/Include

# Project source files
SRCS = Src/filter.c Src/test.c
OBJS = $(SRCS:.c=.o)

# CMSIS-DSP source files (all required matrix functions and potential support files)
CMSIS_SRCS = \
    CMSIS_5/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f64.c \
    CMSIS_5/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.c \
    CMSIS_5/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f64.c \
    CMSIS_5/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.c \
    CMSIS_5/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.c \
    CMSIS_5/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.c \
    CMSIS_5/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f64.c
CMSIS_OBJS = $(CMSIS_SRCS:.c=.o)

# Output executable
TARGET = test_program.exe

# Build rules
all: $(TARGET)

$(TARGET): $(OBJS) $(CMSIS_OBJS)
	$(CC) $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(CMSIS_OBJS) $(TARGET)