# Version
MAJOR = 5
MINOR = 1
RELEASE = 0

# Path to LLVM
LLVM_BIN_PATH := /home/pacheco/Applications/LLVM/llvm-project/build/bin/
PATH := $(LLVM_BIN_PATH):$(PATH)

# LLVM Toolchain
LLVM_CC = clang
LLVM_OPT = opt
LLC = llc
LLVM_MC = llvm-mc
LLVM_LD = clang
LLVM_AR = llvm-ar

# Target Architecture (adjust as needed)
#TARGET_ARCH_LLC = arm
#TARGET_ARCH_CC = arm-linux-gnueabihf

#TARGET_ARCH_LLC = x86-64
#TARGET_ARCH_CC = x86_64-linux-gnu

#TARGET_ARCH_LLC = x86-64
#TARGET_ARCH_CC = x86_64-x86_64-unknown-freebsd14.2

TARGET_ARCH_LLC = aarch64
TARGET_ARCH_CC = aarch64-linux-gnu

#  Registered Targets:
#    aarch64     - AArch64 (little endian)
#    aarch64_32  - AArch64 (little endian ILP32)
#    aarch64_be  - AArch64 (big endian)
#    amdgcn      - AMD GCN GPUs
#    arm         - ARM
#    arm64       - ARM64 (little endian)
#    arm64_32    - ARM64 (little endian ILP32)
#    armeb       - ARM (big endian)
#    avr         - Atmel AVR Microcontroller
#    bpf         - BPF (host endian)
#    bpfeb       - BPF (big endian)
#    bpfel       - BPF (little endian)
#    hexagon     - Hexagon
#    lanai       - Lanai
#    loongarch32 - 32-bit LoongArch
#    loongarch64 - 64-bit LoongArch
#    mips        - MIPS (32-bit big endian)
#    mips64      - MIPS (64-bit big endian)
#    mips64el    - MIPS (64-bit little endian)
#    mipsel      - MIPS (32-bit little endian)
#    msp430      - MSP430 [experimental]
#    nvptx       - NVIDIA PTX 32-bit
#    nvptx64     - NVIDIA PTX 64-bit
#    ppc32       - PowerPC 32
#    ppc32le     - PowerPC 32 LE
#    ppc64       - PowerPC 64
#    ppc64le     - PowerPC 64 LE
#    r600        - AMD GPUs HD2XXX-HD6XXX
#    riscv32     - 32-bit RISC-V
#    riscv64     - 64-bit RISC-V
#    sparc       - Sparc
#    sparcel     - Sparc LE
#    sparcv9     - Sparc V9
#    systemz     - SystemZ
#    thumb       - Thumb
#    thumbeb     - Thumb (big endian)
#    ve          - VE
#    wasm32      - WebAssembly 32-bit
#    wasm64      - WebAssembly 64-bit
#    x86         - 32-bit X86: Pentium-Pro and above
#    x86-64      - 64-bit X86: EM64T and AMD64
#    xcore       - XCore

# Flags
CFLAGS = -I/usr/local/include -std=gnu99  # For c
#CFLAGS = -I/usr/local/include -std=gnu++23 -fPIC # For cpp
CFLAGS += -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude -Wno-gnu-zero-variadic-macro-arguments -O2
OPT_FLAGS = -O2
ASM_FLAGS =
LD_FLAGS = 
LD_LIB += -lc -lpthread -lrt -lm # -lcheck_pic -lsubunit

# Source and build directories
SRC_DIR = src
BUILD_DIR = build
LLVM_IR_DIR = $(BUILD_DIR)/llvm_ir
ASM_DIR = $(BUILD_DIR)/asm

# Documentation
DOCS_DIR = docs

# --- Target 1: xcserial ---
TARGET1_NAME = xcserial
TARGET1_SRC = $(SRC_DIR)/$(TARGET1_NAME).c
TARGET1_LL = $(LLVM_IR_DIR)/$(TARGET1_NAME).ll
TARGET1_S = $(ASM_DIR)/$(TARGET1_NAME).s
TARGET1_OBJ = $(BUILD_DIR)/$(TARGET1_NAME).o
TARGET1_A = $(BUILD_DIR)/lib$(TARGET1_NAME).a
TARGET1_SO = $(BUILD_DIR)/lib$(TARGET1_NAME).so

# --- Build Rules ---
# Create build directories
$(BUILD_DIR) $(LLVM_IR_DIR) $(ASM_DIR):
	@mkdir -p $@
	@echo "Created directory $@"

# Compile .c or .cpp to LLVM IR (.ll)
$(LLVM_IR_DIR)/%.ll: $(SRC_DIR)/%.c | $(LLVM_IR_DIR)
	@echo "Compiling $< to LLVM IR $@"
	$(LLVM_CC) $(CFLAGS) -S -emit-llvm --target=$(TARGET_ARCH_CC) $< -o $@

# Optimize LLVM IR (optional)
$(LLVM_IR_DIR)/%.opt.ll: $(LLVM_IR_DIR)/%.ll
	@echo "Optimizing LLVM IR $< to $@"
	$(LLVM_OPT) $(OPT_FLAGS) $< -o $@

# Compile LLVM IR (.ll) to Assembly (.s)
$(ASM_DIR)/%.s: $(LLVM_IR_DIR)/%.ll | $(ASM_DIR)
	@echo "Compiling LLVM IR $< to Assembly $@"
	$(LLC) -march=$(TARGET_ARCH_LLC) $< -o $@

# Assemble Assembly (.s) to Object (.o)
$(BUILD_DIR)/%.o: $(ASM_DIR)/%.s | $(BUILD_DIR)
	@echo "Assembling $< to Object $@"
	$(LLVM_LD) $(CFLAGS) --target=$(TARGET_ARCH_CC) -c $< -o $@

# Create the static library (.a)
$(TARGET1_A): $(TARGET1_OBJ)
	@echo "Creating static library $(TARGET1_NAME)..."
	$(LLVM_AR) rcs $@ $(TARGET1_OBJ)
	@echo "Built static library: $@"

# Create the dynamic library (.a)
$(TARGET1_SO): $(TARGET1_OBJ)
	@echo "Creating dynamic library $(TARGET1_NAME)..."
	$(LLVM_CC) -shared --target=$(TARGET_ARCH_CC) -o $@ $< $(LD_FLAGS) $(LD_LIB)
	@echo "Built dynamic library: $@"

# Generate the documentation
documentation:
	@echo "Generating documentation..."
	@cd docs && doxygen Doxyfile "PREDEFINED=PROJECT_VERSION=$(MAJOR).$(MINOR).$(RELEASE)"
	@cd ..
	@echo "Documentation generated in $(DOC_DIR)"

# Install the library TYPE is passed by argument
install: $(BUILD_DIR)/lib$(TARGET1_NAME).$(TYPE)
	@sudo cp include/$(TARGET1_NAME).h /usr/local/include/$(TARGET1_NAME).h
	@sudo cp $(BUILD_DIR)/lib$(TARGET1_NAME).$(TYPE) /usr/lib/$(TARGET_ARCH_CC)/lib$(TARGET1_NAME).$(TYPE)
	@sudo ldconfig
	@echo "Done installing..."

release: $(BUILD_DIR)/lib$(TARGET1_NAME).$(TYPE)
	@mkdir -p release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)
	@cp include/$(TARGET1_NAME).h release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)/$(TARGET1_NAME).h
	@cp $(BUILD_DIR)/lib$(TARGET1_NAME).$(TYPE) release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)/lib$(TARGET1_NAME).$(TYPE)
	@echo "Done creating a release for $(TARGET_ARCH_CC)..."
	@echo "Creating release installation bash..."
	@echo   "#!/bin/bash" \
			"\n" \
			"\ncp $(TARGET1_NAME).h /usr/local/include/$(TARGET1_NAME).h" \
			"\ncp lib$(TARGET1_NAME).$(TYPE) /usr/lib/$(TARGET_ARCH_CC)/lib$(TARGET1_NAME).$(TYPE)" \
			"\n" > ./release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)/install.sh
	@echo "Zipping the release lib$(TARGET1_NAME)-$(TARGET_ARCH_CC).zip..."
	@cd release && zip -r lib$(TARGET1_NAME)-$(TARGET_ARCH_CC).zip lib$(TARGET1_NAME)-$(TARGET_ARCH_CC) 


# Uninstall the library TYPE is passed by argument
uninstall:
	@echo "Unistalling library"
	@sudo rm /usr/lib/$(TARGET_ARCH_CC)/lib$(TARGET1_NAME).$(TYPE)
	@sudo rm /usr/local/include/$(TARGET1_NAME).h

# Default target to build all shared library
#all: $(TARGET1_A)
#	@echo "All targets built with LLVM workflow from $(LANG) files."

# Clean rule to remove build artifacts
clean:
	@echo "Cleaning build directory..."
	@rm -rf $(DOCS_DIR)/html
	@rm -rf $(DOCS_DIR)/man
	@rm -rf $(BUILD_DIR)
	@echo "Clean complete."

.PHONY: clean