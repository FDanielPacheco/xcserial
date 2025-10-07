# Version
MAJOR = 1
MINOR = 0
RELEASE = 0

# LLVM Toolchain
LLVM_CC = clang
LLVM_OPT = opt
LLC = llc
LLVM_MC = llvm-mc
LLVM_LD = clang
LLVM_AR = llvm-ar

# Flags
CFLAGS = -I/usr/local/include 
CFLAGS += -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude -Wno-gnu-zero-variadic-macro-arguments -O2
ASM_FLAGS = $(CFLAGS)
OPT_FLAGS = -O2 -strip-debug
LD_FLAGS = 
LD_LIB += -lc -lpthread -ludev

# Source and build directories
SRC_DIR = src
BUILD_DIR = build
LLVM_IR_DIR = $(BUILD_DIR)/llvm_ir
ASM_DIR = $(BUILD_DIR)/asm

# Documentation
DOCS_DIR = docs

# Test 
TEST_DIR = test
TESTS = sync async
TEST_BINS = $(addprefix $(BUILD_DIR)/, $(TESTS))

# --- Target 1: xcserial ---
TARGET1_NAME = xcserial
TARGET1_SRC = $(SRC_DIR)/$(TARGET1_NAME).c
TARGET1_LL = $(LLVM_IR_DIR)/$(TARGET1_NAME).ll
TARGET1_S = $(ASM_DIR)/$(TARGET1_NAME).s
TARGET1_OBJ = $(BUILD_DIR)/$(TARGET1_NAME).o
TARGET1_A = $(BUILD_DIR)/lib$(TARGET1_NAME).a
TARGET1_SO = $(BUILD_DIR)/lib$(TARGET1_NAME).so

# --- Documentation arguments ---
PROJECT_NAME = $(TARGET1_NAME)
PROJECT_NAME_BRIEF = "A C serial port library for Linux (lib$(PROJECT_NAME))"
PROJECT_BRIEF = "$(PROJECT_NAME) is a lightweight C wrapper library for interfacing with non-canonical serial port devices. Supports synchronous and asynchronous communication, as well as automatic hotplug detection and reconnection."

# --- Build Rules ---
single_so:
	@echo "Creating the libraries for the host platform..."
	$(MAKE) clean
	$(MAKE) release TARGET_ARCH_LLC="" TARGET_ARCH_CC="" TYPE=so CF=-fPIC LF="-relocation-model=pic" LDF="" TARGET_INC=""

single_a:
	@echo "Creating the libraries for the host platform..."
	$(MAKE) clean
	$(MAKE) release TARGET_ARCH_LLC="" TARGET_ARCH_CC="" TYPE=a CF=-fPIC LF="-relocation-model=pic" LDF="" TARGET_INC=""


all: 
	@echo "Creating the libraries for the following platforms:"
	@echo "aarch64, x86-64, arm"
	$(MAKE) clean
	$(MAKE) release TARGET_ARCH_LLC=arm TARGET_ARCH_CC=arm-linux-gnueabihf TYPE=so CF=-fPIC LF="-relocation-model=pic" LDF="" TARGET_INC="--target="
	$(MAKE) clean
	$(MAKE) release TARGET_ARCH_LLC=x86-64 TARGET_ARCH_CC=x86_64-linux-gnu TYPE=so CF=-fPIC LF="-relocation-model=pic" LDF="" TARGET_INC="--target="
	$(MAKE) clean
	$(MAKE) release TARGET_ARCH_LLC=aarch64 TARGET_ARCH_CC=aarch64-linux-gnu TYPE=so CF+=-fPIC LF="-relocation-model=pic" LDF="" TARGET_INC="--target="

# Create build directories
$(BUILD_DIR) $(LLVM_IR_DIR) $(ASM_DIR):
	@mkdir -p $@
	@echo "Created directory $@"

# Compile .c or .cpp to LLVM IR (.ll)
$(LLVM_IR_DIR)/%.ll: $(SRC_DIR)/%.c | $(LLVM_IR_DIR)
	@echo "Compiling $< to LLVM IR $@"
	$(LLVM_CC) -std=gnu99 $(CFLAGS) $(CF) -S -emit-llvm $(TARGET_INC)$(TARGET_ARCH_CC) $< -o $@

# Optimize LLVM IR (optional)
$(LLVM_IR_DIR)/%.opt.ll: $(LLVM_IR_DIR)/%.ll
	@echo "Optimizing LLVM IR $< to $@"
	$(LLVM_OPT) $(OPT_FLAGS) $< -o $@

# Compile LLVM IR (.ll) to Assembly (.s)
$(ASM_DIR)/%.s: $(LLVM_IR_DIR)/%.opt.ll | $(ASM_DIR)
	@echo "Compiling LLVM IR $< to Assembly $@"
	$(LLC) -march=$(TARGET_ARCH_LLC) $(LF) $< -o $@

# Assemble Assembly (.s) to Object (.o)
$(BUILD_DIR)/%.o: $(ASM_DIR)/%.s | $(BUILD_DIR)
	@echo "Assembling $< to Object $@"
	$(LLVM_LD) $(ASM_FLAGS) $(CF) $(TARGET_INC)$(TARGET_ARCH_CC) -c $< -o $@

# Create the static library (.a)
$(TARGET1_A): $(TARGET1_OBJ)
	@echo "Creating static library $(TARGET1_NAME)..."
	$(LLVM_AR) rcs $@ $(TARGET1_OBJ)
	@echo "Built static library: $@"

# Create the dynamic library (.a)
$(TARGET1_SO): $(TARGET1_OBJ)
	@echo "Creating dynamic library $(TARGET1_NAME)..."
	$(LLVM_CC) -shared $(TARGET_INC)$(TARGET_ARCH_CC) -o $@ $< $(LD_FLAGS) $(LD_LIB) $(LDF)
	@echo "Built dynamic library: $@"

# Generate the documentation
documentation:
	@echo "Generating documentation..."
	@cp docs/Doxyfile docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_NAME.*|PROJECT_NAME = $(PROJECT_NAME)|' docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_NAME_BRIEF.*|PROJECT_NAME_BRIEF = $(PROJECT_NAME_BRIEF)|' docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_BRIEF.*|PROJECT_BRIEF = $(PROJECT_BRIEF)|' docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_NUMBER.*|PROJECT_NUMBER = $(MAJOR).$(MINOR).$(RELEASE)|' docs/Doxyfile.tmp
	@doxygen docs/Doxyfile.tmp
	@echo "Documentation generated in $(DOC_DIR)"

release: $(BUILD_DIR)/lib$(TARGET1_NAME).$(TYPE)
	@mkdir -p release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)
	@cp include/$(TARGET1_NAME).h release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)/$(TARGET1_NAME).h
	@cp $(BUILD_DIR)/lib$(TARGET1_NAME).$(TYPE) release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)/lib$(TARGET1_NAME).$(TYPE)
	@echo "Done creating a release for $(TARGET_ARCH_CC)..."
	@echo "Creating release installation bash..."
	@echo   "#!/bin/bash" \
			"\n" \
			'\nif [ "$$EUID" -ne 0 ]; then' \
			"\n  echo \"Error: Please run this script with sudo.\"" \
			"\n  exit 1" \
			"\nfi" \
			"\necho \"0: Install\"" \
			"\necho \"1: Uninstall\"" \
			"\nread -p \">> \" choice" \
			'\nif [ "$$choice" -eq 0 ]; then' \
			"\n  echo \"Installing to /usr/local/...\"" \
			"\n  cp $(TARGET1_NAME).h /usr/local/include/$(TARGET1_NAME).h" \
			"\n  cp lib$(TARGET1_NAME).$(TYPE) /usr/lib/$(TARGET_ARCH_CC)/lib$(TARGET1_NAME).$(TYPE)" \
			"\n  ldconfig" \
			"\n  echo \"Installation complete.\"" \
			"\nelse" \
			"\n  echo \"Uninstalling from /usr/local/...\"" \
			"\n  rm /usr/lib/$(TARGET_ARCH_CC)/lib$(TARGET1_NAME).$(TYPE)" \
			"\n  rm /usr/local/include/$(TARGET1_NAME).h" \
			"\n  ldconfig" \
			"\n  echo \"Uninstallation complete.\"" \
			"\nfi" \
			"\n" > ./release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)/install.sh
	@chmod u+x ./release/lib$(TARGET1_NAME)-$(TARGET_ARCH_CC)/install.sh
	@echo "Zipping the release lib$(TARGET1_NAME)-$(TARGET_ARCH_CC).zip..."
	@cd release && zip -r lib$(TARGET1_NAME)-$(TARGET_ARCH_CC).zip lib$(TARGET1_NAME)-$(TARGET_ARCH_CC) 

clean:
	@echo "Cleaning build and documentation directories..."
	@rm -rf $(DOCS_DIR)/html
	@rm -rf $(DOCS_DIR)/man
	@rm -rf $(DOCS_DIR)/Doxyfile.tmp
	@rm -rf $(BUILD_DIR)
	@echo "Clean complete."

cleanrelease:
	@echo "Cleaning the release directory..."
	@rm -rf release
	
example: single_a
	$(MAKE) $(TESTS)

$(TESTS): %: single_a $(BUILD_DIR)/%

$(BUILD_DIR)/%: $(TEST_DIR)/%.c | $(BUILD_DIR)
	@echo "Compiling $< to test binary $@..."
	$(LLVM_CC) -std=gnu99 $(CFLAGS) $< -o $@ -L$(BUILD_DIR) -l$(TARGET1_NAME) $(LD_LIB)

# 
.PHONY: clean