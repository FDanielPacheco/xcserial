# --- Project Metadata ---
NAME     = xcserial
MAJOR    = 0
MINOR    = 4
RELEASE  = 0
VERSION  = $(MAJOR).$(MINOR).$(RELEASE)

# --- Toolchain Discovery ---
CC      = clang
AR      = llvm-ar
STRIP   = llvm-strip
PKG_CON = pkg-config

# --- Arch Lookup ---
TRIPLE_x86_64  = x86_64-linux-gnu
TRIPLE_arm     = arm-linux-gnueabihf
TRIPLE_aarch64 = aarch64-linux-gnu

# Default choice
ARCH ?= x86_64
TARGET_TRIPLE := $(TRIPLE_$(ARCH))
ifeq ($(TARGET_TRIPLE),)
$(error Invalid ARCH '$(ARCH)'. Supported: x86_64, arm, aarch64)
endif

# --- Paths ---
SRC_DIR   = src
INC_DIR   = include
BUILD_DIR = build/$(TARGET_TRIPLE)
OUT_DIR   = release/$(TARGET_TRIPLE)

# --- Compiler Flags (GDB Friendly) ---
# -Og: Optimize for debugging experience
# -ggdb3: Include maximum GDB-specific debug info
# -fPIC: Needed for shared libraries
CFLAGS  = -std=gnu99 -fPIC -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror -Wno-gnu-zero-variadic-macro-arguments
CFLAGS += -I$(INC_DIR) -Og -ggdb3 -D_GNU_SOURCE
CFLAGS += --target=$(TARGET_TRIPLE)
LDFLAGS = -shared -Wl,-soname,lib$(NAME).so.$(MAJOR) --target=$(TARGET_TRIPLE)
LIBS    = -ludev

# --- Target Files ---
OBJ       = $(BUILD_DIR)/$(NAME).o
LIB_A     = $(BUILD_DIR)/lib$(NAME).a
LIB_SO    = $(BUILD_DIR)/lib$(NAME).so.$(VERSION)
PC_FILE   = $(BUILD_DIR)/$(NAME).pc

# --- Default Target ---
all: $(LIB_A) $(LIB_SO) $(PC_FILE)

# --- Compilation ---
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo "  CC      $< [$(TARGET_TRIPLE)]"
	@$(CC) $(CFLAGS) -c $< -o $@

# --- Static Library ---
$(LIB_A): $(OBJ)
	@echo "  AR      $@"
	@$(AR) rcs $@ $^

# --- Shared Library ---
$(LIB_SO): $(OBJ)
	@echo "  LD      $@"
	@$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)
	@ln -sf lib$(NAME).so.$(VERSION) $(BUILD_DIR)/lib$(NAME).so

# --- Pkg-Config Generation ---
$(PC_FILE):
	@echo "  GEN     $@"
	@echo "prefix=/usr/local" > $@
	@echo "exec_prefix=\$${prefix}" >> $@
	@echo "libdir=\$${exec_prefix}/lib" >> $@
	@echo "includedir=\$${prefix}/include" >> $@
	@echo "" >> $@
	@echo "Name: $(NAME)" >> $@
	@echo "Description: Linux epoll serial port library" >> $@
	@echo "Version: $(VERSION)" >> $@
	@echo "Libs: -L\$${libdir} -l$(NAME) $(LIBS)" >> $@
	@echo "Cflags: -I\$${includedir}" >> $@

# --- Release/Cross-Compile Target ---
# Usage: make release ARCH=arm
release: clean all
	@echo "Creating release for $(TARGET_TRIPLE)..."
	@mkdir -p $(OUT_DIR)
	@cp $(INC_DIR)/*.h $(OUT_DIR)/
	@cp $(LIB_A) $(LIB_SO) $(PC_FILE) $(OUT_DIR)/
	@echo "Release ready in $(OUT_DIR)"

# --- Cleanup ---
clean:
	rm -rf build/$(TARGET_TRIPLE) release/$(TARGET_TRIPLE)

cleanall:
	rm -rf build/ release/

$(BUILD_DIR):
	mkdir -p $@

.PHONY: all clean release