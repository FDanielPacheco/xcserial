# --- Project Metadata ---
NAME     = xcserial
MAJOR    = 0
MINOR    = 4
RELEASE  = 0
VERSION  = $(MAJOR).$(MINOR).$(RELEASE)
BRIEF    = "Linux epoll serial port library"

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

# --- Debug ---
# Disable by: make DEBUG=DISABLE
DEBUG ?= 

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
CFLAGS += -D$(DEBUG)_LOGGING
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
	@ln -sf lib$(NAME).so.$(VERSION) $(BUILD_DIR)/lib$(NAME).so.$(MAJOR)
	
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

# --- Documentation with doxygen ---
docs:
	@echo "Generating documentation..."
	@cp docs/Doxyfile docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_NAME.*|PROJECT_NAME = $(NAME)|' docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_NAME_BRIEF.*|PROJECT_NAME_BRIEF = $(BRIEF)|' docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_BRIEF.*|PROJECT_BRIEF = $(BRIEF)|' docs/Doxyfile.tmp
	@sed -i 's|^PROJECT_NUMBER.*|PROJECT_NUMBER = $(VERSION)|' docs/Doxyfile.tmp
	@doxygen docs/Doxyfile.tmp
	@rm docs/Doxyfile.tmp
	@mkdir -p docs/man
	@doxy2man --novalidate docs/xml/xcserial_8h.xml -o docs/man
	@echo "Documentation generated in $(DOC_DIR)"

# --- Dynamic Test Suite ---
.PRECIOUS: $(BUILD_DIR)/test/%
$(BUILD_DIR)/test/%: test/%.c $(LIB_SO)
	@mkdir -p $(BUILD_DIR)/test
	@echo "  TEST    $< -> $@"
	@$(CC) $(CFLAGS) $< -o $@ -L$(BUILD_DIR) -l$(NAME) $(LIBS) -Wl,-rpath,$(abspath $(BUILD_DIR))
test-%: $(BUILD_DIR)/test/%
	@echo "  RUN     $<"
	@$<
	
# --- Cleanup ---
clean:
	rm -rf build/$(TARGET_TRIPLE) release/$(TARGET_TRIPLE) 
cleanall:
	rm -rf build/ release/ docs/html docs/man docs/xml

$(BUILD_DIR):
	mkdir -p $@

.PHONY: all clean release docs test-%