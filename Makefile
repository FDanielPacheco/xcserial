MAJOR = 5
MINOR = 0
RELEASE = 0

CC      := clang
CFLAGS  := -std=gnu99 
CFLAGS  += -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude -O2 -fPIC
AR      := ar rcs
RM      := rm -vir

LIBNAME := xcserial
SRC     := xcserial.c
ENTRY   := src
OBJ     := $(SRC:.c=.o)
DIR     := build

all: lib$(LIBNAME).so docs

$(DIR):
	@mkdir -p build
	@echo "Created directory $(DIR)"

lib$(LIBNAME).so: $(DIR) $(OBJ)
	@echo "Compiling the dynamic library ..."
	$(CC) -shared -o $@ $(DIR)/$(OBJ)
	@mv $@ $(DIR)

static: $(DIR) lib$(LIBNAME).a

lib$(LIBNAME).a: $(OBJ)
	@echo "Compiling the static library ..."
	$(AR) $@ $(DIR)/$(OBJ)
	@mv $@ $(DIR)

%.o: $(ENTRY)/%.c
	$(CC) $(CFLAGS) -c $< -o $@
	@mv $@ $(DIR)

documentation:
	@echo "Generating documentation..."
	@cd docs && doxygen Doxyfile "PREDEFINED=PROJECT_VERSION=$(MAJOR).$(MINOR).$(RELEASE)"
	@echo "Documentation generated in docs/"

clean:
	$(RM) $(DIR)