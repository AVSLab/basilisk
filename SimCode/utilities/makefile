
CC = g++
LIB_A_CMD = g++
RMCMD = rm -f
CCFLAGS= -std=c++11 -gdwarf-3 -Wall -I$(SIMULATION_BASE)/SimCode/
CXXFLAGS= -DOS_LINUX
LIBCFLAGS = $(CCFLAGS) -fPIC
LIBBFLAGS = -shared
LIBAFLAGS = -shared 
SWIG_CMD = swig
BUILD_DIR = $(SIMULATION_BASE)/Binary/
OBJ_DIR = $(BUILD_DIR)/obj
LIB_DIR = $(BUILD_DIR)/lib

CPP_FILES = $(wildcard *.cpp)
C_FILES = $(wildcard *.c)
OBJ_FILES = $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_FILES:.cpp=.o)))
OBJ_FILES += $(addprefix $(OBJ_DIR)/,$(notdir $(C_FILES:.c=.o)))

all: $(LIB_DIR)/libSimUtilities.so

$(LIB_DIR)/lib%.so: $(OBJ_FILES)
	@echo ""
	@echo "Building the utility library"
	@echo ""
	$(LIB_A_CMD) $(LIBAFLAGS) -o $@ $^

$(OBJ_DIR)/%.o: %.cpp
	$(CC) $(LIBCFLAGS) -c -o $@ $<

$(OBJ_DIR)/%.o: %.c
	$(CC) $(LIBCFLAGS) -c -o $@ $<

clean:
	rm $(LIB_DIR)/libSimUtilities.so

