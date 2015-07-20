
CC = g++
LIB_A_CMD = g++
RMCMD = rm -f
CCFLAGS= -gdwarf-3 -Wall -I$(SIMULATION_BASE)/SimCode/
CXXFLAGS= -DOS_LINUX
LIBCFLAGS = $(CCFLAGS) -fPIC
LIBBFLAGS = -shared
LIBAFLAGS = -shared 
SWIG_CMD = swig
BUILD_DIR = $(SIMULATION_BASE)/Binary/
OBJ_DIR = $(BUILD_DIR)/obj
LIB_DIR = $(BUILD_DIR)/lib

CPP_FILES = $(wildcard *.cpp)
OBJ_FILES = $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_FILES:.cpp=.o)))

all: $(LIB_DIR)/libSimMessaging.so

$(LIB_DIR)/libSimMessaging.so: $(OBJ_FILES)
	@echo ""
	@echo "Building the messaging shared library"
	@echo ""
	$(LIB_A_CMD) $(LIBAFLAGS) -o $@ $^

$(OBJ_DIR)/%.o: %.cpp
	$(CC) $(LIBCFLAGS) -c -o $@ $<

clean:
	rm $(OBJ_FILES) \
        $(LIB_DIR)/libSimMessaging.so

