
CC = g++
LIB_A_CMD = g++
RMCMD = rm -f
CCFLAGS= -gdwarf-3 -Wall -I$(SIMULATION_BASE)/ADCSAlgorithms/ -I$(SIMULATION_BASE)/  -I$(SIMULATION_BASE)/SimCode/
CXXFLAGS= -DOS_LINUX
LIBCFLAGS = $(CCFLAGS) -fPIC
LIBBFLAGS = -shared
LIBAFLAGS = -shared -L$(SIMULATION_BASE)/Binary/lib
ENDLINK_LIBS = -lSimMessaging
SWIG_CMD = swig
BUILD_DIR = $(SIMULATION_BASE)/Binary/
OBJ_DIR = $(BUILD_DIR)/obj
LIB_DIR = $(BUILD_DIR)/lib

CPP_FILES = $(wildcard *.cpp)
OBJ_FILES = $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_FILES:.cpp=.o)))

all: $(LIB_DIR)/libADCSMessaging.so

$(LIB_DIR)/libADCSMessaging.so: $(OBJ_FILES)
	@echo ""
	@echo "Building the messaging shared library"
	@echo ""
	$(LIB_A_CMD) $(LIBAFLAGS) -o $@ $^ $(ENDLINK_LIBS)

$(OBJ_DIR)/%.o: %.cpp
	$(CC) $(LIBCFLAGS) -c -o $@ $<

clean:
	rm $(OBJ_FILES) \
        $(LIB_DIR)/libADCSMessaging.so

