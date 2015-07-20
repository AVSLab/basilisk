
CC = g++
LIB_A_CMD = g++
RMCMD = rm -f
CCFLAGS= -std=c++11 -g -Wall -I$(SIMULATION_BASE)/SimCode/ -I/usr/include/python2.7 -I$(SIMULATION_BASE)/External/boost_1_58_0
CXXFLAGS= -I$(PWD)
LIBCFLAGS = $(CCFLAGS) -fPIC
LIBBFLAGS = -shared
LIBAFLAGS = -shared -L$(SIMULATION_BASE)/Binary/lib 
SWIG_CMD = swig -I$(SIMULATION_BASE)/SimCode/utilities

CPP_FILES = $(wildcard *.cpp) $(wildcard *.cxx)
OBJ_BASE = $(basename $(CPP_FILES))
OBJ_FILES = $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_FILES:.cpp=.o)))
ALL_SWIG_FILES = $(wildcard *.i)
SWIG_FILES = $(basename $(ALL_SWIG_FILES))
GEN_SWIG_FILES = $(addsuffix _wrap.cxx,$(SWIG_FILES))
SWIG_WRAP_OBJ = $(addsuffix _wrap.o,$(SWIG_FILES))

ENDLINK_LIBS = -lSimMessaging -lSimUtilities
BUILD_DIR = $(SIMULATION_BASE)/Binary
OBJ_DIR = $(BUILD_DIR)/obj
LIB_DIR = $(BUILD_DIR)/lib
MOD_DIR = $(BUILD_DIR)/modules

UNAME_S = $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    ENDLINK_LIBS += -lpython
endif

all: $(MOD_DIR)/_coarse_sun_sensor.so


$(MOD_DIR)/_%.so: $(OBJ_FILES) $(OBJ_DIR)/%_wrap.o
	@echo ""
	@echo "Building the time library"
	@echo ""
	$(LIB_A_CMD) $(LIBAFLAGS) -o $@ $^ $(ENDLINK_LIBS)

$(OBJ_DIR)/%.o: %.cpp %.h
	$(CC) $(LIBCFLAGS) -c $< -o $@

$(OBJ_DIR)/%_wrap.cxx: %.i %.h
	$(SWIG_CMD) -o $@ -c++ -python $<       

$(OBJ_DIR)/%.py: $(OBJ_DIR)/%_wrap.cxx
	@mv $@ $(MOD_DIR)/

$(OBJ_DIR)/%_wrap.o: $(OBJ_DIR)/%_wrap.cxx $(OBJ_DIR)/%.py
	$(CC) $(LIBCFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm  $(OBJ_DIR)/*coarse_sun_sensor* $(MOD_DIR)/*coarse_sun_sensor*

