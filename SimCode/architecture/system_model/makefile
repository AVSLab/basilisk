
CC = g++
LIB_A_CMD = g++
RMCMD = rm -f
CCFLAGS= -std=c++11 -gdwarf-3 -Wall -I$(SIMULATION_BASE)/SimCode/ -I/usr/include/python2.7
CXXFLAGS= -I$(PWD)
LIBCFLAGS = $(CCFLAGS) -fPIC
LIBBFLAGS = -shared
LIBAFLAGS = -shared -L$(SIMULATION_BASE)/Binary/lib
SWIG_CMD = swig -I$(SIMULATION_BASE)/SimCode/utilities
ENDLINK_LIBS = -lSimMessaging -lSimUtilities
BUILD_DIR = $(SIMULATION_BASE)/Binary
OBJ_DIR = $(BUILD_DIR)/obj
LIB_DIR = $(BUILD_DIR)/lib
MOD_DIR = $(BUILD_DIR)/modules

UNAME_S = $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    ENDLINK_LIBS += -lpython
endif

.SECONDARY: $(OBJ_DIR)/%.py

CPP_FILES = $(wildcard *.cpp) $(wildcard *.cxx)
OBJ_BASE = $(basename $(CPP_FILES))
OBJ_FILES = $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_FILES:.cpp=.o)))
ALL_SWIG_FILES = $(wildcard *.i)
SWIG_FILES = $(basename $(ALL_SWIG_FILES))
GEN_SWIG_FILES = $(addsuffix _wrap.cxx,$(SWIG_FILES))
SWIG_WRAP_OBJ = $(addsuffix _wrap.o,$(SWIG_FILES))

all: $(MOD_DIR)/_sys_model_thread.so $(MOD_DIR)/_sim_model.so

$(MOD_DIR)/_%.so: $(OBJ_FILES) $(OBJ_DIR)/%_wrap.o
	@echo ""
	@echo "Building the sys model thread library"
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
	rm $(MOD_DIR)/_sys_model_thread.so $(MOD_DIR)/_sim_model.so $(MOD_DIR)/sys_model_thread.py $(MOD_DIR)/sim_model.py $(OBJ_DIR)/sim_model_wrap.cxx $(OBJ_DIR)/sys_model_thread_wrap.cxx

