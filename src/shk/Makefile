TARGET = shwfs

# Default to release build
ifndef DEBUG
	ifndef NDEBUG
		NDEBUG = 1
	endif
endif

# Paths
vpath # clears all paths

OBJ = ./obj
vpath %.obj $(OBJ)

SYSTEM_SRC = ../system
vpath %.c $(SYSTEM_SRC)
vpath %.cpp $(SYSTEM_SRC)

PHXSDK_INCLUDE_DIR = /usr/local/active_silicon/phx_sdk-6.23/include
SYSTEM_INCLUDE_DIR = ../system
SUBARCH := $(shell uname -m)
ifeq ($(SUBARCH),x86_64)
	PFW_LIBDIR = /usr/local/active_silicon/phx_sdk-6.23/lib/linux64
	PHX_LIBDIR = /usr/local/active_silicon/phx_sdk-6.23/lib/linux64
else
	PFW_LIBDIR = /usr/local/active_silicon/phx_sdk-6.23/lib/linux
	PHX_LIBDIR = /usr/local/active_silicon/phx_sdk-6.23/lib/linux
endif

LIBS += -L$(PHX_LIBDIR) -lphx # phx api
LIBS += -L$(PFW_LIBDIR) -lpfw # phx firmware library required by phx: libphx.so referes to 'PFW_Close', 'PFW_ParameterGet', 'PFW_Open'
LIBS += -L$(PHX_LIBDIR) -lpbu # phx buffer library
# LIBS += -L$(PHX_LIBDIR) -lpim # phx image library
LIBS += -lm # strangely required by phx: libphx.so referes to 'exp' and 'log'

# Tools
CC = gcc

INCLUDE = -I$(PHXSDK_INCLUDE_DIR) -I$(SYSTEM_INCLUDE_DIR)

CFLAGS =

ifdef DEBUG
	CFLAGS += -ggdb -g
else
	CFLAGS += -O6 -fomit-frame-pointer
endif

# Preprocessor flags
CPPFLAGS = $(INCLUDE) -D_GNU_SOURCE
CPPFLAGS += -D_PHX_LINUX
CPPFLAGS += -D_CDA_LINUX -D_REENTRANT

ifdef DEBUG
	CPPFLAGS += -D_LFG_DEBUG=$(DEBUG)
else
	CPPFLAGS += -O6 -DNDEBUG
endif

# Warnings
COMMON_FLAGS = -pedantic -Wall -W -Wno-unused -Wpointer-arith -Wcast-qual -Wmissing-declarations -Wshadow -Wcast-align
CFLAGS += $(COMMON_FLAGS) -Wnested-externs -Wstrict-prototypes -Wmissing-prototypes -std=c99
# -anci -Wwrite-strings -Werror\
# gcc 2.95.2-9 extras.  Linux kernel 2.2.12 ships with gcc 2.7.2
# CFLAGS += -Wmultichar -Wunknown-pragmas -Wno-sign-compare

CXX = g++
# NB g++ 2.95.2 doesn't put STL iterators in std:: so force no-honor-std
CXXFLAGS = $(COMMON_FLAGS) -ffor-scope -Wno-reorder -Wno-ctor-dtor-privacy -Wno-effc++ -Wno-old-style-cast -std=c++11

# Linker

LINK = gcc
ifdef DEBUG
	LFLAGS = -ggdb -g3
endif

# For ASL development only
ifeq ($(SUBARCH),x86_64)
	RUNPATH = -Xlinker -rpath -Xlinker /usr/local/active_silicon/phx_sdk-6.23/lib/linux64
else
	RUNPATH = -Xlinker -rpath -Xlinker /usr/local/active_silicon/phx_sdk-6.23/lib/linux
endif

LFLAGS += $(RUNPATH)
# End for ASL development only

SRCS = $(TARGET).c phxshwfs.c config.c phoenix.c bobcat.c phoenix_bobcat.c process.cpp

TEMP = $(SRCS:%.c=$(OBJ)/%.obj)
OBJS = $(TEMP:%.cpp=$(OBJ)/%.obj)

erro = \033[31m[make-erro] 
info = \033[32m[make-info] 
warn = \033[33m[make-warn] 
esc = \033[0m

# Implicit rules
.SUFFIXES :
.SUFFIXES : .c .cpp .obj .h .a

$(OBJ)/%.obj : %.c
	@echo "$(info)building $@$(esc)"
	@-mkdir -p $(OBJ)
	$(CC) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<
	@echo "$(info)$@ build complete$(esc)"

$(OBJ)/%.obj : %.cpp
	@echo "$(info)building $@$(esc)"
	@-mkdir -p $(OBJ)
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $@ $<
	@echo "$(info)$@ build complete$(esc)"

# Explicit rules for building objects go here.
all : clean $(TARGET)

$(TARGET) : $(OBJS)
	@echo "$(info)building $(TARGET)$(esc)"
	$(LINK) $(LFLAGS) -o $@ $^ $(LIBS)
	@echo "$(info)$(TARGET) build complete$(esc)"

clean :
	@echo "$(warn)cleaning $(TARGET)$(esc)"
	-rm -f $(TARGET)
	-rm -f $(OBJS)
	-rmdir $(OBJ)
	@echo "$(warn)clean complete$(esc)"

# Dependencies
$(OBJS) : Makefile

$(OBJ)/$(TARGET).obj : $(TARGET).c

# End of file
