CC		= g++
C_FLAGS = -g -Wall

BIN		= bin
SRCS	= src/*.cpp
PROG = bin/main

SOURCES := $(shell find . -name '*.cpp')

HEADERS := $(shell find . -name '*.hpp')


LIB := lib

OPENCV = `pkg-config opencv4 --cflags --libs`
LIBS = $(OPENCV)

# EXECUTABLE	:= main

# $(PROG):$(SRCS)
# 	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
	
EXECUTABLE = main
all: $(BIN)/$(EXECUTABLE)

clean:
	$(RM) $(BIN)/$(EXECUTABLE)

run: all
	./$(BIN)/$(EXECUTABLE)

$(BIN)/$(EXECUTABLE): $(SOURCES) $(HEADERS)	
	$(CC) $(C_FLAGS) -o ./$(BIN)/$(EXECUTABLE) $(SOURCES) $(LIBS)




