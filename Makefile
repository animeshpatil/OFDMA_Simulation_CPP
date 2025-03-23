SRC_DIR = src
BIN_DIR = bin
CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2

SRC = $(SRC_DIR)/base_station.cpp $(SRC_DIR)/user.cpp $(SRC_DIR)/signal_processing.cpp

HEADERS = $(SRC_DIR)/signal_processing.h

OBJS = $(BIN_DIR)/base_station.o $(BIN_DIR)/user.o $(BIN_DIR)/signal_processing.o

BS_EXEC = base_station
USER_EXEC = user

all: build

build: $(BS_EXEC) $(USER_EXEC)

$(BS_EXEC): $(BIN_DIR)/base_station.o $(BIN_DIR)/signal_processing.o
	$(CXX) $(CXXFLAGS) -o $(BS_EXEC) $(BIN_DIR)/base_station.o $(BIN_DIR)/signal_processing.o

$(USER_EXEC): $(BIN_DIR)/user.o $(BIN_DIR)/signal_processing.o
	$(CXX) $(CXXFLAGS) -o $(USER_EXEC) $(BIN_DIR)/user.o $(BIN_DIR)/signal_processing.o

$(BIN_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	if exist $(BIN_DIR)\*.o del /Q $(BIN_DIR)\*.o
	if exist $(SRC_DIR)\*.txt del /Q $(SRC_DIR)\*.txt

run-base-station:
	./$(BS_EXEC)

run-user:
	./$(USER_EXEC) $(UID)

.PHONY: all build clean run-base run-user
