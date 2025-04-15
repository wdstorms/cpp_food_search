# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -O3

# Executable names
TARGET = graph
TEST_TARGET = test_food_search

# Source files
SRCS = main.cpp graph.cpp pacman_graph.cpp a*.cpp bc_decomposition/bcp_graph.cpp
TEST_SRCS = test/test_graph.cpp graph.cpp pacman_graph.cpp a*.cpp bc_decomposition/bcp_graph.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)
TEST_OBJS = $(TEST_SRCS:.cpp=.o)

# Default target
all: $(TARGET)

# Build the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Build the test executable
$(TEST_TARGET): $(TEST_OBJS)
	$(CXX) $(CXXFLAGS) -o $(TEST_TARGET) $(TEST_OBJS)

# Compile object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm *.o

test_clean:
	rm *.o
	rm test/*.o

# Phony targets
.PHONY: all clean test