CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -I./src

SRC := \
	src/API.cpp \
	src/simulator/MmsMouseIO.cpp \
	src/navigation/WallFollower.cpp \
	src/model/Direction.cpp \
	src/tracking/RunMetrics.cpp \
	src/tracking/VisitTracker.cpp \
	src/maze/GoalDetector.cpp \
	src/visualization/MazeVisualizer.cpp \
	src/logging/RunLogger.cpp

APP_SRC := src/main.cpp $(SRC)
TEST_SRC := tests/test_wall_following.cpp \
	src/navigation/WallFollower.cpp \
	src/model/Direction.cpp \
	src/tracking/VisitTracker.cpp \
	src/maze/GoalDetector.cpp

BFS_SRC := \
	src/API.cpp \
	src/simulator/MmsMouseIO.cpp \
	src/navigation/BfsSolver.cpp \
	src/model/Direction.cpp \
	src/tracking/VisitTracker.cpp \
	src/tracking/BfsRunMetrics.cpp \
	src/maze/GoalDetector.cpp

BFS_APP_SRC := src/main_bfs.cpp $(BFS_SRC)
BFS_TEST_SRC := tests/test_bfs.cpp \
	src/navigation/BfsSolver.cpp \
	src/model/Direction.cpp \
	src/maze/GoalDetector.cpp

.PHONY: all clean test bfs bfs-test

all: wall_follower

wall_follower: $(APP_SRC)
	$(CXX) $(CXXFLAGS) $(APP_SRC) -o wall_follower

bfs: bfs_solver

bfs_solver: $(BFS_APP_SRC)
	$(CXX) $(CXXFLAGS) $(BFS_APP_SRC) -o bfs_solver

test: wall_follower_tests
	./wall_follower_tests

wall_follower_tests: $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) -o wall_follower_tests

bfs-test: bfs_tests
	./bfs_tests

bfs_tests: $(BFS_TEST_SRC)
	$(CXX) $(CXXFLAGS) $(BFS_TEST_SRC) -o bfs_tests

clean:
	rm -f wall_follower wall_follower_tests bfs_solver bfs_tests results.csv run.log
