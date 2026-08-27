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

.PHONY: all clean test

all: wall_follower

wall_follower: $(APP_SRC)
	$(CXX) $(CXXFLAGS) $(APP_SRC) -o wall_follower

test: wall_follower_tests
	./wall_follower_tests

wall_follower_tests: $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) -o wall_follower_tests

clean:
	rm -f wall_follower wall_follower_tests results.csv run.log
