CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

SRCS = srcmain.cpp srcAPI.cpp srcFloodfillMicromouse.cpp
TARGET = floodfill_mouse

$(TARGET) $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

clean
	rm -f $(TARGET) mms_debug_log.txt results.csv run.log

.PHONY clean