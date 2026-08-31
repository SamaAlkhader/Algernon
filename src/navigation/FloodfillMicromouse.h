#ifndef FLOODFILL_MICROMOUSE_H
#define FLOODFILL_MICROMOUSE_H

#include <chrono>
#include <set>
#include <vector>

class FloodfillMicromouse {
public:
    FloodfillMicromouse();

    // Runs the full algorithm loop. Returns true if the goal was
    // reached, false if it gave up (no path found, or hit the safety
    // step limit).
    bool run();

private:
    // --- Direction encoding -------------------------------------------
    // 0 = North, 1 = East, 2 = South, 3 = West (clockwise order)
    static const int X_DIRECTION_OFFSET[4];
    static const int Y_DIRECTION_OFFSET[4];
    static const int LEFT_DIRECTION[4];
    static const int RIGHT_DIRECTION[4];
    static const int OPPOSITE_DIRECTION[4];

    int mazeWidth_;
    int mazeHeight_;

    // discoveredWalls_[x][y] is a set of directions (0-3) known to be
    // blocked at that cell.
    std::vector<std::vector<std::set<int>>> discoveredWalls_;

    // floodValues_[x][y] = BFS distance from that cell to the nearest
    // goal cell, using only currently discovered walls. A very large
    // value stands in for "infinity" 
    static constexpr int INF = 1000000000;
    std::vector<std::vector<int>> floodValues_;

    std::vector<std::pair<int, int>> goalCells_;

    int x_, y_, heading_;

    // Tracked purely for the results.csv / run.log evidence files
    int moveCount_;
    int turnCount_;
    int totalCellEntries_;
    std::chrono::steady_clock::time_point startTime_;

    std::vector<std::pair<int, int>> computeGoalCells() const;
    std::vector<std::vector<int>> floodFill() const;
    bool senseAndRecord();
    static char directionToLetter(int direction);
    void face(int targetDirection);
    bool step(int targetDirection);
    void writeResults(bool goalReached, int uniqueCellsVisited) const;
};

#endif  // FLOODFILL_MICROMOUSE_H
