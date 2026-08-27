#ifndef VISUALIZATION_MAZEVISUALIZER_H
#define VISUALIZATION_MAZEVISUALIZER_H

#include "model/Pose.h"
#include "navigation/WallFollower.h"

class MazeVisualizer
{
public:
    explicit MazeVisualizer(bool enabled);

    void showDiscoveredWalls(const Pose& pose, const WallReadings& walls) const;
    void markStart(const Position& position) const;
    void markVisited(const Position& position, int visitCount) const;
    void markGoal(const Position& position) const;

private:
    bool enabled_;
};

#endif
