#ifndef MAZE_GOALDETECTOR_H
#define MAZE_GOALDETECTOR_H

#include "model/Position.h"

class GoalDetector
{
public:
    static bool isGoal(const Position& position, int mazeWidth, int mazeHeight);
};

#endif
