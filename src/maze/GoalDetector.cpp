#include "maze/GoalDetector.h"

bool GoalDetector::isGoal(const Position& position, int mazeWidth, int mazeHeight)
{
    const int centerX1 = mazeWidth / 2 - 1;
    const int centerX2 = mazeWidth / 2;
    const int centerY1 = mazeHeight / 2 - 1;
    const int centerY2 = mazeHeight / 2;

    const bool xIsCenter = position.x == centerX1 || position.x == centerX2;
    const bool yIsCenter = position.y == centerY1 || position.y == centerY2;

    return xIsCenter && yIsCenter;
}
