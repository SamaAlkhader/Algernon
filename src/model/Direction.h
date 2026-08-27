#ifndef MODEL_DIRECTION_H
#define MODEL_DIRECTION_H

#include <string>

enum class Direction
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};

enum class RelativeDirection
{
    FRONT,
    LEFT,
    RIGHT,
    BACK
};

Direction turnLeftFrom(Direction heading);
Direction turnRightFrom(Direction heading);
Direction turnBackFrom(Direction heading);
Direction relativeToAbsolute(Direction heading, RelativeDirection relative);
char directionToMmsChar(Direction direction);
std::string directionToString(Direction direction);

#endif
