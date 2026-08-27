#include "model/Direction.h"

Direction turnLeftFrom(Direction heading)
{
    switch (heading)
    {
    case Direction::NORTH:
        return Direction::WEST;
    case Direction::WEST:
        return Direction::SOUTH;
    case Direction::SOUTH:
        return Direction::EAST;
    case Direction::EAST:
        return Direction::NORTH;
    }
    return Direction::NORTH;
}

Direction turnRightFrom(Direction heading)
{
    switch (heading)
    {
    case Direction::NORTH:
        return Direction::EAST;
    case Direction::EAST:
        return Direction::SOUTH;
    case Direction::SOUTH:
        return Direction::WEST;
    case Direction::WEST:
        return Direction::NORTH;
    }
    return Direction::NORTH;
}

Direction turnBackFrom(Direction heading)
{
    return turnRightFrom(turnRightFrom(heading));
}

Direction relativeToAbsolute(Direction heading, RelativeDirection relative)
{
    switch (relative)
    {
    case RelativeDirection::FRONT:
        return heading;
    case RelativeDirection::LEFT:
        return turnLeftFrom(heading);
    case RelativeDirection::RIGHT:
        return turnRightFrom(heading);
    case RelativeDirection::BACK:
        return turnBackFrom(heading);
    }
    return heading;
}

char directionToMmsChar(Direction direction)
{
    switch (direction)
    {
    case Direction::NORTH:
        return 'n';
    case Direction::EAST:
        return 'e';
    case Direction::SOUTH:
        return 's';
    case Direction::WEST:
        return 'w';
    }
    return 'n';
}

std::string directionToString(Direction direction)
{
    switch (direction)
    {
    case Direction::NORTH:
        return "NORTH";
    case Direction::EAST:
        return "EAST";
    case Direction::SOUTH:
        return "SOUTH";
    case Direction::WEST:
        return "WEST";
    }
    return "UNKNOWN";
}
