#ifndef MODEL_POSE_H
#define MODEL_POSE_H

#include "model/Direction.h"
#include "model/Position.h"

struct Pose
{
    Position position;
    Direction heading;
};

inline Position movedForward(Position position, Direction heading)
{
    switch (heading)
    {
    case Direction::NORTH:
        ++position.y;
        break;
    case Direction::EAST:
        ++position.x;
        break;
    case Direction::SOUTH:
        --position.y;
        break;
    case Direction::WEST:
        --position.x;
        break;
    }
    return position;
}

#endif
