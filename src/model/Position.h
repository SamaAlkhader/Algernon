#ifndef MODEL_POSITION_H
#define MODEL_POSITION_H

struct Position
{
    int x;
    int y;
};

inline bool operator==(const Position& lhs, const Position& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

#endif
