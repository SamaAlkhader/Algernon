#ifndef NAVIGATION_WALLFOLLOWMODE_H
#define NAVIGATION_WALLFOLLOWMODE_H

#include <string>

enum class WallFollowMode
{
    LEFT_HAND,
    RIGHT_HAND
};

inline std::string wallFollowModeToString(WallFollowMode mode)
{
    return mode == WallFollowMode::LEFT_HAND
        ? "Left-Hand Wall Following"
        : "Right-Hand Wall Following";
}

#endif
