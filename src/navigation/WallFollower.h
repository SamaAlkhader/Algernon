#ifndef NAVIGATION_WALLFOLLOWER_H
#define NAVIGATION_WALLFOLLOWER_H

#include "navigation/WallFollowMode.h"
#include "simulator/IMouseIO.h"

enum class Action
{
    FORWARD,
    TURN_LEFT_AND_FORWARD,
    TURN_RIGHT_AND_FORWARD,
    TURN_AROUND_AND_FORWARD
};

struct WallReadings
{
    bool left;
    bool front;
    bool right;
};

class WallFollower
{
public:
    explicit WallFollower(WallFollowMode mode = WallFollowMode::LEFT_HAND);

    Action chooseAction(const WallReadings& walls) const;
    void executeAction(Action action, IMouseIO& mouse) const;
    WallFollowMode mode() const;

private:
    WallFollowMode mode_;
};

std::string actionToString(Action action);

#endif
