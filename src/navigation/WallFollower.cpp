#include "navigation/WallFollower.h"

WallFollower::WallFollower(WallFollowMode mode)
    : mode_(mode)
{
}

Action WallFollower::chooseAction(const WallReadings& walls) const
{
    const bool preferLeft = mode_ == WallFollowMode::LEFT_HAND;

    if (preferLeft)
    {
        if (!walls.left)
        {
            return Action::TURN_LEFT_AND_FORWARD;
        }
        if (!walls.front)
        {
            return Action::FORWARD;
        }
        if (!walls.right)
        {
            return Action::TURN_RIGHT_AND_FORWARD;
        }
        return Action::TURN_AROUND_AND_FORWARD;
    }

    if (!walls.right)
    {
        return Action::TURN_RIGHT_AND_FORWARD;
    }
    if (!walls.front)
    {
        return Action::FORWARD;
    }
    if (!walls.left)
    {
        return Action::TURN_LEFT_AND_FORWARD;
    }
    return Action::TURN_AROUND_AND_FORWARD;
}

void WallFollower::executeAction(Action action, IMouseIO& mouse) const
{
    switch (action)
    {
    case Action::FORWARD:
        break;
    case Action::TURN_LEFT_AND_FORWARD:
        mouse.turnLeft();
        break;
    case Action::TURN_RIGHT_AND_FORWARD:
        mouse.turnRight();
        break;
    case Action::TURN_AROUND_AND_FORWARD:
        mouse.turnRight();
        mouse.turnRight();
        break;
    }

    mouse.moveForward();
}

WallFollowMode WallFollower::mode() const
{
    return mode_;
}

std::string actionToString(Action action)
{
    switch (action)
    {
    case Action::FORWARD:
        return "FORWARD";
    case Action::TURN_LEFT_AND_FORWARD:
        return "TURN_LEFT_AND_FORWARD";
    case Action::TURN_RIGHT_AND_FORWARD:
        return "TURN_RIGHT_AND_FORWARD";
    case Action::TURN_AROUND_AND_FORWARD:
        return "TURN_AROUND_AND_FORWARD";
    }
    return "UNKNOWN";
}
