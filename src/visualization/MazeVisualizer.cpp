#include "visualization/MazeVisualizer.h"

#include "API.h"
#include "model/Direction.h"

MazeVisualizer::MazeVisualizer(bool enabled)
    : enabled_(enabled)
{
}

void MazeVisualizer::showDiscoveredWalls(const Pose& pose, const WallReadings& walls) const
{
    if (!enabled_)
    {
        return;
    }

    if (walls.front)
    {
        API::setWall(pose.position.x, pose.position.y, directionToMmsChar(relativeToAbsolute(pose.heading, RelativeDirection::FRONT)));
    }
    if (walls.left)
    {
        API::setWall(pose.position.x, pose.position.y, directionToMmsChar(relativeToAbsolute(pose.heading, RelativeDirection::LEFT)));
    }
    if (walls.right)
    {
        API::setWall(pose.position.x, pose.position.y, directionToMmsChar(relativeToAbsolute(pose.heading, RelativeDirection::RIGHT)));
    }
}

void MazeVisualizer::markStart(const Position& position) const
{
    if (enabled_)
    {
        API::setColor(position.x, position.y, 'g');
        API::setText(position.x, position.y, "S");
    }
}

void MazeVisualizer::markVisited(const Position& position, int visitCount) const
{
    if (enabled_)
    {
        API::setColor(position.x, position.y, 'y');
        API::setText(position.x, position.y, std::to_string(visitCount));
    }
}

void MazeVisualizer::markGoal(const Position& position) const
{
    if (enabled_)
    {
        API::setColor(position.x, position.y, 'g');
        API::setText(position.x, position.y, "G");
    }
}
