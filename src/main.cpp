#include "logging/RunLogger.h"
#include "maze/GoalDetector.h"
#include "model/Pose.h"
#include "navigation/WallFollower.h"
#include "simulator/MmsMouseIO.h"
#include "tracking/RunMetrics.h"
#include "tracking/VisitTracker.h"
#include "visualization/MazeVisualizer.h"

#include <algorithm>
#include <exception>
#include <iostream>
#include <string>

namespace
{
constexpr WallFollowMode DEFAULT_MODE = WallFollowMode::LEFT_HAND;
constexpr bool ENABLE_VISUALIZATION = true;
constexpr int MAX_STEPS_MULTIPLIER = 20;

WallFollowMode parseMode(int argc, char* argv[])
{
    if (argc > 1 && std::string(argv[1]) == "--right")
    {
        return WallFollowMode::RIGHT_HAND;
    }
    return DEFAULT_MODE;
}

bool actionMovesThroughKnownWall(Action action, const WallReadings& walls)
{
    switch (action)
    {
    case Action::FORWARD:
        return walls.front;
    case Action::TURN_LEFT_AND_FORWARD:
        return walls.left;
    case Action::TURN_RIGHT_AND_FORWARD:
        return walls.right;
    case Action::TURN_AROUND_AND_FORWARD:
        // mms does not expose wallBack() in the minimal interface. In a valid maze,
        // the cell behind the mouse is the cell it came from, so it should be open.
        return false;
    }
    return true;
}

void updatePoseForAction(Pose& pose, Action action)
{
    switch (action)
    {
    case Action::FORWARD:
        break;
    case Action::TURN_LEFT_AND_FORWARD:
        pose.heading = turnLeftFrom(pose.heading);
        break;
    case Action::TURN_RIGHT_AND_FORWARD:
        pose.heading = turnRightFrom(pose.heading);
        break;
    case Action::TURN_AROUND_AND_FORWARD:
        pose.heading = turnBackFrom(pose.heading);
        break;
    }

    pose.position = movedForward(pose.position, pose.heading);
}

void updateMetricsForAction(RunMetrics& metrics, Action action)
{
    switch (action)
    {
    case Action::FORWARD:
        break;
    case Action::TURN_LEFT_AND_FORWARD:
    case Action::TURN_RIGHT_AND_FORWARD:
        metrics.recordTurn();
        break;
    case Action::TURN_AROUND_AND_FORWARD:
        metrics.recordTurnAround();
        break;
    }
    metrics.recordMove();
}

void resetLocalRun(Pose& pose, VisitTracker& visits, RunMetrics& metrics, WallFollowMode mode, MazeVisualizer& visualizer)
{
    pose = {{0, 0}, Direction::NORTH};
    visits.reset();
    visits.recordVisit(pose.position);
    metrics.reset(mode);
    metrics.startTimer();
    visualizer.markStart(pose.position);
}
}

int main(int argc, char* argv[])
{
    const WallFollowMode mode = parseMode(argc, argv);

    MmsMouseIO mouse;
    RunLogger logger("run.log");

    const int mazeWidth = mouse.mazeWidth();
    const int mazeHeight = mouse.mazeHeight();
    const int maxSteps = std::max(1, mazeWidth * mazeHeight * MAX_STEPS_MULTIPLIER);

    Pose pose{{0, 0}, Direction::NORTH};
    VisitTracker visits(mazeWidth, mazeHeight);
    RunMetrics metrics(mode);
    MazeVisualizer visualizer(ENABLE_VISUALIZATION);
    WallFollower wallFollower(mode);

    visits.recordVisit(pose.position);
    metrics.startTimer();
    visualizer.markStart(pose.position);

    logger.logMessage("Starting " + wallFollowModeToString(mode));
    logger.logMessage("Maze size: " + std::to_string(mazeWidth) + " x " + std::to_string(mazeHeight));

    int step = 0;
    bool simulatorError = false;
    while (!GoalDetector::isGoal(pose.position, mazeWidth, mazeHeight))
    {
        if (step >= maxSteps)
        {
            logger.logMessage("SAFETY STOP: maximum step count exceeded");
            break;
        }

        if (mouse.wasReset())
        {
            logger.logMessage("Simulator reset detected. Restarting local tracking.");
            mouse.ackReset();
            resetLocalRun(pose, visits, metrics, mode, visualizer);
            step = 0;
            continue;
        }

        const Pose poseBeforeMove = pose;
        const WallReadings walls{
            mouse.wallLeft(),
            mouse.wallFront(),
            mouse.wallRight()
        };

        visualizer.showDiscoveredWalls(poseBeforeMove, walls);

        const Action action = wallFollower.chooseAction(walls);
        logger.logDecision(step + 1, poseBeforeMove, walls, action, visits.uniqueCellsVisited());

        if (actionMovesThroughKnownWall(action, walls))
        {
            logger.logMessage("ERROR: Selected action would move through a detected wall. Stopping safely.");
            break;
        }

        try
        {
            wallFollower.executeAction(action, mouse);
        }
        catch (const std::exception& error)
        {
            logger.logMessage(std::string("SIMULATOR ERROR: ") + error.what());
            simulatorError = true;
            break;
        }

        updatePoseForAction(pose, action);
        updateMetricsForAction(metrics, action);
        visits.recordVisit(pose.position);
        visualizer.markVisited(pose.position, visits.visitCount(pose.position));

        ++step;
    }

    const bool goalReached = !simulatorError && GoalDetector::isGoal(pose.position, mazeWidth, mazeHeight);
    if (goalReached)
    {
        visualizer.markGoal(pose.position);
    }

    metrics.stopTimer();
    metrics.setGoalReached(goalReached);
    metrics.setFinalPosition(pose.position);
    metrics.setVisitCounts(visits.uniqueCellsVisited(), visits.totalCellEntries());

    logger.writeSummary(metrics, mazeWidth, mazeHeight);
    logger.appendCsvResult(metrics, mazeWidth, mazeHeight, "results.csv");

    return 0;
}
