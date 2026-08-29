// ============================================================================
// main_bfs.cpp - Entry point for the BFS (optimized) algorithm.
// Author: Sahar
//
// Deliberately a SEPARATE executable from main.cpp (Sama's wall-follower
// entry point). Nothing here edits WallFollower.h/.cpp, RunMetrics.h/.cpp,
// RunLogger.h/.cpp, or WallFollowMode.h -- BFS uses its own BfsRunMetrics /
// BfsRunLogger (tracking/BfsRunMetrics.*) instead, which write the exact
// same run.log style and results.csv column layout so Rawan's comparison
// table and heatmap pipeline can read BFS rows identically to the other
// two algorithms.
//
// Shared, reused as-is (no edits): GoalDetector, VisitTracker, Position,
// Direction, Pose, IMouseIO/MmsMouseIO.
// ============================================================================

#include "maze/GoalDetector.h"
#include "model/Pose.h"
#include "navigation/BfsSolver.h"
#include "simulator/MmsMouseIO.h"
#include "tracking/BfsRunMetrics.h"
#include "tracking/VisitTracker.h"
#include "API.h"

#include <algorithm>
#include <exception>
#include <iostream>
#include <string>

namespace
{
constexpr bool ENABLE_VISUALIZATION = true; // reserved; wire up MazeVisualizer here if/when desired
constexpr int MAX_STEPS_MULTIPLIER = 20;    // same safety-stop convention as main.cpp

// Full mapping, return home, final shortest run
constexpr ExplorationMode EXPLORATION_MODE = ExplorationMode::FULL_MAZE;

// Stop learning at center, return home, final known route
// constexpr ExplorationMode EXPLORATION_MODE = ExplorationMode::UNTIL_FIRST_GOAL;

// One pass: explore with frontier BFS and stop at first center
// constexpr ExplorationMode EXPLORATION_MODE = ExplorationMode::SINGLE_PASS_TO_GOAL;

std::string actionName(Action action)
{
    switch (action)
    {
        case Action::FORWARD: return "FORWARD";
        case Action::TURN_LEFT_AND_FORWARD: return "TURN_LEFT_AND_FORWARD";
        case Action::TURN_RIGHT_AND_FORWARD: return "TURN_RIGHT_AND_FORWARD";
        case Action::TURN_AROUND_AND_FORWARD: return "TURN_AROUND_AND_FORWARD";
        case Action::NONE: return "NONE";
    }
    return "UNKNOWN";
}

std::string phaseName(BfsPhase phase)
{
    switch (phase)
    {
        case BfsPhase::EXPLORING: return "learning maze";
        case BfsPhase::RETURNING_HOME: return "returning home";
        case BfsPhase::DRIVING_FINAL: return "final shortest run";
        case BfsPhase::DONE: return "done";
    }
    return "unknown";
}

void showCell(const Position& position, int visitCount, BfsPhase phase)
{
    if (!ENABLE_VISUALIZATION) return;
    char color = 'a';
    if (phase == BfsPhase::EXPLORING)
        color = visitCount == 1 ? 'a' : 'o';
    else if (phase == BfsPhase::RETURNING_HOME)
        color = 'c';
    else if (phase == BfsPhase::DRIVING_FINAL || phase == BfsPhase::DONE)
        color = 'b';

    API::setColor(position.x, position.y, color);
    API::setText(position.x, position.y, std::to_string(visitCount));
}

void showKnownWalls(const BfsSolver& solver)
{
    if (!ENABLE_VISUALIZATION) return;
    const Position position = solver.position();
    const BfsCell& cell = solver.knownMaze()[position.x][position.y];
    for (int direction = 0; direction < 4; ++direction)
    {
        if (cell.wallState[direction] == WallState::WALL)
        {
            API::setWall(position.x, position.y,
                         directionToMmsChar(static_cast<Direction>(direction)));
        }
    }
}

void announce(BfsRunLogger& logger, const std::string& message)
{
    logger.logMessage(message);
    std::cerr << "[BFS] " << message << std::endl;
}
}

int main()
{
    MmsMouseIO mouse;
    BfsRunLogger logger("run.log"); // same filename convention as main.cpp;
                                     // run BFS and wall-follower as separate
                                     // mms algorithm entries so they don't
                                     // clobber each other's run.log mid-session

    const int mazeWidth = mouse.mazeWidth();
    const int mazeHeight = mouse.mazeHeight();
    const int maxSteps = std::max(1, mazeWidth * mazeHeight * MAX_STEPS_MULTIPLIER);

    VisitTracker visits(mazeWidth, mazeHeight);
    BfsRunMetrics metrics;
    BfsSolver solver(mazeWidth, mazeHeight, EXPLORATION_MODE);

    visits.recordVisit(Position{0, 0});
    API::setColor(0, 0, 'g');
    API::setText(0, 0, "S");
    metrics.startTimer();

    std::string explorationLabel = "single pass to center";
    if (EXPLORATION_MODE == ExplorationMode::FULL_MAZE)
        explorationLabel = "two pass, full exploration";
    else if (EXPLORATION_MODE == ExplorationMode::UNTIL_FIRST_GOAL)
        explorationLabel = "two pass, stop learning at first center";
    announce(logger, "Starting BFS (" + explorationLabel + ") on " +
                     std::to_string(mazeWidth) + " x " +
                     std::to_string(mazeHeight) + " maze");

    solver.begin(mouse);
    showKnownWalls(solver);

    int step = 0;
    bool simulatorError = false;
    bool loggedFirstGoal = false;
    BfsPhase previousPhase = solver.phase();

    while (!solver.finished())
    {
        if (step >= maxSteps)
        {
            announce(logger, "SAFETY STOP: maximum step count exceeded");
            break;
        }

        if (mouse.wasReset())
        {
            announce(logger, "Simulator reset detected; stopping because learned walls are no longer valid");
            break;
        }

        bool moved;
        try
        {
            moved = solver.step(mouse);
        }
        catch (const std::exception& error)
        {
            announce(logger, std::string("SIMULATOR ERROR: ") + error.what());
            simulatorError = true;
            break;
        }

        if (!moved)
        {
            break; // solver reports nothing left to do
        }

        ++step;

        const Position pos = solver.position();
        visits.recordVisit(pos);
        showKnownWalls(solver);
        showCell(pos, visits.visitCount(pos), solver.phase());

        Action action = solver.lastAction();
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
            case Action::NONE:
                break;
        }
        metrics.recordMove();

        logger.logStep(step, pos, actionName(action), visits.uniqueCellsVisited());

        if (!loggedFirstGoal && GoalDetector::isGoal(pos, mazeWidth, mazeHeight))
        {
            announce(logger, "Center discovered at step " + std::to_string(step) +
                             "; continuing the learning pass");
            loggedFirstGoal = true;
        }

        if (solver.phase() != previousPhase)
        {
            announce(logger, "Phase: " + phaseName(solver.phase()));
            previousPhase = solver.phase();
        }
    }

    const Position finalPos = solver.position();
    const bool goalReached = !simulatorError && GoalDetector::isGoal(finalPos, mazeWidth, mazeHeight);

    if (goalReached)
    {
        API::setColor(finalPos.x, finalPos.y, 'g');
        API::setText(finalPos.x, finalPos.y, "G");
        announce(logger, "Solved: final shortest run stopped in the center");
    }
    else
    {
        announce(logger, "Not solved: final position is outside the center");
    }

    metrics.stopTimer();
    metrics.setGoalReached(goalReached);
    metrics.setFinalPosition(finalPos);
    metrics.setVisitCounts(visits.uniqueCellsVisited(), visits.totalCellEntries());

    announce(logger, "Summary: steps=" + std::to_string(step) +
                     ", turns=" + std::to_string(metrics.turns()) +
                     ", unique cells=" + std::to_string(metrics.uniqueCellsVisited()) +
                     ", total entries=" + std::to_string(metrics.totalCellEntries()) +
                     ", elapsed=" + std::to_string(metrics.elapsedMilliseconds()) + " ms" +
                     ", final=(" + std::to_string(finalPos.x) + "," +
                     std::to_string(finalPos.y) + ")");

    logger.writeSummary(metrics, mazeWidth, mazeHeight, "BFS");
    logger.appendCsvResult(metrics, mazeWidth, mazeHeight, "results.csv", "BFS");

    // Visit-count matrix for Rawan's heatmap: same VisitTracker the wall
    // follower uses, so the heatmap pipeline reads it identically.
    logger.logMessage("\n--- Visit-count matrix (for heatmap; row = y, printed top->bottom) ---");
    for (int y = mazeHeight - 1; y >= 0; --y)
    {
        std::string row;
        for (int x = 0; x < mazeWidth; ++x)
        {
            row += std::to_string(visits.visitCount(Position{x, y})) + " ";
        }
        logger.logMessage(row);
    }

    return 0;
}

