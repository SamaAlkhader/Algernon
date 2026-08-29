#ifndef NAVIGATION_BFSSOLVER_H
#define NAVIGATION_BFSSOLVER_H

#include <array>
#include <vector>
#include <string>

#include "model/Position.h"
#include "model/Direction.h"
#include "simulator/IMouseIO.h"

// Same Action vocabulary/shape as WallFollower's Action enum, but this is a
// SEPARATE type (not shared) so BfsSolver.h has no compile-time dependency
// on navigation/WallFollower.h. Do not include both WallFollower.h and
// BfsSolver.h in the same translation unit -- their Action enums would
// collide by name. main_bfs.cpp only needs this one.
enum class Action
{
    FORWARD,
    TURN_LEFT_AND_FORWARD,
    TURN_RIGHT_AND_FORWARD,
    TURN_AROUND_AND_FORWARD,
    NONE // solver has no more moves queued (run finished)
};

// Tri-state wall knowledge. Defaulting to UNKNOWN (rather than "open") is
// what keeps the frontier search from tunneling through cells it has never
// actually sensed.
enum class WallState { UNKNOWN, OPEN, WALL };

enum class BfsPhase { EXPLORING, RETURNING_HOME, DRIVING_FINAL, DONE };

enum class ExplorationMode
{
    FULL_MAZE,
    UNTIL_FIRST_GOAL,
    SINGLE_PASS_TO_GOAL
};

// One cell of the solver's internal wall map.
struct BfsCell
{
    // wallState[Direction] indexed by static_cast<int>(Direction).
    std::array<WallState, 4> wallState = {
        WallState::UNKNOWN, WallState::UNKNOWN, WallState::UNKNOWN, WallState::UNKNOWN
    };
    bool explored = false; // has the mouse physically visited this cell?
    int visitCount = 0;    // for VisitTracker / heatmap use
};

// BfsSolver supports full-map two-pass, early-center two-pass, and single-pass
// frontier-BFS runs. The mode is selected without changing the core algorithm.
//
// Unlike WallFollower, BfsSolver is stateful: it must remember discovered
// walls and its own position/heading between calls, since BFS decisions
// depend on everything learned so far, not just the immediate sensor reading.
class BfsSolver
{
public:
    BfsSolver(int mazeWidth, int mazeHeight,
              ExplorationMode mode = ExplorationMode::FULL_MAZE);

    // Called once at the start of a run, after mms confirms mouse pose (0,0)/NORTH.
    void begin(IMouseIO& mouse);

    // Returns true while the solver still has work to do (exploring or
    // driving the final path). Caller (main.cpp) should loop:
    //     while (solver.step(mouse)) { ...log/visualize... }
    bool step(IMouseIO& mouse);

    // Read-only accessors for tracking/visualization/logging code.
    Position position() const;
    Direction heading() const;
    int visitCount(const Position& p) const;
    bool goalReached() const;
    const std::vector<std::vector<BfsCell>>& knownMaze() const;

    // The Action taken by the most recently completed step() call (or
    // Action::NONE before the first step / once the run has finished).
    // Lets main_bfs.cpp log per-step decisions the same way main.cpp does
    // for WallFollower, without BfsSolver needing to know about RunLogger.
    Action lastAction() const;

    // True once step() has returned false and there is truly nothing left
    // to do (distinct from goalReached(), which can be true mid-run while
    // BFS is still finishing exploration/return-home/final-drive phases).
    bool finished() const;
    BfsPhase phase() const;

private:
    int width_;
    int height_;
    ExplorationMode explorationMode_;
    std::vector<std::vector<BfsCell>> maze_; // maze_[x][y]

    Position pos_{0, 0};
    Direction heading_ = Direction::NORTH;

    BfsPhase phase_ = BfsPhase::EXPLORING;

    std::vector<Direction> pendingPath_; // remaining directions in current sub-path
    bool everReachedGoal_ = false;
    Action lastAction_ = Action::NONE;

    void sense(IMouseIO& mouse);
    bool inBounds(const Position& p) const;
    Action stepToward(Direction want) const;
    void applyAction(Action action, IMouseIO& mouse);

    std::vector<Direction> bfsToFrontier(const Position& start) const;
    std::vector<Direction> bfsShortestPath(const Position& start,
                                            const Position& goal) const;
    std::vector<Position> centerCells() const;

    // Lets tests/test_bfs.cpp exercise the private BFS routines directly
    // without needing a full IMouseIO-driven run.
    friend struct BfsSolverTestAccess;
};

std::string actionToString(Action action); // reuse-friendly, matches WallFollower's helper

#endif
