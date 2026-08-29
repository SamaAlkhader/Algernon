#include "navigation/BfsSolver.h"
#include "maze/GoalDetector.h"

#include <queue>
#include <iostream>
#include <algorithm>
#include <utility>

namespace
{
    // Direction <-> offset, indexed by static_cast<int>(Direction)
    // NORTH=0, EAST=1, SOUTH=2, WEST=3 (matches enum declaration order)
    constexpr int DX[4] = { 0, 1, 0, -1 };
    constexpr int DY[4] = { 1, 0, -1, 0 };

    Position moved(const Position& p, Direction d)
    {
        int i = static_cast<int>(d);
        return Position{ p.x + DX[i], p.y + DY[i] };
    }
}

BfsSolver::BfsSolver(int mazeWidth, int mazeHeight, ExplorationMode mode)
    : width_(mazeWidth), height_(mazeHeight), explorationMode_(mode)
{
    maze_.assign(width_, std::vector<BfsCell>(height_));
}

bool BfsSolver::inBounds(const Position& p) const
{
    return p.x >= 0 && p.x < width_ && p.y >= 0 && p.y < height_;
}

void BfsSolver::begin(IMouseIO& mouse)
{
    maze_.assign(width_, std::vector<BfsCell>(height_));
    pos_ = Position{ 0, 0 };
    heading_ = Direction::NORTH;
    pendingPath_.clear();
    everReachedGoal_ = GoalDetector::isGoal(pos_, width_, height_);
    lastAction_ = Action::NONE;
    maze_[pos_.x][pos_.y].explored = true;
    maze_[pos_.x][pos_.y].visitCount = 1;
    sense(mouse);
    phase_ = BfsPhase::EXPLORING;
}

// Reads sensors relative to current heading, converts to absolute Direction,
// and records both sides of each discovered wall.
void BfsSolver::sense(IMouseIO& mouse)
{
    bool front = mouse.wallFront();
    bool left  = mouse.wallLeft();
    bool right = mouse.wallRight();

    Direction frontDir = relativeToAbsolute(heading_, RelativeDirection::FRONT);
    Direction leftDir  = relativeToAbsolute(heading_, RelativeDirection::LEFT);
    Direction rightDir = relativeToAbsolute(heading_, RelativeDirection::RIGHT);

    auto mark = [this](Direction d, bool present)
    {
        int di = static_cast<int>(d);
        WallState state = present ? WallState::WALL : WallState::OPEN;
        maze_[pos_.x][pos_.y].wallState[di] = state;
        Position n = moved(pos_, d);
        if (inBounds(n))
        {
            // opposite direction = (d + 2) % 4 in this enum ordering
            int oi = (di + 2) % 4;
            maze_[n.x][n.y].wallState[oi] = state;
        }
    };

    mark(frontDir, front);
    mark(leftDir,  left);
    mark(rightDir, right);
}

// Translates "I want to face/move in direction `want`" into the same
// Action vocabulary WallFollower uses, based on current heading.
Action BfsSolver::stepToward(Direction want) const
{
    if (want == heading_) return Action::FORWARD;
    if (want == turnLeftFrom(heading_)) return Action::TURN_LEFT_AND_FORWARD;
    if (want == turnRightFrom(heading_)) return Action::TURN_RIGHT_AND_FORWARD;
    return Action::TURN_AROUND_AND_FORWARD; // must be turnBackFrom(heading_)
}

void BfsSolver::applyAction(Action action, IMouseIO& mouse)
{
    lastAction_ = action;
    switch (action)
    {
        case Action::FORWARD:
            break;
        case Action::TURN_LEFT_AND_FORWARD:
            mouse.turnLeft();
            heading_ = turnLeftFrom(heading_);
            break;
        case Action::TURN_RIGHT_AND_FORWARD:
            mouse.turnRight();
            heading_ = turnRightFrom(heading_);
            break;
        case Action::TURN_AROUND_AND_FORWARD:
            mouse.turnRight();
            mouse.turnRight();
            heading_ = turnBackFrom(heading_);
            break;
        case Action::NONE:
            return;
    }
    mouse.moveForward();
    pos_ = moved(pos_, heading_);

    maze_[pos_.x][pos_.y].explored = true;
    maze_[pos_.x][pos_.y].visitCount++;
    sense(mouse);
}

// All center cells for the current maze size (handles both 1-cell-center
// odd sizes and 2x2-center even sizes like 8x8/16x16), delegating the
// actual per-cell test to GoalDetector so both algorithms agree on "goal".
std::vector<Position> BfsSolver::centerCells() const
{
    std::vector<Position> cells;
    for (int x = 0; x < width_; ++x)
        for (int y = 0; y < height_; ++y)
            if (GoalDetector::isGoal(Position{ x, y }, width_, height_))
                cells.push_back(Position{ x, y });
    return cells;
}

// BFS #1: find the shortest path (over cells the mouse has already stood in,
// via edges it has already sensed as OPEN) to the nearest still-unexplored
// cell. The path returned includes the final step off known territory onto
// that unexplored cell, which is exactly the one new cell the mouse will
// sense next.
//
// Correctness note: we only ever traverse an edge whose WallState is OPEN
// (never UNKNOWN), and we only expand through cells that are already
// `explored`. That is what stops the search from "hallucinating" a path
// through cells it has never actually measured. Visited-on-enqueue still
// guarantees the search itself terminates.
std::vector<Direction> BfsSolver::bfsToFrontier(const Position& start) const
{
    std::vector<std::vector<bool>> visited(width_, std::vector<bool>(height_, false));
    std::vector<std::vector<Direction>> parentDir(
        width_, std::vector<Direction>(height_, Direction::NORTH));
    std::vector<std::vector<Position>> parentCell(
        width_, std::vector<Position>(height_, Position{ -1, -1 }));

    std::queue<Position> q;
    visited[start.x][start.y] = true;
    q.push(start);

    while (!q.empty())
    {
        Position cur = q.front(); q.pop();
        const BfsCell& cell = maze_[cur.x][cur.y];

        for (int i = 0; i < 4; ++i)
        {
            if (cell.wallState[i] != WallState::OPEN) continue; // must be a *known* opening
            Position n = moved(cur, static_cast<Direction>(i));
            if (!inBounds(n)) continue;
            if (visited[n.x][n.y]) continue;

            visited[n.x][n.y] = true;
            parentDir[n.x][n.y] = static_cast<Direction>(i);
            parentCell[n.x][n.y] = cur;

            if (!maze_[n.x][n.y].explored)
            {
                // Found it: n is one known-open step beyond explored territory.
                std::vector<Direction> path;
                Position p = n;
                while (!(p == start))
                {
                    Direction d = parentDir[p.x][p.y];
                    path.push_back(d);
                    p = parentCell[p.x][p.y];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            q.push(n); // explored neighbor: keep expanding through it
        }
    }
    return {}; // queue exhausted, no reachable unexplored cell: exploration complete
}

// BFS #2: true shortest path on the known map from `start` to `goal`.
std::vector<Direction> BfsSolver::bfsShortestPath(const Position& start,
                                                   const Position& goal) const
{
    std::vector<std::vector<bool>> visited(width_, std::vector<bool>(height_, false));
    std::vector<std::vector<Direction>> parentDir(
        width_, std::vector<Direction>(height_, Direction::NORTH));
    std::vector<std::vector<Position>> parentCell(
        width_, std::vector<Position>(height_, Position{ -1, -1 }));

    std::queue<Position> q;
    visited[start.x][start.y] = true;
    q.push(start);

    bool found = start == goal;
    Position reached = start;

    while (!q.empty() && !found)
    {
        Position cur = q.front(); q.pop();
        const BfsCell& cell = maze_[cur.x][cur.y];

        for (int i = 0; i < 4; ++i)
        {
            if (cell.wallState[i] != WallState::OPEN) continue; // only travel known-open edges
            Position n = moved(cur, static_cast<Direction>(i));
            if (!inBounds(n)) continue;
            if (visited[n.x][n.y]) continue;
            visited[n.x][n.y] = true;
            parentDir[n.x][n.y] = static_cast<Direction>(i);
            parentCell[n.x][n.y] = cur;
            q.push(n);
            if (n == goal) { found = true; reached = n; break; }
        }
    }

    std::vector<Direction> path;
    if (!found) return path; // failure: unreachable with current wall knowledge

    Position p = reached;
    while (!(p == start))
    {
        Direction d = parentDir[p.x][p.y];
        path.push_back(d);
        p = parentCell[p.x][p.y];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Advances the solver by exactly one physical move. Returns false once the
// entire run (explore -> return home -> drive shortest path) is finished.
bool BfsSolver::step(IMouseIO& mouse)
{
    if (phase_ == BfsPhase::DONE && pendingPath_.empty()) return false;

    while (pendingPath_.empty() && phase_ != BfsPhase::DONE)
    {
        switch (phase_)
        {
            case BfsPhase::EXPLORING:
            {
                pendingPath_ = bfsToFrontier(pos_);
                if (pendingPath_.empty())
                {
                    // No frontier left. If we've reached goal, move on;
                    // otherwise maze is fully known but goal unreachable.
                    phase_ = everReachedGoal_ ? BfsPhase::RETURNING_HOME : BfsPhase::DONE;
                    if (phase_ == BfsPhase::DONE) return false;
                }
                break;
            }
            case BfsPhase::RETURNING_HOME:
            {
                if (pos_ == Position{ 0, 0 })
                {
                    // Already home (e.g. exploration ended right at start);
                    // nothing to walk, go straight to the final scored leg.
                    phase_ = BfsPhase::DRIVING_FINAL;
                    continue; // re-enter switch this same call, no move consumed
                }
                pendingPath_ = bfsShortestPath(pos_, Position{ 0, 0 });
                if (pendingPath_.empty())
                {
                    // Known map says home is unreachable - shouldn't happen
                    // once the whole maze has been explored, but fail safe.
                    phase_ = BfsPhase::DONE;
                    return false;
                }
                break;
            }
            case BfsPhase::DRIVING_FINAL:
            {
#ifdef BFS_DEBUG
                std::cerr << "[DEBUG] entering DRIVING_FINAL at pos=(" << pos_.x << "," << pos_.y << ")\n";
#endif
                auto centers = centerCells();
                if (std::find(centers.begin(), centers.end(), pos_) != centers.end())
                {
                    // Already standing on a center cell: nothing left to drive.
                    phase_ = BfsPhase::DONE;
                    return false;
                }

                // Find nearest known center cell and take the shortest path to it.
                std::vector<Direction> best;
                for (const Position& goal : centers)
                {
                    auto candidate = bfsShortestPath(pos_, goal);
#ifdef BFS_DEBUG
                    std::cerr << "[DEBUG] candidate to (" << goal.x << "," << goal.y << ") len=" << candidate.size() << "\n";
#endif
                    if (!candidate.empty() &&
                        (best.empty() || candidate.size() < best.size()))
                    {
                        best = candidate;
                    }
                }
                pendingPath_ = best;
#ifdef BFS_DEBUG
                std::cerr << "[DEBUG] DRIVING_FINAL chosen path len=" << pendingPath_.size() << "\n";
#endif
                if (pendingPath_.empty()) return false;
                break;
            }
            default:
                return false;
        }
    }

    if (phase_ == BfsPhase::DONE && pendingPath_.empty()) return false;

    if (pendingPath_.empty()) return false;

    Direction want = pendingPath_.front();
    pendingPath_.erase(pendingPath_.begin());
    Action action = stepToward(want);
    applyAction(action, mouse);

    if (phase_ == BfsPhase::RETURNING_HOME && pendingPath_.empty() &&
        pos_ == Position{0, 0})
    {
        phase_ = BfsPhase::DRIVING_FINAL;
    }

    else if (phase_ == BfsPhase::DRIVING_FINAL && pendingPath_.empty())
    {
        phase_ = BfsPhase::DONE;
    }

    if (GoalDetector::isGoal(pos_, width_, height_))
    {
        everReachedGoal_ = true;
        if (phase_ == BfsPhase::EXPLORING &&
            explorationMode_ == ExplorationMode::SINGLE_PASS_TO_GOAL)
        {
            pendingPath_.clear();
            phase_ = BfsPhase::DONE;
        }
        else if (phase_ == BfsPhase::EXPLORING &&
                 explorationMode_ == ExplorationMode::UNTIL_FIRST_GOAL)
        {
            pendingPath_.clear();
            phase_ = BfsPhase::RETURNING_HOME;
        }
    }

    return true;
}

Position BfsSolver::position() const { return pos_; }
Direction BfsSolver::heading() const { return heading_; }
bool BfsSolver::goalReached() const { return everReachedGoal_; }
Action BfsSolver::lastAction() const { return lastAction_; }
bool BfsSolver::finished() const { return phase_ == BfsPhase::DONE && pendingPath_.empty(); }
BfsPhase BfsSolver::phase() const { return phase_; }

int BfsSolver::visitCount(const Position& p) const
{
    if (!inBounds(p)) return 0;
    return maze_[p.x][p.y].visitCount;
}

const std::vector<std::vector<BfsCell>>& BfsSolver::knownMaze() const
{
    return maze_;
}

std::string actionToString(Action action)
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
