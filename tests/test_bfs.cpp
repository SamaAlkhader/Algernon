#include "maze/GoalDetector.h"
#include "navigation/BfsSolver.h"

#include <cassert>
#include <iostream>
#include <stdexcept>

namespace
{
constexpr int DX[4] = {0, 1, 0, -1};
constexpr int DY[4] = {1, 0, -1, 0};

class OpenMazeMouse : public IMouseIO
{
public:
    OpenMazeMouse(int width, int height) : width_(width), height_(height) {}

    int mazeWidth() override { return width_; }
    int mazeHeight() override { return height_; }
    bool wallFront() override { return wall(heading_); }
    bool wallLeft() override { return wall(turnLeftFrom(heading_)); }
    bool wallRight() override { return wall(turnRightFrom(heading_)); }

    void moveForward() override
    {
        if (wall(heading_)) throw std::runtime_error("move through wall");
        position_.x += DX[static_cast<int>(heading_)];
        position_.y += DY[static_cast<int>(heading_)];
    }
    void turnLeft() override { heading_ = turnLeftFrom(heading_); }
    void turnRight() override { heading_ = turnRightFrom(heading_); }

    Position position() const { return position_; }

private:
    bool wall(Direction direction) const
    {
        const int x = position_.x + DX[static_cast<int>(direction)];
        const int y = position_.y + DY[static_cast<int>(direction)];
        return x < 0 || x >= width_ || y < 0 || y >= height_;
    }

    int width_;
    int height_;
    Position position_{0, 0};
    Direction heading_ = Direction::NORTH;
};

int runAndVerify(ExplorationMode mode, bool expectFullMap, bool expectReturnHome)
{
    OpenMazeMouse mouse(4, 4);
    BfsSolver solver(4, 4, mode);
    solver.begin(mouse);

    bool reachedCenter = false;
    bool returnedHome = false;
    int steps = 0;
    while (!solver.finished() && steps < 500)
    {
        assert(solver.step(mouse));
        ++steps;
        if (GoalDetector::isGoal(solver.position(), 4, 4))
            reachedCenter = true;
        if (solver.phase() == BfsPhase::DRIVING_FINAL &&
            solver.position() == Position{0, 0})
            returnedHome = true;
    }

    assert(steps < 500);
    assert(reachedCenter);
    assert(returnedHome == expectReturnHome);
    assert(solver.finished());
    assert(GoalDetector::isGoal(solver.position(), 4, 4));
    assert(mouse.position() == solver.position());

    int exploredCells = 0;
    for (const auto& column : solver.knownMaze())
        for (const BfsCell& cell : column)
            if (cell.explored) ++exploredCells;
    if (expectFullMap) assert(exploredCells == 16);
    else assert(exploredCells < 16);

    return steps;
}

void testAllExplorationModesStopAtCenter()
{
    const int fullSteps = runAndVerify(ExplorationMode::FULL_MAZE, true, true);
    const int fastSteps = runAndVerify(ExplorationMode::UNTIL_FIRST_GOAL, false, true);
    const int onePassSteps = runAndVerify(
        ExplorationMode::SINGLE_PASS_TO_GOAL, false, false);
    assert(fastSteps < fullSteps);
    assert(onePassSteps < fastSteps);
}
}

int main()
{
    testAllExplorationModesStopAtCenter();
    std::cout << "All BFS tests passed.\n";
}
