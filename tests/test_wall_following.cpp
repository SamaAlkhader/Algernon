#include "maze/GoalDetector.h"
#include "model/Pose.h"
#include "navigation/WallFollower.h"
#include "tracking/VisitTracker.h"

#include <cassert>
#include <iostream>

namespace
{
void testLeftHandDecisionLogic()
{
    WallFollower follower(WallFollowMode::LEFT_HAND);

    assert(follower.chooseAction({false, false, false}) == Action::TURN_LEFT_AND_FORWARD);
    assert(follower.chooseAction({true, false, false}) == Action::FORWARD);
    assert(follower.chooseAction({true, true, false}) == Action::TURN_RIGHT_AND_FORWARD);
    assert(follower.chooseAction({true, true, true}) == Action::TURN_AROUND_AND_FORWARD);
    assert(follower.chooseAction({false, true, false}) == Action::TURN_LEFT_AND_FORWARD);
}

void testRightHandDecisionLogic()
{
    WallFollower follower(WallFollowMode::RIGHT_HAND);

    assert(follower.chooseAction({false, false, false}) == Action::TURN_RIGHT_AND_FORWARD);
    assert(follower.chooseAction({false, false, true}) == Action::FORWARD);
    assert(follower.chooseAction({false, true, true}) == Action::TURN_LEFT_AND_FORWARD);
    assert(follower.chooseAction({true, true, true}) == Action::TURN_AROUND_AND_FORWARD);
}

void testHeadingUpdates()
{
    assert(turnLeftFrom(Direction::NORTH) == Direction::WEST);
    assert(turnLeftFrom(Direction::WEST) == Direction::SOUTH);
    assert(turnLeftFrom(Direction::SOUTH) == Direction::EAST);
    assert(turnLeftFrom(Direction::EAST) == Direction::NORTH);

    assert(turnRightFrom(Direction::NORTH) == Direction::EAST);
    assert(turnRightFrom(Direction::EAST) == Direction::SOUTH);
    assert(turnRightFrom(Direction::SOUTH) == Direction::WEST);
    assert(turnRightFrom(Direction::WEST) == Direction::NORTH);
}

void testPositionUpdates()
{
    assert((movedForward({1, 1}, Direction::NORTH) == Position{1, 2}));
    assert((movedForward({1, 1}, Direction::EAST) == Position{2, 1}));
    assert((movedForward({1, 1}, Direction::SOUTH) == Position{1, 0}));
    assert((movedForward({1, 1}, Direction::WEST) == Position{0, 1}));
}

void testGoalDetection()
{
    assert(GoalDetector::isGoal({3, 3}, 8, 8));
    assert(GoalDetector::isGoal({3, 4}, 8, 8));
    assert(GoalDetector::isGoal({4, 3}, 8, 8));
    assert(GoalDetector::isGoal({4, 4}, 8, 8));
    assert(!GoalDetector::isGoal({0, 0}, 8, 8));
    assert(!GoalDetector::isGoal({2, 3}, 8, 8));

    assert(GoalDetector::isGoal({7, 7}, 16, 16));
    assert(GoalDetector::isGoal({7, 8}, 16, 16));
    assert(GoalDetector::isGoal({8, 7}, 16, 16));
    assert(GoalDetector::isGoal({8, 8}, 16, 16));
    assert(!GoalDetector::isGoal({6, 7}, 16, 16));
    assert(!GoalDetector::isGoal({0, 0}, 16, 16));
}

void testVisitTracker()
{
    VisitTracker visits(8, 8);

    visits.recordVisit({0, 0});
    assert(visits.uniqueCellsVisited() == 1);
    assert(visits.totalCellEntries() == 1);
    assert(visits.visitCount({0, 0}) == 1);

    visits.recordVisit({0, 1});
    assert(visits.uniqueCellsVisited() == 2);
    assert(visits.totalCellEntries() == 2);
    assert(visits.visitCount({0, 1}) == 1);

    visits.recordVisit({0, 0});
    assert(visits.uniqueCellsVisited() == 2);
    assert(visits.totalCellEntries() == 3);
    assert(visits.visitCount({0, 0}) == 2);
}
}

int main()
{
    testLeftHandDecisionLogic();
    testRightHandDecisionLogic();
    testHeadingUpdates();
    testPositionUpdates();
    testGoalDetection();
    testVisitTracker();

    std::cout << "All wall-following tests passed." << std::endl;
    return 0;
}
