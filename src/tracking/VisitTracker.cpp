#include "tracking/VisitTracker.h"

VisitTracker::VisitTracker(int mazeWidth, int mazeHeight)
    : mazeWidth_(mazeWidth),
      mazeHeight_(mazeHeight),
      uniqueCellsVisited_(0),
      totalCellEntries_(0),
      visitCounts_(mazeHeight, std::vector<int>(mazeWidth, 0))
{
}

void VisitTracker::reset()
{
    uniqueCellsVisited_ = 0;
    totalCellEntries_ = 0;
    visitCounts_.assign(mazeHeight_, std::vector<int>(mazeWidth_, 0));
}

void VisitTracker::recordVisit(const Position& position)
{
    if (!inBounds(position))
    {
        return;
    }

    int& count = visitCounts_[position.y][position.x];
    if (count == 0)
    {
        ++uniqueCellsVisited_;
    }
    ++count;
    ++totalCellEntries_;
}

int VisitTracker::uniqueCellsVisited() const
{
    return uniqueCellsVisited_;
}

int VisitTracker::totalCellEntries() const
{
    return totalCellEntries_;
}

int VisitTracker::visitCount(const Position& position) const
{
    if (!inBounds(position))
    {
        return 0;
    }
    return visitCounts_[position.y][position.x];
}

bool VisitTracker::inBounds(const Position& position) const
{
    return position.x >= 0 && position.x < mazeWidth_
        && position.y >= 0 && position.y < mazeHeight_;
}
