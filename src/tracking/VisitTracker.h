#ifndef TRACKING_VISITTRACKER_H
#define TRACKING_VISITTRACKER_H

#include "model/Position.h"

#include <vector>

class VisitTracker
{
public:
    VisitTracker(int mazeWidth, int mazeHeight);

    void reset();
    void recordVisit(const Position& position);

    int uniqueCellsVisited() const;
    int totalCellEntries() const;
    int visitCount(const Position& position) const;

private:
    bool inBounds(const Position& position) const;

    int mazeWidth_;
    int mazeHeight_;
    int uniqueCellsVisited_;
    int totalCellEntries_;
    std::vector<std::vector<int>> visitCounts_;
};

#endif
