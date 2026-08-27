#ifndef TRACKING_RUNMETRICS_H
#define TRACKING_RUNMETRICS_H

#include "model/Position.h"
#include "navigation/WallFollowMode.h"

#include <chrono>
#include <string>

class RunMetrics
{
public:
    explicit RunMetrics(WallFollowMode mode);

    void reset(WallFollowMode mode);
    void startTimer();
    void stopTimer();

    void recordMove();
    void recordTurn();
    void recordTurnAround();

    void setGoalReached(bool reached);
    void setFinalPosition(const Position& position);
    void setVisitCounts(int uniqueCellsVisited, int totalCellEntries);

    WallFollowMode mode() const;
    bool goalReached() const;
    int moves() const;
    int turns() const;
    int uniqueCellsVisited() const;
    int totalCellEntries() const;
    Position finalPosition() const;
    long long elapsedMilliseconds() const;

private:
    WallFollowMode mode_;
    bool goalReached_;
    int moves_;
    int turns_;
    int uniqueCellsVisited_;
    int totalCellEntries_;
    Position finalPosition_;
    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::time_point end_;
};

#endif
