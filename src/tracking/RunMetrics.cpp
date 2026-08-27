#include "tracking/RunMetrics.h"

RunMetrics::RunMetrics(WallFollowMode mode)
{
    reset(mode);
}

void RunMetrics::reset(WallFollowMode mode)
{
    mode_ = mode;
    goalReached_ = false;
    moves_ = 0;
    turns_ = 0;
    uniqueCellsVisited_ = 0;
    totalCellEntries_ = 0;
    finalPosition_ = {0, 0};
    start_ = std::chrono::steady_clock::now();
    end_ = start_;
}

void RunMetrics::startTimer()
{
    start_ = std::chrono::steady_clock::now();
    end_ = start_;
}

void RunMetrics::stopTimer()
{
    end_ = std::chrono::steady_clock::now();
}

void RunMetrics::recordMove()
{
    ++moves_;
}

void RunMetrics::recordTurn()
{
    ++turns_;
}

void RunMetrics::recordTurnAround()
{
    turns_ += 2;
}

void RunMetrics::setGoalReached(bool reached)
{
    goalReached_ = reached;
}

void RunMetrics::setFinalPosition(const Position& position)
{
    finalPosition_ = position;
}

void RunMetrics::setVisitCounts(int uniqueCellsVisited, int totalCellEntries)
{
    uniqueCellsVisited_ = uniqueCellsVisited;
    totalCellEntries_ = totalCellEntries;
}

WallFollowMode RunMetrics::mode() const
{
    return mode_;
}

bool RunMetrics::goalReached() const
{
    return goalReached_;
}

int RunMetrics::moves() const
{
    return moves_;
}

int RunMetrics::turns() const
{
    return turns_;
}

int RunMetrics::uniqueCellsVisited() const
{
    return uniqueCellsVisited_;
}

int RunMetrics::totalCellEntries() const
{
    return totalCellEntries_;
}

Position RunMetrics::finalPosition() const
{
    return finalPosition_;
}

long long RunMetrics::elapsedMilliseconds() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count();
}
